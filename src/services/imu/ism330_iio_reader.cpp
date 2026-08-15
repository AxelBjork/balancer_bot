// ism330_iio_reader.cpp
#include "ism330_iio_reader.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <poll.h>
#include <fcntl.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>

#include "services/main/config.h" // for Config::sampling_hz, accel_cfg, gyro_cfg
#include "services/imu/imu_sample_synchronizer.h"

namespace fs = std::filesystem;

namespace {
constexpr double kExpectedAccelScale = 0.000598205;
constexpr double kExpectedGyroScale  = 0.000152716;
constexpr const char* kTriggerName = "imu833";

const char* get_iio_root() {
  const char* env = std::getenv("IIO_ROOT");
  return env ? env : "/sys";
}

// Fixed packed layout: s16 x3 + s64 ts => 2+2+2+8 = 14 bytes (stride aligned to 16)
constexpr std::size_t kOffX = 0;
constexpr std::size_t kOffY = 2;
constexpr std::size_t kOffZ = 4;
constexpr std::size_t kOffTS = 8;
constexpr std::size_t kStride = 16;

inline void tryWrite(const fs::path& p, const std::string& v) {
  std::error_code ec;
  if (!fs::exists(p, ec)) return;
  std::ofstream f(p);
  if (!f) return;
  f << v;
}

inline std::string readBack(const fs::path& p) {
  std::ifstream f(p);
  std::string s;
  if (f) std::getline(f, s);
  return s;
}

inline std::string readOneLineStrict(const fs::path& p) {
  std::ifstream f(p);
  if (!f) throw std::runtime_error("open failed: " + p.string());
  std::string s;
  std::getline(f, s);
  return s;
}

inline void writeOneLineStrict(const fs::path& p, const std::string& value) {
  std::ofstream f(p);
  if (!f) throw std::runtime_error("open failed: " + p.string());
  f << value;
  f.close();
  if (!f) throw std::runtime_error("write failed: " + p.string());
}

double readDoubleStrict(const fs::path& p) {
  const std::string text = readOneLineStrict(p);
  std::size_t consumed = 0;
  double value = 0.0;
  try {
    value = std::stod(text, &consumed);
  } catch (const std::exception&) {
    throw std::runtime_error("invalid numeric value in " + p.string() + ": '" + text + "'");
  }
  if (consumed != text.size() || !std::isfinite(value)) {
    throw std::runtime_error("invalid numeric value in " + p.string() + ": '" + text + "'");
  }
  return value;
}

std::string fixedDecimal(double value, int precision) {
  std::ostringstream text;
  text << std::fixed << std::setprecision(precision) << value;
  return text.str();
}

double writeAndVerifyDouble(const fs::path& p, double requested, int precision,
                            double tolerance) {
  writeOneLineStrict(p, fixedDecimal(requested, precision));
  const double configured = readDoubleStrict(p);
  if (std::abs(configured - requested) > tolerance) {
    throw std::runtime_error(p.string() + ": requested " + fixedDecimal(requested, precision) +
                             ", got " + fixedDecimal(configured, precision));
  }
  return configured;
}

inline std::array<double,3> applyAxisMap(const AxisCfg& c,
                                         const std::array<double,3>& src) {
  std::array<double,3> out;
  out[0] = c.invert_x ? -src[c.x] : src[c.x];
  out[1] = c.invert_y ? -src[c.y] : src[c.y];
  out[2] = c.invert_z ? -src[c.z] : src[c.z];
  return out;
}

inline uint16_t rd16le(const uint8_t* p) {
  return uint16_t(p[0]) | (uint16_t(p[1]) << 8);
}
inline int64_t rd64le_s(const uint8_t* p) {
  int64_t v;
  std::memcpy(&v, p, sizeof(v));
  return v;
}

inline Ism330IioReader::TimePoint iio_monotonic_ns_to_steady(int64_t ts_ns) {
  return Ism330IioReader::TimePoint(std::chrono::nanoseconds{ts_ns});
}

inline std::string devCharFromSysfs(const fs::path& sys) {
  const char* env = std::getenv("IIO_ROOT");
  std::string prefix = env ? (std::string(env) + "/dev/") : "/dev/";
  return prefix + sys.filename().string();
}

fs::path findTriggerDirByName(const std::string& name) {
  const fs::path root{get_iio_root()};
  const fs::path base = root / "bus/iio/devices";
  for (auto& ent : fs::directory_iterator(base)) {
    if (!ent.is_directory()) continue;
    if (ent.path().filename().string().rfind("trigger", 0) != 0) continue;
    try {
      if (readOneLineStrict(ent.path() / "name") == name) return ent.path();
    } catch (...) {}
  }
  fs::create_directories(root / "kernel/config/iio/triggers/hrtimer/" / name);
  for (auto& ent : fs::directory_iterator(base)) {
    if (!ent.is_directory()) continue;
    if (ent.path().filename().string().rfind("trigger", 0) != 0) continue;
    try {
      if (readOneLineStrict(ent.path() / "name") == name) return ent.path();
    } catch (...) {}
  }
  throw std::runtime_error("Failed to create IIO trigger '" + name + "'");
}

} // namespace

struct Ism330IioReader::Impl {
  IMUConfig cfg{};
  std::atomic<bool> alive{false};
  std::thread worker{};
  fs::path accel_sysfs{};
  fs::path gyro_sysfs{};
  int fd_accel{-1};
  int fd_gyro{-1};
  double accel_scale{kExpectedAccelScale};
  double gyro_scale{kExpectedGyroScale};

  void discoverSplitDevices() {
    const fs::path root{get_iio_root()};
    const fs::path base = root / "bus/iio/devices";
    if (!fs::exists(base)) throw std::runtime_error("No IIO sysfs at " + base.string());
    for (auto& ent : fs::directory_iterator(base)) {
      if (!ent.is_directory()) continue;
      if (ent.path().filename().string().rfind("iio:device", 0) != 0) continue;
      const fs::path sys = ent.path();
      std::string name;
      try { name = readOneLineStrict(sys / "name"); } catch (...) { continue; }
      if (name.find("ism330") == std::string::npos &&
          name.find("lsm6d")  == std::string::npos) continue;
      if (name.find("accel") != std::string::npos) accel_sysfs = sys;
      else if (name.find("gyro") != std::string::npos) gyro_sysfs = sys;
    }
    if (accel_sysfs.empty() || gyro_sysfs.empty())
      throw std::runtime_error("ISM330 split accel/gyro devices not found");
  }

  void setSamplingHz() {
    const double accel_hz =
        writeAndVerifyDouble(accel_sysfs / "sampling_frequency", Config::sampling_hz, 6, 0.001);
    const double gyro_hz =
        writeAndVerifyDouble(gyro_sysfs / "sampling_frequency", Config::sampling_hz, 6, 0.001);

    const fs::path tdir = findTriggerDirByName(kTriggerName);
    const fs::path tf = tdir / "sampling_frequency";
    if (!fs::exists(tf))
      throw std::runtime_error("Trigger '" + std::string(kTriggerName) + "' missing sampling_frequency");
    const double trigger_hz = writeAndVerifyDouble(tf, Config::sampling_hz, 6, 0.001);
    std::printf("IMU ODR: accel=%.3f Hz gyro=%.3f Hz trigger=%s@%.3f Hz\n",
                accel_hz, gyro_hz, kTriggerName, trigger_hz);
  }

  void configureScales() {
    accel_scale = writeAndVerifyDouble(accel_sysfs / "in_accel_scale",
                                       kExpectedAccelScale, 9,
                                       kExpectedAccelScale * 0.01);
    gyro_scale = writeAndVerifyDouble(gyro_sysfs / "in_anglvel_scale",
                                      kExpectedGyroScale, 9,
                                      kExpectedGyroScale * 0.01);
    std::printf("IMU scale: accel=%.9f m/s^2/LSB gyro=%.9f rad/s/LSB\n",
                accel_scale, gyro_scale);
  }

  void setTimestampClock(const fs::path& dev) {
    // The controller's PhysicsTick timestamps use std::chrono::steady_clock,
    // which is CLOCK_MONOTONIC on the Linux target. IIO lets each device choose
    // a different timestamp clock and commonly retains that choice across
    // process restarts. Treating a CLOCK_REALTIME sample as steady time makes
    // it appear decades in the future, so select and verify the clock before
    // enabling either buffer rather than relying on the device default.
    tryWrite(dev / "buffer" / "enable", "0");
    const fs::path clock = dev / "current_timestamp_clock";
    writeOneLineStrict(clock, "monotonic");
    const std::string configured = readOneLineStrict(clock);
    if (configured != "monotonic") {
      throw std::runtime_error(dev.string() + ": failed to select monotonic timestamp clock (got '" +
                               configured + "')");
    }
    std::printf("%s timestamp clock now=%s\n", dev.filename().c_str(), configured.c_str());
  }

  void setupBuffer(const fs::path& dev, bool is_accel) {
    writeOneLineStrict(dev / "buffer" / "enable", "0");
    const fs::path scan = dev / "scan_elements";
    const std::string prefix = is_accel ? "in_accel_" : "in_anglvel_";
    writeOneLineStrict(scan / (prefix + "x_en"), "1");
    writeOneLineStrict(scan / (prefix + "y_en"), "1");
    writeOneLineStrict(scan / (prefix + "z_en"), "1");
    writeOneLineStrict(scan / "in_timestamp_en", "1");
    writeOneLineStrict(dev / "trigger" / "current_trigger", kTriggerName);
    const std::string rb = readBack(dev / "trigger" / "current_trigger");
    if (rb != kTriggerName)
      throw std::runtime_error("Failed to attach trigger '" + std::string(kTriggerName) + "'");
    writeOneLineStrict(dev / "buffer" / "length", "4096");
    writeOneLineStrict(dev / "buffer" / "enable", "1");
  }

  void teardownBuffer(const fs::path& dev) {
    tryWrite(dev / "buffer" / "enable", "0");
    tryWrite(dev / "trigger" / "current_trigger", "");
  }

  void openDeviceFds() {
    const std::string da = devCharFromSysfs(accel_sysfs);
    const std::string dg = devCharFromSysfs(gyro_sysfs);
    fd_accel = ::open(da.c_str(), O_RDONLY);
    if (fd_accel < 0) throw std::runtime_error("open failed: " + da);
    fd_gyro = ::open(dg.c_str(), O_RDONLY);
    if (fd_gyro  < 0) throw std::runtime_error("open failed: " + dg);
  }

  void assertFixedLayout(const fs::path& dev, bool is_accel) {
    const fs::path s = dev / "scan_elements";
    const std::string ax = is_accel ? "in_accel_x" : "in_anglvel_x";
    const std::string ay = is_accel ? "in_accel_y" : "in_anglvel_y";
    const std::string az = is_accel ? "in_accel_z" : "in_anglvel_z";
    if (readOneLineStrict(s / (ax + "_type")) != "le:s16/16>>0") throw std::runtime_error(dev.string()+": x type");
    if (readOneLineStrict(s / (ay + "_type")) != "le:s16/16>>0") throw std::runtime_error(dev.string()+": y type");
    if (readOneLineStrict(s / (az + "_type")) != "le:s16/16>>0") throw std::runtime_error(dev.string()+": z type");
    if (readOneLineStrict(s / "in_timestamp_type") != "le:s64/64>>0") throw std::runtime_error(dev.string()+": ts type");
    if (readOneLineStrict(s / (ax + "_index")) != "0") throw std::runtime_error(dev.string()+": x index");
    if (readOneLineStrict(s / (ay + "_index")) != "1") throw std::runtime_error(dev.string()+": y index");
    if (readOneLineStrict(s / (az + "_index")) != "2") throw std::runtime_error(dev.string()+": z index");
    if (readOneLineStrict(s / "in_timestamp_index") != "3") throw std::runtime_error(dev.string()+": ts index");
  }

  void loop() {
    alignas(8) uint8_t bufA[kStride * 512];
    alignas(8) uint8_t bufG[kStride * 512];

    ImuSampleSynchronizer synchronizer;

    const auto emit_pair = [this](const ImuSynchronizedPair& pair) {
      if (cfg.on_sample) {
        cfg.on_sample(pair.accel.value, pair.gyro.value,
                      iio_monotonic_ns_to_steady(pair.timestamp_ns()));
      }
    };

    std::printf("Polling for IMU data...\n");
    while (alive.load(std::memory_order_relaxed)) {
      pollfd pfds[2];
      pfds[0] = { fd_accel, POLLIN, 0 };
      pfds[1] = { fd_gyro,  POLLIN, 0 };
      int r = ::poll(pfds, 2, 1000);
      if (r <= 0) continue;

      if (pfds[0].revents & POLLIN) {
        ssize_t n = ::read(fd_accel, bufA, sizeof(bufA));
        for (size_t off = 0; n > 0 && off + kStride <= static_cast<size_t>(n); off += kStride) {
          const uint8_t* p = bufA + off;
          const int64_t timestamp_ns = rd64le_s(p + kOffTS);
          const std::array<double, 3> raw{
              static_cast<double>(static_cast<int16_t>(rd16le(p + kOffX))) * accel_scale,
              static_cast<double>(static_cast<int16_t>(rd16le(p + kOffY))) * accel_scale,
              static_cast<double>(static_cast<int16_t>(rd16le(p + kOffZ))) * accel_scale};
          if (const auto pair = synchronizer.push_accel(
                  ImuTimedVector{applyAxisMap(Config::accel_cfg, raw), timestamp_ns})) {
            emit_pair(*pair);
          }
        }
      }
      if (pfds[1].revents & POLLIN) {
        ssize_t n = ::read(fd_gyro, bufG, sizeof(bufG));
        for (size_t off = 0; n > 0 && off + kStride <= static_cast<size_t>(n); off += kStride) {
          const uint8_t* p = bufG + off;
          const int64_t timestamp_ns = rd64le_s(p + kOffTS);
          const std::array<double, 3> raw{
              static_cast<double>(static_cast<int16_t>(rd16le(p + kOffX))) * gyro_scale,
              static_cast<double>(static_cast<int16_t>(rd16le(p + kOffY))) * gyro_scale,
              static_cast<double>(static_cast<int16_t>(rd16le(p + kOffZ))) * gyro_scale};
          if (const auto pair = synchronizer.push_gyro(
                  ImuTimedVector{applyAxisMap(Config::gyro_cfg, raw), timestamp_ns})) {
            emit_pair(*pair);
          }
        }
      }
    }
  }
};

Ism330IioReader::Ism330IioReader(IMUConfig cfg) : p_(std::make_unique<Impl>()) {
  if (!cfg.on_sample)
    throw std::runtime_error("Ism330IioReader: on_sample callback is required");
  p_->cfg = std::move(cfg);
  p_->discoverSplitDevices();
  p_->setTimestampClock(p_->accel_sysfs);
  p_->setTimestampClock(p_->gyro_sysfs);
  p_->setSamplingHz();
  p_->configureScales();
  try {
    p_->setupBuffer(p_->accel_sysfs, true);
    p_->setupBuffer(p_->gyro_sysfs, false);
    p_->openDeviceFds();
    p_->assertFixedLayout(p_->accel_sysfs, true);
    p_->assertFixedLayout(p_->gyro_sysfs, false);
  } catch (...) {
    p_->teardownBuffer(p_->accel_sysfs);
    p_->teardownBuffer(p_->gyro_sysfs);
    if (p_->fd_accel >= 0) ::close(p_->fd_accel);
    if (p_->fd_gyro >= 0) ::close(p_->fd_gyro);
    throw;
  }
  p_->alive.store(true, std::memory_order_relaxed);
  p_->worker = std::thread(&Impl::loop, p_.get());
}

Ism330IioReader::~Ism330IioReader() { stop(); }

Ism330IioReader::Ism330IioReader(Ism330IioReader&& o) noexcept : p_(std::move(o.p_)) {}
Ism330IioReader& Ism330IioReader::operator=(Ism330IioReader&& o) noexcept { p_ = std::move(o.p_); return *this; }

void Ism330IioReader::stop() {
  if (!p_) return;
  bool exp = true;
  if (!p_->alive.compare_exchange_strong(exp, false)) return;
  if (p_->worker.joinable()) p_->worker.join();
  p_->teardownBuffer(p_->accel_sysfs);
  p_->teardownBuffer(p_->gyro_sysfs);
  if (p_->fd_accel >= 0) ::close(p_->fd_accel);
  if (p_->fd_gyro  >= 0) ::close(p_->fd_gyro);
}

std::string Ism330IioReader::devnode() const {
  if (!p_) return {};
  return "accel@" + p_->accel_sysfs.filename().string() + " + gyro@" +
         p_->gyro_sysfs.filename().string();
}
