#pragma once
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <string>
#include <thread>

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include "config.h"

namespace fs = std::filesystem;

static inline void tryWrite(const fs::path &p, const std::string &v) {
  std::error_code ec;
  if (!fs::exists(p, ec))
    return;
  std::ofstream f(p);
  if (!f)
    return;
  f << v;
}

static inline std::string readBack(const fs::path &p) {
  std::ifstream f(p);
  std::string s;
  if (f)
    std::getline(f, s);
  return s;
}

static inline std::string readOneLine(const fs::path &p) {
  std::ifstream f(p);
  if (!f)
    throw std::runtime_error("open failed: " + p.string());
  std::string s;
  std::getline(f, s);
  return s;
}

static inline std::array<double, 3>
applyAxisMap(const AxisCfg &c, const std::array<double, 3> &src) {
  std::array<double, 3> out;
  out[0] = c.invert_x ? -src[c.x] : src[c.x];
  out[1] = c.invert_y ? -src[c.y] : src[c.y];
  out[2] = c.invert_z ? -src[c.z] : src[c.z];
  return out;
}

class Ism330IioReader {
public:
  struct IMUConfig {
    std::function<void(double, std::array<double, 3>, std::array<double, 3>,
                       std::chrono::steady_clock::time_point)>
        on_sample;
  };

  // Compile-time constants from your sysfs
  static constexpr double kAccelScale = 0.000598205;
  static constexpr double kGyroScale = 0.000152716;

  static constexpr const char *kTriggerName = "imu833";

  // Fixed packed layout: s16 x3 + s64 ts => 2+2+2+8 = 14 bytes
  static constexpr std::size_t kOffX = 0;
  static constexpr std::size_t kOffY = 2;
  static constexpr std::size_t kOffZ = 4;
  static constexpr std::size_t kOffTS = 6;
  static constexpr std::size_t kStride = 14;

  explicit Ism330IioReader(IMUConfig cfg) : cfg_(std::move(cfg)) {
    if (!cfg_.on_sample)
      throw std::runtime_error(
          "Ism330IioReader: on_sample callback is required");
    discoverSplitDevices();
    setSamplingHz();
    setupBuffer(accel_sysfs_, true);
    setupBuffer(gyro_sysfs_, false);
    openDeviceFds();
    assertFixedLayout(accel_sysfs_, true);
    assertFixedLayout(gyro_sysfs_, false);
    alive_.store(true);
    worker_ = std::thread(&Ism330IioReader::loop, this);
  }

  ~Ism330IioReader() { stop(); }

  void stop() {
    bool exp = true;
    if (!alive_.compare_exchange_strong(exp, false))
      return;
    if (worker_.joinable())
      worker_.join();
    teardownBuffer(accel_sysfs_);
    teardownBuffer(gyro_sysfs_);
    if (fd_accel_ >= 0)
      ::close(fd_accel_);
    if (fd_gyro_ >= 0)
      ::close(fd_gyro_);
  }

  std::string devnode() const {
    return "accel@" + accel_sysfs_.filename().string() + " + gyro@" +
           gyro_sysfs_.filename().string();
  }

private:
  IMUConfig cfg_{};
  std::atomic<bool> alive_{false};
  std::thread worker_{};
  fs::path accel_sysfs_{};
  fs::path gyro_sysfs_{};
  int fd_accel_ = -1;
  int fd_gyro_ = -1;

  static inline std::string devCharFromSysfs(const fs::path &sys) {
    return "/dev/" + sys.filename().string();
  }

  static inline uint16_t rd16le(const uint8_t *p) {
    return (uint16_t)p[0] | (uint16_t(p[1]) << 8);
  }

  static inline uint64_t rd64le(const uint8_t *p) {
    return (uint64_t)p[0] | (uint64_t(p[1]) << 8) | (uint64_t(p[2]) << 16) |
           (uint64_t(p[3]) << 24) | (uint64_t(p[4]) << 32) |
           (uint64_t(p[5]) << 40) | (uint64_t(p[6]) << 48) |
           (uint64_t(p[7]) << 56);
  }

  void discoverSplitDevices() {
    const fs::path base{"/sys/bus/iio/devices"};
    if (!fs::exists(base))
      throw std::runtime_error("No IIO sysfs at " + base.string());
    for (auto &ent : fs::directory_iterator(base)) {
      if (!ent.is_directory())
        continue;
      if (ent.path().filename().string().rfind("iio:device", 0) != 0)
        continue;
      const fs::path sys = ent.path();
      std::string name;
      try {
        name = readOneLine(sys / "name");
      } catch (...) {
        continue;
      }
      if (name.find("ism330") == std::string::npos &&
          name.find("lsm6d") == std::string::npos)
        continue;
      if (name.find("accel") != std::string::npos)
        accel_sysfs_ = sys;
      else if (name.find("gyro") != std::string::npos)
        gyro_sysfs_ = sys;
    }
    if (accel_sysfs_.empty() || gyro_sysfs_.empty())
      throw std::runtime_error("ISM330 split accel/gyro devices not found");
  }

  void setSamplingHz() {
    const std::string hz = std::to_string(Config::sampling_hz);
    tryWrite(accel_sysfs_ / "sampling_frequency", hz);
    tryWrite(gyro_sysfs_ / "sampling_frequency", hz);
    const std::string a = readBack(accel_sysfs_ / "sampling_frequency");
    const std::string g = readBack(gyro_sysfs_ / "sampling_frequency");
    std::printf("ISM330 requested ODR=%s; accel now=%s, gyro now=%s\n",
                hz.c_str(), a.c_str(), g.c_str());

    const fs::path tdir = findTriggerDirByName(kTriggerName);
    const fs::path tf = tdir / "sampling_frequency";
    if (!fs::exists(tf))
      throw std::runtime_error(
          "Trigger '" + std::string(kTriggerName) +
          "' has no sampling_frequency. Is it an hrtimer trigger?");
    tryWrite(tf, hz);
    const std::string tb = readBack(tf);
    if (tb != hz)
      throw std::runtime_error("Failed to set trigger sampling_frequency to " +
                               hz + " (now='" + tb + "')");
    std::printf("Trigger '%s' sampling_frequency now=%s\n", kTriggerName,
                tb.c_str());
  }

  static inline void writeStrict(const fs::path &p, const std::string &v) {
    std::ofstream f(p);
    if (!f)
      throw std::runtime_error("write failed: " + p.string());
    f << v;
    if (!f)
      throw std::runtime_error("write failed (flush): " + p.string());
  }

  static inline void disableAllChannels(const fs::path &dev) {
    const fs::path scan = dev / "scan_elements";
    if (!fs::exists(scan))
      throw std::runtime_error("scan_elements missing under " + dev.string());
    for (auto &ent : fs::directory_iterator(scan)) {
      const std::string fn = ent.path().filename().string();
      if (fn.size() > 3 && fn.rfind("_en") == fn.size() - 3)
        tryWrite(ent.path(), "0");
    }
  }

  static fs::path findTriggerDirByName(const std::string &name) {
    const fs::path base{"/sys/bus/iio/devices"};
    for (auto &ent : fs::directory_iterator(base)) {
      if (ent.is_directory() &&
          ent.path().filename().string().rfind("trigger", 0) == 0) {
        try {
          if (readOneLine(ent.path() / "name") == name)
            return ent.path();
        } catch (...) {
        }
      }
    }
    fs::create_directories("/sys/kernel/config/iio/triggers/hrtimer/" + name);
    for (auto &ent : fs::directory_iterator(base)) {
      if (ent.is_directory() &&
          ent.path().filename().string().rfind("trigger", 0) == 0) {
        try {
          if (readOneLine(ent.path() / "name") == name)
            return ent.path();
        } catch (...) {
        }
      }
    }
    throw std::runtime_error("Failed to create IIO trigger '" + name + "'");
  }

  void setupBuffer(const fs::path &dev, bool is_accel) {
    tryWrite(dev / "buffer" / "enable", "0");
    const fs::path scan = dev / "scan_elements";
    tryWrite(scan / "in_accel_x_en", is_accel ? "1" : "0");
    tryWrite(scan / "in_accel_y_en", is_accel ? "1" : "0");
    tryWrite(scan / "in_accel_z_en", is_accel ? "1" : "0");
    tryWrite(scan / "in_anglvel_x_en", is_accel ? "0" : "1");
    tryWrite(scan / "in_anglvel_y_en", is_accel ? "0" : "1");
    tryWrite(scan / "in_anglvel_z_en", is_accel ? "0" : "1");
    tryWrite(scan / "in_timestamp_en", "1");
    tryWrite(dev / "trigger" / "current_trigger", kTriggerName);
    const std::string rb = readBack(dev / "trigger" / "current_trigger");
    if (rb != kTriggerName)
      throw std::runtime_error("Failed to attach trigger '" +
                               std::string(kTriggerName) + "' to " +
                               dev.string() + " (current='" + rb + "')");
    std::printf("%s: using trigger '%s'\n", dev.filename().c_str(),
                kTriggerName);
    tryWrite(dev / "buffer" / "length", "4096");
    tryWrite(dev / "buffer" / "enable", "1");
  }

  void teardownBuffer(const fs::path &dev) {
    tryWrite(dev / "buffer" / "enable", "0");
    tryWrite(dev / "trigger" / "current_trigger", "");
  }

  void openDeviceFds() {
    const std::string da = devCharFromSysfs(accel_sysfs_);
    const std::string dg = devCharFromSysfs(gyro_sysfs_);
    fd_accel_ = ::open(da.c_str(), O_RDONLY);
    if (fd_accel_ < 0)
      throw std::runtime_error("open failed: " + da);
    fd_gyro_ = ::open(dg.c_str(), O_RDONLY);
    if (fd_gyro_ < 0)
      throw std::runtime_error("open failed: " + dg);
  }

  void assertFixedLayout(const fs::path &dev, bool is_accel) {
    const fs::path s = dev / "scan_elements";
    const std::string ax = is_accel ? "in_accel_x" : "in_anglvel_x";
    const std::string ay = is_accel ? "in_accel_y" : "in_anglvel_y";
    const std::string az = is_accel ? "in_accel_z" : "in_anglvel_z";
    if (readOneLine(s / (ax + "_type")) != "le:s16/16>>0")
      throw std::runtime_error(dev.string() + ": unexpected x type");
    if (readOneLine(s / (ay + "_type")) != "le:s16/16>>0")
      throw std::runtime_error(dev.string() + ": unexpected y type");
    if (readOneLine(s / (az + "_type")) != "le:s16/16>>0")
      throw std::runtime_error(dev.string() + ": unexpected z type");
    if (readOneLine(s / "in_timestamp_type") != "le:s64/64>>0")
      throw std::runtime_error(dev.string() + ": unexpected ts type");
    if (readOneLine(s / (ax + "_index")) != "0")
      throw std::runtime_error(dev.string() + ": x index != 0");
    if (readOneLine(s / (ay + "_index")) != "1")
      throw std::runtime_error(dev.string() + ": y index != 1");
    if (readOneLine(s / (az + "_index")) != "2")
      throw std::runtime_error(dev.string() + ": z index != 2");
    if (readOneLine(s / "in_timestamp_index") != "3")
      throw std::runtime_error(dev.string() + ": ts index != 3");
  }

  void loop() {
    alignas(8) uint8_t bufA[kStride * 512];
    alignas(8) uint8_t bufG[kStride * 512];
    struct Acc {
      double ax;
      double ay;
      double az;
      uint64_t ts;
      bool updated;
    };
    struct Gyr {
      double gx;
      double gy;
      double gz;
      uint64_t ts;
      bool updated;
    };
    Acc lastA{0.0, 0.0, 0.0, 0ULL, false};
    Gyr lastG{0.0, 0.0, 0.0, 0ULL, false};
    uint64_t tsA_emitted = 0ULL;
    uint64_t tsG_emitted = 0ULL;

    std::printf("Polling for IMU data...\n");
    while (alive_.load(std::memory_order_relaxed)) {
      struct pollfd pfds[2];
      pfds[0].fd = fd_accel_;
      pfds[0].events = POLLIN;
      pfds[0].revents = 0;
      pfds[1].fd = fd_gyro_;
      pfds[1].events = POLLIN;
      pfds[1].revents = 0;

      int r = ::poll(pfds, 2, 1000);
      if (r <= 0)
        continue;

      const auto now = std::chrono::steady_clock::now();

      if (pfds[0].revents & POLLIN) {
        ssize_t n = ::read(fd_accel_, bufA, sizeof(bufA));
        if (n > 0) {
          for (size_t off = 0; off + kStride <= (size_t)n; off += kStride) {
            const uint8_t *p = bufA + off;
            int16_t xr = (int16_t)rd16le(p + kOffX);
            int16_t yr = (int16_t)rd16le(p + kOffY);
            int16_t zr = (int16_t)rd16le(p + kOffZ);
            uint64_t ts = rd64le(p + kOffTS);
            lastA.ax = double(xr) * kAccelScale;
            lastA.ay = double(yr) * kAccelScale;
            lastA.az = double(zr) * kAccelScale;
            lastA.ts = ts;
            lastA.updated = true;
          }
        }
      }

      if (pfds[1].revents & POLLIN) {
        ssize_t n = ::read(fd_gyro_, bufG, sizeof(bufG));
        if (n > 0) {
          for (size_t off = 0; off + kStride <= (size_t)n; off += kStride) {
            const uint8_t *p = bufG + off;
            int16_t xr = (int16_t)rd16le(p + kOffX);
            int16_t yr = (int16_t)rd16le(p + kOffY);
            int16_t zr = (int16_t)rd16le(p + kOffZ);
            uint64_t ts = rd64le(p + kOffTS);
            lastG.gx = double(xr) * kGyroScale;
            lastG.gy = double(yr) * kGyroScale;
            lastG.gz = double(zr) * kGyroScale;
            lastG.ts = ts;
            lastG.updated = true;
          }
        }
      }

      if (lastA.updated && lastG.updated) {
        if (lastA.ts != tsA_emitted || lastG.ts != tsG_emitted) {
          const std::array<double, 3> acc_src{lastA.ax, lastA.ay, lastA.az};
          const std::array<double, 3> gyr_src{lastG.gx, lastG.gy, lastG.gz};
          const std::array<double, 3> acc =
              applyAxisMap(Config::accel_cfg, acc_src);
          const std::array<double, 3> gyr =
              applyAxisMap(Config::gyro_cfg, gyr_src);
          const double pitch = std::atan2(-acc[0], acc[2]);
          cfg_.on_sample(pitch, acc, gyr, now);
          tsA_emitted = lastA.ts;
          tsG_emitted = lastG.ts;
          lastA.updated = false;
          lastG.updated = false;
        }
      }
    }
  }
};
