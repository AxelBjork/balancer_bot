#pragma once
#include <atomic>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include "config.h"

namespace fs = std::filesystem;

struct AxisCfg {
  int x = 0, y = 1, z = 2;     // which source axis to use
  bool invert_x = false;
  bool invert_y = false;
  bool invert_z = false;
};

static void tryWrite(const fs::path& p, const std::string& v) {
  std::error_code ec;
  if (!fs::exists(p, ec)) return;
  std::ofstream f(p);
  if (!f) return;
  f << v;
}

static std::string readBack(const fs::path& p) {
  std::ifstream f(p);
  std::string s; if (f) std::getline(f, s); return s;
}


inline std::array<double, 3> applyAxisMap(const AxisCfg& c, const std::array<double, 3>& src) {
  return std::array<double, 3>{
    c.invert_x ? -src[c.x] : src[c.x],
    c.invert_y ? -src[c.y] : src[c.y],
    c.invert_z ? -src[c.z] : src[c.z]
  };
}

class Ism330IioReader {
public:
  struct IMUConfig {
    AxisCfg accel_cfg{.x = 0, .y = 2, .z = 1, .invert_x = true, .invert_z = true};
    AxisCfg gyro_cfg{.x = 0, .y = 2, .z = 1, .invert_x = true, .invert_z = true};

    // Required: called from reader thread per sample
    std::function<void(double pitch_rad, std::array<double, 3> acc, std::array<double, 3> gyrv,
                       std::chrono::steady_clock::time_point ts)> on_sample;
  };

  explicit Ism330IioReader(IMUConfig cfg) : cfg_(std::move(cfg)) {
    if (!cfg_.on_sample) throw std::runtime_error("Ism330IioReader: on_sample callback is required");
    discoverDevices();            // fills accel_sysfs_/gyro_sysfs_ or combo_sysfs_
    openRawChannels();            // opens in_*_raw ifstreams and reads scales
    alive_.store(true);
    std::printf("Ism330IioReader::__start_thread__.\n");
    worker_ = std::thread(&Ism330IioReader::loop, this);
  }

  ~Ism330IioReader() { stop(); }

  void stop() {
    bool exp = true;
    if (!alive_.compare_exchange_strong(exp, false)) return;
    if (worker_.joinable()) worker_.join();
    closeAll();
  }

  // for logs — if split, returns "accel@iio:deviceY + gyro@iio:deviceX"
  std::string devnode() const {
    if (!combo_sysfs_.empty()) return combo_sysfs_.filename().string();
    return "accel@" + accel_sysfs_.filename().string() + " + gyro@" + gyro_sysfs_.filename().string();
  }

private:
  IMUConfig cfg_{};
  std::atomic<bool> alive_{false};
  std::thread worker_;

  // sysfs roots
  fs::path accel_sysfs_; // /sys/bus/iio/devices/iio:deviceX (accel)
  fs::path gyro_sysfs_;  // /sys/bus/iio/devices/iio:deviceY (gyro)
  fs::path combo_sysfs_; // if combined device exists instead

  // scales
  double accel_scale_ = 0.0;
  double gyro_scale_  = 0.0;

  // open raw streams (kept open; we seek to 0 each read)
  std::ifstream acc_raw_[3]; // x,y,z
  std::ifstream gyr_raw_[3]; // x,y,z

  static std::string readOneLine(const fs::path& p) {
    std::ifstream f(p);
    if (!f) throw std::runtime_error("open failed: " + p.string());
    std::string s; std::getline(f, s);
    return s;
  }
  static double readDouble(const fs::path& p) {
    std::ifstream f(p);
    if (!f) throw std::runtime_error("open failed: " + p.string());
    double v=0.0; f >> v; return v;
  }
  static void openRaw(std::ifstream& s, const fs::path& p) {
    s.open(p);
    if (!s) throw std::runtime_error("open raw failed: " + p.string());
  }
  static int readInt(std::ifstream& s) {
    s.clear(); s.seekg(0, std::ios::beg);
    int v=0; s >> v; return v;
  }

  void discoverDevices() {
    const fs::path base{"/sys/bus/iio/devices"};
    if (!fs::exists(base)) throw std::runtime_error("No IIO sysfs at " + base.string());

    fs::path accel, gyro, combo;
    for (auto& ent : fs::directory_iterator(base)) {
      if (!ent.is_directory()) continue;
      if (ent.path().filename().string().rfind("iio:device", 0) != 0) continue;
      const auto sys = ent.path();
      std::string name;
      try { name = readOneLine(sys / "name"); } catch (...) { continue; }
      if (name.find("ism330dhcx_accel") != std::string::npos) accel = sys;
      else if (name.find("ism330dhcx_gyro") != std::string::npos) gyro = sys;
      else if (name.find("lsm6ds") != std::string::npos || name.find("ism330") != std::string::npos) combo = sys;
    }

    if (!combo.empty()) {
      combo_sysfs_ = combo;
      // combined device must have both accel and gyro raw files
      if (!fs::exists(combo_sysfs_ / "in_accel_x_raw") ||
          !fs::exists(combo_sysfs_ / "in_anglvel_x_raw")) {
        // fall back to split if combo unusable
        combo_sysfs_.clear();
      }
    }

    if (combo_sysfs_.empty()) {
      if (accel.empty() || gyro.empty())
        throw std::runtime_error("ISM330: split accel/gyro devices not found");
      accel_sysfs_ = accel;
      gyro_sysfs_  = gyro;
    }
  }

  void setSamplingAndLog() {
    // Split devices
    if (!combo_sysfs_.empty()) return; // you’re on split
    const std::string hz = std::to_string(Config::sampling_hz);
    tryWrite(accel_sysfs_ / "sampling_frequency", hz);
    tryWrite(gyro_sysfs_  / "sampling_frequency", hz);

    auto a = readBack(accel_sysfs_ / "sampling_frequency");
    auto g = readBack(gyro_sysfs_  / "sampling_frequency");
    std::printf("Requested sampling_freq=%s; now=%s\n",
                hz.c_str(), a.c_str());
  }

  void openRawChannels() {
    if (!combo_sysfs_.empty()) {
      accel_scale_ = readDouble(combo_sysfs_ / "in_accel_scale");
      gyro_scale_  = readDouble(combo_sysfs_ / "in_anglvel_scale");
      openRaw(acc_raw_[0], combo_sysfs_ / "in_accel_x_raw");
      openRaw(acc_raw_[1], combo_sysfs_ / "in_accel_y_raw");
      openRaw(acc_raw_[2], combo_sysfs_ / "in_accel_z_raw");
      openRaw(gyr_raw_[0], combo_sysfs_ / "in_anglvel_x_raw");
      openRaw(gyr_raw_[1], combo_sysfs_ / "in_anglvel_y_raw");
      openRaw(gyr_raw_[2], combo_sysfs_ / "in_anglvel_z_raw");
      return;
    }

    // split devices
    setSamplingAndLog();
    accel_scale_ = readDouble(accel_sysfs_ / "in_accel_scale");
    gyro_scale_  = readDouble(gyro_sysfs_  / "in_anglvel_scale");

    openRaw(acc_raw_[0], accel_sysfs_ / "in_accel_x_raw");
    openRaw(acc_raw_[1], accel_sysfs_ / "in_accel_y_raw");
    openRaw(acc_raw_[2], accel_sysfs_ / "in_accel_z_raw");

    openRaw(gyr_raw_[0],  gyro_sysfs_ / "in_anglvel_x_raw");
    openRaw(gyr_raw_[1],  gyro_sysfs_ / "in_anglvel_y_raw");
    openRaw(gyr_raw_[2],  gyro_sysfs_ / "in_anglvel_z_raw");
  }

  void closeAll() {
    for (auto& s : acc_raw_) if (s.is_open()) s.close();
    for (auto& s : gyr_raw_) if (s.is_open()) s.close();
  }

  void loop() {
    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, Config::sampling_hz));
    auto next = clock::now();

    // simple jitter absorber: if we fall behind, just catch up
    while (alive_.load(std::memory_order_relaxed)) {
      next += std::chrono::duration_cast<clock::duration>(period);
      const auto now = clock::now();

      // read raw ints
      const int axr = readInt(acc_raw_[0]);
      const int ayr = readInt(acc_raw_[1]);
      const int azr = readInt(acc_raw_[2]);

      const int gxr = readInt(gyr_raw_[0]);
      const int gyr = readInt(gyr_raw_[1]);
      const int gzr = readInt(gyr_raw_[2]);

      // scale to SI
      const double ax = static_cast<double>(axr) * accel_scale_;
      const double ay = static_cast<double>(ayr) * accel_scale_;
      const double az = static_cast<double>(azr) * accel_scale_;
      const double gx = static_cast<double>(gxr) * gyro_scale_;
      const double gy = static_cast<double>(gyr) * gyro_scale_;
      const double gz = static_cast<double>(gzr) * gyro_scale_;

      const auto acc_src = std::array{ax, ay, az};
      const auto gyr_src = std::array{gx, gy, gz};

      const auto acc = applyAxisMap(cfg_.accel_cfg, acc_src);
      const auto gyrv = applyAxisMap(cfg_.gyro_cfg, gyr_src);

      double ax_m = acc[0];
      double az_m = acc[2];

      const double pitch = std::atan2(-ax_m, az_m);
      cfg_.on_sample(pitch, acc, gyrv, now);

      std::this_thread::sleep_until(next);
    }
  }
};
