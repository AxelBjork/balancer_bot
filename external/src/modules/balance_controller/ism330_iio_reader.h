#pragma once
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <sys/stat.h>
#include <sys/types.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <cmath>

// Minimal helper: read entire file as string
inline std::string read_text_file(const std::string& path) {
  int fd = ::open(path.c_str(), O_RDONLY);
  if (fd < 0) throw std::runtime_error("open: " + path + " failed");
  std::string out;
  char buf[512];
  while (true) {
    const ssize_t n = ::read(fd, buf, sizeof(buf));
    if (n < 0) { ::close(fd); throw std::runtime_error("read: " + path + " failed"); }
    if (n == 0) break;
    out.append(buf, buf + n);
  }
  ::close(fd);
  // trim trailing newlines/spaces
  while (!out.empty() && (out.back()=='\n' || out.back()=='\r' || out.back()==' ')) out.pop_back();
  return out;
}

inline void write_text_file(const std::string& path, std::string_view s) {
  int fd = ::open(path.c_str(), O_WRONLY);
  if (fd < 0) throw std::runtime_error("open: " + path + " for write failed");
  const ssize_t n = ::write(fd, s.data(), s.size());
  ::close(fd);
  if (n < 0 || static_cast<size_t>(n) != s.size()) {
    throw std::runtime_error("write: " + path + " failed");
  }
}

// Find iio:deviceX whose `name` contains any of the provided substrings
inline std::optional<std::string> find_iio_device_by_name(const std::vector<std::string>& needles) {
  const char* base = "/sys/bus/iio/devices";
  DIR* dir = ::opendir(base);
  if (!dir) return std::nullopt;
  struct dirent* ent;
  while ((ent = ::readdir(dir))) {
    if (std::strncmp(ent->d_name, "iio:device", 10) != 0) continue;
    std::string devdir = std::string(base) + "/" + ent->d_name;
    std::string name = read_text_file(devdir + "/name");
    for (const auto& n : needles) {
      if (name.find(n) != std::string::npos) {
        ::closedir(dir);
        return devdir; // sysfs directory path (not /dev node)
      }
    }
  }
  ::closedir(dir);
  return std::nullopt;
}

// Small RAII that turns the IIO buffered capture on/off with selected scan elements.
class Ism330IioReader {
public:
  struct Config {
    // any of these appearing in /sys/bus/iio/devices/iio:deviceX/name will match
    std::vector<std::string> name_hints = {"ism330", "lsm6d", "lsm6dsx"};
    // desired sample rate for both accel+gyro (driver rounds to supported ODR)
    double sampling_hz = 1000.0;
    // ring buffer size (number of samples)
    int buffer_len = 256;
    // axis mapping: which accel/gyro axes correspond to pitch
    // (common: pitch around Y; adjust if your board is rotated)
    // We’ll compute pitch angle from accel_x/z and pitch rate from gyro_y by default.
    int accel_x = 0, accel_y = 1, accel_z = 2; // index 0:x,1:y,2:z
    int gyro_x  = 0, gyro_y  = 1, gyro_z  = 2;
    // callback invoked per sample
    std::function<void(double pitch_rad, double pitch_rate_rad_s,
                       double yaw_rate_rad_s,
                       std::chrono::steady_clock::time_point ts)> on_sample;
  };

  explicit Ism330IioReader(Config cfg) : cfg_(std::move(cfg)) {
    // 1) locate device
    devsys_ = find_iio_device_by_name(cfg_.name_hints).value_or("");
    if (devsys_.empty()) throw std::runtime_error("ISM330 IIO device not found");
    devnode_ = "/dev/" + devsys_.substr(devsys_.rfind('/') + 1);

    // 2) enable accel/gyro scan elements (x,y,z)
    // in_accel_*_en, in_anglvel_*_en
    for (const char axis : {'x','y','z'}) {
      write_text_file(path("scan_elements/in_accel_") + axis + std::string("_en"), "1");
      write_text_file(path("scan_elements/in_anglvel_") + axis + std::string("_en"), "1");
    }

    // 3) set sampling frequency (if present)
    // some drivers expose single `sampling_frequency`
    try {
      write_text_file(path("sampling_frequency"), fmt(cfg_.sampling_hz));
    } catch (...) {
      // fallback: per-sensor nodes if available (not all kernels expose both)
      try { write_text_file(path("in_accel_sampling_frequency"), fmt(cfg_.sampling_hz)); } catch (...) {}
      try { write_text_file(path("in_anglvel_sampling_frequency"), fmt(cfg_.sampling_hz)); } catch (...) {}
    }

    // 4) set buffer length and enable
    write_text_file(path("buffer/length"), std::to_string(cfg_.buffer_len));
    // Open the character device before enabling buffer
    fd_ = ::open(devnode_.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd_ < 0) throw std::runtime_error("open " + devnode_ + " failed");

    write_text_file(path("buffer/enable"), "1");

    // 5) read scales (to convert raw -> SI)
    accel_scale_ = read_double(path("in_accel_scale"));
    gyro_scale_  = read_double(path("in_anglvel_scale"));

    // 6) inspect types (we assume le:s16 here; robust parsing can read *_type)
    //   in_accel_x_type -> e.g. "le:s16/16>>0"
    // If this differs on your kernel, parse and adjust. For ST parts this is typically s16 LE.
    // 7) start thread
    alive_.store(true);
    th_ = std::thread(&Ism330IioReader::run, this);
  }

  ~Ism330IioReader() {
    stop();
  }

  void stop() {
    bool exp = true;
    if (!alive_.compare_exchange_strong(exp, false)) return;
    if (th_.joinable()) th_.join();

    // disable buffer & close
    try { write_text_file(path("buffer/enable"), "0"); } catch (...) {}
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    // disable scan elements (optional)
    for (const char axis : {'x','y','z'}) {
      safe_write(path("scan_elements/in_accel_") + axis + std::string("_en"), "0");
      safe_write(path("scan_elements/in_anglvel_") + axis + std::string("_en"), "0");
    }
  }

  std::string devnode() const { return devnode_; }
  std::string devsys()  const { return devsys_;  }
  double accel_scale()  const { return accel_scale_; }
  double gyro_scale()   const { return gyro_scale_;  }

private:
  std::string path(const std::string& p) const { return devsys_ + "/" + p; }
  static std::string fmt(double v) {
    char buf[64]; std::snprintf(buf, sizeof(buf), "%.0f", v); return buf;
  }
  static double read_double(const std::string& p) {
    return std::stod(read_text_file(p));
  }
  static void safe_write(const std::string& p, std::string_view v) {
    try { write_text_file(p, v); } catch (...) {}
  }

  void run() {
    // Scan layout: accel_x,y,z then gyro_x,y,z in the order enabled by scan_elements.
    // Commonly ST driver orders acc x y z, then gyro x y z, each s16 little-endian.
    // Each sample = 6 * 2 bytes = 12 bytes. If timestamp is enabled, it adds 8 bytes; we’re not enabling it now.
    const size_t stride = 6 * sizeof(int16_t);
    std::vector<uint8_t> buf( stride * static_cast<size_t>(cfg_.buffer_len) );

    while (alive_.load(std::memory_order_relaxed)) {
      const ssize_t n = ::read(fd_, buf.data(), buf.size());
      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          continue;
        }
        // read error; try to continue
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        continue;
      }
      const auto now = std::chrono::steady_clock::now();
      const size_t frames = static_cast<size_t>(n) / stride;
      const uint8_t* p = buf.data();

      for (size_t i = 0; i < frames; ++i, p += stride) {
        const int16_t ax_raw = le16(p + 0);
        const int16_t ay_raw = le16(p + 2);
        const int16_t az_raw = le16(p + 4);
        const int16_t gx_raw = le16(p + 6);
        const int16_t gy_raw = le16(p + 8);
        const int16_t gz_raw = le16(p + 10);

        // apply scales
        const double ax = static_cast<double>(ax_raw) * accel_scale_;
        const double ay = static_cast<double>(ay_raw) * accel_scale_;
        const double az = static_cast<double>(az_raw) * accel_scale_;
        const double gx = static_cast<double>(gx_raw) * gyro_scale_;
        const double gy = static_cast<double>(gy_raw) * gyro_scale_;
        const double gz = static_cast<double>(gz_raw) * gyro_scale_;

        // Axis mapping selection (indices default to x=0,y=1,z=2)
        const double accXYZ[3] = {ax, ay, az};
        const double gyrXYZ[3] = {gx, gy, gz};
        const double acc_x = accXYZ[cfg_.accel_x];
        const double acc_y = accXYZ[cfg_.accel_y];
        const double acc_z = accXYZ[cfg_.accel_z];
        const double gyr_y = gyrXYZ[cfg_.gyro_y];
        const double gyr_z = gyrXYZ[cfg_.gyro_z];

        // Quick pitch estimate (upright gravity ref): pitch = atan2(-ax, az)
        // Adjust sign convention to your controller if needed.
        const double pitch_rad = std::atan2(-acc_x, acc_z);
        const double pitch_rate = gyr_y; // rad/s
        const double yaw_rate   = gyr_z; // rad/s

        if (cfg_.on_sample) cfg_.on_sample(pitch_rad, pitch_rate, yaw_rate, now);
      }
    }
  }

  static int16_t le16(const uint8_t* p) {
    return static_cast<int16_t>(p[0] | (static_cast<int16_t>(p[1]) << 8));
  }

  Config cfg_;
  std::string devsys_;
  std::string devnode_;
  int fd_ = -1;
  double accel_scale_ = 0.0;
  double gyro_scale_  = 0.0;

  std::atomic<bool> alive_{false};
  std::thread th_;
};
