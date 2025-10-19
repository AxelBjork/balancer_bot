#include "../ism330_iio_reader.h"
#include "../pitch_lpf.h"
#include <cmath>
#include <chrono>
#include <thread>

int main() {
  // Filter Setup
  PitchComplementaryFilter filt{};

  // IMU Setup
  Ism330IioReader::IMUConfig imu_cfg;

  const auto t0 = std::chrono::steady_clock::now();

  std::printf("Starting IMU.\n");
  imu_cfg.on_sample = [&](double pitch, std::array<double, 3> acc,
                          std::array<double, 3> gyrv,
                          std::chrono::steady_clock::time_point ts) {
    filt.push_sample(acc, gyrv, ts);
  };
  Ism330IioReader imu(imu_cfg);
  for (int i = 0; i < 10000; ++i) {
    ImuSample s = filt.read_latest();
    const auto now = std::chrono::steady_clock::now();
    const auto age_ms = std::chrono::duration<double, std::milli>(now - s.t).count();

    std::printf("pitch=%.3f°, dpitch=%.3f°, yaw=%.3f°, age=%.3f ms\n",
                s.angle_rad * 180.0 / M_PI, s.gyro_rad_s * 180.0 / M_PI,
                s.yaw_rate_z * 180.0 / M_PI,
                age_ms);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return 0;
}
