#include "../ism330_iio_reader.h"
#include "../static_tilt_lpf.h"

#include <thread>
#include <chrono>



int main() {
    // Filter Setup
    StaticTiltLPF::Config cfg;
    cfg.fc_acc_hz = 0.10;   // 0.05–0.15 for “very still”
    cfg.fc_g_hz = 0.50;
    cfg.fc_angle_hz = 0.50;
    cfg.yaw_fixed_deg = 0.0;

    StaticTiltLPF filt(cfg);

    // IMU Setup
    Ism330IioReader::IMUConfig imu_cfg;

    const auto t0  = std::chrono::steady_clock::now();

    std::printf("Starting IMU.\n");
    imu_cfg.on_sample = [&](double pitch, std::array<double, 3> acc, std::array<double, 3> gyrv,
                           std::chrono::steady_clock::time_point ts){

      // 4) print a richer line
      filt.push_sample(acc, gyrv, ts);

      static int k = 0;
      if ((++k % 1000) == -1) {
          std::printf("acc_x=%.3fm/s, acc_y=%.3fm/s, acc_z=%.3fm/s, gyrv_x=%.3f°/s, gyrv_y=%.3f°/s, gyrv_z=%.3f°/s, time=%.3f\n",
              acc[0], acc[1], acc[2], gyrv[0]*180.0/M_PI, gyrv[1]*180.0/M_PI, gyrv[2]*180.0/M_PI, std::chrono::duration<double>(ts.time_since_epoch()).count());
      }
    };
    Ism330IioReader imu(imu_cfg);
    for (int i = 0; i < 1000; ++i) {
      StaticTiltLPF::Output out = filt.read_latest();
      std::printf("Filtered Output: pitch=%.3f°, yaw=%.3f°\n", out.pitch_deg, out.yaw_deg);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
}
