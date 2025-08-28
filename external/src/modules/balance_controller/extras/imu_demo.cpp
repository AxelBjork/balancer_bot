#include "../ism330_iio_reader.h"
#include <thread>
#include <chrono>



int main() {
    Ism330IioReader::IMUConfig cfg;

    const auto t0  = std::chrono::steady_clock::now();

    std::printf("Starting IMU.\n");
    cfg.on_sample = [&](double pitch, double dpitch, double dyaw, std::array<double, 3> acc, auto ts){

      // 4) print a richer line
      static int k = 0;
      if ((++k % 10) == 0) {
        std::printf("pitch=%.3f°/s, dpitch=%.3f°/s, dyaw=%.3f°, x=%.3f y=%.3f z=%.3f\n",
            pitch*180.0/M_PI, dpitch*180.0/M_PI, dyaw*180.0/M_PI, acc[0], acc[1], acc[2]);
      }
    };
    Ism330IioReader imu(cfg);
    std::this_thread::sleep_for(std::chrono::seconds(20));
    return 0;
}
