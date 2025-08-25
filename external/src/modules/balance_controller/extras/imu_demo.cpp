#include "../ism330_iio_reader.h"
#include <thread>
#include <chrono>


struct Smoother {
  // simple exponential smoothing
  double alpha = 0.1;    // 0..1; lower = smoother
  bool   init  = false;
  double y     = 0.0;
  double step(double x) {
    if (!init) { y = x; init = true; return y; }
    y = (1.0 - alpha) * y + alpha * x;
    return y;
  }
};

int main() {
    Ism330IioReader::IMUConfig cfg;

    Smoother sm_pitch_rate{0.05}, sm_yaw_rate{0.05};
    bool calibrating = true;
    double gp_bias = 0.0, gz_bias = 0.0;
    int    bias_n  = 0;
    const auto t0  = std::chrono::steady_clock::now();

    std::printf("Starting IMU.\n");
    cfg.on_sample = [&](double pitch, double dpitch, double dyaw, auto ts){
      // 1) bias estimate for first ~1 s (keep sensor still!)
      if (calibrating) {
        const double t = std::chrono::duration<double>(ts - t0).count();
        if (t < 1.0) {
          gp_bias += dpitch;
          gz_bias += dyaw;
          ++bias_n;
        } else if (bias_n > 0) {
          gp_bias /= bias_n;
          gz_bias /= bias_n;
          calibrating = false;
          std::printf("Gyro bias: pitch=%.3f°/s yaw=%.3f°/s\n",
                      gp_bias*180.0/M_PI, gz_bias*180.0/M_PI);
        }
      }

      // 2) apply bias once we have it
      if (!calibrating) {
        dpitch -= gp_bias;
        dyaw   -= gz_bias;
      }

      // 3) light smoothing for display
      const double dp_s = sm_pitch_rate.step(dpitch);
      const double dy_s = sm_yaw_rate.step(dyaw);

      // 4) print a richer line
      static int k = 0;
      if ((++k % 10) == 0) {
        std::printf("pitch=%7.2f°  dpitch=%7.2f°/s  dyaw=%7.2f°/s  %s\n",
          pitch*180.0/M_PI, dp_s*180.0/M_PI, dy_s*180.0/M_PI,
          calibrating ? "[calibrating]" : "            ");
        std::fflush(stdout);
      }
    };
    Ism330IioReader imu(cfg);
    std::this_thread::sleep_for(std::chrono::seconds(20));
    return 0;
}
