#include <pigpiod_if2.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <thread>
#include <iostream>

#include "stepper.h"
#include "xbox_controller.h"
#include "motor_runner.h"
#include "config.h"
#include "control_loop.h"
#include "ism330_iio_reader.h"
#include "static_tilt_lpf.h"

// ---------------------- Motor control runner --------------------------------
class App {
public:
  int run(PigpioCtx& _ctx, bool xbox_control = true) {
    if (xbox_control) {
      pad = std::make_unique<XboxController>();
    }
    // Hardware setup
    Stepper::Pins leftPins  {12, 19, 13}; // ENA, STEP, DIR
    Stepper::Pins rightPins { 4, 18, 24}; // ENB, STEP, DIR
    Stepper left(_ctx.handle(), leftPins), right(_ctx.handle(), rightPins);
    MotorRunner L(left, Config::invert_left);
    MotorRunner R(right, Config::invert_right);

    // Start cascaded controller (runs its own thread)
    CascadedController<MotorRunner> ctrl(L, R);

    // Optional: telemetry print every N balance ticks for quick visibility
    // Set to 0 to disable.
    constexpr int kPrintEvery = 10;
    if constexpr (kPrintEvery != -1) {
      std::atomic<int> k{0};
      ctrl.setTelemetrySink([&](const Telemetry& t){
        if ((++k % kPrintEvery) == 0) {
          constexpr double deg = 180.0 / M_PI;
          std::printf(
            "time=%8.3fs  pitch=%8.2f°  gyro=%8.2f°/s  "
            "target_pitch=%8.2f°%s  "
            "balance_out=%7.0f  steer=%7.0f  "
            "L=%7.0f  R=%7.0f\n",
            std::chrono::duration<double>(t.ts.time_since_epoch()).count(),
            t.tilt_rad * deg,
            t.gyro_rad_s * deg,
            t.tilt_target_rad * deg,
            t.tilt_saturated ? "*" : " ",   // mark with * if saturated
            t.u_balance_sps,
            t.steer_split_sps,
            t.left_cmd_sps,
            t.right_cmd_sps
          );
        }
      });
    }

    // IMU reader (IIO; runs its own thread). If not found, we’ll fall back to zeros.
    StaticTiltLPF filt{};
    std::unique_ptr<Ism330IioReader> imu;
    try {
      Ism330IioReader::IMUConfig icfg;

      icfg.on_sample = [&](double pitch, std::array<double, 3> acc, std::array<double, 3> gyrv,
                           std::chrono::steady_clock::time_point ts){

        filt.push_sample(acc, gyrv, ts);

        ImuSample s = filt.read_latest();
        ctrl.pushImu(s);
        static int k = 0;
        if ((++k % 1) == 1) {
          std::printf("pitch=%.3f°, dpitch=%.3f°, yaw=%.3f°, time=%.3f\n",
             s.angle_rad * 180.0 / M_PI, s.gyro_rad_s * 180.0 / M_PI, s.yaw_rate_z * 180.0 / M_PI, std::chrono::duration<double>(ts.time_since_epoch()).count());
          std::printf("acc_x=%.3fm/s, acc_y=%.3fm/s, acc_z=%.3fm/s, gyrv_x=%.3f°/s, gyrv_y=%.3f°/s, gyrv_z=%.3f°/s, time=%.3f\n",
              acc[0], acc[1], acc[2], gyrv[0]*180.0/M_PI, gyrv[1]*180.0/M_PI, gyrv[2]*180.0/M_PI, std::chrono::duration<double>(ts.time_since_epoch()).count());
        }
      };
      imu = std::make_unique<Ism330IioReader>(std::move(icfg));
      std::cout << "IIO IMU started at " << imu->devnode() << "\n";
    } catch (const std::exception& e) {
      std::cerr << "Warning: IMU not started (" << e.what()
                << "). Controller will run with zeroed IMU.\n";
      // Feed zeros at ~1 kHz so the controller has timestamps (very basic fallback).
      std::thread([&ctrl]{
        using clk = std::chrono::steady_clock;
        auto next = clk::now();
        while (!g_stop.load(std::memory_order_relaxed)) {
          next += std::chrono::microseconds(1000);
          ImuSample s{};
          s.angle_rad = 0.0; s.gyro_rad_s = 0.0; s.yaw_rate_z = 0.0; s.t = clk::now();
          ctrl.pushImu(s);
          std::this_thread::sleep_until(next);
        }
      }).detach();
    }

    // Main app loop: read gamepad and feed controller setpoints
    const auto t_end = std::chrono::steady_clock::now()
                     + std::chrono::seconds(Config::run_seconds);
    const auto tick  = std::chrono::milliseconds(1000 / Config::control_hz);

    while (std::chrono::steady_clock::now() < t_end
           && !g_stop.load(std::memory_order_relaxed)) {
      float ly = 0.0;
      float ry = 0;

      if(xbox_control) {
        pad->update();
        // Sticks: map to forward & turn in [-1, 1].
        ly = pad->leftY();
        ry = pad->rightY();
      }
      if (Config::invert_left)  ly = -ly;
      if (Config::invert_right) ry = -ry;

      JoyCmd j{};
      // Sum/diff mapping: sum -> forward, diff -> yaw command
      j.forward = std::clamp(0.5f * (ly + ry), -1.0f, 1.0f);
      j.turn    = std::clamp(0.5f * (ry - ly), -1.0f, 1.0f);
      ctrl.setJoystick(j);

      std::this_thread::sleep_for(tick);
    }

    // shutdown
    g_stop.store(true, std::memory_order_relaxed);
    // ctrl destructor joins its thread; MotorRunner has stop()
    L.stop();
    R.stop();
    return 0;
  }
  std::unique_ptr<XboxController> pad;
};

// --------------------------- main -------------------------------------------
int main() {
  std::signal(SIGINT,  on_signal);
  std::signal(SIGTERM, on_signal);

  PigpioCtx _ctx;   // your pigpio context wrapper
  App app;
  return app.run(_ctx, false);
}