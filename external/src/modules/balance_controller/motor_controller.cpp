#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <pigpiod_if2.h>
#include <thread>

#include "config.h"
#include "control_loop.h"
#include "ism330_iio_reader.h"
#include "motor_runner.h"
#include "pitch_lpf.h"
#include "stepper.h"
#include "xbox_controller.h"

// ---------------------- Motor control runner --------------------------------
class App {
public:
  int run(PigpioCtx &_ctx, bool xbox_control = true) {
    if (xbox_control) {
      pad = std::make_unique<XboxController>();
    }
    // Hardware setup
    Stepper::Pins leftPins{12, 19, 13}; // ENA, STEP, DIR
    Stepper::Pins rightPins{4, 18, 24}; // ENB, STEP, DIR
    Stepper left(_ctx.handle(), leftPins), right(_ctx.handle(), rightPins);
    MotorRunner L(left, Config::invert_left);
    MotorRunner R(right, Config::invert_right);

    // Start cascaded controller (runs its own thread)
    CascadedController<MotorRunner> ctrl(L, R);

    // Optional: telemetry print every N balance ticks for quick visibility
    // Set to 0 to disable.
    if constexpr (Config::kPrintEvery != -1) {
      std::atomic<int> k{0};
      ctrl.setTelemetrySink([&](const Telemetry &t) {
        if ((++k % Config::kPrintEvery) == 0) {
          std::printf(
              "t=%7.3f  θ=%6.2f°  θ̇=%6.2f°/s  r_sp=%6.2f°/s  out=%6.3f  "
              "u=%6.0f%s  I=%7.3f\n",
              t.t_sec,
              t.pitch_deg,
              t.pitch_rate_dps,
              t.rate_sp_dps,
              t.out_norm,
              t.u_sps,
              (std::abs(t.u_sps) >= 0.99 * Config::max_sps) ? "*" : "",  // rail hint
              t.integ_pitch
          );

        }
      });
    }

    // IMU reader (IIO; runs its own thread). If not found, we’ll fall back to
    // zeros.
    PitchComplementaryFilter filt{};
    std::unique_ptr<Ism330IioReader> imu;
    try {
      Ism330IioReader::IMUConfig icfg;

      icfg.on_sample = [&](double pitch, std::array<double, 3> acc,
                           std::array<double, 3> gyrv,
                           std::chrono::steady_clock::time_point ts) {
        filt.push_sample(acc, gyrv, ts);

        ImuSample s = filt.read_latest();
        ctrl.pushImu(s);
        static int k = 0;
        if ((++k % 10) == -1) {
          // std::printf("pitch=%.3f°, dpitch=%.3f°, yaw=%.3f°, time=%.3f\n",
          //    s.angle_rad * 180.0 / M_PI, s.gyro_rad_s * 180.0 / M_PI,
          //    s.yaw_rate_z * 180.0 / M_PI,
          //    std::chrono::duration<double>(ts.time_since_epoch()).count());
          // std::printf("acc_x=%.3fm/s, acc_y=%.3fm/s, acc_z=%.3fm/s,
          // gyrv_x=%.3f°/s, gyrv_y=%.3f°/s, gyrv_z=%.3f°/s, time=%.3f\n",
          //     acc[0], acc[1], acc[2], gyrv[0]*180.0/M_PI, gyrv[1]*180.0/M_PI,
          //     gyrv[2]*180.0/M_PI,
          //     std::chrono::duration<double>(ts.time_since_epoch()).count());
          // Naive picth compare
          static bool hdr = false;
          if (!hdr) {
            std::printf("pitch_f_deg,pitch_r_deg,dtheta_deg,dpitch_f_dps,"
                        "dpitch_r_dps,domega_dps\n");
            hdr = true;
          }
          std::printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                      s.angle_rad * 180.0 / M_PI, pitch * 180.0 / M_PI,
                      (s.angle_rad - pitch) * 180.0 / M_PI,
                      s.gyro_rad_s * 180.0 / M_PI, gyrv[1] * 180.0 / M_PI,
                      (s.gyro_rad_s - gyrv[1]) * 180.0 / M_PI);
        }
      };
      imu = std::make_unique<Ism330IioReader>(std::move(icfg));
      std::cout << "IIO IMU started at " << imu->devnode() << "\n";
    } catch (const std::exception &e) {
      std::cerr << "Warning: IMU not started (" << e.what()
                << "). Controller will run with zeroed IMU.\n";
      // Feed zeros at ~1 kHz so the controller has timestamps (very basic
      // fallback).
      std::thread([&ctrl] {
        using clk = std::chrono::steady_clock;
        auto next = clk::now();
        while (!g_stop.load(std::memory_order_relaxed)) {
          next += std::chrono::microseconds(1000);
          ImuSample s{};
          s.angle_rad = 0.0;
          s.gyro_rad_s = 0.0;
          s.yaw_rate_z = 0.0;
          s.t = clk::now();
          ctrl.pushImu(s);
          std::this_thread::sleep_until(next);
        }
      }).detach();
    }

    // Main app loop: read gamepad and feed controller setpoints
    const auto t_end = std::chrono::steady_clock::now() +
                       std::chrono::seconds(Config::run_seconds);
    const auto tick = std::chrono::milliseconds(1000 / Config::control_hz);

    while (std::chrono::steady_clock::now() < t_end &&
           !g_stop.load(std::memory_order_relaxed)) {
      float ly = 0.0;
      float ry = 0;

      if (xbox_control) {
        pad->update();
        // Sticks: map to forward & turn in [-1, 1].
        ly = pad->leftY();
        ry = pad->rightY();
      }
      if (Config::invert_left)
        ly = -ly;
      if (Config::invert_right)
        ry = -ry;

      JoyCmd j{};
      // Sum/diff mapping: sum -> forward, diff -> yaw command
      j.forward = std::clamp(0.5f * (ly + ry), -1.0f, 1.0f);
      j.turn = std::clamp(0.5f * (ry - ly), -1.0f, 1.0f);
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
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  PigpioCtx _ctx; // your pigpio context wrapper
  App app;
  return app.run(_ctx, false);
}