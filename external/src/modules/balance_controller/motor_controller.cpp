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


// ---------------------- Motor control runner --------------------------------
class App {
public:
  int run(PigpioCtx& _ctx) {
    XboxController pad;

    // Hardware setup
    Stepper::Pins leftPins  {12, 19, 13}; // ENA, STEP, DIR
    Stepper::Pins rightPins { 4, 18, 24}; // ENB, STEP, DIR
    Stepper left(_ctx.handle(), leftPins), right(_ctx.handle(), rightPins);
    MotorRunner L(left), R(right);

    // Controller tunings (good starting points; adjust on hardware)
    ControlTunings gains{};
    gains.hz_balance       = 400;
    gains.hz_outer         = 100;
    gains.max_tilt_rad     = 6.0 * (M_PI / 180.0);
    gains.max_sps          = Config::max_sps;
    // Balance PD (maps tilt error -> sps)
    gains.kp_bal           = 60.0;
    gains.kd_bal           = 1.2;
    // Velocity PI (maps speed error -> tilt target)
    gains.kp_vel           = 1.2;
    gains.ki_vel           = 0.4;
    // Yaw PI (maps yaw-rate error -> steering split)
    gains.yaw_pi_enabled   = true;
    gains.kp_yaw           = 250.0;
    gains.ki_yaw           = 80.0;
    gains.max_yaw_rate_cmd = 1.5;          // rad/s at full stick
    gains.max_steer_sps    = 800.0;        // clamp left/right split

    // Start cascaded controller (runs its own thread)
    CascadedController<MotorRunner> ctrl(L, R, gains);

    // Optional: telemetry print every N balance ticks for quick visibility
    // Set to 0 to disable.
    constexpr int kPrintEvery = -1;
    if constexpr (kPrintEvery != -1) {
      std::atomic<int> k{0};
      ctrl.setTelemetrySink([&](const Telemetry& t){
        if ((++k % kPrintEvery) == 0) {
          std::printf("tilt=%.2f° tgt=%.2f° u=%.0f L=%.0f R=%.0f yawDes=%.2f yaw=%.2f\n",
            t.tilt_rad * 180.0/M_PI, t.tilt_target_rad * 180.0/M_PI,
            t.u_balance_sps, t.left_cmd_sps, t.right_cmd_sps,
            t.desired_yaw_rate, t.actual_yaw_rate);
        }
      });
    }

    // IMU reader (IIO; runs its own thread). If not found, we’ll fall back to zeros.
    std::unique_ptr<Ism330IioReader> imu;
    try {
      Ism330IioReader::Config icfg;
      icfg.sampling_hz = 1000.0;
      // Axis mapping: adjust if your board is rotated. Defaults assume:
      //  pitch uses accel X vs Z and gyro Y; yaw uses gyro Z.
      icfg.accel_x = 0; icfg.accel_y = 1; icfg.accel_z = 2;
      icfg.gyro_y  = 1; icfg.gyro_z  = 2;
      icfg.on_sample = [&](double pitch, double pitch_rate, double yaw_rate,
                           std::chrono::steady_clock::time_point ts){
        ImuSample s;
        s.angle_rad   = pitch;
        s.gyro_rad_s  = pitch_rate;
        s.yaw_rate_z  = yaw_rate;
        s.t           = ts;
        ctrl.pushImu(s);
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
      pad.update();

      // Sticks: map to forward & turn in [-1, 1].
      float ly = pad.leftY();
      float ry = pad.rightY();
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
};

// --------------------------- main -------------------------------------------
int main() {
  std::signal(SIGINT,  on_signal);
  std::signal(SIGTERM, on_signal);

  PigpioCtx _ctx;   // your pigpio context wrapper
  App app;
  return app.run(_ctx);
}