#pragma once

#include <pigpiod_if2.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <thread>

#include "config.h"
#include "control_loop.h"
#include "ism330_iio_reader.h"
#include "motor_runner.h"
#include "pitch_lpf.h"
#include "stepper.h"
#include "xbox_controller.h"

struct PigpioCtx {
  explicit PigpioCtx(const char* host = nullptr, const char* port = nullptr) {
    pi = pigpio_start(const_cast<char*>(host), const_cast<char*>(port));
    if (pi < 0) throw std::runtime_error("pigpio_start failed");
  }
  ~PigpioCtx() {
    pigpio_stop(pi);
  }
  int handle() const {
    return pi;
  }

 private:
  int pi{};
};

template <class MotorRunnerT>
class CascadedController {
 public:
  CascadedController(MotorRunnerT& motors) : motors_(motors) {
    core_.setMotorOutputs(
        [this](float left_sps, float right_sps) { motors_.setTargets(left_sps, right_sps); });

    // Velocity feedback from motor commanded targets (actual step tracking)
    core_.setVelocityFeedback([this]() -> float { return motors_.getActualSpeedSps(); });

    core_.start();
  }

  ~CascadedController() {
    core_.stop();
  }

  void pushImu(const ImuSample& s) {
    core_.pushImu(s);
  }
  void setJoystick(const JoyCmd& j) {
    core_.setJoystick(j);
  }
  void setTelemetrySink(std::function<void(const Telemetry&)> cb) {
    core_.setTelemetrySink(std::move(cb));
  }

 private:
  MotorRunnerT& motors_;
  RateControllerCore core_;
};

// ---------------------- Motor control runner --------------------------------
class ControlApp {
 public:
  int run(PigpioCtx& _ctx, bool xbox_control = true) {
    if (xbox_control) {
      pad = std::make_unique<XboxController>();
    }
    // Hardware setup
    Stepper::Pins leftPins{12, 19, 13};  // ENA, STEP(PWM1), DIR
    Stepper::Pins rightPins{4, 18, 24};  // ENB, STEP(PWM0), DIR

    Stepper left(_ctx.handle(), leftPins, Config::invert_left, /*energize_now=*/true);
    Stepper right(_ctx.handle(), rightPins, Config::invert_right, /*energize_now=*/true);

    // Coordinator at 1 kHz
    MotorRunner motors(left, right, Config::control_hz, 250000.0);

    // Telemetry counter (must outlive ctrl)
    std::atomic<int> k{0};

    // Start cascaded controller (runs its own thread)
    CascadedController<MotorRunner> ctrl(motors);

    // Optional: telemetry print every N balance ticks for quick visibility
    // Set to 0 to disable.
    if constexpr (Config::kPrintEvery != -1) {
      ctrl.setTelemetrySink([&](const Telemetry& t) {
        if ((++k % Config::kPrintEvery) == 0) {
          std::printf(
              "t=%7.3f  θ=%6.2f°  θ̇=%6.2f°/s  r_sp=%6.2f°/s  out=%6.3f  "
              "u=%6.0f%s  I=%7.3f  psp=%5.2f  ve=%5.1f  vi=%5.3f  vp=%5.3f\n",
              t.t_sec, t.pitch_deg, t.pitch_rate_dps, t.rate_sp_dps, t.out_norm, t.u_sps,
              (std::abs(t.u_sps) >= 0.99 * kMaxSps) ? "*" : "",  // rail hint
              t.integ_pitch, t.pitch_sp_deg, t.vel_error, t.vel_i_term, t.vel_p_term);
        }
      });
    }

    // IMU reader (IIO; runs its own thread). If not found, we’ll fall back to
    // zeros.
    PitchComplementaryFilter filt{};
    std::unique_ptr<Ism330IioReader> imu;
    try {
      Ism330IioReader::IMUConfig icfg;

      icfg.on_sample = [&](double pitch, std::array<double, 3> acc, std::array<double, 3> gyrv,
                           std::chrono::steady_clock::time_point ts) {
        filt.push_sample(acc, gyrv, ts);

        ImuSample s = filt.read_latest();
        ctrl.pushImu(s);
        static int k = 0;
        if ((++k % 10) == -1) {
          static bool hdr = false;
          if (!hdr) {
            std::printf(
                "pitch_f_deg,pitch_r_deg,dtheta_deg,dpitch_f_dps,"
                "dpitch_r_dps,domega_dps\n");
            hdr = true;
          }
          std::printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", s.angle_rad * 180.0 / M_PI,
                      pitch * 180.0 / M_PI, (s.angle_rad - pitch) * 180.0 / M_PI,
                      s.gyro_rad_s * 180.0 / M_PI, gyrv[1] * 180.0 / M_PI,
                      (s.gyro_rad_s - gyrv[1]) * 180.0 / M_PI);
        }
      };
      imu = std::make_unique<Ism330IioReader>(std::move(icfg));
      std::cout << "IIO IMU started at " << imu->devnode() << "\n";
    } catch (const std::exception& e) {
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
    // Run for Config::run_seconds of SIMULATION time (not wall-clock time)
    // speedup affects wall-clock execution speed, not simulation duration
    const auto t_end =
        std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::nanoseconds>(
                                               std::chrono::duration<double>(Config::run_seconds));
    const auto tick = std::chrono::duration<double, std::milli>(1000.0 / Config::command_hz);

    while (std::chrono::steady_clock::now() < t_end && !g_stop.load(std::memory_order_relaxed)) {
      float ly = 0.0;
      float ry = 0;

      if (xbox_control) {
        pad->update();
        // Arcade Drive: Left Stick Y = Forward/Swap, Right Stick X = Turn
        // Y-axis is often -1 (Up) to +1 (Down), so invert for Forward.
        ly = -pad->leftY();
        ry = pad->rightX();
      }
      JoyCmd j{.forward = ly, .turn = ry};
      ctrl.setJoystick(j);

      std::this_thread::sleep_for(tick);
    }

    // shutdown
    g_stop.store(true, std::memory_order_relaxed);
    // ctrl destructor joins its thread; MotorRunner has stop()
    return 0;
  }
  std::unique_ptr<XboxController> pad;
};
