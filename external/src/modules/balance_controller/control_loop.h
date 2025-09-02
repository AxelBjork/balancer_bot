#pragma once
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <thread>
#include <algorithm>
#include <functional>

#include "config.h"


// ---- Simple speed estimator ----
// Smooths commanded sps into an "actual" estimate using a 1st-order LPF.
struct SpeedEstimator {
  void reset() {
    v_left_sps.store(0.0, std::memory_order_relaxed);
    v_right_sps.store(0.0, std::memory_order_relaxed);
  }

  void updateTargets(double left_cmd_sps, double right_cmd_sps, double dt_s) {
    const double alpha = 1.0 - std::exp(-2.0 * M_PI * Config::fc_velocity_hz * dt_s);
    const double vl = v_left_sps.load(std::memory_order_relaxed);
    const double vr = v_right_sps.load(std::memory_order_relaxed);
    v_left_sps.store((1.0 - alpha) * vl + alpha * left_cmd_sps, std::memory_order_relaxed);
    v_right_sps.store((1.0 - alpha) * vr + alpha * right_cmd_sps, std::memory_order_relaxed);
  }

  double leftSps() const { return v_left_sps.load(std::memory_order_relaxed); }
  double rightSps() const { return v_right_sps.load(std::memory_order_relaxed); }

private:
  std::atomic<double> v_left_sps{0.0};
  std::atomic<double> v_right_sps{0.0};
};

/*
 * LQR-based two-wheel balancer (balance-in-place only).
 * - Keeps telemetry fields compatible with previous code.
 * - Velocity and yaw control are disabled/zeroed.
 * - Balance computed via a_cmd = -K * [theta, theta_dot, x, v].
 * - Motor command u (steps/s) is integrated from acceleration with rate/amp limits.
 */

template <class MotorRunnerT>
class CascadedController {
public:
  CascadedController(MotorRunnerT& left, MotorRunnerT& right)
    : left_(left), right_(right) {
    worker_ = std::thread(&CascadedController::loop, this);
  }

  ~CascadedController() {
    stop();
  }

  void stop() {
    bool exp = true;
    if (alive_.compare_exchange_strong(exp, false)) {
      if (worker_.joinable()) {
        worker_.join();
      }
    }
  }

  // Push latest IMU sample (call from your IMU ISR/reader)
  void pushImu(const ImuSample& s) {
    latest_imu_.store(s, std::memory_order_relaxed);
  }

  // Joystick kept for API compatibility; ignored in LQR-only mode.
  void setJoystick(const JoyCmd& j) {
    joy_.store(j, std::memory_order_relaxed);
  }

  void setTelemetrySink(std::function<void(const Telemetry&)> cb) {
    tel_cb_ = std::move(cb);
  }

  // For telemetry/debug
  double lastTiltTargetRad() const { return 0.0; } // LQR holds 0
  double lastUBalanceSps() const { return u_balance_sps_.load(std::memory_order_relaxed); }

private:
  // Minimal lock-free “latest value” wrapper
  struct AtomicImu {
    std::atomic<double> angle{0.0}; // pitch (rad, +forward)
    std::atomic<double> gyro{0.0};  // pitch rate (rad/s, +nose-down)
    std::atomic<double> yaw_z{0.0}; // yaw rate (rad/s) (unused here)
    std::atomic<std::int64_t> t_count{0};

    void store(const ImuSample& s, std::memory_order mo) {
      angle.store(s.angle_rad, mo);
      gyro.store(s.gyro_rad_s, mo);
      yaw_z.store(s.yaw_rate_z, mo);
      const auto d = s.t.time_since_epoch();
      t_count.store(static_cast<std::int64_t>(d.count()), mo);
    }
    ImuSample load(std::memory_order mo) const {
      ImuSample s{};
      s.angle_rad  = angle.load(mo);
      s.gyro_rad_s = gyro.load(mo);
      s.yaw_rate_z = yaw_z.load(mo);
      using clock = std::chrono::steady_clock;
      const auto cnt = static_cast<clock::duration::rep>(t_count.load(mo));
      s.t = std::chrono::time_point<clock>(clock::duration(cnt));
      return s;
    }
  };

  struct AtomicJoy {
    std::atomic<float> f{0.0f}, t{0.0f};
    void store(const JoyCmd& j, std::memory_order mo) {
      f.store(j.forward, mo);
      t.store(j.turn, mo);
    }
    JoyCmd load(std::memory_order mo) const {
      return JoyCmd{ f.load(mo), t.load(mo) };
    }
  };


  void loop() {
    using dur = std::chrono::steady_clock::duration;

    const auto dt_bal = std::chrono::duration_cast<dur>(
        std::chrono::duration<double>(1.0 / std::max(50, Config::hz_balance)));
    const double dt_sec = std::chrono::duration<double>(dt_bal).count();

    // Steps→rad/s and m/s scaling
    const double ku = 2.0 * M_PI / static_cast<double>(Config::steps_per_rev); // rad/s per (steps/s)
    const double r  = Config::wheel_radius_m;                                  // m per rad
    const double sps_to_mps = ku * r;

    // Speed estimator for base velocity (from commanded sps)
    SpeedEstimator v_est; v_est.reset();

    // Command state
    double u_prev = 0.0;               // steps/s (shared for both wheels here)
    double cmd_left_sps  = 0.0;
    double cmd_right_sps = 0.0;

    auto t_next_bal = std::chrono::steady_clock::now();

    while (alive_.load(std::memory_order_relaxed) && !g_stop.load(std::memory_order_relaxed)) {
      const auto now = std::chrono::steady_clock::now();

      if (now >= t_next_bal) {
        t_next_bal += dt_bal;

        // ---- Read sensors ----
        const ImuSample s = latest_imu_.load(std::memory_order_relaxed);
        const double theta     = s.angle_rad;   // rad
        const double theta_dot = s.gyro_rad_s;  // rad/s

        // ---- Update velocity estimate from current commanded speeds ----
        v_est.updateTargets(cmd_left_sps, cmd_right_sps, dt_sec);
        const double base_sps_est = 0.5 * (v_est.leftSps() + v_est.rightSps()); // steps/s
        const double x_vel = base_sps_est * sps_to_mps; // m/s

      // ---- LQR control (per-term + total acceleration, m/s^2) ----
      const double a_theta  = -(Config::lqr_k_theta  * theta);
      const double a_dtheta = -(Config::lqr_k_dtheta * theta_dot);
      const double a_v      = -(Config::lqr_k_v      * x_vel);
      const double a_cmd    = (a_theta + a_dtheta + a_v);

      // ---- Integrate to steps/s with rate limit + leak ----
      double du = (a_cmd / (r * ku)) * dt_sec;           // steps/s increment
      bool du_rate_limited = false;
      const double max_du = Config::max_du_per_sec * dt_sec;
      if (du >  max_du) { du =  max_du; du_rate_limited = true; }
      if (du < -max_du) { du = -max_du; du_rate_limited = true; }

      double u_try = u_prev + du;

      // amplitude anti-windup (only if pushing further into rail)
      bool u_amp_limited = false;
      if (u_try >  Config::max_sps && u_prev >= Config::max_sps && du > 0.0) { du = 0.0; u_try = Config::max_sps;  u_amp_limited = true; }
      if (u_try < -Config::max_sps && u_prev <=-Config::max_sps && du < 0.0) { du = 0.0; u_try = -Config::max_sps; u_amp_limited = true; }

      u_prev = u_try;

      // leak toward 0 so u doesn't "run away"
      u_prev -= (u_prev * dt_sec) / Config::tau_u_s;

      double u_balance = std::clamp(u_prev, -Config::max_sps, Config::max_sps);
      u_balance_sps_.store(u_balance, std::memory_order_relaxed);

      // ---- Split & clamp (no steering/yaw in this mode) ----
      const double steer_split = 0.0;
      double left  = std::clamp(u_balance + steer_split,  -Config::max_sps, Config::max_sps);
      double right = std::clamp(u_balance - steer_split,  -Config::max_sps, Config::max_sps);
      cmd_left_sps  = left;
      cmd_right_sps = right;

      left_.setTarget(cmd_left_sps);
      right_.setTarget(cmd_right_sps);

      // ---- Telemetry ----
      emitTelemetryNow(now,
                      theta,
                      theta_dot,
                      x_vel,
                      a_theta,
                      a_dtheta,
                      a_v,
                      a_cmd,
                      du,
                      du_rate_limited,
                      u_balance,
                      u_amp_limited,
                      left,
                      right);
      }
      // small sleep to avoid busy loop
      std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
  }

void emitTelemetryNow(std::chrono::steady_clock::time_point now,
                      double tilt, double gyro,
                      double x_vel_est_mps,
                      double a_theta_mps2,
                      double a_dtheta_mps2,
                      double a_v_mps2,
                      double a_cmd_mps2,
                      double du_sps,
                      bool   du_rate_limited,
                      double u_balance_sps,
                      bool   u_amp_limited,
                      double left_cmd, double right_cmd) {
    if (!tel_cb_) return;
    Telemetry t;
    t.ts               = now;
    t.tilt_rad         = tilt;
    t.gyro_rad_s       = gyro;
    t.x_vel_est_mps    = x_vel_est_mps;

    t.a_theta_mps2     = a_theta_mps2;
    t.a_dtheta_mps2    = a_dtheta_mps2;
    t.a_v_mps2         = a_v_mps2;
    t.a_cmd_mps2       = a_cmd_mps2;

    t.du_sps           = du_sps;
    t.du_rate_limited  = du_rate_limited;
    t.u_balance_sps    = u_balance_sps;
    t.u_amp_limited    = u_amp_limited;

    t.left_cmd_sps     = left_cmd;
    t.right_cmd_sps    = right_cmd;

    tel_cb_(t);
  }

private:
  MotorRunnerT& left_;
  MotorRunnerT& right_;

  std::atomic<bool> alive_{true};
  std::thread worker_;

  // State shared with loop
  AtomicImu latest_imu_;
  AtomicJoy joy_; // retained for API compat; unused
  std::function<void(const Telemetry&)> tel_cb_{};
  std::atomic<double> u_balance_sps_{0.0};

  // Telemetry scratch (kept for structure parity; unused here)
  double steer_split_sps_{0.0};
  double tel_desired_base_sps_ = 0.0;
  double tel_actual_base_sps_  = 0.0;
  double tel_vel_err_sps_      = 0.0;
  double tel_vel_int_state_    = 0.0;
  double yaw_int_ = 0.0;
  double tel_desired_yaw_rate_ = 0.0;
  double tel_actual_yaw_rate_  = 0.0;
  double tel_yaw_err_          = 0.0;
  double tel_yaw_int_          = 0.0;
  bool   tel_tilt_saturated_   = false;
};
