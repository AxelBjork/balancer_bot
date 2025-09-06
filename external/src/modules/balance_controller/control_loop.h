#pragma once
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <thread>

#include "config.h"

static inline double rad2deg(double x) { return x * (180.0 / M_PI); }


/*
 * LQR-based two-wheel balancer (balance-in-place only).
 * - Keeps telemetry fields compatible with previous code.
 * - Velocity and yaw control are disabled/zeroed.
 * - Balance computed via a_cmd = -K * [theta, theta_dot, x, v].
 * - Motor command u (steps/s) is integrated from acceleration with rate/amp
 * limits.
 */

template <class MotorRunnerT> class CascadedController {
public:
  CascadedController(MotorRunnerT &left, MotorRunnerT &right)
      : left_(left), right_(right) {
    worker_ = std::thread(&CascadedController::loop, this);
  }

  ~CascadedController() { stop(); }

  void stop() {
    bool exp = true;
    if (alive_.compare_exchange_strong(exp, false)) {
      if (worker_.joinable()) {
        worker_.join();
      }
    }
  }

  // Push latest IMU sample (call from your IMU ISR/reader)
  void pushImu(const ImuSample &s) {
    latest_imu_.store(s, std::memory_order_relaxed);
  }

  // Joystick kept for API compatibility; ignored in LQR-only mode.
  void setJoystick(const JoyCmd &j) {
    joy_.store(j, std::memory_order_relaxed);
  }

  void setTelemetrySink(std::function<void(const Telemetry &)> cb) {
    tel_cb_ = std::move(cb);
  }

  // For telemetry/debug
  double lastTiltTargetRad() const { return 0.0; } // LQR holds 0
  double lastUBalanceSps() const {
    return u_balance_sps_.load(std::memory_order_relaxed);
  }

private:
  // Minimal lock-free “latest value” wrapper
  struct AtomicImu {
    std::atomic<double> angle{0.0}; // pitch (rad, +forward)
    std::atomic<double> gyro{0.0};  // pitch rate (rad/s, +nose-down)
    std::atomic<double> yaw_z{0.0}; // yaw rate (rad/s) (unused here)
    std::atomic<std::int64_t> t_count{0};

    void store(const ImuSample &s, std::memory_order mo) {
      angle.store(s.angle_rad, mo);
      gyro.store(s.gyro_rad_s, mo);
      yaw_z.store(s.yaw_rate_z, mo);
      const auto d = s.t.time_since_epoch();
      t_count.store(static_cast<std::int64_t>(d.count()), mo);
    }
    ImuSample load(std::memory_order mo) const {
      ImuSample s{};
      s.angle_rad = angle.load(mo);
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
    void store(const JoyCmd &j, std::memory_order mo) {
      f.store(j.forward, mo);
      t.store(j.turn, mo);
    }
    JoyCmd load(std::memory_order mo) const {
      return JoyCmd{f.load(mo), t.load(mo)};
    }
  };
void loop() {
  using dur = std::chrono::steady_clock::duration;
  const auto dt_bal = std::chrono::duration_cast<dur>(
      std::chrono::duration<double>(1.0 / std::max(50, Config::hz_balance)));
  const double dt = std::chrono::duration<double>(dt_bal).count();

  // geometry/scales
  const double ku = 2.0 * M_PI / double(Config::steps_per_rev); // [rad/step]
  const double r  = Config::wheel_radius_m;                      // [m]
  const double m_per_step = r * ku;                              // [m/step]

  // --- local "niche" tunables kept inside the function ---
  const double a_max_mps2              = 6.0;    // soft cap on commanded accel [m/s^2]
  const double tau_aw_s                = 0.03;   // anti-windup back-calc time-constant

  // NEW: slow bias integrator on u (steps/s) to cancel gravity/static friction at small tilt
  const double ki_u_steps_per_rad_s    = 12000.0;                 // (steps/s) per (rad*s)
  const double tau_i_s                 = 0.8;                     // decay on bias integrator [s]
  const double v_freeze_mps            = 0.12;                    // freeze I when moving faster
  const double th_freeze_rad           = 15.0 * (M_PI / 180.0);   // freeze I when tipped far

  // state
  double u_int  = 0.0; // internal (pre-clamp) base command [steps/s] from accel integration path
  double u_bias = 0.0; // NEW: slow bias (steps/s) integrated from angle error

  auto t_next = std::chrono::steady_clock::now();
  while (alive_.load(std::memory_order_relaxed) && !g_stop.load(std::memory_order_relaxed)) {

    if (std::chrono::steady_clock::now() >= t_next) {
      t_next += dt_bal;

      // --- sensors ---
      const ImuSample s = latest_imu_.load(std::memory_order_relaxed);
      const double th   = s.angle_rad;
      const double w    = s.gyro_rad_s;

      // lead
      const double th_pred = th + Config::lead_T_s * w;

      // measured forward speed (for av)
      const double sps_meas  = 0.5 * (left_.actualSps() + right_.actualSps());
      const double v_meas    = sps_meas * m_per_step; // [m/s]

      // --- LQR → accel (m/s^2), softly limited ---
      const double a_theta  = -(Config::lqr_k_theta  * th_pred);
      const double a_dtheta = -(Config::lqr_k_dtheta * w);
      const double a_v      = -(Config::lqr_k_v      * v_meas);
      double a_cmd = a_theta + a_dtheta + a_v;
      a_cmd = std::clamp(a_cmd, -a_max_mps2, a_max_mps2);

      // accel → Δu (steps/s)
      double du = (a_cmd / m_per_step) * dt; // [steps/s]

      // rate-limit Δu
      const double max_du = Config::max_du_per_sec * dt;
      bool du_limited = false;
      if (du >  max_du) { du =  max_du; du_limited = true; }
      if (du < -max_du) { du = -max_du; du_limited = true; }

      // ---------- slow angle-bias integrator directly on u (steps/s) ----------
      // freeze conditions for the bias integrator
      const bool i_freeze =
        (std::abs(v_meas) > v_freeze_mps) ||
        (std::abs(th_pred) > th_freeze_rad);

      // predict unclamped sum for I anti-windup decision
      const double u_sum_pred = (u_int + du) + u_bias;

      // if we’re already at clamp and the angle sign would push further into clamp, freeze I
      const bool pushing_i_into_clamp =
        (u_sum_pred >  Config::max_sps && th_pred > 0.0) ||
        (u_sum_pred < -Config::max_sps && th_pred < 0.0);

      if (!i_freeze && !pushing_i_into_clamp) {
        // integrate angle error (rad) into steps/s bias
        u_bias += ki_u_steps_per_rad_s * th_pred * dt;
      }
      // small leak so it lets go once recovered
      u_bias -= (u_bias * dt) / std::max(0.2, tau_i_s);

      // ---------- base integrator path (your original) ----------
      // predict unclamped next command of base integrator
      const double u_unclamped_int = u_int + du;

      // leak (bias drain) on the base integrator *before* clamp
      const double leak = (u_unclamped_int * dt) / Config::tau_u_s;
      const double u_leaked_int = u_unclamped_int - leak;

      // sum base + bias, then hard clamp to actuator limits
      const double u_sum_leaked = u_leaked_int + u_bias;
      const double u_clamped    = std::clamp(u_sum_leaked, -Config::max_sps, Config::max_sps);
      const bool amp_limited    = (u_clamped != u_sum_leaked);

      // ---------- anti-windup on the base integrator ----------
      // 1) Conditional integration: if we’re clamped and du pushes further into the clamp, drop this step
      const bool pushing_into_clamp =
        (amp_limited && ((u_sum_leaked >  u_clamped && du > 0.0) ||
                         (u_sum_leaked <  u_clamped && du < 0.0)));

      double u_next_int = pushing_into_clamp ? u_int : u_leaked_int;

      // 2) Back-calculation pull of the *base* integrator toward the clamped value minus current bias
      const double kaw          = dt / tau_aw_s;
      const double u_target_int = u_clamped - u_bias; // clamp acts on the sum, so pull base toward (clamp - bias)
      u_next_int += kaw * (u_target_int - u_next_int);

      // finalize internal base state
      u_int = u_next_int;

      // deadzone on the value we actually send (apply to the clamped sum)
      const double dead = Config::deadzone_frac * Config::max_sps;
      double u_out = (std::abs(u_clamped) < dead) ? 0.0 : u_clamped;

      // split (no steer here)
      const double left_cmd  = std::clamp(u_out, -Config::max_sps, Config::max_sps);
      const double right_cmd = left_cmd;

      left_.setTarget(left_cmd);
      right_.setTarget(right_cmd);
      u_balance_sps_.store(u_out, std::memory_order_relaxed);

      // telemetry (unchanged fields + your flags)
      emitTelemetryNow(std::chrono::steady_clock::now(),
        th, w, v_meas,
        a_theta, a_dtheta, a_v, a_cmd,
        du, du_limited, u_out, amp_limited,
        left_cmd, right_cmd);
    }
  }
}
  


  void emitTelemetryNow(std::chrono::steady_clock::time_point now, double tilt,
                        double gyro, double x_vel_est_mps, double a_theta_mps2,
                        double a_dtheta_mps2, double a_v_mps2,
                        double a_cmd_mps2, double du_sps, bool du_rate_limited,
                        double u_balance_sps, bool u_amp_limited,
                        double left_cmd, double right_cmd) {
    if (!tel_cb_)
      return;
    Telemetry t;
    t.ts = now;
    t.tilt_rad = tilt;
    t.gyro_rad_s = gyro;
    t.x_vel_est_mps = x_vel_est_mps;

    t.a_theta_mps2 = a_theta_mps2;
    t.a_dtheta_mps2 = a_dtheta_mps2;
    t.a_v_mps2 = a_v_mps2;
    t.a_cmd_mps2 = a_cmd_mps2;

    t.du_sps = du_sps;
    t.du_rate_limited = du_rate_limited;
    t.u_balance_sps = u_balance_sps;
    t.u_amp_limited = u_amp_limited;

    t.left_cmd_sps = left_cmd;
    t.right_cmd_sps = right_cmd;

    tel_cb_(t);
  }

private:
  MotorRunnerT &left_;
  MotorRunnerT &right_;

  std::atomic<bool> alive_{true};
  std::thread worker_;

  // State shared with loop
  AtomicImu latest_imu_;
  AtomicJoy joy_; // retained for API compat; unused
  std::function<void(const Telemetry &)> tel_cb_{};
  std::atomic<double> u_balance_sps_{0.0};

  // Telemetry scratch (kept for structure parity; unused here)
  double steer_split_sps_{0.0};
  double tel_desired_base_sps_ = 0.0;
  double tel_actual_base_sps_ = 0.0;
  double tel_vel_err_sps_ = 0.0;
  double tel_vel_int_state_ = 0.0;
  double yaw_int_ = 0.0;
  double tel_desired_yaw_rate_ = 0.0;
  double tel_actual_yaw_rate_ = 0.0;
  double tel_yaw_err_ = 0.0;
  double tel_yaw_int_ = 0.0;
  bool tel_tilt_saturated_ = false;
};
