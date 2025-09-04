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
  const double dt_sec = std::chrono::duration<double>(dt_bal).count();

  // Steps→rad/s and m/s scaling
  const double ku = 2.0 * M_PI / static_cast<double>(Config::steps_per_rev);
  const double r  = Config::wheel_radius_m;
  const double sps_to_mps = ku * r;

  double u_prev = 0.0;        // shared for both wheels
  double cmd_left_sps = 0.0;
  double cmd_right_sps = 0.0;

  // Dead-zone memory
  static double dz_state = 0.0;
  const double dead = Config::deadzone_frac * Config::max_sps;

  // Local helpers (not in Config)
  constexpr double kCrossBoost = 4.0;         // dynamic slew boost on reversal/toward-zero
  constexpr double kJerkTau_s  = 0.0025;      // tiny jerk filter on du
  // Low-rate angle boost: full boost at 0 dps, fades out by ~40 dps
  constexpr double kLowRateDps = 40.0;        // “small rate” scale
  constexpr double kThetaBoostMax = 0.8;      // up to +80% more θ gain at very low |θ̇|

  static double du_filt = 0.0;

  auto t_next_bal = std::chrono::steady_clock::now();

  while (alive_.load(std::memory_order_relaxed) && !g_stop.load(std::memory_order_relaxed)) {
    const auto now = std::chrono::steady_clock::now();

    if (now >= t_next_bal) {
      t_next_bal += dt_bal;

      // ---- Read sensors ----
      const ImuSample s = latest_imu_.load(std::memory_order_relaxed);
      const double theta     = s.angle_rad;   // pitch [rad]
      const double theta_dot = s.gyro_rad_s;  // filtered gyro [rad/s]

      // ---- Phase lead (predict a little into the future) ----
      const double theta_pred = theta + Config::lead_T_s * theta_dot;

      // ---- Measured base velocity from runners (steps/s → m/s) ----
      const double left_sps_meas  = left_.actualSps();
      const double right_sps_meas = right_.actualSps();
      const double base_sps_meas  = 0.5 * (left_sps_meas + right_sps_meas);
      const double x_vel          = base_sps_meas * sps_to_mps;

      // ---- LQR (with low-rate angle boost) ----
      // Boost θ contribution when |θ̇| is small to overcome leak / inertia early
      const double rate_dps = std::abs(rad2deg(theta_dot));
      const double rate_scale = std::clamp(rate_dps / kLowRateDps, 0.0, 1.0);
      const double theta_boost = 1.0 + kThetaBoostMax * (1.0 - rate_scale); // ∈ [1, 1+boost]

      const double a_theta  = -(Config::lqr_k_theta  * theta_pred) * theta_boost;
      const double a_dtheta = -(Config::lqr_k_dtheta * theta_dot);
      const double a_v      = -(Config::lqr_k_v      * x_vel);
      const double a_cmd    = (a_theta + a_dtheta + a_v);

      // ---- Integrate to steps/s with dynamic rate limit → leak → desat ----
      double du = (a_cmd / (r * ku)) * dt_sec;

      // Dynamic slew: faster when reversing / pushing toward zero (no new Config)
      const double u_des        = u_prev + du;
      const bool reversing      = (u_prev > 0.0 && u_des < 0.0) || (u_prev < 0.0 && u_des > 0.0);
      const bool toward_zero    = (u_prev * a_cmd) < 0.0;

      double max_du = Config::max_du_per_sec * (reversing || toward_zero ? kCrossBoost : 1.0);
      max_du *= dt_sec;

      bool du_rate_limited = false;
      if (du >  max_du) { du =  max_du; du_rate_limited = true; }
      if (du < -max_du) { du = -max_du; du_rate_limited = true; }

      // tiny jerk smoothing to avoid edgey reversals
      const double a_jerk = std::exp(-dt_sec / std::max(1e-6, kJerkTau_s));
      const double du_smooth = a_jerk * du_filt + (1.0 - a_jerk) * du;
      du_filt = du_smooth;

      u_prev += du_smooth;

      // leak: consider increasing Config::tau_u_s slightly if still “too leaky”
      u_prev -= (u_prev * dt_sec) / Config::tau_u_s;

      // clamp + soft desaturation
      const double u_clamped = std::clamp(u_prev, -Config::max_sps, Config::max_sps);
      const bool   u_amp_limited = (u_clamped != u_prev);
      u_prev += Config::desat_alpha * (u_clamped - u_prev);
      double u_balance = u_clamped;

      // ---- Dead-zone only when near upright and slow ----
      const bool near_upright = std::abs(theta) < 0.5 * Config::max_tilt_rad;
      const bool slow_rate    = rate_dps < (2.0 * Config::still_max_rate_dps);

      if (near_upright && slow_rate && std::abs(u_balance) < dead) {
        u_balance = 0.0;
      } else {
        dz_state = (u_balance > 0.0) ? 1.0 : (u_balance < 0.0 ? -1.0 : dz_state);
      }

      u_balance_sps_.store(u_balance, std::memory_order_relaxed);

      // ---- Split & clamp (no steer) ----
      const double steer_split = 0.0;
      double left  = std::clamp(u_balance + steer_split, -Config::max_sps, Config::max_sps);
      double right = std::clamp(u_balance - steer_split, -Config::max_sps, Config::max_sps);
      cmd_left_sps = left;
      cmd_right_sps = right;

      left_.setTarget(cmd_left_sps);
      right_.setTarget(cmd_right_sps);

      // ---- Telemetry ----
      emitTelemetryNow(now, theta, theta_dot, x_vel,
                       a_theta, a_dtheta, a_v, a_cmd, du_smooth,
                       du_rate_limited, u_balance, u_amp_limited,
                       left, right);
    }

    std::this_thread::sleep_for(std::chrono::microseconds(50));
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
