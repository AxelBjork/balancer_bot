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

  // ====== Local/Niche knobs (stay inside this function) ======
  const double a_max_mps2       = 6.0;     // soft cap on accel [m/s^2]
  const double tau_aw_s         = 0.030;   // back-calc anti-windup τ (amplitude clamp)
  const double tau_aw_slew_s    = 0.020;   // back-calc anti-windup τ (slew clamp)
  const double tau_jerk_s       = 0.020;   // accel 1st-order shaping (jerk limit)
  const double du_boost_factor  = 5.0;     // extra slew when falling (× base)
  const double th_fall_rad      = 0.35;    // ~20°
  const double w_fall_rad_s     = 2.5 * M_PI; // ~450°/s
  const double kcap_th          = 0.80;    // capture velocity = kθ*θ + kw*θ̇
  const double kcap_w           = 0.18;
  const double v_capture_max    = 3.0;     // [m/s] cap capture speed

  const double ki_theta        = 0.55;   // [ (m/s^2) / (rad·s) ], tiny integral on angle
  const double tau_i_leak_s    = 3.0;    // slow leak on integral (prevents creep if left leaning on a wall)
  const double a_i_abs_max     = 0.35;   //  cap integral’s accel contribution
  static double th_int = 0.0;            // ∫θ dt state

  // ====== Timing ======
  const auto dt_bal = std::chrono::duration_cast<dur>(
      std::chrono::duration<double>(1.0 / std::max(50, Config::hz_balance)));
  const double dt = std::chrono::duration<double>(dt_bal).count();

  // ====== Geometry / scales ======
  const double ku = 2.0 * M_PI / double(Config::steps_per_rev); // [rad/step]
  const double r  = Config::wheel_radius_m;                     // [m]
  const double m_per_step = r * ku;                             // [m/step]

  // ====== Persistent state inside this function ======
  static double u_int     = 0.0;  // internal (pre-clamp) command [steps/s]
  static double a_cmd_f   = 0.0;  // jerk-shaped accel [m/s^2]

  auto t_next = std::chrono::steady_clock::now();
  while (alive_.load(std::memory_order_relaxed) && !g_stop.load(std::memory_order_relaxed)) {
    if (std::chrono::steady_clock::now() < t_next) {
      std::this_thread::sleep_for(std::chrono::microseconds(50));
      continue;
    }
    t_next += dt_bal;

    // --- sensors ---
    const ImuSample s = latest_imu_.load(std::memory_order_relaxed);
    const double th   = s.angle_rad;
    const double w    = s.gyro_rad_s;

    // lead
    const double th_pred = th + Config::lead_T_s * w;

    // measured forward speed (for a_v)
    const double sps_meas  = 0.5 * (left_.actualSps() + right_.actualSps());
    const double v_meas    = sps_meas * m_per_step; // [m/s]

    // --- LQR → accel (m/s^2) ---
    const double a_theta  = -(Config::lqr_k_theta  * th_pred);
    const double a_dtheta = -(Config::lqr_k_dtheta * w);
    const double a_v      = -(Config::lqr_k_v      * v_meas);

    th_int += th * dt;
    th_int -= th_int * (dt / std::max(tau_i_leak_s, 1e-3));     // leak
    double a_i = -ki_theta * th_int;
    a_i = std::clamp(a_i, -a_i_abs_max, +a_i_abs_max);

    double a_cmd_raw      = a_theta + a_dtheta + a_v + a_i;

    // Falling severity (0..1)
    const double sev_th = std::clamp(std::abs(th) / th_fall_rad, 0.0, 1.0);
    const double sev_w  = std::clamp(std::abs(w)  / w_fall_rad_s, 0.0, 1.0);
    const double sev    = std::max(sev_th, sev_w);
    const bool   falling = (sev > 1e-6);

    // Soft accel cap; allow a bit more when falling
    const double a_cap = (falling ? 1.8 : 1.0) * a_max_mps2;
    a_cmd_raw = std::clamp(a_cmd_raw, -a_cap, +a_cap);

    // Jerk-shape the accel (first-order toward target)
    const double alpha_jerk = dt / std::max(tau_jerk_s, 1e-3);
    a_cmd_f += (a_cmd_raw - a_cmd_f) * alpha_jerk;

    // --- accel → Δu (steps/s) ---
    double du_req = (a_cmd_f / m_per_step) * dt;

    // Capture when far: command towards a velocity target
    if (falling) {
      const double v_cap = std::clamp(kcap_th * th + kcap_w * w, -v_capture_max, +v_capture_max);
      const double u_tgt = v_cap / m_per_step;     // [steps/s]
      du_req = (u_tgt - u_int);                    // jump toward target this tick; will be slewed
    }

    // --- Adaptive slew limit on Δu ---
    const double base_du = Config::max_du_per_sec;
    const double max_du_per_sec = base_du * (1.0 + sev * (du_boost_factor - 1.0));
    const double max_du = max_du_per_sec * dt;

    bool du_limited = false;
    double du = du_req;
    if (du >  max_du) { du =  max_du; du_limited = true; }
    if (du < -max_du) { du = -max_du; du_limited = true; }

    // Predict unclamped next value (for leak and conditional integration)
    const double u_unclamped = u_int + du;

    // Leak (bias drain) before amplitude clamp
    const double leak = (u_unclamped * dt) / std::max(Config::tau_u_s, 1e-6);
    double u_leaked = u_unclamped - leak;

    // Hard clamp to actuator limits
    const double u_clamped = std::clamp(u_leaked, -Config::max_sps, Config::max_sps);
    const bool   amp_limited = (u_clamped != u_leaked);

    // -------- anti-windup ----------
    // (1) Conditional integration: if pushing further into the clamp, drop this step
    const bool pushing_into_clamp =
      (amp_limited && ((u_leaked >  u_clamped && du > 0.0) ||
                       (u_leaked <  u_clamped && du < 0.0)));

    double u_next = pushing_into_clamp ? u_int : u_leaked;

    // (2) Back-calculation toward *both* clamps
    // toward slew-limited path (if any)
    if (du_limited) {
      const double kaw_slew = dt / std::max(tau_aw_slew_s, 1e-3);
      const double u_slew_path = u_int + du; // the value allowed by slew clamp
      u_next += kaw_slew * (u_slew_path - u_next);
    }
    // toward amplitude rail (if any)
    if (amp_limited) {
      const double kaw_amp = dt / std::max(tau_aw_s, 1e-3);
      u_next += kaw_amp * (u_clamped - u_next);
    }

    // finalize internal state
    u_int = u_next;

    // deadzone on the value we actually send
    const double dead = Config::deadzone_frac * Config::max_sps;
    const double u_val = std::clamp(u_clamped, -Config::max_sps, Config::max_sps);
    double u_out = (std::abs(u_val) < dead) ? 0.0 : u_val;

    // split (no steer here)
    const double left_cmd  = u_out;
    const double right_cmd = u_out;

    left_.setTarget(left_cmd);
    right_.setTarget(right_cmd);
    u_balance_sps_.store(u_out, std::memory_order_relaxed);

    // telemetry
    emitTelemetryNow(std::chrono::steady_clock::now(),
      th, w, v_meas,
      a_theta, a_dtheta, a_v, a_cmd_raw,
      du, du_limited, u_out, amp_limited,
      left_cmd, right_cmd);
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
