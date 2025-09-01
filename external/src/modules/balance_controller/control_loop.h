#pragma once
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <thread>
#include <algorithm>
#include <functional>

#include "config.h"


// ---- Simple speed estimator (fallback until encoders exist) ----
// Smooths commanded sps into an "actual" estimate using a 1st-order LPF.
struct SpeedEstimator {
  void reset() {
    v_left_sps.store(0.0, std::memory_order_relaxed);
    v_right_sps.store(0.0, std::memory_order_relaxed);
  }

  void updateTargets(double left_cmd_sps, double right_cmd_sps, double dt_s) {
    const double alpha = std::clamp(dt_s * 8.0, 0.0, 1.0); // ~8 Hz cutoff
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

// ---- The cascaded controller (owns its own worker thread) ----
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

  // Push latest IMU sample (call from your IMU ISR/reader at ~1 kHz)
  void pushImu(const ImuSample& s) {
    latest_imu_.store(s, std::memory_order_relaxed);
  }

  // Provide joystick: forward, turn in [-1, 1]
  void setJoystick(const JoyCmd& j) {
    joy_.store(j, std::memory_order_relaxed);
  }

  void setTelemetrySink(std::function<void(const Telemetry&)> cb) {
    tel_cb_ = std::move(cb);
  }

  // For telemetry/debug
  double lastTiltTargetRad() const { return tilt_target_rad_.load(std::memory_order_relaxed); }
  double lastUBalanceSps() const { return u_balance_sps_.load(std::memory_order_relaxed); }

private:
  // Minimal lock-free “latest value” wrapper
    struct AtomicImu {
    std::atomic<double> angle{0.0}; // pitch
    std::atomic<double> gyro{0.0};  // pitch rate
    std::atomic<double> yaw_z{0.0}; // NEW: yaw rate
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

  // Main loop: interleaves fast and slow controllers
void loop() {
  using dur = std::chrono::steady_clock::duration;

  const auto dt_out = std::chrono::duration_cast<dur>(
      std::chrono::duration<double>(1.0 / std::max(20, Config::hz_outer)));
  const auto dt_bal = std::chrono::duration_cast<dur>(
      std::chrono::duration<double>(1.0 / std::max(50, Config::hz_balance)));

  // Gating & shaping knobs (tune if needed)
  constexpr double kTiltGateDeg   = 3.0;    // only run vel PI when |pitch| < 3°
  constexpr double kRateGateDegS  = 30.0;   // and |gyro| < 30°/s
  constexpr float  kStickDZ       = 0.02f;  // joystick deadzone

  auto t_next_out = std::chrono::steady_clock::now();
  auto t_next_bal = t_next_out;

  double vel_int = 0.0;
  SpeedEstimator v_est; v_est.reset();

  double cmd_left_sps  = 0.0;
  double cmd_right_sps = 0.0;

  // balance command slew state
  double u_prev = 0.0;

  while (alive_.load(std::memory_order_relaxed) && !g_stop.load(std::memory_order_relaxed)) {
    const auto now = std::chrono::steady_clock::now();

    // --- Outer (velocity PI + steering) ---
    if (now >= t_next_out) {
      t_next_out += dt_out;
      const double dt_sec = std::chrono::duration<double>(dt_out).count();

      JoyCmd j = joy_.load(std::memory_order_relaxed);
      if (std::fabs(j.forward) < kStickDZ) j.forward = 0.0f;
      if (std::fabs(j.turn)    < kStickDZ) j.turn    = 0.0f;

      const double actual_base_sps = 0.5 * (v_est.leftSps() + v_est.rightSps());
      const double desired_base_sps = static_cast<double>(j.forward) * Config::max_sps;

      // Gate velocity PI unless upright & calm (prevents target tilt dancing during recovery)
      const ImuSample s_now = latest_imu_.load(std::memory_order_relaxed);
      const double tilt_deg = s_now.angle_rad * 180.0 / M_PI;
      const double gyro_deg = s_now.gyro_rad_s * 180.0 / M_PI;
      const double u_abs    = std::abs(u_balance_sps_.load(std::memory_order_relaxed));

      const bool ok_for_vel =
          std::abs(tilt_deg) < kTiltGateDeg &&
          std::abs(gyro_deg) < kRateGateDegS &&
          u_abs < 0.8 * Config::max_sps; // also don’t integrate when we’re railing

      // Estimate "actual" base speed from commanded (LPF estimator)
      if (ok_for_vel && std::fabs(j.forward) > 0.0f) {
        v_est.updateTargets(cmd_left_sps, cmd_right_sps, dt_sec);
      } else {
        // decay toward 0 so vel_est doesn’t fight around upright
        const double decay = std::exp(-dt_sec * 4.0); // ~4 Hz decay
        // TODO ADD DECAY
        v_est.updateTargets(cmd_left_sps * 0, cmd_right_sps * 0, dt_sec);
      }

      double tilt_target = 0.0;
      bool saturated = false;

      if (ok_for_vel && std::fabs(j.forward) > 0.0f) {
        const double vel_err = desired_base_sps - actual_base_sps;
        vel_int += vel_err * dt_sec;

        const double sps_max_d = static_cast<double>(Config::max_sps);
        const double tilt_from_vel =
          Config::kp_vel * vel_err * (1.0 / std::max(1.0, sps_max_d)) * Config::max_tilt_rad +
          Config::ki_vel * vel_int * (1.0 / std::max(1.0, sps_max_d)) * Config::max_tilt_rad;

        tilt_target = tilt_from_vel;
        if (tilt_target > Config::max_tilt_rad) { tilt_target = Config::max_tilt_rad; saturated = true; }
        else if (tilt_target < -Config::max_tilt_rad) { tilt_target = -Config::max_tilt_rad; saturated = true; }

        // back-calc anti-windup on tilt clamp
        if (saturated) {
          const bool pushing_out =
            (tilt_from_vel > Config::max_tilt_rad && vel_err > 0.0) ||
            (tilt_from_vel < -Config::max_tilt_rad && vel_err < 0.0);
          if (pushing_out) vel_int -= vel_err * dt_sec;
        }

        // Telemetry
        tel_desired_base_sps_ = desired_base_sps;
        tel_actual_base_sps_  = actual_base_sps;
        tel_vel_err_sps_      = vel_err;
        tel_vel_int_state_    = vel_int;
      } else {
        // Freeze the outer loop when not upright or no forward command
        tilt_target = 0.0;
        vel_int *= 0.98; // gentle decay so it returns toward zero naturally
        tel_desired_base_sps_ = desired_base_sps;
        tel_actual_base_sps_  = actual_base_sps;
        tel_vel_err_sps_      = desired_base_sps - actual_base_sps;
        tel_vel_int_state_    = vel_int;
      }

      tel_tilt_saturated_ = saturated;
      tilt_target_rad_.store(tilt_target, std::memory_order_relaxed);

      // --- Yaw control (unchanged; off unless Config::yaw_pi_enabled) ---
      const double actual_yaw = s_now.yaw_rate_z;
      if (Config::yaw_pi_enabled) {
        const double desired_yaw =
          std::clamp(static_cast<double>(j.turn), -1.0, 1.0) * Config::max_yaw_rate_cmd;
        const double yaw_err = desired_yaw - actual_yaw;
        yaw_int_ += yaw_err * dt_sec;
        double steer = Config::kp_yaw * yaw_err + Config::ki_yaw * yaw_int_;
        const double pre = steer;
        steer = std::clamp(steer, -Config::max_steer_sps, Config::max_steer_sps);
        if (steer != pre) {
          const bool pushing_out = (pre > steer && yaw_err > 0.0) ||
                                   (pre < steer && yaw_err < 0.0);
          if (pushing_out) yaw_int_ -= yaw_err * dt_sec;
        }
        steer_split_sps_ = steer;
        tel_desired_yaw_rate_ = desired_yaw;
        tel_actual_yaw_rate_  = actual_yaw;
        tel_yaw_err_          = yaw_err;
        tel_yaw_int_          = yaw_int_;
      } else {
        steer_split_sps_ = std::clamp(static_cast<double>(j.turn) * Config::k_turn,
                                      -Config::max_steer_sps, Config::max_steer_sps);
        tel_desired_yaw_rate_ = 0.0;
        tel_actual_yaw_rate_  = actual_yaw;
        tel_yaw_err_          = 0.0;
        tel_yaw_int_          = 0.0;
      }
    }

    // --- Balance PD ---
    if (now >= t_next_bal) {
      t_next_bal += dt_bal;

      const ImuSample s = latest_imu_.load(std::memory_order_relaxed);
      double tilt = s.angle_rad;
      const double gyro_deg_s_raw = s.gyro_rad_s * 180.0 / M_PI;
      const double gyro_deg_s     = std::clamp(gyro_deg_s_raw, -Config::gyro_d_abs_limit_deg_s, Config::gyro_d_abs_limit_deg_s);
      const double gyro           = gyro_deg_s * M_PI / 180.0;
      const double tilt_target = tilt_target_rad_.load(std::memory_order_relaxed);

      // Error with a small deadband to avoid chatter
      double err = tilt_target - tilt;

      // Raw PD
      const double u_raw = Config::kp_bal * err + Config::kd_bal * (-gyro);

      // Slew limit (per balance tick) to avoid ping-pong
      const double u_min = u_prev - Config::slew_per_sec;
      const double u_max = u_prev + Config::slew_per_sec;
      const double u_balance = std::clamp(u_raw, u_min, u_max);
      u_prev = u_balance;
      u_balance_sps_.store(u_balance, std::memory_order_relaxed);

      // Split & clamp
      double left  = u_balance + steer_split_sps_;
      double right = u_balance - steer_split_sps_;
      left  = std::clamp(left,  -Config::max_sps, Config::max_sps);
      right = std::clamp(right, -Config::max_sps, Config::max_sps);

      cmd_left_sps  = left;
      cmd_right_sps = right;

      left_.setTarget(cmd_left_sps);
      right_.setTarget(cmd_right_sps);

      // Telemetry at balance rate
      emitTelemetryNow(now, tilt, gyro, tilt_target, u_balance,
                       steer_split_sps_, left, right);
    }

    const auto t_min = (t_next_bal < t_next_out) ? t_next_bal : t_next_out;
    const auto slack = t_min - std::chrono::steady_clock::now();
    if (slack > std::chrono::microseconds(50)) {
      std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
  }
}

private:

  void emitTelemetryNow(std::chrono::steady_clock::time_point now,
                        double tilt, double gyro,
                        double tilt_target,
                        double u_balance,
                        double steer_split,
                        double left_cmd, double right_cmd)
  {
    if (!tel_cb_) return;
    Telemetry t;
    t.ts               = now;
    t.tilt_rad         = tilt;
    t.gyro_rad_s       = gyro;
    t.tilt_target_rad  = tilt_target;
    t.tilt_saturated   = tel_tilt_saturated_;
    t.desired_base_sps = tel_desired_base_sps_;
    t.actual_base_sps  = tel_actual_base_sps_;
    t.vel_err_sps      = tel_vel_err_sps_;
    t.vel_int_state    = tel_vel_int_state_;

    // yaw telemetry
    t.desired_yaw_rate = tel_desired_yaw_rate_;
    t.actual_yaw_rate  = tel_actual_yaw_rate_;
    t.yaw_err          = tel_yaw_err_;
    t.yaw_int_state    = tel_yaw_int_;

    t.u_balance_sps    = u_balance;
    t.steer_split_sps  = steer_split;
    t.left_cmd_sps     = left_cmd;
    t.right_cmd_sps    = right_cmd;
    tel_cb_(t);
    }

  MotorRunnerT& left_;
  MotorRunnerT& right_;

  std::atomic<bool> alive_{true};
  std::thread worker_;

  // State shared between loops
  AtomicImu latest_imu_;
  AtomicJoy joy_;
  std::function<void(const Telemetry&)> tel_cb_{};
  std::atomic<double> tilt_target_rad_{0.0};
  std::atomic<double> u_balance_sps_{0.0};
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
