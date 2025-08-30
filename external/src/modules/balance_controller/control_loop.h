#pragma once
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <thread>
#include <algorithm>
#include <functional>

#include "config.h"


// ---- Joystick command (forward/turn normalized to [-1, 1]) ----
struct JoyCmd {
  float forward;    // + forward speed command
  float turn;       // + left faster, right slower (CCW yaw)
};

// ---- Controller tunables & limits ----
struct ControlTunings {
  // Balance PD
  double kp_bal = 60.0;     // [sps/rad]
  double kd_bal = 1.2;      // [sps/(rad/s)]

  // Velocity PI
  double kp_vel = 1.2;
  double ki_vel = 0.4;

  // Open-loop steering gain (used if yaw PI disabled)
  double k_turn = 600.0;    // [sps / unit turn]

  // Yaw PI (closed-loop steering) -- ENABLE THIS to use gyro Z
  bool   yaw_pi_enabled = false;
  double kp_yaw = 250.0;         // [sps / (rad/s)]
  double ki_yaw = 80.0;          // [sps / (rad/s * s)]
  double max_yaw_rate_cmd = 2.0; // [rad/s] from joystick at |turn|=1
  double max_steer_sps    = 800.0; // clamp steering split

  // Saturations
  double max_tilt_rad = 6.0 * (M_PI / 180.0);
  double max_sps      = static_cast<double>(Config::max_sps);

  // Loop rates
  int hz_balance = 400;
  int hz_outer   = 100;
};

struct Telemetry {
  // timing
  std::chrono::steady_clock::time_point ts{};

  // IMU & references
  double tilt_rad = 0.0;
  double gyro_rad_s = 0.0;
  double tilt_target_rad = 0.0;
  bool   tilt_saturated = false;

  // Velocity loop
  double desired_base_sps = 0.0;
  double actual_base_sps  = 0.0;
  double vel_err_sps      = 0.0;
  double vel_int_state    = 0.0;

  // NEW: Yaw loop
  double desired_yaw_rate = 0.0; // [rad/s]
  double actual_yaw_rate  = 0.0; // [rad/s]
  double yaw_err          = 0.0; // [rad/s]
  double yaw_int_state    = 0.0; // integrator state

  // Balance & steering
  double u_balance_sps   = 0.0;
  double steer_split_sps = 0.0;

  // Motor commands
  double left_cmd_sps  = 0.0;
  double right_cmd_sps = 0.0;
};

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
  CascadedController(MotorRunnerT& left, MotorRunnerT& right, const ControlTunings& g = {})
    : left_(left), right_(right), g_(g) {
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
  using dur   = std::chrono::steady_clock::duration;

  const auto dt_out = std::chrono::duration_cast<dur>(
      std::chrono::duration<double>(1.0 / std::max(20, g_.hz_outer)));
  const auto dt_bal = std::chrono::duration_cast<dur>(
      std::chrono::duration<double>(1.0 / std::max(50, g_.hz_balance)));

  auto t_next_out = std::chrono::steady_clock::now();
  auto t_next_bal = t_next_out;

  double vel_int = 0.0;
  SpeedEstimator v_est; v_est.reset();

  double cmd_left_sps = 0.0;
  double cmd_right_sps = 0.0;

  while (alive_.load(std::memory_order_relaxed) && !g_stop.load(std::memory_order_relaxed)) {
    const auto now = std::chrono::steady_clock::now();

    // --- Outer (velocity PI + steering) ---
    if (now >= t_next_out) {
    t_next_out += dt_out;
    const double dt_sec = std::chrono::duration<double>(dt_out).count();

    JoyCmd j = joy_.load(std::memory_order_relaxed);
    if (std::fabs(j.forward) < 0.02f) j.forward = 0.0f;
    if (std::fabs(j.turn)    < 0.02f) j.turn    = 0.0f;

      const double desired_base_sps = static_cast<double>(j.forward) * g_.max_sps;

      v_est.updateTargets(cmd_left_sps, cmd_right_sps, dt_sec);
      const double actual_base_sps = 0.5 * (v_est.leftSps() + v_est.rightSps());

      const double vel_err = desired_base_sps - actual_base_sps;
      vel_int += vel_err * dt_sec;

      const double tilt_from_vel =
        g_.kp_vel * vel_err * (1.0 / std::max(1.0, g_.max_sps)) * g_.max_tilt_rad +
        g_.ki_vel * vel_int * (1.0 / std::max(1.0, g_.max_sps)) * g_.max_tilt_rad;

      bool saturated = false;
      double tilt_target = tilt_from_vel;
      if (tilt_target > g_.max_tilt_rad) { tilt_target = g_.max_tilt_rad; saturated = true; }
      else if (tilt_target < -g_.max_tilt_rad) { tilt_target = -g_.max_tilt_rad; saturated = true; }

      if (saturated) {
        const bool pushing_out =
          (tilt_from_vel > g_.max_tilt_rad && vel_err > 0.0) ||
          (tilt_from_vel < -g_.max_tilt_rad && vel_err < 0.0);
        if (pushing_out) vel_int -= vel_err * dt_sec;
      }
    tilt_target_rad_.store(tilt_target, std::memory_order_relaxed);
    // Save for telemetry
    tel_desired_base_sps_ = desired_base_sps;
    tel_actual_base_sps_  = actual_base_sps;
    tel_vel_err_sps_      = vel_err;
    tel_vel_int_state_    = vel_int;
    tel_tilt_saturated_   = saturated;

    // --- Yaw control ---
    const ImuSample s_now = latest_imu_.load(std::memory_order_relaxed);
    const double actual_yaw = s_now.yaw_rate_z;

    if (g_.yaw_pi_enabled) {
        // Desired yaw rate from joystick in rad/s (positive = CCW)
        const double desired_yaw = std::clamp(static_cast<double>(j.turn),
                                            -1.0, 1.0) * g_.max_yaw_rate_cmd;
        const double yaw_err = desired_yaw - actual_yaw;

        // Integrate (anti-windup via output clamp below)
        yaw_int_ += yaw_err * dt_sec;

        // Raw PI steering split (in sps)
        double steer = g_.kp_yaw * yaw_err + g_.ki_yaw * yaw_int_;

        // Clamp steering and back-calc anti-windup if pushing further out
        const double pre = steer;
        steer = std::clamp(steer, -g_.max_steer_sps, g_.max_steer_sps);
        if (steer != pre) {
        const bool pushing_out = (pre >  steer && yaw_err > 0.0) ||
                                (pre <  steer && yaw_err < 0.0); // sign-consistent
        if (pushing_out) yaw_int_ -= yaw_err * dt_sec;
        }

        steer_split_sps_ = steer;

        // Telemetry shadows
        tel_desired_yaw_rate_ = desired_yaw;
        tel_actual_yaw_rate_  = actual_yaw;
        tel_yaw_err_          = yaw_err;
        tel_yaw_int_          = yaw_int_;
    } else {
        // Open-loop fallback: same as before
        steer_split_sps_ = std::clamp(static_cast<double>(j.turn) * g_.k_turn,
                                    -g_.max_steer_sps, g_.max_steer_sps);
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
      const double tilt = s.angle_rad;
      const double gyro = s.gyro_rad_s;
      const double tilt_target = tilt_target_rad_.load(std::memory_order_relaxed);
      const double err = tilt_target - tilt;

      const double u_balance = g_.kp_bal * err + g_.kd_bal * (-gyro);
      u_balance_sps_.store(u_balance, std::memory_order_relaxed);

      double left = u_balance + steer_split_sps_;
      double right = u_balance - steer_split_sps_;
      left  = std::clamp(left,  -g_.max_sps, g_.max_sps);
      right = std::clamp(right, -g_.max_sps, g_.max_sps);

      cmd_left_sps = left;
      cmd_right_sps = right;

      left_.setTarget(cmd_left_sps);
      right_.setTarget(cmd_right_sps);

      // emit telemetry at balance rate (good for plots)
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
  ControlTunings g_;

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
