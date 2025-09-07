#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <thread>
#include <algorithm>

// PX4 tiny rate PID (submodule)
#include <rate_control.hpp>                  // PX4-Autopilot/src/lib/rate_control/rate_control.hpp
#include <matrix/matrix/math.hpp>            // PX4-Autopilot/src/lib/matrix/matrix/math.hpp


// Motor runner concept: must have setTarget(double steps_per_sec)

template <class MotorRunnerT>
class CascadedController {
public:
  CascadedController(MotorRunnerT& left, MotorRunnerT& right)
  : left_(left), right_(right)
  {
    using matrix::Vector3f;
    // Configure PX4 rate controller (pitch axis only) from global Config::
    rc_.setPidGains(
      /*P*/ Vector3f(0.f, Config::rate_P, 0.f),
      /*I*/ Vector3f(0.f, Config::rate_I, 0.f),
      /*D*/ Vector3f(0.f, Config::rate_D, 0.f));
    rc_.setIntegratorLimit(Vector3f(0.f, Config::rate_I_lim, 0.f));
    rc_.setFeedForwardGain(Vector3f(0.f, Config::rate_FF, 0.f));

    last_ts_ = std::chrono::steady_clock::now();
    worker_ = std::thread(&CascadedController::loop, this);
  }

  ~CascadedController() { stop(); }

  void stop() {
    bool exp = true;
    if (alive_.compare_exchange_strong(exp, false)) {
      if (worker_.joinable()) worker_.join();
    }
  }

  // Push latest prefiltered IMU sample (angle, gyro)
  void pushImu(const ImuSample& s) { latest_filtered_.store(s, std::memory_order_relaxed); }

  void setJoystick(const JoyCmd& j) { joy_.store(j, std::memory_order_relaxed); }

  void setTelemetrySink(std::function<void(const Telemetry&)> cb) { tel_cb_ = std::move(cb); }

private:
  // ===== Atomic wrappers =====
  struct AtomicImu {
    std::atomic<double> angle{0.0}, gyro{0.0};
    std::atomic<std::int64_t> t_count{0};
    void store(const ImuSample& s, std::memory_order mo) {
      angle.store(s.angle_rad, mo); gyro.store(s.gyro_rad_s, mo);
      const auto d = s.t.time_since_epoch(); t_count.store((std::int64_t)d.count(), mo);
    }
    ImuSample load(std::memory_order mo) const {
      ImuSample s{}; s.angle_rad=angle.load(mo); s.gyro_rad_s=gyro.load(mo);
      using clk=std::chrono::steady_clock; auto cnt=t_count.load(mo);
      s.t = std::chrono::time_point<clk>(clk::duration((clk::duration::rep)cnt)); return s;
    }
  };

  struct AtomicJoy { std::atomic<float> f{0}, t{0}; JoyCmd load(std::memory_order mo) const { return JoyCmd{f.load(mo), t.load(mo)}; } void store(const JoyCmd& j, std::memory_order mo){f.store(j.forward,mo); t.store(j.turn,mo);} };

// ===== Core loop =====
void loop() {
  using namespace std::chrono;
  using matrix::Vector3f;

  const double m_per_step =
      Config::wheel_radius_m * (2.0 * M_PI / Config::steps_per_rev);
  (void)m_per_step; // currently unused; we command in steps/s directly

  auto next = steady_clock::now();
  constexpr int kMinHz = 50;
  const auto dt_nom =
      duration<double>(1.0 / std::max(kMinHz, Config::control_hz));

  while (alive_.load(std::memory_order_relaxed)) {
    next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(dt_nom);
    std::this_thread::sleep_until(next);

    // --- Build (pitch, gyro, dt) ---
    float pitch_rad = 0.f;
    float gyro_rad_s = 0.f;
    float dt = 0.f;
    const auto now = steady_clock::now();

    {
      ImuSample s = latest_filtered_.load(std::memory_order_relaxed);
      pitch_rad = (float)s.angle_rad;
      gyro_rad_s = (float)s.gyro_rad_s;
      dt = std::clamp((float)duration<double>(now - last_ts_).count(),
                      1.f / 2000.f, 0.05f);
      last_ts_ = now;
    }

    // --- Outer: attitude -> pitch-rate setpoint (rad/s) ---
    const float pitch_deg = pitch_rad * 180.f / (float)M_PI;
    const float gyro_dps  = gyro_rad_s * 180.f / (float)M_PI;

    // small lead to counter sensor/compute delay
    const float pitch_pred_deg = pitch_deg + (float)Config::lead_T_s * gyro_dps;
    const float e_deg = -pitch_pred_deg; // target 0 deg

    // Freeze outer I when far from upright or when output is near clamp
    const bool i_far   = std::abs(pitch_deg) > (float)Config::att_i_freeze_deg;
    const bool i_limit = std::abs(u_int_sps_) > 0.9f * (float)Config::max_sps;

    if (!i_far && !i_limit) {
      att_int_ = std::clamp(
          att_int_ + e_deg * dt * (float)Config::att_ki_deg_s_per_deg_s,
          -(float)Config::att_int_lim_deg_s,
          +(float)Config::att_int_lim_deg_s);
    }

    // Linear PI output in deg/s
    const float rate_sp_dps_lin =
        (float)Config::att_kp_deg_s_per_deg * e_deg + att_int_;

    // Soft cap: agile near 0, smoothly limits at high error
    const float sp_max = (float)Config::rate_sp_max_dps; // e.g. 120
    const float rate_sp_dps = sp_max * std::tanh(rate_sp_dps_lin / sp_max);

    // Convert to rad/s for PX4 rate PID
    const float rate_sp_rad_s = rate_sp_dps * (float)M_PI / 180.f;

    // --- Inner: PX4 rate PID with D-term accel estimate (LPF) ---
    const Vector3f rate{0.f, gyro_rad_s, 0.f};
    const Vector3f rate_sp{0.f, rate_sp_rad_s, 0.f};

    // angular acceleration (D input) with 1st-order LPF
    static float prev_gyro = 0.f; // rad/s
    const float acc_y = (dt > 0.f) ? (gyro_rad_s - prev_gyro) / dt : 0.f;
    prev_gyro = gyro_rad_s;

    static float acc_y_f = 0.f;
    const float fc   = (float)Config::acc_lpf_hz;            // e.g. 70 Hz
    const float tau  = 1.f / (2.f * (float)M_PI * fc);
    const float alpha = dt / (dt + tau);
    acc_y_f += alpha * (acc_y - acc_y_f);

    const Vector3f ang_accel{0.f, acc_y_f, 0.f};
    const Vector3f u = rc_.update(rate, rate_sp, ang_accel, dt, /*landed=*/false);

    // map to steps/s and apply slew + clamp
    float u_cmd = u(1) * (float)Config::pitch_out_to_sps;

    // Anti-windup hint to PX4 (pitch axis saturation)
    const bool pos_sat = (u_int_sps_ >= (float)Config::max_sps - 1e-3f) && (u_cmd > u_int_sps_);
    const bool neg_sat = (u_int_sps_ <=-(float)Config::max_sps - 1e-3f) && (u_cmd < u_int_sps_);
    rc_.setPositiveSaturationFlag(1, pos_sat);
    rc_.setNegativeSaturationFlag(1, neg_sat);

    const float max_du = (float)Config::max_du_per_sec * dt;
    u_int_sps_ += std::clamp(u_cmd - u_int_sps_, -max_du, +max_du);
    u_int_sps_  = std::clamp(u_int_sps_, -(float)Config::max_sps, +(float)Config::max_sps);

    const float dead = (float)(Config::deadzone_frac * Config::max_sps);
    if (std::abs(u_int_sps_) < dead) u_int_sps_ = 0.f;

    // Drive motors (no steer here)
    left_.setTarget(u_int_sps_);
    right_.setTarget(u_int_sps_);

    // Telemetry
    if (tel_cb_) {
      rate_ctrl_status_s st{}; rc_.getRateControlStatus(st);
      Telemetry t{};
      t.t_sec          = std::chrono::duration<double>(now.time_since_epoch()).count();
      t.pitch_deg      = pitch_deg;
      t.pitch_rate_dps = gyro_dps;
      t.rate_sp_dps    = rate_sp_dps;
      t.out_norm       = (double)u(1);
      t.u_sps          = (double)u_int_sps_;
      t.integ_pitch    = (double)st.pitchspeed_integ;
      tel_cb_(t);
    }
  }
}


  // ===== Members =====
  MotorRunnerT& left_;
  MotorRunnerT& right_;

  std::thread worker_{};
  std::atomic<bool> alive_{true};

  AtomicImu latest_filtered_{};
  AtomicJoy joy_{};

  RateControl rc_{}; // PX4 rate PID

  std::function<void(const Telemetry&)> tel_cb_{};
  std::chrono::steady_clock::time_point last_ts_{};

  float u_int_sps_{0.f};
  float att_int_{0.f};
};
