#pragma once

#include <atomic>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <thread>
#include <algorithm>

// PX4 tiny rate PID (you already have PX4 as a submodule)
#include <rate_control.hpp>                  // PX4-Autopilot/src/lib/rate_control/rate_control.hpp
#include <matrix/matrix/math.hpp>            // PX4-Autopilot/src/lib/matrix/matrix/math.hpp


// Motor runner concept: must have setTarget(double steps_per_sec)

template <class MotorRunnerT>
class CascadedController {
public:
  struct Config {
    // Geometry / actuators
    int    microstep_mult   = 16;
    double steps_per_rev    = (360.0/1.8) * microstep_mult; // includes microsteps
    double wheel_radius_m   = 0.04; // m
    double max_sps          = 6000.0; // actuator rail [steps/s]
    double max_du_per_sec   = 120000.0; // slew limit [steps/s^2]

    // Outer attitude (deg -> deg/s)
    float  att_kp_deg_s_per_deg   = 6.0f; // start 4–8
    float  att_ki_deg_s_per_deg_s = 0.0f; // start 0; add only to remove steady bias
    float  att_int_lim_deg_s      = 60.0f;
    float  lead_T_s               = 0.015f; // simple lead on measurement

    // Inner PX4 rate PID (pitch axis gains; roll/yaw remain zero)
    float  rate_P                 = 0.12f;
    float  rate_I                 = 0.08f;
    float  rate_D                 = 0.000f; // start 0; add tiny 0.001–0.003 if needed
    float  rate_I_lim             = 0.30f;  // integral clamp (rad/s units)
    float  rate_FF                = 0.00f;  // feed-forward on rate setpoint

    // Map PX4 rate output (normalized torque proxy) -> wheel speed [steps/s]
    float  pitch_out_to_sps       = 2500.0f; // tune so |out_norm|~1 -> comfortable speed (not rail)

    // Deadzone on wheel command to avoid buzz
    double deadzone_frac          = 0.005;   // 0.5% of max_sps

    // Input mode
    bool   use_external_filter    = true;    // true: use ImuSample angle/rate provided by app
                                             // false: do internal simple complementary filter from raw acc+gyro

    // Internal complementary filter (used only if use_external_filter=false)
    double cf_alpha_gyro          = 0.98;    // 0..1, higher = trust gyro more
    double acc_lpf_hz             = 30.0;    // simple 1st-order LPF on accel

    // Safety
    double dt_min_s               = 1.0/2000.0; // ignore crazy small dt
    double dt_max_s               = 0.05;       // skip if dt too large
  };

  CascadedController(MotorRunnerT& left, MotorRunnerT& right, const Config& cfg)
  : left_(left), right_(right), cfg_(cfg)
  {
    using matrix::Vector3f;
    // Configure PX4 rate controller (pitch axis only)
    rc_.setPidGains(
      /*P*/ Vector3f(0.f, cfg_.rate_P, 0.f),
      /*I*/ Vector3f(0.f, cfg_.rate_I, 0.f),
      /*D*/ Vector3f(0.f, cfg_.rate_D, 0.f));
    rc_.setIntegratorLimit(Vector3f(0.f, cfg_.rate_I_lim, 0.f));
    rc_.setFeedForwardGain(Vector3f(0.f, cfg_.rate_FF, 0.f));

    last_ts_ = std::chrono::steady_clock::now();

    // Start worker thread
    worker_ = std::thread(&CascadedController::loop, this);
  }

  // Convenience ctor with default Config{}
  CascadedController(MotorRunnerT& left, MotorRunnerT& right)
  : CascadedController(left, right, Config{}) {}

  ~CascadedController() { stop(); }

  void stop() {
    bool exp = true;
    if (alive_.compare_exchange_strong(exp, false)) {
      if (worker_.joinable()) worker_.join();
    }
  }

  // Push latest prefiltered IMU sample (angle, gyro). Only used if use_external_filter=true
  void pushImu(const ImuSample& s) { latest_filtered_.store(s, std::memory_order_relaxed); }

  // Push raw IMU (acc[g], gyro[rad/s]). Used if use_external_filter=false
  void pushRawImu(const std::array<double,3>& acc_g, const std::array<double,3>& gyro_rad_s,
                  std::chrono::steady_clock::time_point ts)
  {
    Raw raw; raw.acc=acc_g; raw.gyro=gyro_rad_s; raw.ts=ts; latest_raw_.store(raw, std::memory_order_relaxed);
  }

  void setJoystick(const JoyCmd& j) { joy_.store(j, std::memory_order_relaxed); }

  void setTelemetrySink(std::function<void(const Telemetry&)> cb) { tel_cb_ = std::move(cb); }

  // Helper to bind directly to your on_sample callback (choose external/internal path at compile-time)
  auto make_on_sample() {
    // Usage: icfg.on_sample = make_on_sample();
    return [this](double /*pitch_unused*/, std::array<double,3> acc,
                  std::array<double,3> gyrv,
                  std::chrono::steady_clock::time_point ts)
    {
      if (cfg_.use_external_filter) {
        // If you already filter elsewhere, keep your path and call pushImu() separately.
        // This lambda only feeds raw in case you want internal CF:
        (void)acc; (void)gyrv; (void)ts; // no-op
      } else {
        pushRawImu(acc, gyrv, ts); // internal complementary filter path
      }
    };
  }

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

  struct Raw { std::array<double,3> acc{}; std::array<double,3> gyro{}; std::chrono::steady_clock::time_point ts{}; };
  struct AtomicRaw {
    std::array<std::atomic<double>,3> acc{{0,0,0}}; std::array<std::atomic<double>,3> gyro{{0,0,0}};
    std::atomic<std::int64_t> t_count{0};
    void store(const Raw& r, std::memory_order mo){ for(int i=0;i<3;++i){acc[i].store(r.acc[i],mo);gyro[i].store(r.gyro[i],mo);} const auto d=r.ts.time_since_epoch(); t_count.store((std::int64_t)d.count(),mo);}    
    Raw load(std::memory_order mo) const { Raw r{}; for(int i=0;i<3;++i){r.acc[i]=acc[i].load(mo); r.gyro[i]=gyro[i].load(mo);} using clk=std::chrono::steady_clock; auto cnt=t_count.load(mo); r.ts=std::chrono::time_point<clk>(clk::duration((clk::duration::rep)cnt)); return r; }
  };

  struct AtomicJoy { std::atomic<float> f{0}, t{0}; JoyCmd load(std::memory_order mo) const { return JoyCmd{f.load(mo), t.load(mo)}; } void store(const JoyCmd& j, std::memory_order mo){f.store(j.forward,mo); t.store(j.turn,mo);} };

  // ===== Internal CF state (only if use_external_filter=false) =====
  double cf_pitch_rad_ = 0.0;
  double acc_lpf_x_=0, acc_lpf_y_=0, acc_lpf_z_=0; // simple 1st-order LPF state
  bool   acc_lpf_init_ = false;

  // ===== Core loop =====
  void loop() {
    using namespace std::chrono;
    using matrix::Vector3f;

    const double m_per_step = cfg_.wheel_radius_m * (2.0*M_PI / cfg_.steps_per_rev);
    (void)m_per_step; // currently unused; we command in steps/s directly

    auto next = steady_clock::now();
    const auto dt_nom = duration<double>(1.0/400.0);

    while (alive_.load(std::memory_order_relaxed)) {
      next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(dt_nom); std::this_thread::sleep_until(next);

      // --- Build (pitch, gyro, dt) ---
      float pitch_rad = 0.f;
      float gyro_rad_s = 0.f;
      float dt = 0.f;
      auto now = steady_clock::now();

      if (cfg_.use_external_filter) {
        ImuSample s = latest_filtered_.load(std::memory_order_relaxed);
        pitch_rad = (float)s.angle_rad; gyro_rad_s = (float)s.gyro_rad_s;
        dt = std::clamp((float)duration<double>(now - last_ts_).count(), (float)cfg_.dt_min_s, (float)cfg_.dt_max_s);
        last_ts_ = now;
      } else {
        Raw r = latest_raw_.load(std::memory_order_relaxed);
        dt = std::clamp((float)duration<double>(r.ts - last_ts_).count(), (float)cfg_.dt_min_s, (float)cfg_.dt_max_s);
        last_ts_ = r.ts;
        // Low-pass accel
        const float alpha_acc = (float)(2*M_PI*cfg_.acc_lpf_hz * dt);
        const float a = alpha_acc / (1.f + alpha_acc); // simple Tustin-ish first-order
        if (!acc_lpf_init_) { acc_lpf_x_=r.acc[0]; acc_lpf_y_=r.acc[1]; acc_lpf_z_=r.acc[2]; acc_lpf_init_=true; }
        acc_lpf_x_ += (r.acc[0]-acc_lpf_x_)*a; acc_lpf_y_ += (r.acc[1]-acc_lpf_y_)*a; acc_lpf_z_ += (r.acc[2]-acc_lpf_z_)*a;
        // Tilt from accel (assuming pitch about +X forward, +Z up)
        const double pitch_acc = std::atan2(-acc_lpf_x_, acc_lpf_z_); // choose convention
        // Complementary filter
        cf_pitch_rad_ = cfg_.cf_alpha_gyro * (cf_pitch_rad_ + r.gyro[0]*dt) + (1.0 - cfg_.cf_alpha_gyro) * pitch_acc;
        pitch_rad = (float)cf_pitch_rad_;
        gyro_rad_s = (float)r.gyro[0];
      }

      // --- Outer: attitude -> pitch-rate setpoint (rad/s) ---
      const float pitch_deg = pitch_rad * 180.f / (float)M_PI;
      const float gyro_dps  = gyro_rad_s * 180.f / (float)M_PI;
      const float pitch_pred_deg = pitch_deg + cfg_.lead_T_s * gyro_dps;
      const float e_deg = -pitch_pred_deg; // target 0 deg
      att_int_ = std::clamp(att_int_ + e_deg * dt * cfg_.att_ki_deg_s_per_deg_s,
                            -cfg_.att_int_lim_deg_s, +cfg_.att_int_lim_deg_s);
      const float rate_sp_dps   = cfg_.att_kp_deg_s_per_deg * e_deg + att_int_;
      const float rate_sp_rad_s = rate_sp_dps * (float)M_PI / 180.f;

      // --- Inner: PX4 rate PID (no D-term accel input for now) ---
      const Vector3f rate{0.f, gyro_rad_s, 0.f};
      const Vector3f rate_sp{0.f, rate_sp_rad_s, 0.f};
      const Vector3f ang_accel{0.f, 0.f, 0.f};
      const Vector3f u = rc_.update(rate, rate_sp, ang_accel, dt, /*landed=*/false);

      // map to steps/s and apply slew + clamp
      float u_cmd = u(1) * cfg_.pitch_out_to_sps;

      // Anti-windup hint to PX4 (pitch axis saturation)
      const bool pos_sat = (u_int_sps_ >= (float)cfg_.max_sps - 1e-3f) && (u_cmd > u_int_sps_);
      const bool neg_sat = (u_int_sps_ <=-(float)cfg_.max_sps + 1e-3f) && (u_cmd < u_int_sps_);
      rc_.setPositiveSaturationFlag(1, pos_sat);
      rc_.setNegativeSaturationFlag(1, neg_sat);

      const float max_du = (float)cfg_.max_du_per_sec * dt;
      u_int_sps_ += std::clamp(u_cmd - u_int_sps_, -max_du, +max_du);
      u_int_sps_  = std::clamp(u_int_sps_, -(float)cfg_.max_sps, +(float)cfg_.max_sps);

      const float dead = (float)(cfg_.deadzone_frac * cfg_.max_sps);
      if (std::abs(u_int_sps_) < dead) u_int_sps_ = 0.f;

      // Drive motors (no steer here)
      left_.setTarget(u_int_sps_);
      right_.setTarget(u_int_sps_);

      // Telemetry
      if (tel_cb_) {
        rate_ctrl_status_s st{}; rc_.getRateControlStatus(st);
        Telemetry t{}; t.t_sec = std::chrono::duration<double>(now.time_since_epoch()).count();
        t.pitch_deg = pitch_deg; t.pitch_rate_dps = gyro_dps; t.rate_sp_dps = rate_sp_dps;
        t.out_norm = (double)u(1); t.u_sps = (double)u_int_sps_; t.integ_pitch = (double)st.pitchspeed_integ;
        tel_cb_(t);
      }
    }
  }

  // ===== Members =====
  MotorRunnerT& left_;
  MotorRunnerT& right_;
  Config cfg_{};

  std::thread worker_{};
  std::atomic<bool> alive_{true};

  AtomicImu latest_filtered_{};
  AtomicRaw latest_raw_{};
  AtomicJoy joy_{};

  RateControl rc_{}; // PX4 rate PID

  std::function<void(const Telemetry&)> tel_cb_{};
  std::chrono::steady_clock::time_point last_ts_{};

  float u_int_sps_{0.f};
  float att_int_{0.f};
};
