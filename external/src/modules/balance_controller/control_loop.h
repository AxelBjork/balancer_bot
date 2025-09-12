// control_loop.h (simplified)
#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <thread>
#include <algorithm>
#include "config.h"

// PX4 tiny rate PID (submodule)
#include <rate_control.hpp>
#include <matrix/matrix/math.hpp>



// Motor runner concept: must have setTarget(double steps_per_sec)

template <class MotorRunnerT>
class CascadedController {
public:
  CascadedController(MotorRunnerT& left, MotorRunnerT& right)
  : left_(left), right_(right) {
    using matrix::Vector3f;

    // Configure PX4 rate controller (pitch axis only)
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
      if (worker_.joinable()) { worker_.join(); }
    }
  }

  // Push latest prefiltered IMU sample (angle, gyro)
  void pushImu(const ImuSample& s) { latest_filtered_.store(s, std::memory_order_relaxed); }

  // Joystick is ignored in the simplified in-place balancer but kept for API compatibility
  void setJoystick(const JoyCmd& j) { (void)j; }

  void setTelemetrySink(std::function<void(const Telemetry&)> cb) { tel_cb_ = std::move(cb); }

private:
  // ===== Atomic wrappers =====
  struct AtomicImu {
    std::atomic<double> angle{0.0}, gyro{0.0};
    std::atomic<std::int64_t> t_count{0};
    void store(const ImuSample& s, std::memory_order mo) {
      angle.store(s.angle_rad, mo);
      gyro.store(s.gyro_rad_s, mo);
      const auto d = s.t.time_since_epoch();
      t_count.store((std::int64_t)d.count(), mo);
    }
    ImuSample load(std::memory_order mo) const {
      ImuSample s{};
      s.angle_rad = angle.load(mo);
      s.gyro_rad_s = gyro.load(mo);
      using clk = std::chrono::steady_clock;
      auto cnt = t_count.load(mo);
      s.t = std::chrono::time_point<clk>(clk::duration((clk::duration::rep)cnt));
      return s;
    }
  };

  // ===== Core loop (minimal inverted pendulum, in-place) =====
  void loop() {
    using namespace std::chrono;
    using matrix::Vector3f;

    auto next = steady_clock::now();
    constexpr int kMinHz = 50;
    const auto dt_nom = duration<double>(1.0 / std::max(kMinHz, Config::control_hz));

    while (alive_.load(std::memory_order_relaxed)) {
      next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(dt_nom);
      std::this_thread::sleep_until(next);

      float pitch_rad = 0.f;
      float gyro_rad_s = 0.f;
      float dt = 0.f;
      const auto now = steady_clock::now();

      {
        ImuSample s = latest_filtered_.load(std::memory_order_relaxed);
        pitch_rad = (float)s.angle_rad;
        gyro_rad_s = (float)s.gyro_rad_s;
        dt = std::clamp((float)duration<double>(now - last_ts_).count(), 1.f / 2000.f, 0.05f);
        last_ts_ = now;
      }

      // ----- Minimal outer loop: angle -> pitch rate setpoint (rad/s)
      // Target is 0 rad. Proportional only: rate_sp = -K * angle
      const float rate_sp_rad_s = (float)(-Config::angle_to_rate_k * pitch_rad);

      // ----- Inner: PX4 rate PID (pitch axis). We provide no accel feed.
      const Vector3f rate{0.f, gyro_rad_s, 0.f};
      const Vector3f rate_sp{0.f, rate_sp_rad_s, 0.f};
      const Vector3f ang_accel{0.f, 0.f, 0.f};

      const Vector3f u = rc_.update(rate, rate_sp, ang_accel, dt, /*landed=*/false);

      // Map PX4 normalized output to steps/s and clamp
      float u_sps = u(1) * (float)Config::pitch_out_to_sps;
      u_sps = std::clamp(u_sps, -(float)Config::max_sps, +(float)Config::max_sps);

      // Drive both wheels equally (no steering)
      left_.setTarget(u_sps);
      right_.setTarget(u_sps);

      if (tel_cb_) {
        rate_ctrl_status_s st{}; rc_.getRateControlStatus(st);
        Telemetry t{};
        t.t_sec          = duration<double>(now.time_since_epoch()).count();
        t.pitch_deg      = pitch_rad * 180.0 / M_PI;
        t.pitch_rate_dps = gyro_rad_s * 180.0 / M_PI;
        t.rate_sp_dps    = rate_sp_rad_s * 180.0 / M_PI;
        t.out_norm       = (double)u(1);
        t.u_sps          = (double)u_sps;
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

  RateControl rc_{}; // PX4 rate PID

  std::function<void(const Telemetry&)> tel_cb_{};
  std::chrono::steady_clock::time_point last_ts_{};
};
