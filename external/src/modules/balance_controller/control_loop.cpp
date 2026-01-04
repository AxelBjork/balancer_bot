// control_loop.cpp
#include "control_loop.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <thread>
#include <utility>

#include "config_pid.h"

// PX4 includes live ONLY here
#include <uORB/topics/rate_ctrl_status.h>  // if you need the status struct

#include <matrix/matrix/math.hpp>
#include <pid/PID.hpp>
#include <rate_control.hpp>

using matrix::Vector3f;

namespace {}

struct RateControllerCore::Impl {
  std::thread worker{};
  std::atomic<bool> alive{false};

  // IO
  std::function<void(float, float)> motors_cb;
  std::function<void(const Telemetry&)> tel_cb;
  std::function<float()> velocity_cb;  // Velocity feedback in steps/s

  // State
  struct AtomicImu {
    std::atomic<double> angle{0.0}, gyro{0.0};
    std::atomic<std::int64_t> t_count{0};
    void store(const ImuSample& s) {
      angle.store(s.angle_rad, std::memory_order_relaxed);
      gyro.store(s.gyro_rad_s, std::memory_order_relaxed);
      auto d = s.t.time_since_epoch();
      t_count.store((std::int64_t)d.count(), std::memory_order_relaxed);
    }
    ImuSample load() const {
      ImuSample s{};
      s.angle_rad = angle.load(std::memory_order_relaxed);
      s.gyro_rad_s = gyro.load(std::memory_order_relaxed);
      using clk = std::chrono::steady_clock;
      auto cnt = t_count.load(std::memory_order_relaxed);
      s.t = std::chrono::time_point<clk>(clk::duration((clk::duration::rep)cnt));
      return s;
    }
  } latest{};

  struct AtomicJoy {
    std::atomic<float> forward{0.0f}, turn{0.0f};
    void store(const JoyCmd& j) {
      forward.store(j.forward, std::memory_order_relaxed);
      turn.store(j.turn, std::memory_order_relaxed);
    }
    JoyCmd load(std::memory_order order) const {
      return JoyCmd{forward.load(order), turn.load(order)};
    }
  } latest_joy{};

  std::chrono::steady_clock::time_point last_ts{};
  std::chrono::steady_clock::time_point start_ts{};
  RateControl rc{};    // PX4 rate PID
  PID velocity_pid{};  // Velocity PID (steps/s error -> pitch angle setpoint)

  // Split-rate state
  int vel_decimation_counter{0};
  float dt_velocity_accum{0.0f};
  float pitch_setpoint_rad{0.0f};

  void thread_fn() {
    using namespace std::chrono;
    alive.store(true, std::memory_order_relaxed);
    start_ts = steady_clock::now();
    last_ts = start_ts;
    auto next = last_ts;
    const auto dt_nom = duration<double>(1.0 / ConfigPid::control_hz);

    while (alive.load(std::memory_order_relaxed)) {
      next += duration_cast<steady_clock::duration>(dt_nom);
      std::this_thread::sleep_until(next);

      // read imu
      ImuSample s = latest.load();
      float pitch_rad = s.angle_rad;
      float gyro_rad_s = s.gyro_rad_s;
      float dt = std::clamp(duration<float>(s.t - last_ts).count(), 1.f / 2000.f, 0.05f);
      last_ts = s.t;

      // Get velocity feedback (steps/s)
      float current_velocity_sps = velocity_cb ? velocity_cb() : 0.0f;

      // Decimate Velocity Loop (e.g. 100Hz -> 10Hz)
      dt_velocity_accum += dt;
      if (++vel_decimation_counter >= ConfigPid::velocity_decimation) {
          vel_decimation_counter = 0;
          
          // Outermost loop: velocity PID -> pitch angle setpoint
          // Update using accumulated time since last update
          pitch_setpoint_rad = velocity_pid.update(current_velocity_sps, dt_velocity_accum);
          pitch_setpoint_rad = std::clamp(pitch_setpoint_rad, -(float)ConfigPid::max_pitch_setpoint_rad,
                                          (float)ConfigPid::max_pitch_setpoint_rad);
          
          dt_velocity_accum = 0.0f;
      }

      // Middle loop: pitch angle error -> pitch rate setpoint
      float pitch_error_rad = pitch_setpoint_rad - pitch_rad;
      float rate_sp_rad_s = (float)(ConfigPid::angle_to_rate_k * pitch_error_rad);

      // inner PX4 rate PID
      const Vector3f rate{0.f, gyro_rad_s, 0.f};
      const Vector3f rate_sp{0.f, rate_sp_rad_s, 0.f};
      const Vector3f ang_acc{0.f, 0.f, 0.f};

      const Vector3f u = rc.update(rate, rate_sp, ang_acc, dt, /*landed=*/false);

      float u_sps = u(1) * ConfigPid::pitch_out_to_sps;
      u_sps = std::clamp<float>(u_sps, -ConfigPid::max_sps, +ConfigPid::max_sps);

      // Mixing
      float turn_sps = latest_joy.load(std::memory_order_relaxed).turn * ConfigPid::max_sps *
                       0.5f;  // Scale turn
      float left_sps = u_sps + turn_sps;
      float right_sps = u_sps - turn_sps;

      if (motors_cb) motors_cb(left_sps, right_sps);

      if (tel_cb) {
        rate_ctrl_status_s st{};
        rc.getRateControlStatus(st);
        Telemetry t{};
        t.t_sec = duration<double>(last_ts - start_ts).count();
        t.age_ms = duration<double, std::milli>(steady_clock::now() - last_ts).count();
        t.pitch_deg = pitch_rad * 180.0 / M_PI;
        t.pitch_rate_dps = gyro_rad_s * 180.0 / M_PI;
        t.rate_sp_dps = rate_sp_rad_s * 180.0 / M_PI;
        t.out_norm = u(1);
        t.u_sps = u_sps;
        t.integ_pitch = st.pitchspeed_integ;        
        // Debug Vel PID
        t.vel_error = 0.0f - current_velocity_sps; // Setpoint is 0
        t.vel_i_term = velocity_pid.getIntegral();
        t.vel_p_term = t.vel_error * ConfigPid::vel_P; // Approximate P-term
        t.pitch_sp_deg = pitch_setpoint_rad * 180.0 / M_PI;
        
        tel_cb(std::move(t));
      }
    }
  }
};

RateControllerCore::RateControllerCore() : p_(new Impl) {
  // Configure PX4 PID gains once
  p_->rc.setPidGains(
      /*P*/ Vector3f(0.f, ConfigPid::rate_P, 0.f),
      /*I*/ Vector3f(0.f, ConfigPid::rate_I, 0.f),
      /*D*/ Vector3f(0.f, ConfigPid::rate_D, 0.f));
  p_->rc.setIntegratorLimit(Vector3f(0.f, ConfigPid::rate_I_lim, 0.f));
  p_->rc.setFeedForwardGain(Vector3f(0.f, ConfigPid::rate_FF, 0.f));

  // Configure velocity PID
  p_->velocity_pid.setGains(ConfigPid::vel_P, ConfigPid::vel_I, ConfigPid::vel_D);
  p_->velocity_pid.setIntegralLimit(ConfigPid::vel_I_lim);
  p_->velocity_pid.setOutputLimit(ConfigPid::max_pitch_setpoint_rad);
  p_->velocity_pid.setSetpoint(0.0f);  // Target velocity = 0 (maintain position)
}

RateControllerCore::~RateControllerCore() {
  stop();
  delete p_;
}

void RateControllerCore::start() {
  if (p_->alive.load(std::memory_order_relaxed)) return;
  p_->worker = std::thread(&Impl::thread_fn, p_);
}

void RateControllerCore::stop() {
  if (!p_->alive.load(std::memory_order_relaxed)) return;
  p_->alive.store(false, std::memory_order_relaxed);
  if (p_->worker.joinable()) p_->worker.join();
}

void RateControllerCore::pushImu(const ImuSample& s) {
  p_->latest.store(s);
}
void RateControllerCore::setJoystick(const JoyCmd& j) {
  p_->latest_joy.store(j);
}
void RateControllerCore::setTelemetrySink(std::function<void(const Telemetry&)> cb) {
  p_->tel_cb = std::move(cb);
}
void RateControllerCore::setMotorOutputs(std::function<void(float, float)> motors_cb) {
  p_->motors_cb = std::move(motors_cb);
}
void RateControllerCore::setVelocityFeedback(std::function<float()> velocity_cb) {
  p_->velocity_cb = std::move(velocity_cb);
}
