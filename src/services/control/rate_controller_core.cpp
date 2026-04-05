#include "rate_controller_core.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "config.h"

#include <uORB/topics/rate_ctrl_status.h>

#include <matrix/matrix/math.hpp>
#include <pid/PID.hpp>
#include <rate_control.hpp>

using matrix::Vector3f;

struct RateControllerCore::Impl {
  std::function<void(float, float)> motors_cb;
  std::function<void(const Telemetry&)> tel_cb;
  std::function<float()> velocity_cb;

  ImuSample latest_imu{};
  JoyCmd latest_joy{0.0f, 0.0f};
  bool have_imu{false};
  bool initialized{false};

  std::chrono::steady_clock::time_point start_ts{};
  RateControl rc{};
  PID velocity_pid{};

  int vel_decimation_counter{0};
  float dt_velocity_accum{0.0f};
  float pitch_setpoint_rad{0.0f};
};

RateControllerCore::RateControllerCore() : p_(new Impl) {
  p_->rc.setPidGains(
      Vector3f(0.f, ConfigPid::rate_P, 0.f),
      Vector3f(0.f, ConfigPid::rate_I, 0.f),
      Vector3f(0.f, ConfigPid::rate_D, 0.f));
  p_->rc.setIntegratorLimit(Vector3f(0.f, ConfigPid::rate_I_lim, 0.f));
  p_->rc.setFeedForwardGain(Vector3f(0.f, ConfigPid::rate_FF, 0.f));

  p_->velocity_pid.setGains(ConfigPid::vel_P, ConfigPid::vel_I, ConfigPid::vel_D);
  p_->velocity_pid.setIntegralLimit(ConfigPid::vel_I_lim);
  p_->velocity_pid.setOutputLimit(kMaxPitchSetpointRad);
  p_->velocity_pid.setSetpoint(0.0f);
}

RateControllerCore::~RateControllerCore() {
  stop();
  delete p_;
}

void RateControllerCore::start() {}

void RateControllerCore::stop() {}

void RateControllerCore::step(double dt_s, std::chrono::steady_clock::time_point now) {
  if (!p_->have_imu) {
    return;
  }

  const float dt = std::clamp(static_cast<float>(dt_s), 1.0f / 2000.0f, 0.05f);
  if (!p_->initialized) {
    p_->initialized = true;
    p_->start_ts = now;
  }

  const float pitch_rad = static_cast<float>(p_->latest_imu.angle_rad);
  const float gyro_rad_s = static_cast<float>(p_->latest_imu.gyro_rad_s);
  const float current_velocity_sps = p_->velocity_cb ? p_->velocity_cb() : 0.0f;

  const JoyCmd joy = p_->latest_joy;
  const float target_vel_sps = joy.forward * static_cast<float>(kMaxSps);
  p_->velocity_pid.setSetpoint(target_vel_sps);

  p_->dt_velocity_accum += dt;
  if (++p_->vel_decimation_counter >= kVelocityDecimation) {
    p_->vel_decimation_counter = 0;
    p_->pitch_setpoint_rad = p_->velocity_pid.update(current_velocity_sps, p_->dt_velocity_accum);
    p_->pitch_setpoint_rad = std::clamp(p_->pitch_setpoint_rad,
                                        -static_cast<float>(kMaxPitchSetpointRad),
                                        static_cast<float>(kMaxPitchSetpointRad));
    p_->dt_velocity_accum = 0.0f;
  }

  const float pitch_error_rad = p_->pitch_setpoint_rad - pitch_rad;
  const float rate_sp_rad_s = static_cast<float>(ConfigPid::angle_to_rate_k * pitch_error_rad);

  const Vector3f rate{0.f, gyro_rad_s, 0.f};
  const Vector3f rate_sp{0.f, rate_sp_rad_s, 0.f};
  const Vector3f ang_acc{0.f, 0.f, 0.f};
  const Vector3f u = p_->rc.update(rate, rate_sp, ang_acc, dt, false);

  float u_sps = u(1) * static_cast<float>(kPitchOutToSps);
  u_sps = std::clamp(u_sps, -static_cast<float>(kMaxSps), static_cast<float>(kMaxSps));

  const float turn_sps = joy.turn * static_cast<float>(kMaxSps) * 0.5f;
  const float left_sps = u_sps + turn_sps;
  const float right_sps = u_sps - turn_sps;

  if (p_->motors_cb) {
    p_->motors_cb(left_sps, right_sps);
  }

  if (p_->tel_cb) {
    rate_ctrl_status_s st{};
    p_->rc.getRateControlStatus(st);

    Telemetry t{};
    t.t_sec = std::chrono::duration<double>(now - p_->start_ts).count();
    t.age_ms = std::chrono::duration<double, std::milli>(now - p_->latest_imu.t).count();
    t.pitch_deg = pitch_rad * 180.0 / M_PI;
    t.pitch_rate_dps = gyro_rad_s * 180.0 / M_PI;
    t.rate_sp_dps = rate_sp_rad_s * 180.0 / M_PI;
    t.out_norm = u(1);
    t.u_sps = u_sps;
    t.integ_pitch = st.pitchspeed_integ;
    t.vel_error = target_vel_sps - current_velocity_sps;
    t.vel_i_term = p_->velocity_pid.getIntegral();
    t.vel_p_term = t.vel_error * ConfigPid::vel_P;
    t.pitch_sp_deg = p_->pitch_setpoint_rad * 180.0 / M_PI;

    p_->tel_cb(std::move(t));
  }
}

void RateControllerCore::pushImu(const ImuSample& s) {
  p_->latest_imu = s;
  p_->have_imu = true;
}

void RateControllerCore::setJoystick(const JoyCmd& j) {
  p_->latest_joy = j;
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
