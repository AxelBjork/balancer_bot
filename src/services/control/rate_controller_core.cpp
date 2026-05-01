#include "rate_controller_core.h"

#include <uORB/topics/rate_ctrl_status.h>

#include <algorithm>
#include <cmath>
#include <matrix/matrix/math.hpp>
#include <rate_control.hpp>
#include <utility>

#include "services/main/config.h"

using matrix::Vector3f;

struct RateControllerCore::Impl {
  static constexpr double kLeanTrimI = 0.03;
  static constexpr double kLeanTrimMaxDeg = 4.0;
  static constexpr double kLeanTrimDecayS = 30.0;

  std::function<void(double, double)> motors_cb;
  std::function<void(const Telemetry&)> tel_cb;
  std::function<double()> velocity_cb;
  std::function<double()> position_cb;

  ImuSample latest_imu{};
  JoyCmd latest_joy{0.0, 0.0};
  bool have_imu{false};
  bool initialized{false};

  std::chrono::steady_clock::time_point start_ts{};
  RateControl rc{};

  double pitch_setpoint_rad{0.0};
  double lean_trim_rad{0.0};
  bool lean_trim_active{false};
};

RateControllerCore::RateControllerCore() : p_(new Impl) {
  p_->rc.setPidGains(Vector3f(0.0f, static_cast<float>(ConfigPid::rate_P), 0.0f),
                     Vector3f(0.0f, static_cast<float>(ConfigPid::rate_I), 0.0f),
                     Vector3f(0.0f, static_cast<float>(ConfigPid::rate_D), 0.0f));
  p_->rc.setIntegratorLimit(Vector3f(0.0f, static_cast<float>(ConfigPid::rate_I_lim), 0.0f));
  p_->rc.setFeedForwardGain(Vector3f(0.0f, static_cast<float>(ConfigPid::rate_FF), 0.0f));
}

RateControllerCore::~RateControllerCore() {
  stop();
  delete p_;
}

void RateControllerCore::start() {
}
void RateControllerCore::stop() {
}

void RateControllerCore::step(double dt_s, std::chrono::steady_clock::time_point now) {
  if (!p_->have_imu) {
    return;
  }

  const double dt = std::clamp(dt_s, 1.0 / 2000.0, 0.05);
  if (!p_->initialized) {
    p_->initialized = true;
    p_->start_ts = now;
  }

  const double pitch_rad = p_->latest_imu.angle_rad;
  const double pitch_rate_rad_s = p_->latest_imu.gyro_rad_s;
  const double pitch_accel_rad_s2 = p_->latest_imu.pitch_accel_rad_s2;
  const double measured_velocity_sps = p_->velocity_cb ? p_->velocity_cb() : 0.0;
  const JoyCmd joy = p_->latest_joy;

  // Velocity braking (always toward zero)
  const double vel_error_sps = -measured_velocity_sps;
  const double velocity_pitch_setpoint_rad = ConfigPid::vel_P * vel_error_sps;

  // Joystick: additive pitch offset
  const double joy_pitch_rad = static_cast<double>(joy.forward) * kMaxPitchSetpointRad;

  p_->pitch_setpoint_rad =
      std::clamp(velocity_pitch_setpoint_rad + p_->lean_trim_rad + joy_pitch_rad,
                 -kMaxPitchSetpointRad, kMaxPitchSetpointRad);

  const double rate_sp_rad_s =
      ConfigPid::pitch_P * (p_->pitch_setpoint_rad - pitch_rad) - ConfigPid::pitch_D * pitch_rate_rad_s;

  const Vector3f u = p_->rc.update({0.0f, static_cast<float>(pitch_rate_rad_s), 0.0f},
                                   {0.0f, static_cast<float>(rate_sp_rad_s), 0.0f},
                                   {0.0f, static_cast<float>(pitch_accel_rad_s2), 0.0f},
                                   static_cast<float>(dt), false);

  double u_sps = std::clamp(static_cast<double>(u(1)) * kPitchOutToSps, -kMaxSps, kMaxSps);
  const bool command_saturated = std::abs(u_sps) >= (0.99 * kMaxSps);

  // Lean trim: accumulate proportional to velocity
  const double max_trim_rad =
      std::clamp(Impl::kLeanTrimMaxDeg * M_PI / 180.0, 0.0, kMaxPitchSetpointRad);
  if (std::abs(joy.forward) > Config::deadzone && Impl::kLeanTrimDecayS > 0.0) {
    p_->lean_trim_rad *= (1.0 - std::clamp(dt / Impl::kLeanTrimDecayS, 0.0, 1.0));
    p_->lean_trim_active = false;
  } else if (Impl::kLeanTrimI != 0.0 && !command_saturated) {
    p_->lean_trim_rad = std::clamp(
        p_->lean_trim_rad - Impl::kLeanTrimI * (measured_velocity_sps / kMaxSps) * dt,
        -max_trim_rad, max_trim_rad);
    p_->lean_trim_active = std::abs(measured_velocity_sps) > 1e-3;
  } else {
    p_->lean_trim_active = false;
  }

  const double turn_sps = static_cast<double>(joy.turn) * kMaxSps * 0.5;
  if (p_->motors_cb) p_->motors_cb(u_sps + turn_sps, u_sps - turn_sps);

  if (p_->tel_cb) {
    Telemetry t{};
    t.t_sec = std::chrono::duration<double>(now - p_->start_ts).count();
    t.age_ms = std::chrono::duration<double, std::milli>(now - p_->latest_imu.t).count();
    t.pitch_deg = pitch_rad * 180.0 / M_PI;
    t.pitch_rate_dps = pitch_rate_rad_s * 180.0 / M_PI;
    t.filtered_pitch_rate_dps = t.pitch_rate_dps;
    t.u_sps = u_sps;
    t.turn_sps = turn_sps;
    t.vel_error = vel_error_sps;
    t.vel_p_term = velocity_pitch_setpoint_rad;
    t.measured_vel_sps = measured_velocity_sps;
    t.pitch_ref_from_vel_deg = velocity_pitch_setpoint_rad * 180.0 / M_PI;
    t.pitch_ref_from_pos_deg = 0.0;
    t.pitch_error_deg = (p_->pitch_setpoint_rad - pitch_rad) * 180.0 / M_PI;
    t.pitch_sp_deg = p_->pitch_setpoint_rad * 180.0 / M_PI;
    t.pitch_trim_deg = p_->lean_trim_rad * 180.0 / M_PI;
    t.trim_active = p_->lean_trim_active ? 1.0 : 0.0;
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

void RateControllerCore::setMotorOutputs(std::function<void(double, double)> motors_cb) {
  p_->motors_cb = std::move(motors_cb);
}

void RateControllerCore::setVelocityFeedback(std::function<double()> velocity_cb) {
  p_->velocity_cb = std::move(velocity_cb);
}

void RateControllerCore::setPositionFeedback(std::function<double()> position_cb) {
  p_->position_cb = std::move(position_cb);
}
