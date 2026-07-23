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

  std::function<void(double, double)> motors_cb;
  std::function<void(const Telemetry&)> tel_cb;

  ImuSample latest_imu{};
  JoyCmd latest_joy{0.0, 0.0};
  bool have_imu{false};
  bool initialized{false};

  std::chrono::steady_clock::time_point start_ts{};
  RateControl rc{};

  double pitch_setpoint_rad{0.0};
  double nominal_acceleration_m_s2{0.0};
  double lean_trim_rad{0.0};
  bool lean_trim_active{false};
  double measured_velocity_sps{0.0};
  double vel_error_sps{0.0};
  double velocity_pitch_setpoint_rad{0.0};
  double turn_sps{0.0};
  double last_u_sps{0.0};
  bool command_saturated{false};
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

void RateControllerCore::updateOuterLoop(double measured_velocity_sps, double dt_s) {
  const JoyCmd joy = p_->latest_joy;
  if (!std::isfinite(measured_velocity_sps) || !std::isfinite(dt_s) || dt_s <= 0.0 ||
      !std::isfinite(joy.forward) || !std::isfinite(ConfigPid::max_acceleration_m_s2) ||
      !std::isfinite(ConfigPid::max_jerk_m_s3) || !std::isfinite(ConfigPid::velocity_damping_per_s)) {
    return;
  }

  const double corrected_axle_velocity_sps = measured_velocity_sps;
  p_->measured_velocity_sps = corrected_axle_velocity_sps;
  p_->vel_error_sps = -corrected_axle_velocity_sps;

  const double max_trim_rad = std::clamp(ConfigPid::lean_trim_max_deg * M_PI / 180.0, 0.0, kMaxPitchSetpointRad);

  if (std::abs(joy.forward) > Config::deadzone && kLeanTrimDecayS > 0.0) {
    p_->lean_trim_rad *= (1.0 - std::clamp(dt_s / kLeanTrimDecayS, 0.0, 1.0));
    p_->lean_trim_active = false;
  } else if (ConfigPid::lean_trim_I != 0.0 && !p_->command_saturated) {
    p_->lean_trim_rad =
        std::clamp(p_->lean_trim_rad -
                       ConfigPid::lean_trim_I * (measured_velocity_sps / kMaxSps) * dt_s,
                   -max_trim_rad, max_trim_rad);
    p_->lean_trim_active = std::abs(measured_velocity_sps) > 1e-3;
  } else {
    p_->lean_trim_active = false;
  }

  const double max_acceleration_m_s2 = std::max(0.0, ConfigPid::max_acceleration_m_s2);
  const double max_jerk_m_s3 = std::max(0.0, ConfigPid::max_jerk_m_s3);
  const double target_acceleration_m_s2 =
      std::clamp(joy.forward, -1.0, 1.0) * max_acceleration_m_s2;
  const double max_acceleration_delta_m_s2 = max_jerk_m_s3 * dt_s;
  p_->nominal_acceleration_m_s2 += std::clamp(
      target_acceleration_m_s2 - p_->nominal_acceleration_m_s2,
      -max_acceleration_delta_m_s2, max_acceleration_delta_m_s2);

  // On release, zero stick slews this retained state toward zero. Until it
  // reaches zero, damping still subtracts the measured velocity and requests
  // braking rather than turning the release into a motor-speed feed-forward.
  const double corrected_axle_velocity_m_s = corrected_axle_velocity_sps * Config::meters_per_step;
  const double commanded_acceleration_m_s2 = p_->nominal_acceleration_m_s2 -
                                             ConfigPid::velocity_damping_per_s * corrected_axle_velocity_m_s;
  p_->velocity_pitch_setpoint_rad = std::atan2(commanded_acceleration_m_s2, Config::g0);
  p_->pitch_setpoint_rad = std::clamp(p_->velocity_pitch_setpoint_rad + p_->lean_trim_rad,
                                      -kMaxPitchSetpointRad, kMaxPitchSetpointRad);
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
  const JoyCmd joy = p_->latest_joy;
  p_->turn_sps = static_cast<double>(joy.turn) * kPitchOutToSps * 0.4;

  const double rate_sp_rad_s =
      ConfigPid::pitch_P * (p_->pitch_setpoint_rad - pitch_rad) - ConfigPid::pitch_D * pitch_rate_rad_s;

  const Vector3f u = p_->rc.update({0.0f, static_cast<float>(pitch_rate_rad_s), 0.0f},
                                   {0.0f, static_cast<float>(rate_sp_rad_s), 0.0f},
                                   {0.0f, static_cast<float>(pitch_accel_rad_s2), 0.0f},
                                   static_cast<float>(dt), false);

  double u_sps = std::clamp(static_cast<double>(u(1)) * kPitchOutToSps, -kMaxSps, kMaxSps);
  p_->last_u_sps = u_sps;
  p_->command_saturated = std::abs(u_sps) >= (0.99 * kMaxSps);

  if (p_->motors_cb) p_->motors_cb(u_sps + p_->turn_sps, u_sps - p_->turn_sps);

  if (p_->tel_cb) {
    Telemetry t{};
    t.t_sec = std::chrono::duration<double>(now - p_->start_ts).count();
    t.age_ms = std::chrono::duration<double, std::milli>(now - p_->latest_imu.t).count();
    t.pitch_deg = pitch_rad * 180.0 / M_PI;
    t.pitch_rate_dps = pitch_rate_rad_s * 180.0 / M_PI;
    t.filtered_pitch_rate_dps = t.pitch_rate_dps;
    t.u_sps = u_sps;
    t.turn_sps = p_->turn_sps;
    t.vel_error = p_->vel_error_sps;
    t.vel_p_term = p_->velocity_pitch_setpoint_rad;
    t.measured_vel_sps = p_->measured_velocity_sps;
    t.pitch_ref_from_vel_deg = p_->velocity_pitch_setpoint_rad * 180.0 / M_PI;
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
