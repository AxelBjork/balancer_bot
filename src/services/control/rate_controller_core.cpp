#include "rate_controller_core.h"

#include <uORB/topics/rate_ctrl_status.h>

#include <algorithm>
#include <cmath>
#include <matrix/matrix/math.hpp>
#include <rate_control.hpp>
#include <utility>

#include "services/main/config.h"

using matrix::Vector3f;

namespace {

constexpr double kMaxPitchRateSpRadS = 4.0;
constexpr double kMaxImuAgeS = 0.030;
constexpr double kMaxImuFutureS = 0.002;
constexpr double kFalloverMarginRad = 5.0 * M_PI / 180.0;
constexpr double kVelocityLoopPeriodS = 1.0 / 50.0;

double joystick_to_sps(double command) {
  const double magnitude = std::abs(command);
  if (magnitude <= Config::deadzone) return 0.0;
  const double normalized =
      std::clamp((magnitude - Config::deadzone) / (1.0 - Config::deadzone), 0.0, 1.0);
  return std::copysign(normalized * ConfigPid::drive_max_sps, command);
}

}  // namespace

struct RateControllerCore::Impl {
  std::function<void(double, double)> motors_cb;
  std::function<void(const Telemetry&)> tel_cb;

  ImuSample latest_imu{};
  JoyCmd latest_joy{0.0, 0.0};
  bool have_imu{false};
  bool initialized{false};

  std::chrono::steady_clock::time_point start_ts{};
  RateControl rc{};

  double target_velocity_sps{0.0};
  double measured_velocity_sps{0.0};
  double vel_error_sps{0.0};
  double velocity_p_rad{0.0};
  double velocity_i_rad{0.0};
  double pitch_setpoint_rad{0.0};
  double rate_setpoint_rad_s{0.0};
  double turn_sps{0.0};
  double last_u_sps{0.0};
  double outer_elapsed_s{0.0};
  bool command_saturated{false};
  bool balance_saturated_positive{false};
  bool balance_saturated_negative{false};
  bool actuator_fault{false};
  uint32_t controller_fault_flags{ControllerFaultNone};
  uint32_t controller_saturation_flags{ControllerSaturationNone};
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

void RateControllerCore::setMotorFeedback(double measured_velocity_sps, bool actuator_fault) {
  p_->measured_velocity_sps = measured_velocity_sps;
  p_->actuator_fault = actuator_fault;
}

void RateControllerCore::step(double dt_s, std::chrono::steady_clock::time_point now) {
  const double dt = std::clamp(dt_s, 1.0 / 2000.0, 0.05);
  if (!p_->initialized) {
    p_->initialized = true;
    p_->start_ts = now;
  }

  const auto reset_outputs = [this]() {
    p_->rc.resetIntegral(1);
    p_->target_velocity_sps = 0.0;
    p_->vel_error_sps = 0.0;
    p_->velocity_p_rad = 0.0;
    p_->velocity_i_rad = 0.0;
    p_->pitch_setpoint_rad = 0.0;
    p_->rate_setpoint_rad_s = 0.0;
    p_->turn_sps = 0.0;
    p_->last_u_sps = 0.0;
    p_->outer_elapsed_s = 0.0;
    p_->command_saturated = false;
    p_->balance_saturated_positive = false;
    p_->balance_saturated_negative = false;
    p_->controller_saturation_flags = ControllerSaturationNone;
  };

  const auto publish_telemetry = [this, now](double pitch_rad, double pitch_rate_rad_s,
                                             double age_ms) {
    if (!p_->tel_cb) return;
    Telemetry t{};
    t.t_sec = std::chrono::duration<double>(now - p_->start_ts).count();
    t.age_ms = age_ms;
    t.pitch_deg = pitch_rad * 180.0 / M_PI;
    t.pitch_rate_dps = pitch_rate_rad_s * 180.0 / M_PI;
    t.filtered_pitch_rate_dps = t.pitch_rate_dps;
    t.u_sps = p_->last_u_sps;
    t.turn_sps = p_->turn_sps;
    t.target_vel_sps = p_->target_velocity_sps;
    t.vel_error = p_->vel_error_sps;
    t.vel_p_term_deg = p_->velocity_p_rad * 180.0 / M_PI;
    t.vel_i_term_deg = p_->velocity_i_rad * 180.0 / M_PI;
    t.measured_vel_sps = p_->measured_velocity_sps;
    t.pitch_error_deg = (p_->pitch_setpoint_rad - pitch_rad) * 180.0 / M_PI;
    t.pitch_sp_deg = p_->pitch_setpoint_rad * 180.0 / M_PI;
    t.rate_sp_dps = p_->rate_setpoint_rad_s * 180.0 / M_PI;
    t.rate_error_dps = (p_->rate_setpoint_rad_s - pitch_rate_rad_s) * 180.0 / M_PI;
    t.command_saturated = p_->command_saturated ? 1.0 : 0.0;
    t.actuator_fault = p_->actuator_fault ? 1.0 : 0.0;
    t.controller_fault_flags = p_->controller_fault_flags;
    t.controller_saturation_flags = p_->controller_saturation_flags;
    p_->tel_cb(std::move(t));
  };

  if (!p_->have_imu) {
    reset_outputs();
    p_->controller_fault_flags = ControllerFaultNoImu;
    if (p_->motors_cb) p_->motors_cb(0.0, 0.0);
    publish_telemetry(0.0, 0.0, 0.0);
    return;
  }

  const double pitch_rad = p_->latest_imu.angle_rad;
  const double pitch_rate_rad_s = p_->latest_imu.gyro_rad_s;
  const double pitch_accel_rad_s2 = p_->latest_imu.pitch_accel_rad_s2;
  const double imu_age_s = std::chrono::duration<double>(now - p_->latest_imu.t).count();
  const double pitch_limit_rad = ConfigPid::pitch_max_deg * M_PI / 180.0;
  const double fallover_limit_rad =
      std::max(Config::max_tilt_rad, pitch_limit_rad + kFalloverMarginRad);

  uint32_t fault_flags = ControllerFaultNone;
  if (imu_age_s > kMaxImuAgeS) fault_flags |= ControllerFaultStaleImu;
  if (imu_age_s < -kMaxImuFutureS) fault_flags |= ControllerFaultFutureImu;
  if (std::abs(pitch_rad) > fallover_limit_rad) fault_flags |= ControllerFaultFallover;
  if (p_->actuator_fault) fault_flags |= ControllerFaultActuator;
  if (fault_flags != ControllerFaultNone) {
    reset_outputs();
    p_->controller_fault_flags = fault_flags;
    if (p_->motors_cb) p_->motors_cb(0.0, 0.0);
    publish_telemetry(pitch_rad, pitch_rate_rad_s, imu_age_s * 1000.0);
    return;
  }

  p_->controller_fault_flags = ControllerFaultNone;
  p_->controller_saturation_flags = ControllerSaturationNone;

  const JoyCmd joy = p_->latest_joy;

  p_->outer_elapsed_s += dt;
  if (p_->outer_elapsed_s + 1e-12 >= kVelocityLoopPeriodS) {
    const double outer_dt = p_->outer_elapsed_s;
    p_->outer_elapsed_s = 0.0;
    p_->target_velocity_sps = joystick_to_sps(joy.forward);
    p_->vel_error_sps = p_->target_velocity_sps - p_->measured_velocity_sps;
    p_->velocity_p_rad = ConfigPid::velocity_P * p_->vel_error_sps * M_PI / 180.0;

    const double integral_limit_rad = ConfigPid::velocity_I_limit_deg * M_PI / 180.0;
    const double integral_delta =
        ConfigPid::velocity_I * p_->vel_error_sps * outer_dt * M_PI / 180.0;
    const double candidate_i =
        std::clamp(p_->velocity_i_rad + integral_delta, -integral_limit_rad, integral_limit_rad);
    const double candidate_pitch = p_->velocity_p_rad + candidate_i;
    const bool pitch_would_saturate = std::abs(candidate_pitch) > pitch_limit_rad;
    const bool drives_further_into_pitch_limit =
        pitch_would_saturate && (candidate_pitch * integral_delta > 0.0);
    const bool drives_further_into_actuator_limit = p_->command_saturated;
    if (!drives_further_into_pitch_limit && !drives_further_into_actuator_limit) {
      p_->velocity_i_rad = candidate_i;
    }
    p_->pitch_setpoint_rad =
        std::clamp(p_->velocity_p_rad + p_->velocity_i_rad, -pitch_limit_rad, pitch_limit_rad);
  }
  if (std::abs(p_->velocity_p_rad + p_->velocity_i_rad) > pitch_limit_rad) {
    p_->controller_saturation_flags |= ControllerSaturationPitch;
  }

  const double rate_sp_raw_rad_s = ConfigPid::angle_P * (p_->pitch_setpoint_rad - pitch_rad) -
                                   ConfigPid::angle_D * pitch_rate_rad_s;
  const double rate_sp_rad_s =
      std::clamp(rate_sp_raw_rad_s, -kMaxPitchRateSpRadS, kMaxPitchRateSpRadS);
  if (rate_sp_rad_s != rate_sp_raw_rad_s) {
    p_->controller_saturation_flags |= ControllerSaturationRate;
  }
  p_->rate_setpoint_rad_s = rate_sp_rad_s;

  // RateControl's sign is opposite the robot-forward wheel command at this
  // mechanical boundary, so saturation flags are swapped as well.
  p_->rc.setPositiveSaturationFlag(1, p_->balance_saturated_negative);
  p_->rc.setNegativeSaturationFlag(1, p_->balance_saturated_positive);

  const Vector3f u = p_->rc.update({0.0f, static_cast<float>(pitch_rate_rad_s), 0.0f},
                                   {0.0f, static_cast<float>(rate_sp_rad_s), 0.0f},
                                   {0.0f, static_cast<float>(pitch_accel_rad_s2), 0.0f},
                                   static_cast<float>(dt), false);

  // The rate loop supplies the balance correction around the commanded wheel
  // speed. Without this kinematic feed-forward, a velocity-command actuator
  // can only sustain travel by holding a permanent attitude/rate error.
  const double raw_balance_sps =
      p_->target_velocity_sps - static_cast<double>(u(1)) * ConfigPid::output_scale_sps;
  const double balance_limit_sps = std::clamp(ConfigPid::balance_max_sps, 0.0, kMaxSps);
  const double balance_sps = std::clamp(raw_balance_sps, -balance_limit_sps, balance_limit_sps);
  const double raw_turn_sps =
      std::clamp(static_cast<double>(joy.turn), -1.0, 1.0) * ConfigPid::turn_max_sps;
  const double turn_limit_sps = std::max(0.0, balance_limit_sps - std::abs(balance_sps));
  p_->turn_sps = std::clamp(raw_turn_sps, -turn_limit_sps, turn_limit_sps);
  if (balance_sps != raw_balance_sps) {
    p_->controller_saturation_flags |= ControllerSaturationBalance;
  }
  if (p_->turn_sps != raw_turn_sps) {
    p_->controller_saturation_flags |= ControllerSaturationTurn;
  }

  const double left_sps =
      std::clamp(balance_sps + p_->turn_sps, -balance_limit_sps, balance_limit_sps);
  const double right_sps =
      std::clamp(balance_sps - p_->turn_sps, -balance_limit_sps, balance_limit_sps);

  p_->last_u_sps = balance_sps;
  p_->command_saturated = std::abs(left_sps) >= (0.99 * balance_limit_sps) ||
                          std::abs(right_sps) >= (0.99 * balance_limit_sps);
  p_->balance_saturated_positive = raw_balance_sps > balance_limit_sps;
  p_->balance_saturated_negative = raw_balance_sps < -balance_limit_sps;

  if (p_->motors_cb) p_->motors_cb(left_sps, right_sps);
  publish_telemetry(pitch_rad, pitch_rate_rad_s, imu_age_s * 1000.0);
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
