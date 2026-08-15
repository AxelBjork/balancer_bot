#include "rate_controller_core.h"

#include <uORB/topics/rate_ctrl_status.h>

#include <algorithm>
#include <cmath>
#include <matrix/matrix/math.hpp>
#include <rate_control.hpp>
#include <utility>

#include "services/main/config.h"

using matrix::Vector3f;

namespace rate_controller_detail {
double wrap_angle_delta(double angle_rad) {
  return std::remainder(angle_rad, 2.0 * M_PI);
}
}  // namespace rate_controller_detail

namespace {

constexpr double kMaxImuAgeS = 0.030;
constexpr double kMaxImuFutureS = 0.002;
constexpr double kFalloverMarginRad = 5.0 * M_PI / 180.0;
constexpr double kFalloverRearmPitchRad = 10.0 * M_PI / 180.0;
constexpr double kFalloverRearmRateRadS = 30.0 * M_PI / 180.0;
constexpr double kVelocityLoopPeriodS = 1.0 / 100.0;
// A fixed command-slew safety limit, deliberately not a PID tuning parameter.
constexpr double kDriveMaxJerkMps3 = 6.0;
double normalized_forward_command(double command) {
  const double magnitude = std::abs(command);
  if (magnitude <= Config::deadzone) return 0.0;
  const double normalized =
      std::clamp((magnitude - Config::deadzone) / (1.0 - Config::deadzone), 0.0, 1.0);
  return std::copysign(normalized, command);
}

double move_toward(double value, double target, double max_delta) {
  if (value < target) return std::min(value + max_delta, target);
  if (value > target) return std::max(value - max_delta, target);
  return value;
}

double drive_pitch_limit_rad() {
  // For the compact plant model, theta_ddot = 0 requires theta = x_ddot / g.
  // atan2 extends that equilibrium relation away from the small-angle limit.
  const double drive_equilibrium_rad =
      std::atan2(std::max(0.0, ConfigPid::drive_max_acceleration_mps2), Config::g0);
  const double trim_limit_rad =
      std::max(0.0, ConfigPid::velocity_I_limit_deg) * M_PI / 180.0;
  return std::min(kMaxPitchSetpointRad, std::max(drive_equilibrium_rad, trim_limit_rad));
}

constexpr double kStepsPerRad = Config::steps_per_rev / (2.0 * M_PI);

}  // namespace

struct RateControllerCore::Impl {
  std::function<void(double, double)> motors_cb;
  std::function<void(const Telemetry&)> tel_cb;

  ImuSample latest_imu{};
  JoyCmd latest_joy{0.0, 0.0};
  bool have_imu{false};
  bool initialized{false};
  bool balance_armed{false};

  std::chrono::steady_clock::time_point start_ts{};
  RateControl rc{};

  int64_t left_actual_steps{0};
  int64_t right_actual_steps{0};
  bool have_motor_feedback{false};
  bool velocity_observer_seeded{false};
  double previous_common_steps{0.0};
  double previous_pitch_rad{0.0};
  double raw_completed_velocity_sps{0.0};
  double corrected_axle_velocity_sps{0.0};
  double filtered_raw_completed_velocity_sps{0.0};
  double nominal_acceleration_mps2{0.0};
  double velocity_damping_acceleration_mps2{0.0};
  double com_trim_rad{0.0};
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

void RateControllerCore::setMotorFeedback(int64_t left_actual_steps, int64_t right_actual_steps,
                                          bool actuator_fault) {
  p_->left_actual_steps = left_actual_steps;
  p_->right_actual_steps = right_actual_steps;
  p_->have_motor_feedback = true;
  p_->actuator_fault = actuator_fault;
}

void RateControllerCore::step(double dt_s, std::chrono::steady_clock::time_point now) {
  const double dt = std::clamp(dt_s, 1.0 / 2000.0, 0.05);
  if (!p_->initialized) {
    p_->initialized = true;
    p_->start_ts = now;
  }

  const auto invalidate_velocity_observer = [this]() {
    p_->velocity_observer_seeded = false;
    p_->raw_completed_velocity_sps = 0.0;
    p_->corrected_axle_velocity_sps = 0.0;
    p_->filtered_raw_completed_velocity_sps = 0.0;
  };
  const auto reset_outputs = [this, &invalidate_velocity_observer]() {
    p_->rc.resetIntegral(1);
    p_->nominal_acceleration_mps2 = 0.0;
    p_->velocity_damping_acceleration_mps2 = 0.0;
    invalidate_velocity_observer();
    // com_trim_rad is the bounded physical COM trim. Preserve it across a
    // transient fault/fallover; only dynamic command and balance state reset.
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
    t.nominal_acceleration_mps2 = p_->nominal_acceleration_mps2;
    t.raw_completed_velocity_sps = p_->filtered_raw_completed_velocity_sps;
    t.corrected_axle_velocity_sps = p_->corrected_axle_velocity_sps;
    t.velocity_damping_acceleration_mps2 = p_->velocity_damping_acceleration_mps2;
    t.com_trim_deg = p_->com_trim_rad * 180.0 / M_PI;
    t.target_vel_sps = t.nominal_acceleration_mps2;
    t.vel_error = -t.corrected_axle_velocity_sps;
    t.vel_p_term_deg = t.velocity_damping_acceleration_mps2;
    t.vel_i_term_deg = t.com_trim_deg;
    t.measured_vel_sps = t.corrected_axle_velocity_sps;
    t.pitch_error_deg = (p_->pitch_setpoint_rad - pitch_rad) * 180.0 / M_PI;
    t.pitch_sp_deg = p_->pitch_setpoint_rad * 180.0 / M_PI;
    t.rate_sp_dps = p_->rate_setpoint_rad_s * 180.0 / M_PI;
    t.rate_error_dps = (p_->rate_setpoint_rad_s - pitch_rate_rad_s) * 180.0 / M_PI;
    t.command_saturated = p_->command_saturated;
    t.actuator_fault = p_->actuator_fault;
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
  const double pitch_limit_rad = drive_pitch_limit_rad();
  const double fallover_limit_rad =
      std::max(Config::max_tilt_rad, pitch_limit_rad + kFalloverMarginRad);
  const JoyCmd joy = p_->latest_joy;
  const double forward_command =
      std::isfinite(joy.forward) ? normalized_forward_command(joy.forward) : 0.0;
  const bool fallover_recovery_requested = forward_command != 0.0;

  uint32_t fault_flags = ControllerFaultNone;
  if (imu_age_s > kMaxImuAgeS) fault_flags |= ControllerFaultStaleImu;
  if (imu_age_s < -kMaxImuFutureS) fault_flags |= ControllerFaultFutureImu;
  if (p_->balance_armed && !fallover_recovery_requested &&
      std::abs(pitch_rad) > fallover_limit_rad) {
    p_->balance_armed = false;
  }
  const bool rearm_attitude_allowed =
      fallover_recovery_requested || std::abs(pitch_rad) <= kFalloverRearmPitchRad;
  if (!p_->balance_armed && rearm_attitude_allowed &&
      std::abs(pitch_rate_rad_s) <= kFalloverRearmRateRadS) {
    p_->balance_armed = true;
  }
  if (!p_->balance_armed) fault_flags |= ControllerFaultFallover;
  if (p_->actuator_fault) fault_flags |= ControllerFaultActuator;
  if (fault_flags != ControllerFaultNone) {
    reset_outputs();
    p_->controller_fault_flags = fault_flags;
    if (p_->motors_cb) p_->motors_cb(0.0, 0.0);
    publish_telemetry(pitch_rad, pitch_rate_rad_s, imu_age_s * 1000.0);
    return;
  }

  p_->controller_fault_flags = ControllerFaultNone;
  const bool controller_was_saturated =
      p_->command_saturated || p_->controller_saturation_flags != ControllerSaturationNone;
  p_->controller_saturation_flags = ControllerSaturationNone;

  p_->outer_elapsed_s += dt;
  if (p_->outer_elapsed_s + 1e-12 >= kVelocityLoopPeriodS) {
    const double outer_dt = p_->outer_elapsed_s;
    p_->outer_elapsed_s = 0.0;
    const double common_steps =
        0.5 * (static_cast<double>(p_->left_actual_steps) + static_cast<double>(p_->right_actual_steps));
    if (!p_->have_motor_feedback) {
      invalidate_velocity_observer();
    } else if (!p_->velocity_observer_seeded) {
      p_->previous_common_steps = common_steps;
      p_->previous_pitch_rad = pitch_rad;
      p_->velocity_observer_seeded = true;
    } else {
      const double delta_q_steps = common_steps - p_->previous_common_steps;
      const double delta_pitch_rad =
          rate_controller_detail::wrap_angle_delta(pitch_rad - p_->previous_pitch_rad);
      // Counter resets are not vehicle motion.  A plausible outer-loop delta
      // is comfortably below this bound even at the motor rail.
      const double reset_limit_steps = std::max(64.0, 4.0 * kMaxSps * outer_dt);
      if (std::abs(delta_q_steps) > reset_limit_steps) {
        invalidate_velocity_observer();
        p_->previous_common_steps = common_steps;
        p_->previous_pitch_rad = pitch_rad;
        p_->velocity_observer_seeded = true;
      } else {
        const double raw_velocity_sps = delta_q_steps / outer_dt;
        const double corrected_velocity_sps =
            (delta_q_steps + kStepsPerRad * delta_pitch_rad) / outer_dt;
        const double alpha = std::exp(-2.0 * M_PI * Config::fc_velocity_hz * outer_dt);
        p_->raw_completed_velocity_sps = raw_velocity_sps;
        p_->filtered_raw_completed_velocity_sps =
            alpha * p_->filtered_raw_completed_velocity_sps + (1.0 - alpha) * raw_velocity_sps;
        p_->corrected_axle_velocity_sps =
            alpha * p_->corrected_axle_velocity_sps + (1.0 - alpha) * corrected_velocity_sps;
        p_->previous_common_steps = common_steps;
        p_->previous_pitch_rad = pitch_rad;
      }
    }

    const double corrected_velocity_mps = p_->corrected_axle_velocity_sps * Config::meters_per_step;
    double requested_nominal_acceleration_mps2 =
        forward_command * ConfigPid::drive_max_acceleration_mps2;
    const double speed_limit_sps = std::max(1.0, ConfigPid::drive_max_sps);
    if (requested_nominal_acceleration_mps2 * p_->corrected_axle_velocity_sps > 0.0) {
      const double taper_start_sps = 0.75 * speed_limit_sps;
      const double taper = std::clamp((speed_limit_sps - std::abs(p_->corrected_axle_velocity_sps)) /
                                          (speed_limit_sps - taper_start_sps),
                                      0.0, 1.0);
      requested_nominal_acceleration_mps2 *= taper;
    }
    p_->nominal_acceleration_mps2 = move_toward(
        p_->nominal_acceleration_mps2, requested_nominal_acceleration_mps2,
        kDriveMaxJerkMps3 * outer_dt);
    p_->velocity_damping_acceleration_mps2 =
        -ConfigPid::velocity_damping_per_s * corrected_velocity_mps;

    // This integral is only the stationary physical COM trim. At neutral it
    // must remain active while correcting drift: qualifying it on already-low
    // speed, pitch, or pitch rate creates a self-sustaining moving equilibrium.
    // Command tracking has no integral state, so motion cannot become delayed
    // stored pitch.
    const bool com_trim_learning = forward_command == 0.0 &&
                                   p_->nominal_acceleration_mps2 == 0.0 &&
                                   !controller_was_saturated;
    const double integral_limit_rad = ConfigPid::velocity_I_limit_deg * M_PI / 180.0;
    const double integral_delta =
        com_trim_learning
            ? -ConfigPid::velocity_I * p_->corrected_axle_velocity_sps * outer_dt * M_PI / 180.0
            : 0.0;
    const double candidate_i =
        std::clamp(p_->com_trim_rad + integral_delta, -integral_limit_rad, integral_limit_rad);
    const double commanded_acceleration_mps2 =
        p_->nominal_acceleration_mps2 + p_->velocity_damping_acceleration_mps2;
    const double acceleration_pitch_rad = std::atan2(commanded_acceleration_mps2, Config::g0);
    const double candidate_pitch =
        acceleration_pitch_rad + candidate_i;
    const bool pitch_would_saturate = std::abs(candidate_pitch) > pitch_limit_rad;
    if (com_trim_learning && !pitch_would_saturate) {
      p_->com_trim_rad = candidate_i;
    }
    p_->pitch_setpoint_rad = std::clamp(
        acceleration_pitch_rad + p_->com_trim_rad,
        -pitch_limit_rad, pitch_limit_rad);
  }
  if (std::abs(std::atan2(p_->nominal_acceleration_mps2 +
                               p_->velocity_damping_acceleration_mps2,
                           Config::g0) +
               p_->com_trim_rad) > pitch_limit_rad) {
    p_->controller_saturation_flags |= ControllerSaturationPitch;
  }

  const double rate_sp_raw_rad_s = ConfigPid::angle_P * (p_->pitch_setpoint_rad - pitch_rad) -
                                   ConfigPid::angle_D * pitch_rate_rad_s;
  const double pitch_rate_max_rad_s = ConfigPid::pitch_rate_max_sps / kStepsPerRad;
  const double rate_sp_rad_s =
      std::clamp(rate_sp_raw_rad_s, -pitch_rate_max_rad_s, pitch_rate_max_rad_s);
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

  // The outer loop requests motion through pitch. The rate loop alone allocates
  // the wheel command, avoiding a second direct-speed term that can cancel its
  // balance correction at the motor boundary.
  // A normalized motor command of 1.0 maps to one wheel revolution per second.
  const double raw_balance_sps = -static_cast<double>(u(1)) * Config::steps_per_rev;
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

void RateControllerCore::clearImu() {
  p_->latest_imu = ImuSample{};
  p_->have_imu = false;
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
