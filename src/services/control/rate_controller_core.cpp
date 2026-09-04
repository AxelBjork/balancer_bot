#include "rate_controller_core.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

#include "services/main/config.h"
#include "services/control/velocity_reference_planner.h"

namespace rate_controller_detail {
double wrap_angle_delta(double angle_rad) {
  return std::remainder(angle_rad, 2.0 * M_PI);
}
}  // namespace rate_controller_detail

namespace {

constexpr double kMaxImuAgeS = 0.030;
constexpr double kMaxImuFutureS = 0.002;
constexpr double kFalloverRearmPitchRad = 10.0 * M_PI / 180.0;
constexpr double kFalloverRearmRateRadS = 30.0 * M_PI / 180.0;
constexpr double kFallenRecoveryStartMinPitchRad = 60.0 * M_PI / 180.0;
constexpr double kFallenRecoveryStartMaxPitchRad = 70.0 * M_PI / 180.0;
constexpr double kFallenRecoveryOutwardRateToleranceRadS = 5.0 * M_PI / 180.0;
constexpr double kChassisRotationFeedforwardMinFieldCapSps = 32000.0;
constexpr double kPitchRateKinematicFeedforwardFraction = 0.88;
constexpr double kRateCorrectionBlendStartPitchRad = 5.0 * M_PI / 180.0;
constexpr double kRateCorrectionFullPitchRad = 10.0 * M_PI / 180.0;
constexpr double kRateCorrectionBlendStartRateRadS = 15.0 * M_PI / 180.0;
constexpr double kRateCorrectionFullRateRadS = 30.0 * M_PI / 180.0;
constexpr double kVelocityLoopPeriodS = 1.0 / 100.0;
// COM trim is a physical-bias estimate, so convergence is judged from a
// quiet equilibrium rather than from elapsed time alone.  These thresholds
// are deliberately conservative enough to distinguish a stopped robot from
// the hundreds-of-SPS motion seen in the hardware captures.
// The quiet detector uses a low-frequency RMS metric rather than an
// instantaneous gyro threshold. This lets the harmless residual ~33 Hz
// mode pass through a static-bias decision without changing attitude feedback.
constexpr double kComTrimQuietRateMetricLpfHz = 2.0;
constexpr double kComTrimQuietRateRmsDps = 10.0;
// These thresholds represent physical motion, while the observer interface
// remains in motor SPS.  The verified 1/32 geometry doubles the reported SPS
// for the same wheel speed compared with the retired 1/16 configuration.
constexpr double kComTrimMotionVelocityEnterSps = 300.0;
constexpr double kComTrimMotionVelocityExitSps = 200.0;
constexpr double kComTrimUntrustedBiasVelocityMaxSps = 500.0;
constexpr double kComTrimMotionRateEnterDps = 15.0;
constexpr double kComTrimMotionRateExitDps = 8.0;
constexpr double kComTrimMotionPitchErrorEnterDeg = 2.0;
constexpr double kComTrimQuietPitchErrorDeg = 1.0;
constexpr double kComTrimUntrustedBiasPitchErrorMaxDeg = 3.0;
// The derivative is the optional adaptive COM gain multiplied by the SI
// velocity estimate.
constexpr double kComTrimQuietDerivativeDegPerS = 0.20;
constexpr double kComTrimQuietDwellS = 2.0;
// The equilibrium candidate is the measured body angle after removing the
// currently commanded drive and velocity contributions.  It is deliberately
// much slower than the velocity-control path: this is a static-bias witness,
// not another feedback path.  Trust requires the low-pass estimate to have
// caught up with the quiet candidate and then remain stable for a full dwell.
constexpr double kComTrimEquilibriumCandidateLpfHz = 0.5;
constexpr double kComTrimEquilibriumLpfHz = 0.25;
constexpr double kComTrimEquilibriumResidualDeg = 0.08;
constexpr double kComTrimEquilibriumRateDegPerS = 0.05;
constexpr double kComTrimEquilibriumConvergenceDwellS = 1.0;
double normalized_forward_command(double command) {
  const double magnitude = std::abs(command);
  if (magnitude <= Config::deadzone) return 0.0;
  const double normalized =
      std::clamp((magnitude - Config::deadzone) / (1.0 - Config::deadzone), 0.0, 1.0);
  return std::copysign(normalized, command);
}

double outer_pitch_limit_rad() {
  return std::clamp(ConfigPid::values.outer_pitch_limit_deg, 0.0,
                   Config::max_motion_pitch_setpoint_deg) * M_PI / 180.0;
}

constexpr double kStepsPerRad = Config::steps_per_rev / (2.0 * M_PI);

double chassis_rotation_field_feedforward_gain(
    double field_cap_sps, double absolute_pitch_rad,
    double absolute_pitch_rate_rad_s, bool recovery_envelope_requested) {
  // The stepper field is expressed in the rotating chassis frame. During a
  // high-authority held maneuver, feed its known chassis-rotation component
  // forward instead of disguising that coordinate conversion as a retuned
  // pitch-rate gain. Low-authority reference configurations never enter this
  // high-field coordinate regime and retain their configured controller.
  if (!recovery_envelope_requested ||
      field_cap_sps < kChassisRotationFeedforwardMinFieldCapSps) {
    return 0.0;
  }
  const double pitch_blend = std::clamp(
      (absolute_pitch_rad - kRateCorrectionBlendStartPitchRad) /
          (kRateCorrectionFullPitchRad - kRateCorrectionBlendStartPitchRad),
      0.0, 1.0);
  const double rate_blend = std::clamp(
      (absolute_pitch_rate_rad_s - kRateCorrectionBlendStartRateRadS) /
          (kRateCorrectionFullRateRadS - kRateCorrectionBlendStartRateRadS),
      0.0, 1.0);
  const double recovery_envelope_blend = std::max(pitch_blend, rate_blend);
  return -recovery_envelope_blend *
         kPitchRateKinematicFeedforwardFraction * kStepsPerRad;
}

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
  uint64_t pid_generation{0};

  int64_t left_actual_steps{0};
  int64_t right_actual_steps{0};
  bool have_motor_feedback{false};
  bool velocity_observer_seeded{false};
  double previous_common_steps{0.0};
  double previous_pitch_rad{0.0};
  double raw_completed_velocity_sps{0.0};
  double corrected_axle_velocity_sps{0.0};
  double filtered_raw_completed_velocity_sps{0.0};
  double completed_step_acceleration_sps2{0.0};
  double velocity_control_sps{0.0};
  bool velocity_feedback_valid{false};
  bool velocity_feedback_active{false};
  double user_velocity_mps{0.0};
  double reference_velocity_mps{0.0};
  double velocity_feedback_reference_mps{0.0};
  double filtered_reference_velocity_mps{0.0};
  bool reference_velocity_filter_initialized{false};
  double reference_acceleration_mps2{0.0};
  double reference_jerk_mps3{0.0};
  double velocity_feedback_estimate_mps{0.0};
  double velocity_error_mps{0.0};
  double velocity_feedback_acceleration_mps2{0.0};
  double velocity_p_acceleration_mps2{0.0};
  double velocity_i_acceleration_mps2{0.0};
  double velocity_integral_state_mps_s{0.0};
  bool velocity_integral_limited{false};
  bool velocity_anti_windup_active{false};
  double acceleration_raw_mps2{0.0};
  double acceleration_cmd_mps2{0.0};
  double drive_pitch_target_rad{0.0};
  bool outer_acceleration_limited{false};
  bool outer_pitch_target_limited{false};
  bool planner_acceleration_limited{false};
  bool planner_jerk_limited{false};
  VelocityReferencePlanner velocity_planner;
  double nominal_acceleration_mps2{0.0};
  double velocity_damping_acceleration_mps2{0.0};
  double com_trim_rad{0.0};
  bool com_trim_acquired{false};
  double com_trim_quiet_elapsed_s{0.0};
  double com_trim_equilibrium_candidate_rad{0.0};
  double com_trim_equilibrium_pitch_rad{0.0};
  double com_trim_equilibrium_previous_rad{0.0};
  double com_trim_equilibrium_stable_elapsed_s{0.0};
  bool com_trim_equilibrium_candidate_seeded{false};
  bool com_trim_equilibrium_seeded{false};
  double trim_quiet_rate_squared_dps2{0.0};
  bool trim_motion_active{false};
  bool trim_learning_allowed{false};
  bool trim_learning_enabled{false};
  uint8_t trim_learning_block_reason{ComTrimLearningBlockFault};
  double velocity_pitch_request_unclamped_rad{0.0};
  double velocity_pitch_request_limited_rad{0.0};
  bool velocity_authority_limited{false};
  double pitch_target_unclamped_rad{0.0};
  uint8_t pitch_target_limit_reason{PitchTargetLimitNone};
  double pitch_setpoint_rad{0.0};
  double turn_sps{0.0};
  double last_u_sps{0.0};
  double outer_elapsed_s{0.0};
  bool command_saturated{false};
  bool balance_saturated_positive{false};
  bool balance_saturated_negative{false};
  bool actuator_fault{false};
  uint32_t controller_fault_flags{ControllerFaultNone};
  uint32_t controller_saturation_flags{ControllerSaturationNone};
  double pitch_feedback_sps{0.0};
  double pitch_rate_feedback_sps{0.0};
  double pitch_accel_feedback_sps{0.0};
  double drive_feedforward_sps{0.0};
  double balance_correction_sps{0.0};
  double common_unclamped_sps{0.0};
  double balance_unclamped_sps{0.0};
  bool matched_reference_filter_enabled{true};
  bool simulation_drive_feedforward_enabled{true};
  bool simulation_controller_enabled{true};
};

RateControllerCore::RateControllerCore() : p_(new Impl) {
  applyPidConfig();
}

void RateControllerCore::applyPidConfig() {
  const double trim_limit_rad =
      std::max(0.0, ConfigPid::values.adaptive_com_trim_limit_deg) * M_PI / 180.0;
  p_->com_trim_rad = std::clamp(p_->com_trim_rad, -trim_limit_rad, trim_limit_rad);
  if (ConfigPid::values.adaptive_com_trim_enabled < 0.5) {
    p_->com_trim_rad = 0.0;
    p_->com_trim_acquired = false;
  }
  // A configuration reload changes the outer-loop state-space. Start the
  // reference and integral states from zero so a stale command or integral
  // cannot become a reload-induced acceleration kick.
  p_->velocity_planner.reset();
  p_->reference_velocity_mps = 0.0;
  p_->velocity_feedback_reference_mps = 0.0;
  p_->filtered_reference_velocity_mps = 0.0;
  p_->reference_velocity_filter_initialized = false;
  p_->reference_acceleration_mps2 = 0.0;
  p_->reference_jerk_mps3 = 0.0;
  p_->velocity_integral_state_mps_s = 0.0;
  p_->velocity_p_acceleration_mps2 = 0.0;
  p_->velocity_i_acceleration_mps2 = 0.0;
  p_->velocity_integral_limited = false;
  p_->velocity_anti_windup_active = false;
  p_->pid_generation = ConfigPid::generation();
}

RateControllerCore::~RateControllerCore() {
  stop();
  delete p_;
}

void RateControllerCore::setSimulationOuterLoopOptions(
    bool endpoint_continuity_enabled, bool matched_reference_filter_enabled) {
  p_->velocity_planner.setEndpointContinuityEnabled(endpoint_continuity_enabled);
  p_->matched_reference_filter_enabled = matched_reference_filter_enabled;
  p_->filtered_reference_velocity_mps = 0.0;
  p_->velocity_feedback_reference_mps = 0.0;
  p_->reference_velocity_filter_initialized = false;
}

void RateControllerCore::setSimulationDriveFeedforwardEnabled(bool enabled) {
  p_->simulation_drive_feedforward_enabled = enabled;
}

void RateControllerCore::setSimulationControllerEnabled(bool enabled) {
  p_->simulation_controller_enabled = enabled;
  if (!enabled) p_->balance_armed = false;
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
  if (p_->pid_generation != ConfigPid::generation()) {
    applyPidConfig();
  }
  const double dt = std::clamp(dt_s, 1.0 / 2000.0, 0.05);
  if (!p_->initialized) {
    p_->initialized = true;
    p_->start_ts = now;
  }

  const auto invalidate_velocity_observer = [this]() {
    p_->velocity_observer_seeded = false;
    p_->velocity_feedback_valid = false;
    p_->velocity_feedback_active = false;
    p_->raw_completed_velocity_sps = 0.0;
    p_->corrected_axle_velocity_sps = 0.0;
    p_->filtered_raw_completed_velocity_sps = 0.0;
    p_->completed_step_acceleration_sps2 = 0.0;
    p_->velocity_control_sps = 0.0;
    p_->velocity_feedback_reference_mps = 0.0;
    p_->filtered_reference_velocity_mps = 0.0;
    p_->reference_velocity_filter_initialized = false;
    p_->velocity_feedback_estimate_mps = 0.0;
    p_->velocity_error_mps = 0.0;
    p_->velocity_feedback_acceleration_mps2 = 0.0;
    p_->velocity_p_acceleration_mps2 = 0.0;
    p_->velocity_i_acceleration_mps2 = 0.0;
    p_->velocity_integral_state_mps_s = 0.0;
    p_->velocity_integral_limited = false;
    p_->velocity_anti_windup_active = false;
  };
  const auto reset_outputs = [this, &invalidate_velocity_observer]() {
    p_->velocity_planner.reset();
    p_->user_velocity_mps = 0.0;
    p_->reference_velocity_mps = 0.0;
    p_->velocity_feedback_reference_mps = 0.0;
    p_->filtered_reference_velocity_mps = 0.0;
    p_->reference_velocity_filter_initialized = false;
    p_->reference_acceleration_mps2 = 0.0;
    p_->reference_jerk_mps3 = 0.0;
    p_->acceleration_raw_mps2 = 0.0;
    p_->acceleration_cmd_mps2 = 0.0;
    p_->drive_pitch_target_rad = 0.0;
    p_->outer_acceleration_limited = false;
    p_->outer_pitch_target_limited = false;
    p_->planner_acceleration_limited = false;
    p_->planner_jerk_limited = false;
    p_->nominal_acceleration_mps2 = 0.0;
    p_->velocity_damping_acceleration_mps2 = 0.0;
    invalidate_velocity_observer();
    // com_trim_rad is the optional bounded adaptive trim. Preserve it across
    // a transient fault/fallover; only dynamic command and balance state reset.
    p_->pitch_setpoint_rad = 0.0;
    p_->turn_sps = 0.0;
    p_->last_u_sps = 0.0;
    p_->outer_elapsed_s = 0.0;
    p_->com_trim_quiet_elapsed_s = 0.0;
    p_->com_trim_equilibrium_candidate_rad = 0.0;
    p_->com_trim_equilibrium_pitch_rad = 0.0;
    p_->com_trim_equilibrium_previous_rad = 0.0;
    p_->com_trim_equilibrium_stable_elapsed_s = 0.0;
    p_->com_trim_equilibrium_candidate_seeded = false;
    p_->com_trim_equilibrium_seeded = false;
    p_->trim_quiet_rate_squared_dps2 = 0.0;
    p_->trim_motion_active = true;
    p_->trim_learning_allowed = false;
    p_->trim_learning_enabled = false;
    p_->trim_learning_block_reason = ComTrimLearningBlockFault;
    p_->velocity_pitch_request_unclamped_rad = 0.0;
    p_->velocity_pitch_request_limited_rad = 0.0;
    p_->velocity_authority_limited = false;
    p_->pitch_target_unclamped_rad = 0.0;
    p_->pitch_target_limit_reason = PitchTargetLimitNone;
    p_->command_saturated = false;
    p_->balance_saturated_positive = false;
    p_->balance_saturated_negative = false;
    p_->controller_saturation_flags = ControllerSaturationNone;
    p_->pitch_feedback_sps = 0.0;
    p_->pitch_rate_feedback_sps = 0.0;
    p_->pitch_accel_feedback_sps = 0.0;
    p_->drive_feedforward_sps = 0.0;
    p_->balance_correction_sps = 0.0;
    p_->common_unclamped_sps = 0.0;
    p_->balance_unclamped_sps = 0.0;
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
    // The old acceleration/damping aliases remain zero and deprecated. The
    // canonical SI fields below are the only authoritative outer-loop data.
    t.raw_completed_velocity_sps = p_->filtered_raw_completed_velocity_sps;
    t.completed_step_acceleration_sps2 = p_->completed_step_acceleration_sps2;
    t.corrected_axle_velocity_sps = p_->corrected_axle_velocity_sps;
    t.velocity_control_sps = p_->velocity_control_sps;
    const double telemetry_trim_rad =
        ConfigPid::values.fixed_com_trim_deg * M_PI / 180.0 +
        (ConfigPid::values.adaptive_com_trim_enabled >= 0.5 ? p_->com_trim_rad : 0.0);
    t.com_trim_deg = telemetry_trim_rad * 180.0 / M_PI;
    t.user_velocity_mps = p_->user_velocity_mps;
    t.reference_velocity_mps = p_->reference_velocity_mps;
    t.velocity_feedback_reference_mps = p_->velocity_feedback_reference_mps;
    t.reference_acceleration_mps2 = p_->reference_acceleration_mps2;
    t.reference_jerk_mps3 = p_->reference_jerk_mps3;
    t.velocity_feedback_estimate_mps = p_->velocity_feedback_estimate_mps;
    t.velocity_error_mps = p_->velocity_error_mps;
    t.velocity_feedback_acceleration_mps2 = p_->velocity_feedback_acceleration_mps2;
    t.velocity_p_acceleration_mps2 = p_->velocity_p_acceleration_mps2;
    t.velocity_i_acceleration_mps2 = p_->velocity_i_acceleration_mps2;
    t.velocity_integral_state_mps_s = p_->velocity_integral_state_mps_s;
    t.velocity_integral_limited = p_->velocity_integral_limited;
    t.velocity_anti_windup_active = p_->velocity_anti_windup_active;
    t.acceleration_raw_mps2 = p_->acceleration_raw_mps2;
    t.acceleration_cmd_mps2 = p_->acceleration_cmd_mps2;
    t.drive_pitch_target_deg = p_->drive_pitch_target_rad * 180.0 / M_PI;
    t.fixed_com_trim_deg = ConfigPid::values.fixed_com_trim_deg;
    t.velocity_feedback_valid = p_->velocity_feedback_valid;
    t.velocity_feedback_active = p_->velocity_feedback_active;
    t.outer_acceleration_limited = p_->outer_acceleration_limited;
    t.outer_pitch_target_limited = p_->outer_pitch_target_limited;
    t.active_drive_max_velocity_mps = ConfigPid::values.drive_max_velocity_mps;
    t.active_drive_max_acceleration_mps2 = 0.0;
    t.active_drive_max_deceleration_mps2 = 0.0;
    t.active_planner_max_acceleration_mps2 =
        ConfigPid::values.planner_max_acceleration_mps2;
    t.active_planner_max_deceleration_mps2 =
        ConfigPid::values.planner_max_deceleration_mps2;
    t.active_planner_max_jerk_mps3 = ConfigPid::values.planner_max_jerk_mps3;
    t.active_velocity_gain_per_s = ConfigPid::values.velocity_gain_per_s;
    t.active_velocity_i_gain_per_s2 = ConfigPid::values.velocity_i_gain_per_s2;
    t.active_velocity_i_leak_time_s = ConfigPid::values.velocity_i_leak_time_s;
    t.active_velocity_i_acceleration_limit_mps2 =
        ConfigPid::values.velocity_i_acceleration_limit_mps2;
    t.active_velocity_feedback_cutoff_hz = ConfigPid::values.velocity_feedback_cutoff_hz;
    t.active_outer_pitch_limit_deg = ConfigPid::values.outer_pitch_limit_deg;
    t.active_fixed_com_trim_deg = ConfigPid::values.fixed_com_trim_deg;
    t.adaptive_com_trim_enabled = ConfigPid::values.adaptive_com_trim_enabled >= 0.5;
    t.legacy_outer_fields_valid = false;
    t.planner_acceleration_limited = p_->planner_acceleration_limited;
    t.planner_jerk_limited = p_->planner_jerk_limited;
    t.final_pitch_target_deg = p_->pitch_setpoint_rad * 180.0 / M_PI;
    t.pitch_error_deg = (p_->pitch_setpoint_rad - pitch_rad) * 180.0 / M_PI;
    t.pitch_sp_deg = p_->pitch_setpoint_rad * 180.0 / M_PI;
    t.command_saturated = p_->command_saturated;
    t.actuator_fault = p_->actuator_fault;
    t.controller_fault_flags = p_->controller_fault_flags;
    t.controller_saturation_flags = p_->controller_saturation_flags;
    t.trim_learning_enabled = p_->trim_learning_enabled;
    t.trim_learning_block_reason = p_->trim_learning_block_reason;
    t.pitch_feedback_sps = p_->pitch_feedback_sps;
    t.pitch_rate_feedback_sps = p_->pitch_rate_feedback_sps;
    t.pitch_accel_feedback_sps = p_->pitch_accel_feedback_sps;
    t.drive_feedforward_sps = p_->drive_feedforward_sps;
    t.balance_correction_sps = p_->balance_correction_sps;
    t.common_unclamped_sps = p_->common_unclamped_sps;
    t.velocity_pitch_target_deg = 0.0;
    t.balance_unclamped_sps = p_->balance_unclamped_sps;
    const bool recovery_envelope_requested =
        std::isfinite(p_->latest_joy.forward) &&
        normalized_forward_command(p_->latest_joy.forward) != 0.0;
    t.active_pitch_gain_sps_per_rad = ConfigPid::values.pitch_gain;
    t.active_pitch_rate_gain_sps_per_rad_s =
        ConfigPid::values.pitch_rate_gain +
        chassis_rotation_field_feedforward_gain(
            ConfigPid::values.balance_max_sps, std::abs(pitch_rad),
            std::abs(pitch_rate_rad_s), recovery_envelope_requested);
    t.active_pitch_accel_gain_sps_per_rad_s2 = ConfigPid::values.pitch_accel_gain;
    t.active_velocity_pitch_gain_rad_per_sps = 0.0;
    t.active_velocity_control_cutoff_hz = 0.0;
    t.active_velocity_observer_cutoff_hz = Config::fc_velocity_hz;
    t.active_com_trim_gain_deg_per_sps_s = 0.0;
    t.active_com_trim_limit_deg = 0.0;
    t.active_velocity_pitch_limit_deg = 0.0;
    t.active_accel_lpf_hz = Config::imu_accel_lpf_hz;
    t.active_gyro_derivative_lpf_hz = Config::imu_gyro_derivative_lpf_hz;
    t.active_config_generation = ConfigPid::generation();
    t.velocity_pitch_request_unclamped_deg = 0.0;
    t.velocity_pitch_request_limited_deg = 0.0;
    t.pitch_target_unclamped_deg = p_->pitch_target_unclamped_rad * 180.0 / M_PI;
    // This field belonged to the removed damping/pitch outer loop. Keep it as
    // an append-only compatibility field, but do not give it new semantics.
    t.velocity_authority_limited = false;
    t.trim_trusted = p_->com_trim_acquired;
    t.trim_learning_allowed = p_->trim_learning_allowed;
    t.trim_quiet_rate_rms_dps = std::sqrt(std::max(0.0, p_->trim_quiet_rate_squared_dps2));
    t.pitch_target_limit_reason = p_->pitch_target_limit_reason;
    p_->tel_cb(std::move(t));
  };

  if (!p_->have_imu) {
    reset_outputs();
    p_->controller_fault_flags = ControllerFaultNoImu;
    if (p_->motors_cb) p_->motors_cb(0.0, 0.0);
    publish_telemetry(0.0, 0.0, 0.0);
    return;
  }

  // The braced fallen-state recovery fixture is intentionally inert until a
  // fresh recovery command enables it. This is simulator-only; production
  // never calls setSimulationControllerEnabled(). Once enabled, the normal
  // fallover shutdown/re-arm logic below remains authoritative.
  if (!p_->simulation_controller_enabled) {
    p_->balance_armed = false;
    reset_outputs();
    p_->controller_fault_flags = ControllerFaultNone;
    if (p_->motors_cb) p_->motors_cb(0.0, 0.0);
    publish_telemetry(p_->latest_imu.angle_rad, p_->latest_imu.gyro_rad_s, 0.0);
    return;
  }

  const double pitch_rad = p_->latest_imu.angle_rad;
  const double pitch_rate_rad_s = p_->latest_imu.gyro_rad_s;
  const double pitch_accel_rad_s2 = p_->latest_imu.pitch_accel_rad_s2;
  const JoyCmd joy = p_->latest_joy;
  const double forward_command =
      std::isfinite(joy.forward) ? normalized_forward_command(joy.forward) : 0.0;
  const bool recovery_envelope_requested = forward_command != 0.0;
  const double chassis_rotation_field_feedforward_gain_sps_per_rad_s =
      chassis_rotation_field_feedforward_gain(
          ConfigPid::values.balance_max_sps, std::abs(pitch_rad),
          std::abs(pitch_rate_rad_s), recovery_envelope_requested);
  const double active_pitch_rate_gain =
      ConfigPid::values.pitch_rate_gain +
      chassis_rotation_field_feedforward_gain_sps_per_rad_s;
  const auto feedback_terms = [pitch_rad, pitch_rate_rad_s, pitch_accel_rad_s2,
                               active_pitch_rate_gain](double pitch_target_rad) {
    // The IMU pitch/rate convention is positive forward, while a positive
    // wheel SPS command produces negative body angular acceleration at this
    // mechanical boundary.  Therefore the robot-forward motor command uses
    // the physical equivalent of negative feedback: positive pitch error,
    // positive pitch rate, and positive pitch acceleration all command the
    // wheels forward. The three terms remain independent in SPS/rad,
    // SPS/(rad/s), and SPS/(rad/s^2). The effective rate coefficient includes
    // only the explicit chassis-rotation field feedforward above.
    return std::array<double, 3>{
        ConfigPid::values.pitch_gain * (pitch_rad - pitch_target_rad),
        active_pitch_rate_gain * pitch_rate_rad_s,
        ConfigPid::values.pitch_accel_gain * pitch_accel_rad_s2,
    };
  };
  const double imu_age_s = std::chrono::duration<double>(now - p_->latest_imu.t).count();
  const double pitch_limit_rad = kMaxPitchSetpointRad;
  const double fallover_limit_rad = Config::fallover_shutdown_deg * M_PI / 180.0;
  uint32_t fault_flags = ControllerFaultNone;
  if (imu_age_s > kMaxImuAgeS) fault_flags |= ControllerFaultStaleImu;
  if (imu_age_s < -kMaxImuFutureS) fault_flags |= ControllerFaultFutureImu;
  const double absolute_pitch_rad = std::abs(pitch_rad);
  const bool recovery_command_direction_valid =
      forward_command != 0.0 && forward_command * pitch_rad > 0.0;
  const bool fallen_recovery_start_allowed =
      recovery_command_direction_valid &&
      absolute_pitch_rad >= kFallenRecoveryStartMinPitchRad &&
      absolute_pitch_rad <= kFallenRecoveryStartMaxPitchRad &&
      std::abs(pitch_rate_rad_s) <= kFalloverRearmRateRadS;
  const bool high_pitch_recovery_continuing =
      p_->balance_armed && recovery_command_direction_valid &&
      absolute_pitch_rad > fallover_limit_rad &&
      absolute_pitch_rad <= kFallenRecoveryStartMaxPitchRad &&
      pitch_rad * pitch_rate_rad_s <=
          absolute_pitch_rad * kFallenRecoveryOutwardRateToleranceRadS;
  const bool high_pitch_control_allowed =
      fallen_recovery_start_allowed || high_pitch_recovery_continuing;
  if (p_->balance_armed && absolute_pitch_rad > fallover_limit_rad &&
      !high_pitch_control_allowed) {
    p_->balance_armed = false;
  }
  const bool rearm_attitude_allowed =
      fallen_recovery_start_allowed || absolute_pitch_rad <= kFalloverRearmPitchRad;
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
        if (!p_->velocity_feedback_valid) {
          // The first valid delta initializes both filter stages. It is a
          // measurement, not an acceleration kick into the controller.
          p_->filtered_raw_completed_velocity_sps = raw_velocity_sps;
          p_->corrected_axle_velocity_sps = corrected_velocity_sps;
          p_->velocity_control_sps = corrected_velocity_sps;
          p_->completed_step_acceleration_sps2 = 0.0;
          p_->velocity_feedback_valid = true;
        } else {
          const double previous_filtered_velocity = p_->filtered_raw_completed_velocity_sps;
          p_->filtered_raw_completed_velocity_sps =
              alpha * p_->filtered_raw_completed_velocity_sps + (1.0 - alpha) * raw_velocity_sps;
          p_->completed_step_acceleration_sps2 =
              (p_->filtered_raw_completed_velocity_sps - previous_filtered_velocity) / outer_dt;
          p_->corrected_axle_velocity_sps =
              alpha * p_->corrected_axle_velocity_sps + (1.0 - alpha) * corrected_velocity_sps;
        }
        p_->previous_common_steps = common_steps;
        p_->previous_pitch_rad = pitch_rad;
      }
    }

    {
      const double control_cutoff_hz =
          std::max(0.001, ConfigPid::values.velocity_feedback_cutoff_hz);
      if (std::abs(control_cutoff_hz - Config::fc_velocity_hz) < 1e-9) {
        p_->velocity_control_sps = p_->corrected_axle_velocity_sps;
      } else if (p_->velocity_feedback_valid) {
        const double control_alpha = std::exp(-2.0 * M_PI * control_cutoff_hz * outer_dt);
        p_->velocity_control_sps =
            control_alpha * p_->velocity_control_sps +
            (1.0 - control_alpha) * p_->corrected_axle_velocity_sps;
      }

      const VelocityReferenceState reference = p_->velocity_planner.update(
          forward_command * ConfigPid::values.drive_max_velocity_mps, outer_dt,
          ConfigPid::values.planner_max_acceleration_mps2,
          ConfigPid::values.planner_max_deceleration_mps2,
          ConfigPid::values.planner_max_jerk_mps3);
      p_->user_velocity_mps = reference.user_velocity_mps;
      p_->reference_velocity_mps = reference.reference_velocity_mps;
      p_->reference_acceleration_mps2 = reference.reference_acceleration_mps2;
      p_->reference_jerk_mps3 = reference.reference_jerk_mps3;
      p_->planner_acceleration_limited = reference.acceleration_limited;
      p_->planner_jerk_limited = reference.jerk_limited;
      p_->velocity_feedback_estimate_mps =
          p_->velocity_control_sps * Config::meters_per_step;
      const double reference_measurement_alpha =
          std::exp(-2.0 * M_PI * Config::fc_velocity_hz * outer_dt);
      const double reference_control_alpha =
          std::exp(-2.0 * M_PI * control_cutoff_hz * outer_dt);
      if (!p_->matched_reference_filter_enabled) {
        // The unfiltered comparison is retained only by the simulator A/B
        // fixture. The production path below uses the same two poles as the
        // measured velocity path.
        p_->filtered_reference_velocity_mps = p_->reference_velocity_mps;
        p_->velocity_feedback_reference_mps = p_->reference_velocity_mps;
        p_->reference_velocity_filter_initialized = true;
      } else if (!p_->reference_velocity_filter_initialized ||
                 !p_->velocity_feedback_valid) {
        // Seed on the same outer-loop sample as the measured observer. This
        // avoids manufacturing a P kick when a command is already active at
        // observer startup/revalidation.
        p_->filtered_reference_velocity_mps = p_->reference_velocity_mps;
        p_->velocity_feedback_reference_mps = p_->reference_velocity_mps;
        p_->reference_velocity_filter_initialized = p_->velocity_feedback_valid;
      } else {
        p_->filtered_reference_velocity_mps =
            reference_measurement_alpha * p_->filtered_reference_velocity_mps +
            (1.0 - reference_measurement_alpha) * p_->reference_velocity_mps;
        if (std::abs(control_cutoff_hz - Config::fc_velocity_hz) < 1e-9) {
          p_->velocity_feedback_reference_mps = p_->filtered_reference_velocity_mps;
        } else {
          p_->velocity_feedback_reference_mps =
              reference_control_alpha * p_->velocity_feedback_reference_mps +
              (1.0 - reference_control_alpha) * p_->filtered_reference_velocity_mps;
        }
      }
      p_->velocity_error_mps =
          p_->velocity_feedback_reference_mps - p_->velocity_feedback_estimate_mps;
      p_->velocity_feedback_active = p_->velocity_feedback_valid;
      p_->velocity_p_acceleration_mps2 =
          p_->velocity_feedback_active
              ? ConfigPid::values.velocity_gain_per_s * p_->velocity_error_mps
              : 0.0;

      // The integral state is in m/s*s. Use the exact first-order leaky
      // update so the result is stable for long outer-loop periods and does
      // not depend on Euler-step size. Integral accumulation is inhibited
      // when it would push the outer command farther into its authority
      // limit, or when the previous inner command was already saturated.
      p_->velocity_integral_limited = false;
      p_->velocity_anti_windup_active = false;
      if (!p_->velocity_feedback_active || ConfigPid::values.velocity_i_gain_per_s2 <= 0.0) {
        p_->velocity_integral_state_mps_s = 0.0;
        p_->velocity_i_acceleration_mps2 = 0.0;
      } else {
        const double integral_gain = ConfigPid::values.velocity_i_gain_per_s2;
        const double leak_time_s = std::max(1e-6, ConfigPid::values.velocity_i_leak_time_s);
        const double leak = std::exp(-outer_dt / leak_time_s);
        const double state_limit =
            ConfigPid::values.velocity_i_acceleration_limit_mps2 / integral_gain;
        const double previous_state = p_->velocity_integral_state_mps_s;
        double next_state = previous_state * leak +
                            p_->velocity_error_mps * leak_time_s * (1.0 - leak);
        if (std::abs(next_state) > state_limit) {
          next_state = std::copysign(state_limit, next_state);
          p_->velocity_integral_limited = true;
        }
        double next_i_acceleration = integral_gain * next_state;
        const double motion_pitch_limit_rad = outer_pitch_limit_rad();
        const double max_outer_acceleration_mps2 =
            Config::g0 * std::tan(std::min(motion_pitch_limit_rad, kMaxPitchSetpointRad));
        const double candidate_raw_acceleration =
            p_->reference_acceleration_mps2 + p_->velocity_p_acceleration_mps2 +
            next_i_acceleration;
        const bool integral_pushes_outer_limit =
            (candidate_raw_acceleration > max_outer_acceleration_mps2 &&
             p_->velocity_error_mps > 0.0) ||
            (candidate_raw_acceleration < -max_outer_acceleration_mps2 &&
             p_->velocity_error_mps < 0.0);
        const bool integral_pushes_inner_limit =
            controller_was_saturated && std::abs(p_->velocity_error_mps) > 1e-12;
        if (integral_pushes_outer_limit || integral_pushes_inner_limit) {
          next_state = previous_state * leak;
          next_state = std::clamp(next_state, -state_limit, state_limit);
          next_i_acceleration = integral_gain * next_state;
          p_->velocity_anti_windup_active = true;
        }
        p_->velocity_integral_state_mps_s = next_state;
        p_->velocity_i_acceleration_mps2 = std::clamp(
            next_i_acceleration,
            -ConfigPid::values.velocity_i_acceleration_limit_mps2,
            ConfigPid::values.velocity_i_acceleration_limit_mps2);
        p_->velocity_integral_limited =
            p_->velocity_integral_limited ||
            std::abs(p_->velocity_i_acceleration_mps2) >=
                ConfigPid::values.velocity_i_acceleration_limit_mps2 - 1e-12;
      }
      p_->velocity_feedback_acceleration_mps2 =
          p_->velocity_p_acceleration_mps2 + p_->velocity_i_acceleration_mps2;
      p_->acceleration_raw_mps2 =
          p_->reference_acceleration_mps2 + p_->velocity_feedback_acceleration_mps2;
      const double motion_pitch_limit_rad = outer_pitch_limit_rad();
      const double max_outer_acceleration_mps2 =
          Config::g0 * std::tan(std::min(motion_pitch_limit_rad, kMaxPitchSetpointRad));
      p_->acceleration_cmd_mps2 = std::clamp(
          p_->acceleration_raw_mps2, -max_outer_acceleration_mps2, max_outer_acceleration_mps2);
      p_->outer_acceleration_limited =
          std::abs(p_->acceleration_cmd_mps2 - p_->acceleration_raw_mps2) > 1e-12;
      p_->drive_pitch_target_rad = std::atan2(p_->acceleration_cmd_mps2, Config::g0);
      p_->outer_pitch_target_limited = p_->outer_acceleration_limited;
      p_->velocity_authority_limited = p_->outer_acceleration_limited;
      p_->nominal_acceleration_mps2 = 0.0;
      p_->velocity_damping_acceleration_mps2 = 0.0;
      p_->velocity_pitch_request_unclamped_rad = 0.0;
      p_->velocity_pitch_request_limited_rad = 0.0;

      // The optional learner is deliberately isolated from v1. When enabled
      // it uses the same quiet-state witness as before, but its trim is never
      // part of the default controller configuration or tuner search.
      const bool adaptive_trim_enabled =
          ConfigPid::values.adaptive_com_trim_enabled >= 0.5;
      const bool trim_base_conditions =
          p_->user_velocity_mps == 0.0 &&
          std::abs(p_->reference_acceleration_mps2) <= 1e-12;
      const double integral_limit_rad =
          ConfigPid::values.adaptive_com_trim_limit_deg * M_PI / 180.0;
      const double unrestricted_integral_delta =
          -ConfigPid::values.adaptive_com_trim_gain_deg_per_mps_s *
          p_->velocity_feedback_estimate_mps * outer_dt * M_PI / 180.0;
      const double trim_derivative_deg_per_s =
          std::abs(ConfigPid::values.adaptive_com_trim_gain_deg_per_mps_s *
                   p_->velocity_feedback_estimate_mps);

    // Estimate low-frequency rate energy for the state machine only.  The
    // squared-rate envelope avoids repeated instantaneous threshold crossings
    // from the locked ~33 Hz attitude residual while retaining sensitivity to
    // the 0.15--0.25 Hz motion seen in the hardware failure.
      const double quiet_rate_alpha =
          std::exp(-2.0 * M_PI * kComTrimQuietRateMetricLpfHz * outer_dt);
      const double pitch_rate_dps = pitch_rate_rad_s * 180.0 / M_PI;
      p_->trim_quiet_rate_squared_dps2 =
          quiet_rate_alpha * p_->trim_quiet_rate_squared_dps2 +
          (1.0 - quiet_rate_alpha) * pitch_rate_dps * pitch_rate_dps;
      const double trim_quiet_rate_rms_dps =
          std::sqrt(std::max(0.0, p_->trim_quiet_rate_squared_dps2));

      const double candidate_pitch_without_trim = p_->drive_pitch_target_rad;
      const double configured_fixed_trim_rad =
          ConfigPid::values.fixed_com_trim_deg * M_PI / 180.0;
      const double candidate_pitch = candidate_pitch_without_trim + configured_fixed_trim_rad +
                                     (adaptive_trim_enabled ? p_->com_trim_rad : 0.0);
      const bool pitch_would_saturate = std::abs(candidate_pitch) > kMaxPitchSetpointRad;
      const double trim_balance_limit_sps =
          std::clamp(ConfigPid::values.balance_max_sps, 0.0, kMaxSps);
      const auto candidate_terms = feedback_terms(candidate_pitch);
      const bool balance_would_saturate =
          std::abs(candidate_terms[0] + candidate_terms[1] + candidate_terms[2]) >
          trim_balance_limit_sps;
    const bool previous_saturation_blocked = controller_was_saturated;
    const bool trim_saturation_blocked =
        previous_saturation_blocked || p_->velocity_authority_limited ||
        pitch_would_saturate || balance_would_saturate;

    const double pitch_error_deg =
        std::abs(p_->pitch_setpoint_rad - pitch_rad) * 180.0 / M_PI;
      const bool quiet_equilibrium_sample =
          trim_base_conditions &&
          std::abs(p_->velocity_control_sps) <= kComTrimMotionVelocityExitSps &&
          trim_quiet_rate_rms_dps <= kComTrimQuietRateRmsDps &&
          pitch_error_deg <= kComTrimQuietPitchErrorDeg &&
          trim_derivative_deg_per_s <= kComTrimQuietDerivativeDegPerS &&
          !trim_saturation_blocked;

    // Keep this estimate independent of the learned trim itself.  At a quiet
    // equilibrium, actual pitch minus the drive/velocity command is the
    // observable angle that the acquisition state must see settle.  Motion
    // does not update the estimate and clears only its convergence dwell; a
    // later quiet period must prove convergence again if the candidate moved.
      const double raw_equilibrium_candidate_rad = rate_controller_detail::wrap_angle_delta(
          pitch_rad - candidate_pitch_without_trim - configured_fixed_trim_rad);
    if (!p_->com_trim_equilibrium_candidate_seeded) {
      p_->com_trim_equilibrium_candidate_rad = raw_equilibrium_candidate_rad;
      p_->com_trim_equilibrium_candidate_seeded = true;
    } else {
      const double candidate_alpha =
          std::exp(-2.0 * M_PI * kComTrimEquilibriumCandidateLpfHz * outer_dt);
      const double candidate_delta = rate_controller_detail::wrap_angle_delta(
          raw_equilibrium_candidate_rad - p_->com_trim_equilibrium_candidate_rad);
      p_->com_trim_equilibrium_candidate_rad = rate_controller_detail::wrap_angle_delta(
          p_->com_trim_equilibrium_candidate_rad + (1.0 - candidate_alpha) * candidate_delta);
    }
    const double equilibrium_candidate_rad = p_->com_trim_equilibrium_candidate_rad;
    if (!quiet_equilibrium_sample) {
      p_->com_trim_equilibrium_stable_elapsed_s = 0.0;
    } else if (!p_->com_trim_equilibrium_seeded) {
      p_->com_trim_equilibrium_pitch_rad = equilibrium_candidate_rad;
      p_->com_trim_equilibrium_previous_rad = equilibrium_candidate_rad;
      p_->com_trim_equilibrium_seeded = true;
      p_->com_trim_equilibrium_stable_elapsed_s = 0.0;
    } else {
      const double equilibrium_alpha =
          std::exp(-2.0 * M_PI * kComTrimEquilibriumLpfHz * outer_dt);
      const double previous_estimate = p_->com_trim_equilibrium_pitch_rad;
      const double estimate_delta = rate_controller_detail::wrap_angle_delta(
          equilibrium_candidate_rad - previous_estimate);
      p_->com_trim_equilibrium_pitch_rad = rate_controller_detail::wrap_angle_delta(
          previous_estimate + (1.0 - equilibrium_alpha) * estimate_delta);
      const double estimate_rate_deg_per_s =
          std::abs(rate_controller_detail::wrap_angle_delta(
                       p_->com_trim_equilibrium_pitch_rad -
                       p_->com_trim_equilibrium_previous_rad) /
                   outer_dt) *
          180.0 / M_PI;
      const double estimate_residual_deg =
          std::abs(rate_controller_detail::wrap_angle_delta(
                       equilibrium_candidate_rad - p_->com_trim_equilibrium_pitch_rad)) *
          180.0 / M_PI;
      if (estimate_rate_deg_per_s <= kComTrimEquilibriumRateDegPerS &&
          estimate_residual_deg <= kComTrimEquilibriumResidualDeg) {
        p_->com_trim_equilibrium_stable_elapsed_s += outer_dt;
      } else {
        p_->com_trim_equilibrium_stable_elapsed_s = 0.0;
      }
      p_->com_trim_equilibrium_previous_rad = p_->com_trim_equilibrium_pitch_rad;
    }
    const bool quiet_equilibrium_converged =
        p_->com_trim_equilibrium_seeded &&
        p_->com_trim_equilibrium_stable_elapsed_s >=
            kComTrimEquilibriumConvergenceDwellS;
    const bool quiet_for_dwell = quiet_equilibrium_sample;

    // A larger static COM bias can create a low-rate, nearly constant lean
    // before enough trim has been learned. Permit that bounded untrusted
    // acquisition candidate to reduce the bias; ordinary commands, fast
    // motion, authority limiting, saturation, and an already-entered motion
    // state remain hard blocks.
      const bool static_bias_learning_candidate =
        adaptive_trim_enabled && !p_->com_trim_acquired && !p_->trim_motion_active && trim_base_conditions &&
        std::abs(p_->velocity_control_sps) > kComTrimMotionVelocityExitSps &&
        std::abs(p_->velocity_control_sps) <= kComTrimUntrustedBiasVelocityMaxSps &&
        trim_quiet_rate_rms_dps <= kComTrimQuietRateRmsDps &&
        pitch_error_deg > kComTrimQuietPitchErrorDeg &&
        pitch_error_deg <= kComTrimUntrustedBiasPitchErrorMaxDeg &&
        !trim_saturation_blocked;

      const bool motion_entered =
        (!static_bias_learning_candidate &&
         (std::abs(p_->velocity_control_sps) > kComTrimMotionVelocityEnterSps ||
          trim_quiet_rate_rms_dps > kComTrimMotionRateEnterDps ||
          (pitch_error_deg > kComTrimMotionPitchErrorEnterDeg &&
           (std::abs(p_->velocity_control_sps) > kComTrimMotionVelocityExitSps ||
            trim_quiet_rate_rms_dps > kComTrimMotionRateExitDps)) ||
         trim_saturation_blocked));
    if (motion_entered) {
      p_->trim_motion_active = true;
      p_->com_trim_quiet_elapsed_s = 0.0;
    } else if (quiet_for_dwell) {
      p_->com_trim_quiet_elapsed_s += outer_dt;
    } else {
      p_->com_trim_quiet_elapsed_s = 0.0;
    }
    bool quiet_dwell_reached = quiet_for_dwell &&
                               p_->com_trim_quiet_elapsed_s >= kComTrimQuietDwellS;
    if (p_->trim_motion_active && quiet_dwell_reached) {
      // Hysteresis comes from the looser enter thresholds and this dwell. A
      // subsequent dynamic excursion must establish another full quiet dwell.
      p_->trim_motion_active = false;
      p_->com_trim_quiet_elapsed_s = 0.0;
      quiet_dwell_reached = false;
    }

      p_->trim_learning_allowed = adaptive_trim_enabled &&
          ((trim_base_conditions && !p_->trim_motion_active && !trim_saturation_blocked) ||
           static_bias_learning_candidate);
      const bool com_trim_learning =
          p_->trim_learning_allowed && (!p_->com_trim_acquired || quiet_dwell_reached);
    const double integral_delta = com_trim_learning ? unrestricted_integral_delta : 0.0;
    const double candidate_i =
        std::clamp(p_->com_trim_rad + integral_delta, -integral_limit_rad, integral_limit_rad);
      if (com_trim_learning) p_->com_trim_rad = candidate_i;
    if (!p_->com_trim_acquired && trim_base_conditions && quiet_dwell_reached &&
        quiet_equilibrium_converged && !trim_saturation_blocked) {
      p_->com_trim_acquired = true;
    }
      p_->trim_learning_enabled = com_trim_learning;
    if (com_trim_learning) {
      p_->trim_learning_block_reason = ComTrimLearningBlockNone;
    } else if (forward_command != 0.0) {
      p_->trim_learning_block_reason = ComTrimLearningBlockCommand;
    } else if (p_->nominal_acceleration_mps2 != 0.0) {
      p_->trim_learning_block_reason = ComTrimLearningBlockNominalAcceleration;
    } else if (p_->velocity_authority_limited) {
      p_->trim_learning_block_reason = ComTrimLearningBlockVelocityAuthorityLimited;
    } else if (pitch_would_saturate) {
      p_->trim_learning_block_reason = ComTrimLearningBlockPitchSetpointSaturation;
    } else if (balance_would_saturate) {
      p_->trim_learning_block_reason = ComTrimLearningBlockBalanceSaturation;
    } else if (previous_saturation_blocked) {
      p_->trim_learning_block_reason = ComTrimLearningBlockPreviousControllerSaturation;
    } else if (p_->trim_motion_active) {
      p_->trim_learning_block_reason = ComTrimLearningBlockMoving;
    } else {
      p_->trim_learning_block_reason = ComTrimLearningBlockQuietDwell;
    }
      if (!adaptive_trim_enabled) {
        p_->com_trim_rad = 0.0;
        p_->com_trim_acquired = false;
        p_->trim_learning_allowed = false;
        p_->trim_learning_enabled = false;
      }
      const double effective_trim_rad = configured_fixed_trim_rad +
                                        (adaptive_trim_enabled ? p_->com_trim_rad : 0.0);
      p_->pitch_target_unclamped_rad = candidate_pitch_without_trim + effective_trim_rad;
      p_->pitch_target_limit_reason = PitchTargetLimitNone;
      if (p_->velocity_authority_limited) {
        p_->pitch_target_limit_reason |= PitchTargetLimitVelocityAuthority;
      }
      if (std::abs(p_->pitch_target_unclamped_rad) > kMaxPitchSetpointRad) {
        p_->pitch_target_limit_reason |= PitchTargetLimitTotalPitch;
        p_->outer_pitch_target_limited = true;
      }
      p_->pitch_setpoint_rad = std::clamp(p_->pitch_target_unclamped_rad,
                                          -kMaxPitchSetpointRad, kMaxPitchSetpointRad);
    }
  }
  if (std::abs(p_->pitch_target_unclamped_rad) > pitch_limit_rad) {
    p_->controller_saturation_flags |= ControllerSaturationPitch;
  }

  // The motion operating point is a kinematic field-speed request. Keep it
  // separate from attitude feedback so the inner loop only has to correct
  // pitch/rate/acceleration errors around the moving operating point. The
  // simulator-only switch exists solely for deterministic A/B captures; the
  // production default is the feedforward architecture.
  const double drive_feedforward_sps =
      p_->simulation_drive_feedforward_enabled
          ? p_->reference_velocity_mps / Config::meters_per_step
          : 0.0;
  p_->drive_feedforward_sps = drive_feedforward_sps;

  double raw_balance_sps = 0.0;
  p_->pitch_feedback_sps = 0.0;
  p_->pitch_rate_feedback_sps = 0.0;
  p_->pitch_accel_feedback_sps = 0.0;
  const auto terms = feedback_terms(p_->pitch_setpoint_rad);
  p_->pitch_feedback_sps = terms[0];
  p_->pitch_rate_feedback_sps = terms[1];
  p_->pitch_accel_feedback_sps = terms[2];
  raw_balance_sps = terms[0] + terms[1] + terms[2];
  p_->balance_correction_sps = raw_balance_sps;
  p_->balance_unclamped_sps = raw_balance_sps;
  p_->common_unclamped_sps = drive_feedforward_sps + raw_balance_sps;
  const double balance_limit_sps = std::clamp(ConfigPid::values.balance_max_sps, 0.0, kMaxSps);
  // Clamp only after summing the nominal drive and balance correction. A
  // correction in the opposing direction can therefore cancel or reverse
  // the drive request before the final common authority boundary is reached.
  const double balance_sps = std::clamp(p_->common_unclamped_sps,
                                        -balance_limit_sps, balance_limit_sps);
  const double raw_turn_sps =
      std::clamp(static_cast<double>(joy.turn), -1.0, 1.0) * ConfigPid::values.turn_max_sps;
  const double turn_limit_sps = std::max(0.0, balance_limit_sps - std::abs(balance_sps));
  p_->turn_sps = std::clamp(raw_turn_sps, -turn_limit_sps, turn_limit_sps);
  if (balance_sps != p_->common_unclamped_sps) {
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
  p_->balance_saturated_positive = p_->common_unclamped_sps > balance_limit_sps;
  p_->balance_saturated_negative = p_->common_unclamped_sps < -balance_limit_sps;

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
