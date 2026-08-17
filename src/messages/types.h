#pragma once
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <string>

#include "config_pid_values.h"

// ---- IMU sample (from ISM330DHCX conditioning) ----
// angle_rad: bounded gyro/gravity pitch (+ forward), gyro_rad_s: filtered pitch rate used by control.
struct ImuSample {
  double angle_rad = 0.0;
  double gyro_rad_s = 0.0;
  double pitch_accel_rad_s2 = 0.0;
  double yaw_rate_z = 0.0;
  std::chrono::steady_clock::time_point t{};
};

// ---- Joystick command (forward/turn normalized to [-1, 1]) ----
struct JoyCmd {
  double forward;  // + forward speed command
  double turn;     // + left faster, right slower (CCW yaw)
};

enum ControllerFaultFlag : uint32_t {
  ControllerFaultNone = 0,
  ControllerFaultNoImu = 1u << 0,
  ControllerFaultStaleImu = 1u << 1,
  ControllerFaultFutureImu = 1u << 2,
  ControllerFaultFallover = 1u << 3,
  ControllerFaultActuator = 1u << 4,
};

enum ControllerSaturationFlag : uint32_t {
  ControllerSaturationNone = 0,
  ControllerSaturationPitch = 1u << 0,
  ControllerSaturationBalance = 1u << 2,
  ControllerSaturationTurn = 1u << 3,
};

// Bitmask explaining why the total pitch target was limited. A velocity bit
// means the dedicated velocity contribution was limited before composition;
// the total bit means the final pitch safety envelope was also reached.
enum PitchTargetLimitReason : uint8_t {
  PitchTargetLimitNone = 0,
  PitchTargetLimitVelocityAuthority = 1u << 0,
  PitchTargetLimitTotalPitch = 1u << 1,
};

enum ActuatorSaturationFlag : uint32_t {
  ActuatorSaturationNone = 0,
  ActuatorSaturationLeftSlew = 1u << 0,
  ActuatorSaturationRightSlew = 1u << 1,
};

// Why COM-trim learning is currently disabled.  A value of None means that
// learning is enabled for the current controller cycle.
enum ComTrimLearningBlockReason : uint8_t {
  ComTrimLearningBlockNone = 0,
  ComTrimLearningBlockCommand = 1,
  ComTrimLearningBlockNominalAcceleration = 2,
  ComTrimLearningBlockPreviousControllerSaturation = 3,
  ComTrimLearningBlockPitchSetpointSaturation = 4,
  // Legacy value retained so older telemetry captures remain decodable. The
  // explicit state-feedback controller has no rate-setpoint stage.
  ComTrimLearningBlockLegacyRateSetpointSaturation = 5,
  ComTrimLearningBlockMoving = 6,
  ComTrimLearningBlockQuietDwell = 7,
  ComTrimLearningBlockBalanceSaturation = 8,
  // Value 9 remains reserved for compatibility with older diagnostic captures.
  ComTrimLearningBlockFault = 10,
  // The velocity loop requested more pitch than its dedicated authority limit.
  ComTrimLearningBlockVelocityAuthorityLimited = 11,
  // Direct pitch-authority diagnostics explicitly freeze COM learning.
  ComTrimLearningBlockPitchAuthorityDiagnostic = 12,
};

// ---- Telemetry (controller diagnostics) ----
struct Telemetry {
  double t_sec{};
  double age_ms{};
  double pitch_deg{};
  double pitch_rate_dps{};
  double filtered_pitch_rate_dps{};
  double u_sps{};     // wheel command [steps/s]
  double turn_sps{};  // differential steering command [steps/s]
  // Deprecated compatibility diagnostics.  New analysis must use the
  // canonical SI fields below; these remain only for decoding old captures.
  double nominal_acceleration_mps2{};
  double raw_completed_velocity_sps{};
  double corrected_axle_velocity_sps{};
  double velocity_control_sps{};
  double velocity_damping_acceleration_mps2{};
  double com_trim_deg{};
  double user_velocity_mps{};
  double reference_velocity_mps{};
  double reference_acceleration_mps2{};
  double velocity_feedback_estimate_mps{};
  double velocity_error_mps{};
  double velocity_feedback_acceleration_mps2{};
  double acceleration_raw_mps2{};
  double acceleration_cmd_mps2{};
  double drive_pitch_target_deg{};
  double fixed_com_trim_deg{};
  bool velocity_feedback_valid{};
  bool velocity_feedback_active{};
  bool outer_acceleration_limited{};
  bool outer_pitch_target_limited{};
  double active_drive_max_velocity_mps{};
  double active_drive_max_acceleration_mps2{};
  double active_drive_max_deceleration_mps2{};
  double active_velocity_gain_per_s{};
  double active_velocity_feedback_cutoff_hz{};
  double active_outer_pitch_limit_deg{};
  double active_fixed_com_trim_deg{};
  bool adaptive_com_trim_enabled{};
  bool legacy_outer_fields_valid{};
  // Revised planner/PI diagnostics. These are canonical for the v12 outer
  // loop; the older damping and velocity-pitch aliases above remain zero.
  double reference_jerk_mps3{};
  double velocity_p_acceleration_mps2{};
  double velocity_i_acceleration_mps2{};
  double velocity_integral_state_mps_s{};
  double final_pitch_target_deg{};
  double active_planner_max_acceleration_mps2{};
  double active_planner_max_deceleration_mps2{};
  double active_planner_max_jerk_mps3{};
  double active_velocity_i_gain_per_s2{};
  double active_velocity_i_leak_time_s{};
  double active_velocity_i_acceleration_limit_mps2{};
  bool planner_acceleration_limited{};
  bool planner_jerk_limited{};
  bool velocity_integral_limited{};
  bool velocity_anti_windup_active{};
  // Compatibility aliases for in-process test harnesses. Wire payloads use
  // the current telemetry names above.
  double target_vel_sps{};
  double vel_error{};
  double vel_p_term_deg{};
  double vel_i_term_deg{};
  double measured_vel_sps{};
  double pitch_error_deg{};
  double pitch_sp_deg{};
  bool command_saturated{};
  bool actuator_fault{};
  uint32_t controller_fault_flags{};
  uint32_t controller_saturation_flags{};
  bool trim_learning_enabled{};
  uint8_t trim_learning_block_reason{ComTrimLearningBlockFault};
  // Explicit attitude/contribution diagnostics for the production
  // pitch/rate/acceleration state-feedback controller.
  double pitch_feedback_sps{};
  double pitch_rate_feedback_sps{};
  double pitch_accel_feedback_sps{};
  double velocity_pitch_target_deg{};
  double balance_unclamped_sps{};
  double active_pitch_gain_sps_per_rad{};
  double active_pitch_rate_gain_sps_per_rad_s{};
  double active_pitch_accel_gain_sps_per_rad_s2{};
  double active_velocity_pitch_gain_rad_per_sps{};
  double active_velocity_control_cutoff_hz{};
  double active_velocity_observer_cutoff_hz{};
  double active_com_trim_gain_deg_per_sps_s{};
  double active_com_trim_limit_deg{};
  double active_velocity_pitch_limit_deg{};
  double active_accel_lpf_hz{};
  double active_gyro_lpf_hz{};
  double active_gyro_derivative_lpf_hz{};
  uint64_t active_config_generation{};
  // Outer-loop diagnostics. These fields make velocity authority and COM-trim
  // acquisition state reconstructable from hardware telemetry.
  double velocity_pitch_request_unclamped_deg{};
  double velocity_pitch_request_limited_deg{};
  bool velocity_authority_limited{};
  double pitch_target_unclamped_deg{};
  uint8_t pitch_target_limit_reason{};
  bool trim_trusted{};
  bool trim_learning_allowed{};
  double trim_quiet_rate_rms_dps{};
  // Future direct pitch-authority captures explicitly identify the diagnostic
  // target and the frozen trim used to compose the final attitude target.
  bool pitch_authority_diagnostic_active{};
  double pitch_authority_diagnostic_target_deg{};
  double pitch_authority_diagnostic_com_trim_deg{};
  double pitch_authority_diagnostic_remaining_s{};
  uint32_t pitch_authority_diagnostic_request_id{};
  double pitch_authority_diagnostic_command_age_ms{};
  double completed_step_acceleration_sps2{};
};

using ConfigPidValues = ipc::ConfigPidValuesPayload;

enum class ConfigPidValidationCode : uint8_t {
  Accepted = 0,
  NonFinite = 1,
  Negative = 2,
  NonPositive = 3,
  OutOfRange = 4,
};

// ---- Controller Configuration ----
// Runtime-configurable controller values loaded from pid.conf. The process starts with a
// zero-initialized block; the application must load pid.conf before enabling control.
#define BALANCER_STRINGIFY_DETAIL(value) #value
#define BALANCER_STRINGIFY(value) BALANCER_STRINGIFY_DETAIL(value)
#define BALANCER_PID_CONFIG_VERSION 12

struct ConfigPid {
  inline static constexpr int config_version = BALANCER_PID_CONFIG_VERSION;
  inline static constexpr char config_version_marker[] =
      "BALANCER_PID_CONFIG_VERSION=" BALANCER_STRINGIFY(BALANCER_PID_CONFIG_VERSION);

  // One complete runtime snapshot avoids a second set of scalar storage and makes
  // configuration updates atomic at the configuration-block boundary.
  inline static ConfigPidValues values{};
  // Optional safety mode for passive IMU/telemetry measurements. This is intentionally
  // separate because it is not part of the numeric live-override payload.
  inline static bool controller_enabled{};

  static std::string resolve_path(const std::string& default_path) {
    if (const char* env = std::getenv("BALANCER_PID_CONF")) {
      if (*env != '\0') {
        return env;
      }
    }
    return default_path;
  }

  static ConfigPidValues numeric_values();
  static ConfigPidValidationCode validate_numeric(const ConfigPidValues& values);
  static void apply_numeric(const ConfigPidValues& values);
  static uint64_t generation();
  static void load(const std::string& path);
  static void save(const std::string& path);
};

#undef BALANCER_PID_CONFIG_VERSION
#undef BALANCER_STRINGIFY
#undef BALANCER_STRINGIFY_DETAIL
