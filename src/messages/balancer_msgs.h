#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "config_pid_values.h"
#include "core_msgs.h"
#include "msg_base.h"  // pull in global MsgId

namespace ipc {

constexpr std::size_t kMaxSimDisturbances = 10;

struct DOC_DESC(
    "Raw accelerometer and gyroscope sample emitted before chassis-referenced pitch estimation. "
    "Hardware and hardware-like simulation feed this message into `ImuService`, which performs "
    "bounded IMU-only signal conditioning and republishes controller-facing `ImuData`.")
    ImuRawPayload {
  std::array<double, 3> acc;
  std::array<double, 3> gyr;
  uint64_t timestamp_us;
};

struct DOC_DESC(
    "Chassis-referenced IMU sample published by the IMU service for controller and diagnostic use. "
    "The controller consumes the pitch fields only while `estimate_valid` is true; an invalid "
    "sample immediately clears its current IMU. Raw accelerometer and gyro vectors are carried "
    "alongside the bounded gyro/gravity pitch, rate, and derivative for diagnostics.")
    ImuSamplePayload {
  double pitch_rad;
  double pitch_rate_rad_s;
  double pitch_accel_rad_s2;
  std::array<double, 3> acc;
  std::array<double, 3> gyr;
  uint64_t timestamp_us;
  bool estimate_valid;
};

struct DOC_DESC(
    "Normalized joystick command injected by an external client or the runtime input layer.")
    JoystickCommandPayload {
  double forward;
  double turn;
};

struct DOC_DESC(
    "User-supervised pitch-authority diagnostic command. When active, the controller disables "
    "drive and velocity pitch contributions, freezes COM learning at the supplied trim, and "
    "feeds the validated direct target through the normal final pitch-target and safety path. "
    "Targets are limited to 0, +/-1, +/-2, or +/-4 degrees.")
    PitchAuthorityDiagnosticCommandPayload {
  uint32_t request_id;
  uint8_t active;
  uint8_t reserved0;
  uint16_t reserved1;
  double target_deg;
  double com_trim_deg;
  // Active commands expire unless refreshed. A bounded duration keeps a lost
  // diagnostic client from leaving a direct target selected indefinitely.
  double duration_s;
};

struct DOC_DESC(
    "Complete in-memory controller override from the dashboard. This message intentionally excludes the "
    "file schema version and controller enable mode.") PidConfigOverridePayload {
  uint32_t request_id;
  uint32_t reserved;
  ConfigPidValuesPayload values;
};

struct DOC_DESC("Result of applying a dashboard PID override and the currently active values.")
    PidConfigStatusPayload {
  uint32_t request_id;
  uint8_t accepted;
  uint8_t result_code;
  uint16_t reserved;
  ConfigPidValuesPayload values;
};

struct DOC_DESC("Wheel speed targets emitted by the controller in steps per second.")
    MotorTargetsPayload {
  double left_sps;
  double right_sps;
};

struct DOC_DESC(
    "Detailed controller telemetry streamed to the active UDP runtime peer and used by the "
    "telemetry "
    "server for logging and live visibility. "
    "Actuator saturation bit 0 is left slew limiting and bit 1 is right slew limiting. "
    "trim_learning_block_reason is zero when COM-trim learning is enabled; otherwise it uses "
    "the ComTrimLearningBlockReason enum.")
    SystemTelemetryPayload {
  uint32_t run_id;
  uint32_t controller_fault_flags;
  uint32_t controller_saturation_flags;
  uint64_t imu_timestamp_us;
  float t_sec;
  float age_ms;
  float pitch_deg;
  float pitch_rate_dps;
  float raw_acc_pitch_deg;
  float fused_pitch_deg;
  float gyro_pitch_rate_dps;
  float filtered_pitch_rate_dps;
  float u_sps;
  float turn_sps;
  float nominal_acceleration_mps2;
  float raw_completed_velocity_sps;
  float corrected_axle_velocity_sps;
  float velocity_damping_acceleration_mps2;
  float com_trim_deg;
  float pitch_error_deg;
  float pitch_sp_deg;
  float left_target_sps;
  float right_target_sps;
  float left_slewed_sps;
  float right_slewed_sps;
  float motor_update_dt_ms;
  float motor_feedback_age_ms;
  int32_t left_actual_steps;
  int32_t right_actual_steps;
  uint32_t actuator_saturation_flags;
  bool command_saturated;
  bool actuator_fault;
  uint8_t trim_learning_enabled;
  uint8_t trim_learning_block_reason;
  uint16_t trim_learning_reserved;
  float pitch_feedback_sps;
  float pitch_rate_feedback_sps;
  float pitch_accel_feedback_sps;
  float velocity_pitch_target_deg;
  float balance_unclamped_sps;
  float active_pitch_gain_sps_per_rad;
  float active_pitch_rate_gain_sps_per_rad_s;
  float active_pitch_accel_gain_sps_per_rad_s2;
  float active_velocity_pitch_gain_rad_per_sps;
  float active_velocity_control_cutoff_hz;
  float active_velocity_observer_cutoff_hz;
  float active_com_trim_gain_deg_per_sps_s;
  float active_com_trim_limit_deg;
  float active_accel_lpf_hz;
  float active_gyro_lpf_hz;
  float active_gyro_derivative_lpf_hz;
  uint64_t active_config_generation;
  // Outer-loop authority and COM acquisition diagnostics appended after the
  // established telemetry payload for compatibility with older readers.
  float velocity_pitch_request_unclamped_deg;
  float velocity_pitch_request_limited_deg;
  float pitch_target_unclamped_deg;
  float active_velocity_pitch_limit_deg;
  float trim_quiet_rate_rms_dps;
  bool velocity_authority_limited;
  bool trim_trusted;
  bool trim_learning_allowed;
  uint8_t pitch_target_limit_reason;
  float velocity_control_sps;
  // Direct pitch-authority diagnostic state appended for future capture
  // reconstruction. Existing target/contribution fields remain the
  // production path and are not repurposed for this mode.
  bool pitch_authority_diagnostic_active;
  float pitch_authority_diagnostic_target_deg;
  float pitch_authority_diagnostic_com_trim_deg;
  float pitch_authority_diagnostic_remaining_s;
  uint32_t pitch_authority_diagnostic_request_id;
  float pitch_authority_diagnostic_command_age_ms;
  float completed_step_acceleration_sps2;
  // Source-side timing metadata is appended so the dashboard can distinguish
  // sender pauses, transport loss, and receiver-side bursts. packet_seq is
  // assigned when the controller produces a telemetry payload; it therefore
  // also exposes packets coalesced or dropped by the transport boundary.
  uint64_t packet_seq;
  uint64_t loop_seq;
  uint64_t sender_monotonic_ns;
  // Canonical SI velocity-reference outer-loop telemetry. These fields are
  // appended so existing payload offsets remain stable for older readers.
  float user_velocity_mps;
  float reference_velocity_mps;
  float reference_acceleration_mps2;
  float velocity_feedback_estimate_mps;
  float velocity_error_mps;
  float velocity_feedback_acceleration_mps2;
  float acceleration_raw_mps2;
  float acceleration_cmd_mps2;
  float drive_pitch_target_deg;
  float fixed_com_trim_deg;
  bool velocity_feedback_valid;
  bool velocity_feedback_active;
  bool outer_acceleration_limited;
  bool outer_pitch_target_limited;
  float active_drive_max_velocity_mps;
  float active_drive_max_acceleration_mps2;
  float active_drive_max_deceleration_mps2;
  float active_velocity_gain_per_s;
  float active_velocity_feedback_cutoff_hz;
  float active_outer_pitch_limit_deg;
  float active_fixed_com_trim_deg;
  bool adaptive_com_trim_enabled;
  bool legacy_outer_fields_valid;
  // Revised jerk-limited planner and bounded leaky-integral diagnostics are
  // appended to preserve all established wire offsets.
  float reference_jerk_mps3;
  float velocity_p_acceleration_mps2;
  float velocity_i_acceleration_mps2;
  float velocity_integral_state_mps_s;
  float final_pitch_target_deg;
  float active_planner_max_acceleration_mps2;
  float active_planner_max_deceleration_mps2;
  float active_planner_max_jerk_mps3;
  float active_velocity_i_gain_per_s2;
  float active_velocity_i_leak_time_s;
  float active_velocity_i_acceleration_limit_mps2;
  bool planner_acceleration_limited;
  bool planner_jerk_limited;
  bool velocity_integral_limited;
  bool velocity_anti_windup_active;
};

struct SimulatorTelemetryPayload {
  SystemTelemetryPayload system;
  uint32_t seed;
  float plant_pitch_deg;
  float plant_pitch_rate_dps;
  float plant_position_m;
  float plant_velocity_mps;
  float target_wheel_velocity;
  float actual_wheel_velocity;
  float plant_velocity_error;
  float f_cmd;
  float f_app;
  float external_force_n;
  float external_com_bias_rad;
  float x_ddot;
  float theta_ddot;
  float phase_error_steps;
  float missed_steps;
  float traction_limit_n;
  float motor_force_limit_n;
  float total_mass_scale;
  float pitch_inertia_scale;
  float motor_max_force_n;
  float motor_no_load_speed_mps;
  float motor_velocity_damping;
  float motor_tau_s;
  float traction_coefficient;
  float pitch_damping;
  float cart_damping;
  float phase_error_limit_steps;
  float tire_stiffness_n_per_m;
  float tire_damping_n_s_per_m;
  float wheel_equivalent_mass_kg;
  bool force_saturated;
};

struct DOC_DESC(
    "Internal motor feedback sample published by the motor service. It carries the currently "
    "slewed wheel commands, a steps-derived average wheel-speed estimate used by diagnostics, "
    "and the integrated actual step counts. Actuator saturation bit "
    "0 is left slew limiting and bit 1 is right slew limiting.") MotorFeedbackPayload {
  double left_slewed_sps;
  double right_slewed_sps;
  double measured_avg_sps;
  double update_dt_ms;
  double feedback_age_ms;
  int64_t left_actual_steps;
  int64_t right_actual_steps;
  uint32_t actuator_saturation_flags;
  uint8_t actuator_fault;
};

constexpr uint8_t kSimDisturbanceStep = 0;
constexpr uint8_t kSimDisturbanceRamp = 1;
constexpr uint8_t kSimDisturbanceHoldBias = 2;
inline constexpr std::size_t kMaxSimJoySegments = 4;
inline constexpr std::size_t kMaxSimPitchAuthoritySegments = 12;

struct DOC_DESC("One deterministic joystick segment scheduled on the simulator timeline.")
    SimJoySegmentPayload {
  double start_s;
  double duration_s;
  double forward;
  double turn;
  double forward_end;
  double turn_end;
};

struct DOC_DESC(
    "One scheduled simulator plant disturbance segment. Step disturbances apply a constant "
    "external horizontal force and COM bias for the active window. Ramp disturbances interpolate "
    "from the start values to the end values across the window. Hold-bias disturbances apply a "
    "constant external force and COM bias from start_s until duration_s expires, or to the end of "
    "the run when duration_s is non-positive.") SimDisturbancePayload {
  uint8_t kind;
  uint8_t reserved0;
  uint16_t reserved1;
  double start_s;
  double duration_s;
  double force_n;
  double com_bias_rad;
    double force_n_end;
  double com_bias_rad_end;
};

struct DOC_DESC(
    "One scheduled direct pitch-authority diagnostic segment. The simulator refreshes the "
    "short-lived diagnostic command while this segment is active so the production safety and "
    "attitude-target path is exercised without enabling drive, velocity, or COM learning.")
    SimPitchAuthoritySegmentPayload {
  double start_s;
  double duration_s;
  double target_deg;
  double com_trim_deg;
};

struct DOC_DESC(
    "Request from Python to start a single simulator run. Only one run may be active at a time. "
    "The optional velocity-estimator impairment fields affect only the controller-facing "
    "velocity feedback; plant telemetry remains ground truth.")
    SimStartRunPayload {
  uint32_t run_id;
  uint8_t physics_profile;
  uint8_t has_physics_override;
  uint16_t telemetry_stride;
  uint16_t transfer_scenario_index;
  uint16_t reserved1;
  double duration_s;
  double initial_pitch_deg;
  double com_angle_offset_rad;
  double total_mass_scale;
  double pitch_inertia_scale;
  double motor_max_force_n;
  double motor_no_load_speed_mps;
  double motor_velocity_damping;
  double motor_tau_s;
  double traction_coefficient;
  double pitch_damping;
  double cart_damping;
  double phase_error_limit_steps;
  double tire_stiffness_n_per_m;
  double tire_damping_n_s_per_m;
  double wheel_equivalent_mass_kg;
  double imu_pitch_lag_s;
  uint32_t imu_noise_seed;
  double accel_noise_std_mps2;
  double gyro_noise_std_rad_s;
  double imu_timestamp_jitter_us;
  double imu_sample_loss_rate;
  std::array<double, 3> accel_bias_mps2;
  std::array<double, 3> gyro_bias_rad_s;
  std::array<ipc::SimDisturbancePayload, kMaxSimDisturbances> disturbances;
  std::array<ipc::SimJoySegmentPayload, kMaxSimJoySegments> joy_segments;
  std::array<char, 128> pid_config_path;
  double initial_velocity_mps;
  double velocity_estimator_bias_mps;
  double velocity_estimator_bias_drift_mps_per_s;
  double velocity_estimator_scale;
  double velocity_estimator_latency_s;
  // Initial body rate is simulator-only state used for reproducing measured
  // startup/recovery transients. It has no hardware wire consumer.
  double initial_pitch_rate_dps;
  std::array<ipc::SimPitchAuthoritySegmentPayload, kMaxSimPitchAuthoritySegments>
      pitch_authority_segments;
  // Test-only deterministic refresh dropout used to verify the diagnostic
  // watchdog. A zero duration leaves the scheduled refresh path intact.
  double pitch_authority_refresh_dropout_start_s;
  double pitch_authority_refresh_dropout_duration_s;
};

struct DOC_DESC("Immediate simulator reply indicating whether a start request was accepted.")
    SimStartAckPayload {
  uint32_t run_id;
  uint8_t accepted;
  uint8_t status_code;
  uint16_t reserved;
};

struct DOC_DESC("Request from Python to stop the currently running simulator scenario.")
    SimStopRunPayload {
  uint32_t run_id;
};

struct DOC_DESC("Terminal simulator status emitted once per accepted run.") SimRunDonePayload {
  uint32_t run_id;
  uint8_t reason_code;
  uint8_t reserved0;
  uint16_t reserved1;
  uint32_t sample_count;
  double elapsed_s;
  double final_pitch_deg;
  double max_abs_pitch_deg;
  double tail_rms_pitch_deg;
  double tail_rail_fraction;
  double tail_mean_abs_pitch_deg;
  double max_abs_position_m;
  double tail_mean_abs_velocity_mps;
  double max_continuous_saturation_s;
  uint32_t actuator_fault_count;
  uint32_t controller_fault_flags;
};

}  // namespace ipc

// MessageTraits is in the global namespace in msg_base.h
template <>
struct MessageTraits<MsgId::ImuRawData> {
  using Payload = ipc::ImuRawPayload;
};

template <>
struct MessageTraits<MsgId::ImuData> {
  using Payload = ipc::ImuSamplePayload;
};

template <>
struct MessageTraits<MsgId::JoystickCommand> {
  using Payload = ipc::JoystickCommandPayload;
};

template <>
struct MessageTraits<MsgId::ExternalJoystickCommand> {
  using Payload = ipc::JoystickCommandPayload;
};

template <>
struct MessageTraits<MsgId::PitchAuthorityDiagnosticCommand> {
  using Payload = ipc::PitchAuthorityDiagnosticCommandPayload;
};

template <>
struct MessageTraits<MsgId::PidConfigOverride> {
  using Payload = ipc::PidConfigOverridePayload;
};

template <>
struct MessageTraits<MsgId::PidConfigStatus> {
  using Payload = ipc::PidConfigStatusPayload;
};

template <>
struct MessageTraits<MsgId::MotorTargets> {
  using Payload = ipc::MotorTargetsPayload;
};

template <>
struct MessageTraits<MsgId::SystemTelemetry> {
  using Payload = ipc::SystemTelemetryPayload;
};

template <>
struct MessageTraits<MsgId::SimulatorTelemetry> {
  using Payload = ipc::SimulatorTelemetryPayload;
};

template <>
struct MessageTraits<MsgId::MotorFeedback> {
  using Payload = ipc::MotorFeedbackPayload;
};

template <>
struct MessageTraits<MsgId::SimStartRun> {
  using Payload = ipc::SimStartRunPayload;
};

template <>
struct MessageTraits<MsgId::SimStartAck> {
  using Payload = ipc::SimStartAckPayload;
};

template <>
struct MessageTraits<MsgId::SimStopRun> {
  using Payload = ipc::SimStopRunPayload;
};

template <>
struct MessageTraits<MsgId::SimRunDone> {
  using Payload = ipc::SimRunDonePayload;
};
