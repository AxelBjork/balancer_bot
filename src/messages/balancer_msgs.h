#pragma once

#include <array>
#include <cstdint>
#include <string_view>

#include "core_msgs.h"
#include "msg_base.h"  // pull in global MsgId

namespace ipc {

constexpr std::size_t kMaxSimDisturbances = 10;

struct DOC_DESC(
    "Raw accelerometer and gyroscope sample emitted before attitude fusion. Hardware and "
    "hardware-like simulation feed this message into `ImuService`, which applies the complementary "
    "pitch filter and republishes the controller-facing `ImuData`.") ImuRawPayload {
  std::array<double, 3> acc;
  std::array<double, 3> gyr;
  uint64_t timestamp_us;
};

struct DOC_DESC(
    "Fused IMU sample published by the IMU service and accepted by the SIL harness. The controller "
    "consumes `pitch_rad`, `pitch_rate_rad_s`, and "
    "`pitch_accel_rad_s2`; raw accelerometer and gyro vectors are carried alongside them "
    "for diagnostics.") ImuSamplePayload {
  double pitch_rad;
  double pitch_rate_rad_s;
  double pitch_accel_rad_s2;
  std::array<double, 3> acc;
  std::array<double, 3> gyr;
  uint64_t timestamp_us;
};

struct DOC_DESC("Normalized joystick command injected by Python tests or the runtime input layer.")
    JoystickCommandPayload {
  double forward;
  double turn;
};

struct DOC_DESC("Wheel speed targets emitted by the controller in steps per second.")
    MotorTargetsPayload {
  double left_sps;
  double right_sps;
};

struct DOC_DESC(
    "Detailed controller telemetry streamed out over UDP and used for runtime logging/visibility.")
    SystemTelemetryPayload {
  uint32_t run_id;
  double t_sec;
  double age_ms;
  double pitch_deg;
  double pitch_rate_dps;
  double raw_acc_pitch_deg;
  double fused_pitch_deg;
  double gyro_pitch_rate_dps;
  double filtered_pitch_rate_dps;
  double u_sps;
  double turn_sps;
  double vel_error;
  double measured_vel_sps;
  double vel_p_term;
  double position_target_vel_sps;
  double pitch_ref_from_vel_deg;
  double pitch_ref_from_pos_deg;
  double pitch_error_deg;
  double pitch_sp_deg;
  double pitch_trim_deg;
  double trim_active;
  double left_applied_sps;
  double right_applied_sps;
  double motor_update_dt_ms;
  double motor_feedback_age_ms;
  int64_t left_actual_steps;
  int64_t right_actual_steps;
  double plant_pitch_deg;
  double plant_pitch_rate_dps;
  double plant_position_m;
  double plant_velocity_mps;
  double target_wheel_velocity;
  double actual_wheel_velocity;
  double plant_velocity_error;
  double f_cmd;
  double f_app;
  double external_force_n;
  double external_com_bias_rad;
  double x_ddot;
  double theta_ddot;
  double force_saturated;
};

struct DOC_DESC(
    "Internal motor feedback sample published by the motor service. It carries the currently "
    "applied wheel rates after slew limiting, a steps-derived average wheel-speed estimate used by "
    "closed-loop hardware feedback, and the integrated actual step counts.") MotorFeedbackPayload {
  double left_applied_sps;
  double right_applied_sps;
  double measured_avg_sps;
  double update_dt_ms;
  double feedback_age_ms;
  int64_t left_actual_steps;
  int64_t right_actual_steps;
};

constexpr uint8_t kSimDisturbanceStep = 0;
constexpr uint8_t kSimDisturbanceRamp = 1;
constexpr uint8_t kSimDisturbanceHoldBias = 2;
constexpr uint8_t kSimImuModeDirectFused = 0;
constexpr uint8_t kSimImuModeHardwareLpf = 1;

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
    "Request from Python to start a single simulator run. Only one run may be active at a time.")
    SimStartRunPayload {
  uint32_t run_id;
  uint8_t physics_profile;
  uint8_t reserved0;
  uint16_t reserved1;
  double duration_s;
  double initial_pitch_deg;
  double com_angle_offset_rad;
  double wheel_slip_factor;
  double velocity_feedback_scale;
  double velocity_feedback_tau_s;
  double imu_pitch_lag_s;
  uint32_t imu_noise_seed;
  double accel_noise_std_mps2;
  double gyro_noise_std_rad_s;
  std::array<double, 3> accel_bias_mps2;
  std::array<double, 3> gyro_bias_rad_s;
  std::array<ipc::SimDisturbancePayload, kMaxSimDisturbances> disturbances;
  std::array<char, 128> pid_config_path;
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
};

}  // namespace ipc

// MessageTraits is in the global namespace in msg_base.h
template <>
struct MessageTraits<MsgId::ImuRawData> {
  using Payload = ipc::ImuRawPayload;
  static constexpr std::string_view name = "ImuRawData";
};

template <>
struct MessageTraits<MsgId::ImuData> {
  using Payload = ipc::ImuSamplePayload;
  static constexpr std::string_view name = "ImuData";
};

template <>
struct MessageTraits<MsgId::JoystickCommand> {
  using Payload = ipc::JoystickCommandPayload;
  static constexpr std::string_view name = "JoystickCommand";
};

template <>
struct MessageTraits<MsgId::MotorTargets> {
  using Payload = ipc::MotorTargetsPayload;
  static constexpr std::string_view name = "MotorTargets";
};

template <>
struct MessageTraits<MsgId::SystemTelemetry> {
  using Payload = ipc::SystemTelemetryPayload;
  static constexpr std::string_view name = "SystemTelemetry";
};

template <>
struct MessageTraits<MsgId::MotorFeedback> {
  using Payload = ipc::MotorFeedbackPayload;
  static constexpr std::string_view name = "MotorFeedback";
};

template <>
struct MessageTraits<MsgId::SimStartRun> {
  using Payload = ipc::SimStartRunPayload;
  static constexpr std::string_view name = "SimStartRun";
};

template <>
struct MessageTraits<MsgId::SimStartAck> {
  using Payload = ipc::SimStartAckPayload;
  static constexpr std::string_view name = "SimStartAck";
};

template <>
struct MessageTraits<MsgId::SimStopRun> {
  using Payload = ipc::SimStopRunPayload;
  static constexpr std::string_view name = "SimStopRun";
};

template <>
struct MessageTraits<MsgId::SimRunDone> {
  using Payload = ipc::SimRunDonePayload;
  static constexpr std::string_view name = "SimRunDone";
};
