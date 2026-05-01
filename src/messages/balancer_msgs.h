#pragma once

#include "msg_base.h" // pull in global MsgId
#include "core_msgs.h"
#include <array>
#include <cstdint>
#include <string_view>

namespace ipc {

constexpr std::size_t kMaxSimDisturbances = 10;

struct DOC_DESC("Fused IMU sample published by the IMU service and accepted by the SIL harness. The controller consumes `pitch_rad` plus `filtered_pitch_rate_rad_s`; raw accelerometer and gyro vectors are carried alongside them for diagnostics.") ImuSamplePayload {
    double pitch_rad;
    double filtered_pitch_rate_rad_s;
    std::array<double, 3> acc;
    std::array<double, 3> gyr;
    uint64_t timestamp_us;
};

struct DOC_DESC("Normalized joystick command injected by Python tests or the runtime input layer.") JoystickCommandPayload {
    float forward;
    float turn;
};

struct DOC_DESC("Wheel speed targets emitted by the controller in steps per second.") MotorTargetsPayload {
    float left_sps;
    float right_sps;
};

struct DOC_DESC("Detailed controller telemetry streamed out over UDP and used for runtime logging/visibility.") SystemTelemetryPayload {
    uint32_t run_id;
    float t_sec;
    float sim_time_s;
    float age_ms;
    float pitch_deg;
    float pitch_rate_dps;
    float raw_acc_pitch_deg;
    float fused_pitch_deg;
    float gyro_pitch_rate_dps;
    float filtered_pitch_rate_dps;
    float rate_sp_dps;
    float out_norm;
    float u_sps;
    float integ_pitch;
    float vel_error;
    float vel_p_term;
    float vel_i_term;
    float target_vel_sps;
    float measured_vel_sps;
    float filtered_vel_sps;
    float position_target_vel_sps;
    float pitch_ref_from_vel_deg;
    float pitch_ref_from_pos_deg;
    float pitch_error_deg;
    float rate_error_dps;
    float pitch_sp_deg;
    float effective_pitch_sp_deg;
    float pitch_trim_deg;
    float trim_active;
    float command_saturated;
    float left_applied_sps;
    float right_applied_sps;
    int64_t left_actual_steps;
    int64_t right_actual_steps;
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
    float force_saturated;
};

struct DOC_DESC("Internal motor feedback sample published by the motor service. It carries the currently applied wheel rates after slew limiting, a steps-derived average wheel-speed estimate used by closed-loop hardware feedback, and the integrated actual step counts.") MotorFeedbackPayload {
    float left_applied_sps;
    float right_applied_sps;
    float measured_avg_sps;
    int64_t left_actual_steps;
    int64_t right_actual_steps;
};

constexpr uint8_t kSimDisturbanceStep = 0;
constexpr uint8_t kSimDisturbanceRamp = 1;
constexpr uint8_t kSimDisturbanceHoldBias = 2;

struct DOC_DESC("One scheduled simulator plant disturbance segment. Step disturbances apply a constant external horizontal force and COM bias for the active window. Ramp disturbances interpolate from the start values to the end values across the window. Hold-bias disturbances apply a constant external force and COM bias from start_s until duration_s expires, or to the end of the run when duration_s is non-positive.") SimDisturbancePayload {
    uint8_t kind;
    uint8_t reserved0;
    uint16_t reserved1;
    double start_s;
    double duration_s;
    float force_n;
    float com_bias_rad;
    float force_n_end;
    float com_bias_rad_end;
};

struct DOC_DESC("Request from Python to start a single simulator run. Only one run may be active at a time.") SimStartRunPayload {
    uint32_t run_id;
    uint8_t physics_profile;
    uint8_t reserved0;
    uint16_t reserved1;
    double duration_s;
    double initial_pitch_deg;
    double com_angle_offset_rad;
    float wheel_slip_factor;
    float velocity_feedback_scale;
    double velocity_feedback_tau_s;
    double imu_pitch_lag_s;
    std::array<ipc::SimDisturbancePayload, kMaxSimDisturbances> disturbances;
    std::array<char, 128> pid_config_path;
};

struct DOC_DESC("Immediate simulator reply indicating whether a start request was accepted.") SimStartAckPayload {
    uint32_t run_id;
    uint8_t accepted;
    uint8_t status_code;
    uint16_t reserved;
};

struct DOC_DESC("Request from Python to stop the currently running simulator scenario.") SimStopRunPayload {
    uint32_t run_id;
};

struct DOC_DESC("Terminal simulator status emitted once per accepted run.") SimRunDonePayload {
    uint32_t run_id;
    uint8_t reason_code;
    uint8_t reserved0;
    uint16_t reserved1;
    uint32_t sample_count;
    float elapsed_s;
    float final_pitch_deg;
    float max_abs_pitch_deg;
    float tail_rms_pitch_deg;
    float tail_rail_fraction;
    float tail_mean_abs_pitch_deg;
    float max_abs_position_m;
    float tail_mean_abs_velocity_mps;
};

} // namespace ipc

// MessageTraits is in the global namespace in msg_base.h
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
