#pragma once

#include "msg_base.h" // pull in global MsgId
#include "core_msgs.h"
#include <array>
#include <cstdint>
#include <string_view>

namespace ipc {

// Extend the original enum by casting
constexpr MsgId ImuData = MsgId::ImuData;
constexpr MsgId JoystickCommand = MsgId::JoystickCommand;
constexpr MsgId MotorTargets = MsgId::MotorTargets;
constexpr MsgId SystemTelemetry = MsgId::SystemTelemetry;
constexpr MsgId MotorFeedback = MsgId::MotorFeedback;

struct DOC_DESC("Fused IMU sample published by the IMU service and accepted by the SIL harness.") ImuSamplePayload {
    double pitch_rad;
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
    float t_sec;
    float age_ms;
    float pitch_deg;
    float pitch_rate_dps;
    float rate_sp_dps;
    float out_norm;
    float u_sps;
    float integ_pitch;
    float vel_error;
    float vel_p_term;
    float vel_i_term;
    float pitch_sp_deg;
    float effective_pitch_sp_deg;
    float pitch_trim_deg;
    float trim_active;
};

struct DOC_DESC("Internal motor feedback sample published by the motor service. It reflects the currently applied wheel rates after slew limiting plus the integrated actual step counts used for closed-loop hardware feedback.") MotorFeedbackPayload {
    float left_applied_sps;
    float right_applied_sps;
    int64_t left_actual_steps;
    int64_t right_actual_steps;
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
