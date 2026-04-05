#pragma once

#include "msg_base.h" // pull in global MsgId
#include <array>
#include <cstdint>
#include <string_view>

namespace ipc {

// Extend the original enum by casting
constexpr MsgId ImuData = static_cast<MsgId>(3000);
constexpr MsgId JoystickCommand = static_cast<MsgId>(3001);
constexpr MsgId MotorTargets = static_cast<MsgId>(3002);
constexpr MsgId SystemTelemetry = static_cast<MsgId>(3003);

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

struct DOC_DESC("Compact controller telemetry streamed out over UDP for SIL observation.") SystemTelemetryPayload {
    float core_cpu_usage;
    float loop_time_us;
};

} // namespace ipc

// MessageTraits is in the global namespace in msg_base.h
template <>
struct MessageTraits<ipc::ImuData> {
    using Payload = ipc::ImuSamplePayload;
    static constexpr std::string_view name = "ImuData";
};

template <>
struct MessageTraits<ipc::JoystickCommand> {
    using Payload = ipc::JoystickCommandPayload;
    static constexpr std::string_view name = "JoystickCommand";
};

template <>
struct MessageTraits<ipc::MotorTargets> {
    using Payload = ipc::MotorTargetsPayload;
    static constexpr std::string_view name = "MotorTargets";
};

template <>
struct MessageTraits<ipc::SystemTelemetry> {
    using Payload = ipc::SystemTelemetryPayload;
    static constexpr std::string_view name = "SystemTelemetry";
};
