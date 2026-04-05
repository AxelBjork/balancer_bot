#pragma once

#include "msg_base.h" // pull in global MsgId
#include <cstdint>
#include <array>

namespace ipc {

// Extend the original enum by casting
constexpr MsgId ImuData = static_cast<MsgId>(3000);
constexpr MsgId JoystickCommand = static_cast<MsgId>(3001);
constexpr MsgId MotorTargets = static_cast<MsgId>(3002);
constexpr MsgId SystemTelemetry = static_cast<MsgId>(3003);

struct ImuSamplePayload {
    double pitch_rad;
    std::array<double, 3> acc;
    std::array<double, 3> gyr;
    uint64_t timestamp_us;
};

struct JoystickCommandPayload {
    float forward;
    float turn;
};

struct MotorTargetsPayload {
    float left_sps;
    float right_sps;
};

struct SystemTelemetryPayload {
    float core_cpu_usage;
    float loop_time_us;
};

} // namespace ipc

// MessageTraits is in the global namespace in msg_base.h
template <>
struct MessageTraits<ipc::ImuData> {
    using Payload = ipc::ImuSamplePayload;
};

template <>
struct MessageTraits<ipc::JoystickCommand> {
    using Payload = ipc::JoystickCommandPayload;
};

template <>
struct MessageTraits<ipc::MotorTargets> {
    using Payload = ipc::MotorTargetsPayload;
};

template <>
struct MessageTraits<ipc::SystemTelemetry> {
    using Payload = ipc::SystemTelemetryPayload;
};
