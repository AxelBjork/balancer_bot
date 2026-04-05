#pragma once

#include "control_loop.h"
#include "publisher.h"
#include "balancer_msgs.h"

namespace sil {

class DOC_DESC("Consumes IMU and joystick inputs, runs the cascaded balancing controller, and publishes motor targets plus lightweight telemetry.") ControlService {
public:
    using Publishes = ipc::MsgList<ipc::MotorTargets, ipc::SystemTelemetry>;
    using Subscribes = ipc::MsgList<ipc::ImuData, ipc::JoystickCommand>;

    explicit ControlService(ipc::MessageBus& bus);
    ~ControlService() = default;

    void start() {}
    void stop() {}

    template <MsgId Id>
    void on_message(const typename MessageTraits<Id>::Payload& p) {}

private:
    ipc::TypedPublisher<ControlService> bus_;
    RateControllerCore core_;
    
    // Local cache for open-loop velocity feedback trick
    float last_left_sps_ = 0.0f;
    float last_right_sps_ = 0.0f;
};

template <>
inline void ControlService::on_message<ipc::JoystickCommand>(const ipc::JoystickCommandPayload& p) {
    JoyCmd cmd{p.forward, p.turn};
    core_.setJoystick(cmd);
}

template <>
inline void ControlService::on_message<ipc::ImuData>(const ipc::ImuSamplePayload& p) {
    ImuSample s{};
    s.angle_rad = p.pitch_rad;
    s.gyro_rad_s = p.gyr[1]; 
    s.yaw_rate_z = p.gyr[2]; 
    s.t = std::chrono::steady_clock::time_point(std::chrono::microseconds(p.timestamp_us));
    core_.pushImu(s);
}

} // namespace sil
