#pragma once

#include "config.h"
#include "services/control/rate_controller_core.h"
#include "publisher.h"
#include "messages/balancer_msgs.h"
#include <cmath>

namespace sil {

inline constexpr char kControlServiceDoc[] =
    "Owns the balancing control pipeline that converts `PhysicsTick`, `ImuData`, and "
    "`JoystickCommand`, and `MotorFeedback` inputs into wheel-speed targets and streaming controller "
    "telemetry.\n\n"
    "This service is intentionally thin: it caches the latest bus inputs, translates them into the "
    "`RateControllerCore` API, and republishes the core's outputs as reflected IPC payloads. The "
    "control law itself is a cascaded structure. A joystick forward command or position-hold "
    "correction first becomes a target wheel velocity in steps per second. A velocity PID then "
    "produces a body pitch setpoint, which is blended down as the measured tilt grows so balance "
    "recovery can dominate near larger disturbances. The pitch error is then turned into a rate "
    "setpoint and fed through the PX4 `RateControl` block:\n\n"
    "$$ v_{target} = u_{joy} \\cdot k_{max\\_sps} + v_{hold} $$\n\n"
    "$$ \\theta_{sp} = \\operatorname{blend}(\\theta_{vel}, \\theta_{trim}) $$\n\n"
    "$$ \\omega_{sp} = k_{angle\\rightarrow rate}(\\theta_{sp} - \\theta) $$\n\n"
    "The resulting normalized pitch-axis effort is scaled into motor commands in steps per second, "
    "clamped to the configured ceiling, and split into left/right wheel targets by adding a turn "
    "term.\n\n"
    "The service also exposes several practical adaptations that matter for balancing behavior: it "
    "uses real motor feedback from `MotorService` whenever it is available and falls back to the "
    "last commanded wheel speeds only in SIL-style configurations where no hardware feedback "
    "exists. It also integrates a small position-hold correction when the operator is not driving, "
    "learns a slow lean-trim bias from persistent drift, and resets or decays that trim when the "
    "robot is highly tilted or actively commanded. Every control step also publishes "
    "`SystemTelemetry`, including pitch, pitch rate, rate setpoint, controller output, wheel-speed "
    "command, velocity error, integral terms, effective trim, and trim-active state so the SIL "
    "harness can inspect controller internals without attaching directly to the core.";

class DOC_DESC(kControlServiceDoc) ControlService {
public:
    static constexpr const char* kDocDescription = kControlServiceDoc;

    using Publishes = ipc::MsgList<MsgId::MotorTargets, MsgId::SystemTelemetry>;
    using Subscribes = ipc::MsgList<MsgId::PhysicsTick, MsgId::ImuData, MsgId::JoystickCommand, MsgId::MotorFeedback>;

    explicit ControlService(ipc::MessageBus& bus);
    ~ControlService() = default;

    void start() {}
    void stop() {}

    template <MsgId Id>
    void on_message(const typename MessageTraits<Id>::Payload& p) {}

private:
    ipc::TypedPublisher<ControlService> bus_;
    RateControllerCore core_;
    
    // Fallback proxy for SIL when no explicit motor feedback is available.
    float last_left_sps_ = 0.0f;
    float last_right_sps_ = 0.0f;
    float last_applied_avg_sps_ = 0.0f;
    float last_position_m_ = 0.0f;
    bool have_motor_feedback_ = false;
    float last_raw_acc_pitch_deg_ = 0.0f;
    float last_fused_pitch_deg_ = 0.0f;
    float last_gyro_pitch_rate_dps_ = 0.0f;
};

template <>
inline void ControlService::on_message<MsgId::JoystickCommand>(const ipc::JoystickCommandPayload& p) {
    JoyCmd cmd{p.forward, p.turn};
    core_.setJoystick(cmd);
}

template <>
inline void ControlService::on_message<MsgId::ImuData>(const ipc::ImuSamplePayload& p) {
    ImuSample s{};
    s.angle_rad = p.pitch_rad;
    s.gyro_rad_s = p.gyr[1]; 
    s.yaw_rate_z = p.gyr[2]; 
    s.t = std::chrono::steady_clock::time_point(std::chrono::microseconds(p.timestamp_us));
    const double ax = p.acc[0];
    const double ay = p.acc[1];
    const double az = p.acc[2];
    last_raw_acc_pitch_deg_ = static_cast<float>(
        std::atan2(-ax, std::sqrt(ay * ay + az * az)) * (180.0 / M_PI));
    last_fused_pitch_deg_ = static_cast<float>(p.pitch_rad * (180.0 / M_PI));
    last_gyro_pitch_rate_dps_ = static_cast<float>(p.gyr[1] * (180.0 / M_PI));
    core_.pushImu(s);
}

template <>
inline void ControlService::on_message<MsgId::PhysicsTick>(const PhysicsTickPayload& p) {
    const auto now = std::chrono::steady_clock::time_point(std::chrono::microseconds(p.sim_time_us));
    core_.step(p.dt_s, now);
}

template <>
inline void ControlService::on_message<MsgId::MotorFeedback>(const ipc::MotorFeedbackPayload& p) {
    last_applied_avg_sps_ = 0.5f * (p.left_applied_sps + p.right_applied_sps);
    const float avg_steps = 0.5f * static_cast<float>(p.left_actual_steps + p.right_actual_steps);
    last_position_m_ = avg_steps * static_cast<float>(Config::meters_per_step);
    have_motor_feedback_ = true;
}

} // namespace sil
