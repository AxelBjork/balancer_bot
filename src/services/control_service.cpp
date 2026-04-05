#include "control_service.h"

namespace sil {

ControlService::ControlService(ipc::MessageBus& bus)
    : bus_(bus)
{
    core_.setMotorOutputs([this](float left, float right) {
        last_left_sps_ = left;
        last_right_sps_ = right;
        ipc::MotorTargetsPayload p{};
        p.left_sps = left;
        p.right_sps = right;
        bus_.publish<ipc::MotorTargets>(p);
    });
    
    // Open loop velocity feedback approximation
    core_.setVelocityFeedback([this]() { 
        return (last_left_sps_ + last_right_sps_) / 2.0f;
    });
    
    // Telemetry publishing
    core_.setTelemetrySink([this](const Telemetry& t) {
        ipc::SystemTelemetryPayload p{};
        p.t_sec = static_cast<float>(t.t_sec);
        p.age_ms = static_cast<float>(t.age_ms);
        p.pitch_deg = static_cast<float>(t.pitch_deg);
        p.pitch_rate_dps = static_cast<float>(t.pitch_rate_dps);
        p.rate_sp_dps = static_cast<float>(t.rate_sp_dps);
        p.out_norm = static_cast<float>(t.out_norm);
        p.u_sps = static_cast<float>(t.u_sps);
        p.integ_pitch = static_cast<float>(t.integ_pitch);
        p.vel_error = static_cast<float>(t.vel_error);
        p.vel_p_term = static_cast<float>(t.vel_p_term);
        p.vel_i_term = static_cast<float>(t.vel_i_term);
        p.pitch_sp_deg = static_cast<float>(t.pitch_sp_deg);
        bus_.publish<ipc::SystemTelemetry>(p);
    });
}

} // namespace sil
