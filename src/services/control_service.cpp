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
        bus_.publish<MsgId::MotorTargets>(p);
    });
    
    core_.setVelocityFeedback([this]() {
        if (have_motor_feedback_) {
            return last_applied_avg_sps_;
        }
        return 0.5f * (last_left_sps_ + last_right_sps_);
    });

    core_.setPositionFeedback([this]() {
        if (have_motor_feedback_) {
            return last_position_m_;
        }
        return 0.0f;
    });
    
    // Telemetry publishing
    core_.setTelemetrySink([this](const Telemetry& t) {
        ipc::SystemTelemetryPayload p{};
        p.run_id = 0;
        p.t_sec = static_cast<float>(t.t_sec);
        p.sim_time_s = static_cast<float>(t.t_sec);
        p.age_ms = static_cast<float>(t.age_ms);
        p.pitch_deg = static_cast<float>(t.pitch_deg);
        p.pitch_rate_dps = static_cast<float>(t.pitch_rate_dps);
        p.raw_acc_pitch_deg = last_raw_acc_pitch_deg_;
        p.fused_pitch_deg = last_fused_pitch_deg_;
        p.gyro_pitch_rate_dps = last_gyro_pitch_rate_dps_;
        p.filtered_pitch_rate_dps = last_filtered_pitch_rate_dps_;
        p.rate_sp_dps = static_cast<float>(t.rate_sp_dps);
        p.out_norm = static_cast<float>(t.out_norm);
        p.u_sps = static_cast<float>(t.u_sps);
        p.integ_pitch = static_cast<float>(t.integ_pitch);
        p.vel_error = static_cast<float>(t.vel_error);
        p.vel_p_term = static_cast<float>(t.vel_p_term);
        p.vel_i_term = static_cast<float>(t.vel_i_term);
        p.target_vel_sps = static_cast<float>(t.target_vel_sps);
        p.measured_vel_sps = static_cast<float>(t.measured_vel_sps);
        p.filtered_vel_sps = static_cast<float>(t.filtered_vel_sps);
        p.position_target_vel_sps = static_cast<float>(t.position_target_vel_sps);
        p.pitch_sp_deg = static_cast<float>(t.pitch_sp_deg);
        p.effective_pitch_sp_deg = static_cast<float>(t.effective_pitch_sp_deg);
        p.pitch_trim_deg = static_cast<float>(t.pitch_trim_deg);
        p.trim_active = static_cast<float>(t.trim_active);
        p.command_saturated = static_cast<float>(t.command_saturated);
        p.plant_pitch_deg = static_cast<float>(t.pitch_deg);
        p.plant_pitch_rate_dps = static_cast<float>(t.pitch_rate_dps);
        p.plant_position_m = last_position_m_;
        p.plant_velocity_mps = last_applied_avg_sps_ * static_cast<float>(Config::meters_per_step);
        p.target_wheel_velocity = 0.0f;
        p.actual_wheel_velocity = last_applied_avg_sps_;
        p.plant_velocity_error = 0.0f;
        p.f_cmd = 0.0f;
        p.f_app = 0.0f;
        p.external_force_n = 0.0f;
        p.external_com_bias_rad = 0.0f;
        p.x_ddot = 0.0f;
        p.theta_ddot = 0.0f;
        p.force_saturated = 0.0f;
        bus_.publish<MsgId::SystemTelemetry>(p);
    });
}

} // namespace sil
