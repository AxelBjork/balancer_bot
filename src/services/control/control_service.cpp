#include "services/control/control_service.h"

namespace sil {

ControlService::ControlService(ipc::MessageBus& bus) : bus_(bus) {
  core_.setMotorOutputs([this](double left, double right) {
    last_left_sps_ = left;
    last_right_sps_ = right;
    ipc::MotorTargetsPayload p{};
    p.left_sps = left;
    p.right_sps = right;
    bus_.publish<MsgId::MotorTargets>(p);
  });

  // Telemetry publishing
  core_.setTelemetrySink([this](const Telemetry& t) {
    ipc::SystemTelemetryPayload p{};
    p.run_id = 0;
    p.controller_fault_flags = t.controller_fault_flags;
    p.controller_saturation_flags = t.controller_saturation_flags;
    p.t_sec = static_cast<float>(t.t_sec);
    p.age_ms = static_cast<float>(t.age_ms);
    p.pitch_deg = static_cast<float>(t.pitch_deg);
    p.pitch_rate_dps = static_cast<float>(t.pitch_rate_dps);
    p.raw_acc_pitch_deg = static_cast<float>(last_raw_acc_pitch_deg_);
    p.fused_pitch_deg = static_cast<float>(last_fused_pitch_deg_);
    p.gyro_pitch_rate_dps = static_cast<float>(last_gyro_pitch_rate_dps_);
    p.filtered_pitch_rate_dps = static_cast<float>(last_filtered_pitch_rate_dps_);
    p.u_sps = static_cast<float>(t.u_sps);
    p.turn_sps = static_cast<float>(t.turn_sps);
    p.nominal_acceleration_mps2 = static_cast<float>(t.nominal_acceleration_mps2);
    p.raw_completed_velocity_sps = static_cast<float>(t.raw_completed_velocity_sps);
    p.corrected_axle_velocity_sps = static_cast<float>(t.corrected_axle_velocity_sps);
    p.velocity_damping_acceleration_mps2 =
        static_cast<float>(t.velocity_damping_acceleration_mps2);
    p.com_trim_deg = static_cast<float>(t.com_trim_deg);
    p.pitch_error_deg = static_cast<float>(t.pitch_error_deg);
    p.pitch_sp_deg = static_cast<float>(t.pitch_sp_deg);
    p.rate_setpoint_dps = static_cast<float>(t.rate_sp_dps);
    p.rate_error_dps = static_cast<float>(t.rate_error_dps);
    p.command_saturated = t.command_saturated;
    p.actuator_fault = t.actuator_fault;
    p.left_target_sps = static_cast<float>(last_left_sps_);
    p.right_target_sps = static_cast<float>(last_right_sps_);
    p.left_slewed_sps =
        static_cast<float>(have_motor_feedback_ ? latest_motor_feedback_.left_slewed_sps : last_left_sps_);
    p.right_slewed_sps =
        static_cast<float>(have_motor_feedback_ ? latest_motor_feedback_.right_slewed_sps : last_right_sps_);
    p.motor_update_dt_ms = static_cast<float>(have_motor_feedback_ ? latest_motor_feedback_.update_dt_ms : 0.0);
    p.motor_feedback_age_ms =
        static_cast<float>(have_motor_feedback_ ? latest_motor_feedback_.feedback_age_ms : 0.0);
    p.left_actual_steps = static_cast<int32_t>(have_motor_feedback_ ? latest_motor_feedback_.left_actual_steps : 0);
    p.right_actual_steps = static_cast<int32_t>(have_motor_feedback_ ? latest_motor_feedback_.right_actual_steps : 0);
    p.actuator_saturation_flags =
        have_motor_feedback_ ? latest_motor_feedback_.actuator_saturation_flags : 0u;
    bus_.publish<MsgId::SystemTelemetry>(p);
  });
}

}  // namespace sil
