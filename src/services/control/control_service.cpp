#include "services/control/control_service.h"

#include <algorithm>

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
    p.t_sec = t.t_sec;
    p.age_ms = t.age_ms;
    p.pitch_deg = t.pitch_deg;
    p.pitch_rate_dps = t.pitch_rate_dps;
    p.raw_acc_pitch_deg = last_raw_acc_pitch_deg_;
    p.fused_pitch_deg = last_fused_pitch_deg_;
    p.gyro_pitch_rate_dps = last_gyro_pitch_rate_dps_;
    p.filtered_pitch_rate_dps = last_filtered_pitch_rate_dps_;
    p.u_sps = t.u_sps;
    p.turn_sps = t.turn_sps;
    p.vel_error = t.vel_error;
    p.measured_vel_sps = t.measured_vel_sps;
    p.vel_p_term = t.vel_p_term;
    p.pitch_ref_from_vel_deg = t.pitch_ref_from_vel_deg;
    p.pitch_error_deg = t.pitch_error_deg;
    p.pitch_sp_deg = t.pitch_sp_deg;
    p.pitch_trim_deg = t.pitch_trim_deg;
    p.trim_active = t.trim_active;
    p.left_applied_sps = have_motor_feedback_ ? latest_motor_feedback_.left_applied_sps : last_left_sps_;
    p.right_applied_sps =
        have_motor_feedback_ ? latest_motor_feedback_.right_applied_sps : last_right_sps_;
    p.motor_update_dt_ms = have_motor_feedback_ ? latest_motor_feedback_.update_dt_ms : 0.0;
    p.motor_feedback_age_ms = have_motor_feedback_ ? latest_motor_feedback_.feedback_age_ms : 0.0;
    p.left_actual_steps = have_motor_feedback_ ? latest_motor_feedback_.left_actual_steps : 0;
    p.right_actual_steps = have_motor_feedback_ ? latest_motor_feedback_.right_actual_steps : 0;
    p.plant_pitch_deg = t.pitch_deg;
    p.plant_pitch_rate_dps = t.pitch_rate_dps;
    bus_.publish<MsgId::SystemTelemetry>(p);
  });
}

}  // namespace sil
