#include "services/control/control_service.h"

#include <chrono>
#include <unistd.h>

namespace {

uint32_t make_run_id() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  const auto ticks = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
  const auto pid = static_cast<uint32_t>(::getpid());
  const uint32_t value = static_cast<uint32_t>(ticks) ^ static_cast<uint32_t>(ticks >> 32) ^ pid;
  return value == 0 ? 1u : value;
}

}  // namespace

namespace sil {

ControlService::ControlService(ipc::MessageBus& bus) : bus_(bus), run_id_(make_run_id()) {
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
    p.run_id = run_id_;
    p.packet_seq = ++packet_seq_;
    p.loop_seq = loop_seq_;
    p.sender_monotonic_ns = current_tick_timestamp_us_ * 1000ULL;
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
    p.trim_learning_enabled = t.trim_learning_enabled ? 1u : 0u;
    p.trim_learning_block_reason = t.trim_learning_block_reason;
    p.trim_learning_reserved = 0u;
    p.pitch_feedback_sps = static_cast<float>(t.pitch_feedback_sps);
    p.pitch_rate_feedback_sps = static_cast<float>(t.pitch_rate_feedback_sps);
    p.pitch_accel_feedback_sps = static_cast<float>(t.pitch_accel_feedback_sps);
    p.velocity_pitch_target_deg = static_cast<float>(t.velocity_pitch_target_deg);
    p.balance_unclamped_sps = static_cast<float>(t.balance_unclamped_sps);
    p.active_pitch_gain_sps_per_rad = static_cast<float>(t.active_pitch_gain_sps_per_rad);
    p.active_pitch_rate_gain_sps_per_rad_s =
        static_cast<float>(t.active_pitch_rate_gain_sps_per_rad_s);
    p.active_pitch_accel_gain_sps_per_rad_s2 =
        static_cast<float>(t.active_pitch_accel_gain_sps_per_rad_s2);
    p.active_velocity_pitch_gain_rad_per_sps =
        static_cast<float>(t.active_velocity_pitch_gain_rad_per_sps);
    p.active_velocity_control_cutoff_hz =
        static_cast<float>(t.active_velocity_control_cutoff_hz);
    p.active_velocity_observer_cutoff_hz =
        static_cast<float>(t.active_velocity_observer_cutoff_hz);
    p.active_com_trim_gain_deg_per_sps_s =
        static_cast<float>(t.active_com_trim_gain_deg_per_sps_s);
    p.active_com_trim_limit_deg = static_cast<float>(t.active_com_trim_limit_deg);
    p.active_velocity_pitch_limit_deg =
        static_cast<float>(t.active_velocity_pitch_limit_deg);
    p.active_accel_lpf_hz = static_cast<float>(t.active_accel_lpf_hz);
    p.active_gyro_lpf_hz = static_cast<float>(t.active_gyro_lpf_hz);
    p.active_gyro_derivative_lpf_hz = static_cast<float>(t.active_gyro_derivative_lpf_hz);
    p.active_config_generation = t.active_config_generation;
    p.velocity_pitch_request_unclamped_deg =
        static_cast<float>(t.velocity_pitch_request_unclamped_deg);
    p.velocity_pitch_request_limited_deg =
        static_cast<float>(t.velocity_pitch_request_limited_deg);
    p.pitch_target_unclamped_deg = static_cast<float>(t.pitch_target_unclamped_deg);
    p.trim_quiet_rate_rms_dps = static_cast<float>(t.trim_quiet_rate_rms_dps);
    p.velocity_authority_limited = t.velocity_authority_limited;
    p.trim_trusted = t.trim_trusted;
    p.trim_learning_allowed = t.trim_learning_allowed;
    p.pitch_target_limit_reason = t.pitch_target_limit_reason;
    p.velocity_control_sps = static_cast<float>(t.velocity_control_sps);
    p.pitch_authority_diagnostic_active = t.pitch_authority_diagnostic_active;
    p.pitch_authority_diagnostic_target_deg =
        static_cast<float>(t.pitch_authority_diagnostic_target_deg);
    p.pitch_authority_diagnostic_com_trim_deg =
        static_cast<float>(t.pitch_authority_diagnostic_com_trim_deg);
    p.pitch_authority_diagnostic_remaining_s =
        static_cast<float>(t.pitch_authority_diagnostic_remaining_s);
    p.pitch_authority_diagnostic_request_id =
        t.pitch_authority_diagnostic_request_id;
    p.pitch_authority_diagnostic_command_age_ms =
        static_cast<float>(t.pitch_authority_diagnostic_command_age_ms);
    p.completed_step_acceleration_sps2 =
        static_cast<float>(t.completed_step_acceleration_sps2);
    p.user_velocity_mps = static_cast<float>(t.user_velocity_mps);
    p.reference_velocity_mps = static_cast<float>(t.reference_velocity_mps);
    p.reference_acceleration_mps2 = static_cast<float>(t.reference_acceleration_mps2);
    p.velocity_feedback_estimate_mps = static_cast<float>(t.velocity_feedback_estimate_mps);
    p.velocity_error_mps = static_cast<float>(t.velocity_error_mps);
    p.velocity_feedback_acceleration_mps2 =
        static_cast<float>(t.velocity_feedback_acceleration_mps2);
    p.acceleration_raw_mps2 = static_cast<float>(t.acceleration_raw_mps2);
    p.acceleration_cmd_mps2 = static_cast<float>(t.acceleration_cmd_mps2);
    p.drive_pitch_target_deg = static_cast<float>(t.drive_pitch_target_deg);
    p.fixed_com_trim_deg = static_cast<float>(t.fixed_com_trim_deg);
    p.velocity_feedback_valid = t.velocity_feedback_valid;
    p.velocity_feedback_active = t.velocity_feedback_active;
    p.outer_acceleration_limited = t.outer_acceleration_limited;
    p.outer_pitch_target_limited = t.outer_pitch_target_limited;
    p.active_drive_max_velocity_mps = static_cast<float>(t.active_drive_max_velocity_mps);
    p.active_drive_max_acceleration_mps2 =
        static_cast<float>(t.active_drive_max_acceleration_mps2);
    p.active_drive_max_deceleration_mps2 =
        static_cast<float>(t.active_drive_max_deceleration_mps2);
    p.active_velocity_gain_per_s = static_cast<float>(t.active_velocity_gain_per_s);
    p.active_velocity_feedback_cutoff_hz =
        static_cast<float>(t.active_velocity_feedback_cutoff_hz);
    p.active_outer_pitch_limit_deg = static_cast<float>(t.active_outer_pitch_limit_deg);
    p.active_fixed_com_trim_deg = static_cast<float>(t.active_fixed_com_trim_deg);
    p.adaptive_com_trim_enabled = t.adaptive_com_trim_enabled;
    p.legacy_outer_fields_valid = t.legacy_outer_fields_valid;
    p.reference_jerk_mps3 = static_cast<float>(t.reference_jerk_mps3);
    p.velocity_p_acceleration_mps2 = static_cast<float>(t.velocity_p_acceleration_mps2);
    p.velocity_i_acceleration_mps2 = static_cast<float>(t.velocity_i_acceleration_mps2);
    p.velocity_integral_state_mps_s = static_cast<float>(t.velocity_integral_state_mps_s);
    p.final_pitch_target_deg = static_cast<float>(t.final_pitch_target_deg);
    p.active_planner_max_acceleration_mps2 =
        static_cast<float>(t.active_planner_max_acceleration_mps2);
    p.active_planner_max_deceleration_mps2 =
        static_cast<float>(t.active_planner_max_deceleration_mps2);
    p.active_planner_max_jerk_mps3 = static_cast<float>(t.active_planner_max_jerk_mps3);
    p.active_velocity_i_gain_per_s2 = static_cast<float>(t.active_velocity_i_gain_per_s2);
    p.active_velocity_i_leak_time_s = static_cast<float>(t.active_velocity_i_leak_time_s);
    p.active_velocity_i_acceleration_limit_mps2 =
        static_cast<float>(t.active_velocity_i_acceleration_limit_mps2);
    p.planner_acceleration_limited = t.planner_acceleration_limited;
    p.planner_jerk_limited = t.planner_jerk_limited;
    p.velocity_integral_limited = t.velocity_integral_limited;
    p.velocity_anti_windup_active = t.velocity_anti_windup_active;
    bus_.publish<MsgId::SystemTelemetry>(p);
  });
}

}  // namespace sil
