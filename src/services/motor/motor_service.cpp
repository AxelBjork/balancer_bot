#include "motor_service.h"

namespace sil {

void MotorService::handle_motor_targets(const ipc::MotorTargetsPayload& p) {
  if (!runner_) {
    return;
  }

  runner_->setTargets(p.left_sps, p.right_sps, current_tick_us_);
  const auto feedback = runner_->getFeedbackSample();

  ipc::MotorFeedbackPayload payload{};
  payload.left_slewed_sps = feedback.left_slewed_sps;
  payload.right_slewed_sps = feedback.right_slewed_sps;
  payload.measured_avg_sps = feedback.measured_avg_sps;
  payload.update_dt_ms = feedback.update_dt_ms;
  payload.feedback_age_ms = feedback.feedback_age_ms;
  payload.left_actual_steps = feedback.left_actual_steps;
  payload.right_actual_steps = feedback.right_actual_steps;
  payload.actuator_saturation_flags = feedback.actuator_saturation_flags;
  payload.actuator_fault = feedback.actuator_fault ? 1u : 0u;
  bus_.publish<MsgId::MotorFeedback>(payload);
}

template <>
void MotorService::on_message<MsgId::PhysicsTick>(const PhysicsTickPayload& p) {
  current_tick_us_ = p.timestamp_us;
}

template <>
void MotorService::on_message<MsgId::MotorTargets>(const ipc::MotorTargetsPayload& p) {
  handle_motor_targets(p);
}

}  // namespace sil
