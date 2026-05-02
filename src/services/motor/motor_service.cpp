#include "motor_service.h"

namespace sil {

void MotorService::handle_motor_targets(const ipc::MotorTargetsPayload& p) {
  if (!runner_) {
    return;
  }

  runner_->setTargets(p.left_sps, p.right_sps, current_tick_us_);
  const auto feedback = runner_->getFeedbackSample();

  ipc::MotorFeedbackPayload payload{};
  payload.left_applied_sps = feedback.left_applied_sps;
  payload.right_applied_sps = feedback.right_applied_sps;
  payload.measured_avg_sps = feedback.measured_avg_sps;
  payload.update_dt_ms = feedback.update_dt_ms;
  payload.feedback_age_ms = feedback.feedback_age_ms;
  payload.left_actual_steps = feedback.left_actual_steps;
  payload.right_actual_steps = feedback.right_actual_steps;
  bus_.publish<MsgId::MotorFeedback>(payload);
}

template <>
void MotorService::on_message<MsgId::PhysicsTick>(const PhysicsTickPayload& p) {
  current_tick_us_ = p.sim_time_us;
}

template <>
void MotorService::on_message<MsgId::MotorTargets>(const ipc::MotorTargetsPayload& p) {
  handle_motor_targets(p);
}

}  // namespace sil
