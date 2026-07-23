#pragma once

#include <algorithm>
#include <cmath>

#include "services/main/config.h"
#include "messages/balancer_msgs.h"
#include "publisher.h"
#include "services/control/rate_controller_core.h"

namespace sil {

inline constexpr char kControlServiceDoc[] =
    "Owns the balancing control pipeline that converts `PhysicsTick`, `ImuData`, and "
    "`JoystickCommand`, and `MotorFeedback` inputs into wheel-speed targets and streaming "
    "controller "
    "telemetry.\n\n"
    "A ramped joystick command supplies a governed wheel-speed reference. At 50 Hz, velocity "
    "error and target acceleration form the pitch reference, while a bounded integral term learns "
    "only the stationary center-of-mass trim:\n\n"
    "$$ \\theta_{sp} = k_{vp}(v_{ref} - v) "
    "+ \\operatorname{atan2}(a_{ref}s_m,g) + \\theta_{COM} $$\n\n"
    "$$ \\omega_{sp} = k_{pitch}(\\theta_{sp} - \\theta) - k_{pitch\\_rate}\\dot{\\theta} $$\n\n"
    "The pitch-rate controller supplies the wheel command before turn allocation. Motor output can "
    "initially reduce or reverse to acquire lean. Faults clear dynamic state but preserve bounded "
    "COM trim. Telemetry reports the pitch-reference terms, commands, feedback, saturation, and "
    "faults.";

class DOC_DESC(kControlServiceDoc) ControlService {
 public:
  static constexpr const char* kDocDescription = kControlServiceDoc;

  using Publishes = ipc::MsgList<MsgId::MotorTargets, MsgId::SystemTelemetry>;
  using Subscribes = ipc::MsgList<MsgId::PhysicsTick, MsgId::ImuData, MsgId::JoystickCommand,
                                  MsgId::MotorFeedback>;

  explicit ControlService(ipc::MessageBus& bus);
  ~ControlService() = default;

  void start() {
  }
  void stop() {
  }

  template <MsgId Id>
  void on_message(const typename MessageTraits<Id>::Payload& p) {
  }

 private:
  ipc::TypedPublisher<ControlService> bus_;
  RateControllerCore core_;

  double last_left_sps_ = 0.0;
  double last_right_sps_ = 0.0;
  ipc::MotorFeedbackPayload latest_motor_feedback_{};
  bool have_motor_feedback_ = false;
  double observed_velocity_sps_ = 0.0;
  double filtered_velocity_sps_ = 0.0;
  double last_raw_acc_pitch_deg_ = 0.0;
  double last_fused_pitch_deg_ = 0.0;
  double last_gyro_pitch_rate_dps_ = 0.0;
  double last_filtered_pitch_rate_dps_ = 0.0;
  bool have_feedback_reference_ = false;
  int64_t feedback_left_steps_ = 0;
  int64_t feedback_right_steps_ = 0;
  double feedback_pitch_rad_ = 0.0;
};

template <>
inline void ControlService::on_message<MsgId::JoystickCommand>(
    const ipc::JoystickCommandPayload& p) {
  JoyCmd cmd{p.forward, p.turn};
  core_.setJoystick(cmd);
}

template <>
inline void ControlService::on_message<MsgId::ImuData>(const ipc::ImuSamplePayload& p) {
  ImuSample s{};
  s.angle_rad = p.pitch_rad;
  s.gyro_rad_s = p.pitch_rate_rad_s;
  s.pitch_accel_rad_s2 = p.pitch_accel_rad_s2;
  s.yaw_rate_z = p.gyr[2];
  s.t = std::chrono::steady_clock::time_point(std::chrono::microseconds(p.timestamp_us));
  const double ax = p.acc[0];
  const double az = p.acc[2];
  last_raw_acc_pitch_deg_ = std::atan2(-ax, -az) * (180.0 / M_PI);
  last_fused_pitch_deg_ = p.pitch_rad * (180.0 / M_PI);
  last_gyro_pitch_rate_dps_ = p.gyr[1] * (180.0 / M_PI);
  last_filtered_pitch_rate_dps_ = p.pitch_rate_rad_s * (180.0 / M_PI);
  core_.pushImu(s);
}

template <>
inline void ControlService::on_message<MsgId::PhysicsTick>(const PhysicsTickPayload& p) {
  const auto now = std::chrono::steady_clock::time_point(std::chrono::microseconds(p.timestamp_us));
  core_.step(p.dt_s, now);
}

template <>
inline void ControlService::on_message<MsgId::MotorFeedback>(const ipc::MotorFeedbackPayload& p) {
  latest_motor_feedback_ = p;
  const double dt_s = std::max(0.0, p.update_dt_ms / 1000.0);
  // Completed steps are motor-relative. Correct common-mode displacement with
  // the paired fused-pitch displacement before filtering it as axle velocity.
  if (!have_feedback_reference_) {
    feedback_left_steps_ = p.left_actual_steps;
    feedback_right_steps_ = p.right_actual_steps;
    feedback_pitch_rad_ = last_fused_pitch_deg_ * M_PI / 180.0;
    have_feedback_reference_ = true;
    observed_velocity_sps_ = p.measured_avg_sps;
  } else if (dt_s > 0.0) {
    const double motor_delta_steps = 0.5 * static_cast<double>(
        (p.left_actual_steps - feedback_left_steps_) +
        (p.right_actual_steps - feedback_right_steps_));
    const double pitch_rad = last_fused_pitch_deg_ * M_PI / 180.0;
    const double pitch_steps = Config::steps_per_rev * (pitch_rad - feedback_pitch_rad_) /
                               (2.0 * M_PI);
    observed_velocity_sps_ = (motor_delta_steps + pitch_steps) / dt_s;
    feedback_left_steps_ = p.left_actual_steps;
    feedback_right_steps_ = p.right_actual_steps;
    feedback_pitch_rad_ = pitch_rad;
  }
  if (!have_motor_feedback_ || Config::fc_velocity_hz <= 0.0 || dt_s <= 0.0) {
    filtered_velocity_sps_ = observed_velocity_sps_;
  } else {
    const double alpha = std::exp(-2.0 * M_PI * Config::fc_velocity_hz * dt_s);
    filtered_velocity_sps_ = alpha * filtered_velocity_sps_ + (1.0 - alpha) * observed_velocity_sps_;
  }
  core_.setMotorFeedback(filtered_velocity_sps_, p.actuator_fault != 0);
  have_motor_feedback_ = true;
}

}  // namespace sil
