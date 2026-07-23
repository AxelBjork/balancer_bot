#pragma once

#include <algorithm>
#include <cmath>

#include "messages/balancer_msgs.h"
#include "publisher.h"
#include "services/control/rate_controller_core.h"
#include "services/main/config.h"

namespace sil {

inline constexpr char kControlServiceDoc[] =
    "Owns the balancing control pipeline that converts `PhysicsTick`, `ImuData`, and "
    "`JoystickCommand`, and `MotorFeedback` inputs into wheel-speed targets and streaming "
    "controller "
    "telemetry.\n\n"
    "This service is intentionally thin: it caches the latest bus inputs, translates them into the "
    "`RateControllerCore` API, and republishes the core's outputs as reflected IPC payloads. The "
    "control law itself is a physics-shaped outer loop wrapped around the PX4 pitch-rate "
    "controller. A joystick forward command produces a target wheel velocity in steps per second. "
    "The velocity term is combined with lean-trim and angle-trim biases are "
    "added, and the PX4 `RateControl` block tracks a damped pitch-rate setpoint:\n\n"
    "$$ \\theta_{sp} = k_{vel}(v_{ref} - v) + \\theta_{trim} $$\n\n"
    "$$ \\omega_{sp} = k_{pitch}(\\theta_{sp} - \\theta) - k_{pitch\\_rate}\\dot{\\theta} $$\n\n"
    "The resulting normalized pitch-axis effort is scaled into motor commands in steps per second, "
    "clamped to the configured ceiling, and split into left/right wheel targets by adding a turn "
    "term.\n\n"
    "The service also exposes several practical adaptations that matter for balancing behavior: it "
    "uses real motor feedback from `MotorService` whenever it is available and falls back to the "
    "last commanded wheel speeds only in SIL-style configurations where no hardware feedback "
    "exists. It learns a slow lean-trim bias from persistent drift, and resets or decays that trim "
    "when the "
    "robot is highly tilted or actively commanded. Every control step also publishes "
    "`SystemTelemetry`, including fused pitch, filtered pitch rate, raw IMU diagnostics, "
    "pitch-reference decomposition, rate setpoint, controller output, wheel-speed command, "
    "per-wheel applied feedback, and trim state so the SIL harness can inspect controller "
    "internals without attaching directly to the core.";

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

  // Fallback proxy for SIL when no explicit motor feedback is available.
  double last_left_sps_ = 0.0;
  double last_right_sps_ = 0.0;
  ipc::MotorFeedbackPayload latest_motor_feedback_{};
  bool have_motor_feedback_ = false;
  double observed_velocity_sps_ = 0.0;
  double filtered_velocity_sps_ = 0.0;
  double velocity_observer_accum_s_ = 0.0;
  double last_raw_acc_pitch_deg_ = 0.0;
  double last_fused_pitch_deg_ = 0.0;
  double last_gyro_pitch_rate_dps_ = 0.0;
  double last_filtered_pitch_rate_dps_ = 0.0;
  // The stepper encoder measures motor-relative wheel motion.  A body pitch
  // rotation therefore appears as common-mode wheel motion even with a fixed
  // axle.  Keep a paired pitch/count sample to remove that kinematic term.
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
  const double ay = p.acc[1];
  const double az = p.acc[2];
  last_raw_acc_pitch_deg_ = std::atan2(-ax, std::sqrt(ay * ay + az * az)) * (180.0 / M_PI);
  last_fused_pitch_deg_ = p.pitch_rad * (180.0 / M_PI);
  last_gyro_pitch_rate_dps_ = p.gyr[1] * (180.0 / M_PI);
  last_filtered_pitch_rate_dps_ = p.pitch_rate_rad_s * (180.0 / M_PI);
  core_.pushImu(s);
}

template <>
inline void ControlService::on_message<MsgId::PhysicsTick>(const PhysicsTickPayload& p) {
  const auto now = std::chrono::steady_clock::time_point(std::chrono::microseconds(p.sim_time_us));
  core_.step(p.dt_s, now);
}

template <>
inline void ControlService::on_message<MsgId::MotorFeedback>(const ipc::MotorFeedbackPayload& p) {
  latest_motor_feedback_ = p;
  const double dt_s = p.update_dt_ms / 1000.0;
  // MotorRunner's positive step convention is opposite the controller's
  // forward convention, hence the leading minus.  For q_dot = u_dot/r -
  // theta_dot, this makes pitch contamination +steps_per_rev*delta_theta/(2pi*dt).
  if (!have_feedback_reference_) {
    feedback_left_steps_ = p.left_actual_steps;
    feedback_right_steps_ = p.right_actual_steps;
    feedback_pitch_rad_ = last_fused_pitch_deg_ * M_PI / 180.0;
    have_feedback_reference_ = true;
    observed_velocity_sps_ = -p.measured_avg_sps;
  } else if (dt_s > 0.0) {
    const int64_t left_delta_steps = p.left_actual_steps - feedback_left_steps_;
    const int64_t right_delta_steps = p.right_actual_steps - feedback_right_steps_;
    const double raw_sps = -0.5 * static_cast<double>(left_delta_steps + right_delta_steps) / dt_s;
    const double pitch_delta_rad = last_fused_pitch_deg_ * M_PI / 180.0 - feedback_pitch_rad_;
    const double pitch_contamination_sps =
        Config::steps_per_rev * pitch_delta_rad / (2.0 * M_PI * dt_s);
    // Retain compatibility with legacy feedback publishers that only populate
    // measured_avg_sps and leave integrated counts fixed.
    observed_velocity_sps_ =
        (left_delta_steps == 0 && right_delta_steps == 0 && std::abs(p.measured_avg_sps) > 1e-9)
            ? -p.measured_avg_sps
            : raw_sps - pitch_contamination_sps;
    feedback_left_steps_ = p.left_actual_steps;
    feedback_right_steps_ = p.right_actual_steps;
    feedback_pitch_rad_ = last_fused_pitch_deg_ * M_PI / 180.0;
  }
  if (!have_motor_feedback_) {
    filtered_velocity_sps_ = observed_velocity_sps_;
  }
  const double observer_dt_s = std::max(0.0, dt_s);
  velocity_observer_accum_s_ += observer_dt_s;
  const double period_s = (Config::fc_velocity_hz > 0.0) ? (1.0 / Config::fc_velocity_hz) : 0.0;
  if (!have_motor_feedback_ || period_s <= 0.0 || velocity_observer_accum_s_ + 1e-12 >= period_s) {
    filtered_velocity_sps_ = observed_velocity_sps_;
    velocity_observer_accum_s_ = 0.0;
  }
  core_.updateOuterLoop(have_motor_feedback_ ? filtered_velocity_sps_ : 0.0, observer_dt_s);
  have_motor_feedback_ = true;
}

}  // namespace sil
