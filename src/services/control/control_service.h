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
    "exists. It learns a slow lean-trim bias from persistent drift, and resets or decays that trim when the "
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
  bool have_previous_feedback_sample_ = false;
  double observed_velocity_sps_ = 0.0;
  double previous_common_actual_steps_ = 0.0;
  uint64_t previous_feedback_timestamp_us_ = 0;
  double previous_feedback_pitch_rad_ = 0.0;
  // Latest controller-facing fused pitch. Motor feedback is emitted at the current physics tick,
  // so this is the pitch at the feedback interval end rather than a pitch-rate approximation.
  double latest_fused_pitch_rad_ = 0.0;
  double last_raw_acc_pitch_deg_ = 0.0;
  double last_fused_pitch_deg_ = 0.0;
  double last_gyro_pitch_rate_dps_ = 0.0;
  double last_filtered_pitch_rate_dps_ = 0.0;
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
  latest_fused_pitch_rad_ = p.pitch_rad;
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
  // Retain the legacy 50 ms commanded-pulse estimate for diagnostics only. It must not be mixed
  // with the instantaneous pitch displacement correction below.
  observed_velocity_sps_ = -p.measured_avg_sps;
  if (have_previous_feedback_sample_ &&
      p.feedback_timestamp_us > previous_feedback_timestamp_us_) {
    const double dt_s =
        static_cast<double>(p.feedback_timestamp_us - previous_feedback_timestamp_us_) / 1e6;
    // Fixed-axle sign check: with the axle fixed, wheel pulse displacement is the negative of
    // body pitch displacement. Thus completed_common_delta_steps + pitch_delta_steps is zero.
    const double current_common_actual_steps =
        0.5 * (static_cast<double>(p.left_actual_steps) + static_cast<double>(p.right_actual_steps));
    // This is -0.5 * ((left - previous_left) + (right - previous_right)).
    const double completed_common_delta_steps =
        previous_common_actual_steps_ - current_common_actual_steps;
    const double pitch_delta_steps = Config::steps_per_rev *
        (latest_fused_pitch_rad_ - previous_feedback_pitch_rad_) / (2.0 * M_PI);
    const double corrected_axle_delta_steps = completed_common_delta_steps + pitch_delta_steps;
    core_.updateOuterLoop(corrected_axle_delta_steps / dt_s, dt_s);
  }
  previous_common_actual_steps_ =
      0.5 * (static_cast<double>(p.left_actual_steps) + static_cast<double>(p.right_actual_steps));
  previous_feedback_timestamp_us_ = p.feedback_timestamp_us;
  previous_feedback_pitch_rad_ = latest_fused_pitch_rad_;
  have_previous_feedback_sample_ = true;
  have_motor_feedback_ = true;
}

}  // namespace sil
