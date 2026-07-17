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
    "A 50 Hz velocity PI produces the pitch reference, and the PX4 `RateControl` block "
    "tracks a damped pitch-rate setpoint:\n\n"
    "$$ \\theta_{sp} = k_{vp}(v_{ref} - v) + \\int k_{vi}(v_{ref} - v)dt $$\n\n"
    "$$ \\omega_{sp} = k_{pitch}(\\theta_{sp} - \\theta) - k_{pitch\\_rate}\\dot{\\theta} $$\n\n"
    "The target wheel speed is fed forward at the velocity-command actuator boundary. The "
    "normalized pitch-axis effort is scaled into a balance correction, clamped to the configured "
    "ceiling, and split into left/right wheel targets with balance-priority turn allocation.\n\n"
    "The service also exposes several practical adaptations that matter for balancing behavior: it "
    "uses completed-pulse feedback from `MotorService`. It resets both controller integrators on "
    "fallover, stale IMU data, or an actuator fault. Every control step also publishes "
    "`SystemTelemetry`, including fused pitch, filtered pitch rate, raw IMU diagnostics, "
    "pitch-reference decomposition, rate setpoint, controller output, wheel-speed command, "
    "per-wheel applied feedback, saturation, and actuator-fault state so the SIL harness can inspect controller "
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
  const auto now = std::chrono::steady_clock::time_point(std::chrono::microseconds(p.timestamp_us));
  core_.step(p.dt_s, now);
}

template <>
inline void ControlService::on_message<MsgId::MotorFeedback>(const ipc::MotorFeedbackPayload& p) {
  latest_motor_feedback_ = p;
  observed_velocity_sps_ = p.measured_avg_sps;
  const double dt_s = std::max(0.0, p.update_dt_ms / 1000.0);
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
