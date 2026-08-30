#pragma once

#include <cmath>
#include <cstdint>
#include <functional>
#include <utility>

#include "messages/balancer_msgs.h"
#include "publisher.h"
#include "services/control/rate_controller_core.h"
#include "services/main/config_pid_io.h"

namespace sil {

inline constexpr char kControlServiceDoc[] =
    "Owns the balancing control pipeline that converts `PhysicsTick`, `ImuData`, "
    "`JoystickCommand`, and `MotorFeedback` inputs into "
    "wheel-speed targets and streaming "
    "controller "
    "telemetry.\n\n"
    "At 100 Hz, completed common-mode steps are corrected for chassis pitch to observe axle "
    "velocity and filtered at 10 Hz. A separate configurable velocity-control filter can add "
    "a slower velocity-feedback pole without changing that observer. A reversal-aware velocity "
    "reference planner produces v_ref and a_ref; velocity feedback then contributes to one "
    "shared acceleration authority before conversion to a motion pitch target:\n\n"
    "$$ a_{raw} = a_{ref} + K_v(v_{ref} - v_{feedback}), \\quad "
    "a_{cmd} = \\mathrm{clamp}(a_{raw}, \\pm g\\tan(\\theta_{limit})) $$\n\n"
    "$$ \\theta_{sp} = \\mathrm{atan2}(a_{cmd},g) + \\theta_{COM} $$\n\n"
    "The optional adaptive COM learner is disabled in the v1 configuration; fixed COM trim is "
    "the only default trim input.\n\n"
    "The attitude controller uses independently configured state feedback at the robot-forward "
    "motor boundary:\n\n"
    "$$ u = K_{pitch}(\\theta-\\theta_{sp}) + K_{rate}\\dot{\\theta} "
    "+ K_{accel}\\ddot{\\theta} $$\n\n"
    "The signs above account for the electrical/motor-boundary polarity; they are equivalent to "
    "the public negative-feedback form in the controller's internal rate convention. The explicit "
    "terms are independently tunable and reported in telemetry. The attitude controller supplies "
    "a balance correction around nominal drive before final common limiting and turn allocation. "
    "Balance can therefore cancel or reverse drive. Motor output can "
    "initially reduce or reverse to acquire lean. Faults clear dynamic state but preserve bounded "
    "COM trim. Telemetry reports the pitch-reference terms, target/post-slew/applied commands, "
    "feedback, saturation, and faults. In `actuator_saturation_flags`, bit 0 is left slew "
    "limiting and bit 1 is right slew limiting.";

class DOC_DESC(kControlServiceDoc) ControlService {
 public:
  static constexpr const char* kDocDescription = kControlServiceDoc;

  using Publishes = ipc::MsgList<MsgId::MotorTargets, MsgId::SystemTelemetry,
                                 MsgId::PidConfigStatus>;
using Subscribes = ipc::MsgList<MsgId::PhysicsTick, MsgId::ImuData, MsgId::JoystickCommand,
                                  MsgId::MotorFeedback, MsgId::PidConfigOverride>;

  explicit ControlService(ipc::MessageBus& bus);
  ~ControlService() = default;

  void start() {
  }
  void stop() {
  }

  // Simulator-only A/B hook for controller experiments. It is not wired to
  // production messages or configuration; defaults keep both corrected paths.
  void setSimulationOuterLoopOptions(bool endpoint_continuity_enabled,
                                     bool matched_reference_filter_enabled) {
    core_.setSimulationOuterLoopOptions(endpoint_continuity_enabled,
                                         matched_reference_filter_enabled);
  }

  // Simulator-only A/B hook for the moving-operating-point command
  // composition. It is not exposed through production configuration or IPC.
  void setSimulationDriveFeedforwardEnabled(bool enabled) {
    core_.setSimulationDriveFeedforwardEnabled(enabled);
  }

  // Simulator-only recovery-fixture gate; not exposed through production
  // configuration or IPC.
  void setSimulationControllerEnabled(bool enabled) {
    core_.setSimulationControllerEnabled(enabled);
  }

  // Provides the simulator with in-process canonical telemetry that is not
  // part of the append-only wire payload. Hardware has no caller for this.
  void setSimulationTelemetrySink(std::function<void(const Telemetry&)> sink) {
    simulation_telemetry_sink_ = std::move(sink);
  }

  template <MsgId Id>
  void on_message(const typename MessageTraits<Id>::Payload& p) {
  }

 private:
  ipc::TypedPublisher<ControlService> bus_;
  RateControllerCore core_;

  const uint32_t run_id_;
  uint64_t loop_seq_ = 0;
  uint64_t packet_seq_ = 0;
  uint64_t current_tick_timestamp_us_ = 0;

  double last_left_sps_ = 0.0;
  double last_right_sps_ = 0.0;
  ipc::MotorFeedbackPayload latest_motor_feedback_{};
  bool have_motor_feedback_ = false;
  double last_raw_acc_pitch_deg_ = 0.0;
  double last_fused_pitch_deg_ = 0.0;
  double last_gyro_pitch_rate_dps_ = 0.0;
  double last_filtered_pitch_rate_dps_ = 0.0;
  std::function<void(const Telemetry&)> simulation_telemetry_sink_;
};

template <>
inline void ControlService::on_message<MsgId::JoystickCommand>(
    const ipc::JoystickCommandPayload& p) {
  core_.setJoystick(JoyCmd{p.forward, p.turn});
}

template <>
inline void ControlService::on_message<MsgId::PidConfigOverride>(
    const ipc::PidConfigOverridePayload& p) {
  bus_.publish<MsgId::PidConfigStatus>(apply_pid_config_override(p));
}

template <>
inline void ControlService::on_message<MsgId::ImuData>(const ipc::ImuSamplePayload& p) {
  const double ax = p.acc[0];
  const double az = p.acc[2];
  last_raw_acc_pitch_deg_ = std::atan2(-ax, -az) * (180.0 / M_PI);
  last_gyro_pitch_rate_dps_ = p.gyr[1] * (180.0 / M_PI);
  if (!p.estimate_valid) {
    last_fused_pitch_deg_ = 0.0;
    last_filtered_pitch_rate_dps_ = 0.0;
    core_.clearImu();
    return;
  }

  ImuSample s{};
  s.angle_rad = p.pitch_rad;
  s.gyro_rad_s = p.pitch_rate_rad_s;
  s.pitch_accel_rad_s2 = p.pitch_accel_rad_s2;
  s.yaw_rate_z = p.gyr[2];
  s.t = std::chrono::steady_clock::time_point(std::chrono::microseconds(p.timestamp_us));
  last_fused_pitch_deg_ = p.pitch_rad * (180.0 / M_PI);
  last_filtered_pitch_rate_dps_ = p.pitch_rate_rad_s * (180.0 / M_PI);
  core_.pushImu(s);
}

template <>
inline void ControlService::on_message<MsgId::PhysicsTick>(const PhysicsTickPayload& p) {
  ++loop_seq_;
  current_tick_timestamp_us_ = p.timestamp_us;
  const auto now = std::chrono::steady_clock::time_point(std::chrono::microseconds(p.timestamp_us));
  core_.step(p.dt_s, now);
}

template <>
inline void ControlService::on_message<MsgId::MotorFeedback>(const ipc::MotorFeedbackPayload& p) {
  latest_motor_feedback_ = p;
  core_.setMotorFeedback(p.left_actual_steps, p.right_actual_steps, p.actuator_fault != 0);
  have_motor_feedback_ = true;
}

}  // namespace sil
