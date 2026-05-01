#pragma once

#include "messages/balancer_msgs.h"
#include "motor_runner.h"
#include "publisher.h"

namespace sil {

inline constexpr char kMotorServiceDoc[] =
    "Implements the actuator boundary between reflected IPC commands and the low-level motor "
    "runner.\n\n"
    "The service subscribes only to `MotorTargets` and deliberately contains almost no control "
    "state of its own. Its job is to accept wheel-speed targets expressed in steps per second and "
    "forward them to the configured `MotorRunner` if one is attached:\n\n"
    "$$ u_L, u_R \\; [\\mathrm{steps/s}] \\rightarrow \\texttt{MotorRunner::setTargets}(u_L, u_R) "
    "$$\n\n"
    "Keeping this service narrow is intentional. Closed-loop balance, trim estimation, and "
    "telemetry all remain in `ControlService` and `RateControllerCore`, while hardware-specific "
    "pulse generation, slew limiting, and direction control remain below this layer in the motor "
    "runner. When hardware is present the service also republishes the runner's applied rate, "
    "steps-derived average speed estimate, and integrated step state as `MotorFeedback`, which "
    "lets "
    "`ControlService` use the real actuator state instead of assuming the last commanded target "
    "was "
    "achieved. In SIL or unit-test "
    "configurations the runner pointer may be null, allowing the bus and controller stack to "
    "execute without requiring a physical motor backend.";

class DOC_DESC(kMotorServiceDoc) MotorService {
 public:
  static constexpr const char* kDocDescription = kMotorServiceDoc;

  using Publishes = ipc::MsgList<MsgId::MotorFeedback>;
  using Subscribes = ipc::MsgList<MsgId::MotorTargets>;

  explicit MotorService(ipc::MessageBus& bus, MotorRunner* runner) : bus_(bus), runner_(runner) {
  }
  ~MotorService() = default;

  void start() {
  }
  void stop() {
  }

  template <MsgId Id>
  void on_message(const typename MessageTraits<Id>::Payload& p) {
  }

 private:
  ipc::TypedPublisher<MotorService> bus_;
  MotorRunner* runner_;
};

template <>
inline void MotorService::on_message<MsgId::MotorTargets>(const ipc::MotorTargetsPayload& p) {
  if (runner_) {
    runner_->setTargets(p.left_sps, p.right_sps);
    const auto feedback = runner_->getFeedbackSample();
    ipc::MotorFeedbackPayload payload{};
    payload.left_applied_sps = feedback.left_applied_sps;
    payload.right_applied_sps = feedback.right_applied_sps;
    payload.measured_avg_sps = feedback.measured_avg_sps;
    payload.left_actual_steps = feedback.left_actual_steps;
    payload.right_actual_steps = feedback.right_actual_steps;
    bus_.publish<MsgId::MotorFeedback>(payload);
  }
}

}  // namespace sil
