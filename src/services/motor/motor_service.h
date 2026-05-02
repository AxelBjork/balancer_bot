#pragma once

#include "messages/balancer_msgs.h"
#include "services/motor/motor_runner.h"
#include "publisher.h"

namespace sil {

inline constexpr char kMotorServiceDoc[] =
    "Implements the actuator boundary between reflected IPC commands and the low-level motor "
    "runner.\n\n"
    "The service subscribes to `PhysicsTick` and `MotorTargets` and deliberately contains almost "
    "no control state of its own. Its job is to remember the latest physics timestamp, accept "
    "wheel-speed targets expressed in steps per second, and forward them to the configured "
    "`MotorRunner` if one is attached:\n\n"
    "$$ u_L, u_R \\; [\\mathrm{steps/s}] \\rightarrow \\texttt{MotorRunner::setTargets}(u_L, u_R) "
    "$$\n\n"
    "Keeping this service narrow is intentional. Closed-loop balance, trim estimation, and "
    "telemetry all remain in `ControlService` and `RateControllerCore`, while hardware-specific "
    "pulse generation, slew limiting, and direction control remain below this layer in the motor "
    "runner. The service also listens for `PhysicsTick` so it can keep the runner aligned with "
    "the current physics time before forwarding motor targets. When hardware is present the "
    "service also republishes the runner's applied rate, steps-derived average speed estimate, "
    "and integrated step state as `MotorFeedback`, which "
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
  using Subscribes = ipc::MsgList<MsgId::PhysicsTick, MsgId::MotorTargets>;

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
  void handle_motor_targets(const ipc::MotorTargetsPayload& p);

  ipc::TypedPublisher<MotorService> bus_;
  MotorRunner* runner_;
  uint64_t current_tick_us_ = 0;
};

template <>
void MotorService::on_message<MsgId::PhysicsTick>(const PhysicsTickPayload& p);

template <>
void MotorService::on_message<MsgId::MotorTargets>(const ipc::MotorTargetsPayload& p);

}  // namespace sil
