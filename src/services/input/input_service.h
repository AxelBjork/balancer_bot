#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>

#include "messages/balancer_msgs.h"
#include "messages/types.h"
#include "publisher.h"

class XboxController;

namespace sil {

inline constexpr auto kExternalJoystickWatchdog = std::chrono::milliseconds(250);

class JoystickInputArbiter {
 public:
  void accept_external(const ipc::JoystickCommandPayload& payload, bool xbox_available,
                       std::chrono::steady_clock::time_point now) {
    std::lock_guard lock(mutex_);
    if (xbox_available || !valid(payload)) {
      clear_locked();
      return;
    }
    if (payload.forward == 0.0 && payload.turn == 0.0) {
      // Zero is the explicit external-release boundary. Dashboard heartbeats are
      // nonzero packets and are refreshed before the watchdog expires.
      clear_locked();
      return;
    }
    external_ = {payload.forward, payload.turn};
    external_deadline_ = now + kExternalJoystickWatchdog;
    external_active_ = true;
  }

  JoyCmd resolve(bool xbox_available, JoyCmd xbox, std::chrono::steady_clock::time_point now) {
    std::lock_guard lock(mutex_);
    if (xbox_available) {
      clear_locked();
      return xbox;
    }
    if (external_active_ && now < external_deadline_) {
      return external_;
    }
    clear_locked();
    return {0.0, 0.0};
  }

  void clear() {
    std::lock_guard lock(mutex_);
    clear_locked();
  }

 private:
  static bool valid(const ipc::JoystickCommandPayload& payload) {
    return std::isfinite(payload.forward) && std::isfinite(payload.turn) &&
           payload.forward >= -1.0 && payload.forward <= 1.0 && payload.turn >= -1.0 &&
           payload.turn <= 1.0;
  }

  void clear_locked() {
    external_ = {0.0, 0.0};
    external_deadline_ = {};
    external_active_ = false;
  }

  std::mutex mutex_;
  JoyCmd external_{0.0, 0.0};
  std::chrono::steady_clock::time_point external_deadline_{};
  bool external_active_{false};
};

inline constexpr char kInputServiceDoc[] =
    "Arbitrates hardware and external joystick input, then publishes resolved normalized "
    "`JoystickCommand` messages to the bus.\n\n"
    "This service isolates the platform-dependent gamepad reading (SDL2) from the main "
    "application logic. An available Xbox controller has priority; otherwise a validated "
    "external command is held until its short watchdog expires or an explicit neutral "
    "command is received.";

class DOC_DESC(kInputServiceDoc) InputService {
 public:
  static constexpr const char* kDocDescription = kInputServiceDoc;

  using Subscribes = ipc::MsgList<MsgId::ExternalJoystickCommand>;
  using Publishes = ipc::MsgList<MsgId::JoystickCommand>;

  explicit InputService(ipc::MessageBus& bus);
  ~InputService();

  void start();
  void stop();

  template <MsgId Id>
  void on_message(const typename MessageTraits<Id>::Payload&) {
  }

 private:
  ipc::TypedPublisher<InputService> bus_;
  std::unique_ptr<XboxController> pad_;
  std::atomic<bool> running_{false};
  std::atomic<bool> xbox_available_{false};
  std::thread worker_;
  JoystickInputArbiter arbiter_;

  void accept_external_command(const ipc::JoystickCommandPayload& payload);
  void publish_command(JoyCmd command);
  void run();
};

template <>
inline void InputService::on_message<MsgId::ExternalJoystickCommand>(
    const ipc::JoystickCommandPayload& payload) {
  accept_external_command(payload);
}

}  // namespace sil
