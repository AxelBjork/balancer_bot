#pragma once

#include <memory>
#include <atomic>
#include <thread>

#include "publisher.h"
#include "messages/balancer_msgs.h"

class XboxController;

namespace sil {

inline constexpr char kInputServiceDoc[] =
    "Reads input from a hardware Xbox controller and publishes normalized `JoystickCommand` "
    "messages to the bus.\n\n"
    "This service isolates the platform-dependent gamepad reading (SDL2) from the main "
    "application logic. It runs a dedicated worker thread at a fixed cadence, polling "
    "the controller state and emitting standard forward/turn commands. This allows the "
    "controller to be replaced or simulated by external sources like Python tests or "
    "UDP injection without modifying the balancing application.";

class DOC_DESC(kInputServiceDoc) InputService {
 public:
  static constexpr const char* kDocDescription = kInputServiceDoc;

  using Publishes = ipc::MsgList<MsgId::JoystickCommand>;

  explicit InputService(ipc::MessageBus& bus);
  ~InputService();

  void start();
  void stop();

 private:
  ipc::TypedPublisher<InputService> bus_;
  std::unique_ptr<XboxController> pad_;
  std::atomic<bool> running_{false};
  std::thread worker_;

  void run();
};

}  // namespace sil
