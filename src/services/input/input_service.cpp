#include "services/input/input_service.h"
#include "services/input/xbox_controller.h"
#include "services/main/config.h"

namespace sil {

InputService::InputService(ipc::MessageBus& bus)
    : bus_(bus) {
  try {
    pad_ = std::make_unique<XboxController>();
  } catch (...) {
    // Gamepad not found or SDL init failed
  }
}

InputService::~InputService() {
  stop();
}

void InputService::start() {
  if (running_.exchange(true)) return;
  // Establish Xbox priority before the UDP bridge can deliver an external command. This
  // preserves strict source arbitration even if the worker has not reached its first tick yet.
  arbiter_.clear();
  xbox_available_.store(pad_ && pad_->isAvailable(), std::memory_order_release);
  worker_ = std::thread(&InputService::run, this);
}

void InputService::stop() {
  if (!running_.exchange(false)) return;
  xbox_available_.store(false, std::memory_order_release);
  arbiter_.clear();
  if (worker_.joinable()) {
    worker_.join();
  }
  publish_command({0.0, 0.0});
}

void InputService::accept_external_command(const ipc::JoystickCommandPayload& payload) {
  if (!running_.load(std::memory_order_acquire)) return;
  const auto now = std::chrono::steady_clock::now();
  const bool xbox_available = xbox_available_.load(std::memory_order_acquire);
  arbiter_.accept_external(payload, xbox_available, now);

  if (!running_.load(std::memory_order_acquire)) {
    arbiter_.clear();
    return;
  }

  // Publish accepted external input immediately. The periodic worker still renews the
  // resolved stream and enforces the watchdog, but a packet must not wait for the next
  // command-service tick before reaching the control loop.
  if (!xbox_available) {
    publish_command(arbiter_.resolve(false, {0.0, 0.0}, now));
  }
}

void InputService::publish_command(JoyCmd command) {
  ipc::JoystickCommandPayload payload{};
  payload.forward = command.forward;
  payload.turn = command.turn;
  bus_.publish<MsgId::JoystickCommand>(payload);
}

void InputService::run() {
  const auto tick = std::chrono::duration<double, std::milli>(1000.0 / Config::command_hz);

  while (running_) {
    const bool xbox_available = pad_ && pad_->isAvailable();
    xbox_available_.store(xbox_available, std::memory_order_release);
    JoyCmd xbox_command{0.0, 0.0};

    if (xbox_available) {
      pad_->update();
      // Arcade Drive: Left Stick Y = Forward (inverted)
      // Steering: Combined Right Stick X (standard) and Right Stick Y (combined steer)
      xbox_command.forward = -pad_->leftY();
      xbox_command.turn = pad_->rightX();
    }

    const JoyCmd command = arbiter_.resolve(xbox_available, xbox_command,
                                            std::chrono::steady_clock::now());
    publish_command(command);

    std::this_thread::sleep_for(tick);
  }
}

}  // namespace sil
