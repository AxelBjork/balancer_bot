#include "input_service.h"
#include "xbox_controller.h"
#include "config.h"

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
  worker_ = std::thread(&InputService::run, this);
}

void InputService::stop() {
  if (!running_.exchange(false)) return;
  if (worker_.joinable()) {
    worker_.join();
  }
}

void InputService::run() {
  const auto tick = std::chrono::duration<double, std::milli>(1000.0 / Config::command_hz);
  
  while (running_) {
    float ly = 0.0f;
    float rx = 0.0f;

    if (pad_ && pad_->isAvailable()) {
      pad_->update();
      // Arcade Drive: Left Stick Y = Forward (inverted)
      // Steering: Combined Right Stick X (standard) and Right Stick Y (combined steer)
      ly = -pad_->leftY();
      rx = pad_->rightX() - pad_->rightY();
      rx = std::clamp(rx, -1.0f, 1.0f);
    }

    ipc::JoystickCommandPayload j{};
    j.forward = static_cast<double>(ly);
    j.turn = static_cast<double>(rx);
    bus_.publish<MsgId::JoystickCommand>(j);

    std::this_thread::sleep_for(tick);
  }
}

}  // namespace sil
