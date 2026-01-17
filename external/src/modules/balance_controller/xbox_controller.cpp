// XboxController.cpp  (all the heavy stuff lives here)
#include "xbox_controller.h"

#include <SDL2/SDL.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

struct XboxController::Impl {
  SDL_Joystick* joystick{nullptr};
  int axis_leftX{0};
  int axis_leftY{1};
  int axis_rightX{3};
  int axis_rightY{4};
  float deadzone{0.05f};

  static float normalize(Sint16 v) {
    return static_cast<float>(v) / 32768.0f;  // [-1, 1]
  }

  float apply_deadzone(float v) const {
    float a = std::fabs(v);
    if (a < deadzone) {
      return 0.0f;
    }
    float s = (a - deadzone) / (1.0f - deadzone);
    return (v < 0.0f) ? -s : s;
  }
};

XboxController::XboxController() : impl_(std::make_unique<Impl>()) {
  if (SDL_Init(SDL_INIT_JOYSTICK) < 0) {
    throw std::runtime_error("SDL_Init failed");
  }
  if (SDL_NumJoysticks() < 1) {
    printf("No joystick found\n");
    return;
  }
  impl_->joystick = SDL_JoystickOpen(0);
  if (!impl_->joystick) {
    throw std::runtime_error("Failed to open joystick 0");
  }
}

XboxController::~XboxController() {
  if (impl_ && impl_->joystick) {
    SDL_JoystickClose(impl_->joystick);
  }
  SDL_Quit();
}

XboxController::XboxController(XboxController&& other) noexcept : impl_(std::move(other.impl_)) {
}

XboxController& XboxController::operator=(XboxController&& other) noexcept {
  if (this != &other) {
    impl_ = std::move(other.impl_);
  }
  return *this;
}

void XboxController::update() {
  SDL_JoystickUpdate();
}

void XboxController::setDeadzone(float dz) {
  impl_->deadzone = std::clamp(dz, 0.0f, 0.9f);
}

void XboxController::setAxisMap(int leftY_axis, int rightY_axis) {
  impl_->axis_leftY = leftY_axis;
  impl_->axis_rightY = rightY_axis;
}

float XboxController::leftX() const {
  Sint16 raw = SDL_JoystickGetAxis(impl_->joystick, impl_->axis_leftX);
  return impl_->apply_deadzone(Impl::normalize(raw));
}

float XboxController::rightX() const {
  Sint16 raw = SDL_JoystickGetAxis(impl_->joystick, impl_->axis_rightX);
  return impl_->apply_deadzone(Impl::normalize(raw));
}

float XboxController::leftY() const {
  Sint16 raw = SDL_JoystickGetAxis(impl_->joystick, impl_->axis_leftY);
  return impl_->apply_deadzone(Impl::normalize(raw));
}

float XboxController::rightY() const {
  Sint16 raw = SDL_JoystickGetAxis(impl_->joystick, impl_->axis_rightY);
  return impl_->apply_deadzone(Impl::normalize(raw));
}
