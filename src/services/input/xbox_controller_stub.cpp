#include "services/input/xbox_controller.h"

#include <algorithm>
#include <utility>

struct XboxController::Impl {
  float deadzone{0.05f};
  int axis_leftY{1};
  int axis_rightY{4};
};

XboxController::XboxController() : impl_(std::make_unique<Impl>()) {
}

XboxController::~XboxController() = default;

XboxController::XboxController(XboxController&& other) noexcept : impl_(std::move(other.impl_)) {
}

XboxController& XboxController::operator=(XboxController&& other) noexcept {
  if (this != &other) {
    impl_ = std::move(other.impl_);
  }
  return *this;
}

void XboxController::update() {
}

void XboxController::setDeadzone(float dz) {
  impl_->deadzone = std::clamp(dz, 0.0f, 0.9f);
}

void XboxController::setAxisMap(int leftY_axis, int rightY_axis) {
  impl_->axis_leftY = leftY_axis;
  impl_->axis_rightY = rightY_axis;
}

float XboxController::leftX() const {
  return 0.0f;
}

float XboxController::rightX() const {
  return 0.0f;
}

float XboxController::leftY() const {
  return 0.0f;
}

float XboxController::rightY() const {
  return 0.0f;
}

bool XboxController::isAvailable() const {
  return false;
}
