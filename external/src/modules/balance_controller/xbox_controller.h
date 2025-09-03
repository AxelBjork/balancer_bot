// XboxController.h
#pragma once

#include <SDL2/SDL.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

class XboxController {
public:
  XboxController() {
    if (SDL_Init(SDL_INIT_JOYSTICK) < 0) {
      throw std::runtime_error("SDL_Init failed");
    }
    if (SDL_NumJoysticks() < 1) {
      throw std::runtime_error("No joystick found");
    }
    joystick_ = SDL_JoystickOpen(0);
    if (!joystick_) {
      throw std::runtime_error("Failed to open joystick 0");
    }
  }

  ~XboxController() {
    if (joystick_)
      SDL_JoystickClose(joystick_);
    SDL_Quit();
  }

  void update() { SDL_JoystickUpdate(); }

  // Set deadzone [0..1). Default 0.05.
  void setDeadzone(float dz) { deadzone_ = std::clamp(dz, 0.0f, 0.9f); }

  // Optional: remap which axes are used for left/right Y
  void setAxisMap(int leftY_axis, int rightY_axis) {
    axis_leftY_ = leftY_axis;
    axis_rightY_ = rightY_axis;
  }

  float leftY() const {
    return apply_deadzone(
        normalize(SDL_JoystickGetAxis(joystick_, axis_leftY_)));
  }

  float rightY() const {
    return apply_deadzone(
        normalize(SDL_JoystickGetAxis(joystick_, axis_rightY_)));
  }

private:
  SDL_Joystick *joystick_{nullptr};
  int axis_leftY_ = 1;  // your working mapping
  int axis_rightY_ = 4; // your working mapping
  float deadzone_ = 0.05f;

  static float normalize(Sint16 value) {
    return static_cast<float>(value) / 32768.0f; // [-1,1]
  }

  float apply_deadzone(float v) const {
    float a = std::fabs(v);
    if (a < deadzone_)
      return 0.0f;
    float s = (a - deadzone_) / (1.0f - deadzone_);
    return (v < 0 ? -s : s);
  }
};
