// rate_controller_core.h
#pragma once
#include <chrono>
#include <functional>
#include <cstdio>

#include "control_interface.h"

// Non-template core; hides PX4/Matrix in the .cpp
class RateControllerCore {
 public:
  RateControllerCore();
  ~RateControllerCore();

  // non-copyable, movable if you like
  RateControllerCore(const RateControllerCore&) = delete;
  RateControllerCore& operator=(const RateControllerCore&) = delete;

  void start();
  void stop();

  void pushImu(const ImuSample& s);
  void setJoystick(const JoyCmd& j);  // kept for API compat; may be no-op
  void setTelemetrySink(std::function<void(const Telemetry&)> cb);

  // Callbacks to drive motors (steps/s). You wire these from the wrapper.
  void setMotorOutputs(std::function<void(float, float)> motor_cb);

  // Callback to get velocity feedback (average of left+right in steps/s)
  void setVelocityFeedback(std::function<float()> velocity_cb);

 private:
  struct Impl;  // PIMPL hides PX4/Matrix + thread
  Impl* p_;     // or std::unique_ptr<Impl>
};

template <class MotorRunnerT>
class CascadedController {
 public:
  CascadedController(MotorRunnerT& motors) : motors_(motors) {
    core_.setMotorOutputs(
        [this](float left_sps, float right_sps) { 
            motors_.setTargets(left_sps, right_sps); 
        });

    core_.setVelocityFeedback([this]() -> float {
      // Velocity estimator using step tracking
      auto now = std::chrono::steady_clock::now();

      int64_t left_steps = motors_.getLeftSteps();
      int64_t right_steps = motors_.getRightSteps();

      if (last_update_time_.time_since_epoch().count() == 0) {
        // First call - initialize
        last_left_steps_ = left_steps;
        last_right_steps_ = right_steps;
        last_update_time_ = now;
        return 0.0f;
      }

      std::chrono::duration<float> dt = now - last_update_time_;
      float dt_sec = dt.count();

      if (dt_sec < 0.001f) {
        return last_velocity_;  // Too soon, return cached value
      }

      // Calculate velocity from step changes
      int64_t left_delta = left_steps - last_left_steps_;
      int64_t right_delta = right_steps - last_right_steps_;

      float left_velocity = (float)left_delta / dt_sec;
      float right_velocity = (float)right_delta / dt_sec;

      float velocity = (left_velocity + right_velocity) / 2.0f;

      // Update state
      last_left_steps_ = left_steps;
      last_right_steps_ = right_steps;
      last_update_time_ = now;
      last_velocity_ = velocity;

      return velocity;
    });

    core_.start();
  }

  ~CascadedController() {
    core_.stop();
  }

  void pushImu(const ImuSample& s) {
    core_.pushImu(s);
  }
  void setJoystick(const JoyCmd& j) {
    core_.setJoystick(j);
  }
  void setTelemetrySink(std::function<void(const Telemetry&)> cb) {
    core_.setTelemetrySink(std::move(cb));
  }

 private:
  MotorRunnerT& motors_;
  RateControllerCore core_;

  // Velocity estimator state
  std::chrono::steady_clock::time_point last_update_time_;
  int64_t last_left_steps_{0};
  int64_t last_right_steps_{0};
  float last_velocity_{0.0f};
};