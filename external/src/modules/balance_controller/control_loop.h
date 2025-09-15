// rate_controller_core.h
#pragma once
#include <functional>
#include <chrono>
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
  void setJoystick(const JoyCmd& j); // kept for API compat; may be no-op
  void setTelemetrySink(std::function<void(const Telemetry&)> cb);

  // Callbacks to drive motors (steps/s). You wire these from the wrapper.
  void setMotorOutputs(std::function<void(float)> left_cb,
                       std::function<void(float)> right_cb);

private:
  struct Impl;          // PIMPL hides PX4/Matrix + thread
  Impl* p_;             // or std::unique_ptr<Impl>
};


template <class MotorRunnerT>
class CascadedController {
public:
  CascadedController(MotorRunnerT& motors)
  : motors_(motors) {
    core_.setMotorOutputs(
      [this](float sps) { motors_.setLeft(sps);  },
      [this](float sps) { motors_.setRight(sps); });
    core_.start();
  }

  ~CascadedController() { core_.stop(); }

  void pushImu(const ImuSample& s) { core_.pushImu(s); }
  void setJoystick(const JoyCmd& j) { core_.setJoystick(j); }
  void setTelemetrySink(std::function<void(const Telemetry&)> cb) { core_.setTelemetrySink(std::move(cb)); }

private:
  MotorRunnerT& motors_;
  RateControllerCore core_;
};