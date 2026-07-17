#pragma once
#include <chrono>
#include <functional>

#include "messages/types.h"

// Fixed scheduling/safety constants. Tunable limits live in ConfigPid v2.
static constexpr double kMaxSps = 12000.0;
static constexpr double kPitchOutToSps = 3200.0;
static constexpr double kMaxPitchSetpointRad = 45.0 * 3.14159265358979323846 / 180.0;

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
  void step(double dt_s, std::chrono::steady_clock::time_point now);

  void pushImu(const ImuSample& s);
  void setJoystick(const JoyCmd& j);
  void setMotorFeedback(double measured_velocity_sps, bool actuator_fault);
  void setTelemetrySink(std::function<void(const Telemetry&)> cb);

  // Callbacks to drive motors (steps/s). You wire these from the wrapper.
  void setMotorOutputs(std::function<void(double, double)> motor_cb);

 private:
  struct Impl;  // PIMPL hides PX4/Matrix + thread
  Impl* p_;     // or std::unique_ptr<Impl>
};
