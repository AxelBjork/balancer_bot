#pragma once
#include <chrono>
#include <functional>

#include "messages/types.h"

// ====== Control Loop Constants ======
// Motor / speed ceiling (primary scaling knob)
static constexpr double kMaxSps = 4000.0;  // actuator command ceiling (steps/s)
static constexpr double kPitchOutToSps = 3200;
static constexpr double kLeanTrimDecayS = 6.0;
static constexpr double kMaxPitchSetpointRad = 0.4;  // ~17 degrees max lean

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
  // The forward axis requests longitudinal acceleration; turn remains differential steering.
  void setJoystick(const JoyCmd& j);
  void updateOuterLoop(double measured_velocity_sps, double dt_s);
  void setTelemetrySink(std::function<void(const Telemetry&)> cb);

  // Callbacks to drive motors (steps/s). You wire these from the wrapper.
  void setMotorOutputs(std::function<void(double, double)> motor_cb);

 private:
  struct Impl;  // PIMPL hides PX4/Matrix + thread
  Impl* p_;     // or std::unique_ptr<Impl>
};
