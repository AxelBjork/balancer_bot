// rate_controller_core.h
#pragma once
#include <cstdio>
#include <functional>

#include "types.h"

// ====== Control Loop Constants ======
// Motor / speed ceiling (primary scaling knob)
static constexpr double kMaxSps = 4000.0;         // clamp for wheel speed command (steps/s)
static constexpr double kPitchOutToSps = 3200.0;  // PX4 normalized -> steps/s

// Velocity loop decimation
static constexpr int kVelocityDecimation = 40;       // Run every 40th cycle (400Hz -> 10Hz)
static constexpr double kMaxPitchSetpointRad = 0.3;  // ~17 degrees max lean

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
