#pragma once
#include <chrono>
#include <functional>

#include "types.h"

// ====== Control Loop Constants ======
// Motor / speed ceiling (primary scaling knob)
static constexpr double kMaxSps = 4000.0;            // clamp for wheel speed command (steps/s)
static constexpr double kPitchOutToSps = 3200;

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
  void step(double dt_s, std::chrono::steady_clock::time_point now);

  void pushImu(const ImuSample& s);
  void setJoystick(const JoyCmd& j);  // kept for API compat; may be no-op
  void setTelemetrySink(std::function<void(const Telemetry&)> cb);

  // Callbacks to drive motors (steps/s). You wire these from the wrapper.
  void setMotorOutputs(std::function<void(double, double)> motor_cb);

  // Callback to get velocity feedback (average of left+right in steps/s)
  void setVelocityFeedback(std::function<double()> velocity_cb);

  // Callback to get average wheel position (meters)
  void setPositionFeedback(std::function<double()> position_cb);

 private:
  struct Impl;  // PIMPL hides PX4/Matrix + thread
  Impl* p_;     // or std::unique_ptr<Impl>
};
