#pragma once
#include <chrono>
#include <cstdint>
#include <functional>

#include "messages/types.h"

// Fixed scheduling/safety constants. Tunable limits and state-feedback gains
// live in the v10 ConfigPid schema.
static constexpr double kMaxSps = 12000.0;
static constexpr double kMaxPitchSetpointRad = 45.0 * 3.14159265358979323846 / 180.0;

namespace rate_controller_detail {
// Kept separate so the circular observer convention is directly testable.
double wrap_angle_delta(double angle_rad);
}

// Non-template control core; the production law is explicit state feedback.
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
  void clearImu();
  void setJoystick(const JoyCmd& j);
  // Enable a future user-supervised direct pitch-authority measurement. The
  // target is accepted only for the explicit 0/1/2/4 degree test set, the
  // supplied COM trim must remain inside the configured trim envelope, and
  // active requests must carry a strictly increasing nonzero request ID.
  bool setPitchAuthorityDiagnostic(bool active, double target_deg, double com_trim_deg,
                                   double duration_s, uint32_t request_id = 0);
  void applyPidConfig();
  void setMotorFeedback(int64_t left_actual_steps, int64_t right_actual_steps,
                        bool actuator_fault);
  void setTelemetrySink(std::function<void(const Telemetry&)> cb);

  // Callbacks to drive motors (steps/s). You wire these from the wrapper.
  void setMotorOutputs(std::function<void(double, double)> motor_cb);

 private:
  struct Impl;  // PIMPL keeps controller state and implementation details private
  Impl* p_;     // or std::unique_ptr<Impl>
};
