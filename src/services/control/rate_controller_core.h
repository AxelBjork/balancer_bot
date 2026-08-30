#pragma once
#include <chrono>
#include <cstdint>
#include <functional>

#include "messages/types.h"
#include "services/main/config.h"

// Fixed scheduling/safety constants. Tunable limits and state-feedback gains
// live in the v12 ConfigPid schema.
static constexpr double kMaxSps = Config::max_step_rate_sps;
static constexpr double kMaxPitchSetpointRad =
    Config::max_motion_pitch_setpoint_deg * 3.14159265358979323846 / 180.0;

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
  void applyPidConfig();
  // Simulator-only A/B controls. Defaults use the production path with both
  // fixes enabled; no production configuration or runtime selector uses it.
  void setSimulationOuterLoopOptions(bool endpoint_continuity_enabled,
                                     bool matched_reference_filter_enabled);
  // Simulator-only A/B hook for the drive-feedforward architecture. Production
  // always uses the new architecture; this is not a configuration selector.
  void setSimulationDriveFeedforwardEnabled(bool enabled);
  // Simulator-only recovery-fixture gate. Production control is always
  // enabled through its normal startup path.
  void setSimulationControllerEnabled(bool enabled);
  void setMotorFeedback(int64_t left_actual_steps, int64_t right_actual_steps,
                        bool actuator_fault);
  void setTelemetrySink(std::function<void(const Telemetry&)> cb);

  // Callbacks to drive motors (steps/s). You wire these from the wrapper.
  void setMotorOutputs(std::function<void(double, double)> motor_cb);

 private:
  struct Impl;  // PIMPL keeps controller state and implementation details private
  Impl* p_;     // or std::unique_ptr<Impl>
};
