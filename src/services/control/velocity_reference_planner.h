#pragma once

// Reversal-aware velocity reference generation for the translational outer
// loop.  The planner contains no controller, estimator, or simulator
// dependencies: it only advances a bounded velocity reference from one
// command sample to the next.

struct VelocityReferenceState {
  double user_velocity_mps = 0.0;
  double reference_velocity_mps = 0.0;
  double reference_acceleration_mps2 = 0.0;
  double reference_jerk_mps3 = 0.0;
  bool acceleration_limited = false;
  bool jerk_limited = false;
};

class VelocityReferencePlanner {
 public:
  void reset(double reference_velocity_mps = 0.0);

  // This is a simulator/test-only compatibility hook. Production leaves the
  // endpoint-continuous implementation enabled. It exists so an offline A/B
  // can reproduce the pre-fix behavior without adding a production config
  // selector or changing controller gains.
  void setEndpointContinuityEnabled(bool enabled) { endpoint_continuity_enabled_ = enabled; }

  VelocityReferenceState update(double user_velocity_mps, double dt_s,
                                double max_acceleration_mps2,
                                double max_deceleration_mps2,
                                double max_jerk_mps3 = 0.0);

  const VelocityReferenceState& state() const { return state_; }

 private:
  VelocityReferenceState updateLegacy(double user_velocity_mps, double dt_s,
                                      double max_acceleration_mps2,
                                      double max_deceleration_mps2,
                                      double max_jerk_mps3);

  VelocityReferenceState state_{};
  // Endpoint acceleration is the acceleration at the current reference
  // sample. The jerk-enabled implementation integrates the continuous
  // acceleration trajectory between samples, so v_ref and a_ref remain
  // consistent at the terminal point instead of landing with a nonzero
  // acceleration and clearing it one update later.
  double acceleration_command_mps2_ = 0.0;
  bool endpoint_continuity_enabled_ = true;
  bool profile_active_ = false;
  double profile_target_mps_ = 0.0;
  double profile_elapsed_s_ = 0.0;
  double profile_duration_s_ = 0.0;
  double profile_start_velocity_mps_ = 0.0;
  double profile_start_acceleration_mps2_ = 0.0;
};
