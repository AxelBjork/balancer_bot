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

  VelocityReferenceState update(double user_velocity_mps, double dt_s,
                                double max_acceleration_mps2,
                                double max_deceleration_mps2,
                                double max_jerk_mps3 = 0.0);

  const VelocityReferenceState& state() const { return state_; }

 private:
  VelocityReferenceState state_{};
  // The acceleration command is kept separately from the reported
  // transition acceleration. When a transition lands exactly on its target,
  // it is replaced by the actual final transition so the next update cannot
  // carry an unbounded command through the target or reversal zero crossing.
  double acceleration_command_mps2_ = 0.0;
};
