#include "velocity_reference_planner.h"

#include <algorithm>
#include <cmath>

namespace {

double move_toward(double value, double target, double max_delta) {
  if (value < target) return std::min(value + max_delta, target);
  if (value > target) return std::max(value - max_delta, target);
  return value;
}

double signed_limit(double limit, double delta) {
  if (delta > 0.0) return limit;
  if (delta < 0.0) return -limit;
  return 0.0;
}

}  // namespace

void VelocityReferencePlanner::reset(double reference_velocity_mps) {
  state_.user_velocity_mps = reference_velocity_mps;
  state_.reference_velocity_mps = reference_velocity_mps;
  state_.reference_acceleration_mps2 = 0.0;
  state_.reference_jerk_mps3 = 0.0;
  state_.acceleration_limited = false;
  state_.jerk_limited = false;
  acceleration_command_mps2_ = 0.0;
}

VelocityReferenceState VelocityReferencePlanner::update(double user_velocity_mps, double dt_s,
                                                        double max_acceleration_mps2,
                                                        double max_deceleration_mps2,
                                                        double max_jerk_mps3) {
  const double finite_user_velocity = std::isfinite(user_velocity_mps) ? user_velocity_mps : 0.0;
  const double dt = std::isfinite(dt_s) ? std::max(0.0, dt_s) : 0.0;
  const double acceleration_limit =
      std::isfinite(max_acceleration_mps2) ? std::max(0.0, max_acceleration_mps2) : 0.0;
  const double deceleration_limit =
      std::isfinite(max_deceleration_mps2) ? std::max(0.0, max_deceleration_mps2) : 0.0;
  const double jerk_limit =
      std::isfinite(max_jerk_mps3) ? std::max(0.0, max_jerk_mps3) : 0.0;

  const double old_reference_velocity = state_.reference_velocity_mps;
  const double old_acceleration_command = acceleration_command_mps2_;
  double active_target = finite_user_velocity;

  // A sign reversal is explicitly two legs: brake the existing motion to
  // zero, then accelerate toward the new signed target.  This is why the
  // deceleration limit is selected by speed magnitude rather than by the
  // sign of dv.
  if (old_reference_velocity != 0.0 && finite_user_velocity != 0.0 &&
      std::signbit(old_reference_velocity) != std::signbit(finite_user_velocity)) {
    active_target = 0.0;
  }

  state_.user_velocity_mps = finite_user_velocity;
  state_.acceleration_limited = false;
  state_.jerk_limited = false;

  if (dt == 0.0) {
    state_.reference_acceleration_mps2 = 0.0;
    state_.reference_jerk_mps3 = 0.0;
    return state_;
  }

  const double velocity_delta = active_target - old_reference_velocity;
  const bool slowing_down = std::abs(active_target) < std::abs(old_reference_velocity);
  const double limit = slowing_down ? deceleration_limit : acceleration_limit;
  const double desired_acceleration = signed_limit(limit, velocity_delta);

  if (velocity_delta == 0.0) {
    const double next_acceleration_command =
        jerk_limit > 0.0
            ? move_toward(old_acceleration_command, 0.0, jerk_limit * dt)
            : 0.0;
    acceleration_command_mps2_ = next_acceleration_command;
    state_.reference_acceleration_mps2 = 0.0;
    state_.reference_jerk_mps3 =
        (next_acceleration_command - old_acceleration_command) / dt;
    state_.acceleration_limited = false;
    state_.jerk_limited = std::abs(next_acceleration_command) > 1e-12;
    return state_;
  }

  double next_acceleration_command = desired_acceleration;
  if (jerk_limit > 0.0) {
    next_acceleration_command = move_toward(
        old_acceleration_command, desired_acceleration, jerk_limit * dt);
    state_.jerk_limited = std::abs(next_acceleration_command - desired_acceleration) > 1e-12;
  }
  next_acceleration_command = std::clamp(next_acceleration_command, -limit, limit);
  const double commanded_jerk_mps3 =
      (next_acceleration_command - old_acceleration_command) / dt;
  state_.acceleration_limited = limit > 0.0 &&
                                std::abs(next_acceleration_command) >= limit - 1e-12;

  double next_reference_velocity =
      old_reference_velocity + next_acceleration_command * dt;
  const bool crossed_target =
      (velocity_delta > 0.0 && next_reference_velocity > active_target) ||
      (velocity_delta < 0.0 && next_reference_velocity < active_target);
  const bool landed_on_target = next_reference_velocity == active_target;
  if (crossed_target || landed_on_target) {
    next_reference_velocity = active_target;
    // Do not let the acceleration ramp carry the reference through a target
    // or the zero crossing of a reversal. Retain the actual final transition
    // acceleration so the next update can ramp it back to zero within the
    // jerk bound while holding the reference exactly at its target.
    acceleration_command_mps2_ =
        (next_reference_velocity - old_reference_velocity) / dt;
  } else {
    acceleration_command_mps2_ = next_acceleration_command;
  }

  state_.reference_velocity_mps = next_reference_velocity;
  state_.reference_acceleration_mps2 =
      (next_reference_velocity - old_reference_velocity) / dt;
  state_.reference_jerk_mps3 = commanded_jerk_mps3;
  return state_;
}
