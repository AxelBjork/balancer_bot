#include "velocity_reference_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>

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

struct CubicVelocityProfile {
  double start_velocity_mps = 0.0;
  double start_acceleration_mps2 = 0.0;
  double target_velocity_mps = 0.0;
  double duration_s = 0.0;
  double c2 = 0.0;
  double c3 = 0.0;

  double velocity(double time_s) const {
    return start_velocity_mps + start_acceleration_mps2 * time_s + c2 * time_s * time_s +
           c3 * time_s * time_s * time_s;
  }

  double acceleration(double time_s) const {
    return start_acceleration_mps2 + 2.0 * c2 * time_s + 3.0 * c3 * time_s * time_s;
  }

  double jerk(double time_s) const { return 2.0 * c2 + 6.0 * c3 * time_s; }
};

CubicVelocityProfile make_profile(double start_velocity_mps, double start_acceleration_mps2,
                                   double target_velocity_mps, double duration_s) {
  const double delta_velocity = target_velocity_mps - start_velocity_mps;
  const double inverse_duration = 1.0 / duration_s;
  const double inverse_duration_squared = inverse_duration * inverse_duration;
  CubicVelocityProfile profile;
  profile.start_velocity_mps = start_velocity_mps;
  profile.start_acceleration_mps2 = start_acceleration_mps2;
  profile.target_velocity_mps = target_velocity_mps;
  profile.duration_s = duration_s;
  profile.c2 = 3.0 * delta_velocity * inverse_duration_squared -
               2.0 * start_acceleration_mps2 * inverse_duration;
  profile.c3 = -2.0 * delta_velocity * inverse_duration_squared * inverse_duration +
               start_acceleration_mps2 * inverse_duration_squared;
  return profile;
}

bool profile_is_valid(const CubicVelocityProfile& profile, double max_positive_acceleration_mps2,
                      double max_negative_acceleration_mps2, double max_jerk_mps3) {
  const double duration = profile.duration_s;
  const double acceleration_at_start = profile.acceleration(0.0);
  const double acceleration_at_end = profile.acceleration(duration);
  const double acceleration_min = std::min(acceleration_at_start, acceleration_at_end);
  const double acceleration_max = std::max(acceleration_at_start, acceleration_at_end);

  // Acceleration is quadratic. Its only possible interior extremum is where
  // jerk is zero; checking it avoids relying on a coarse telemetry sample.
  if (std::abs(profile.c3) > 1e-15) {
    const double extremum_time = -profile.c2 / (3.0 * profile.c3);
    if (extremum_time > 0.0 && extremum_time < duration) {
      const double acceleration_at_extremum = profile.acceleration(extremum_time);
      if (acceleration_at_extremum < -max_negative_acceleration_mps2 - 1e-10 ||
          acceleration_at_extremum > max_positive_acceleration_mps2 + 1e-10) {
        return false;
      }
    }
  }
  if (acceleration_min < -max_negative_acceleration_mps2 - 1e-10 ||
      acceleration_max > max_positive_acceleration_mps2 + 1e-10) {
    return false;
  }

  if (std::max(std::abs(profile.jerk(0.0)), std::abs(profile.jerk(duration))) >
      max_jerk_mps3 + 1e-10) {
    return false;
  }

  // Never cross the target before the final sample. A reversal may continue
  // briefly in its original direction because its initial acceleration can
  // still have the old sign, but it must not pass the active target and come
  // back to it.
  const double delta_velocity = profile.target_velocity_mps - profile.start_velocity_mps;
  if (std::abs(delta_velocity) > 1e-12) {
    constexpr int kVelocitySamples = 64;
    for (int index = 1; index < kVelocitySamples; ++index) {
      const double time = duration * static_cast<double>(index) /
                          static_cast<double>(kVelocitySamples);
      const double velocity = profile.velocity(time);
      if ((delta_velocity > 0.0 && velocity > profile.target_velocity_mps + 1e-10) ||
          (delta_velocity < 0.0 && velocity < profile.target_velocity_mps - 1e-10)) {
        return false;
      }
    }
  }
  return true;
}

double choose_profile_duration(double start_velocity_mps, double start_acceleration_mps2,
                               double target_velocity_mps, double dt_s,
                               double max_positive_acceleration_mps2,
                               double max_negative_acceleration_mps2, double max_jerk_mps3) {
  constexpr int kMaximumProfileSamples = 10000;
  for (int sample_count = 1; sample_count <= kMaximumProfileSamples; ++sample_count) {
    const double duration = dt_s * static_cast<double>(sample_count);
    const auto profile = make_profile(start_velocity_mps, start_acceleration_mps2,
                                      target_velocity_mps, duration);
    if (profile_is_valid(profile, max_positive_acceleration_mps2,
                         max_negative_acceleration_mps2,
                         max_jerk_mps3)) {
      return duration;
    }
  }
  return std::numeric_limits<double>::quiet_NaN();
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
  profile_active_ = false;
  profile_target_mps_ = reference_velocity_mps;
  profile_elapsed_s_ = 0.0;
  profile_duration_s_ = 0.0;
  profile_start_velocity_mps_ = reference_velocity_mps;
  profile_start_acceleration_mps2_ = 0.0;
}

VelocityReferenceState VelocityReferencePlanner::update(double user_velocity_mps, double dt_s,
                                                        double max_acceleration_mps2,
                                                        double max_deceleration_mps2,
                                                        double max_jerk_mps3) {
  if (!endpoint_continuity_enabled_) {
    return updateLegacy(user_velocity_mps, dt_s, max_acceleration_mps2,
                        max_deceleration_mps2, max_jerk_mps3);
  }

  const double finite_user_velocity = std::isfinite(user_velocity_mps) ? user_velocity_mps : 0.0;
  const double dt = std::isfinite(dt_s) ? std::max(0.0, dt_s) : 0.0;
  const double acceleration_limit =
      std::isfinite(max_acceleration_mps2) ? std::max(0.0, max_acceleration_mps2) : 0.0;
  const double deceleration_limit =
      std::isfinite(max_deceleration_mps2) ? std::max(0.0, max_deceleration_mps2) : 0.0;
  const double jerk_limit =
      std::isfinite(max_jerk_mps3) ? std::max(0.0, max_jerk_mps3) : 0.0;

  const double old_reference_velocity = state_.reference_velocity_mps;
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
    state_.reference_acceleration_mps2 = acceleration_command_mps2_;
    state_.reference_jerk_mps3 = 0.0;
    return state_;
  }

  if (std::abs(active_target - old_reference_velocity) <= 1e-12 &&
      std::abs(acceleration_command_mps2_) <= 1e-12) {
    profile_active_ = false;
    state_.reference_velocity_mps = active_target;
    state_.reference_acceleration_mps2 = 0.0;
    state_.reference_jerk_mps3 = 0.0;
    state_.acceleration_limited = false;
    state_.jerk_limited = false;
    return state_;
  }

  // Continue a terminal trajectory once the ordinary bounded acceleration
  // update has brought the reference close enough that landing it with the
  // current acceleration would cross the target. The terminal profile is
  // sampled at the outer-loop cadence and ends at (target, zero acceleration).
  if (profile_active_ && std::abs(profile_target_mps_ - active_target) <= 1e-12) {
    const double old_acceleration_command = acceleration_command_mps2_;
    profile_elapsed_s_ = std::min(profile_duration_s_, profile_elapsed_s_ + dt);
    const auto profile = make_profile(profile_start_velocity_mps_,
                                      profile_start_acceleration_mps2_, profile_target_mps_,
                                      profile_duration_s_);
    double next_reference_velocity = profile.velocity(profile_elapsed_s_);
    double next_acceleration_command = profile.acceleration(profile_elapsed_s_);
    if (profile_elapsed_s_ >= profile_duration_s_ - 1e-12) {
      next_reference_velocity = profile_target_mps_;
      next_acceleration_command = 0.0;
      profile_active_ = false;
    }
    state_.reference_velocity_mps = next_reference_velocity;
    state_.reference_acceleration_mps2 = next_acceleration_command;
    state_.reference_jerk_mps3 =
        (next_acceleration_command - old_acceleration_command) / dt;
    state_.acceleration_limited =
        std::abs(next_acceleration_command) >=
        ((next_acceleration_command >= 0.0 ? acceleration_limit : deceleration_limit) - 1e-10);
    state_.jerk_limited = jerk_limit > 0.0 &&
                          std::abs(state_.reference_jerk_mps3) >= jerk_limit - 1e-8;
    acceleration_command_mps2_ = next_acceleration_command;
    return state_;
  }

  if (profile_active_) {
    // A changed command retargets from the current continuous state. This is
    // still jerk bounded because the new profile begins at the current
    // endpoint acceleration.
    profile_active_ = false;
  }

  const double velocity_delta = active_target - old_reference_velocity;
  const bool slowing_down = std::abs(active_target) < std::abs(old_reference_velocity);
  const double limit = slowing_down ? deceleration_limit : acceleration_limit;
  const double desired_acceleration = signed_limit(limit, velocity_delta);
  double next_acceleration_command = desired_acceleration;
  if (jerk_limit > 0.0) {
    next_acceleration_command = move_toward(
        acceleration_command_mps2_, desired_acceleration, jerk_limit * dt);
    state_.jerk_limited =
        std::abs(next_acceleration_command - desired_acceleration) > 1e-12;
  }
  state_.acceleration_limited =
      limit > 0.0 && std::abs(next_acceleration_command) > 1e-12 &&
      std::abs(next_acceleration_command) >= limit - 1e-12;
  const double next_reference_velocity =
      jerk_limit > 0.0
          ? old_reference_velocity +
                0.5 * (acceleration_command_mps2_ + next_acceleration_command) * dt
          : old_reference_velocity + next_acceleration_command * dt;
  const bool crossed_target =
      (velocity_delta > 0.0 && next_reference_velocity > active_target) ||
      (velocity_delta < 0.0 && next_reference_velocity < active_target);
  const bool landed_on_target = next_reference_velocity == active_target;

  // Start the terminal profile before the ordinary next step consumes the
  // remaining velocity error. If the next acceleration step would leave less
  // velocity error than is needed to ramp that acceleration to zero under
  // the jerk bound, waiting one more sample would make a continuous arrival
  // impossible.
  const bool terminal_ramp_needed =
      jerk_limit > 0.0 && velocity_delta * next_acceleration_command > 0.0 &&
      std::abs(active_target - next_reference_velocity) <=
          (next_acceleration_command * next_acceleration_command) / (2.0 * jerk_limit) +
              1e-12;

  if ((crossed_target || landed_on_target || terminal_ramp_needed) && jerk_limit > 0.0) {
    profile_target_mps_ = active_target;
    profile_elapsed_s_ = 0.0;
    profile_start_velocity_mps_ = old_reference_velocity;
    profile_start_acceleration_mps2_ = acceleration_command_mps2_;
    const double profile_delta = profile_target_mps_ - profile_start_velocity_mps_;
    const bool profile_slowing_down =
        std::abs(profile_target_mps_) < std::abs(profile_start_velocity_mps_);
    const bool positive_is_target_direction = profile_delta > 0.0;
    const double target_direction_limit =
        profile_slowing_down ? deceleration_limit : acceleration_limit;
    const double opposite_direction_limit =
        profile_slowing_down ? acceleration_limit : deceleration_limit;
    const double profile_positive_limit =
        positive_is_target_direction ? target_direction_limit : opposite_direction_limit;
    const double profile_negative_limit =
        positive_is_target_direction ? opposite_direction_limit : target_direction_limit;
    profile_duration_s_ = choose_profile_duration(
        profile_start_velocity_mps_, profile_start_acceleration_mps2_, profile_target_mps_, dt,
        profile_positive_limit, profile_negative_limit, jerk_limit);
    if (std::isfinite(profile_duration_s_)) {
      profile_active_ = true;
      profile_elapsed_s_ = std::min(profile_duration_s_, dt);
      const auto profile = make_profile(profile_start_velocity_mps_,
                                        profile_start_acceleration_mps2_, profile_target_mps_,
                                        profile_duration_s_);
      state_.reference_velocity_mps = profile.velocity(profile_elapsed_s_);
      state_.reference_acceleration_mps2 = profile.acceleration(profile_elapsed_s_);
      state_.reference_jerk_mps3 =
          (state_.reference_acceleration_mps2 - acceleration_command_mps2_) / dt;
      acceleration_command_mps2_ = state_.reference_acceleration_mps2;
      return state_;
    }
  }

  if (crossed_target || landed_on_target) {
    state_.reference_velocity_mps = active_target;
    state_.reference_acceleration_mps2 = 0.0;
    state_.reference_jerk_mps3 =
        (0.0 - acceleration_command_mps2_) / dt;
    acceleration_command_mps2_ = 0.0;
  } else {
    state_.reference_velocity_mps = next_reference_velocity;
    state_.reference_acceleration_mps2 = next_acceleration_command;
    state_.reference_jerk_mps3 =
        (next_acceleration_command - acceleration_command_mps2_) / dt;
    acceleration_command_mps2_ = next_acceleration_command;
  }
  return state_;
}

VelocityReferenceState VelocityReferencePlanner::updateLegacy(
    double user_velocity_mps, double dt_s, double max_acceleration_mps2,
    double max_deceleration_mps2, double max_jerk_mps3) {
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
        jerk_limit > 0.0 ? move_toward(old_acceleration_command, 0.0, jerk_limit * dt) : 0.0;
    acceleration_command_mps2_ = next_acceleration_command;
    state_.reference_acceleration_mps2 = 0.0;
    state_.reference_jerk_mps3 =
        (next_acceleration_command - old_acceleration_command) / dt;
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
  double next_reference_velocity = old_reference_velocity + next_acceleration_command * dt;
  const bool crossed_target =
      (velocity_delta > 0.0 && next_reference_velocity > active_target) ||
      (velocity_delta < 0.0 && next_reference_velocity < active_target);
  if (crossed_target || next_reference_velocity == active_target) {
    next_reference_velocity = active_target;
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
