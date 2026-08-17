#pragma once

#include "msg_base.h"

namespace ipc {

struct DOC_DESC("The shared numeric controller configuration block carried by override and status messages.")
    ConfigPidValuesPayload {
  // Maximum normalized-command speed [m/s].
  double drive_max_velocity_mps;
  // Velocity-reference feedback gain [1/s].
  double velocity_gain_per_s;
  // Slow control-path pole after the compiled measurement filter [Hz].
  double velocity_feedback_cutoff_hz;
  // Motion pitch authority and fixed physical COM trim [deg].
  double outer_pitch_limit_deg;
  double fixed_com_trim_deg;
  // The legacy COM learner remains available for controlled experiments but is
  // disabled in the approved v12 configuration.
  double adaptive_com_trim_enabled;
  double adaptive_com_trim_gain_deg_per_mps_s;
  double adaptive_com_trim_limit_deg;
  // Turn and balance authority limits [SPS].
  double turn_max_sps;
  double balance_max_sps;
  // Explicit attitude gains [SPS/rad], [SPS/(rad/s)], and [SPS/(rad/s^2)].
  double pitch_gain;
  double pitch_rate_gain;
  double pitch_accel_gain;
  // Planner and bounded leaky-integral outer-loop parameters.
  double planner_max_acceleration_mps2;
  double planner_max_deceleration_mps2;
  double planner_max_jerk_mps3;
  double velocity_i_gain_per_s2;
  double velocity_i_leak_time_s;
  double velocity_i_acceleration_limit_mps2;
};

}  // namespace ipc
