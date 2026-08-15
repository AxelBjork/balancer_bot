#pragma once

#include "msg_base.h"

namespace ipc {

struct DOC_DESC("The shared numeric controller configuration block carried by override and status messages.")
    ConfigPidValuesPayload {
  // Maximum commanded forward acceleration [m/s^2].
  double drive_max_acceleration_mps2;
  // Translational damping coefficient [1/s], applied to corrected axle speed.
  double velocity_damping_per_s;
  // Maximum pitch contribution that velocity regulation may request [deg].
  // This is separate from the total pitch safety limit and COM-trim limit.
  double velocity_pitch_limit_deg;
  // Stationary COM-trim adaptation [deg/(SPS*s)].
  double velocity_I;
  // Absolute COM-trim limit [deg].
  double velocity_I_limit_deg;
  // Common-mode drive, turn, and balance authority limits [SPS].
  double drive_max_sps;
  double turn_max_sps;
  double balance_max_sps;
  // Explicit attitude gains [SPS/rad], [SPS/(rad/s)], and [SPS/(rad/s^2)].
  double pitch_gain;
  double pitch_rate_gain;
  double pitch_accel_gain;
  // The completed-step observer and this control-path filter are deliberately
  // separate. The former remains at the compiled estimator bandwidth.
  double velocity_control_cutoff_hz;
};

}  // namespace ipc
