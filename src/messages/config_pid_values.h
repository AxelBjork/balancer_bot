#pragma once

#include "msg_base.h"

namespace ipc {

struct DOC_DESC("The shared numeric PID configuration block carried by override and status messages.")
    ConfigPidValuesPayload {
  double rate_P;
  double rate_I;
  double rate_D;
  double rate_I_lim;
  double rate_FF;
  double drive_max_acceleration_mps2;
  double velocity_damping_per_s;
  double velocity_I;
  double velocity_I_limit_deg;
  double angle_P;
  double angle_D;
  double pitch_rate_max_sps;
  double drive_max_sps;
  double turn_max_sps;
  double balance_max_sps;
};

}  // namespace ipc
