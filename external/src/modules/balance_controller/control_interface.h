#pragma once
#include <chrono>
// ---- IMU sample (from ISM330DHCX fusion later) ----
// angle_rad: pitch angle (+ forward), gyro_rad_s: pitch rate (+ when nose down)
struct ImuSample {
  double angle_rad = 0.0;
  double gyro_rad_s = 0.0;
  double yaw_rate_z = 0.0;
  std::chrono::steady_clock::time_point t{};
};

// ---- Joystick command (forward/turn normalized to [-1, 1]) ----
struct JoyCmd {
  float forward;    // + forward speed command
  float turn;       // + left faster, right slower (CCW yaw)
};

// ---- Telemetry (per-term LQR contributions) ----
struct Telemetry {
  double t_sec;
  double age_ms;
  double pitch_deg;
  double pitch_rate_dps;
  double rate_sp_dps;
  double out_norm;     // PX4 rate controller normalized output (pitch axis)
  double u_sps;        // wheel command [steps/s]
  double integ_pitch;  // PX4 integral state for pitch
};
