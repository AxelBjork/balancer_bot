#pragma once
#include <chrono>
#include <cstdlib>
#include <string>

// ---- IMU sample (from ISM330DHCX fusion) ----
// angle_rad: pitch angle (+ forward), gyro_rad_s: pitch rate (+ when nose down)
struct ImuSample {
  double angle_rad = 0.0;
  double gyro_rad_s = 0.0;
  double yaw_rate_z = 0.0;
  std::chrono::steady_clock::time_point t{};
};

// ---- Joystick command (forward/turn normalized to [-1, 1]) ----
struct JoyCmd {
  float forward;  // + forward speed command
  float turn;     // + left faster, right slower (CCW yaw)
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
  // Velocity PID debug
  double vel_error;
  double vel_p_term;
  double vel_i_term;
  double pitch_sp_deg;
  double effective_pitch_sp_deg;
  double pitch_trim_deg;
  double trim_active;
};

// ---- PID Configuration ----
// Runtime-configurable PID gains loaded from pid.conf
// Default values are set here, can be overridden at runtime by load()
struct ConfigPid {
  // PX4 Rate PID (inner loop, pitch axis only)
  inline static double rate_P = 0.75;  // User requested (Validated Stable)
  inline static double rate_I = 0.0;   // 0.02
  inline static double rate_D = 0.6;   // 0.6 (Critical for stability)
  inline static double rate_I_lim = 0.15;
  inline static double rate_FF = 0.0;  // 0.06

  // Minimal outer mapping: angle(rad) -> rate_sp(rad/s)
  inline static double angle_to_rate_k = 12.0;
  // Slow trim: persistent angle error [rad] -> pitch setpoint bias integrator [rad / s]
  inline static double angle_I = 0.0;
  // Very slow trim: persistent balance effort -> lean bias setpoint for COM offset rejection.
  inline static double lean_trim_I = 0.0;
  inline static double lean_trim_max_deg = 4.0;
  inline static double lean_trim_decay_s = 3.0;

  // Velocity PID (outermost loop): velocity error -> pitch angle setpoint
  inline static double vel_P = -0.000055;  // Reduced damping to prevent saturation
  inline static double vel_I = -0.000055;  // Moderate integrator
  inline static double vel_D = 0.0;        // derivative gain
  inline static double vel_I_lim = 0.15;   // Allow reasonable windup for correction

  // Position hold (supervisory loop): position error [m] -> velocity setpoint [steps/s]
  inline static double pos_P = 0.0;

  static std::string resolve_path(const std::string& default_path) {
    if (const char* env = std::getenv("BALANCER_PID_CONF")) {
      if (*env != '\0') {
        return env;
      }
    }
    return default_path;
  }

  static void load(const std::string& path);
  static void save(const std::string& path);
};
