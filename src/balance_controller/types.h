#pragma once
#include <chrono>
#include <cstdlib>
#include <string>

// ---- IMU sample (from ISM330DHCX fusion) ----
// angle_rad: fused pitch angle (+ forward), gyro_rad_s: filtered pitch rate used by control.
struct ImuSample {
  double angle_rad = 0.0;
  double gyro_rad_s = 0.0;
  double yaw_rate_z = 0.0;
  std::chrono::steady_clock::time_point t{};
};

// ---- Joystick command (forward/turn normalized to [-1, 1]) ----
struct JoyCmd {
  double forward;  // + forward speed command
  double turn;     // + left faster, right slower (CCW yaw)
};

// ---- Telemetry (controller diagnostics) ----
struct Telemetry {
  double t_sec;
  double age_ms;
  double pitch_deg;
  double pitch_rate_dps;
  double filtered_pitch_rate_dps;
  double rate_sp_dps;
  double out_norm;     // PX4 rate controller normalized output (pitch axis)
  double u_sps;        // wheel command [steps/s]
  double turn_sps;     // differential steering command [steps/s]
  double integ_pitch;  // PX4 integral state for pitch
  // Translational outer-loop diagnostics
  double vel_error;
  double vel_p_term;
  double vel_i_term;
  double target_vel_sps;
  double measured_vel_sps;
  double filtered_vel_sps;
  double position_target_vel_sps;
  double pitch_ref_from_vel_deg;
  double pitch_ref_from_pos_deg;
  double pitch_error_deg;
  double rate_error_dps;
  double pitch_sp_deg;
  double effective_pitch_sp_deg;
  double pitch_trim_deg;
  double trim_active;
  double command_saturated;
};

// ---- PID Configuration ----
// Runtime-configurable PID gains loaded from pid.conf
// Default values are set here, can be overridden at runtime by load()
struct ConfigPid {
  // PX4 Rate PID (inner loop, pitch axis only)
  inline static double rate_P = 0.25;
  inline static double rate_I = 0.0;
  inline static double rate_D = 0.4;
  inline static double rate_I_lim = 0.15;
  inline static double rate_FF = 0.0;

  // Deprecated legacy outer-loop fields retained for config compatibility.
  inline static double angle_to_rate_k = 12.0;
  inline static double vel_P = -0.000055;
  inline static double vel_I = -0.000055;
  inline static double vel_D = 0.0;
  inline static double vel_I_lim = 0.15;
  inline static double pos_P = 0.0;

  // Physics-based outer loop:
  // pitch_ref = outer_k_pos * (x_ref - x) + outer_k_vel * (v_ref - v)
  // rate_sp   = outer_k_pitch * (pitch_ref - pitch) - outer_k_pitch_rate * pitch_rate
  inline static double outer_k_pos = 0.0;
  inline static double outer_k_vel = 0.000055;
  inline static double outer_k_pitch = 12.0;
  inline static double outer_k_pitch_rate = 0.25;

  // Slow trim: persistent angle error [rad] -> pitch setpoint bias integrator [rad / s]
  inline static double angle_I = 0.0;
  // Very slow trim: persistent balance effort -> lean bias setpoint for COM offset rejection.
  inline static double lean_trim_I = 0.0;
  inline static double lean_trim_max_deg = 4.0;
  inline static double lean_trim_decay_s = 3.0;

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
