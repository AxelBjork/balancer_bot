#pragma once
#include <chrono>
#include <cstdlib>
#include <string>

// ---- IMU sample (from ISM330DHCX fusion) ----
// angle_rad: fused pitch angle (+ forward), gyro_rad_s: filtered pitch rate used by control.
struct ImuSample {
  double angle_rad = 0.0;
  double gyro_rad_s = 0.0;
  double pitch_accel_rad_s2 = 0.0;
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
  double u_sps;     // wheel command [steps/s]
  double turn_sps;  // differential steering command [steps/s]
  // Translational outer-loop diagnostics. These are retained for consumers of the
  // original telemetry schema; prefer the explicitly named velocity fields below.
  double vel_error;  // Legacy: zero-reference error of corrected_axle_velocity_sps [steps/s].
  double
      vel_p_term;  // Legacy: velocity-loop pitch contribution from corrected axle velocity [rad].
  double measured_vel_sps;  // Legacy: corrected_axle_velocity_sps [steps/s].
  double raw_common_mode_completed_step_velocity_sps;  // Signed common-mode velocity from completed
                                                       // motor steps [steps/s].
  double pitch_motion_correction_sps;  // Pitch-rate correction applied to the common-mode velocity
                                       // [steps/s].
  double corrected_axle_velocity_sps;  // Pitch-motion-corrected axle velocity [steps/s].
  double corrected_axle_velocity_mps;  // Pitch-motion-corrected axle velocity [m/s].
  double nominal_acceleration_mps2;  // Acceleration represented by the velocity-only pitch request
                                     // [m/s^2].
  double
      commanded_acceleration_mps2;  // Acceleration represented by the final pitch request [m/s^2].
  double acceleration_pitch_contribution_deg;  // Velocity-loop pitch contribution derived from
                                               // nominal acceleration [deg].
  double current_pitch_trim_deg;  // Current learned pitch trim included in the final request [deg].
  double pitch_ref_from_vel_deg;
  double pitch_error_deg;
  double pitch_sp_deg;
  double pitch_trim_deg;
  double trim_active;
};

// ---- PID Configuration ----
// Runtime-configurable PID gains loaded from pid.conf
// Default values are set here, can be overridden at runtime by load()
struct ConfigPid {
  // PX4 Rate PID (inner loop, pitch axis only)
  inline static double rate_P = 0.25;
  inline static double rate_I = 0.0;
  inline static double rate_D = 0.0;
  inline static double rate_I_lim = 0.15;
  inline static double rate_FF = 0.0;

  // Physics-based outer loop:
  // pitch_ref = vel_P * velocity_error + internal trim_bias
  // rate_sp   = pitch_P * (pitch_ref - pitch) - pitch_D * pitch_rate
  inline static double vel_P = 0.00015;
  inline static double lean_trim_I = 0.60;
  inline static double lean_trim_max_deg = 4.0;
  inline static double pitch_P = 12.0;
  inline static double pitch_D = 0.25;

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
