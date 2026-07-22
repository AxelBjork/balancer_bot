#pragma once
#include <chrono>
#include <cstdint>
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

enum ControllerFaultFlag : uint32_t {
  ControllerFaultNone = 0,
  ControllerFaultNoImu = 1u << 0,
  ControllerFaultStaleImu = 1u << 1,
  ControllerFaultFutureImu = 1u << 2,
  ControllerFaultFallover = 1u << 3,
  ControllerFaultActuator = 1u << 4,
};

enum ControllerSaturationFlag : uint32_t {
  ControllerSaturationNone = 0,
  ControllerSaturationPitch = 1u << 0,
  ControllerSaturationRate = 1u << 1,
  ControllerSaturationBalance = 1u << 2,
  ControllerSaturationTurn = 1u << 3,
};

// ---- Telemetry (controller diagnostics) ----
struct Telemetry {
  double t_sec{};
  double age_ms{};
  double pitch_deg{};
  double pitch_rate_dps{};
  double filtered_pitch_rate_dps{};
  double u_sps{};     // wheel command [steps/s]
  double turn_sps{};  // differential steering command [steps/s]
  // Translational outer-loop diagnostics
  double target_vel_sps{};
  double vel_error{};
  double vel_p_term_deg{};
  double vel_i_term_deg{};
  double measured_vel_sps{};
  double pitch_error_deg{};
  double pitch_sp_deg{};
  double rate_sp_dps{};
  double rate_error_dps{};
  bool command_saturated{};
  bool actuator_fault{};
  uint32_t controller_fault_flags{};
  uint32_t controller_saturation_flags{};
};

// ---- PID Configuration ----
// Runtime-configurable PID gains loaded from pid.conf
// Default values are set here, can be overridden at runtime by load()
struct ConfigPid {
  inline static constexpr int config_version = 3;

  // PX4 Rate PID (inner loop, pitch axis only)
  inline static double rate_P = 0.25;
  inline static double rate_I = 0.0;
  inline static double rate_D = 0.0;
  inline static double rate_I_lim = 0.15;
  inline static double rate_FF = 0.0;

  // Velocity feedback / stationary COM trim (50 Hz), angle-to-rate, and allocation.
  // Velocity gains produce degrees from an error expressed in steps/s; velocity_I
  // is learned only at a stationary command and remains the COM trim limit.
  inline static double velocity_P = 0.0020;
  inline static double velocity_I = 0.0010;
  inline static double velocity_I_limit_deg = 4.0;
  inline static double angle_P = 12.0;
  inline static double angle_D = 0.25;
  inline static double drive_max_sps = 1200.0;
  inline static double turn_max_sps = 1600.0;
  inline static double pitch_max_deg = 10.0;
  inline static double balance_max_sps = 12000.0;
  inline static double output_scale_sps = 3200.0;
  // Optional safety mode for passive IMU/telemetry measurements.
  inline static bool controller_enabled = true;

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
