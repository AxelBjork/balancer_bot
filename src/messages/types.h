#pragma once
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <string>

#include "config_pid_values.h"

// ---- IMU sample (from ISM330DHCX conditioning) ----
// angle_rad: bounded gyro/gravity pitch (+ forward), gyro_rad_s: filtered pitch rate used by control.
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

enum ActuatorSaturationFlag : uint32_t {
  ActuatorSaturationNone = 0,
  ActuatorSaturationLeftSlew = 1u << 0,
  ActuatorSaturationRightSlew = 1u << 1,
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
  // Translational outer-loop diagnostics.  These names intentionally match the
  // stable wire slots, whose types/order are retained across the v4 migration.
  double nominal_acceleration_mps2{};
  double raw_completed_velocity_sps{};
  double corrected_axle_velocity_sps{};
  double velocity_damping_acceleration_mps2{};
  double com_trim_deg{};
  // Compatibility aliases for in-process test harnesses.  Wire payloads use
  // only the v4 names above.
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

using ConfigPidValues = ipc::ConfigPidValuesPayload;

enum class ConfigPidValidationCode : uint8_t {
  Accepted = 0,
  NonFinite = 1,
  Negative = 2,
  NonPositive = 3,
  OutOfRange = 4,
};

// ---- PID Configuration ----
// Runtime-configurable PID gains loaded from pid.conf
// Default values are set here, can be overridden at runtime by load()
#define BALANCER_STRINGIFY_DETAIL(value) #value
#define BALANCER_STRINGIFY(value) BALANCER_STRINGIFY_DETAIL(value)
#define BALANCER_PID_CONFIG_VERSION 6

struct ConfigPid {
  inline static constexpr int config_version = BALANCER_PID_CONFIG_VERSION;
  inline static constexpr char config_version_marker[] =
      "BALANCER_PID_CONFIG_VERSION=" BALANCER_STRINGIFY(BALANCER_PID_CONFIG_VERSION);

  // PX4 Rate PID (inner loop, pitch axis only)
  inline static double rate_P = 0.25;
  inline static double rate_I = 0.0;
  inline static double rate_D = 0.0;
  inline static double rate_I_lim = 0.15;
  inline static double rate_FF = 0.0;

  // Acceleration command / corrected-velocity damping / stationary COM trim
  // (100 Hz with a 10 Hz velocity filter), angle-to-rate, and allocation.
  // velocity_I is learned only at a stationary command and remains the COM trim limit.
  inline static double drive_max_acceleration_mps2 = 1.5;
  inline static double velocity_damping_per_s = 8.0;
  inline static double velocity_I = 0.0010;
  inline static double velocity_I_limit_deg = 4.0;
  inline static double angle_P = 12.0;
  inline static double angle_D = 0.25;
  inline static double pitch_rate_max_sps = 2000.0;
  inline static double drive_max_sps = 1200.0;
  inline static double turn_max_sps = 1600.0;
  inline static double balance_max_sps = 12000.0;
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

  static ConfigPidValues numeric_values();
  static ConfigPidValidationCode validate_numeric(const ConfigPidValues& values);
  static void apply_numeric(const ConfigPidValues& values);
  static uint64_t generation();
  static void load(const std::string& path);
  static void save(const std::string& path);
};

#undef BALANCER_PID_CONFIG_VERSION
#undef BALANCER_STRINGIFY
#undef BALANCER_STRINGIFY_DETAIL
