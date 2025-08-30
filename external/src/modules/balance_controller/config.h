#pragma once
#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>

struct AxisCfg {
  int x = 0, y = 1, z = 2;
  bool invert_x = false, invert_y = false, invert_z = false;
};
// ---------------------- Compile-time configuration ---------------------------
struct Config {
  // General
  static constexpr int   run_seconds   = 30;
  // IMU
  static constexpr double sampling_hz  = 833.000; // available 12.500 26.000 52.000 104.000 208.000 416.000 833.000
  static constexpr AxisCfg accel_cfg = {.x = 0, .y = 2, .z = 1, .invert_x = true, .invert_z = true};
  static constexpr AxisCfg gyro_cfg = {.x = 0, .y = 2, .z = 1, .invert_x = true, .invert_z = true};

  // LPF IMU Filter
  static constexpr double fc_acc_hz   = 3.50;      // LPF per accel axis
  static constexpr double fc_g_hz     = 0.15;      // LPF for |a| -> g_est (clamped)
  static constexpr double fc_angle_hz = 1.50;      // LPF on final roll/pitch
  static constexpr double g_rel_band  = 0.10;      // clamp |a| to g_est ± band*g_est
  static constexpr double g0_guess    = 9.81;      // init fallback
  static constexpr double fallback_dt_s = 1.0 / 400.0; // used on first/invalid dt
  static constexpr double yaw_fixed_deg = 0.0;     // static mode: constant yaw

  // Cascaded Controller
  static constexpr int   hz_balance        = 400;
  static constexpr int   hz_outer          = 100;
  // Saturations
  static constexpr double max_tilt_rad     = 6.0 * (M_PI / 180.0);

  // Balance PD: tilt error -> sps
  static constexpr double kp_bal           = 60.0;   // [sps/rad]
  static constexpr double kd_bal           = 1.2;    // [sps/(rad/s)]

  // Velocity PI: speed error -> tilt target
  static constexpr double kp_vel           = 1.2;
  static constexpr double ki_vel           = 0.4;

  // Open-loop steering (used if yaw PI disabled)
  static constexpr double k_turn           = 600.0;  // [sps / unit turn]

  // Yaw PI (closed-loop steering)
  static constexpr bool   yaw_pi_enabled   = true;   // enable Z gyro loop
  static constexpr double kp_yaw           = 250.0;  // [sps/(rad/s)]
  static constexpr double ki_yaw           = 80.0;   // [sps/(rad/s*s)]
  static constexpr double max_yaw_rate_cmd = 1.5;    // [rad/s] at |turn|=1
  static constexpr double max_steer_sps    = 800.0;  // clamp steering split


  // Motor Config
  static constexpr int    control_hz    = 100;
  static constexpr double max_sps       = 1200;
  static constexpr float  deadzone      = 0.05f;
  static constexpr bool   invert_left   = false;
  static constexpr bool   invert_right  = false;
};
// ---------------------------------------------------------------------------

static std::atomic<bool> g_stop{false};
static void on_signal(int) { g_stop.store(true, std::memory_order_relaxed); }


// ---- IMU sample (from ISM330DHCX fusion later) ----
// angle_rad: pitch angle (+ forward), gyro_rad_s: pitch rate (+ when nose down)
struct ImuSample {
  double angle_rad = 0.0;
  double gyro_rad_s = 0.0;
  double yaw_rate_z = 0.0;
  std::chrono::steady_clock::time_point t{};
};
