#pragma once
#include <atomic>

struct AxisCfg {
  int x = 0, y = 1, z = 2;
  bool invert_x = false, invert_y = false, invert_z = false;
};
// ---------------------- Compile-time configuration ---------------------------
struct Config {
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

  static constexpr int   run_seconds   = 30;
  static constexpr double sampling_hz  = 833.000; // available 12.500 26.000 52.000 104.000 208.000 416.000 833.000
  // Motor Control
  static constexpr int   control_hz    = 100;
  static constexpr int   max_sps       = 1200;
  static constexpr float deadzone      = 0.05f;
  static constexpr bool  invert_left   = false;
  static constexpr bool  invert_right  = false;
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
