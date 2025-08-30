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