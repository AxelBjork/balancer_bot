#pragma once
#include <atomic>
// ---------------------- Compile-time configuration ---------------------------
struct Config {
  static inline constexpr int   run_seconds   = 30;
  static inline constexpr int   control_hz    = 100;
  static inline constexpr int   max_sps       = 1200;
  static inline constexpr double sampling_hz  = 416.000; // available 12.500 26.000 52.000 104.000 208.000 416.000 833.000
  static inline constexpr float deadzone      = 0.05f;
  static inline constexpr bool  invert_left   = false;
  static inline constexpr bool  invert_right  = false;
};
// ---------------------------------------------------------------------------

static std::atomic<bool> g_stop{false};
static void on_signal(int) { g_stop.store(true, std::memory_order_relaxed); }