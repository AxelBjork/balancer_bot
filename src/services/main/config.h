#pragma once
#include <atomic>
#include <cmath>

struct AxisCfg {
  int x = 0, y = 1, z = 2;
  bool invert_x = false, invert_y = false, invert_z = false;
};

struct Config {
  // ========= General =========
  static constexpr int run_seconds = 120;
  static constexpr double wheel_diam_m = 0.080;              // 80 mm
  static constexpr double steps_per_rev = 360 / 1.8 * 16.0;  // 1.8° * 16x
  static constexpr double meters_per_step = M_PI * wheel_diam_m / steps_per_rev;

  // ========= IMU =========
  static constexpr double sampling_hz = 833.000;  // available 12.5 26 52 104 208 416 833
  static constexpr AxisCfg accel_cfg = {.x = 0, .y = 2, .z = 1, .invert_x = true, .invert_z = true};
  static constexpr AxisCfg gyro_cfg = {.x = 0, .y = 2, .z = 1, .invert_x = true, .invert_z = true};

  // LPFs (angles from accel, magnitude-to-g estimate, final angle LPF)
  static constexpr double fallback_dt_s = 1.0 / 400.0;  // Sampling + fallbacks
  static constexpr double fc_gyro_lpf_hz = 100.0;       // Gyro path: low lag while taming sensor noise
  static constexpr double fc_gyro_accel_lpf_hz = 30.0;  // Gyro derivative path for rate D
  // Complementary accel correction (slow)
  // Accelerometer correction must remain slow: wheel acceleration is
  // indistinguishable from tilt in a two-wheel balancing robot. Gyro carries
  // the dynamic attitude estimate; accel only removes long-term drift.
  static constexpr double fc_acc_corr_hz = 0.05;
  // Completed motor steps are a balance-state input and retain enough
  // bandwidth to catch the pole. The governed target is fed forward directly.
  static constexpr double fc_velocity_hz = 50.0;
  static constexpr double g0 = 9.81;
  static constexpr double g_band_rel = 0.12;  // accept |a| in [g*(1-..), g*(1+..)]
  // Horizontal specific force is indistinguishable from tilt. Bound each
  // correction so gravity remains a continuous reference without allowing a
  // dynamic acceleration to abruptly steer the estimate.
  static constexpr double accel_correction_max_innovation_deg = 3.0;
  static constexpr double fc_acc_prefilt_hz = 30.0;  // prefilter on accel (10–20 Hz)

  // ========= Controller rates & limits =========
  static constexpr int control_hz = 400;  // Main control loop frequency
  // Pulse frequency can change much faster than the wheel itself; the phase-
  // error/missed-step plant supplies the physical acceleration limit.
  static constexpr double motor_slew_sps_per_s = 100000.0;

  static constexpr double max_tilt_rad = 25.0 * (M_PI / 180.0);

  static constexpr int command_hz = 100;
  static constexpr int kPrintEvery = 100;
  static constexpr double deadzone = 0.05;
  // Public motion commands use robot-forward as positive. These electrical
  // inversions preserve the calibrated physical wheel polarity for the
  // mirrored motor installation; inversion remains confined to the hardware
  // boundary so controller, simulator, joystick, and telemetry signs agree.
  static constexpr bool invert_left = true;
  static constexpr bool invert_right = false;

  // ========= Time Budget =========
  static constexpr double pitch_rise_ms = 7000.0;
  static constexpr double dpitch_rise_ms = 6.0;   // gyro path must be <= ~6 ms 10->90
};

static_assert(Config::fc_acc_corr_hz == 0.0 ||
                  Config::fc_acc_corr_hz >= 0.35 / (Config::pitch_rise_ms / 1000.0),
              "fc_acc_corr_hz too low for required rise time budget");

static_assert((2.2 / (2 * M_PI * Config::fc_gyro_lpf_hz)) * 1e3 <= Config::dpitch_rise_ms,
              "fc_gyro_lpf_hz too low for required gyro rise-time budget");

// ---------------------------------------------------------------------------

static std::atomic<bool> g_stop{false};
inline void on_signal(int) {
  g_stop.store(true, std::memory_order_relaxed);
}
