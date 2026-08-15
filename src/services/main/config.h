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

  static constexpr double g0 = 9.81;
  // PX4-style signal conditioning followed by bounded gyro/gravity pitch.
  // No mounting, gyro-bias, gravity-recovery, or COM correction is learned.
  static constexpr double imu_accel_lpf_hz = 15.0;
  static constexpr double imu_gyro_lpf_hz = 30.0;
  static constexpr double imu_gyro_derivative_lpf_hz = 10.0;
  // Gravity anchors DC pitch while the gyro carries short-term motion.
  static constexpr double imu_attitude_correction_hz = 0.5;
  static constexpr double imu_gravity_innovation_limit_rad = 2.5 * M_PI / 180.0;
  static constexpr double imu_gyro_notch_hz = 0.0;
  static constexpr double imu_gyro_notch_bandwidth_hz = 0.0;
  static constexpr double imu_max_sample_gap_periods = 4.0;
  static constexpr double imu_height_m = 0.070;
  static constexpr bool imu_lever_arm_correction_enabled = false;
  static constexpr double imu_specific_force_min_mps2 = 0.1 * g0;
  static constexpr double imu_specific_force_max_mps2 = 3.5 * g0;
  // Corrected axle velocity is used only by the 100 Hz outer damping/trim loop.
  // 10 Hz removes completed-step quantization without the phase lag that made
  // the proposed 1 Hz filter miss the rocking-disturbance catch.
  static constexpr double fc_velocity_hz = 10.0;

  // ========= Controller rates & limits =========
  static constexpr int control_hz = 400;  // Main control loop frequency
  // Pulse frequency can change much faster than the wheel itself; the phase-
  // error/missed-step plant supplies the physical acceleration limit.
  static constexpr double motor_slew_sps_per_s = 200000.0;

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

};

static_assert(Config::sampling_hz > 0.0);
static_assert(Config::imu_accel_lpf_hz > 0.0 &&
              Config::imu_accel_lpf_hz < Config::sampling_hz / 2.0);
static_assert(Config::imu_gyro_lpf_hz > 0.0 &&
              Config::imu_gyro_lpf_hz < Config::sampling_hz / 2.0);
static_assert(Config::imu_gyro_derivative_lpf_hz > 0.0 &&
              Config::imu_gyro_derivative_lpf_hz < Config::sampling_hz / 2.0);
static_assert(Config::imu_attitude_correction_hz > 0.0 &&
              Config::imu_attitude_correction_hz < Config::sampling_hz / 2.0);
static_assert(Config::imu_gravity_innovation_limit_rad > 0.0 &&
              Config::imu_gravity_innovation_limit_rad < M_PI);
static_assert((Config::imu_gyro_notch_hz == 0.0 &&
               Config::imu_gyro_notch_bandwidth_hz == 0.0) ||
              (Config::imu_gyro_notch_hz > 0.0 &&
               Config::imu_gyro_notch_hz < Config::sampling_hz / 2.0 &&
               Config::imu_gyro_notch_bandwidth_hz > 0.0));
static_assert(Config::imu_max_sample_gap_periods > 1.0);
static_assert(Config::imu_height_m >= 0.0);
static_assert(Config::imu_specific_force_min_mps2 > 0.0);
static_assert(Config::imu_specific_force_max_mps2 >
              Config::imu_specific_force_min_mps2);

// ---------------------------------------------------------------------------

static std::atomic<bool> g_stop{false};
inline void on_signal(int) {
  g_stop.store(true, std::memory_order_relaxed);
}
