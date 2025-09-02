#pragma once
#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>

struct AxisCfg {
  int x = 0, y = 1, z = 2;
  bool invert_x = false, invert_y = false, invert_z = false;
};


struct Config {
  // ========= General =========
  static constexpr int   run_seconds   = 30;

  // ========= IMU =========
  static constexpr double sampling_hz  = 833.000; // available 12.500 26.000 52.000 104.000 208.000 416.000 833.000
  static constexpr AxisCfg accel_cfg = {.x = 0, .y = 2, .z = 1, .invert_x = true, .invert_z = true};
  static constexpr AxisCfg gyro_cfg  = {.x = 0, .y = 2, .z = 1, .invert_x = true, .invert_z = true};

  // LPFs (angles from accel, magnitude-to-g estimate, final angle LPF)
  static constexpr double fallback_dt_s        = 1.0 / 400.0;  // Sampling + fallbacks
  static constexpr double fc_gyro_lpf_hz       = 35.0;         // Gyro path, 30–45 Hz: low lag, tame noise
  static constexpr double fc_acc_corr_hz       = 0.8;    // Complementary accel correction (slow), 0.5–1.2 Hz: drift trim without lag
  static constexpr double fc_velocity_hz       = 22.0;   // Velocity estimate (fast), 20–30 Hz: smooths out noise
  static constexpr double g0                   = 9.81;
  static constexpr double g_band_rel           = 0.12;   // accept |a| in [g*(1-..), g*(1+..)]
  static constexpr double max_use_pitch_deg    = 75.0;   // ignore accel when near ±90°
  static constexpr double still_max_rate_dps   = 2.0;    // Stationary detector for gyro bias learning |gyro| < this
  static constexpr double still_max_err_deg    = 3.0;    // |acc_pitch - est| < this
  static constexpr double fc_gyro_bias_hz      = 0.1;    // very slow bias update (~10 s τ)
  static constexpr double fc_acc_prefilt_hz    = 15.0;   // prefilter on accel (helps with vibey bots) 10–20 Hz

  // ========= Controller rates & limits =========
  static constexpr int   hz_balance      = 416;
  static constexpr int   hz_outer        = 100;
  static constexpr double max_tilt_rad   = 5.0 * (M_PI / 180.0);

  // ====== Motor / speed ceiling (primary scaling knob) ======
  // Set this to your *true* max steps-per-second at the wheels.
  static constexpr double max_sps        = 3000.0;

  // ========= Balancer LQR (tilt error -> sps) =========
  static constexpr double max_du_per_sec  = 9000;
  static constexpr int    microstep_mult  = 16;
  static constexpr int    steps_per_rev   = 360/1.8 * microstep_mult;   // includes microsteps
  static constexpr double wheel_radius_m  = 0.04;   // m

  static constexpr double lqr_k_theta     = 88.1637;
  static constexpr double lqr_k_dtheta    = 12.98;
  static constexpr double lqr_k_v         = 5.72;
  static constexpr double tau_u_s         = 0.45;

  // ========= App I/O =========
  static constexpr int    control_hz    = 100;
  static constexpr float  deadzone      = 0.05f;
  static constexpr bool   invert_left   = true;
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

// ---- Joystick command (forward/turn normalized to [-1, 1]) ----
struct JoyCmd {
  float forward;    // + forward speed command
  float turn;       // + left faster, right slower (CCW yaw)
};

struct Telemetry {
  // timing
  std::chrono::steady_clock::time_point ts{};

  // IMU & references
  double tilt_rad = 0.0;
  double gyro_rad_s = 0.0;
  double tilt_target_rad = 0.0;
  bool   tilt_saturated = false;

  // Velocity loop
  double desired_base_sps = 0.0;
  double actual_base_sps  = 0.0;
  double vel_err_sps      = 0.0;
  double vel_int_state    = 0.0;

  // Yaw loop
  double desired_yaw_rate = 0.0; // [rad/s]
  double actual_yaw_rate  = 0.0; // [rad/s]
  double yaw_err          = 0.0; // [rad/s]
  double yaw_int_state    = 0.0; // integrator state

  // Balance & steering
  double u_balance_sps   = 0.0;
  double steer_split_sps = 0.0;

  // Motor commands
  double left_cmd_sps  = 0.0;
  double right_cmd_sps = 0.0;
};