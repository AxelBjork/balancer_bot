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
  static constexpr double fc_gyro_lpf_hz       = 80.0;         // Gyro path, 30–45 Hz: low lag, tame noise
  static constexpr double fc_acc_corr_hz       = 0.8;    // Complementary accel correction (slow), 0.5–1.2 Hz: drift trim without lag
  static constexpr double fc_velocity_hz       = 80.0;   // Velocity estimate (fast), 20–30 Hz: smooths out noise
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
  static constexpr double max_sps        = 6000.0;

  // ========= Balancer LQR (tilt error -> sps) =========
  static constexpr double max_du_per_sec  = 120000;
  static constexpr int    microstep_mult  = 16;
  static constexpr int    steps_per_rev   = 360/1.8 * microstep_mult;   // includes microsteps
  static constexpr double wheel_radius_m  = 0.04;   // m

  static constexpr double lqr_k_theta     = 4.00;
  static constexpr double lqr_k_dtheta    = 2.50;
  static constexpr double lqr_k_v         = 1.50;
  static constexpr double tau_u_s         = 0.45;

  // ========= App I/O =========
  static constexpr int    control_hz    = 416;
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

// ---- Telemetry (per-term LQR contributions) ----
struct Telemetry {
  // timing
  std::chrono::steady_clock::time_point ts{};

  // IMU
  double tilt_rad     = 0.0;  // θ
  double gyro_rad_s   = 0.0;  // θ̇

  // Velocity estimate (from SpeedEstimator on commanded sps)
  double x_vel_est_mps = 0.0; // m/s

  // LQR contributions (all in m/s², sign as applied to a_cmd)
  double a_theta_mps2   = 0.0; // -Kθ * θ
  double a_dtheta_mps2  = 0.0; // -Kdθ * θ̇
  double a_v_mps2       = 0.0; // -Kv  * ẋ
  double a_cmd_mps2     = 0.0; // sum

  // Inner speed integrator
  double du_sps          = 0.0; // steps/s increment this tick (after rate limit)
  bool   du_rate_limited = false;
  double u_balance_sps   = 0.0; // commanded base steps/s (post-leak/clamp)
  bool   u_amp_limited   = false;

  // Motor commands
  double left_cmd_sps  = 0.0;
  double right_cmd_sps = 0.0;
};
