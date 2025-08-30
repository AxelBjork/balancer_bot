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
  // ========= General =========
  static constexpr int   run_seconds   = 30;

  // ========= IMU =========
  static constexpr double sampling_hz  = 833.000; // available 12.500 26.000 52.000 104.000 208.000 416.000 833.000
  static constexpr AxisCfg accel_cfg = {.x = 0, .y = 2, .z = 1, .invert_x = true, .invert_z = true};
  static constexpr AxisCfg gyro_cfg  = {.x = 0, .y = 2, .z = 1, .invert_x = true, .invert_z = true};

  // LPFs (angles from accel, magnitude-to-g estimate, final angle LPF)
  static constexpr double fc_acc_hz      = 10.0;
  static constexpr double fc_g_hz        = 0.15;
  static constexpr double fc_angle_hz    = 1.50;
  static constexpr double g_rel_band     = 0.10;
  static constexpr double g0_guess       = 9.81;
  static constexpr double fallback_dt_s  = 1.0 / 400.0;
  static constexpr double yaw_fixed_deg  = 0.0;
  // D-path gyro LPF (pitch axis) — calmer D = 20–35 Hz is typical
  static constexpr double fc_gyro_pitch_hz = 40.0;   // Hz

  // ========= Controller rates & limits =========
  static constexpr int   hz_balance      = 100;
  static constexpr int   hz_outer        = 100;
  static constexpr double max_tilt_rad   = 5.0 * (M_PI / 180.0);

  // ====== Motor / speed ceiling (primary scaling knob) ======
  // Set this to your *true* max steps-per-second at the wheels.
  static constexpr double max_sps        = 26500.0;   // example for 16× microstepping; adjust as needed

  // ========= Balance PD (tilt error -> sps) =========
  // Scale Kp as a fraction of max_sps per rad.
  // Example: kp_frac = 0.30 means 0.30*max_sps per 1 rad (~17.45°) of error.
  static constexpr double kp_bal_frac    = -0.04;     // unit: (sps/rad) / max_sps
  // Use derivative time so Kd tracks Kp automatically (Kd = Kp * Td).
  // 0.06–0.09 s is a good starting range; smaller = snappier, larger = more damping.
  static constexpr double Td_bal_s       = 0.6;     // seconds

  static constexpr double kp_bal         = kp_bal_frac * max_sps;
  static constexpr double kd_bal         = kp_bal * Td_bal_s;  // [sps/(rad/s)]

  // ========= Velocity PI (speed error -> tilt target) =========
  // These already normalize by max_sps in your code; keep as-is.
  static constexpr double kp_vel         = 0.8;
  static constexpr double ki_vel         = 0.1;

  // ========= Steering =========
  // Open-loop steering gain (unit turn -> sps), as a fraction of max_sps.
  static constexpr double k_turn_frac    = 0.75;
  static constexpr double k_turn         = k_turn_frac * max_sps;

  // Yaw PI (if you enable later). Gains scale with max_sps too.
  static constexpr bool   yaw_pi_enabled   = false;
  static constexpr double kp_yaw_frac      = 0.20;   // per (rad/s), scaled by max_sps
  static constexpr double ki_yaw_frac      = 0.14;   // per (rad/s*s), scaled by max_sps
  static constexpr double kp_yaw           = kp_yaw_frac * max_sps;
  static constexpr double ki_yaw           = ki_yaw_frac * max_sps;
  static constexpr double max_yaw_rate_cmd = 1.5;    // [rad/s] at |turn|=1
  static constexpr double max_steer_frac   = 0.90;
  static constexpr double max_steer_sps    = max_steer_frac * max_sps;

  // ========= Command shaping (scales with max_sps) =========
  // // Slew: fraction of max_sps per second (e.g., 0.35 * max_sps / s)
  static constexpr double slew_per_sec_frac   = 0.35;
  static constexpr double slew_per_sec         = slew_per_sec_frac * max_sps;


  // // Gyro clamp for D path (deg/s) — not tied to max_sps; hardware/IMU dependent.
  static constexpr double gyro_d_abs_limit_deg_s = 500.0;

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