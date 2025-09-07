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
  static constexpr int   run_seconds   = 50;

  // ========= IMU =========
  static constexpr double sampling_hz  = 833.000; // available 12.500 26.000 52.000 104.000 208.000 416.000 833.000
  static constexpr AxisCfg accel_cfg = {.x = 0, .y = 2, .z = 1, .invert_x = true, .invert_z = true};
  static constexpr AxisCfg gyro_cfg  = {.x = 0, .y = 2, .z = 1, .invert_x = true, .invert_z = true};

  // LPFs (angles from accel, magnitude-to-g estimate, final angle LPF)
  static constexpr double fallback_dt_s        = 1.0 / 400.0;  // Sampling + fallbacks
  static constexpr double fc_gyro_lpf_hz       = 100.0;         // Gyro path, 30–45 Hz: low lag, tame noise
  static constexpr double fc_acc_corr_hz       = 2.2;    // Complementary accel correction (slow), 0.5–1.2 Hz: drift trim without lag
  static constexpr double fc_velocity_hz       = 50.0;   // Velocity estimate (fast), 20–30 Hz: smooths out noise
  static constexpr double g0                   = 9.81;
  static constexpr double g_band_rel           = 0.12;   // accept |a| in [g*(1-..), g*(1+..)]
  static constexpr double max_use_pitch_deg    = 75.0;   // ignore accel when near ±90°
  static constexpr double still_max_rate_dps   = 2.0;    // Stationary detector for gyro bias learning |gyro| < this
  static constexpr double still_max_err_deg    = 3.0;    // |acc_pitch - est| < this
  static constexpr double fc_gyro_bias_hz      = 0.2;    // very slow bias update (~10 s τ)
  static constexpr double fc_acc_prefilt_hz    = 30.0;   // prefilter on accel (helps with vibey bots) 10–20 Hz

  // ========= Controller rates & limits =========
  static constexpr int   hz_balance      = 400;
  static constexpr int   hz_outer        = 100;
  static constexpr double max_tilt_rad   = 5.0 * (M_PI / 180.0);

  // ====== Motor / speed ceiling (primary scaling knob) ======
  // Set this to your *true* max steps-per-second at the wheels.
  static constexpr double max_sps        = 6000.0;

  // ========= Balancer LQR (tilt error -> sps) =========
  static constexpr double max_du_per_sec  = 120000;
  static constexpr double deadzone_frac   = 0.005;     // 5–10% of max_sps

  static constexpr int    microstep_mult  = 16;
  static constexpr int    steps_per_rev   = 360/1.8 * microstep_mult;   // includes microsteps
  static constexpr double wheel_radius_m  = 0.04;   // m
  static constexpr double lead_T_s        = 0.010;     // 30–60 ms works well; start 0.04


  // ===== PX4 Rate PID (inner loop, pitch axis only) =====
  static constexpr double rate_P      = 0.14;   // P gain on pitch rate error
  static constexpr double rate_I      = 0.10;   // I gain on pitch rate error
  static constexpr double rate_D      = 0.006;   // D gain on angular accel (start 0; 0.001–0.003 later)
  static constexpr double rate_I_lim  = 0.40;   // integrator clamp (rad/s units inside PX4 rate_control)
  static constexpr double rate_FF     = 0.00;   // feed-forward on rate setpoint (usually 0 for balancing)

  // ===== Outer attitude PI (deg -> deg/s setpoint) =====
  static constexpr double att_kp_deg_s_per_deg    = 4.5;  // P: deg/s per deg of pitch error
  static constexpr double att_ki_deg_s_per_deg_s  = 0.10;  // I: deg/s per (deg·s) (start 0)
  static constexpr double att_int_lim_deg_s       = 40.0; // clamp for outer I (deg/s)
  static constexpr double att_i_freeze_deg        = 18.0; // freeze outer I beyond this |pitch| (set 180 to disable)

  // ===== Outer shaping =====
  static constexpr double rate_sp_max_dps   = 80.0; // soft cap for rate setpoint
  static constexpr double outer_knee_deg    = 6.0;   // linear region around upright

  // ===== D-term accel LPF =====
  static constexpr double acc_lpf_hz        = 50.0;  // 50–90 Hz is a good start

  // ===== Output mapping (controller -> wheels) =====
  static constexpr double pitch_out_to_sps = 3200.0; // normalized PX4 output → steps/s scaling

  
  // ========= App I/O =========
  static constexpr int    control_hz    = 400;
  static constexpr int    kPrintEvery   = 50;
  static constexpr float  deadzone      = 0.05f;
  static constexpr bool   invert_left   = true;
  static constexpr bool   invert_right  = false;

// ========= Time Budget =========
  static constexpr double pitch_rise_ms = 200.0;     // accel correction must be <= 200 ms (10->90)
  static constexpr double dpitch_rise_ms = 6.0;   // gyro path must be <= ~6 ms 10->90
};



static_assert(Config::fc_acc_corr_hz >= 0.35 / (Config::pitch_rise_ms/1000.0),
              "fc_acc_corr_hz too low for required rise time budget");

static_assert( (2.2 / (2*M_PI*Config::fc_gyro_lpf_hz)) * 1e3 <= Config::dpitch_rise_ms,
               "fc_gyro_lpf_hz too low for required gyro rise-time budget" );


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
  double t_sec;
  double pitch_deg;
  double pitch_rate_dps;
  double rate_sp_dps;
  double out_norm;     // PX4 rate controller normalized output (pitch axis)
  double u_sps;        // wheel command [steps/s]
  double integ_pitch;  // PX4 integral state for pitch
};

