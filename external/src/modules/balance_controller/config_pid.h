#pragma once

struct ConfigPid {
  // ====== Motor / speed ceiling (primary scaling knob) ======

  static constexpr double max_sps = 4000.0;           // clamp for wheel speed command (steps/s)
  static constexpr double pitch_out_to_sps = 3200.0;  // PX4 normalized -> steps/s

  // PX4 Rate PID (inner loop, pitch axis only)
  // Start simple: P only (I,D = 0). Tune rate_P first.
  static constexpr double rate_P = 0.75;   // 0.75
  static constexpr double rate_I = 0.0;   // 0.02
  static constexpr double rate_D = 0.5;   // 0.12
  static constexpr double rate_I_lim = 0.15;
  static constexpr double rate_FF = 0.0;  // 0.06

  // Minimal outer mapping: angle(rad) -> rate_sp(rad/s)
  // This is effectively a proportional controller on angle.
  static constexpr double angle_to_rate_k =
      6.0;  // Reduced from 15.0 to prevent whiplash

  // Velocity PID (outermost loop): velocity error -> pitch angle setpoint
  // Split-Rate Control: Velocity loop runs at 10 Hz (decimated)
  // This allows stronger gains without saturation/instability
  static constexpr int velocity_decimation = 40;         // Run every 40th cycle (400Hz -> 10Hz)
  
  static constexpr double vel_P = -0.000005;             // Reduced damping to prevent saturation
  static constexpr double vel_I = -0.000005;             // Moderate integrator
  static constexpr double vel_D = 0.0;                   // derivative gain
  static constexpr double vel_I_lim = 0.15;              // Allow reasonable windup for correction
  static constexpr double max_pitch_setpoint_rad = 0.3;  // ~17 degrees max lean

  // Loop rate
  static constexpr int control_hz = 400;
};
