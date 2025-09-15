#pragma once


struct ConfigPid {
// ====== Motor / speed ceiling (primary scaling knob) ======

  static constexpr double max_sps           = 12000.0;      // clamp for wheel speed command (steps/s)
  static constexpr double pitch_out_to_sps  = 3200.0;      // PX4 normalized -> steps/s

  // PX4 Rate PID (inner loop, pitch axis only)
  // Start simple: P only (I,D = 0). Tune rate_P first.
  static constexpr double rate_P      = 0.34;  // try 0.12–0.30
  static constexpr double rate_I      = 0.02;  // keep 0 while tuning
  static constexpr double rate_D      = 0.15;  // keep 0 while tuning
  static constexpr double rate_I_lim  = 0.30;  // unused when I=0
  static constexpr double rate_FF     = 0.00;  // usually 0 for balancing

  // Minimal outer mapping: angle(rad) -> rate_sp(rad/s)
  // This is effectively a proportional controller on angle.
  static constexpr double angle_to_rate_k = 8.0; // rad/s per rad (≈ 0.14 * 180 for deg->dps intuition)

  // Loop rate
  static constexpr int control_hz       = 1000;
};