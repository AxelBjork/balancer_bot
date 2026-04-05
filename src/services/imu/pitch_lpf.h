#pragma once
#include <array>
#include <atomic>
#include <chrono>
#include <functional>

#include "types.h"

class PitchComplementaryFilter {
 public:
  using Acc3 = std::array<double, 3>;
  using Gyr3 = std::array<double, 3>;
  using TimePoint = std::chrono::steady_clock::time_point;

  PitchComplementaryFilter();

  // Writer (single producer)
  void push_sample(const Acc3& acc, const Gyr3& gyrv, TimePoint ts);

  // Adapter-friendly member
  std::function<void(const Acc3&, const Gyr3&, TimePoint)> push_data;

  // Readers (multi-consumer)
  ImuSample read_latest() const;
  void reset();

 private:
  // state
  bool init_{false};
  bool have_last_ts_{false};
  TimePoint last_ts_{};
  Acc3 acc_f_{0.0, 0.0, 0.0};
  double pitch_{0.0};      // fused estimate (rad, +forward)
  double gyro_lpf_{0.0};   // rad/s (pitch axis)
  double gyro_bias_{0.0};  // rad/s

  // lock-free publish
  mutable std::atomic<uint64_t> seq_{0};
  alignas(64) ImuSample pub_{};

  // helpers (declared; defined in .cpp)
  static double exp_coeff(double fc_hz, double dt);
  static double rad2deg(double x);
  static double wrap_pi(double x);
  static double norm(const Acc3& v);
  static double acc_pitch(const Acc3& a);
  bool accel_reliable(const Acc3& a, double pitch_pred) const;
  double compute_dt(TimePoint ts);
  void publish(double pitch, double gyro_pitch, double yaw_rate_z, TimePoint ts);
};
