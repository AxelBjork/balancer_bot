#pragma once

#include <array>
#include <chrono>
#include <optional>

#include <lib/mathlib/math/filter/FilteredDerivative.hpp>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/mathlib/math/filter/NotchFilter.hpp>

#include "messages/types.h"

namespace imu_pitch_detail {

// Solves algebraic gravity pitch after the optional IMU lever-arm acceleration
// has been removed. Exposed so the rigid-body signs and full-circle convention
// can be tested independently of signal conditioning.
std::optional<double> solve_pitch(double acc_x, double acc_z, double gyro_rate,
                                  double gyro_accel);

}  // namespace imu_pitch_detail

struct ImuPitchEstimate {
  ImuSample sample{};
  bool valid{false};
};

class ImuPitchEstimator {
 public:
  using Acc3 = std::array<double, 3>;
  using Gyr3 = std::array<double, 3>;
  using TimePoint = std::chrono::steady_clock::time_point;

  struct Settings {
    struct GyroNotch {
      double center_hz{0.0};
      double bandwidth_hz{0.0};
    };

    double sample_hz{0.0};
    double accel_lpf_hz{0.0};
    double gyro_derivative_lpf_hz{0.0};
    double attitude_correction_hz{0.0};
    double gravity_innovation_limit_rad{0.0};
    std::array<GyroNotch, 2> gyro_notches{};
    double max_sample_gap_periods{0.0};
    double imu_height_m{0.0};
    double specific_force_min_mps2{0.0};
    double specific_force_max_mps2{0.0};
    bool lever_arm_correction_enabled{false};

    static Settings production();
  };

  ImuPitchEstimator();
  explicit ImuPitchEstimator(Settings settings);

  ImuPitchEstimate push_sample(const Acc3& acc, const Gyr3& gyr, TimePoint ts);
  void reset();

  // Test-only simulator hook. Hardware and production code never call this;
  // it lets a recovery experiment start with a known attitude while keeping
  // the normal first-sample filter initialization and sensor path.
  void set_initial_pitch_for_simulation(double pitch_rad);

 private:
  static bool finite3(const Acc3& value);
  ImuPitchEstimate seed(const Acc3& acc, const Gyr3& gyr, TimePoint ts);
  ImuPitchEstimate make_estimate(double acc_x, double acc_z, double gyro_rate,
                                 double gyro_accel, double yaw_rate, TimePoint ts);

  Settings settings_;
  math::LowPassFilter2p<float> accel_x_lpf_;
  math::LowPassFilter2p<float> accel_z_lpf_;
  std::array<math::NotchFilter<float>, 2> gyro_y_notches_;
  const float gyro_derivative_time_constant_s_;
  FilteredDerivative<float> gyro_y_derivative_;
  TimePoint last_timestamp_{};
  double pitch_rad_{0.0};
  bool initialized_{false};
  std::optional<double> initial_pitch_override_rad_;
};
