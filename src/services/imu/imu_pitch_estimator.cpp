#include "imu_pitch_estimator.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "services/main/config.h"

namespace {

using Settings = ImuPitchEstimator::Settings;
using TimePoint = ImuPitchEstimator::TimePoint;

// Locked inner-loop vibration rejection. These are deliberately IMU
// implementation details rather than live PID/configuration parameters: the
// controller should always receive the same conditioned pitch-rate signal.
constexpr std::array<ImuPitchEstimator::Settings::GyroNotch, 2>
    kProductionGyroNotches{{
        {.center_hz = Config::imu_gyro_notch_1_center_hz,
         .bandwidth_hz = Config::imu_gyro_notch_1_bandwidth_hz},
        {.center_hz = Config::imu_gyro_notch_2_center_hz,
         .bandwidth_hz = Config::imu_gyro_notch_2_bandwidth_hz},
    }};

double seconds_between(TimePoint newer, TimePoint older) {
  return std::chrono::duration<double>(newer - older).count();
}

bool valid_cutoff(double cutoff_hz, double sample_hz) {
  return std::isfinite(cutoff_hz) && cutoff_hz > 0.0 && cutoff_hz < sample_hz / 2.0;
}

void validate_settings(const Settings& settings) {
  if (!std::isfinite(settings.sample_hz) || settings.sample_hz <= 0.0 ||
      !valid_cutoff(settings.accel_lpf_hz, settings.sample_hz) ||
      !valid_cutoff(settings.gyro_derivative_lpf_hz, settings.sample_hz) ||
      !valid_cutoff(settings.attitude_correction_hz, settings.sample_hz) ||
      !std::isfinite(settings.gravity_innovation_limit_rad) ||
      settings.gravity_innovation_limit_rad <= 0.0 ||
      settings.gravity_innovation_limit_rad >= M_PI ||
      !std::isfinite(settings.max_sample_gap_periods) ||
      settings.max_sample_gap_periods <= 1.0 || !std::isfinite(settings.imu_height_m) ||
      settings.imu_height_m < 0.0 ||
      !std::isfinite(settings.specific_force_min_mps2) ||
      settings.specific_force_min_mps2 <= 0.0 ||
      !std::isfinite(settings.specific_force_max_mps2) ||
      settings.specific_force_max_mps2 <= settings.specific_force_min_mps2) {
    throw std::invalid_argument("invalid IMU pitch estimator settings");
  }

  if (settings.gyro_software_lpf_hz != 0.0 &&
      !valid_cutoff(settings.gyro_software_lpf_hz, settings.sample_hz)) {
    throw std::invalid_argument("invalid legacy IMU gyro low-pass settings");
  }

  for (const auto& notch : settings.gyro_notches) {
    const bool notch_disabled = notch.center_hz == 0.0 && notch.bandwidth_hz == 0.0;
    const bool notch_valid =
        valid_cutoff(notch.center_hz, settings.sample_hz) &&
        std::isfinite(notch.bandwidth_hz) && notch.bandwidth_hz > 0.0;
    if (!notch_disabled && !notch_valid) {
      throw std::invalid_argument("invalid IMU gyro notch settings");
    }
  }
}

std::optional<double> solve_pitch_with_settings(double acc_x, double acc_z,
                                                double gyro_rate, double gyro_accel,
                                                const Settings& settings,
                                                bool apply_lever_arm) {
  if (!std::isfinite(acc_x) || !std::isfinite(acc_z) ||
      !std::isfinite(gyro_rate) || !std::isfinite(gyro_accel)) {
    return std::nullopt;
  }

  double pitch_plane_x = acc_x;
  double pitch_plane_z = acc_z;
  if (apply_lever_arm) {
    pitch_plane_x -= settings.imu_height_m * gyro_accel;
    pitch_plane_z += settings.imu_height_m * gyro_rate * gyro_rate;
  }

  const double magnitude = std::hypot(pitch_plane_x, pitch_plane_z);
  if (!std::isfinite(magnitude) ||
      magnitude < settings.specific_force_min_mps2 ||
      magnitude > settings.specific_force_max_mps2) {
    return std::nullopt;
  }

  return std::atan2(-pitch_plane_x, -pitch_plane_z);
}

}  // namespace

namespace imu_pitch_detail {

std::optional<double> solve_pitch(double acc_x, double acc_z, double gyro_rate,
                                  double gyro_accel) {
  return solve_pitch_with_settings(acc_x, acc_z, gyro_rate, gyro_accel,
                                   Settings::production(), true);
}

}  // namespace imu_pitch_detail

ImuPitchEstimator::Settings ImuPitchEstimator::Settings::production() {
  return Settings{
      .sample_hz = Config::sampling_hz,
      .accel_lpf_hz = Config::imu_accel_lpf_hz,
      .gyro_software_lpf_hz = 0.0,
      .gyro_derivative_lpf_hz = Config::imu_gyro_derivative_lpf_hz,
      .attitude_correction_hz = Config::imu_attitude_correction_hz,
      .gravity_innovation_limit_rad = Config::imu_gravity_innovation_limit_rad,
      .gyro_notches = kProductionGyroNotches,
      .max_sample_gap_periods = Config::imu_max_sample_gap_periods,
      .imu_height_m = Config::imu_height_m,
      .specific_force_min_mps2 = Config::imu_specific_force_min_mps2,
      .specific_force_max_mps2 = Config::imu_specific_force_max_mps2,
      .lever_arm_correction_enabled = Config::imu_lever_arm_correction_enabled,
  };
}

ImuPitchEstimator::Settings ImuPitchEstimator::Settings::legacy_simulation_reference() {
  auto settings = Settings::production();
  // Keep the simulator's historical software-conditioning reference explicit,
  // but use the current production baseline for comparison: 32 Hz / 10 Hz
  // notch followed by the 30 Hz two-pole software gyro LPF.
  settings.gyro_notches = {{{.center_hz = 32.0, .bandwidth_hz = 10.0},
                            {.center_hz = 0.0, .bandwidth_hz = 0.0}}};
  settings.gyro_software_lpf_hz = 30.0;
  return settings;
}

ImuPitchEstimator::ImuPitchEstimator() : ImuPitchEstimator(Settings::production()) {
}

ImuPitchEstimator::ImuPitchEstimator(Settings settings)
    : settings_(settings),
      accel_x_lpf_(static_cast<float>(settings.sample_hz),
                   static_cast<float>(settings.accel_lpf_hz)),
      accel_z_lpf_(static_cast<float>(settings.sample_hz),
                   static_cast<float>(settings.accel_lpf_hz)),
      gyro_derivative_time_constant_s_(static_cast<float>(
          1.0 / (2.0 * M_PI * settings.gyro_derivative_lpf_hz))) {
  validate_settings(settings_);
  for (std::size_t index = 0; index < settings_.gyro_notches.size(); ++index) {
    const auto& notch = settings_.gyro_notches[index];
    if (notch.center_hz > 0.0) {
      const bool configured = gyro_y_notches_[index].setParameters(
          static_cast<float>(settings_.sample_hz),
          static_cast<float>(notch.center_hz),
          static_cast<float>(notch.bandwidth_hz));
      if (!configured) {
        throw std::invalid_argument("failed to configure IMU gyro notch");
      }
    } else {
      gyro_y_notches_[index].disable();
    }
  }
  if (settings_.gyro_software_lpf_hz > 0.0) {
    gyro_y_legacy_lpf_ = std::make_unique<math::LowPassFilter2p<float>>(
        static_cast<float>(settings_.sample_hz),
        static_cast<float>(settings_.gyro_software_lpf_hz));
  }
  gyro_y_derivative_.setParameters(
      static_cast<float>(1.0 / settings_.sample_hz),
      gyro_derivative_time_constant_s_);
}

ImuPitchEstimate ImuPitchEstimator::push_sample(const Acc3& acc, const Gyr3& gyr,
                                                TimePoint ts) {
  ImuPitchEstimate invalid;
  invalid.sample.t = ts;
  if (!finite3(acc) || !finite3(gyr)) {
    reset();
    return invalid;
  }

  if (!initialized_) {
    return seed(acc, gyr, ts);
  }

  const double dt_s = seconds_between(ts, last_timestamp_);
  if (!std::isfinite(dt_s) || dt_s <= 0.0) {
    reset();
    return invalid;
  }

  const double max_gap_s = settings_.max_sample_gap_periods / settings_.sample_hz;
  if (dt_s > max_gap_s) {
    reset();
    return seed(acc, gyr, ts);
  }

  const float acc_x = static_cast<float>(acc[0]);
  const float acc_z = static_cast<float>(acc[2]);
  const float gyro_y = static_cast<float>(gyr[1]);
  if (!std::isfinite(acc_x) || !std::isfinite(acc_z) || !std::isfinite(gyro_y)) {
    reset();
    return invalid;
  }

  const double filtered_acc_x = accel_x_lpf_.apply(acc_x);
  const double filtered_acc_z = accel_z_lpf_.apply(acc_z);
  const float lower_notched_gyro_y = gyro_y_notches_[0].apply(gyro_y);
  const float notched_gyro_y = gyro_y_notches_[1].apply(lower_notched_gyro_y);
  // The ISM330's chip-side LPF1 (140 Hz at the production 833 Hz ODR) is the
  // high-frequency conditioning stage. Keep the software path to the
  // controller as notch-only so the balance loop does not pay the delay of a
  // second generic 30 Hz low-pass filter.
  const double filtered_gyro_y = gyro_y_legacy_lpf_
                                     ? gyro_y_legacy_lpf_->apply(notched_gyro_y)
                                     : notched_gyro_y;
  gyro_y_derivative_.setParameters(
      static_cast<float>(dt_s),
      gyro_derivative_time_constant_s_);
  const double filtered_gyro_accel =
      gyro_y_derivative_.update(static_cast<float>(filtered_gyro_y));
  last_timestamp_ = ts;

  auto result = make_estimate(filtered_acc_x, filtered_acc_z, filtered_gyro_y,
                              filtered_gyro_accel, gyr[2], ts);
  if (!result.valid) return result;

  const double predicted_pitch =
      std::remainder(pitch_rad_ + filtered_gyro_y * dt_s, 2.0 * M_PI);
  const double gravity_innovation =
      std::remainder(result.sample.angle_rad - predicted_pitch, 2.0 * M_PI);
  const double bounded_innovation =
      std::clamp(gravity_innovation, -settings_.gravity_innovation_limit_rad,
                 settings_.gravity_innovation_limit_rad);
  const double gravity_gain =
      1.0 - std::exp(-2.0 * M_PI * settings_.attitude_correction_hz * dt_s);
  pitch_rad_ = std::remainder(predicted_pitch + gravity_gain * bounded_innovation,
                              2.0 * M_PI);
  result.sample.angle_rad = pitch_rad_;
  return result;
}

void ImuPitchEstimator::reset() {
  initialized_ = false;
  last_timestamp_ = {};
  pitch_rad_ = 0.0;
  gyro_y_notches_[0].reset();
  gyro_y_notches_[1].reset();
  if (gyro_y_legacy_lpf_) gyro_y_legacy_lpf_->reset(0.0f);
  gyro_y_derivative_.reset(0.0f);
}

void ImuPitchEstimator::set_initial_pitch_for_simulation(double pitch_rad) {
  if (std::isfinite(pitch_rad)) {
    initial_pitch_override_rad_ = std::remainder(pitch_rad, 2.0 * M_PI);
  } else {
    initial_pitch_override_rad_.reset();
  }
}

bool ImuPitchEstimator::finite3(const Acc3& value) {
  return std::all_of(value.begin(), value.end(),
                     [](double item) { return std::isfinite(item); });
}

ImuPitchEstimate ImuPitchEstimator::seed(const Acc3& acc, const Gyr3& gyr,
                                         TimePoint ts) {
  ImuPitchEstimate invalid;
  invalid.sample.t = ts;

  const float acc_x = static_cast<float>(acc[0]);
  const float acc_z = static_cast<float>(acc[2]);
  const float gyro_y = static_cast<float>(gyr[1]);
  if (!std::isfinite(acc_x) || !std::isfinite(acc_z) || !std::isfinite(gyro_y)) {
    reset();
    return invalid;
  }

  const double filtered_acc_x = accel_x_lpf_.reset(acc_x);
  const double filtered_acc_z = accel_z_lpf_.reset(acc_z);
  gyro_y_notches_[0].reset(gyro_y);
  gyro_y_notches_[1].reset(gyro_y);
  float filtered_gyro_y = gyro_y;
  if (gyro_y_legacy_lpf_) {
    filtered_gyro_y = gyro_y_legacy_lpf_->reset(filtered_gyro_y);
  }
  gyro_y_derivative_.setParameters(
      static_cast<float>(1.0 / settings_.sample_hz),
      gyro_derivative_time_constant_s_);
  gyro_y_derivative_.reset(0.0f);
  (void)gyro_y_derivative_.update(filtered_gyro_y);
  initialized_ = true;
  last_timestamp_ = ts;

  auto result = make_estimate(filtered_acc_x, filtered_acc_z, filtered_gyro_y, 0.0,
                              gyr[2], ts);
  if (result.valid) {
    pitch_rad_ = initial_pitch_override_rad_.value_or(0.0);
    initial_pitch_override_rad_.reset();
    result.sample.angle_rad = pitch_rad_;
  }
  return result;
}

ImuPitchEstimate ImuPitchEstimator::make_estimate(double acc_x, double acc_z,
                                                  double gyro_rate,
                                                  double gyro_accel,
                                                  double yaw_rate,
                                                  TimePoint ts) {
  ImuPitchEstimate result;
  result.sample.t = ts;
  const auto pitch = solve_pitch_with_settings(
      acc_x, acc_z, gyro_rate, gyro_accel, settings_,
      settings_.lever_arm_correction_enabled);
  if (!pitch) {
    reset();
    return result;
  }

  result.sample.angle_rad = *pitch;
  result.sample.gyro_rad_s = gyro_rate;
  result.sample.pitch_accel_rad_s2 = gyro_accel;
  result.sample.yaw_rate_z = yaw_rate;
  result.valid = true;
  return result;
}
