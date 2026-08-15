#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <complex>
#include <limits>
#include <random>
#include <vector>

#include "services/imu/imu_pitch_estimator.h"
#include "services/main/config.h"

namespace {

constexpr double kTwoPi = 2.0 * M_PI;

double deg2rad(double degrees) {
  return degrees * M_PI / 180.0;
}

double rad2deg(double radians) {
  return radians * 180.0 / M_PI;
}

double wrap_pi(double angle) {
  return std::remainder(angle, kTwoPi);
}

double px4_lpf2p_magnitude(double sample_hz, double cutoff_hz,
                           double signal_hz) {
  const double ohm = std::tan(M_PI * cutoff_hz / sample_hz);
  const double c = 1.0 + std::sqrt(2.0) * ohm + ohm * ohm;
  const double b0 = ohm * ohm / c;
  const double b1 = 2.0 * b0;
  const double b2 = b0;
  const double a1 = 2.0 * (ohm * ohm - 1.0) / c;
  const double a2 = (1.0 - std::sqrt(2.0) * ohm + ohm * ohm) / c;
  const double phase = -kTwoPi * signal_hz / sample_hz;
  const std::complex<double> z1 = std::polar(1.0, phase);
  const std::complex<double> z2 = z1 * z1;
  return std::abs((b0 + b1 * z1 + b2 * z2) /
                  (1.0 + a1 * z1 + a2 * z2));
}

double first_order_lpf_magnitude(double sample_hz, double cutoff_hz,
                                 double signal_hz) {
  const double pole = std::exp(-kTwoPi * cutoff_hz / sample_hz);
  const std::complex<double> z1 =
      std::polar(1.0, -kTwoPi * signal_hz / sample_hz);
  return std::abs((1.0 - pole) / (1.0 - pole * z1));
}

std::array<double, 3> imu_accel(double pitch, double gyro_rate = 0.0,
                                double gyro_accel = 0.0,
                                double body_x_accel = 0.0) {
  return {
      body_x_accel - Config::g0 * std::sin(pitch) +
          Config::imu_height_m * gyro_accel,
      0.0,
      -Config::g0 * std::cos(pitch) -
          Config::imu_height_m * gyro_rate * gyro_rate,
  };
}

using TimePoint = ImuPitchEstimator::TimePoint;

TimePoint at_seconds(double seconds) {
  return TimePoint(std::chrono::duration_cast<TimePoint::duration>(
      std::chrono::duration<double>(seconds)));
}

ImuPitchEstimate feed_static(ImuPitchEstimator& estimator, double start_s,
                             double duration_s, double pitch,
                             double gyro_bias = 0.0) {
  ImuPitchEstimate result;
  const int samples =
      std::max(1, static_cast<int>(std::ceil(duration_s * Config::sampling_hz)));
  for (int sample = 0; sample <= samples; ++sample) {
    const double time_s = start_s + static_cast<double>(sample) / Config::sampling_hz;
    result = estimator.push_sample(imu_accel(pitch), {0.0, gyro_bias, 0.0},
                                   at_seconds(time_s));
  }
  return result;
}

struct SineMeasurement {
  double gain{0.0};
  double phase_rad{0.0};
  double delay_s{0.0};
  bool finite{true};
};

template <typename InputFn, typename OutputFn>
SineMeasurement measure_sine(ImuPitchEstimator& estimator, double frequency_hz,
                             double input_amplitude, InputFn input_fn,
                             OutputFn output_fn) {
  constexpr double duration_s = 3.0;
  constexpr double measure_start_s = 2.0;
  double sin_sum = 0.0;
  double cos_sum = 0.0;
  double sin_sq = 0.0;
  double cos_sq = 0.0;
  int count = 0;
  bool finite = true;
  ImuPitchEstimate result;

  const int sample_count = static_cast<int>(duration_s * Config::sampling_hz);
  for (int sample = 0; sample <= sample_count; ++sample) {
    const double time_s = static_cast<double>(sample) / Config::sampling_hz;
    const double phase = kTwoPi * frequency_hz * time_s;
    const auto input = input_fn(input_amplitude * std::sin(phase));
    result = estimator.push_sample(input.first, input.second, at_seconds(time_s));
    if (time_s < measure_start_s || !result.valid) continue;

    const double output = output_fn(result.sample);
    finite = finite && std::isfinite(output);
    const double sine = std::sin(phase);
    const double cosine = std::cos(phase);
    sin_sum += output * sine;
    cos_sum += output * cosine;
    sin_sq += sine * sine;
    cos_sq += cosine * cosine;
    ++count;
  }

  EXPECT_GT(count, 100);
  const double sine_coefficient = sin_sum / sin_sq;
  const double cosine_coefficient = cos_sum / cos_sq;
  const double output_amplitude =
      std::hypot(sine_coefficient, cosine_coefficient);
  const double phase_rad =
      std::atan2(cosine_coefficient, sine_coefficient);
  return SineMeasurement{
      .gain = output_amplitude / input_amplitude,
      .phase_rad = phase_rad,
      .delay_s = -phase_rad / (kTwoPi * frequency_hz),
      .finite = finite,
  };
}

auto pitch_input() {
  return [](double pitch) {
    return std::pair{imu_accel(pitch), std::array<double, 3>{0.0, 0.0, 0.0}};
  };
}

auto gyro_input() {
  return [](double gyro_rate) {
    return std::pair{imu_accel(0.0), std::array<double, 3>{0.0, gyro_rate, 0.0}};
  };
}

}  // namespace

TEST(ImuPitchAlgebraTest, ReconstructsSignedFullCirclePitchAndLeverArmMotion) {
  for (const double pitch_deg : {-179.0, -90.0, -25.0, -10.0, 0.0, 10.0, 25.0,
                                 90.0, 179.0}) {
    for (const double rate : {-3.0, 0.0, 3.0}) {
      for (const double accel : {-20.0, 0.0, 20.0}) {
        const double expected = deg2rad(pitch_deg);
        const auto specific_force = imu_accel(expected, rate, accel);
        const auto solved = imu_pitch_detail::solve_pitch(
            specific_force[0], specific_force[2], rate, accel);
        ASSERT_TRUE(solved.has_value()) << pitch_deg << " " << rate << " " << accel;
        EXPECT_NEAR(wrap_pi(*solved - expected), 0.0, 1e-12);
      }
    }
  }
}

TEST(ImuPitchAlgebraTest, CrossesPlusMinusPiWithoutFolding) {
  for (const double pitch_deg : {179.999, 180.001, -179.999, -180.001, 540.0}) {
    const double expected = deg2rad(pitch_deg);
    const auto specific_force = imu_accel(expected);
    const auto solved =
        imu_pitch_detail::solve_pitch(specific_force[0], specific_force[2], 0.0, 0.0);
    ASSERT_TRUE(solved.has_value());
    EXPECT_GE(*solved, -M_PI);
    EXPECT_LE(*solved, M_PI);
    EXPECT_NEAR(wrap_pi(*solved - expected), 0.0, 1e-12);
  }
}

TEST(ImuPitchAlgebraTest, RejectsNonFiniteAndOutOfRangeSpecificForce) {
  EXPECT_FALSE(imu_pitch_detail::solve_pitch(
                   std::numeric_limits<double>::quiet_NaN(), -Config::g0, 0.0, 0.0)
                   .has_value());
  EXPECT_FALSE(imu_pitch_detail::solve_pitch(0.0, -0.5, 0.0, 0.0).has_value());
  EXPECT_FALSE(imu_pitch_detail::solve_pitch(0.0, -40.0, 0.0, 0.0).has_value());
}

TEST(ImuPitchEstimatorTest, FirstValidSampleStartsPitchAtZero) {
  ImuPitchEstimator estimator;
  const double pitch = deg2rad(7.0);
  const auto result = estimator.push_sample(
      imu_accel(pitch), {0.0, 0.4, 0.2}, at_seconds(1.0));

  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.sample.angle_rad, 0.0);
  EXPECT_NEAR(result.sample.gyro_rad_s, 0.4, 3e-6);
  EXPECT_EQ(result.sample.pitch_accel_rad_s2, 0.0);
  EXPECT_EQ(result.sample.yaw_rate_z, 0.2);
}

TEST(ImuPitchEstimatorTest, ReconstructsSymmetricStaticPitchWithTimestampJitter) {
  for (const double pitch_deg : {-25.0, -10.0, 0.0, 10.0, 25.0}) {
    ImuPitchEstimator estimator;
    ImuPitchEstimate result;
    double previous_time_s = 5.0;
    for (int sample = 0; sample <= static_cast<int>(7.0 * Config::sampling_hz);
         ++sample) {
      const double nominal =
          5.0 + static_cast<double>(sample) / Config::sampling_hz;
      const double jitter = sample == 0 ? 0.0 : 0.00015 * std::sin(sample * 0.7);
      const double time_s = std::max(previous_time_s + 1e-6, nominal + jitter);
      result = estimator.push_sample(imu_accel(deg2rad(pitch_deg)),
                                     {0.0, 0.0, 0.0}, at_seconds(time_s));
      previous_time_s = time_s;
    }
    ASSERT_TRUE(result.valid) << pitch_deg;
    EXPECT_NEAR(rad2deg(wrap_pi(result.sample.angle_rad - deg2rad(pitch_deg))),
                0.0, 1e-4);
  }
}

TEST(ImuPitchEstimatorTest, GyroBackedDynamicPitchTracksBothDirections) {
  ImuPitchEstimator estimator;
  constexpr double frequency_hz = 5.0;
  constexpr double amplitude_rad = 10.0 * M_PI / 180.0;
  double squared_error = 0.0;
  double positive_peak = 0.0;
  double negative_peak = 0.0;
  int count = 0;

  for (int sample = 0; sample <= static_cast<int>(3.0 * Config::sampling_hz);
       ++sample) {
    const double time_s = static_cast<double>(sample) / Config::sampling_hz;
    const double phase = kTwoPi * frequency_hz * time_s;
    const double pitch = amplitude_rad * std::sin(phase);
    const double rate = amplitude_rad * kTwoPi * frequency_hz * std::cos(phase);
    const auto result = estimator.push_sample(
        imu_accel(pitch), {0.0, rate, 0.0}, at_seconds(time_s));
    ASSERT_TRUE(result.valid);
    if (time_s < 1.0) continue;

    const double error = wrap_pi(result.sample.angle_rad - pitch);
    squared_error += error * error;
    positive_peak = std::max(positive_peak, result.sample.angle_rad);
    negative_peak = std::min(negative_peak, result.sample.angle_rad);
    ++count;
  }

  ASSERT_GT(count, 100);
  EXPECT_LT(rad2deg(std::sqrt(squared_error / count)), 2.0);
  EXPECT_GT(rad2deg(positive_peak), 8.0);
  EXPECT_LT(rad2deg(negative_peak), -8.0);
}

TEST(ImuPitchEstimatorTest, StartupHistoryDecaysWithoutLearnedOffset) {
  ImuPitchEstimator first;
  ImuPitchEstimator second;
  (void)feed_static(first, 0.0, 0.05, deg2rad(-20.0), 2.0);
  (void)feed_static(second, 0.0, 0.05, deg2rad(20.0), -2.0);

  const auto first_result = feed_static(first, 0.06, 4.0, deg2rad(7.0));
  const auto second_result = feed_static(second, 0.06, 4.0, deg2rad(7.0));
  ASSERT_TRUE(first_result.valid);
  ASSERT_TRUE(second_result.valid);
  EXPECT_NEAR(first_result.sample.angle_rad, second_result.sample.angle_rad, 1e-7);
  EXPECT_NEAR(rad2deg(first_result.sample.angle_rad), 7.0, 1e-4);
}

TEST(ImuPitchEstimatorTest, ConstantGyroBiasCannotAccumulatePitchDrift) {
  ImuPitchEstimator estimator;
  constexpr double pitch = 5.0 * M_PI / 180.0;
  ImuPitchEstimate halfway;
  ImuPitchEstimate result;
  const int samples = static_cast<int>(120.0 * Config::sampling_hz);
  for (int sample = 0; sample <= samples; ++sample) {
    result = estimator.push_sample(
        imu_accel(pitch), {0.0, 0.01, 0.0},
        at_seconds(static_cast<double>(sample) / Config::sampling_hz));
    if (sample == samples / 2) halfway = result;
  }
  ASSERT_TRUE(halfway.valid);
  ASSERT_TRUE(result.valid);
  EXPECT_LT(std::abs(rad2deg(result.sample.angle_rad) - 5.0), 0.20);
  EXPECT_NEAR(result.sample.angle_rad, halfway.sample.angle_rad, 1e-8);
  EXPECT_NEAR(result.sample.gyro_rad_s, 0.01, 1e-5);
}

TEST(ImuPitchEstimatorTest, TranslationHasNoPersistentEstimatorOffset) {
  ImuPitchEstimator estimator;
  const auto translated = estimator.push_sample(
      imu_accel(0.0, 0.0, 0.0, 2.0), {0.0, 0.0, 0.0}, at_seconds(0.0));
  ASSERT_TRUE(translated.valid);
  EXPECT_DOUBLE_EQ(translated.sample.angle_rad, 0.0);

  const auto recovered = feed_static(estimator, 0.01, 4.0, 0.0);
  ASSERT_TRUE(recovered.valid);
  EXPECT_NEAR(recovered.sample.angle_rad, 0.0, 1e-5);
}

TEST(ImuPitchEstimatorTest, NonFiniteSampleResetsAndNextValidSampleStartsAtZero) {
  ImuPitchEstimator estimator;
  ASSERT_TRUE(estimator.push_sample(imu_accel(deg2rad(3.0)), {0.0, 0.0, 0.0},
                                    at_seconds(0.0))
                  .valid);
  const auto invalid = estimator.push_sample(
      {std::numeric_limits<double>::infinity(), 0.0, -Config::g0},
      {0.0, 0.0, 0.0}, at_seconds(0.002));
  EXPECT_FALSE(invalid.valid);

  const auto recovered = estimator.push_sample(
      imu_accel(deg2rad(-4.0)), {0.0, 0.0, 0.0}, at_seconds(0.004));
  ASSERT_TRUE(recovered.valid);
  EXPECT_DOUBLE_EQ(recovered.sample.angle_rad, 0.0);
  EXPECT_EQ(recovered.sample.pitch_accel_rad_s2, 0.0);
}

TEST(ImuPitchEstimatorTest, DuplicateAndBackwardTimestampsInvalidateAndReset) {
  for (const double bad_time_s : {1.0, 0.5}) {
    ImuPitchEstimator estimator;
    ASSERT_TRUE(estimator.push_sample(imu_accel(0.0), {0.0, 0.0, 0.0},
                                      at_seconds(1.0))
                    .valid);
    EXPECT_FALSE(estimator.push_sample(imu_accel(0.0), {0.0, 0.0, 0.0},
                                       at_seconds(bad_time_s))
                     .valid);
    const auto recovered = estimator.push_sample(
        imu_accel(deg2rad(2.0)), {0.0, 0.0, 0.0}, at_seconds(1.01));
    ASSERT_TRUE(recovered.valid);
    EXPECT_DOUBLE_EQ(recovered.sample.angle_rad, 0.0);
  }
}

TEST(ImuPitchEstimatorTest, LargeGapReseedsCurrentSampleWithZeroDerivative) {
  ImuPitchEstimator estimator;
  ASSERT_TRUE(estimator.push_sample(imu_accel(0.0), {0.0, 1.0, 0.0},
                                    at_seconds(0.0))
                  .valid);
  const auto result = estimator.push_sample(
      imu_accel(deg2rad(-6.0)), {0.0, -2.0, 0.0}, at_seconds(0.010));
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.sample.angle_rad, 0.0);
  EXPECT_NEAR(result.sample.gyro_rad_s, -2.0, 1e-5);
  EXPECT_EQ(result.sample.pitch_accel_rad_s2, 0.0);
}

TEST(ImuPitchEstimatorTest, TimestampJitterAndOneDroppedSampleRemainValid) {
  ImuPitchEstimator estimator;
  uint64_t timestamp_us = 1;
  for (int sample = 0; sample < 1000; ++sample) {
    const uint64_t increment_us =
        sample == 500 ? 2401 : static_cast<uint64_t>(1200 + (sample % 3));
    timestamp_us += increment_us;
    const auto result = estimator.push_sample(
        imu_accel(deg2rad(1.0)), {0.0, 0.0, 0.0},
        TimePoint(std::chrono::microseconds(timestamp_us)));
    EXPECT_TRUE(result.valid) << sample;
  }
}

TEST(ImuPitchEstimatorTest, RejectsBroadMagnitudeLimitsAndRecovers) {
  ImuPitchEstimator estimator;
  EXPECT_FALSE(estimator.push_sample({0.0, 0.0, -0.5}, {0.0, 0.0, 0.0},
                                     at_seconds(0.0))
                   .valid);
  EXPECT_TRUE(estimator.push_sample(imu_accel(0.0), {0.0, 0.0, 0.0},
                                    at_seconds(0.01))
                  .valid);
  bool rejected_high_force = false;
  double last_time_s = 0.0112;
  for (int sample = 0; sample < 60; ++sample) {
    last_time_s = 0.0112 + static_cast<double>(sample) / Config::sampling_hz;
    const auto result = estimator.push_sample(
        {0.0, 0.0, -40.0}, {0.0, 0.0, 0.0},
        at_seconds(last_time_s));
    if (!result.valid) {
      rejected_high_force = true;
      break;
    }
  }
  EXPECT_TRUE(rejected_high_force);
  EXPECT_TRUE(estimator.push_sample(imu_accel(0.0), {0.0, 0.0, 0.0},
                                    at_seconds(last_time_s + 0.01))
                  .valid);
}

TEST(ImuPitchEstimatorTest, ProductionDefaultLeavesLeverArmCorrectionDisabled) {
  const auto settings = ImuPitchEstimator::Settings::production();
  EXPECT_DOUBLE_EQ(settings.accel_lpf_hz, 15.0);
  EXPECT_DOUBLE_EQ(settings.gyro_lpf_hz, 30.0);
  EXPECT_DOUBLE_EQ(settings.gyro_derivative_lpf_hz, 10.0);
  EXPECT_DOUBLE_EQ(settings.attitude_correction_hz, 0.5);
  EXPECT_NEAR(rad2deg(settings.gravity_innovation_limit_rad), 2.5, 1e-12);
  ASSERT_FALSE(settings.lever_arm_correction_enabled);
  EXPECT_EQ(settings.gyro_notch_hz, 0.0);
  EXPECT_EQ(settings.gyro_notch_bandwidth_hz, 0.0);
  ImuPitchEstimator estimator;
  constexpr double angular_accel = 20.0;
  const auto specific_force = imu_accel(0.0, 0.0, angular_accel);
  ImuPitchEstimate result;
  for (int sample = 0; sample <= static_cast<int>(4.0 * Config::sampling_hz);
       ++sample) {
    result = estimator.push_sample(
        specific_force, {0.0, 0.0, 0.0},
        at_seconds(static_cast<double>(sample) / Config::sampling_hz));
  }

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.sample.angle_rad,
              std::atan2(-specific_force[0], -specific_force[2]), 1e-5);
  EXPECT_GT(std::abs(result.sample.angle_rad), deg2rad(5.0));
}

TEST(ImuPitchEstimatorTest, EnabledLeverArmCorrectionRemovesSteadyRateTerm) {
  auto settings = ImuPitchEstimator::Settings::production();
  settings.lever_arm_correction_enabled = true;
  ImuPitchEstimator estimator(settings);
  constexpr double pitch = 0.0;
  constexpr double rate = 3.0;
  const auto result = estimator.push_sample(
      imu_accel(pitch, rate, 0.0), {0.0, rate, 0.0}, at_seconds(0.0));

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.sample.angle_rad, pitch, 1e-7);
}

TEST(ImuPitchEstimatorTest, GravityPathMatchesCompositeResponseAndDelay) {
  for (const double frequency_hz : {5.0, 30.0, 100.0}) {
    ImuPitchEstimator estimator;
    const auto measured = measure_sine(
        estimator, frequency_hz, deg2rad(1.0), pitch_input(),
        [](const ImuSample& sample) { return sample.angle_rad; });
    const double expected =
        px4_lpf2p_magnitude(Config::sampling_hz, Config::imu_accel_lpf_hz,
                            frequency_hz) *
        first_order_lpf_magnitude(Config::sampling_hz,
                                  Config::imu_attitude_correction_hz,
                                  frequency_hz);
    EXPECT_TRUE(measured.finite);
    EXPECT_NEAR(measured.gain, expected, 0.05 * expected) << frequency_hz;
    if (frequency_hz == 5.0) {
      EXPECT_GT(measured.delay_s, 0.0);
      EXPECT_LT(measured.delay_s, 0.065);
    }
  }
}

TEST(ImuPitchEstimatorTest, GyroFilterMatchesPx4ResponseAndDelay) {
  for (const double frequency_hz : {5.0, 60.0, 120.0}) {
    ImuPitchEstimator estimator;
    const auto measured = measure_sine(
        estimator, frequency_hz, 1.0, gyro_input(),
        [](const ImuSample& sample) { return sample.gyro_rad_s; });
    const double expected = px4_lpf2p_magnitude(
        Config::sampling_hz, Config::imu_gyro_lpf_hz, frequency_hz);
    EXPECT_TRUE(measured.finite);
    EXPECT_NEAR(measured.gain, expected, 0.05 * expected) << frequency_hz;
    if (frequency_hz == 5.0) {
      EXPECT_GT(measured.delay_s, 0.0);
      EXPECT_LT(measured.delay_s, 0.010);
    }
  }
}

TEST(ImuPitchEstimatorTest, DerivativePathIsFiniteAndRejectsHighFrequency) {
  ImuPitchEstimator low_frequency;
  const auto low = measure_sine(
      low_frequency, 5.0, 1.0, gyro_input(),
      [](const ImuSample& sample) { return sample.pitch_accel_rad_s2; });
  ImuPitchEstimator high_frequency;
  const auto high = measure_sine(
      high_frequency, 100.0, 1.0, gyro_input(),
      [](const ImuSample& sample) { return sample.pitch_accel_rad_s2; });

  const double normalized_low = low.gain / (kTwoPi * 5.0);
  const double normalized_high = high.gain / (kTwoPi * 100.0);
  EXPECT_TRUE(low.finite);
  EXPECT_TRUE(high.finite);
  EXPECT_GT(normalized_low, 0.8);
  EXPECT_LT(normalized_high, 0.25 * normalized_low);
}

TEST(ImuPitchEstimatorTest, ConfiguredNotchSuppressesItsCenterFrequency) {
  auto settings = ImuPitchEstimator::Settings::production();
  settings.gyro_notch_hz = 80.0;
  settings.gyro_notch_bandwidth_hz = 20.0;
  ImuPitchEstimator notched(settings);
  ImuPitchEstimator unnotched;

  const auto with_notch = measure_sine(
      notched, 80.0, 1.0, gyro_input(),
      [](const ImuSample& sample) { return sample.gyro_rad_s; });
  const auto without_notch = measure_sine(
      unnotched, 80.0, 1.0, gyro_input(),
      [](const ImuSample& sample) { return sample.gyro_rad_s; });

  EXPECT_TRUE(with_notch.finite);
  EXPECT_TRUE(without_notch.finite);
  EXPECT_LT(with_notch.gain, 0.1 * without_notch.gain);
}

TEST(ImuPitchEstimatorTest, AccelAndGyroVibrationAreAttenuatedWithoutMeanShift) {
  for (const double frequency_hz : {40.0, 100.0}) {
    ImuPitchEstimator estimator;
    const auto pitch = measure_sine(
        estimator, frequency_hz, deg2rad(5.0), pitch_input(),
        [](const ImuSample& sample) { return sample.angle_rad; });
    EXPECT_LT(pitch.gain, frequency_hz == 40.0 ? 0.18 : 0.04);
  }

  ImuPitchEstimator gyro_estimator;
  const auto gyro = measure_sine(
      gyro_estimator, 100.0, 1.0, gyro_input(),
      [](const ImuSample& sample) { return sample.gyro_rad_s; });
  EXPECT_LT(gyro.gain, 0.12);
}

TEST(ImuPitchEstimatorTest, LowPassReducesStaticAccelNoiseWithoutBias) {
  ImuPitchEstimator estimator;
  std::mt19937 rng(77);
  std::normal_distribution<double> noise(0.0, 0.10);
  constexpr double pitch = 3.0 * M_PI / 180.0;
  double raw_sq = 0.0;
  double filtered_sq = 0.0;
  double filtered_sum = 0.0;
  int count = 0;

  for (int sample = 0; sample < 4000; ++sample) {
    auto acc = imu_accel(pitch);
    acc[0] += noise(rng);
    acc[2] += noise(rng);
    const auto result = estimator.push_sample(
        acc, {0.0, 0.0, 0.0},
        at_seconds(static_cast<double>(sample) / Config::sampling_hz));
    if (sample > static_cast<int>(1.0 * Config::sampling_hz) && result.valid) {
      const double raw_pitch = std::atan2(-acc[0], -acc[2]);
      raw_sq += std::pow(wrap_pi(raw_pitch - pitch), 2);
      filtered_sq += std::pow(wrap_pi(result.sample.angle_rad - pitch), 2);
      filtered_sum += result.sample.angle_rad;
      ++count;
    }
  }

  ASSERT_GT(count, 100);
  EXPECT_LT(std::sqrt(filtered_sq / count), std::sqrt(raw_sq / count));
  EXPECT_LT(rad2deg(std::sqrt(filtered_sq / count)), 1.0);
  EXPECT_NEAR(rad2deg(filtered_sum / count), 3.0, 0.1);
}
