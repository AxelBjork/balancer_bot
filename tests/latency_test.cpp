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

struct BiquadCoefficients {
  double b0{1.0};
  double b1{0.0};
  double b2{0.0};
  double a1{0.0};
  double a2{0.0};
};

struct BiquadResponse {
  std::complex<double> transfer{};
  double group_delay_ms{0.0};
};

// These coefficient builders intentionally follow the formulas in the
// production PX4 filter implementations, including their float coefficient
// arithmetic. The response calculation below then operates on those exact
// coefficients rather than on a continuous-time approximation.
BiquadCoefficients notch_coefficients(double sample_hz, double notch_hz,
                                      double bandwidth_hz) {
  const float sample_freq = static_cast<float>(sample_hz);
  const float notch_freq = static_cast<float>(notch_hz);
  const float bandwidth = static_cast<float>(bandwidth_hz);
  const float alpha =
      std::tan(static_cast<float>(M_PI) * bandwidth / sample_freq);
  const float beta = -std::cos(2.0f * static_cast<float>(M_PI) * notch_freq /
                               sample_freq);
  const float a0_inv = 1.0f / (alpha + 1.0f);
  const float b0 = a0_inv;
  const float b1 = 2.0f * beta * a0_inv;
  const float a2 = (1.0f - alpha) * a0_inv;
  return {
      .b0 = b0,
      .b1 = b1,
      .b2 = b0,
      .a1 = b1,
      .a2 = a2,
  };
}

BiquadCoefficients low_pass_2p_coefficients(double sample_hz,
                                            double cutoff_hz) {
  const float sample_freq = static_cast<float>(sample_hz);
  const float cutoff = static_cast<float>(cutoff_hz);
  const float ohm = std::tan(static_cast<float>(M_PI) * cutoff / sample_freq);
  const float c = 1.0f + 2.0f * std::cos(static_cast<float>(M_PI) / 4.0f) * ohm +
                  ohm * ohm;
  const float b0 = ohm * ohm / c;
  return {
      .b0 = b0,
      .b1 = 2.0f * b0,
      .b2 = b0,
      .a1 = 2.0f * (ohm * ohm - 1.0f) / c,
      .a2 = (1.0f - 2.0f * std::cos(static_cast<float>(M_PI) / 4.0f) * ohm +
             ohm * ohm) /
            c,
  };
}

BiquadResponse evaluate_biquad(const BiquadCoefficients& coefficients,
                               double sample_hz, double frequency_hz) {
  const double omega = kTwoPi * frequency_hz / sample_hz;
  const std::complex<double> z = std::polar(1.0, -omega);
  const std::complex<double> dz_domega =
      std::complex<double>(0.0, -1.0) * z;
  const std::complex<double> numerator =
      coefficients.b0 + coefficients.b1 * z + coefficients.b2 * z * z;
  const std::complex<double> denominator =
      1.0 + coefficients.a1 * z + coefficients.a2 * z * z;
  const std::complex<double> numerator_z =
      coefficients.b1 + 2.0 * coefficients.b2 * z;
  const std::complex<double> denominator_z =
      coefficients.a1 + 2.0 * coefficients.a2 * z;
  const std::complex<double> transfer = numerator / denominator;
  const std::complex<double> d_transfer_domega =
      (numerator_z * dz_domega * denominator -
       numerator * denominator_z * dz_domega) /
      (denominator * denominator);
  const double group_delay_ms =
      -std::imag(d_transfer_domega / transfer) * 1000.0 / sample_hz;
  return {.transfer = transfer, .group_delay_ms = group_delay_ms};
}

BiquadResponse evaluate_current_rate_chain(double frequency_hz) {
  const auto notch = evaluate_biquad(
      notch_coefficients(Config::sampling_hz, 32.0, 10.0),
      Config::sampling_hz, frequency_hz);
  const auto lpf = evaluate_biquad(
      low_pass_2p_coefficients(Config::sampling_hz, 30.0),
      Config::sampling_hz, frequency_hz);
  return {
      .transfer = notch.transfer * lpf.transfer,
      .group_delay_ms = notch.group_delay_ms + lpf.group_delay_ms,
  };
}

BiquadResponse evaluate_candidate_rate_chain(double frequency_hz) {
  return evaluate_biquad(
      notch_coefficients(Config::sampling_hz, 33.4, 8.0),
      Config::sampling_hz, frequency_hz);
}

double gain_db(const BiquadResponse& response) {
  return 20.0 * std::log10(std::abs(response.transfer));
}

double phase_deg(const BiquadResponse& response) {
  return std::arg(response.transfer) * 180.0 / M_PI;
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
  EXPECT_DOUBLE_EQ(settings.gyro_software_lpf_hz, 0.0);
  EXPECT_DOUBLE_EQ(settings.gyro_derivative_lpf_hz, 10.0);
  EXPECT_DOUBLE_EQ(settings.attitude_correction_hz, 0.5);
  EXPECT_NEAR(rad2deg(settings.gravity_innovation_limit_rad), 2.5, 1e-12);
  ASSERT_FALSE(settings.lever_arm_correction_enabled);
  EXPECT_DOUBLE_EQ(settings.gyro_notch_hz, 33.4);
  EXPECT_DOUBLE_EQ(settings.gyro_notch_bandwidth_hz, 8.0);
  EXPECT_DOUBLE_EQ(Config::imu_gyro_lpf1_bandwidth_hz, 140.0);
  EXPECT_EQ(Config::imu_gyro_lpf1_ftype, 0b010);
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

TEST(ImuPitchEstimatorTest, CurrentSoftwareReferenceUses32HzNotchAnd30HzLowPass) {
  const auto settings =
      ImuPitchEstimator::Settings::legacy_simulation_reference();
  EXPECT_DOUBLE_EQ(settings.gyro_notch_hz, 32.0);
  EXPECT_DOUBLE_EQ(settings.gyro_notch_bandwidth_hz, 10.0);
  EXPECT_DOUBLE_EQ(settings.gyro_software_lpf_hz, 30.0);
}

TEST(ImuPitchEstimatorTest, ProductionNotchSuppressesLockedHardwareBand) {
  const auto settings = ImuPitchEstimator::Settings::production();
  ImuPitchEstimator notched(settings);

  auto unnotched_settings = settings;
  unnotched_settings.gyro_notch_hz = 0.0;
  unnotched_settings.gyro_notch_bandwidth_hz = 0.0;
  ImuPitchEstimator unnotched(unnotched_settings);

  const auto with_notch = measure_sine(
      notched, settings.gyro_notch_hz, 1.0, gyro_input(),
      [](const ImuSample& sample) { return sample.gyro_rad_s; });
  const auto without_notch = measure_sine(
      unnotched, settings.gyro_notch_hz, 1.0, gyro_input(),
      [](const ImuSample& sample) { return sample.gyro_rad_s; });

  EXPECT_TRUE(with_notch.finite);
  EXPECT_TRUE(without_notch.finite);
  EXPECT_LT(with_notch.gain, 0.1 * without_notch.gain);
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

TEST(ImuPitchEstimatorTest, GyroRatePathHasNoGenericSoftwareLowPass) {
  auto unnotched_settings = ImuPitchEstimator::Settings::production();
  unnotched_settings.gyro_notch_hz = 0.0;
  unnotched_settings.gyro_notch_bandwidth_hz = 0.0;

  for (const double frequency_hz : {5.0, 100.0}) {
    ImuPitchEstimator notched;
    ImuPitchEstimator unnotched(unnotched_settings);
    const auto measured = measure_sine(
        notched, frequency_hz, 1.0, gyro_input(),
        [](const ImuSample& sample) { return sample.gyro_rad_s; });
    const auto bypass = measure_sine(
        unnotched, frequency_hz, 1.0, gyro_input(),
        [](const ImuSample& sample) { return sample.gyro_rad_s; });
    EXPECT_TRUE(measured.finite);
    EXPECT_TRUE(bypass.finite);
    EXPECT_GT(measured.gain / bypass.gain, frequency_hz == 5.0 ? 0.97 : 0.80)
        << frequency_hz;
    if (frequency_hz == 5.0) {
      EXPECT_LT(std::abs(measured.delay_s), 0.002);
    }
  }
}

TEST(ImuPitchEstimatorTest, RateFilterAnalyticalResponseUsesImplemented833HzCoefficients) {
  struct ExpectedResponse {
    double frequency_hz;
    double current_gain_db;
    double current_phase_deg;
    double current_group_delay_ms;
    double candidate_gain_db;
    double candidate_phase_deg;
    double candidate_group_delay_ms;
  };
  constexpr std::array<ExpectedResponse, 3> expected{{
      {2.0, -0.001818, -6.516073, 9.084050, -0.000910, -0.829280, 1.159862},
      {5.0, -0.014320, -16.450239, 9.350268, -0.005903, -2.112103, 1.225799},
      {10.0, -0.103594, -34.032329, 10.285622, -0.027142, -4.527168, 1.497761},
  }};

  for (const auto& item : expected) {
    const auto current = evaluate_current_rate_chain(item.frequency_hz);
    const auto candidate = evaluate_candidate_rate_chain(item.frequency_hz);
    EXPECT_NEAR(gain_db(current), item.current_gain_db, 2e-5)
        << item.frequency_hz;
    EXPECT_NEAR(phase_deg(current), item.current_phase_deg, 2e-5)
        << item.frequency_hz;
    EXPECT_NEAR(current.group_delay_ms, item.current_group_delay_ms, 2e-5)
        << item.frequency_hz;
    EXPECT_NEAR(gain_db(candidate), item.candidate_gain_db, 2e-5)
        << item.frequency_hz;
    EXPECT_NEAR(phase_deg(candidate), item.candidate_phase_deg, 2e-5)
        << item.frequency_hz;
    EXPECT_NEAR(candidate.group_delay_ms, item.candidate_group_delay_ms, 2e-5)
        << item.frequency_hz;

    auto current_settings = ImuPitchEstimator::Settings::production();
    current_settings.gyro_notch_hz = 32.0;
    current_settings.gyro_notch_bandwidth_hz = 10.0;
    current_settings.gyro_software_lpf_hz = 30.0;
    ImuPitchEstimator current_estimator(current_settings);
    ImuPitchEstimator candidate_estimator;
    const auto measured_current = measure_sine(
        current_estimator, item.frequency_hz, 1.0, gyro_input(),
        [](const ImuSample& sample) { return sample.gyro_rad_s; });
    const auto measured_candidate = measure_sine(
        candidate_estimator, item.frequency_hz, 1.0, gyro_input(),
        [](const ImuSample& sample) { return sample.gyro_rad_s; });
    EXPECT_TRUE(measured_current.finite);
    EXPECT_TRUE(measured_candidate.finite);
    EXPECT_NEAR(measured_current.gain, std::abs(current.transfer), 2e-3)
        << item.frequency_hz;
    EXPECT_NEAR(measured_current.phase_rad, std::arg(current.transfer), 2e-3)
        << item.frequency_hz;
    EXPECT_NEAR(measured_candidate.gain, std::abs(candidate.transfer), 2e-3)
        << item.frequency_hz;
    EXPECT_NEAR(measured_candidate.phase_rad, std::arg(candidate.transfer), 2e-3)
        << item.frequency_hz;
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
  EXPECT_GT(gyro.gain, 0.80);
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
