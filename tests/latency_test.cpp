#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>

#include "services/main/config.h"
#include "services/imu/pitch_lpf.h"
#include "services/motor/stepper.h"

// --- Helpers ---
static inline double deg2rad(double d) {
  return d * M_PI / 180.0;
}
static inline double rad2deg(double r) {
  return r * 180.0 / M_PI;
}

// Build accel consistent with acc_pitch() = atan2(-ax, -az).
// For a desired pitch angle, mounted gravity is ax=-sin(pitch), az=-cos(pitch).
static inline std::array<double, 3> accel_for_pitch_g(double pitch_rad) {
  const double s = std::sin(pitch_rad), c = std::cos(pitch_rad);
  return {-s * Config::g0, 0.0, -c * Config::g0};
}

static inline double raw_pitch_from_acc_rad(const std::array<double, 3>& acc) {
  return std::atan2(-acc[0], -acc[2]);
}

static inline double wrap_pi(double x) {
  while (x > M_PI) x -= 2 * M_PI;
  while (x < -M_PI) x += 2 * M_PI;
  return x;
}

/*
TEST(DataPathSanity, StepperStepN_NoDirChange_IsFast) {
  Stepper step(0, Stepper::Pins{5u, 6u, 13u});

  using clock = std::chrono::steady_clock;

  constexpr unsigned period_us = 50u;
  constexpr int iterations = 200;
  const long expected_us =
      static_cast<long>(iterations) * period_us; // ~10,000 us

  const auto t0 = clock::now();
  for (int i = 0; i < iterations; ++i) {
    step.stepN(1u, period_us, true); // no dir flip => no DIR setup delay
  }
  const auto us =
      std::chrono::duration_cast<std::chrono::microseconds>(clock::now() - t0)
          .count();

  // Accept reasonable runtime overhead, but make sure we’re nowhere near the “2
  // ms per call” disaster. If we were wrongly sleeping 2 ms per call, we'd see
  // ~400,000 us here.
  const long max_reasonable_us = expected_us + 25'000; // expected + 25 ms slack
  EXPECT_LT(us, max_reasonable_us)
      << "Cumulative latency too high for fast path (no dir flips): " << us
      << " us; expected ~" << expected_us << " us";
}
*/

/*
TEST(DataPathSanity, StepperStepN_DirFlip_PaysSetupDelayOnce) {
  Stepper::Pins pins{5u, 6u, 13u};
  Stepper step(0, pins);

  using clock = std::chrono::steady_clock;

  // First call (forward): fast
  const auto t0 = clock::now();
  step.stepN(1u, 100, true);
  const auto t1 = clock::now();
  const auto first_us =
      std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

  // Second call (reverse): should incur DIR setup delay (~2000 µs) once
  const auto t2 = clock::now();
  step.stepN(1u, 100, false);
  const auto t3 = clock::now();
  const auto second_us =
      std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();

  // Fast path budget (pulse only)
  EXPECT_LT(first_us, 300) << "First stepN too slow: " << first_us << " us";

  // Expect ≈ 2 ms (+ a little pulse time and scheduler jitter).
  constexpr int kDirSetupUs = 2000;
  EXPECT_GE(second_us, kDirSetupUs - 300)
      << "Dir-flip did not pay expected setup delay.";
  EXPECT_LE(second_us, kDirSetupUs + 600)
      << "Dir-flip stepN too slow: " << second_us << " us";
}
*/

/*
TEST(DataPathSanity, StepperStepN_ManyDirFlips_CumulativeMatchesSetupDelay) {
  Stepper step(0, Stepper::Pins{5u, 6u, 13u});

  using clock = std::chrono::steady_clock;
  const auto t0 = clock::now();

  const int flips = 200;
  for (int i = 0; i < flips; ++i) {
    step.stepN(1u, 50u, (i & 1) == 0);
  }

  const auto us =
      std::chrono::duration_cast<std::chrono::microseconds>(clock::now() - t0)
          .count();

  // Expect around flips * 2000 µs +/- some margin
  const long expected = flips * 2000L;
  const long tol = static_cast<long>(expected * 0.1); // 10% tolerance
  EXPECT_NEAR(us, expected, tol)
      << "Cumulative latency differs from expected DIR setup total.";
}
*/

// ------------------------------------------------------------
// 2) PitchComplementaryFilter tracks large step (0, to 20°).
//    A physical attitude change has a matching gyro signal. An accel-only
//    discontinuity represents linear specific force and must not tilt the
//    estimate.
// ------------------------------------------------------------
TEST(DataPathSanity, ComplementaryFilterTracksRotationAndRejectsAccelOnlyJump) {
  using clock = std::chrono::steady_clock;

  // ---- Required budgets for this project (tune to your plant)

  // ---- Test setup
  const double fs_hz = Config::sampling_hz;  // e.g., 833 Hz
  const auto tick = std::chrono::nanoseconds{std::llround(1e9 / fs_hz)};

  const double target_deg = 20.0;  // keep within reliable range
  const double target_rad = deg2rad(target_deg);

  auto accel_for_pitch_g = [](double pitch_rad) {
    const double s = std::sin(pitch_rad), c = std::cos(pitch_rad);
    return std::array<double, 3>{-s * Config::g0, 0.0, -c * Config::g0};
  };

  PitchComplementaryFilter filt;
  auto now = clock::now();
  auto push = [&](double pitch_true, double gyro_y_rps) -> ImuSample {
    now += tick;
    filt.push_sample(accel_for_pitch_g(pitch_true), {0.0, gyro_y_rps, 0.0}, now);
    return filt.read_latest();
  };

  // Prime
  for (int i = 0; i < 5; ++i) (void)push(0.0, 0.0);

  for (int i = 0; i < static_cast<int>(0.25 * fs_hz); ++i) {
    (void)push(target_rad, 0.0);
  }
  EXPECT_LT(std::abs(filt.read_latest().angle_rad), deg2rad(2.0));

  filt.reset();
  for (int i = 0; i < 5; ++i) (void)push(0.0, 0.0);
  constexpr double rotation_s = 0.5;
  const double rate_rad_s = target_rad / rotation_s;
  const int rotation_samples = static_cast<int>(rotation_s * fs_hz);
  for (int i = 1; i <= rotation_samples; ++i) {
    const double physical_pitch = target_rad * static_cast<double>(i) / rotation_samples;
    (void)push(physical_pitch, rate_rad_s);
  }
  for (int i = 0; i < static_cast<int>(0.1 * fs_hz); ++i) {
    (void)push(target_rad, 0.0);
  }
  EXPECT_NEAR(filt.read_latest().angle_rad, target_rad, deg2rad(1.0));
}

TEST(DataPathSanity, ComplementaryFilterCorrectsLargeGravityErrorGradually) {
  using clock = std::chrono::steady_clock;

  constexpr double fs_hz = Config::sampling_hz;
  const auto tick = std::chrono::nanoseconds{std::llround(1e9 / fs_hz)};
  auto now = clock::now();
  PitchComplementaryFilter filt;

  now += tick;
  filt.push_sample(accel_for_pitch_g(deg2rad(40.0)), {0.0, 0.0, 0.0}, now);
  for (int i = 0; i < static_cast<int>(0.4 * fs_hz); ++i) {
    now += tick;
    filt.push_sample(accel_for_pitch_g(0.0), {0.0, 0.0, 0.0}, now);
  }
  const double after_short_correction_deg = rad2deg(filt.read_latest().angle_rad);
  EXPECT_GT(after_short_correction_deg, 35.0);
  EXPECT_LT(after_short_correction_deg, 40.0);

  // A persistent, low-rate gravity disagreement is an estimator recovery,
  // not ordinary dynamic accelerometer feedback. It must converge smoothly
  // on a hardware-useful time scale instead of retaining a false lean for
  // tens of seconds.
  for (int i = 0; i < static_cast<int>(5.0 * fs_hz); ++i) {
    now += tick;
    filt.push_sample(accel_for_pitch_g(0.0), {0.0, 0.0, 0.0}, now);
  }
  EXPECT_NEAR(rad2deg(filt.read_latest().angle_rad), 0.0, 1.0);
}

TEST(DataPathSanity, ComplementaryFilterReacquiresMeanGravityThroughDynamicAccelPitch) {
  using clock = std::chrono::steady_clock;

  constexpr double fs_hz = Config::sampling_hz;
  const auto tick = std::chrono::nanoseconds{std::llround(1e9 / fs_hz)};
  auto now = clock::now();
  PitchComplementaryFilter filt;

  now += tick;
  filt.push_sample(accel_for_pitch_g(deg2rad(15.0)), {0.0, 0.0, 0.0}, now);

  // Approximate the large raw accelerometer-pitch variation seen while the
  // wheels are active. Its mean still points upright, so the persistent
  // gravity reference should recover the estimator without following every
  // apparent tilt cycle.
  constexpr double apparent_pitch_frequency_hz = 6.0;
  constexpr double duration_s = 4.0;
  for (int i = 1; i <= static_cast<int>(duration_s * fs_hz); ++i) {
    const double t_s = static_cast<double>(i) / fs_hz;
    const double apparent_pitch_rad =
        deg2rad(15.0) * std::sin(2.0 * M_PI * apparent_pitch_frequency_hz * t_s);
    now += tick;
    filt.push_sample(accel_for_pitch_g(apparent_pitch_rad), {0.0, 0.0, 0.0}, now);
  }

  EXPECT_NEAR(rad2deg(filt.read_latest().angle_rad), 0.0, 2.0);
}

TEST(DataPathSanity, ComplementaryFilterTracksRepeatedRotationsWithoutDeadReckoning) {
  using clock = std::chrono::steady_clock;

  constexpr double fs_hz = Config::sampling_hz;
  const auto tick = std::chrono::nanoseconds{std::llround(1e9 / fs_hz)};
  auto now = clock::now();
  PitchComplementaryFilter filt;

  now += tick;
  filt.push_sample(accel_for_pitch_g(0.0), {0.0, 0.0, 0.0}, now);

  constexpr double rotation_count = 4.0;
  constexpr double rotation_duration_s = 4.0;
  const double rate_rad_s = rotation_count * 2.0 * M_PI / rotation_duration_s;
  const int rotation_samples = static_cast<int>(rotation_duration_s * fs_hz);
  double max_wrapped_error_deg = 0.0;
  for (int i = 1; i <= rotation_samples; ++i) {
    const double physical_pitch = rate_rad_s * static_cast<double>(i) / fs_hz;
    now += tick;
    filt.push_sample(accel_for_pitch_g(physical_pitch), {0.0, rate_rad_s, 0.0}, now);
    const double estimated = filt.read_latest().angle_rad;
    EXPECT_GE(estimated, -M_PI);
    EXPECT_LE(estimated, M_PI);
    max_wrapped_error_deg =
        std::max(max_wrapped_error_deg, std::abs(rad2deg(wrap_pi(estimated - physical_pitch))));
  }
  EXPECT_LT(max_wrapped_error_deg, 8.0);

  for (int i = 0; i < static_cast<int>(3.0 * fs_hz); ++i) {
    now += tick;
    filt.push_sample(accel_for_pitch_g(0.0), {0.0, 0.0, 0.0}, now);
  }
  EXPECT_NEAR(rad2deg(filt.read_latest().angle_rad), 0.0, 3.0);
}

TEST(DataPathSanity, ComplementaryFilterUsesSignedGravityAcrossFullPitchRange) {
  using clock = std::chrono::steady_clock;
  const auto tick = std::chrono::nanoseconds{std::llround(1e9 / Config::sampling_hz)};

  for (const double pitch_deg : {0.0, 90.0, -90.0, 135.0, -135.0, 180.0}) {
    PitchComplementaryFilter filt;
    const auto now = clock::now() + tick;
    filt.push_sample(accel_for_pitch_g(deg2rad(pitch_deg)), {0.0, 0.0, 0.0}, now);
    const double error = wrap_pi(filt.read_latest().angle_rad - deg2rad(pitch_deg));
    EXPECT_NEAR(rad2deg(error), 0.0, 1e-6) << pitch_deg;
  }
}

TEST(DataPathSanity, ComplementaryFilterCrossesWrappedBoundaryAndIgnoresUnreliableAccel) {
  using clock = std::chrono::steady_clock;
  constexpr double fs_hz = Config::sampling_hz;
  const auto tick = std::chrono::nanoseconds{std::llround(1e9 / fs_hz)};
  auto now = clock::now();
  PitchComplementaryFilter filt;

  now += tick;
  filt.push_sample(accel_for_pitch_g(deg2rad(179.0)), {0.0, 0.0, 0.0}, now);
  const double rate_rad_s = deg2rad(8.0);
  for (int i = 1; i <= static_cast<int>(0.5 * fs_hz); ++i) {
    const double physical_pitch = deg2rad(179.0) + rate_rad_s * static_cast<double>(i) / fs_hz;
    now += tick;
    filt.push_sample(accel_for_pitch_g(physical_pitch), {0.0, rate_rad_s, 0.0}, now);
  }
  const double crossed_pitch = filt.read_latest().angle_rad;
  EXPECT_GE(crossed_pitch, -M_PI);
  EXPECT_LE(crossed_pitch, M_PI);
  EXPECT_NEAR(rad2deg(wrap_pi(crossed_pitch - deg2rad(183.0))), 0.0, 1.0);

  const auto held_pitch = filt.read_latest().angle_rad;
  for (int i = 0; i < static_cast<int>(2.0 * fs_hz); ++i) {
    now += tick;
    filt.push_sample({3.0 * Config::g0, 0.0, 0.0}, {0.0, 0.0, 0.0}, now);
  }
  EXPECT_NEAR(rad2deg(wrap_pi(filt.read_latest().angle_rad - held_pitch)), 0.0, 0.1);
}

TEST(DataPathSanity, ComplementaryFilter_GyroLPF_RiseTime_RateStep) {
  using clock = std::chrono::steady_clock;

  const double fs_hz = Config::sampling_hz;  // e.g., 833 Hz
  const double dt_s = 1.0 / fs_hz;
  const auto tick = std::chrono::nanoseconds{std::llround(1e9 / fs_hz)};

  // Step the Y gyro from 0 to +Ω
  const double step_dps = 150.0;  // realistic fast rotation
  const double step_rps = deg2rad(step_dps);
  const double y10 = 0.10 * step_rps;
  const double y90 = 0.90 * step_rps;

  // Conservative bound again
  const double tau_gyro = 1.0 / (2.0 * M_PI * std::max(0.1, Config::fc_gyro_lpf_hz));
  const int maxN = static_cast<int>(std::ceil(5.0 * tau_gyro * fs_hz));

  auto accel_for_pitch_g = [](double pitch_rad) {
    const double s = std::sin(pitch_rad), c = std::cos(pitch_rad);
    return std::array<double, 3>{-s * Config::g0, 0.0, -c * Config::g0};
  };

  PitchComplementaryFilter filt;
  auto now = clock::now();
  auto push = [&](double pitch_true, double gyro_y_rps) -> ImuSample {
    now += tick;
    filt.push_sample(accel_for_pitch_g(pitch_true), {0.0, gyro_y_rps, 0.0}, now);
    return filt.read_latest();
  };

  // Prime at rest
  for (int i = 0; i < 5; ++i) (void)push(0.0, 0.0);

  // Rate step and hold
  (void)push(0.0, step_rps);

  int k10 = -1, k90 = -1;
  for (int k = 1; k <= maxN; ++k) {
    const ImuSample s = push(0.0, step_rps);
    const double y = s.gyro_rad_s;  // this is the filtered gyro (post-LPF)
    if (k10 < 0 && y >= y10) k10 = k;
    if (k90 < 0 && y >= y90) {
      k90 = k;
      break;
    }
  }

  const double tr_ms = (k10 > 0 && k90 > 0) ? (k90 - k10) * dt_s * 1e3 : maxN * dt_s * 1e3;
  std::printf(
      "[Gyro rise] step=%g°/s fs=%.0fHz k10=%d k90=%d rise=%.1f ms "
      "(bound=%d)\n",
      step_dps, fs_hz, k10, k90, tr_ms, maxN);

  ASSERT_NE(k10, -1) << "Gyro LPF never crossed 10% within bound.";
  ASSERT_NE(k90, -1) << "Gyro LPF never crossed 90% within bound.";
  EXPECT_LE(tr_ms, Config::dpitch_rise_ms) << "Gyro path is too slow for control needs.";
}

TEST(DataPathSanity, ComplementaryFilter_HoldsNonzeroPitchUnderNoisyInput) {
  using clock = std::chrono::steady_clock;

  constexpr double fs_hz = Config::sampling_hz;
  constexpr int total_samples = static_cast<int>(8.0 * fs_hz);
  constexpr int tail_samples = static_cast<int>(2.0 * fs_hz);
  const auto tick = std::chrono::nanoseconds{std::llround(1e9 / fs_hz)};
  const double target_rad = deg2rad(4.0);

  std::mt19937 rng(42);
  std::normal_distribution<double> accel_noise(0.0, 0.20);
  std::normal_distribution<double> gyro_noise(0.0, 0.015);

  PitchComplementaryFilter filt;
  auto now = clock::now();
  double tail_sum_rad = 0.0;
  int tail_count = 0;

  for (int i = 0; i < total_samples; ++i) {
    auto acc = accel_for_pitch_g(target_rad);
    for (double& axis : acc) {
      axis += accel_noise(rng);
    }
    const std::array<double, 3> gyr{0.0, gyro_noise(rng), 0.0};
    now += tick;
    filt.push_sample(acc, gyr, now);
    const ImuSample out = filt.read_latest();
    if (i >= total_samples - tail_samples) {
      tail_sum_rad += out.angle_rad;
      ++tail_count;
    }
  }

  ASSERT_GT(tail_count, 0);
  EXPECT_NEAR(rad2deg(tail_sum_rad / tail_count), 4.0, 0.35);
}

TEST(DataPathSanity, ComplementaryFilter_ReducesRawAccelPitchNoise) {
  using clock = std::chrono::steady_clock;

  constexpr double fs_hz = Config::sampling_hz;
  constexpr int total_samples = static_cast<int>(8.0 * fs_hz);
  constexpr int warmup_samples = static_cast<int>(2.0 * fs_hz);
  const auto tick = std::chrono::nanoseconds{std::llround(1e9 / fs_hz)};
  const double target_rad = deg2rad(3.0);

  std::mt19937 rng(77);
  std::normal_distribution<double> accel_noise(0.0, 0.25);
  std::normal_distribution<double> gyro_noise(0.0, 0.010);

  PitchComplementaryFilter filt;
  auto now = clock::now();
  double raw_sq = 0.0;
  double fused_sq = 0.0;
  int count = 0;

  for (int i = 0; i < total_samples; ++i) {
    auto acc = accel_for_pitch_g(target_rad);
    for (double& axis : acc) {
      axis += accel_noise(rng);
    }
    const std::array<double, 3> gyr{0.0, gyro_noise(rng), 0.0};
    now += tick;
    filt.push_sample(acc, gyr, now);
    const ImuSample out = filt.read_latest();
    if (i >= warmup_samples) {
      const double raw_err = wrap_pi(raw_pitch_from_acc_rad(acc) - target_rad);
      const double fused_err = wrap_pi(out.angle_rad - target_rad);
      raw_sq += raw_err * raw_err;
      fused_sq += fused_err * fused_err;
      ++count;
    }
  }

  ASSERT_GT(count, 0);
  const double raw_rms_deg = rad2deg(std::sqrt(raw_sq / count));
  const double fused_rms_deg = rad2deg(std::sqrt(fused_sq / count));
  EXPECT_LT(fused_rms_deg, raw_rms_deg * 0.55);
}

TEST(DataPathSanity, ComplementaryFilter_GravityBoundsAFixedGyroBias) {
  using clock = std::chrono::steady_clock;

  constexpr double fs_hz = Config::sampling_hz;
  constexpr int total_samples = static_cast<int>(12.0 * fs_hz);
  constexpr int tail_samples = static_cast<int>(2.0 * fs_hz);
  const auto tick = std::chrono::nanoseconds{std::llround(1e9 / fs_hz)};
  const double target_rad = deg2rad(5.0);

  std::mt19937 rng(123);
  std::normal_distribution<double> accel_noise(0.0, 0.15);
  std::normal_distribution<double> gyro_noise(0.0, 0.006);

  PitchComplementaryFilter filt;
  auto now = clock::now();
  double tail_sum_rad = 0.0;
  int tail_count = 0;

  for (int i = 0; i < total_samples; ++i) {
    auto acc = accel_for_pitch_g(target_rad);
    for (double& axis : acc) {
      axis += accel_noise(rng);
    }
    const std::array<double, 3> gyr{0.0, 0.010 + gyro_noise(rng), 0.0};
    now += tick;
    filt.push_sample(acc, gyr, now);
    const ImuSample out = filt.read_latest();
    if (i >= total_samples - tail_samples) {
      tail_sum_rad += out.angle_rad;
      ++tail_count;
    }
  }

  ASSERT_GT(tail_count, 0);
  EXPECT_NEAR(rad2deg(tail_sum_rad / tail_count), 5.0, 2.5);
}
