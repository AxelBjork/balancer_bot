#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>

#include "config.h"
#include "pitch_lpf.h"
#include "stepper.h"

// --- Helpers ---
static inline double deg2rad(double d) {
  return d * M_PI / 180.0;
}
static inline double rad2deg(double r) {
  return r * 180.0 / M_PI;
}

// Build accel consistent with your acc_pitch() = atan2(-ax, az).
// For a desired pitch angle (rad), choose ax = -sin(pitch), az = cos(pitch).
static inline std::array<double, 3> accel_for_pitch_g(double pitch_rad) {
  const double s = std::sin(pitch_rad), c = std::cos(pitch_rad);
  return {-s * Config::g0, 0.0, c * Config::g0};
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
//    We feed samples at a fixed dt and check convergence in "a few" samples.
// ------------------------------------------------------------
TEST(DataPathSanity, ComplementaryFilter_RiseTime_20DegStep) {
  using clock = std::chrono::steady_clock;

  // ---- Required budgets for this project (tune to your plant)

  // ---- Test setup
  const double fs_hz = Config::sampling_hz;  // e.g., 833 Hz
  const double dt_s = 1.0 / fs_hz;
  const auto tick = std::chrono::nanoseconds{(long long)std::llround(1e9 / fs_hz)};

  const double target_deg = 20.0;  // keep within reliable range
  const double target_rad = deg2rad(target_deg);
  const double y10 = 0.10 * target_rad;
  const double y90 = 0.90 * target_rad;

  // Conservative sample bound (5× slower time constant among corr/gyro LPFs)
  const double tau_corr = 1.0 / (2.0 * M_PI * std::max(0.1, Config::fc_acc_corr_hz));
  const double tau_gyro = 1.0 / (2.0 * M_PI * std::max(0.1, Config::fc_gyro_lpf_hz));
  const double tau_dom = std::max(tau_corr, tau_gyro);
  const int maxN = (int)std::ceil(5.0 * tau_dom * fs_hz);

  auto accel_for_pitch_g = [](double pitch_rad) {
    const double s = std::sin(pitch_rad), c = std::cos(pitch_rad);
    return std::array<double, 3>{-s * Config::g0, 0.0, c * Config::g0};  // atan2(-ax, az)
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

  // Instant step and hold (gyro=0 so we isolate accel-correction)
  (void)push(target_rad, 0.0);

  int k10 = -1, k90 = -1;
  for (int k = 1; k <= maxN; ++k) {
    const double y = push(target_rad, 0.0).angle_rad;
    if (k10 < 0 && y >= y10) k10 = k;
    if (k90 < 0 && y >= y90) {
      k90 = k;
      break;
    }
  }

  // Always print a compact summary
  const double t10_ms = (k10 > 0 ? k10 : maxN) * dt_s * 1e3;
  const double t90_ms = (k90 > 0 ? k90 : maxN) * dt_s * 1e3;
  const double tr_ms = (k10 > 0 && k90 > 0) ? (t90_ms - t10_ms) : (maxN * dt_s * 1e3);
  std::printf(
      "[Pitch rise] step=%.1f° fs=%.0fHz k10=%d (%.1f ms) k90=%d (%.1f "
      "ms) rise=%.1f ms (bound=%d)\n",
      target_deg, fs_hz, k10, t10_ms, k90, t90_ms, tr_ms, maxN);

  ASSERT_NE(k10, -1) << "Never crossed 10% within bound.";
  ASSERT_NE(k90, -1) << "Never crossed 90% within bound.";

  // Project requirement: accel-correction must converge fast enough
  EXPECT_LE(tr_ms, Config::pitch_rise_ms) << "Accel-correction path is too slow for control needs.";
}

TEST(DataPathSanity, ComplementaryFilter_GyroLPF_RiseTime_RateStep) {
  using clock = std::chrono::steady_clock;

  // ---- Required budget for fast loop
  constexpr double kRiseAllow_ms = 6.0;  // gyro path must be <= ~6 ms 10->90

  const double fs_hz = Config::sampling_hz;  // e.g., 833 Hz
  const double dt_s = 1.0 / fs_hz;
  const auto tick = std::chrono::nanoseconds{(long long)std::llround(1e9 / fs_hz)};

  // Step the Y gyro from 0 to +Ω
  const double step_dps = 150.0;  // realistic fast rotation
  const double step_rps = deg2rad(step_dps);
  const double y10 = 0.10 * step_rps;
  const double y90 = 0.90 * step_rps;

  // Conservative bound again
  const double tau_gyro = 1.0 / (2.0 * M_PI * std::max(0.1, Config::fc_gyro_lpf_hz));
  const int maxN = (int)std::ceil(5.0 * tau_gyro * fs_hz);

  auto accel_for_pitch_g = [](double pitch_rad) {
    const double s = std::sin(pitch_rad), c = std::cos(pitch_rad);
    return std::array<double, 3>{-s * Config::g0, 0.0, c * Config::g0};
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