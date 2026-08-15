#include "simulator/simulator_runner.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <iostream>
#include <iterator>
#include <limits>
#include <random>
#include <string>
#include <utility>

#include "services/imu/imu_pitch_estimator.h"
#include "services/main/config.h"
#include "services/motor/motor_runner.h"
#include "simulator/tuner_support.h"

namespace {

// Lowest pitch stiffness that passed the physical StepperPhase 1/32 sanity
// check with the locked 350 SPS/(rad/s) rate feedback. This remains an
// offline diagnostic reference, not a production or hardware tune.
constexpr double kOneThirtySecondPitchGain = 12000.0;

std::string sim_pid_path() {
  // Keep this in-process simulator reference suite on the explicit historical
  // DirectActuator profile. The production/default pid.conf is now the
  // StepperPhaseElectrical profile.
  return (std::filesystem::path(BALANCER_REPO_ROOT) /
          "tests/data/direct_actuator_pid.conf")
      .string();
}

struct ScopedStateFeedbackConfig {
  ConfigPidValues values = ConfigPid::numeric_values();
  bool controller_enabled = ConfigPid::controller_enabled;

  ~ScopedStateFeedbackConfig() {
    ConfigPid::apply_numeric(values);
    ConfigPid::controller_enabled = controller_enabled;
  }
};

int matrix_rank(std::vector<std::array<double, 4>> rows) {
  constexpr double kEps = 1e-9;
  int rank = 0;
  int pivot_col = 0;
  while (rank < static_cast<int>(rows.size()) && pivot_col < 4) {
    int pivot = rank;
    while (pivot < static_cast<int>(rows.size()) && std::abs(rows[pivot][pivot_col]) < kEps) {
      ++pivot;
    }
    if (pivot == static_cast<int>(rows.size())) {
      ++pivot_col;
      continue;
    }
    std::swap(rows[rank], rows[pivot]);
    const double pivot_value = rows[rank][pivot_col];
    for (int col = pivot_col; col < 4; ++col) {
      rows[rank][col] /= pivot_value;
    }
    for (int row = 0; row < static_cast<int>(rows.size()); ++row) {
      if (row == rank) {
        continue;
      }
      const double factor = rows[row][pivot_col];
      if (std::abs(factor) < kEps) {
        continue;
      }
      for (int col = pivot_col; col < 4; ++col) {
        rows[row][col] -= factor * rows[rank][col];
      }
    }
    ++rank;
    ++pivot_col;
  }
  return rank;
}

double raw_pitch_deg(const std::array<double, 3>& acc) {
  return std::atan2(-acc[0], -acc[2]) * 180.0 / M_PI;
}

std::array<double, 4> add_inputs(const std::array<double, 4>& left,
                                 const std::array<double, 4>& right) {
  std::array<double, 4> result{};
  for (size_t index = 0; index < result.size(); ++index) {
    result[index] = left[index] + right[index];
  }
  return result;
}

double timeline_difference(const SimulatorRunResult& left, const SimulatorRunResult& right) {
  const size_t count = std::min(left.rows.size(), right.rows.size());
  double difference = 0.0;
  for (size_t index = 0; index < count; ++index) {
    const auto& a = left.rows[index];
    const auto& b = right.rows[index];
    difference = std::max({difference, std::abs(a.plant_pitch_deg - b.plant_pitch_deg),
                           std::abs(a.plant_position - b.plant_position),
                           std::abs(a.actual_wheel_velocity - b.actual_wheel_velocity),
                           std::abs(a.f_app - b.f_app), std::abs(a.missed_steps - b.missed_steps),
                           std::abs(a.traction_limit_n - b.traction_limit_n),
                           std::abs(a.motor_force_limit_n - b.motor_force_limit_n)});
  }
  return difference;
}

double rms_pitch_in_window(const SimulatorRunResult& result, double start_s, double end_s) {
  double squared_sum = 0.0;
  size_t count = 0;
  for (const auto& row : result.rows) {
    if (row.sim_time_s < start_s || row.sim_time_s >= end_s) {
      continue;
    }
    squared_sum += row.plant_pitch_deg * row.plant_pitch_deg;
    ++count;
  }
  return count > 0 ? std::sqrt(squared_sum / static_cast<double>(count)) : 0.0;
}

template <typename Getter>
double band_rms_signal_in_window(const SimulatorRunResult& result, double start_s, double end_s,
                                 int low_hz, int high_hz, Getter getter) {
  std::vector<double> samples;
  for (const auto& row : result.rows) {
    if (row.sim_time_s >= start_s && row.sim_time_s < end_s) {
      samples.push_back(getter(row));
    }
  }
  if (samples.empty()) {
    return 0.0;
  }
  double band_variance = 0.0;
  for (int frequency_hz = low_hz; frequency_hz <= high_hz; ++frequency_hz) {
    double sine_sum = 0.0;
    double cosine_sum = 0.0;
    for (size_t index = 0; index < samples.size(); ++index) {
      const double phase = 2.0 * M_PI * static_cast<double>(frequency_hz) *
                           static_cast<double>(index) / 400.0;
      sine_sum += samples[index] * std::sin(phase);
      cosine_sum += samples[index] * std::cos(phase);
    }
    const double scale = 2.0 / static_cast<double>(samples.size());
    const double amplitude = scale * std::hypot(sine_sum, cosine_sum);
    band_variance += 0.5 * amplitude * amplitude;
  }
  return std::sqrt(band_variance);
}

double band_rms_pitch_in_window(const SimulatorRunResult& result, double start_s, double end_s,
                                int low_hz, int high_hz) {
  return band_rms_signal_in_window(result, start_s, end_s, low_hz, high_hz,
                                   [](const auto& row) { return row.plant_pitch_deg; });
}

template <typename Getter>
double signal_rms_in_window(const SimulatorRunResult& result, double start_s, double end_s,
                            Getter getter) {
  double squared_sum = 0.0;
  size_t count = 0;
  for (const auto& row : result.rows) {
    if (row.sim_time_s < start_s || row.sim_time_s >= end_s) continue;
    const double value = getter(row);
    squared_sum += value * value;
    ++count;
  }
  return count > 0 ? std::sqrt(squared_sum / static_cast<double>(count)) : 0.0;
}

template <typename Getter>
double signal_peak_in_window(const SimulatorRunResult& result, double start_s, double end_s,
                             Getter getter) {
  double peak = 0.0;
  for (const auto& row : result.rows) {
    if (row.sim_time_s < start_s || row.sim_time_s >= end_s) continue;
    peak = std::max(peak, std::abs(getter(row)));
  }
  return peak;
}

template <typename Getter>
double dominant_frequency_hz(const SimulatorRunResult& result, double start_s, double end_s,
                             Getter getter) {
  std::vector<std::pair<double, double>> samples;
  for (const auto& row : result.rows) {
    if (row.sim_time_s >= start_s && row.sim_time_s < end_s) {
      samples.emplace_back(row.sim_time_s, getter(row));
    }
  }
  if (samples.size() < 8) return 0.0;

  double mean = 0.0;
  for (const auto& sample : samples) mean += sample.second;
  mean /= static_cast<double>(samples.size());

  double best_frequency = 0.0;
  double best_amplitude = -1.0;
  for (double frequency_hz = 0.5; frequency_hz <= 30.0; frequency_hz += 0.5) {
    double sine_sum = 0.0;
    double cosine_sum = 0.0;
    for (const auto& sample : samples) {
      const double phase = 2.0 * M_PI * frequency_hz * (sample.first - samples.front().first);
      const double value = sample.second - mean;
      sine_sum += value * std::sin(phase);
      cosine_sum += value * std::cos(phase);
    }
    const double amplitude =
        2.0 * std::hypot(sine_sum, cosine_sum) / static_cast<double>(samples.size());
    if (amplitude > best_amplitude) {
      best_amplitude = amplitude;
      best_frequency = frequency_hz;
    }
  }
  return best_frequency;
}

double settling_time_s(const SimulatorRunResult& result, double start_s,
                       double pitch_threshold_deg = 0.5, double rate_threshold_dps = 8.0,
                       double dwell_s = 0.5) {
  if (result.fell || result.rows.empty()) return std::numeric_limits<double>::infinity();
  for (size_t index = 0; index < result.rows.size(); ++index) {
    if (result.rows[index].sim_time_s < start_s) continue;
    const double end_s = result.rows[index].sim_time_s + dwell_s;
    bool quiet = true;
    for (size_t next = index; next < result.rows.size(); ++next) {
      if (result.rows[next].sim_time_s >= end_s) break;
      if (std::abs(result.rows[next].plant_pitch_deg) > pitch_threshold_deg ||
          std::abs(result.rows[next].plant_pitch_rate_dps) > rate_threshold_dps) {
        quiet = false;
        break;
      }
    }
    if (quiet && result.rows.back().sim_time_s >= end_s) {
      return result.rows[index].sim_time_s - start_s;
    }
  }
  return std::numeric_limits<double>::infinity();
}

struct ControllerScenarioMetrics {
  bool fell = false;
  double settling_s = std::numeric_limits<double>::infinity();
  double overshoot_deg = 0.0;
  double tail_rms_deg = 0.0;
  double peak_rate_dps = 0.0;
  double command_rms_sps = 0.0;
  double command_peak_sps = 0.0;
  double saturation_fraction = 0.0;
  double dominant_frequency_hz = 0.0;
};

ControllerScenarioMetrics measure_controller_scenario(const SimulatorRunResult& result,
                                                       double start_s = 0.0) {
  const double end_s = result.rows.empty() ? start_s : result.rows.back().sim_time_s + 1e-9;
  ControllerScenarioMetrics metrics;
  metrics.fell = result.fell;
  metrics.settling_s = settling_time_s(result, start_s);
  metrics.overshoot_deg = signal_peak_in_window(
      result, start_s, end_s, [](const auto& row) { return row.plant_pitch_deg; });
  metrics.tail_rms_deg = result.tail_rms_pitch_deg;
  metrics.peak_rate_dps = signal_peak_in_window(
      result, start_s, end_s, [](const auto& row) { return row.plant_pitch_rate_dps; });
  metrics.command_rms_sps = signal_rms_in_window(
      result, start_s, end_s, [](const auto& row) { return row.u_sps; });
  metrics.command_peak_sps = signal_peak_in_window(
      result, start_s, end_s, [](const auto& row) { return row.u_sps; });
  const double saturation_sum = signal_rms_in_window(
      result, start_s, end_s, [](const auto& row) { return row.command_saturated; });
  metrics.saturation_fraction = saturation_sum * saturation_sum;
  metrics.dominant_frequency_hz = dominant_frequency_hz(
      result, start_s, end_s, [](const auto& row) { return row.plant_pitch_deg; });
  return metrics;
}

std::string classify_controller_metrics(const ControllerScenarioMetrics& metrics) {
  if (metrics.fell) return "unstable";
  if (metrics.saturation_fraction > 0.05) return "excessive_command_activity";
  if (!std::isfinite(metrics.settling_s)) return "weakly_stable";
  if (metrics.settling_s > 4.0) return "overdamped_or_sluggish";
  if (metrics.tail_rms_deg > 0.25) return "underdamped";
  return "well_damped";
}

double first_time_inside_pitch_band(const SimulatorRunResult& result, double pitch_limit_deg,
                                    double start_s = 0.0) {
  if (result.fell) return std::numeric_limits<double>::infinity();
  for (const auto& row : result.rows) {
    if (row.sim_time_s < start_s) continue;
    if (std::abs(row.plant_pitch_deg) <= pitch_limit_deg) {
      return row.sim_time_s - start_s;
    }
  }
  return std::numeric_limits<double>::infinity();
}

double peak_force_fraction(const SimulatorRunResult& result) {
  double peak = 0.0;
  for (const auto& row : result.rows) {
    const double available = std::min(std::abs(row.motor_force_limit_n),
                                      std::abs(row.traction_limit_n));
    if (available > 1e-9) {
      peak = std::max(peak, std::abs(row.f_app) / available);
    }
  }
  return peak;
}

double recovery_margin(const SimulatorRunResult& result) {
  double margin = 0.0;
  for (const auto& row : result.rows) {
    margin = std::max(
        margin, std::max({std::abs(row.plant_pitch_deg) / 6.0,
                          std::abs(row.plant_pitch_rate_dps) / 40.0,
                          std::abs(row.plant_velocity / Config::meters_per_step) / 1000.0}));
  }
  return margin;
}

double first_time_velocity_band_with_dwell(const SimulatorRunResult& result, double start_s,
                                           double threshold_sps, double dwell_s) {
  for (size_t index = 0; index < result.rows.size(); ++index) {
    const auto& candidate = result.rows[index];
    if (candidate.sim_time_s < start_s ||
        std::abs(candidate.plant_velocity / Config::meters_per_step) > threshold_sps) {
      continue;
    }
    const double dwell_end_s = candidate.sim_time_s + dwell_s;
    bool remains_in_band = true;
    for (size_t dwell_index = index; dwell_index < result.rows.size(); ++dwell_index) {
      const auto& row = result.rows[dwell_index];
      if (row.sim_time_s > dwell_end_s) break;
      if (std::abs(row.plant_velocity / Config::meters_per_step) > threshold_sps * 2.0) {
        remains_in_band = false;
        break;
      }
    }
    if (remains_in_band && result.rows.back().sim_time_s >= dwell_end_s) {
      return candidate.sim_time_s;
    }
  }
  return std::numeric_limits<double>::infinity();
}

double position_at_or_after(const SimulatorRunResult& result, double time_s) {
  for (const auto& row : result.rows) {
    if (row.sim_time_s >= time_s) return row.plant_position;
  }
  return result.rows.empty() ? 0.0 : result.rows.back().plant_position;
}

double maximum_position_displacement(const SimulatorRunResult& result, double start_s,
                                     double end_s) {
  const double initial_position = position_at_or_after(result, start_s);
  double displacement = 0.0;
  for (const auto& row : result.rows) {
    if (row.sim_time_s < start_s || row.sim_time_s > end_s) continue;
    displacement = std::max(displacement, std::abs(row.plant_position - initial_position));
  }
  return displacement;
}

struct EquilibriumAngleReplay {
  double raw_deg = 0.0;
  double candidate_deg = 0.0;
  double estimate_deg = 0.0;
  double max_candidate_estimate_gap_deg = 0.0;
  double prototype_trim_deg = 0.0;
  double quiet_time_s = 0.0;
};

EquilibriumAngleReplay replay_equilibrium_angle(const SimulatorRunResult& result) {
  EquilibriumAngleReplay replay;
  bool seeded = false;
  constexpr double kOuterDtS = 0.01;
  for (size_t index = 0; index < result.rows.size(); index += 4) {
    const auto& row = result.rows[index];
    // Reconstruct the controller's q_eq_raw from the signals already present
    // in the simulator timeline.  This is deliberately an offline replay;
    // it does not replace the production trim state machine.
    const double drive_pitch_deg = row.pitch_target_unclamped_deg -
                                    row.velocity_pitch_request_limited_deg - row.com_trim_deg;
    const double raw_deg = row.pitch_deg - drive_pitch_deg -
                           row.velocity_pitch_request_limited_deg;
    if (!seeded) {
      replay.candidate_deg = raw_deg;
      replay.estimate_deg = raw_deg;
      seeded = true;
    } else {
      const double candidate_alpha = std::exp(-2.0 * M_PI * 0.5 * kOuterDtS);
      const double estimate_alpha = std::exp(-2.0 * M_PI * 0.25 * kOuterDtS);
      replay.candidate_deg += (1.0 - candidate_alpha) * (raw_deg - replay.candidate_deg);
      replay.estimate_deg += (1.0 - estimate_alpha) *
                             (replay.candidate_deg - replay.estimate_deg);
    }
    const bool quiet = std::abs(row.velocity_control_sps) <= 100.0 &&
                       row.trim_quiet_rate_rms_dps <= 10.0 &&
                       std::abs(row.pitch_error_deg) <= 1.0 &&
                       row.velocity_authority_limited < 0.5 &&
                       row.trim_learning_block_reason !=
                           static_cast<uint32_t>(ComTrimLearningBlockCommand) &&
                       row.controller_saturation_flags == 0;
    if (quiet) {
      replay.quiet_time_s += kOuterDtS;
      const double trim_alpha = std::exp(-2.0 * M_PI * 0.25 * kOuterDtS);
      replay.prototype_trim_deg += (1.0 - trim_alpha) *
                                   (replay.estimate_deg - replay.prototype_trim_deg);
    }
    replay.raw_deg = raw_deg;
    replay.max_candidate_estimate_gap_deg =
        std::max(replay.max_candidate_estimate_gap_deg,
                 std::abs(replay.candidate_deg - replay.estimate_deg));
  }
  return replay;
}

struct RecoveryInitialState {
  const char* name;
  double pitch_deg;
  double pitch_rate_dps;
  double velocity_sps;
};

SimulatorScenario attitude_recovery_scenario(PhysicsProfile profile,
                                             const RecoveryInitialState& initial) {
  SimulatorScenario scenario;
  scenario.name = std::string("attitude_recovery_") + initial.name;
  scenario.duration_s = 10.0;
  scenario.physics_profile = profile;
  scenario.initial_pitch_deg = initial.pitch_deg;
  scenario.initial_pitch_rate_dps = initial.pitch_rate_dps;
  scenario.initial_velocity_mps = initial.velocity_sps * Config::meters_per_step;
  return scenario;
}

SimulatorScenario attitude_reference_scenario(PhysicsProfile profile, double initial_pitch_deg) {
  SimulatorScenario scenario;
  scenario.name = "attitude_reference";
  scenario.duration_s = 6.0;
  scenario.initial_pitch_deg = initial_pitch_deg;
  scenario.physics_profile = profile;
  return scenario;
}

SimulatorScenario push_reference_scenario(PhysicsProfile profile) {
  SimulatorScenario scenario;
  scenario.name = "push_reference";
  scenario.duration_s = 6.0;
  scenario.physics_profile = profile;
  scenario.disturbances.push_back(SimulatorDisturbance{
      .kind = SimulatorDisturbanceKind::Step,
      .start_s = 1.0,
      .duration_s = 0.10,
      .force_n = 0.25,
  });
  return scenario;
}

SimulatorScenario commanded_pitch_reference_scenario(PhysicsProfile profile) {
  SimulatorScenario scenario;
  scenario.name = "commanded_pitch_reference";
  scenario.duration_s = 7.0;
  scenario.physics_profile = profile;
  scenario.joy_segments.push_back(SimulatorJoySegment{
      .start_s = 1.0,
      .duration_s = 1.0,
      .forward = 0.20,
      .forward_end = 0.20,
  });
  scenario.joy_segments.push_back(SimulatorJoySegment{
      .start_s = 3.0,
      .duration_s = 1.0,
      .forward = -0.20,
      .forward_end = -0.20,
  });
  return scenario;
}

SimulatorScenario velocity_reference_scenario(PhysicsProfile profile,
                                              double initial_velocity_mps = 0.0) {
  SimulatorScenario scenario;
  scenario.name = "velocity_reference";
  scenario.duration_s = 14.0;
  scenario.physics_profile = profile;
  scenario.initial_velocity_mps = initial_velocity_mps;
  scenario.joy_segments = {
      SimulatorJoySegment{.start_s = 1.0,
                          .duration_s = 2.0,
                          .forward = 0.35,
                          .forward_end = 0.35},
      SimulatorJoySegment{.start_s = 6.0,
                          .duration_s = 2.0,
                          .forward = -0.35,
                          .forward_end = -0.35},
  };
  return scenario;
}

SimulatorScenario slow_stop_reference_scenario(PhysicsProfile profile) {
  SimulatorScenario scenario;
  scenario.name = "slow_stop_reference";
  scenario.duration_s = 12.0;
  scenario.physics_profile = profile;
  scenario.joy_segments.push_back(SimulatorJoySegment{
      .start_s = 1.0,
      .duration_s = 1.0,
      .forward = 0.15,
      .forward_end = 0.15,
  });
  return scenario;
}

SimulatorScenario isolated_velocity_sign_scenario(PhysicsProfile profile,
                                                   double initial_velocity_mps) {
  SimulatorScenario scenario;
  scenario.name = "isolated_velocity_sign";
  scenario.duration_s = 12.0;
  scenario.physics_profile = profile;
  scenario.initial_velocity_mps = initial_velocity_mps;
  return scenario;
}

TEST(SimulatorRunnerTest, PositivePitchProducesCorrectiveWheelAndPlantResponse) {
  auto scenario = simulator_named_scenario("pitch_bias_pos", PhysicsProfile::Simplified);
  ASSERT_TRUE(scenario.has_value());
  scenario->duration_s = 0.05;

  const auto result = run_simulator_scenario(*scenario, sim_pid_path());
  ASSERT_FALSE(result.rows.empty());

  const auto it = std::find_if(result.rows.begin(), result.rows.end(),
                               [](const auto& row) { return std::abs(row.f_app) > 1e-6; });
  ASSERT_NE(it, result.rows.end());

  EXPECT_GT(it->f_app, 0.0);
}

TEST(SimulatorRunnerTest, NegativePitchProducesOppositeCorrectiveResponse) {
  auto scenario = simulator_named_scenario("pitch_bias_neg", PhysicsProfile::Simplified);
  ASSERT_TRUE(scenario.has_value());
  scenario->duration_s = 0.05;

  const auto result = run_simulator_scenario(*scenario, sim_pid_path());
  ASSERT_FALSE(result.rows.empty());

  const auto it = std::find_if(result.rows.begin(), result.rows.end(),
                               [](const auto& row) { return std::abs(row.f_app) > 1e-6; });
  ASSERT_NE(it, result.rows.end());

  EXPECT_LT(it->f_app, 0.0);
}

TEST(SimulatorRunnerTest, TelemetryTracksPlantPitch) {
  auto scenario = simulator_named_scenario("neutral_hold", PhysicsProfile::Simplified);
  ASSERT_TRUE(scenario.has_value());
  scenario->duration_s = 0.1;

  const auto result = run_simulator_scenario(*scenario, sim_pid_path());
  ASSERT_FALSE(result.rows.empty());

  for (const auto& row : result.rows) {
    EXPECT_TRUE(std::isfinite(row.pitch_deg));
    EXPECT_TRUE(std::isfinite(row.plant_pitch_deg));
    EXPECT_NEAR(row.pitch_deg, row.plant_pitch_deg, 1e-3);
  }
}

TEST(SimulatorRunnerTest, ImuFiltersReduceStaticRawImuNoiseWithoutStartupBias) {
  BalancerSimulator::Config cfg;
  cfg.initial_pitch_deg = 4.0;
  cfg.physics_profile = PhysicsProfile::Simplified;
  BalancerSimulator sim(cfg);

  ImuPitchEstimator estimator;
  std::mt19937 rng(909);
  std::normal_distribution<double> accel_noise(0.0, 0.10);
  std::normal_distribution<double> gyro_noise(0.0, 0.002);

  constexpr double fs_hz = Config::sampling_hz;
  constexpr int total_samples = static_cast<int>(4.0 * fs_hz);
  constexpr int warmup_samples = static_cast<int>(1.0 * fs_hz);
  const auto tick = std::chrono::nanoseconds{std::llround(1e9 / fs_hz)};
  auto now = std::chrono::steady_clock::now();

  double raw_sq = 0.0;
  double fused_sq = 0.0;
  double fused_sum = 0.0;
  int count = 0;

  for (int i = 0; i < total_samples; ++i) {
    ipc::ImuRawPayload raw = sim.make_raw_imu_payload(static_cast<uint64_t>((i + 1) * 1e6 / fs_hz));
    for (int axis = 0; axis < 3; ++axis) {
      raw.acc[axis] += accel_noise(rng);
      raw.gyr[axis] += gyro_noise(rng);
    }
    now += tick;
    const auto estimate = estimator.push_sample(raw.acc, raw.gyr, now);
    if (i >= warmup_samples && estimate.valid) {
      const double raw_err = raw_pitch_deg(raw.acc) - cfg.initial_pitch_deg;
      const double fused_err =
          estimate.sample.angle_rad * 180.0 / M_PI - cfg.initial_pitch_deg;
      raw_sq += raw_err * raw_err;
      fused_sq += fused_err * fused_err;
      fused_sum += estimate.sample.angle_rad * 180.0 / M_PI;
      ++count;
    }
  }

  ASSERT_GT(count, 0);
  const double raw_rms = std::sqrt(raw_sq / count);
  const double fused_rms = std::sqrt(fused_sq / count);
  EXPECT_LT(fused_rms, raw_rms);
  EXPECT_LT(fused_rms, 1.0);
  EXPECT_NEAR(fused_sum / count, cfg.initial_pitch_deg, 0.35);
}

TEST(SimulatorRunnerTest, PositiveAndNegativeComOffsetsProduceMirroredPlantResponse) {
  BalancerSimulator::Config positive_config;
  positive_config.initial_pitch_deg = 0.0;
  positive_config.com_angle_offset_rad = 0.001;
  BalancerSimulator positive(positive_config);

  BalancerSimulator::Config negative_config = positive_config;
  negative_config.com_angle_offset_rad = -0.001;
  BalancerSimulator negative(negative_config);

  for (int step = 0; step < 40; ++step) {
    positive.step(1.0 / 833.0);
    negative.step(1.0 / 833.0);
  }

  EXPECT_GT(positive.state().pitch, 0.0);
  EXPECT_LT(negative.state().pitch, 0.0);
  EXPECT_NEAR(positive.state().pitch, -negative.state().pitch, 1e-10);
  EXPECT_NEAR(positive.state().velocity, -negative.state().velocity, 1e-10);
}

TEST(SimulatorRunnerTest, HardwareNominalDerivedValuesStayConsistent) {
  using Nominal = BalancerSimulator::HardwareNominal;
  EXPECT_DOUBLE_EQ(Nominal::combined_stall_force_n,
                   Nominal::motor_count * Nominal::motor_stall_torque_nm /
                       Nominal::wheel_radius);
  EXPECT_DOUBLE_EQ(Nominal::meters_per_step,
                   2.0 * M_PI * Nominal::wheel_radius / Nominal::steps_per_rev);
  EXPECT_DOUBLE_EQ(Nominal::stepper_phase_meters_per_step, Config::meters_per_step);
  EXPECT_DOUBLE_EQ(Nominal::stepper_phase_steps_per_rev, Config::steps_per_rev);
  EXPECT_DOUBLE_EQ(BalancerSimulator::physics_for_profile(PhysicsProfile::Realistic).max_force_n,
                   Nominal::combined_stall_force_n);
}

TEST(SimulatorRunnerTest, InitialPitchRateIsAppliedSymmetrically) {
  BalancerSimulator::Config positive_config;
  positive_config.initial_pitch_deg = 0.0;
  positive_config.com_angle_offset_rad = 0.0;
  positive_config.initial_pitch_rate_dps = 30.0;
  BalancerSimulator positive(positive_config);

  BalancerSimulator::Config negative_config = positive_config;
  negative_config.initial_pitch_rate_dps = -30.0;
  BalancerSimulator negative(negative_config);

  EXPECT_NEAR(positive.state().pitch_rate, M_PI / 6.0, 1e-12);
  EXPECT_NEAR(negative.state().pitch_rate, -M_PI / 6.0, 1e-12);
  positive.step(1.0 / 833.0);
  negative.step(1.0 / 833.0);
  EXPECT_NEAR(positive.state().pitch, -negative.state().pitch, 1e-10);
  EXPECT_NEAR(positive.state().pitch_rate, -negative.state().pitch_rate, 1e-10);
}

TEST(SimulatorRunnerTest, TuningScenarioCatalogsAreNominalAndSymmetric) {
  const auto inner = tuning_inner_scenario_set();
  ASSERT_EQ(inner.size(), 5u);
  EXPECT_EQ(inner[0].name, "tuning_inner_neutral");
  EXPECT_DOUBLE_EQ(inner[3].initial_pitch_rate_dps, 30.0);
  EXPECT_DOUBLE_EQ(inner[4].initial_pitch_rate_dps, -30.0);

  const auto authority = tuning_authority_scenario_set();
  ASSERT_EQ(authority.size(), 4u);
  EXPECT_DOUBLE_EQ(authority[0].initial_pitch_deg, 6.0);
  EXPECT_DOUBLE_EQ(authority[1].initial_pitch_deg, -6.0);
  ASSERT_EQ(authority[2].disturbances.size(), 1u);
  EXPECT_DOUBLE_EQ(authority[2].disturbances[0].force_n, 0.5);
  EXPECT_DOUBLE_EQ(authority[3].disturbances[0].force_n, -0.5);

  const auto drive = tuning_drive_scenario_set();
  ASSERT_EQ(drive.size(), 1u);
  ASSERT_EQ(drive[0].joy_segments.size(), 2u);
  EXPECT_DOUBLE_EQ(drive[0].joy_segments[0].start_s, 1.0);
  EXPECT_DOUBLE_EQ(drive[0].joy_segments[0].duration_s, 2.0);
  EXPECT_DOUBLE_EQ(drive[0].joy_segments[0].forward, 0.5);
  EXPECT_DOUBLE_EQ(drive[0].joy_segments[1].start_s, 5.0);
  EXPECT_DOUBLE_EQ(drive[0].joy_segments[1].forward, -0.5);
  const auto velocity = tuning_velocity_scenario_set();
  const auto trim = tuning_trim_scenario_set();
  for (const auto* catalog : {&inner, &authority, &drive, &velocity, &trim}) {
    for (const auto& scenario : *catalog) {
      EXPECT_EQ(scenario.physics_profile, PhysicsProfile::Realistic);
      EXPECT_FALSE(scenario.physics_override.has_value());
      EXPECT_DOUBLE_EQ(scenario.imu_pitch_lag_s, 0.0);
      EXPECT_DOUBLE_EQ(scenario.accel_noise_std_mps2, 0.0);
      EXPECT_DOUBLE_EQ(scenario.gyro_noise_std_rad_s, 0.0);
      for (const auto& segment : scenario.joy_segments) EXPECT_DOUBLE_EQ(segment.turn, 0.0);
    }
  }

  const auto stepper_velocity =
      tuning_velocity_scenario_set(PhysicsProfile::StepperPhaseElectrical);
  for (const auto& scenario : stepper_velocity) {
    EXPECT_EQ(scenario.physics_profile, PhysicsProfile::StepperPhaseElectrical);
    EXPECT_FALSE(scenario.physics_override.has_value());
  }
}

TEST(TunerSupportTest, MetricsAndParetoUtilitiesAreDeterministic) {
  SimulatorRunResult result;
  result.scenario.name = "tuning_inner_release_pos";
  result.scenario.duration_s = 1.0;
  result.scenario.initial_pitch_deg = 1.0;
  const auto row = [](double time_s, double pitch_deg, double rate_dps, double command_sps) {
    SimulatorTimelineRow value;
    value.sim_time_s = time_s;
    value.plant_pitch_deg = pitch_deg;
    value.plant_pitch_rate_dps = rate_dps;
    value.u_sps = command_sps;
    return value;
  };
  result.rows = {row(0.0, 1.0, -10.0, 10.0), row(0.25, 0.4, -2.0, 5.0),
                 row(0.50, 0.2, 1.0, 2.0), row(0.75, 0.1, 1.0, 1.0),
                 row(1.00, 0.1, 1.0, 1.0)};
  const auto metrics = calculate_tuning_metrics(result);
  EXPECT_TRUE(metrics.safe);
  EXPECT_TRUE(metrics.settled);
  EXPECT_NEAR(metrics.arrest_time_s, 0.5, 1e-12);
  EXPECT_GT(metrics.command_total_variation_sps, 0.0);
  EXPECT_NE(tuning_metrics_csv_header().find("settling_time_s"), std::string::npos);
  EXPECT_NE(tuning_metrics_csv_row(metrics).find(','), std::string::npos);
  EXPECT_TRUE(tuning_metrics_dominate({1.0, 2.0}, {1.0, 3.0}));
  EXPECT_FALSE(tuning_metrics_dominate({2.0, 1.0}, {1.0, 2.0}));
  EXPECT_EQ(normalized_tuning_metric(2.0, 1.0), 2.0);
}

TEST(TunerSupportTest, StageRankingObjectivesAndDampingTieBreakAreDeterministic) {
  TunerRankingSummary value;
  value.score = 1.0;
  value.worst_settling_time_s = 2.0;
  value.total_pitch_iae_deg_s = 3.0;
  value.worst_peak_pitch_deg = 4.0;
  value.worst_peak_rate_dps = 5.0;
  value.neutral_command_variation_sps = 6.0;
  value.worst_arrest_time_s = 0.4;
  value.worst_rebound_ratio = 0.3;
  value.max_continuous_saturation_s = 0.2;
  value.residual_velocity_sps = 50.0;
  value.post_recovery_command_variation_sps = 70.0;
  value.total_velocity_iae_sps_s = 8.0;
  value.total_drive_tracking_mae_sps = 9.0;
  value.total_stop_speed_rms_sps = 10.0;
  value.trim_speed_magnitude_sps = 11.0;
  value.trim_symmetry_sps = 12.0;

  EXPECT_EQ(tuning_pareto_objectives(TunerRankingStage::Inner, value),
            (std::vector<double>{2.0, 3.0, 4.0, 5.0, 6.0}));
  EXPECT_EQ(tuning_pareto_objectives(TunerRankingStage::Authority, value),
            (std::vector<double>{0.4, 2.0, 0.3, 0.2, 50.0, 70.0}));
  EXPECT_EQ(tuning_pareto_objectives(TunerRankingStage::Velocity, value),
            (std::vector<double>{8.0, 9.0, 2.0, 4.0, 70.0}));
  EXPECT_EQ(tuning_pareto_objectives(TunerRankingStage::Drive, value),
            (std::vector<double>{9.0, 10.0, 4.0, 0.2, 70.0}));
  EXPECT_EQ(tuning_pareto_objectives(TunerRankingStage::Trim, value),
            (std::vector<double>{11.0, 12.0, 70.0, 2.0}));
  EXPECT_EQ(tuning_pareto_objectives(TunerRankingStage::Joint, value),
            (std::vector<double>{2.0, 3.0, 6.0, 8.0, 9.0}));

  TunerRankingSummary lower_damping = value;
  lower_damping.velocity_damping_per_s = 7.0;
  TunerRankingSummary higher_damping = value;
  higher_damping.velocity_damping_per_s = 8.0;
  EXPECT_TRUE(tuning_velocity_scores_equivalent(1.05, 1.0));
  EXPECT_FALSE(tuning_velocity_scores_equivalent(1.051, 1.0));
  EXPECT_TRUE(tuning_stage_tie_break_less(TunerRankingStage::Velocity, lower_damping,
                                           higher_damping));
  EXPECT_FALSE(tuning_stage_tie_break_less(TunerRankingStage::Velocity, higher_damping,
                                            lower_damping));

  TunerRankingSummary asymmetric_trim = value;
  asymmetric_trim.trim_speed_magnitude_sps = 10.0;
  asymmetric_trim.trim_symmetry_sps = 4.0;
  TunerRankingSummary symmetric_trim = asymmetric_trim;
  symmetric_trim.trim_symmetry_sps = 0.0;
  EXPECT_TRUE(tuning_metrics_dominate(
      tuning_pareto_objectives(TunerRankingStage::Trim, symmetric_trim),
      tuning_pareto_objectives(TunerRankingStage::Trim, asymmetric_trim)));
}

TEST(SimulatorRunnerTest, UnifiedEngineIsDeterministicForSameSeedAndInputs) {
  SimulatorScenario scenario;
  scenario.duration_s = 0.5;
  scenario.physics_profile = PhysicsProfile::Realistic;
  scenario.initial_pitch_deg = 0.1;
  scenario.imu_noise_seed = 42;
  scenario.accel_noise_std_mps2 = 0.02;
  scenario.gyro_noise_std_rad_s = 0.001;
  scenario.imu_timestamp_jitter_us = 50.0;

  SimulatorEngine first(scenario);
  SimulatorEngine second(scenario);
  for (int step = 0; step < 200; ++step) {
    const auto a = first.step();
    const auto b = second.step();
    EXPECT_DOUBLE_EQ(a.plant_pitch_deg, b.plant_pitch_deg);
    EXPECT_DOUBLE_EQ(a.fused_pitch_deg, b.fused_pitch_deg);
    EXPECT_DOUBLE_EQ(a.left_actual_steps, b.left_actual_steps);
    EXPECT_DOUBLE_EQ(a.f_app, b.f_app);
  }
}

TEST(SimulatorRunnerTest, ScheduledAndUdpStyleJoystickInputsProduceEquivalentTimelines) {
  SimulatorScenario scheduled_scenario;
  scheduled_scenario.duration_s = 0.5;
  scheduled_scenario.physics_profile = PhysicsProfile::Realistic;
  scheduled_scenario.imu_noise_seed = 77;
  scheduled_scenario.joy_segments.push_back(SimulatorJoySegment{
      .start_s = 0.0,
      .duration_s = 0.0,
      .forward = 0.4,
      .turn = -0.2,
      .forward_end = 0.4,
      .turn_end = -0.2,
  });
  SimulatorScenario external_scenario = scheduled_scenario;
  external_scenario.joy_segments.clear();

  SimulatorEngine scheduled(scheduled_scenario);
  SimulatorEngine udp_style(external_scenario);
  udp_style.set_joystick(0.4, -0.2);
  for (int step = 0; step < 200; ++step) {
    const auto direct = scheduled.step();
    const auto wrapped = udp_style.step();
    EXPECT_DOUBLE_EQ(direct.plant_pitch_deg, wrapped.plant_pitch_deg);
    EXPECT_DOUBLE_EQ(direct.plant_position, wrapped.plant_position);
    EXPECT_DOUBLE_EQ(direct.left_actual_steps, wrapped.left_actual_steps);
    EXPECT_DOUBLE_EQ(direct.right_actual_steps, wrapped.right_actual_steps);
    EXPECT_DOUBLE_EQ(direct.fused_pitch_deg, wrapped.fused_pitch_deg);
  }
}

TEST(SimulatorRunnerTest, DirectAndUdpStylePathsUseProductionSlewAndPulseFrameTiming) {
  ScopedStateFeedbackConfig restore;
  ConfigPid::load(sim_pid_path());

  EXPECT_DOUBLE_EQ(Config::motor_slew_sps_per_s, 200000.0);
  EXPECT_EQ(DualWave::kFrameUs, 2500U);

  SimulatorScenario direct_scenario;
  direct_scenario.duration_s = 0.05;
  direct_scenario.physics_profile = PhysicsProfile::Realistic;
  direct_scenario.joy_segments.push_back(SimulatorJoySegment{
      .start_s = 0.0,
      .duration_s = 0.0,
      .forward = 1.0,
      .turn = 0.0,
      .forward_end = 1.0,
      .turn_end = 0.0,
  });
  SimulatorScenario udp_style_scenario = direct_scenario;
  udp_style_scenario.joy_segments.clear();
  SimulatorEngine direct(direct_scenario);
  SimulatorEngine udp_style(udp_style_scenario);
  udp_style.set_joystick(1.0, 0.0);

  for (int step = 0; step < 20; ++step) {
    const auto direct_row = direct.step();
    const auto udp_row = udp_style.step();
    EXPECT_DOUBLE_EQ(direct_row.motor_update_dt_ms, 2.5);
    EXPECT_DOUBLE_EQ(udp_row.motor_update_dt_ms, 2.5);
    EXPECT_LE(std::abs(direct_row.left_slewed_sps -
                       (step == 0 ? 0.0 : direct_row.left_sps)),
              500.0);
    EXPECT_DOUBLE_EQ(direct_row.left_slewed_sps, udp_row.left_slewed_sps);
    EXPECT_DOUBLE_EQ(direct_row.right_slewed_sps, udp_row.right_slewed_sps);
    EXPECT_EQ(direct_row.actuator_saturation_flags, udp_row.actuator_saturation_flags);
  }
}

TEST(SimulatorRunnerTest, NineHertzAlternatingForceRecoveryDecaysWithoutRailsOrFaults) {
  SimulatorScenario scenario;
  scenario.name = "nine_hertz_alternating_force_recovery";
  scenario.duration_s = 8.0;
  scenario.physics_profile = PhysicsProfile::DirectActuator;
  constexpr double kHalfPeriodS = 1.0 / 18.0;
  for (int half_cycle = 0; half_cycle < 18; ++half_cycle) {
    scenario.disturbances.push_back(SimulatorDisturbance{
        .kind = SimulatorDisturbanceKind::Step,
        .start_s = 1.0 + static_cast<double>(half_cycle) * kHalfPeriodS,
        .duration_s = kHalfPeriodS,
        .force_n = half_cycle % 2 == 0 ? 1.0 : -1.0,
    });
  }

  const SimulatorRunResult result = run_simulator_scenario(scenario, sim_pid_path());
  ASSERT_EQ(result.rows.size(), 3200U);
  double excitation_peak_pitch_deg = 0.0;
  for (const auto& row : result.rows) {
    if (row.sim_time_s >= 1.0 && row.sim_time_s < 2.0) {
      excitation_peak_pitch_deg =
          std::max(excitation_peak_pitch_deg, std::abs(row.plant_pitch_deg));
    }
    EXPECT_EQ(row.controller_fault_flags, 0U);
    EXPECT_EQ(row.actuator_fault, 0.0);
    EXPECT_EQ(row.command_saturated, 0.0);
    EXPECT_EQ(row.force_saturated, 0.0);
  }
  const double first_post_excitation_band_rms =
      band_rms_pitch_in_window(result, 2.0, 3.0, 6, 12);
  const double late_band_rms = band_rms_pitch_in_window(result, 7.0, 8.0, 6, 12);
  const double late_total_rms = rms_pitch_in_window(result, 7.0, 8.0);
  EXPECT_GT(excitation_peak_pitch_deg, 0.05);
  EXPECT_GT(first_post_excitation_band_rms, 1e-4);
  EXPECT_LT(late_band_rms, 0.5 * first_post_excitation_band_rms);
  EXPECT_LT(late_total_rms, 0.5);
}

TEST(SimulatorRunnerTest, TranslationalAccelerationChangesRawImuSpecificForce) {
  BalancerSimulator::Config config;
  config.initial_pitch_deg = 0.0;
  config.com_angle_offset_rad = 0.0;
  BalancerSimulator simulator(config);
  const auto at_rest = simulator.make_raw_imu_payload(0);
  simulator.set_external_force_n(3.0);
  simulator.step(1.0 / 833.0);
  const auto accelerating = simulator.make_raw_imu_payload(1200);

  EXPECT_GT(std::abs(accelerating.acc[0] - at_rest.acc[0]), 0.1);
  EXPECT_GT(std::abs(simulator.diagnostics().x_ddot), 0.1);
}

TEST(SimulatorRunnerTest, TranslationCreatesSymmetricApparentPitchAndFilterRecovers) {
  BalancerSimulator::Config rest_config;
  rest_config.initial_pitch_deg = 0.0;
  rest_config.com_angle_offset_rad = 0.0;
  const BalancerSimulator at_rest_simulator(rest_config);
  const auto at_rest = at_rest_simulator.make_raw_imu_payload(0);

  std::array<double, 2> apparent_pitch{};
  for (size_t index = 0; index < apparent_pitch.size(); ++index) {
    const double sign = index == 0 ? -1.0 : 1.0;
    BalancerSimulator simulator(rest_config);
    simulator.set_external_force_n(sign * 3.0);
    simulator.step(1.0 / Config::sampling_hz);
    const auto translated = simulator.make_raw_imu_payload(1200);

    ImuPitchEstimator estimator;
    auto estimate =
        estimator.push_sample(translated.acc, translated.gyr,
                              ImuPitchEstimator::TimePoint{});
    ASSERT_TRUE(estimate.valid);
    apparent_pitch[index] = std::atan2(-translated.acc[0], -translated.acc[2]);
    EXPECT_DOUBLE_EQ(estimate.sample.angle_rad, 0.0);

    for (int sample = 1; sample <= static_cast<int>(4.0 * Config::sampling_hz);
         ++sample) {
      estimate = estimator.push_sample(
          at_rest.acc, at_rest.gyr,
          ImuPitchEstimator::TimePoint(std::chrono::duration_cast<
                                      ImuPitchEstimator::TimePoint::duration>(
              std::chrono::duration<double>(
                  static_cast<double>(sample) / Config::sampling_hz))));
    }
    ASSERT_TRUE(estimate.valid);
    EXPECT_NEAR(estimate.sample.angle_rad, 0.0, 1e-4);
  }

  EXPECT_GT(apparent_pitch[0], 0.0);
  EXPECT_LT(apparent_pitch[1], 0.0);
  EXPECT_NEAR(apparent_pitch[0], -apparent_pitch[1], 1e-6);
}

TEST(SimulatorRunnerTest, RawGyroBiasChangesRateButCreatesOnlyBoundedPitchOffset) {
  BalancerSimulator::Config config;
  config.initial_pitch_deg = 4.0;
  config.com_angle_offset_rad = 0.0;
  const BalancerSimulator simulator(config);
  const auto raw = simulator.make_raw_imu_payload(0);

  ImuPitchEstimator unbiased;
  ImuPitchEstimator biased;
  ImuPitchEstimate unbiased_estimate;
  ImuPitchEstimate biased_estimate;
  for (int sample = 0; sample <= static_cast<int>(0.25 * Config::sampling_hz);
       ++sample) {
    const auto timestamp =
        ImuPitchEstimator::TimePoint(std::chrono::duration_cast<
                                    ImuPitchEstimator::TimePoint::duration>(
            std::chrono::duration<double>(
                static_cast<double>(sample) / Config::sampling_hz)));
    auto biased_gyro = raw.gyr;
    biased_gyro[1] += 0.05;
    unbiased_estimate = unbiased.push_sample(raw.acc, raw.gyr, timestamp);
    biased_estimate = biased.push_sample(raw.acc, biased_gyro, timestamp);
  }

  ASSERT_TRUE(unbiased_estimate.valid);
  ASSERT_TRUE(biased_estimate.valid);
  EXPECT_LT(std::abs(biased_estimate.sample.angle_rad -
                     unbiased_estimate.sample.angle_rad),
            M_PI / 180.0);
  EXPECT_NEAR(biased_estimate.sample.gyro_rad_s -
                  unbiased_estimate.sample.gyro_rad_s,
              0.05, 1e-5);
}

TEST(SimulatorRunnerTest, EveryRetainedPlantParameterAffectsTheTimeline) {
  SimulatorScenario nominal;
  nominal.duration_s = 0.8;
  nominal.initial_pitch_deg = 2.0;
  nominal.physics_profile = PhysicsProfile::Realistic;
  nominal.disturbances.push_back(SimulatorDisturbance{
      .kind = SimulatorDisturbanceKind::Step,
      .start_s = 0.1,
      .duration_s = 0.1,
      .force_n = 3.0,
  });
  nominal.physics_override = BalancerSimulator::physics_for_profile(PhysicsProfile::Realistic);
  const auto baseline = run_simulator_scenario(nominal, sim_pid_path());

  const std::vector<std::function<void(SimulatorScenario&)>> variations{
      [](auto& value) { value.total_mass_scale = 1.1; },
      [](auto& value) { value.pitch_inertia_scale = 1.2; },
      [](auto& value) { value.physics_override->no_load_speed_mps *= 0.7; },
      [](auto& value) { value.physics_override->traction_coefficient = 0.7; },
      [](auto& value) { value.physics_override->motor_velocity_damping *= 0.7; },
      [](auto& value) { value.physics_override->cart_damping = 0.3; },
      [](auto& value) { value.physics_override->pitch_damping = 0.0; },
      [](auto& value) { value.physics_override->motor_tau_s = 0.020; },
      [](auto& value) { value.physics_override->phase_error_limit_steps = 4.0; },
      [](auto& value) { value.physics_override->tire_stiffness_n_per_m *= 0.7; },
      [](auto& value) { value.physics_override->tire_damping_n_s_per_m *= 0.7; },
      [](auto& value) { value.physics_override->wheel_equivalent_mass_kg *= 1.3; },
  };

  for (size_t index = 0; index < variations.size(); ++index) {
    SimulatorScenario varied = nominal;
    variations[index](varied);
    const auto result = run_simulator_scenario(varied, sim_pid_path());
    EXPECT_GT(timeline_difference(baseline, result), 1e-8) << "variation " << index;
  }
}

TEST(SimulatorRunnerTest, EveryRetainedImuImpairmentAffectsTheTimeline) {
  SimulatorScenario nominal;
  nominal.duration_s = 0.5;
  nominal.initial_pitch_deg = 1.0;
  nominal.physics_profile = PhysicsProfile::Realistic;
  nominal.imu_noise_seed = 31415;
  const auto baseline = run_simulator_scenario(nominal, sim_pid_path());

  const std::vector<std::function<void(SimulatorScenario&)>> variations{
      [](auto& value) { value.accel_noise_std_mps2 = 0.1; },
      [](auto& value) { value.gyro_noise_std_rad_s = 0.01; },
      [](auto& value) { value.accel_bias_mps2[0] = 0.1; },
      [](auto& value) { value.gyro_bias_rad_s[1] = 0.002; },
      [](auto& value) { value.imu_pitch_lag_s = 0.01; },
      [](auto& value) { value.imu_timestamp_jitter_us = 100.0; },
      [](auto& value) { value.imu_sample_loss_rate = 0.2; },
  };

  for (size_t index = 0; index < variations.size(); ++index) {
    SimulatorScenario varied = nominal;
    variations[index](varied);
    const auto result = run_simulator_scenario(varied, sim_pid_path());
    double difference = timeline_difference(baseline, result);
    for (size_t row = 0; row < std::min(baseline.rows.size(), result.rows.size()); ++row) {
      difference = std::max(
          {difference,
           std::abs(baseline.rows[row].fused_pitch_deg - result.rows[row].fused_pitch_deg),
           std::abs(baseline.rows[row].gyro_pitch_rate_dps - result.rows[row].gyro_pitch_rate_dps),
           static_cast<double>(
               baseline.rows[row].imu_timestamp_us > result.rows[row].imu_timestamp_us
                   ? baseline.rows[row].imu_timestamp_us - result.rows[row].imu_timestamp_us
                   : result.rows[row].imu_timestamp_us - baseline.rows[row].imu_timestamp_us)});
    }
    EXPECT_GT(difference, 1e-8) << "variation " << index;
  }
}

TEST(SimulatorRunnerTest, RampDisturbanceBuildsCommandMagnitudeOverTime) {
  SimulatorScenario scenario;
  scenario.name = "ramp_disturbance";
  scenario.duration_s = 1.0;
  scenario.physics_profile = PhysicsProfile::Simplified;
  scenario.disturbances.push_back(SimulatorDisturbance{
      .kind = SimulatorDisturbanceKind::Ramp,
      .start_s = 0.1,
      .duration_s = 0.5,
      .force_n = 0.0f,
      .com_bias_rad = 0.0f,
      .force_n_end = 0.5f,
      .com_bias_rad_end = 0.0f,
  });

  const auto result = run_simulator_scenario(scenario, sim_pid_path());
  ASSERT_FALSE(result.rows.empty());

  double early_force = 0.0;
  double late_force = 0.0;
  int early_count = 0;
  int late_count = 0;
  for (const auto& row : result.rows) {
    if (row.sim_time_s >= 0.15 && row.sim_time_s < 0.25) {
      early_force += row.external_force_n;
      ++early_count;
    }
    if (row.sim_time_s >= 0.45 && row.sim_time_s < 0.55) {
      late_force += row.external_force_n;
      ++late_count;
    }
  }

  ASSERT_GT(early_count, 0);
  ASSERT_GT(late_count, 0);
  EXPECT_GT(late_force / late_count, early_force / early_count);
}

TEST(SimulatorRunnerTest, HoldBiasDisturbancePersistsUntilRunEnds) {
  SimulatorScenario scenario;
  scenario.name = "hold_bias";
  scenario.duration_s = 1.0;
  scenario.physics_profile = PhysicsProfile::Simplified;
  scenario.disturbances.push_back(SimulatorDisturbance{
      .kind = SimulatorDisturbanceKind::HoldBias,
      .start_s = 0.2,
      .duration_s = 0.0,
      .force_n = 0.0f,
      .com_bias_rad = 0.02f,
  });

  const auto result = run_simulator_scenario(scenario, sim_pid_path());
  ASSERT_FALSE(result.rows.empty());

  const auto tail = result.rows.back();
  EXPECT_GT(tail.external_com_bias_rad, 0.0);
}

TEST(SimulatorRunnerTest, JoySegmentsDriveForwardAndTurnCommands) {
  SimulatorScenario scenario;
  scenario.name = "joy_segments";
  scenario.duration_s = 0.8;
  scenario.physics_profile = PhysicsProfile::Simplified;
  scenario.joy_segments.push_back(SimulatorJoySegment{
      .start_s = 0.1,
      .duration_s = 0.4,
      .forward = 0.0,
      .turn = 0.0,
      .forward_end = 0.6,
      .turn_end = 0.7,
  });

  const auto result = run_simulator_scenario(scenario, sim_pid_path());
  ASSERT_FALSE(result.rows.empty());

  double max_pitch_sp = 0.0;
  double max_nominal_acceleration = 0.0;
  double max_turn_split = 0.0;
  for (const auto& row : result.rows) {
    if (row.sim_time_s >= 0.2 && row.sim_time_s <= 0.5) {
      max_pitch_sp = std::max(max_pitch_sp, std::abs(row.pitch_sp_deg));
      max_nominal_acceleration =
          std::max(max_nominal_acceleration, std::abs(row.nominal_acceleration_mps2));
      max_turn_split = std::max(max_turn_split, std::abs(row.left_sps - row.right_sps));
    }
  }

  EXPECT_GT(max_pitch_sp, 0.25);
  EXPECT_GT(max_nominal_acceleration, 0.1);
  EXPECT_GT(max_turn_split, 100.0);
}

TEST(SimulatorRunnerTest, DriveForceRespectsMotorAndTractionLimits) {
  auto scenario = simulator_named_scenario("pitch_bias_pos", PhysicsProfile::Realistic);
  ASSERT_TRUE(scenario.has_value());
  scenario->duration_s = 0.05;

  const auto result = run_simulator_scenario(*scenario, sim_pid_path());
  ASSERT_FALSE(result.rows.empty());

  const auto it = std::find_if(result.rows.begin(), result.rows.end(),
                               [](const auto& row) { return std::abs(row.f_app) > 1e-6; });
  ASSERT_NE(it, result.rows.end());
  EXPECT_LE(std::abs(it->f_app), it->motor_force_limit_n + 1e-9);
  EXPECT_LE(std::abs(it->f_app), it->traction_limit_n + 1e-9);
  EXPECT_LE(std::abs(it->phase_error_steps), result.physics.phase_error_limit_steps + 1e-9);
}

TEST(SimulatorRunnerTest, MotorAuthorityEnvelopeUsesRotorSpeedNotRequestedSpeed) {
  BalancerSimulator simulator;
  simulator.set_motor_targets(8000.0, 8000.0);
  simulator.set_emitted_steps(10.0, 10.0);
  simulator.step(1.0 / 833.0);
  EXPECT_NEAR(simulator.diagnostics().motor_force_limit_n, simulator.physics().max_force_n, 1e-9);

  double minimum_limit = simulator.diagnostics().motor_force_limit_n;
  for (int step = 1; step < 300; ++step) {
    simulator.set_emitted_steps(10.0 + 10.0 * step, 10.0 + 10.0 * step);
    simulator.step(1.0 / 833.0);
    minimum_limit = std::min(minimum_limit, simulator.diagnostics().motor_force_limit_n);
  }
  EXPECT_LT(minimum_limit, simulator.physics().max_force_n * 0.95);
}

TEST(SimulatorRunnerTest, InitialPitchDoesNotCreateMotorPhaseOrMissedSteps) {
  for (const double initial_pitch_deg : {67.0, -67.0}) {
    BalancerSimulator::Config config;
    config.initial_pitch_deg = initial_pitch_deg;
    config.com_angle_offset_rad = 0.0;
    BalancerSimulator simulator(config);
    simulator.set_emitted_steps(0.0, 0.0);

    simulator.step(0.0);

    EXPECT_NEAR(simulator.diagnostics().phase_error_steps, 0.0, 1e-12)
        << initial_pitch_deg;
    EXPECT_NEAR(simulator.diagnostics().missed_steps, 0.0, 1e-12) << initial_pitch_deg;
  }
}

TEST(SimulatorRunnerTest, MotorForceAppliesSymmetricReactionTorqueBeforeTireForceBuilds) {
  const auto model = BalancerSimulator::linearized_upright_model(
      BalancerSimulator::physics_for_profile(PhysicsProfile::Realistic));
  std::array<double, 2> translation_accels{};
  std::array<double, 2> pitch_accels{};

  for (size_t index = 0; index < pitch_accels.size(); ++index) {
    const double direction = index == 0 ? 1.0 : -1.0;
    BalancerSimulator::Config config;
    config.initial_pitch_deg = 0.0;
    config.com_angle_offset_rad = 0.0;
    config.physics_override =
        BalancerSimulator::physics_for_profile(PhysicsProfile::Realistic);
    config.physics_override->motor_tau_s = 0.0;
    config.physics_override->motor_velocity_damping = 0.0;
    config.physics_override->tire_stiffness_n_per_m = 0.0;
    config.physics_override->tire_damping_n_s_per_m = 0.0;
    BalancerSimulator simulator(config);
    simulator.set_emitted_steps(direction, direction);

    simulator.step(1e-6);

    ASSERT_NEAR(simulator.diagnostics().f_app, 0.0, 1e-12);
    EXPECT_NEAR(simulator.diagnostics().x_ddot,
                model.motor_force_input[1] * simulator.diagnostics().f_cmd, 1e-9);
    EXPECT_NEAR(simulator.diagnostics().theta_ddot,
                model.motor_force_input[3] * simulator.diagnostics().f_cmd, 1e-9);
    translation_accels[index] = simulator.diagnostics().x_ddot;
    pitch_accels[index] = simulator.diagnostics().theta_ddot;
  }

  EXPECT_GT(translation_accels[0], 0.0);
  EXPECT_LT(translation_accels[1], 0.0);
  EXPECT_NEAR(translation_accels[0], -translation_accels[1], 1e-12);
  EXPECT_LT(pitch_accels[0], 0.0);
  EXPECT_GT(pitch_accels[1], 0.0);
  EXPECT_NEAR(pitch_accels[0], -pitch_accels[1], 1e-12);
}

TEST(SimulatorRunnerTest, ChassisPitchRateContributesToMotorSpeedAndAuthorityEnvelope) {
  for (const double initial_pitch_deg : {5.0, -5.0}) {
    BalancerSimulator::Config config;
    config.initial_pitch_deg = initial_pitch_deg;
    config.com_angle_offset_rad = 0.0;
    config.physics_override =
        BalancerSimulator::physics_for_profile(PhysicsProfile::Realistic);
    config.physics_override->tire_stiffness_n_per_m = 0.0;
    config.physics_override->tire_damping_n_s_per_m = 0.0;
    BalancerSimulator simulator(config);
    simulator.set_emitted_steps(0.0, 0.0);

    simulator.step(0.01);
    const double relative_speed = simulator.diagnostics().actual_wheel_velocity;
    EXPECT_NEAR(relative_speed,
                -BalancerSimulator::HardwareNominal::wheel_radius * simulator.state().pitch_rate,
                1e-12);
    EXPECT_NEAR(simulator.diagnostics().velocity_error, -relative_speed, 1e-12);
    EXPECT_NE(relative_speed, 0.0);

    simulator.step(0.0);
    const double expected_limit =
        simulator.physics().max_force_n *
        std::clamp(1.0 - std::abs(relative_speed) / simulator.physics().no_load_speed_mps,
                   0.0, 1.0);
    EXPECT_NEAR(simulator.diagnostics().motor_force_limit_n, expected_limit, 1e-12);
    EXPECT_NE(simulator.diagnostics().phase_error_steps, 0.0);
  }
}

TEST(SimulatorRunnerTest, TireDeflectionDoesNotBecomeMissedMotorSteps) {
  BalancerSimulator simulator;
  simulator.set_emitted_steps(0.0, 0.0);
  simulator.set_external_force_n(0.5);
  bool saw_tire_force = false;
  for (int step = 0; step < 20; ++step) {
    simulator.step(1.0 / 833.0);
    saw_tire_force = saw_tire_force || std::abs(simulator.diagnostics().f_app) > 1e-6;
  }
  EXPECT_TRUE(saw_tire_force);
  EXPECT_NEAR(simulator.diagnostics().missed_steps, 0.0, 1e-12);
}

TEST(SimulatorRunnerTest, SmallAngleLinearizedPlantIsControllableWithOverdampedPoleTargets) {
  const auto physics = BalancerSimulator::physics_for_profile(PhysicsProfile::Realistic);
  const auto model = BalancerSimulator::linearized_upright_model(physics);

  std::vector<std::array<double, 4>> controllability;
  std::array<double, 4> bk = add_inputs(model.horizontal_force_input, model.motor_force_input);
  for (int power = 0; power < 4; ++power) {
    controllability.push_back({bk[0], bk[1], bk[2], bk[3]});
    std::array<double, 4> next{};
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        next[row] += model.A[row][col] * bk[col];
      }
    }
    bk = next;
  }

  EXPECT_EQ(matrix_rank(controllability), 4);

  const auto poles = BalancerSimulator::overdamped_candidate_poles(physics);
  EXPECT_LT(poles[0], 0.0);
  EXPECT_LT(poles[1], poles[0]);
  EXPECT_LT(poles[2], poles[1]);
  EXPECT_LT(poles[3], poles[2]);
}

TEST(SimulatorReferenceTest, StateFeedbackReferenceProfilesExposeArchitectureBoundary) {
  ScopedStateFeedbackConfig restore;
  ConfigPid::load(sim_pid_path());
  ConfigPid::values.velocity_damping_per_s = 0.0;
  ConfigPid::values.velocity_I = 0.0;
  ConfigPid::values.balance_max_sps = 12000.0;

  struct Candidate {
    const char* name;
    double pitch;
    double rate;
    double accel;
  };
  const std::array<Candidate, 19> candidates = {{
      {"kp6000_kr250", 6000.0, 250.0, 0.0},
      {"kp6000_kr500", 6000.0, 500.0, 0.0},
      {"kp6000_kr750", 6000.0, 750.0, 0.0},
      {"kp6000_kr1000", 6000.0, 1000.0, 0.0},
      {"kp6000_kr1500", 6000.0, 1500.0, 0.0},
      {"kp8000_kr250", 8000.0, 250.0, 0.0},
      {"kp8000_kr500", 8000.0, 500.0, 0.0},
      {"kp8000_kr750", 8000.0, 750.0, 0.0},
      {"kp8000_kr1000", 8000.0, 1000.0, 0.0},
      {"kp8000_kr1500", 8000.0, 1500.0, 0.0},
      {"kp9600_kr500", 9600.0, 500.0, 0.0},
      {"kp9600_kr1000", 9600.0, 1000.0, 0.0},
      {"kp9600_kr1500", 9600.0, 1500.0, 0.0},
      {"kp12000_kr500", 12000.0, 500.0, 0.0},
      {"kp12000_kr1000", 12000.0, 1000.0, 0.0},
      {"current_008_027_equivalent", 6912.0, 256.0, 0.0},
      {"current_011_036_equivalent", 12672.0, 352.0, 0.0},
      {"historical_state_feedback", 9600.0, 1000.0, 32.0},
      {"current_014_035_equivalent", 15680.0, 448.0, 0.0},
  }};
  const std::array<PhysicsProfile, 3> profiles = {{
      PhysicsProfile::DirectActuator,
      PhysicsProfile::RetiredSimpleForce,
      PhysicsProfile::Realistic,
  }};

  for (const auto& candidate : candidates) {
    ConfigPid::values.pitch_gain = candidate.pitch;
    ConfigPid::values.pitch_rate_gain = candidate.rate;
    ConfigPid::values.pitch_accel_gain = candidate.accel;
    for (const auto profile : profiles) {
      SimulatorScenario scenario;
      scenario.name = candidate.name;
      scenario.initial_pitch_deg = 0.0;
      scenario.duration_s = 8.0;
      scenario.physics_profile = profile;
      scenario.disturbances.push_back(SimulatorDisturbance{
          .kind = SimulatorDisturbanceKind::Step,
          .start_s = 1.0,
          .duration_s = 0.10,
          .force_n = 0.25,
      });
      const auto result = run_simulator_scenario_with_loaded_pid(scenario);
      ASSERT_FALSE(result.rows.empty());
      const auto& last = result.rows.back();
      std::cout << "state_feedback " << candidate.name << ' '
                << BalancerSimulator::profile_name(profile) << " fell=" << result.fell
                << " max_pitch_deg=" << result.max_abs_pitch_deg
                << " tail_rms_deg=" << result.tail_rms_pitch_deg
                << " command_rms_sample=" << std::abs(last.u_sps)
                << " pitch_term_sps=" << last.pitch_feedback_sps
                << " rate_term_sps=" << last.pitch_rate_feedback_sps
                << " accel_term_sps=" << last.pitch_accel_feedback_sps << '\n';
      EXPECT_DOUBLE_EQ(last.active_pitch_gain_sps_per_rad, candidate.pitch);
      EXPECT_DOUBLE_EQ(last.active_pitch_rate_gain_sps_per_rad_s, candidate.rate);
    }
  }
}

TEST(SimulatorReferenceTest, DirectActuatorReferenceProfileIsExplicitlyDistinct) {
  const auto ideal = BalancerSimulator::physics_for_profile(PhysicsProfile::DirectActuator);
  const auto retired_simple =
      BalancerSimulator::physics_for_profile(PhysicsProfile::RetiredSimpleForce);
  const auto retired_no_slip =
      BalancerSimulator::physics_for_profile(PhysicsProfile::RetiredNoSlipActuator);
  const auto stress = BalancerSimulator::physics_for_profile(PhysicsProfile::ActuatorStress);
  const auto nominal = BalancerSimulator::physics_for_profile(PhysicsProfile::Realistic);

  EXPECT_TRUE(ideal.direct_force);
  EXPECT_TRUE(retired_simple.direct_force);
  EXPECT_FALSE(nominal.direct_force);
  EXPECT_FALSE(stress.direct_force);
  EXPECT_FALSE(retired_no_slip.direct_force);
  const auto stepper = BalancerSimulator::physics_for_profile(PhysicsProfile::StepperPhase);
  EXPECT_FALSE(stepper.direct_force);
  EXPECT_TRUE(stepper.stepper_phase);
  EXPECT_DOUBLE_EQ(stepper.max_force_n,
                   BalancerSimulator::HardwareNominal::stepper_combined_force_n);
  EXPECT_TRUE(retired_no_slip.stepper_phase);
  EXPECT_TRUE(retired_no_slip.stepper_phase_electrical);
  EXPECT_DOUBLE_EQ(ideal.motor_tau_s, 0.0);
  EXPECT_DOUBLE_EQ(nominal.motor_tau_s, 0.002);
  EXPECT_DOUBLE_EQ(stress.motor_tau_s, 0.020);
  EXPECT_GT(ideal.direct_force_per_sps, 0.0);
  EXPECT_DOUBLE_EQ(ideal.direct_force_per_sps, retired_simple.direct_force_per_sps);
  EXPECT_DOUBLE_EQ(retired_no_slip.stepper_current_limit_a,
                   BalancerSimulator::HardwareNominal::stepper_current_limit_a);
}

TEST(SimulatorReferenceTest, DirectActuatorKeepsIdealForceCompatibilityAlias) {
  EXPECT_EQ(static_cast<int>(PhysicsProfile::DirectActuator), 3);
  EXPECT_EQ(static_cast<int>(PhysicsProfile::IdealForce),
            static_cast<int>(PhysicsProfile::DirectActuator));
  EXPECT_EQ(BalancerSimulator::profile_name(PhysicsProfile::DirectActuator), "direct_actuator");
  EXPECT_EQ(BalancerSimulator::profile_name(PhysicsProfile::IdealForce), "direct_actuator");
  const auto direct = BalancerSimulator::physics_for_profile(PhysicsProfile::DirectActuator);
  const auto compatibility = BalancerSimulator::physics_for_profile(PhysicsProfile::IdealForce);
  EXPECT_EQ(direct.direct_force, compatibility.direct_force);
  EXPECT_DOUBLE_EQ(direct.direct_force_per_sps, compatibility.direct_force_per_sps);
}

TEST(SimulatorModelIdentificationTest, RetiredActuatorAliasIsReportedWithoutOldModel) {
  ScopedStateFeedbackConfig restore;
  ConfigPid::load(sim_pid_path());
  ConfigPid::values.velocity_damping_per_s = 0.0;
  ConfigPid::values.velocity_I = 0.0;
  ConfigPid::values.velocity_pitch_limit_deg = 0.0;
  ConfigPid::values.pitch_rate_gain = 350.0;
  ConfigPid::values.pitch_accel_gain = 0.0;

  const std::array<RecoveryInitialState, 7> states = {{
      {"small_release", 1.0, 0.0, 0.0},
      {"low_speed", 2.0, 0.0, 500.0},
      {"low_speed_negative", -2.0, 0.0, -500.0},
      {"state_a", 4.3, 0.0, 964.0},
      {"state_b", 9.0, 0.0, 1800.0},
      {"state_a_negative", -4.3, 0.0, -964.0},
      {"state_b_negative", -9.0, 0.0, -1800.0},
  }};
  const auto physics =
      BalancerSimulator::physics_for_profile(PhysicsProfile::RetiredNoSlipActuator);
  std::cout << "no_slip_parameters max_force_n=" << physics.max_force_n
            << " no_load_speed_mps=" << physics.no_load_speed_mps
            << " command_delay_s=" << physics.command_delay_s
            << " motor_tau_s=" << physics.motor_tau_s
            << " force_per_sps=" << physics.direct_force_per_sps << '\n';

  struct NoSlipCandidate {
    const char* name;
    double force_per_sps;
    double tau_s;
    double delay_s;
    double no_load_speed_mps;
    bool speed_dependent_force_limit;
  };
  for (const auto& candidate : std::array<NoSlipCandidate, 13>{
           {{"direct40_fast", 40.0 * Config::meters_per_step, 0.002, 0.005, 1.2, true},
            {"direct40_nominal", 40.0 * Config::meters_per_step, 0.008, 0.0075, 1.2, true},
            {"direct40_slow", 40.0 * Config::meters_per_step, 0.012, 0.0075, 1.2, true},
            {"direct80_nominal", 80.0 * Config::meters_per_step, 0.008, 0.0075, 1.2, true},
            {"direct120_nominal", 120.0 * Config::meters_per_step, 0.008, 0.0075, 1.2, true},
            {"direct40_low_speed", 40.0 * Config::meters_per_step, 0.008, 0.0075, 0.5, true},
            {"direct40_high_speed", 40.0 * Config::meters_per_step, 0.008, 0.0075, 2.0, true},
            {"direct40_short_delay", 40.0 * Config::meters_per_step, 0.008, 0.002, 1.2, true},
            {"direct40_long_delay", 40.0 * Config::meters_per_step, 0.008, 0.015, 1.2, true},
            {"direct40_fast_high_speed", 40.0 * Config::meters_per_step, 0.002, 0.005, 2.0, true},
            {"direct60_fast_high_speed", 60.0 * Config::meters_per_step, 0.002, 0.005, 2.0, true},
            {"direct80_fast_high_speed", 80.0 * Config::meters_per_step, 0.002, 0.005, 2.0, true},
            {"direct40_fast_no_speed_limit", 40.0 * Config::meters_per_step, 0.002, 0.005, 2.0, false}}}) {
    auto candidate_physics = physics;
    candidate_physics.force_from_velocity_error = false;
    candidate_physics.direct_force_per_sps = candidate.force_per_sps;
    candidate_physics.motor_tau_s = candidate.tau_s;
    candidate_physics.command_delay_s = candidate.delay_s;
    candidate_physics.no_load_speed_mps = candidate.no_load_speed_mps;
    candidate_physics.speed_dependent_force_limit = candidate.speed_dependent_force_limit;
    for (const double pitch_gain : {6000.0, 8000.0}) {
      ConfigPid::values.pitch_gain = pitch_gain;
      std::cout << "no_slip_candidate name=" << candidate.name << " kp=" << pitch_gain;
      for (const auto& initial : std::array<RecoveryInitialState, 3>{
               {{"small", 1.0, 0.0, 0.0}, {"state_a", 4.3, 0.0, 964.0},
                {"state_b", 9.0, 0.0, 1800.0}}}) {
        auto candidate_scenario = attitude_recovery_scenario(PhysicsProfile::RetiredNoSlipActuator,
                                                              initial);
        candidate_scenario.physics_override = candidate_physics;
        const auto candidate_result =
            run_simulator_scenario_with_loaded_pid(candidate_scenario);
        std::cout << ' ' << initial.name << "=" << (candidate_result.fell ? "fall" : "hold")
                  << ":peak=" << candidate_result.max_abs_pitch_deg << ":v="
                  << candidate_result.rows.back().plant_velocity / Config::meters_per_step;
      }
      std::cout << '\n';
    }
  }

  {
    auto reference = attitude_recovery_scenario(PhysicsProfile::DirectActuator, states[0]);
    reference.physics_override = BalancerSimulator::physics_for_profile(PhysicsProfile::DirectActuator);
    reference.physics_override->cart_damping = 1.0;
    ConfigPid::values.pitch_gain = 6000.0;
    const auto result = run_simulator_scenario_with_loaded_pid(reference);
    std::cout << "no_slip_control_reference direct_actuator_cart1 fell=" << result.fell
              << " peak_pitch_deg=" << result.max_abs_pitch_deg << '\n';
  }

  for (const double pitch_gain : {6000.0, 8000.0}) {
    ConfigPid::values.pitch_gain = pitch_gain;
    for (const auto& initial : states) {
      const auto result = run_simulator_scenario_with_loaded_pid(
          attitude_recovery_scenario(PhysicsProfile::RetiredNoSlipActuator, initial));
      ASSERT_FALSE(result.rows.empty());
      if (pitch_gain == 6000.0 && std::string(initial.name) == "small_release") {
        for (const double sample_s : {0.0, 0.05, 0.10, 0.20, 0.40, 0.80}) {
          auto it = std::lower_bound(
              result.rows.begin(), result.rows.end(), sample_s,
              [](const auto& row, double time_s) { return row.sim_time_s < time_s; });
          if (it == result.rows.end()) it = std::prev(result.rows.end());
          std::cout << "no_slip_trace t=" << it->sim_time_s
                    << " pitch=" << it->plant_pitch_deg << " rate=" << it->plant_pitch_rate_dps
                    << " v_sps=" << it->plant_velocity / Config::meters_per_step
                    << " u=" << it->u_sps << " f=" << it->f_app
                    << " force_limit=" << it->motor_force_limit_n << '\n';
        }
      }
      if (std::string(initial.name) == "state_a") {
        for (const double sample_s : {0.0, 0.10, 0.20, 0.40, 0.80, 1.20}) {
          auto it = std::lower_bound(
              result.rows.begin(), result.rows.end(), sample_s,
              [](const auto& row, double time_s) { return row.sim_time_s < time_s; });
          if (it == result.rows.end()) it = std::prev(result.rows.end());
          std::cout << "no_slip_state_a_trace kp=" << pitch_gain << " t=" << it->sim_time_s
                    << " pitch_deg=" << it->plant_pitch_deg
                    << " rate_dps=" << it->plant_pitch_rate_dps
                    << " velocity_sps=" << it->plant_velocity / Config::meters_per_step
                    << " command_sps=" << it->u_sps << " applied_force_n=" << it->f_app
                    << " force_limit_n=" << it->motor_force_limit_n
                    << " wheel_accel_mps2=" << it->x_ddot << '\n';
        }
      }
      const double peak_rate = signal_peak_in_window(
          result, 0.0, result.rows.back().sim_time_s + 1e-9,
          [](const auto& row) { return row.plant_pitch_rate_dps; });
      const double peak_force = signal_peak_in_window(
          result, 0.0, result.rows.back().sim_time_s + 1e-9,
          [](const auto& row) { return row.f_app; });
      const double minimum_speed_limit = [&] {
        double value = std::numeric_limits<double>::infinity();
        for (const auto& row : result.rows) value = std::min(value, row.motor_force_limit_n);
        return value;
      }();
      std::cout << "no_slip_recovery kp=" << pitch_gain << " state=" << initial.name
                << " fell=" << result.fell << " fault_flags=" << result.controller_fault_flags
                << " peak_pitch_deg=" << result.max_abs_pitch_deg
                << " peak_rate_dps=" << peak_rate << " peak_force_n=" << peak_force
                << " minimum_speed_force_limit_n=" << minimum_speed_limit
                << " final_velocity_sps="
                << result.rows.back().plant_velocity / Config::meters_per_step << '\n';
    }
  }

  ConfigPid::values.velocity_damping_per_s = 8.0;
  ConfigPid::values.velocity_pitch_limit_deg = 4.0;
  for (const double pitch_gain : {6000.0, 8000.0}) {
    ConfigPid::values.pitch_gain = pitch_gain;
    for (const auto& initial : states) {
      const auto result = run_simulator_scenario_with_loaded_pid(
          attitude_recovery_scenario(PhysicsProfile::RetiredNoSlipActuator, initial));
      ASSERT_FALSE(result.rows.empty());
      std::cout << "no_slip_outer_recovery kp=" << pitch_gain << " state=" << initial.name
                << " fell=" << result.fell << " fault_flags=" << result.controller_fault_flags
                << " peak_pitch_deg=" << result.max_abs_pitch_deg
                << " final_velocity_sps="
                << result.rows.back().plant_velocity / Config::meters_per_step << '\n';
    }
  }

  ConfigPid::values.velocity_damping_per_s = 2.0;
  ConfigPid::values.velocity_control_cutoff_hz = 0.5;
  ConfigPid::values.velocity_pitch_limit_deg = 2.0;
  for (const double pitch_gain : {6000.0, 8000.0}) {
    ConfigPid::values.pitch_gain = pitch_gain;
    for (const auto& initial : states) {
      const auto result = run_simulator_scenario_with_loaded_pid(
          attitude_recovery_scenario(PhysicsProfile::RetiredNoSlipActuator, initial));
      ASSERT_FALSE(result.rows.empty());
      std::cout << "no_slip_conservative_recovery kp=" << pitch_gain
                << " state=" << initial.name << " fell=" << result.fell
                << " fault_flags=" << result.controller_fault_flags
                << " peak_pitch_deg=" << result.max_abs_pitch_deg
                << " final_velocity_sps="
                << result.rows.back().plant_velocity / Config::meters_per_step << '\n';
    }
  }
}

TEST(SimulatorModelIdentificationTest, KnownAttitudeColdStartIsMeasuredSeparately) {
  ScopedStateFeedbackConfig restore;
  ConfigPid::load(sim_pid_path());
  ConfigPid::values.velocity_damping_per_s = 0.0;
  ConfigPid::values.velocity_I = 0.0;
  ConfigPid::values.velocity_pitch_limit_deg = 0.0;
  ConfigPid::values.pitch_rate_gain = 350.0;
  ConfigPid::values.pitch_accel_gain = 0.0;

  for (const double pitch_gain : {6000.0, 8000.0}) {
    ConfigPid::values.pitch_gain = pitch_gain;
    auto estimator_limited = attitude_recovery_scenario(
        PhysicsProfile::DirectActuator, RecoveryInitialState{"cold_start_50deg_estimator_limited", 50.0,
                                                          0.0, 0.0});
    const auto limited_result = run_simulator_scenario_with_loaded_pid(estimator_limited);
    ASSERT_FALSE(limited_result.rows.empty());
    std::cout << "cold_start_split kp=" << pitch_gain
              << " estimator_limited_fell=" << limited_result.fell
              << " estimator_limited_peak_pitch_deg=" << limited_result.max_abs_pitch_deg;
    for (const double known_pitch_deg : {45.0, 50.0}) {
      auto known_attitude = estimator_limited;
      known_attitude.name = "recovery_known_attitude";
      known_attitude.initial_pitch_deg = known_pitch_deg;
      known_attitude.initial_fused_pitch_deg = known_pitch_deg;
      const auto known_result = run_simulator_scenario_with_loaded_pid(known_attitude);
      ASSERT_FALSE(known_result.rows.empty());
      std::cout << " known_pitch_deg=" << known_pitch_deg
                << ":fell=" << known_result.fell
                << ":fault_flags=" << known_result.controller_fault_flags
                << ":peak_pitch_deg=" << known_result.max_abs_pitch_deg
                << ":return_s=" << first_time_inside_pitch_band(known_result, 1.0)
                << ":fused_initial=" << known_result.rows.front().fused_pitch_deg;
    }
    std::cout << '\n';
    // The normal controller safety envelope disarms a neutral-command run
    // above the configured fallover limit (25 deg). This is intentionally a
    // measurement, not a pass/fail claim about 45/50 deg actuator authority.
    EXPECT_TRUE(limited_result.fell);
  }
}

TEST(SimulatorModelIdentificationTest, EstimatorLimitedColdStartBoundaryIsMeasured) {
  ScopedStateFeedbackConfig restore;
  ConfigPid::load(sim_pid_path());
  ConfigPid::values.velocity_damping_per_s = 0.0;
  ConfigPid::values.velocity_I = 0.0;
  ConfigPid::values.velocity_pitch_limit_deg = 0.0;
  ConfigPid::values.pitch_rate_gain = 350.0;
  ConfigPid::values.pitch_accel_gain = 0.0;

  for (const double pitch_gain : {6000.0, 8000.0}) {
    ConfigPid::values.pitch_gain = pitch_gain;
    for (const double initial_pitch_deg : {10.0, 15.0, 18.0, 20.0, 22.0, 24.0, 25.0,
                                           30.0, 35.0, 40.0, 45.0, 50.0}) {
      RecoveryInitialState initial{"estimator_limited", initial_pitch_deg, 0.0, 0.0};
      const auto result = run_simulator_scenario_with_loaded_pid(
          attitude_recovery_scenario(PhysicsProfile::DirectActuator, initial));
      ASSERT_FALSE(result.rows.empty());
      double estimate_convergence_s = std::numeric_limits<double>::infinity();
      for (const auto& row : result.rows) {
        if (std::abs(row.fused_pitch_deg - row.plant_pitch_deg) <= 5.0) {
          estimate_convergence_s = row.sim_time_s;
          break;
        }
      }
      std::cout << "cold_start_boundary kp=" << pitch_gain
                << " initial_pitch_deg=" << initial_pitch_deg
                << " fell=" << result.fell
                << " controller_fault_flags=" << result.controller_fault_flags
                << " max_actual_pitch_deg=" << result.max_abs_pitch_deg
                << " max_rate_dps="
                << signal_peak_in_window(
                       result, 0.0, result.rows.back().sim_time_s + 1e-9,
                       [](const auto& row) { return row.plant_pitch_rate_dps; })
                << " estimate_within5deg_s=" << estimate_convergence_s
                << " max_command_sps="
                << signal_peak_in_window(result, 0.0, result.rows.back().sim_time_s + 1e-9,
                                         [](const auto& row) { return row.u_sps; })
                << " max_applied_force_n="
                << signal_peak_in_window(result, 0.0, result.rows.back().sim_time_s + 1e-9,
                                         [](const auto& row) { return row.f_app; })
                << " margin_to70_deg=" << 70.0 - result.max_abs_pitch_deg << '\n';
    }
  }
}

TEST(SimulatorModelIdentificationTest, RetiredNoSlipAliasDoesNotReintroduceOuterModel) {
  ScopedStateFeedbackConfig restore;
  ConfigPid::load(sim_pid_path());
  ConfigPid::values.pitch_rate_gain = 350.0;
  ConfigPid::values.pitch_accel_gain = 0.0;
  ConfigPid::values.velocity_I = 0.001;

  struct OuterCandidate {
    const char* name;
    double cutoff_hz;
    double damping_per_s;
    double pitch_limit_deg;
  };
  const std::array<OuterCandidate, 2> candidates = {{
      {"current_3hz_8_per_s_4deg", 3.0, 8.0, 4.0},
      {"conservative_0p5hz_2_per_s_2deg", 0.5, 2.0, 2.0},
  }};
  for (const auto& candidate : candidates) {
    ConfigPid::values.velocity_control_cutoff_hz = candidate.cutoff_hz;
    ConfigPid::values.velocity_damping_per_s = candidate.damping_per_s;
    ConfigPid::values.velocity_pitch_limit_deg = candidate.pitch_limit_deg;
    for (const double pitch_gain : {6000.0, 8000.0}) {
      ConfigPid::values.pitch_gain = pitch_gain;
      for (const double initial_velocity_sps : {0.0, 250.0, 500.0, 1000.0}) {
        const auto result = run_simulator_scenario_with_loaded_pid(
            velocity_reference_scenario(PhysicsProfile::RetiredNoSlipActuator,
                                         initial_velocity_sps * Config::meters_per_step));
        ASSERT_FALSE(result.rows.empty());
        const double peak_velocity = signal_peak_in_window(
            result, 0.0, result.rows.back().sim_time_s + 1e-9,
            [](const auto& row) { return row.plant_velocity / Config::meters_per_step; });
        const double peak_pitch = signal_peak_in_window(
            result, 0.0, result.rows.back().sim_time_s + 1e-9,
            [](const auto& row) { return row.plant_pitch_deg; });
        const double peak_rate = signal_peak_in_window(
            result, 0.0, result.rows.back().sim_time_s + 1e-9,
            [](const auto& row) { return row.plant_pitch_rate_dps; });
        const double authority_time = signal_rms_in_window(
            result, 0.0, result.rows.back().sim_time_s + 1e-9,
            [](const auto& row) { return row.velocity_authority_limited; });
        std::cout << "no_slip_outer_candidate name=" << candidate.name
                  << " kp=" << pitch_gain << " initial_velocity_sps=" << initial_velocity_sps
                  << " fell=" << result.fell << " peak_velocity_sps=" << peak_velocity
                  << " peak_pitch_deg=" << peak_pitch << " peak_rate_dps=" << peak_rate
                  << " final_velocity_sps="
                  << result.rows.back().plant_velocity / Config::meters_per_step
                  << " final_trim_deg=" << result.rows.back().com_trim_deg
                  << " authority_time_fraction=" << authority_time
                  << " recovery_margin=" << recovery_margin(result) << '\n';
      }
    }
  }
}

TEST(SimulatorModelIdentificationTest, ComAcquisitionEnvelopeUsesMaintainedProfilesAndAlias) {
  ScopedStateFeedbackConfig restore;
  ConfigPid::load(sim_pid_path());
  ConfigPid::values.pitch_gain = 8000.0;
  ConfigPid::values.pitch_rate_gain = 350.0;
  ConfigPid::values.pitch_accel_gain = 0.0;
  ConfigPid::values.velocity_control_cutoff_hz = 0.5;
  ConfigPid::values.velocity_damping_per_s = 2.0;
  ConfigPid::values.velocity_pitch_limit_deg = 2.0;
  ConfigPid::values.velocity_I = 0.001;

  for (const auto profile : {PhysicsProfile::DirectActuator,
                             PhysicsProfile::RetiredNoSlipActuator}) {
    for (const double offset_rad : {0.002, 0.004, 0.006, 0.008, 0.010, 0.012, 0.015, 0.020,
                                    -0.002, -0.004, -0.006, -0.008, -0.010, -0.012, -0.015,
                                    -0.020}) {
      SimulatorScenario scenario;
      scenario.name = "com_acquisition_envelope";
      scenario.duration_s = 30.0;
      scenario.physics_profile = profile;
      scenario.com_angle_offset_rad = offset_rad;
      const auto result = run_simulator_scenario_with_loaded_pid(scenario);
      ASSERT_FALSE(result.rows.empty());
      double trust_time_s = std::numeric_limits<double>::infinity();
      double max_abs_velocity_sps = 0.0;
      double max_abs_pitch_deg = 0.0;
      uint32_t block_reason_mask = 0;
      for (const auto& row : result.rows) {
        if (row.trim_trusted > 0.5 && !std::isfinite(trust_time_s)) trust_time_s = row.sim_time_s;
        max_abs_velocity_sps =
            std::max(max_abs_velocity_sps,
                     std::abs(row.plant_velocity / Config::meters_per_step));
        max_abs_pitch_deg = std::max(max_abs_pitch_deg, std::abs(row.plant_pitch_deg));
        if (row.trim_learning_block_reason < 32) {
          block_reason_mask |= 1U << row.trim_learning_block_reason;
        }
      }
      std::cout << "com_acquisition profile=" << BalancerSimulator::profile_name(profile)
                << " offset_mrad=" << offset_rad * 1000.0
                << " trusted=" << std::isfinite(trust_time_s)
                << " trust_time_s=" << trust_time_s
                << " final_trim_deg=" << result.rows.back().com_trim_deg
                << " final_trim_error_deg="
                << (result.rows.back().com_trim_deg + offset_rad * 180.0 / M_PI)
                << " max_pitch_deg=" << max_abs_pitch_deg
                << " max_velocity_sps=" << max_abs_velocity_sps
                << " block_reason_mask=" << block_reason_mask << '\n';
    }
  }
}

TEST(SimulatorReferenceTest, VelocityControlCutoffIsSeparateFromObserverAndObservable) {
  ScopedStateFeedbackConfig restore;
  ConfigPid::load(sim_pid_path());
  ConfigPid::values.pitch_gain = 8000.0;
  ConfigPid::values.pitch_rate_gain = 500.0;
  ConfigPid::values.pitch_accel_gain = 0.0;
  ConfigPid::values.velocity_damping_per_s = 13.0;
  ConfigPid::values.velocity_I = 0.0;
  ConfigPid::values.balance_max_sps = 12000.0;

  for (const double cutoff_hz : {10.0, 5.0, 3.0, 2.0, 1.0}) {
    ConfigPid::values.velocity_control_cutoff_hz = cutoff_hz;
    SimulatorScenario scenario;
    scenario.name = "velocity_cutoff";
    scenario.duration_s = 8.0;
    scenario.physics_profile = PhysicsProfile::DirectActuator;
    scenario.joy_segments = {
        SimulatorJoySegment{.start_s = 1.0, .duration_s = 2.0, .forward = 0.35},
        SimulatorJoySegment{.start_s = 4.0, .duration_s = 2.0, .forward = -0.35},
    };
    const auto result = run_simulator_scenario_with_loaded_pid(scenario);
    ASSERT_FALSE(result.rows.empty());
    double peak_velocity_target_deg = 0.0;
    double peak_command_sps = 0.0;
    for (const auto& row : result.rows) {
      peak_velocity_target_deg =
          std::max(peak_velocity_target_deg, std::abs(row.velocity_pitch_target_deg));
      peak_command_sps = std::max(peak_command_sps, std::abs(row.u_sps));
    }
    const auto& last = result.rows.back();
    std::cout << "velocity_control cutoff_hz=" << cutoff_hz
              << " observer_hz=" << last.active_velocity_observer_cutoff_hz
              << " fell=" << result.fell << " tail_rms_deg=" << result.tail_rms_pitch_deg
              << " peak_velocity_pitch_target_deg=" << peak_velocity_target_deg
              << " peak_command_sps=" << peak_command_sps << '\n';
    EXPECT_DOUBLE_EQ(last.active_velocity_control_cutoff_hz, cutoff_hz);
    EXPECT_DOUBLE_EQ(last.active_velocity_observer_cutoff_hz, Config::fc_velocity_hz);
  }
}

TEST(SimulatorReferenceTest, AccelerationFeedbackReferenceSweepIsObservable) {
  ScopedStateFeedbackConfig restore;
  ConfigPid::load(sim_pid_path());
  ConfigPid::values.pitch_gain = 6000.0;
  ConfigPid::values.pitch_rate_gain = 500.0;
  ConfigPid::values.velocity_damping_per_s = 0.0;
  ConfigPid::values.velocity_I = 0.0;
  ConfigPid::values.balance_max_sps = 12000.0;

  for (const double accel_gain : {0.0, 8.0, 16.0, 32.0}) {
    ConfigPid::values.pitch_accel_gain = accel_gain;
    SimulatorScenario scenario;
    scenario.name = "acceleration_feedback";
    scenario.duration_s = 8.0;
    scenario.physics_profile = PhysicsProfile::DirectActuator;
    scenario.disturbances.push_back(SimulatorDisturbance{
        .kind = SimulatorDisturbanceKind::Step,
        .start_s = 1.0,
        .duration_s = 0.10,
        .force_n = 0.25,
    });
    const auto result = run_simulator_scenario_with_loaded_pid(scenario);
    ASSERT_FALSE(result.rows.empty());
    double peak_command_sps = 0.0;
    for (const auto& row : result.rows) {
      peak_command_sps = std::max(peak_command_sps, std::abs(row.u_sps));
    }
    const auto& last = result.rows.back();
    std::cout << "acceleration_feedback gain=" << accel_gain << " fell=" << result.fell
              << " tail_rms_deg=" << result.tail_rms_pitch_deg
              << " peak_command_sps=" << peak_command_sps << '\n';
    EXPECT_DOUBLE_EQ(last.active_pitch_accel_gain_sps_per_rad_s2, accel_gain);
  }
}

TEST(SimulatorReferenceTest, GyroDisturbanceSensitivityIsMeasuredAroundHardwareBand) {
  ScopedStateFeedbackConfig restore;
  ConfigPid::load(sim_pid_path());
  ConfigPid::values.velocity_damping_per_s = 0.0;
  ConfigPid::values.velocity_I = 0.0;
  ConfigPid::values.balance_max_sps = 12000.0;

  struct Candidate {
    const char* name;
    double pitch;
    double rate;
    double accel;
  };
  const std::array<Candidate, 6> candidates = {{
      {"new_8000_500", 8000.0, 500.0, 0.0},
      {"broad_7000_500", 7000.0, 500.0, 0.0},
      {"low_6000_500", 6000.0, 500.0, 0.0},
      {"current_014_035", 15680.0, 448.0, 0.0},
      {"historical_no_accel", 9600.0, 1000.0, 0.0},
      {"historical_accel", 9600.0, 1000.0, 32.0},
  }};
  for (const auto& candidate : candidates) {
    ConfigPid::values.pitch_gain = candidate.pitch;
    ConfigPid::values.pitch_rate_gain = candidate.rate;
    ConfigPid::values.pitch_accel_gain = candidate.accel;
    for (const double frequency_hz : {7.8, 8.0, 8.4, 8.8}) {
      SimulatorScenario scenario;
      scenario.name = candidate.name;
      scenario.duration_s = 3.0;
      scenario.physics_profile = PhysicsProfile::DirectActuator;
      scenario.gyro_pitch_disturbance_frequency_hz = frequency_hz;
      scenario.gyro_pitch_disturbance_amplitude_rad_s = 0.10;
      const auto result = run_simulator_scenario_with_loaded_pid(scenario);
      ASSERT_FALSE(result.rows.empty());
      double command_squared = 0.0;
      size_t sample_count = 0;
      for (const auto& row : result.rows) {
        if (row.sim_time_s < 1.0) continue;
        command_squared += row.u_sps * row.u_sps;
        ++sample_count;
      }
      const double command_rms = sample_count > 0
                                     ? std::sqrt(command_squared / static_cast<double>(sample_count))
                                     : 0.0;
      const double pitch_rms = signal_rms_in_window(
          result, 1.0, 3.0, [](const auto& row) { return row.plant_pitch_deg; });
      const double filtered_rate_rms = signal_rms_in_window(
          result, 1.0, 3.0, [](const auto& row) { return row.filtered_pitch_rate_dps; });
      std::cout << "gyro_disturbance candidate=" << candidate.name
                << " frequency_hz=" << frequency_hz << " fell=" << result.fell
                << " command_rms_sps=" << command_rms
                << " command_sensitivity_sps_per_rad_s=" << command_rms / 0.10
                << " plant_pitch_rms_deg=" << pitch_rms
                << " filtered_rate_rms_dps=" << filtered_rate_rms
                << " tail_rms_deg=" << result.tail_rms_pitch_deg << '\n';
    }
  }
}

TEST(SimulatorReferenceTest, StateFeedbackPitchRateNeighborhoodIsMappedOnReferenceProfiles) {
  ScopedStateFeedbackConfig restore;
  ConfigPid::load(sim_pid_path());
  ConfigPid::values.velocity_damping_per_s = 0.0;
  ConfigPid::values.velocity_I = 0.0;
  ConfigPid::values.pitch_accel_gain = 0.0;

  const std::array<double, 5> pitch_gains = {6000.0, 7000.0, 8000.0, 9000.0, 10000.0};
  const std::array<double, 5> rate_gains = {400.0, 500.0, 600.0, 700.0, 800.0};
  const std::array<PhysicsProfile, 2> profiles = {PhysicsProfile::DirectActuator,
                                                   PhysicsProfile::RetiredSimpleForce};

  for (const PhysicsProfile profile : profiles) {
    for (const double pitch_gain : pitch_gains) {
      for (const double rate_gain : rate_gains) {
        ConfigPid::values.pitch_gain = pitch_gain;
        ConfigPid::values.pitch_rate_gain = rate_gain;
        std::cout << "pitch_rate_map profile=" << BalancerSimulator::profile_name(profile)
                  << " kp=" << pitch_gain << " kr=" << rate_gain;
        for (const double release_deg : {1.0, 2.0, 5.0}) {
          const auto release = attitude_reference_scenario(profile, release_deg);
          const auto result = run_simulator_scenario_with_loaded_pid(release);
          const auto metrics = measure_controller_scenario(result);
          std::cout << " release" << release_deg << "="
                    << classify_controller_metrics(metrics) << ":tail=" << metrics.tail_rms_deg
                    << ":settle=" << metrics.settling_s << ":rate=" << metrics.peak_rate_dps
                    << ":cmd=" << metrics.command_rms_sps << "/" << metrics.command_peak_sps
                    << ":sat=" << metrics.saturation_fraction << ":f="
                    << metrics.dominant_frequency_hz;
        }
        const auto push = run_simulator_scenario_with_loaded_pid(push_reference_scenario(profile));
        const auto push_metrics = measure_controller_scenario(push, 1.1);
        std::cout << " push=" << classify_controller_metrics(push_metrics)
                  << ":tail=" << push_metrics.tail_rms_deg << ":settle="
                  << push_metrics.settling_s << ":rate=" << push_metrics.peak_rate_dps
                  << ":cmd=" << push_metrics.command_rms_sps << "/" << push_metrics.command_peak_sps
                  << ":sat=" << push_metrics.saturation_fraction << ":f="
                  << push_metrics.dominant_frequency_hz;
        const auto command = run_simulator_scenario_with_loaded_pid(
            commanded_pitch_reference_scenario(profile));
        const auto command_metrics = measure_controller_scenario(command, 2.1);
        std::cout << " command=" << classify_controller_metrics(command_metrics)
                  << ":tail=" << command_metrics.tail_rms_deg << ":settle="
                  << command_metrics.settling_s << ":rate=" << command_metrics.peak_rate_dps
                  << ":cmd=" << command_metrics.command_rms_sps << "/"
                  << command_metrics.command_peak_sps << ":sat="
                  << command_metrics.saturation_fraction << ":f="
                  << command_metrics.dominant_frequency_hz << '\n';
      }
    }
  }
}

TEST(SimulatorReferenceTest, PitchGainRecoveryEnvelopeAndColdStartAreMapped) {
  ScopedStateFeedbackConfig restore;
  ConfigPid::load(sim_pid_path());
  // This is an attitude-only experiment.  Zeroing the outer gains keeps the
  // target at the fixed equilibrium without changing the checked-in config
  // or the locked attitude/notch dimensions.
  ConfigPid::values.pitch_rate_gain = 350.0;
  ConfigPid::values.pitch_accel_gain = 0.0;
  ConfigPid::values.velocity_damping_per_s = 0.0;
  ConfigPid::values.velocity_I = 0.0;

  const std::array<RecoveryInitialState, 20> states = {{
      {"pitch_p1", 1.0, 0.0, 0.0},
      {"pitch_n1", -1.0, 0.0, 0.0},
      {"pitch_p2", 2.0, 0.0, 0.0},
      {"pitch_n2", -2.0, 0.0, 0.0},
      {"pitch_p4", 4.0, 0.0, 0.0},
      {"pitch_n4", -4.0, 0.0, 0.0},
      {"pitch_p6", 6.0, 0.0, 0.0},
      {"pitch_n6", -6.0, 0.0, 0.0},
      {"rate_p20", 2.0, 20.0, 0.0},
      {"rate_n20", 2.0, -20.0, 0.0},
      {"rate_p40", 2.0, 40.0, 0.0},
      {"rate_n40", 2.0, -40.0, 0.0},
      {"velocity_p500", 2.0, 0.0, 500.0},
      {"velocity_n500", 2.0, 0.0, -500.0},
      {"velocity_p1000", 2.0, 0.0, 1000.0},
      {"velocity_n1000", 2.0, 0.0, -1000.0},
      {"aligned_pitch_rate", 4.0, 40.0, 0.0},
      {"opposed_pitch_rate", 4.0, -40.0, 0.0},
      {"aligned_pitch_velocity", 4.0, 0.0, 1000.0},
      {"opposed_pitch_velocity", 4.0, 0.0, -1000.0},
  }};
  const std::array<double, 4> pitch_gains = {6000.0, 8000.0, 10000.0, 12000.0};

  for (const auto profile : {PhysicsProfile::DirectActuator,
                             PhysicsProfile::RetiredSimpleForce}) {
    for (const double pitch_gain : pitch_gains) {
      ConfigPid::values.pitch_gain = pitch_gain;
      size_t recovered = 0;
      size_t safety_rejected = 0;
      double worst_peak_pitch = 0.0;
      double worst_peak_rate = 0.0;
      double worst_peak_command = 0.0;
      double worst_force_fraction = 0.0;
      double worst_return_s = 0.0;
      double worst_fast_activity = 0.0;
      double worst_final_velocity_sps = 0.0;
      for (const auto& initial : states) {
        const auto result = run_simulator_scenario_with_loaded_pid(
            attitude_recovery_scenario(profile, initial));
        ASSERT_FALSE(result.rows.empty());
        const double return_s = first_time_inside_pitch_band(result, 1.0);
        const double peak_rate = signal_peak_in_window(
            result, 0.0, result.rows.back().sim_time_s + 1e-9,
            [](const auto& row) { return row.plant_pitch_rate_dps; });
        const double peak_command = signal_peak_in_window(
            result, 0.0, result.rows.back().sim_time_s + 1e-9,
            [](const auto& row) { return row.u_sps; });
        const double fast_activity = band_rms_signal_in_window(
            result, 2.0, result.rows.back().sim_time_s + 1e-9, 20, 40,
            [](const auto& row) { return row.u_sps; });
        const double final_velocity_sps =
            result.rows.back().plant_velocity / Config::meters_per_step;
        if (!result.fell && result.controller_fault_flags == ControllerFaultNone &&
            std::isfinite(return_s)) {
          ++recovered;
        }
        if ((result.controller_fault_flags & ControllerFaultFallover) != 0U) {
          ++safety_rejected;
        }
        worst_peak_pitch = std::max(worst_peak_pitch, result.max_abs_pitch_deg);
        worst_peak_rate = std::max(worst_peak_rate, peak_rate);
        worst_peak_command = std::max(worst_peak_command, peak_command);
        worst_force_fraction = std::max(worst_force_fraction, peak_force_fraction(result));
        if (std::isfinite(return_s)) worst_return_s = std::max(worst_return_s, return_s);
        worst_fast_activity = std::max(worst_fast_activity, fast_activity);
        worst_final_velocity_sps =
            std::max(worst_final_velocity_sps, std::abs(final_velocity_sps));
        std::cout << "recovery_state profile=" << BalancerSimulator::profile_name(profile)
                  << " kp=" << pitch_gain << " state=" << initial.name
                  << " fell=" << result.fell << " peak_pitch_deg=" << result.max_abs_pitch_deg
                  << " controller_fault_flags=" << result.controller_fault_flags
                  << " peak_rate_dps=" << peak_rate << " peak_command_sps=" << peak_command
                  << " return_inside_1s=" << return_s << " final_velocity_sps="
                  << final_velocity_sps << " force_fraction=" << peak_force_fraction(result)
                  << " command_20_40_rms_sps=" << fast_activity << '\n';
        if (profile == PhysicsProfile::DirectActuator &&
            std::string(initial.name) == "pitch_p6") {
          std::cout << "recovery_terms profile=" << BalancerSimulator::profile_name(profile)
                    << " kp=" << pitch_gain << " state=" << initial.name
                    << " pitch_term_peak_sps="
                    << signal_peak_in_window(
                           result, 0.0, result.rows.back().sim_time_s + 1e-9,
                           [](const auto& row) { return row.pitch_feedback_sps; })
                    << " rate_term_peak_sps="
                    << signal_peak_in_window(
                           result, 0.0, result.rows.back().sim_time_s + 1e-9,
                           [](const auto& row) { return row.pitch_rate_feedback_sps; })
                    << " command_peak_sps=" << peak_command
                    << " applied_force_peak_n="
                    << signal_peak_in_window(
                           result, 0.0, result.rows.back().sim_time_s + 1e-9,
                           [](const auto& row) { return row.f_app; })
                    << " pitch_accel_peak_rad_s2="
                    << signal_peak_in_window(
                           result, 0.0, result.rows.back().sim_time_s + 1e-9,
                           [](const auto& row) { return row.theta_ddot; })
                    << " wheel_accel_peak_m_s2="
                    << signal_peak_in_window(
                           result, 0.0, result.rows.back().sim_time_s + 1e-9,
                           [](const auto& row) { return row.x_ddot; })
                    << '\n';
        }
      }
        std::cout << "recovery_envelope profile=" << BalancerSimulator::profile_name(profile)
                << " kp=" << pitch_gain << " recovered=" << recovered << "/" << states.size()
                << " safety_rejected=" << safety_rejected
                << " worst_peak_pitch_deg=" << worst_peak_pitch
                << " worst_peak_rate_dps=" << worst_peak_rate
                << " worst_peak_command_sps=" << worst_peak_command
                << " worst_return_inside_1s=" << worst_return_s
                << " worst_force_fraction=" << worst_force_fraction
                << " worst_command_20_40_rms_sps=" << worst_fast_activity
                << " worst_final_velocity_sps=" << worst_final_velocity_sps << '\n';

      SimulatorScenario neutral;
      neutral.name = "attitude_recovery_neutral";
      neutral.duration_s = 180.0;
      neutral.physics_profile = profile;
      const auto neutral_result = run_simulator_scenario_with_loaded_pid(neutral);
      ASSERT_FALSE(neutral_result.rows.empty());
      const double neutral_rate_rms = signal_rms_in_window(
          neutral_result, 120.0, 180.0, [](const auto& row) { return row.plant_pitch_rate_dps; });
      const double neutral_command_rms = signal_rms_in_window(
          neutral_result, 120.0, 180.0, [](const auto& row) { return row.u_sps; });
      const double neutral_command_fast = band_rms_signal_in_window(
          neutral_result, 120.0, 180.0, 20, 40, [](const auto& row) { return row.u_sps; });
      const double neutral_rate_fast = band_rms_signal_in_window(
          neutral_result, 120.0, 180.0, 20, 40,
          [](const auto& row) { return row.plant_pitch_rate_dps; });
      std::cout << "recovery_neutral profile=" << BalancerSimulator::profile_name(profile)
                << " kp=" << pitch_gain << " fell=" << neutral_result.fell
                << " pitch_rms_deg=" << neutral_result.tail_rms_pitch_deg
                << " rate_rms_dps=" << neutral_rate_rms
                << " command_rms_sps=" << neutral_command_rms
                << " command_20_40_rms_sps=" << neutral_command_fast
                << " rate_20_40_rms_dps=" << neutral_rate_fast << '\n';
    }

    for (const auto& initial : std::array<RecoveryInitialState, 4>{{
             {"cold_50deg", 50.0, 0.0, 0.0},
             {"cold_52deg", 52.0, 0.0, 0.0},
             {"hardware_state_a", 4.3, 0.0, 964.0},
             {"hardware_state_b", 9.0, 0.0, 1800.0},
         }}) {
      std::cout << "recovery_boundary profile=" << BalancerSimulator::profile_name(profile)
                << " state=" << initial.name;
      for (const double pitch_gain : pitch_gains) {
        ConfigPid::values.pitch_gain = pitch_gain;
        const auto result = run_simulator_scenario_with_loaded_pid(
            attitude_recovery_scenario(profile, initial));
        ASSERT_FALSE(result.rows.empty());
        const double return_s = first_time_inside_pitch_band(result, 1.0);
        std::cout << " kp=" << pitch_gain << ":fell=" << result.fell
                  << ":fault_flags=" << result.controller_fault_flags
                  << ":peak_pitch=" << result.max_abs_pitch_deg
                  << ":peak_rate="
                  << signal_peak_in_window(result, 0.0, result.rows.back().sim_time_s + 1e-9,
                                           [](const auto& row) { return row.plant_pitch_rate_dps; })
                  << ":peak_command="
                  << signal_peak_in_window(result, 0.0, result.rows.back().sim_time_s + 1e-9,
                                           [](const auto& row) { return row.u_sps; })
                  << ":return=" << return_s << ":final_velocity_sps="
                  << result.rows.back().plant_velocity / Config::meters_per_step;
        if (profile == PhysicsProfile::DirectActuator &&
            std::string(initial.name) == "cold_52deg") {
          const std::array<double, 8> trace_times_s = {
              0.0, 0.025, 0.05, 0.10, 0.20, 0.40, 0.80, 1.20};
          for (const double trace_time_s : trace_times_s) {
            auto row_it = std::lower_bound(
                result.rows.begin(), result.rows.end(), trace_time_s,
                [](const auto& row, double time_s) { return row.sim_time_s < time_s; });
            if (row_it == result.rows.end()) row_it = std::prev(result.rows.end());
            std::cout << "\ncold_start_trace state=" << initial.name << " kp=" << pitch_gain
                      << " t_s=" << row_it->sim_time_s
                      << " fused_pitch_deg=" << row_it->fused_pitch_deg
                      << " actual_pitch_deg=" << row_it->plant_pitch_deg
                      << " pitch_rate_dps=" << row_it->plant_pitch_rate_dps
                      << " pitch_error_deg=" << row_it->pitch_error_deg
                      << " pitch_term_sps=" << row_it->pitch_feedback_sps
                      << " rate_term_sps=" << row_it->pitch_rate_feedback_sps
                      << " command_sps=" << row_it->u_sps
                      << " applied_force_n=" << row_it->f_app
                      << " pitch_accel_rad_s2=" << row_it->theta_ddot
                      << " wheel_accel_m_s2=" << row_it->x_ddot
                      << " fall_margin_deg=" << 70.0 - std::abs(row_it->plant_pitch_deg);
          }
          std::cout << '\n';
        }
      }
      std::cout << '\n';
    }
  }
}

TEST(SimulatorReferenceTest, SlowSimpleVelocityBaselineIsMappedInsideRecoveryEnvelope) {
  ScopedStateFeedbackConfig restore;
  ConfigPid::load(sim_pid_path());
  // Use the lowest candidate that materially improved the ordinary recovery
  // envelope in the attitude-only experiment.  The outer matrix is isolated
  // from COM adaptation so it measures translation control, not bias learning.
  ConfigPid::values.pitch_gain = kOneThirtySecondPitchGain;
  ConfigPid::values.pitch_rate_gain = 350.0;
  ConfigPid::values.pitch_accel_gain = 0.0;
  ConfigPid::values.velocity_I = 0.0;
  ConfigPid::values.velocity_pitch_limit_deg = 2.0;

  for (const double cutoff_hz : {0.5, 1.0, 1.5}) {
    for (const double damping_per_s : {1.0, 2.0, 3.0, 4.0}) {
      ConfigPid::values.velocity_control_cutoff_hz = cutoff_hz;
      ConfigPid::values.velocity_damping_per_s = damping_per_s;
      for (const auto profile : {PhysicsProfile::DirectActuator,
                                 PhysicsProfile::RetiredSimpleForce}) {
        for (const double initial_velocity_sps : {0.0, 500.0, -500.0, 1000.0, -1000.0}) {
          const auto result = run_simulator_scenario_with_loaded_pid(
              velocity_reference_scenario(profile,
                                          initial_velocity_sps * Config::meters_per_step));
          ASSERT_FALSE(result.rows.empty());
          const double peak_velocity_sps = signal_peak_in_window(
              result, 0.0, result.rows.back().sim_time_s + 1e-9,
              [](const auto& row) { return row.plant_velocity / Config::meters_per_step; });
          const double peak_pitch = signal_peak_in_window(
              result, 0.0, result.rows.back().sim_time_s + 1e-9,
              [](const auto& row) { return row.plant_pitch_deg; });
          const double peak_rate = signal_peak_in_window(
              result, 0.0, result.rows.back().sim_time_s + 1e-9,
              [](const auto& row) { return row.plant_pitch_rate_dps; });
          const double command_rms = signal_rms_in_window(
              result, 0.0, result.rows.back().sim_time_s + 1e-9,
              [](const auto& row) { return row.u_sps; });
          const double pitch_target_peak = signal_peak_in_window(
              result, 0.0, result.rows.back().sim_time_s + 1e-9,
              [](const auto& row) { return row.velocity_pitch_request_limited_deg; });
          const double authority_fraction = signal_rms_in_window(
              result, 0.0, result.rows.back().sim_time_s + 1e-9,
              [](const auto& row) { return row.velocity_authority_limited; });
          const double margin = recovery_margin(result);
          std::cout << "velocity_simple profile=" << BalancerSimulator::profile_name(profile)
                    << " cutoff_hz=" << cutoff_hz << " damping_per_s=" << damping_per_s
                    << " initial_velocity_sps=" << initial_velocity_sps
                    << " fell=" << result.fell << " peak_velocity_sps=" << peak_velocity_sps
                    << " peak_pitch_deg=" << peak_pitch << " peak_rate_dps=" << peak_rate
                    << " final_velocity_sps="
                    << result.rows.back().plant_velocity / Config::meters_per_step
                    << " pitch_request_peak_deg=" << pitch_target_peak
                    << " authority_fraction=" << authority_fraction
                    << " command_rms_sps=" << command_rms << " recovery_margin=" << margin
                    << " final_position_m=" << std::abs(result.rows.back().plant_position)
                    << '\n';
          if (profile == PhysicsProfile::DirectActuator) {
            EXPECT_EQ(result.controller_fault_flags, 0U);
            EXPECT_EQ(result.actuator_fault_count, 0U);
            EXPECT_FALSE(result.fell);
          } else {
            // Retired model aliases are retained only as compatibility
            // diagnostics. Their failures are not acceptance criteria for the
            // slow baseline.
            EXPECT_FALSE(result.rows.empty());
          }
        }
      }

      const auto stop_result = run_simulator_scenario_with_loaded_pid(
          slow_stop_reference_scenario(PhysicsProfile::DirectActuator));
      ASSERT_FALSE(stop_result.rows.empty());
      const double stop_at_s =
          first_time_velocity_band_with_dwell(stop_result, 2.0, 50.0, 0.5);
      const double threshold_crossing_time_s =
          std::isfinite(stop_at_s) ? stop_at_s - 2.0 : std::numeric_limits<double>::infinity();
      const double threshold_crossing_distance_m =
          std::isfinite(stop_at_s)
              ? maximum_position_displacement(stop_result, 2.0, stop_at_s)
              : std::numeric_limits<double>::infinity();
      std::cout << "velocity_threshold_crossing profile=DirectActuator cutoff_hz=" << cutoff_hz
                << " damping_per_s=" << damping_per_s << " fell=" << stop_result.fell
                << " threshold_crossing_time_s=" << threshold_crossing_time_s
                << " threshold_crossing_distance_m=" << threshold_crossing_distance_m
                << " peak_velocity_sps="
                << signal_peak_in_window(
                       stop_result, 0.0, stop_result.rows.back().sim_time_s + 1e-9,
                       [](const auto& row) {
                         return row.plant_velocity / Config::meters_per_step;
                       })
                << " peak_pitch_deg="
                << signal_peak_in_window(
                       stop_result, 0.0, stop_result.rows.back().sim_time_s + 1e-9,
                       [](const auto& row) { return row.plant_pitch_deg; })
                << " peak_rate_dps="
                << signal_peak_in_window(
                       stop_result, 0.0, stop_result.rows.back().sim_time_s + 1e-9,
                       [](const auto& row) { return row.plant_pitch_rate_dps; })
                << " command_rms_sps="
                << signal_rms_in_window(
                       stop_result, 2.0, stop_result.rows.back().sim_time_s + 1e-9,
                       [](const auto& row) { return row.u_sps; })
                << '\n';
      if (cutoff_hz == 0.5 && damping_per_s == 2.0) {
        const auto release_it = std::lower_bound(
            stop_result.rows.begin(), stop_result.rows.end(), 2.0,
            [](const auto& row, double time_s) { return row.sim_time_s < time_s; });
        const auto following_it = release_it == stop_result.rows.end()
                                      ? std::prev(stop_result.rows.end())
                                      : release_it;
        const auto preceding_it = release_it == stop_result.rows.begin()
                                      ? release_it
                                      : std::prev(release_it);
        const double release_velocity_sps =
            std::abs(preceding_it->plant_velocity / Config::meters_per_step);
        const double following_velocity_sps =
            std::abs(following_it->plant_velocity / Config::meters_per_step);
        const double ideal_tau_s = 1.0 / damping_per_s;
        const double ideal_50_crossing_s =
            release_velocity_sps > 50.0
                ? ideal_tau_s * std::log(release_velocity_sps / 50.0)
                : 0.0;
        std::cout << "velocity_stop_metric_audit cutoff_hz=" << cutoff_hz
                  << " damping_per_s=" << damping_per_s
                  << " velocity_before_release_sps=" << release_velocity_sps
                  << " velocity_first_after_release_sps=" << following_velocity_sps
                  << " control_velocity_first_after_release_sps="
                  << following_it->velocity_control_sps
                  << " damping_accel_first_after_release_mps2="
                  << following_it->velocity_damping_acceleration_mps2
                  << " actual_accel_first_after_release_mps2=" << following_it->x_ddot
                  << " ideal_tau_s=" << ideal_tau_s
                  << " ideal_50_crossing_s=" << ideal_50_crossing_s
                  << " measured_metric_threshold_s=" << threshold_crossing_time_s
                  << " metric_definition=abs_velocity<=50_sps_then_abs_velocity<=100_sps_for_0.5s\n";
      }
      EXPECT_FALSE(stop_result.fell);
    }
  }
}

TEST(SimulatorReferenceTest, IsolatedVelocitySignIsSymmetricAndDamped) {
  ScopedStateFeedbackConfig restore;
  ConfigPid::load(sim_pid_path());
  ConfigPid::values.pitch_gain = kOneThirtySecondPitchGain;
  ConfigPid::values.pitch_rate_gain = 350.0;
  ConfigPid::values.pitch_accel_gain = 0.0;
  ConfigPid::values.velocity_control_cutoff_hz = 3.0;
  ConfigPid::values.velocity_damping_per_s = 8.0;
  ConfigPid::values.velocity_I = 0.0;

  const auto positive = run_simulator_scenario_with_loaded_pid(
      isolated_velocity_sign_scenario(PhysicsProfile::DirectActuator, 0.15));
  const auto negative = run_simulator_scenario_with_loaded_pid(
      isolated_velocity_sign_scenario(PhysicsProfile::DirectActuator, -0.15));
  ASSERT_FALSE(positive.rows.empty());
  ASSERT_FALSE(negative.rows.empty());
  const double positive_final = positive.rows.back().plant_velocity;
  const double negative_final = negative.rows.back().plant_velocity;
  std::cout << "isolated_velocity_sign positive_final_mps=" << positive_final
            << " negative_final_mps=" << negative_final
            << " positive_fell=" << positive.fell << " negative_fell=" << negative.fell
            << '\n';
  EXPECT_FALSE(positive.fell);
  EXPECT_FALSE(negative.fell);
  EXPECT_LT(std::abs(positive_final), 0.15);
  EXPECT_LT(std::abs(negative_final), 0.15);
  EXPECT_NEAR(positive_final, -negative_final, 0.02);
}

TEST(SimulatorReferenceTest, LargerStaticComOffsetsAcquireWithoutDynamicTrimDrift) {
  ScopedStateFeedbackConfig restore;
  ConfigPid::load(sim_pid_path());
  ConfigPid::values.pitch_gain = kOneThirtySecondPitchGain;
  ConfigPid::values.pitch_rate_gain = 350.0;
  ConfigPid::values.pitch_accel_gain = 0.0;
  ConfigPid::values.velocity_control_cutoff_hz = 3.0;
  ConfigPid::values.velocity_damping_per_s = 8.0;
  ConfigPid::values.velocity_I = 0.001;

  for (const double offset_rad : {0.004, -0.004, 0.008, -0.008}) {
    SimulatorScenario scenario;
    scenario.name = "larger_static_com_offset";
    scenario.duration_s = 120.0;
    scenario.physics_profile = PhysicsProfile::DirectActuator;
    scenario.com_angle_offset_rad = offset_rad;
    const auto result = run_simulator_scenario_with_loaded_pid(scenario);
    ASSERT_FALSE(result.rows.empty());
    bool saw_trim_learning = false;
    double max_abs_position = 0.0;
    for (const auto& row : result.rows) {
      saw_trim_learning = saw_trim_learning || row.trim_learning_enabled > 0.5;
      max_abs_position = std::max(max_abs_position, std::abs(row.plant_position));
    }
    std::cout << "static_com_offset offset_rad=" << offset_rad << " fell=" << result.fell
              << " final_trim_deg=" << result.rows.back().com_trim_deg
              << " max_pitch_deg=" << result.max_abs_pitch_deg
              << " max_abs_position_m=" << max_abs_position
              << " saw_trim_learning=" << saw_trim_learning << '\n';
    EXPECT_FALSE(result.fell);
    EXPECT_TRUE(saw_trim_learning);
    EXPECT_LT(offset_rad * result.rows.back().com_trim_deg, 0.0);
    EXPECT_LT(max_abs_position, 3.0);
  }
}

TEST(SimulatorReferenceTest, EquilibriumAngleComEstimatorReplayCoversStaticAndChangedBias) {
  ScopedStateFeedbackConfig restore;
  ConfigPid::load(sim_pid_path());
  ConfigPid::values.pitch_gain = 10000.0;
  ConfigPid::values.pitch_rate_gain = 350.0;
  ConfigPid::values.pitch_accel_gain = 0.0;
  ConfigPid::values.velocity_control_cutoff_hz = 3.0;
  ConfigPid::values.velocity_damping_per_s = 8.0;
  ConfigPid::values.velocity_pitch_limit_deg = 4.0;
  ConfigPid::values.velocity_I = 0.001;

  for (const double offset_rad : {0.002, -0.002, 0.004, -0.004, 0.008, -0.008,
                                  0.020, -0.020}) {
    SimulatorScenario scenario;
    scenario.name = "equilibrium_angle_com_replay";
    scenario.duration_s = 180.0;
    scenario.physics_profile = PhysicsProfile::DirectActuator;
    scenario.com_angle_offset_rad = offset_rad;
    const auto result = run_simulator_scenario_with_loaded_pid(scenario);
    ASSERT_FALSE(result.rows.empty());
    const auto replay = replay_equilibrium_angle(result);
    const auto trust = std::find_if(
        result.rows.begin(), result.rows.end(),
        [](const auto& row) { return row.trim_trusted > 0.5; });
    const double expected_trim_deg = -offset_rad * 180.0 / M_PI;
    const double final_trim_deg = result.rows.back().com_trim_deg;
    std::cout << "equilibrium_com_static offset_rad=" << offset_rad
              << " fell=" << result.fell << " trust_time_s="
              << (trust == result.rows.end() ? std::numeric_limits<double>::infinity()
                                               : trust->sim_time_s)
              << " final_trim_deg=" << final_trim_deg
              << " expected_trim_deg=" << expected_trim_deg
              << " trim_error_deg=" << std::abs(final_trim_deg - expected_trim_deg)
              << " q_eq_raw_deg=" << replay.raw_deg
              << " q_eq_candidate_deg=" << replay.candidate_deg
              << " q_eq_hat_deg=" << replay.estimate_deg
              << " prototype_trim_deg=" << replay.prototype_trim_deg
              << " max_q_eq_gap_deg=" << replay.max_candidate_estimate_gap_deg
              << " quiet_time_s=" << replay.quiet_time_s
              << " max_position_m=" << std::abs(result.rows.back().plant_position) << '\n';
    EXPECT_FALSE(result.fell);
    EXPECT_EQ(result.controller_fault_flags, 0U);
    EXPECT_EQ(result.actuator_fault_count, 0U);
    EXPECT_TRUE(std::any_of(result.rows.begin(), result.rows.end(), [](const auto& row) {
      return row.trim_learning_enabled > 0.5;
    }));
    EXPECT_LT(offset_rad * final_trim_deg, 0.0);
  }

  SimulatorScenario changed;
  changed.name = "equilibrium_angle_com_replay_changed_bias";
  changed.duration_s = 180.0;
  changed.physics_profile = PhysicsProfile::DirectActuator;
  changed.com_angle_offset_rad = 0.004;
  changed.disturbances.push_back(SimulatorDisturbance{
      .kind = SimulatorDisturbanceKind::HoldBias,
      .start_s = 100.0,
      .duration_s = 60.0,
      .com_bias_rad = 0.004,
  });
  const auto changed_result = run_simulator_scenario_with_loaded_pid(changed);
  ASSERT_FALSE(changed_result.rows.empty());
  const auto changed_replay = replay_equilibrium_angle(changed_result);
  std::cout << "equilibrium_com_changed fell=" << changed_result.fell
            << " trim_before_change="
            << changed_result.rows[39999].com_trim_deg
            << " final_trim_deg=" << changed_result.rows.back().com_trim_deg
            << " q_eq_hat_deg=" << changed_replay.estimate_deg
            << " prototype_trim_deg=" << changed_replay.prototype_trim_deg
            << " max_position_m=" << std::abs(changed_result.rows.back().plant_position)
            << '\n';
  EXPECT_FALSE(changed_result.fell);
}

TEST(SimulatorReferenceTest, StateFeedbackControllerShapesAreComparedAcrossReferenceProfiles) {
  ScopedStateFeedbackConfig restore;
  ConfigPid::load(sim_pid_path());
  ConfigPid::values.velocity_damping_per_s = 0.0;
  ConfigPid::values.velocity_I = 0.0;

  struct Candidate {
    const char* name;
    double pitch;
    double rate;
    double accel;
  };
  const std::array<Candidate, 4> candidates = {{
      {"new_8000_500_0", 8000.0, 500.0, 0.0},
      {"historical_9600_1000_32", 9600.0, 1000.0, 32.0},
      {"current_15680_448_0", 15680.0, 448.0, 0.0},
      {"historical_9600_1000_0", 9600.0, 1000.0, 0.0},
  }};
  const std::array<PhysicsProfile, 2> profiles = {PhysicsProfile::DirectActuator,
                                                   PhysicsProfile::RetiredSimpleForce};
  for (const auto& candidate : candidates) {
    ConfigPid::values.pitch_gain = candidate.pitch;
    ConfigPid::values.pitch_rate_gain = candidate.rate;
    ConfigPid::values.pitch_accel_gain = candidate.accel;
    for (const PhysicsProfile profile : profiles) {
      const auto release = run_simulator_scenario_with_loaded_pid(
          attitude_reference_scenario(profile, 2.0));
      const auto push = run_simulator_scenario_with_loaded_pid(push_reference_scenario(profile));
      const auto command = run_simulator_scenario_with_loaded_pid(
          commanded_pitch_reference_scenario(profile));
      const auto release_metrics = measure_controller_scenario(release);
      const auto push_metrics = measure_controller_scenario(push, 1.1);
      const auto command_metrics = measure_controller_scenario(command, 2.1);
      std::cout << "controller_shape candidate=" << candidate.name
                << " profile=" << BalancerSimulator::profile_name(profile)
                << " release=" << classify_controller_metrics(release_metrics)
                << ":tail=" << release_metrics.tail_rms_deg
                << ":settle=" << release_metrics.settling_s
                << ":cmd=" << release_metrics.command_rms_sps << "/"
                << release_metrics.command_peak_sps << " push="
                << classify_controller_metrics(push_metrics) << ":tail="
                << push_metrics.tail_rms_deg << ":settle=" << push_metrics.settling_s
                << ":cmd=" << push_metrics.command_rms_sps << "/"
                << push_metrics.command_peak_sps << " command="
                << classify_controller_metrics(command_metrics) << ":tail="
                << command_metrics.tail_rms_deg << ":settle=" << command_metrics.settling_s
                << ":cmd=" << command_metrics.command_rms_sps << "/"
                << command_metrics.command_peak_sps << '\n';
    }
  }
}

TEST(SimulatorReferenceTest, AccelerationFeedbackSmallGainCheckIsExplicit) {
  ScopedStateFeedbackConfig restore;
  ConfigPid::load(sim_pid_path());
  ConfigPid::values.pitch_gain = 8000.0;
  ConfigPid::values.pitch_rate_gain = 500.0;
  ConfigPid::values.velocity_damping_per_s = 0.0;
  ConfigPid::values.velocity_I = 0.0;

  for (const double accel_gain : {0.0, 2.0, 4.0, 8.0}) {
    ConfigPid::values.pitch_accel_gain = accel_gain;
    const auto result = run_simulator_scenario_with_loaded_pid(push_reference_scenario(
        PhysicsProfile::DirectActuator));
    const auto metrics = measure_controller_scenario(result, 1.1);
    const double high_frequency_command = band_rms_signal_in_window(
        result, 2.0, 6.0, 20, 30, [](const auto& row) { return row.u_sps; });
    std::cout << "accel_small_gain gain=" << accel_gain << " class="
              << classify_controller_metrics(metrics) << " tail=" << metrics.tail_rms_deg
              << " settle=" << metrics.settling_s << " cmd_rms=" << metrics.command_rms_sps
              << " cmd_peak=" << metrics.command_peak_sps << " pitch_20_30_rms="
              << high_frequency_command << '\n';
  }
}

TEST(SimulatorReferenceTest, VelocityControlBandwidthAndDampingMapIsReported) {
  ScopedStateFeedbackConfig restore;
  ConfigPid::load(sim_pid_path());
  ConfigPid::values.pitch_gain = 8000.0;
  ConfigPid::values.pitch_rate_gain = 500.0;
  ConfigPid::values.pitch_accel_gain = 0.0;
  ConfigPid::values.velocity_I = 0.0;

  for (const double cutoff_hz : {2.0, 3.0, 4.0, 5.0}) {
    for (const double damping_per_s : {8.0, 10.0, 13.0, 16.0}) {
      ConfigPid::values.velocity_control_cutoff_hz = cutoff_hz;
      ConfigPid::values.velocity_damping_per_s = damping_per_s;
      const auto result = run_simulator_scenario_with_loaded_pid(
          velocity_reference_scenario(PhysicsProfile::DirectActuator));
      ASSERT_FALSE(result.rows.empty());
      const auto metrics = measure_controller_scenario(result, 1.0);
      const double velocity_rms = signal_rms_in_window(
          result, 10.0, 14.0, [](const auto& row) { return row.plant_velocity; });
      const double final_velocity = result.rows.back().plant_velocity;
      const double peak_target = signal_peak_in_window(
          result, 0.0, 14.0, [](const auto& row) { return row.velocity_pitch_target_deg; });
      const double velocity_target_8hz_rms = band_rms_signal_in_window(
          result, 2.0, 6.0, 7, 9, [](const auto& row) { return row.velocity_pitch_target_deg; });
      const double command_8hz_rms = band_rms_signal_in_window(
          result, 2.0, 6.0, 7, 9, [](const auto& row) { return row.u_sps; });
      std::cout << "velocity_map cutoff_hz=" << cutoff_hz
                << " damping_per_s=" << damping_per_s << " class="
                << classify_controller_metrics(metrics) << " fell=" << result.fell
                << " pitch_tail=" << metrics.tail_rms_deg << " velocity_rms=" << velocity_rms
                << " final_velocity_mps=" << final_velocity << " peak_pitch_target_deg="
                << peak_target << " command_rms_sps=" << metrics.command_rms_sps
                << " command_peak_sps=" << metrics.command_peak_sps
                << " velocity_target_8hz_rms_deg=" << velocity_target_8hz_rms
                << " command_8hz_rms_sps=" << command_8hz_rms << '\n';
    }
  }

  for (const double initial_velocity_mps : {0.15, -0.15}) {
    ConfigPid::values.velocity_control_cutoff_hz = 3.0;
    ConfigPid::values.velocity_damping_per_s = 13.0;
    const auto result = run_simulator_scenario_with_loaded_pid(
        velocity_reference_scenario(PhysicsProfile::DirectActuator, initial_velocity_mps));
    std::cout << "velocity_initial_state initial_velocity_mps=" << initial_velocity_mps
              << " fell=" << result.fell << " final_velocity_mps="
              << result.rows.back().plant_velocity << " final_position_m="
              << result.rows.back().plant_position << " tail_pitch_rms_deg="
              << result.tail_rms_pitch_deg << '\n';
  }

  // Repeat the center/edge bandwidth choices on the historical simple-force
  // reference. This is a robustness comparison, not a tuning authority.
  for (const double cutoff_hz : {2.0, 3.0, 5.0}) {
    ConfigPid::values.velocity_control_cutoff_hz = cutoff_hz;
    ConfigPid::values.velocity_damping_per_s = 13.0;
    const auto result = run_simulator_scenario_with_loaded_pid(
        velocity_reference_scenario(PhysicsProfile::RetiredSimpleForce));
    const auto metrics = measure_controller_scenario(result, 1.0);
    std::cout << "velocity_reference profile=retired_simple_force_alias cutoff_hz=" << cutoff_hz
              << " fell=" << result.fell << " tail=" << metrics.tail_rms_deg
              << " command_rms_sps=" << metrics.command_rms_sps
              << " final_velocity_mps=" << result.rows.back().plant_velocity << '\n';
  }
}

TEST(SimulatorReferenceTest, StateFeedbackComTrimAcquisitionFreezeAndMaintenanceAreReported) {
  ScopedStateFeedbackConfig restore;
  ConfigPid::load(sim_pid_path());
  ConfigPid::values.pitch_gain = 8000.0;
  ConfigPid::values.pitch_rate_gain = 500.0;
  ConfigPid::values.pitch_accel_gain = 0.0;
  ConfigPid::values.velocity_control_cutoff_hz = 3.0;
  ConfigPid::values.velocity_damping_per_s = 13.0;
  ConfigPid::values.velocity_pitch_limit_deg = 4.0;
  ConfigPid::values.velocity_I = 0.001;

  SimulatorScenario scenario;
  scenario.name = "state_feedback_com_acquisition_motion_maintenance";
  scenario.duration_s = 90.0;
  scenario.physics_profile = PhysicsProfile::DirectActuator;
  scenario.com_angle_offset_rad = 0.002;
  scenario.joy_segments = {
      SimulatorJoySegment{.start_s = 30.0,
                          .duration_s = 4.0,
                          .forward = 0.25,
                          .forward_end = 0.25},
  };
  scenario.disturbances.push_back(SimulatorDisturbance{
      .kind = SimulatorDisturbanceKind::Step,
      .start_s = 40.0,
      .duration_s = 0.10,
      .force_n = 0.25,
  });
  const auto result = run_simulator_scenario_with_loaded_pid(scenario);
  ASSERT_FALSE(result.rows.empty());

  double max_trim_before_motion = 0.0;
  double min_trim_during_motion = std::numeric_limits<double>::infinity();
  double max_trim_during_motion = -std::numeric_limits<double>::infinity();
  bool saw_acquired = false;
  bool saw_motion_block = false;
  bool saw_quiet_learning = false;
  for (const auto& row : result.rows) {
    if (row.sim_time_s < 30.0) max_trim_before_motion =
        std::max(max_trim_before_motion, std::abs(row.com_trim_deg));
    if (row.sim_time_s >= 30.0 && row.sim_time_s < 44.0) {
      min_trim_during_motion = std::min(min_trim_during_motion, row.com_trim_deg);
      max_trim_during_motion = std::max(max_trim_during_motion, row.com_trim_deg);
      saw_motion_block = saw_motion_block || row.trim_learning_enabled < 0.5;
    }
    saw_acquired = saw_acquired ||
                   (row.trim_learning_block_reason == ComTrimLearningBlockNone &&
                    std::abs(row.com_trim_deg) > 0.01);
    saw_quiet_learning = saw_quiet_learning ||
                         (row.sim_time_s > 50.0 && row.trim_learning_enabled > 0.5);
  }
  const double motion_trim_span = max_trim_during_motion - min_trim_during_motion;
  std::cout << "com_trim_state_feedback fell=" << result.fell
            << " max_trim_before_motion_deg=" << max_trim_before_motion
            << " motion_trim_span_deg=" << motion_trim_span
            << " final_trim_deg=" << result.rows.back().com_trim_deg
            << " saw_acquired=" << saw_acquired << " saw_motion_block=" << saw_motion_block
            << " saw_quiet_learning=" << saw_quiet_learning
            << " tail_pitch_rms_deg=" << result.tail_rms_pitch_deg << '\n';
}

TEST(SimulatorReferenceTest, LowerGainStateFeedbackCandidatesGetVelocityBandwidthCheck) {
  ScopedStateFeedbackConfig restore;
  ConfigPid::load(sim_pid_path());
  ConfigPid::values.pitch_accel_gain = 0.0;
  ConfigPid::values.velocity_I = 0.0;

  struct Candidate {
    const char* name;
    double pitch;
    double rate;
  };
  const std::array<Candidate, 3> candidates = {{
      {"broad_7000_500", 7000.0, 500.0},
      {"low_6000_500", 6000.0, 500.0},
      {"mid_6500_450", 6500.0, 450.0},
  }};
  for (const auto& candidate : candidates) {
    ConfigPid::values.pitch_gain = candidate.pitch;
    ConfigPid::values.pitch_rate_gain = candidate.rate;
    for (const double cutoff_hz : {2.0, 3.0, 5.0}) {
      for (const double damping_per_s : {10.0, 13.0, 16.0}) {
        ConfigPid::values.velocity_control_cutoff_hz = cutoff_hz;
        ConfigPid::values.velocity_damping_per_s = damping_per_s;
        const auto result = run_simulator_scenario_with_loaded_pid(
            velocity_reference_scenario(PhysicsProfile::DirectActuator));
        const auto metrics = measure_controller_scenario(result, 1.0);
        const double velocity_rms = signal_rms_in_window(
            result, 10.0, 14.0, [](const auto& row) { return row.plant_velocity; });
        std::cout << "selected_velocity candidate=" << candidate.name
                  << " cutoff_hz=" << cutoff_hz << " damping_per_s=" << damping_per_s
                  << " fell=" << result.fell << " pitch_tail=" << metrics.tail_rms_deg
                  << " velocity_rms=" << velocity_rms
                  << " command_rms_sps=" << metrics.command_rms_sps
                  << " command_peak_sps=" << metrics.command_peak_sps << '\n';
      }
    }
  }
}

TEST(SimulatorReferenceTest, StateFeedbackRobustnessAndDiagnosticProfilesAreReported) {
  ScopedStateFeedbackConfig restore;
  ConfigPid::load(sim_pid_path());
  ConfigPid::values.pitch_gain = 8000.0;
  ConfigPid::values.pitch_rate_gain = 500.0;
  ConfigPid::values.pitch_accel_gain = 0.0;
  ConfigPid::values.velocity_damping_per_s = 0.0;
  ConfigPid::values.velocity_I = 0.0;

  struct Candidate {
    const char* name;
    double pitch;
    double rate;
  };
  const std::array<Candidate, 4> candidates = {{
      {"center_8000_500", 8000.0, 500.0},
      {"broad_7000_500", 7000.0, 500.0},
      {"low_6000_500", 6000.0, 500.0},
      {"mid_6500_450", 6500.0, 450.0},
  }};
  const std::array<PhysicsProfile, 2> profiles = {PhysicsProfile::DirectActuator,
                                                   PhysicsProfile::RetiredSimpleForce};

  struct Variation {
    const char* name;
    double mass = 1.0;
    double inertia = 1.0;
    double moment = 1.0;
    double force = 1.0;
    double pitch_damping = 0.02;
    double motor_tau = -1.0;
    double imu_lag_s = 0.0;
  };
  const std::array<Variation, 14> variations = {{
      {"nominal"},
      {"H_low", 1.0, 1.0, 0.90},
      {"H_high", 1.0, 1.0, 1.10},
      {"J_low", 1.0, 0.85},
      {"J_high", 1.0, 1.15},
      {"mass_low", 0.90},
      {"mass_high", 1.10},
      {"force_low", 1.0, 1.0, 1.0, 0.80},
      {"force_high", 1.0, 1.0, 1.0, 1.20},
      {"damping_low", 1.0, 1.0, 1.0, 1.0, 0.01},
      {"damping_high", 1.0, 1.0, 1.0, 1.0, 0.04},
      {"motor_tau_low", 1.0, 1.0, 1.0, 1.0, 0.02, 0.100},
      {"motor_tau_high", 1.0, 1.0, 1.0, 1.0, 0.02, 0.200},
      {"sensor_lag_1ms", 1.0, 1.0, 1.0, 1.0, 0.02, -1.0, 0.001},
  }};

  for (const auto& candidate : candidates) {
    ConfigPid::values.pitch_gain = candidate.pitch;
    ConfigPid::values.pitch_rate_gain = candidate.rate;
    for (const PhysicsProfile profile : profiles) {
      for (const auto& variation : variations) {
        SimulatorScenario scenario = attitude_reference_scenario(profile, 2.0);
        scenario.name = variation.name;
        scenario.total_mass_scale = variation.mass;
        scenario.pitch_inertia_scale = variation.inertia;
        scenario.first_mass_moment_scale = variation.moment;
        scenario.imu_pitch_lag_s = variation.imu_lag_s;
        scenario.physics_override = BalancerSimulator::physics_for_profile(profile);
        scenario.physics_override->max_force_n *= variation.force;
        scenario.physics_override->pitch_damping = variation.pitch_damping;
        if (variation.motor_tau >= 0.0) scenario.physics_override->motor_tau_s = variation.motor_tau;
        const auto result = run_simulator_scenario_with_loaded_pid(scenario);
        std::cout << "robustness candidate=" << candidate.name
                  << " profile=" << BalancerSimulator::profile_name(profile)
                  << " variation=" << variation.name << " fell=" << result.fell
                  << " max_pitch_deg=" << result.max_abs_pitch_deg
                  << " tail_rms_deg=" << result.tail_rms_pitch_deg
                  << " max_sat_s=" << result.max_continuous_saturation_s << '\n';
      }
    }
  }

  for (const auto& candidate : candidates) {
    ConfigPid::values.pitch_gain = candidate.pitch;
    ConfigPid::values.pitch_rate_gain = candidate.rate;
    for (const PhysicsProfile profile : {PhysicsProfile::Realistic,
                                         PhysicsProfile::ActuatorStress}) {
      const auto result = run_simulator_scenario_with_loaded_pid(
          attitude_reference_scenario(profile, 2.0));
      double phase_fraction = 0.0;
      double motor_fraction = 0.0;
      double force_fraction = 0.0;
      for (const auto& row : result.rows) {
        phase_fraction += row.phase_saturated;
        motor_fraction += row.motor_force_saturated;
        force_fraction += row.force_saturated;
      }
      const double count = static_cast<double>(result.rows.size());
      std::cout << "phase_tire_diagnostic candidate=" << candidate.name
                << " profile=" << BalancerSimulator::profile_name(profile)
                << " fell=" << result.fell << " tail_rms_deg=" << result.tail_rms_pitch_deg
                << " command_peak_sps="
                << signal_peak_in_window(result, 0.0, result.scenario.duration_s,
                                         [](const auto& row) { return row.u_sps; })
                << " phase_sat_fraction=" << phase_fraction / count
                << " motor_sat_fraction=" << motor_fraction / count
                << " force_sat_fraction=" << force_fraction / count << '\n';
    }
  }
}

TEST(SimulatorReferenceTest, StateFeedbackContributionTelemetryMatchesActiveConfiguration) {
  ScopedStateFeedbackConfig restore;
  ConfigPid::load(sim_pid_path());
  ConfigPid::values.pitch_gain = 8000.0;
  ConfigPid::values.pitch_rate_gain = 500.0;
  ConfigPid::values.pitch_accel_gain = 0.0;
  ConfigPid::values.velocity_control_cutoff_hz = 3.0;
  ConfigPid::values.velocity_damping_per_s = 13.0;
  ConfigPid::values.velocity_I = 0.001;

  SimulatorScenario scenario;
  scenario.duration_s = 0.25;
  scenario.physics_profile = PhysicsProfile::DirectActuator;
  const auto result = run_simulator_scenario_with_loaded_pid(scenario);
  ASSERT_FALSE(result.rows.empty());
  const auto& row = result.rows.back();
  EXPECT_DOUBLE_EQ(row.active_pitch_gain_sps_per_rad, 8000.0);
  EXPECT_DOUBLE_EQ(row.active_pitch_rate_gain_sps_per_rad_s, 500.0);
  EXPECT_DOUBLE_EQ(row.active_pitch_accel_gain_sps_per_rad_s2, 0.0);
  EXPECT_DOUBLE_EQ(row.active_velocity_control_cutoff_hz, 3.0);
  EXPECT_DOUBLE_EQ(row.active_velocity_observer_cutoff_hz, Config::fc_velocity_hz);
  EXPECT_NEAR(row.active_com_trim_gain_deg_per_sps_s, 0.001, 1e-9);
  EXPECT_DOUBLE_EQ(row.active_velocity_pitch_limit_deg, 4.0);
  EXPECT_TRUE(std::isfinite(row.pitch_feedback_sps));
  EXPECT_TRUE(std::isfinite(row.pitch_rate_feedback_sps));
  EXPECT_TRUE(std::isfinite(row.pitch_accel_feedback_sps));
  EXPECT_TRUE(std::isfinite(row.velocity_pitch_target_deg));
  EXPECT_TRUE(std::isfinite(row.velocity_pitch_request_unclamped_deg));
  EXPECT_TRUE(std::isfinite(row.velocity_pitch_request_limited_deg));
  EXPECT_TRUE(std::isfinite(row.pitch_target_unclamped_deg));
  EXPECT_TRUE(std::isfinite(row.trim_quiet_rate_rms_dps));
  EXPECT_TRUE(std::isfinite(row.balance_unclamped_sps));
}

TEST(SimulatorReferenceTest, EndToEndStateScalingPreservesRadiansAtControllerBoundary) {
  ScopedStateFeedbackConfig restore;
  ConfigPid::load(sim_pid_path());
  ConfigPid::values.pitch_gain = 6000.0;
  ConfigPid::values.pitch_rate_gain = 350.0;
  ConfigPid::values.pitch_accel_gain = 0.0;
  ConfigPid::values.velocity_damping_per_s = 0.0;
  ConfigPid::values.velocity_I = 0.0;
  ConfigPid::values.velocity_control_cutoff_hz = 3.0;
  ConfigPid::values.balance_max_sps = Config::max_step_rate_sps;

  SimulatorScenario scenario;
  scenario.name = "state_scaling_audit";
  scenario.duration_s = 0.05;
  scenario.physics_profile = PhysicsProfile::DirectActuator;
  scenario.initial_pitch_deg = 0.5;
  scenario.initial_fused_pitch_deg = 0.5;
  scenario.initial_pitch_rate_dps = 1.0;
  const auto result = run_simulator_scenario_with_loaded_pid(scenario);
  ASSERT_FALSE(result.rows.empty());

  bool saw_nonzero_rate = false;
  for (const auto& row : result.rows) {
    // The simulator sensor boundary reports radians internally and the
    // timeline converts only for display.  The raw gyro is the plant rate;
    // the controller-facing rate is the filtered/notched value.
    EXPECT_NEAR(row.pitch_deg, row.fused_pitch_deg, 1.0e-12);
    EXPECT_NEAR(row.pitch_rate_dps, row.filtered_pitch_rate_dps, 1.0e-12);
    EXPECT_NEAR(row.pitch_feedback_sps,
                -ConfigPid::values.pitch_gain * row.pitch_error_deg * M_PI / 180.0,
                1.0e-4);
    EXPECT_NEAR(row.pitch_rate_feedback_sps,
                ConfigPid::values.pitch_rate_gain * row.pitch_rate_dps * M_PI / 180.0,
                1.0e-4);
    EXPECT_NEAR(row.balance_unclamped_sps,
                row.pitch_feedback_sps + row.pitch_rate_feedback_sps +
                    row.pitch_accel_feedback_sps,
                1.0e-4);
    saw_nonzero_rate = saw_nonzero_rate || std::abs(row.gyro_pitch_rate_dps) > 0.1;
  }
  EXPECT_TRUE(saw_nonzero_rate);
}

TEST(SimulatorReferenceTest, SelectedStateFeedbackCandidateLongHorizonAndDriveChecksAreReported) {
  ScopedStateFeedbackConfig restore;
  ConfigPid::load(sim_pid_path());
  ConfigPid::values.pitch_accel_gain = 0.0;
  ConfigPid::values.velocity_control_cutoff_hz = 3.0;
  ConfigPid::values.velocity_damping_per_s = 16.0;
  ConfigPid::values.velocity_I = 0.001;

  struct Candidate {
    const char* name;
    double pitch;
    double rate;
  };
  const std::array<Candidate, 2> candidates = {{
      {"selected_6000_500", 6000.0, 500.0},
      {"alternative_7000_500", 7000.0, 500.0},
  }};
  for (const auto& candidate : candidates) {
    ConfigPid::values.pitch_gain = candidate.pitch;
    ConfigPid::values.pitch_rate_gain = candidate.rate;

    SimulatorScenario neutral;
    neutral.name = candidate.name;
    neutral.duration_s = 180.0;
    neutral.physics_profile = PhysicsProfile::DirectActuator;
    neutral.com_angle_offset_rad = 0.002;
    const auto neutral_result = run_simulator_scenario_with_loaded_pid(neutral);
    const auto neutral_metrics = measure_controller_scenario(neutral_result, 30.0);
    std::cout << "long_neutral candidate=" << candidate.name
              << " fell=" << neutral_result.fell
              << " max_pitch_deg=" << neutral_result.max_abs_pitch_deg
              << " tail_rms_deg=" << neutral_result.tail_rms_pitch_deg
              << " final_velocity_mps=" << neutral_result.rows.back().plant_velocity
              << " final_trim_deg=" << neutral_result.rows.back().com_trim_deg
              << " command_rms_sps=" << neutral_metrics.command_rms_sps << '\n';

    SimulatorScenario drive = neutral;
    drive.name = std::string(candidate.name) + "_drive_stop_reverse";
    drive.duration_s = 30.0;
    drive.joy_segments = {
        SimulatorJoySegment{.start_s = 8.0,
                            .duration_s = 4.0,
                            .forward = 0.25,
                            .forward_end = 0.25},
        SimulatorJoySegment{.start_s = 18.0,
                            .duration_s = 4.0,
                            .forward = -0.25,
                            .forward_end = -0.25},
    };
    const auto drive_result = run_simulator_scenario_with_loaded_pid(drive);
    const auto drive_metrics = measure_controller_scenario(drive_result, 8.0);
    std::cout << "long_drive candidate=" << candidate.name
              << " fell=" << drive_result.fell
              << " max_pitch_deg=" << drive_result.max_abs_pitch_deg
              << " tail_rms_deg=" << drive_result.tail_rms_pitch_deg
              << " final_velocity_mps=" << drive_result.rows.back().plant_velocity
              << " final_trim_deg=" << drive_result.rows.back().com_trim_deg
              << " command_rms_sps=" << drive_metrics.command_rms_sps
              << " command_peak_sps=" << drive_metrics.command_peak_sps << '\n';

    SimulatorScenario push = neutral;
    push.name = std::string(candidate.name) + "_push";
    push.duration_s = 80.0;
    push.disturbances.push_back(SimulatorDisturbance{
        .kind = SimulatorDisturbanceKind::Step,
        .start_s = 35.0,
        .duration_s = 0.10,
        .force_n = 0.35,
    });
    const auto push_result = run_simulator_scenario_with_loaded_pid(push);
    const auto push_metrics = measure_controller_scenario(push_result, 35.1);
    std::cout << "long_push candidate=" << candidate.name
              << " fell=" << push_result.fell
              << " max_pitch_deg=" << push_result.max_abs_pitch_deg
              << " tail_rms_deg=" << push_result.tail_rms_pitch_deg
              << " final_velocity_mps=" << push_result.rows.back().plant_velocity
              << " final_trim_deg=" << push_result.rows.back().com_trim_deg
              << " command_rms_sps=" << push_metrics.command_rms_sps << '\n';
  }
}

TEST(SimulatorRunnerTest, NonlinearSmallAngleAccelerationMatchesLinearizedForceAndPitchSigns) {
  constexpr double initial_pitch_rad = 0.05 * M_PI / 180.0;
  constexpr double external_force_n = 0.01;
  BalancerSimulator::Config config;
  config.initial_pitch_deg = initial_pitch_rad * 180.0 / M_PI;
  config.com_angle_offset_rad = 0.0;
  config.physics_profile = PhysicsProfile::Realistic;
  BalancerSimulator simulator(config);
  simulator.set_external_force_n(external_force_n);

  const auto model = BalancerSimulator::linearized_upright_model(simulator.physics());
  const double expected_x_accel =
      model.A[1][2] * initial_pitch_rad +
      model.horizontal_force_input[1] * external_force_n;
  const double expected_pitch_accel =
      model.A[3][2] * initial_pitch_rad +
      model.horizontal_force_input[3] * external_force_n;

  simulator.step(1e-6);
  EXPECT_NEAR(simulator.diagnostics().x_ddot, expected_x_accel, 2e-5);
  EXPECT_NEAR(simulator.diagnostics().theta_ddot, expected_pitch_accel, 2e-4);
}

TEST(SimulatorTransferTest, NominalReferenceMeetsGatesAndSecondaryIsDiagnostic) {
  ConfigPid::load(sim_pid_path());
  const auto scenarios = transfer_scenario_set();
  ASSERT_EQ(scenarios.size(), 7u);
  for (const auto& scenario : scenarios) {
    SCOPED_TRACE(scenario.name);
    const auto result = run_simulator_scenario_with_loaded_pid(scenario);
    const auto acceptance = evaluate_transfer_scenario(result);
    if (scenario.physics_profile == PhysicsProfile::DirectActuator) {
      EXPECT_TRUE(acceptance.accepted) << [&]() {
        std::string joined;
        for (const auto& failure : acceptance.failures) {
          if (!joined.empty()) joined += ",";
          joined += failure;
        }
        return joined;
      }();
    } else {
      // The secondary actuator-stress fixture is a model-sensitivity report,
      // not a calibrated acceptance target for the shared controller.
      std::cout << "secondary_diagnostic " << scenario.name << " accepted="
                << acceptance.accepted << " fell=" << result.fell << " failures=";
      for (size_t index = 0; index < acceptance.failures.size(); ++index) {
        if (index != 0) std::cout << ',';
        std::cout << acceptance.failures[index];
      }
      std::cout << '\n';
      EXPECT_FALSE(result.rows.empty());
      EXPECT_EQ(result.actuator_fault_count, 0u);
      EXPECT_EQ(result.controller_fault_flags & ControllerFaultActuator, 0u);
    }
  }
}

TEST(SimulatorTransferTest, DirectActuatorReferencePidWorksOnNominalPlant) {
  ConfigPid::load(sim_pid_path());
  EXPECT_DOUBLE_EQ(ConfigPid::values.pitch_gain, 6000.0);
  EXPECT_DOUBLE_EQ(ConfigPid::values.pitch_rate_gain, 350.0);
  EXPECT_DOUBLE_EQ(ConfigPid::values.pitch_accel_gain, 0.0);
  EXPECT_DOUBLE_EQ(ConfigPid::values.velocity_control_cutoff_hz, 3.0);
  EXPECT_DOUBLE_EQ(ConfigPid::values.velocity_damping_per_s, 8.0);
  EXPECT_DOUBLE_EQ(ConfigPid::values.velocity_I, 0.001);
  auto scenario = simulator_named_scenario("neutral_hold", PhysicsProfile::DirectActuator);
  ASSERT_TRUE(scenario.has_value());
  scenario->duration_s = 180.0;
  scenario->com_angle_offset_rad = 0.001;

  const auto result = run_simulator_scenario_with_loaded_pid(*scenario);
  EXPECT_FALSE(result.fell);
  EXPECT_LT(result.max_abs_pitch_deg, 5.0);
  EXPECT_LT(result.tail_rms_pitch_deg, 0.20);
  EXPECT_EQ(result.controller_fault_flags, 0U);
  EXPECT_EQ(result.actuator_fault_count, 0U);
}

TEST(SimulatorTransferTest, CheckedInDefaultPidAcquiresTrimAndRecoversSmallPush) {
  ConfigPid::load(sim_pid_path());
  auto scenario = simulator_named_scenario("neutral_hold", PhysicsProfile::DirectActuator);
  ASSERT_TRUE(scenario.has_value());
  scenario->name = "default_com_acquisition_small_push";
  scenario->duration_s = 120.0;
  scenario->com_angle_offset_rad = 0.001;
  scenario->disturbances.push_back(SimulatorDisturbance{
      .kind = SimulatorDisturbanceKind::Step,
      .start_s = 40.0,
      .duration_s = 0.10,
      .force_n = 0.5,
  });

  const auto result = run_simulator_scenario_with_loaded_pid(*scenario);
  ASSERT_FALSE(result.rows.empty());
  EXPECT_FALSE(result.fell);
  EXPECT_LT(result.max_abs_pitch_deg, 5.0);
  EXPECT_LT(result.tail_rms_pitch_deg, 0.20);
  EXPECT_EQ(result.controller_fault_flags, 0U);
  EXPECT_EQ(result.actuator_fault_count, 0U);

  const auto acquired_trim = std::any_of(
      result.rows.begin(), result.rows.end(), [](const auto& row) {
        return std::abs(row.com_trim_deg) > 0.01;
      });
  const auto learning_resumed = std::any_of(
      result.rows.begin(), result.rows.end(), [](const auto& row) {
        return row.sim_time_s >= 60.0 && row.trim_learning_enabled > 0.5;
      });
  EXPECT_TRUE(acquired_trim);
  EXPECT_TRUE(learning_resumed);
}

TEST(SimulatorReferenceTest, VelocityAuthorityGainAndLimitMatrixIsReported) {
  ScopedStateFeedbackConfig restore;
  ConfigPid::load(sim_pid_path());
  ConfigPid::values.pitch_gain = 6000.0;
  ConfigPid::values.pitch_rate_gain = 350.0;
  ConfigPid::values.pitch_accel_gain = 0.0;
  ConfigPid::values.velocity_control_cutoff_hz = 3.0;
  ConfigPid::values.velocity_I = 0.0;
  ConfigPid::values.balance_max_sps = 12000.0;

  const auto report = [](const char* label, double damping, double limit_deg,
                         double disturbance_sign_sps, double disturbance_force_n) {
    ConfigPid::values.velocity_damping_per_s = damping;
    ConfigPid::values.velocity_pitch_limit_deg = limit_deg;
    SimulatorScenario scenario;
    scenario.name = "velocity_authority_disturbance";
    scenario.duration_s = 12.0;
    scenario.physics_profile = PhysicsProfile::DirectActuator;
    scenario.disturbances.push_back(SimulatorDisturbance{
        .kind = SimulatorDisturbanceKind::Step,
        .start_s = 0.5,
        .duration_s = 1.0,
        .force_n = disturbance_sign_sps > 0.0 ? disturbance_force_n : -disturbance_force_n,
    });
    const auto result = run_simulator_scenario_with_loaded_pid(scenario);
    double max_velocity_sps = 0.0;
    double max_unclamped_pitch_deg = 0.0;
    double max_limited_pitch_deg = 0.0;
    double authority_fraction = 0.0;
    for (const auto& row : result.rows) {
      max_velocity_sps = std::max(max_velocity_sps, std::abs(row.plant_velocity) /
                                                     Config::meters_per_step);
      max_unclamped_pitch_deg =
          std::max(max_unclamped_pitch_deg,
                   std::abs(row.velocity_pitch_request_unclamped_deg));
      max_limited_pitch_deg =
          std::max(max_limited_pitch_deg,
                   std::abs(row.velocity_pitch_request_limited_deg));
      authority_fraction += row.velocity_authority_limited > 0.5 ? 1.0 : 0.0;
    }
    authority_fraction /= static_cast<double>(std::max<size_t>(1, result.rows.size()));
    std::cout << "velocity_authority label=" << label << " damping=" << damping
              << " limit_deg=" << limit_deg << " disturbance_sign_sps=" << disturbance_sign_sps
              << " disturbance_force_n=" << disturbance_force_n
              << " fell=" << result.fell << " max_velocity_sps=" << max_velocity_sps
              << " max_unclamped_pitch_deg=" << max_unclamped_pitch_deg
              << " max_limited_pitch_deg=" << max_limited_pitch_deg
              << " authority_fraction=" << authority_fraction
              << " final_velocity_sps="
              << result.rows.back().plant_velocity / Config::meters_per_step
              << " final_trim_deg=" << result.rows.back().com_trim_deg << '\n';
    return result;
  };

  for (const double damping : {4.0, 6.0, 8.0, 10.0, 12.0, 16.0}) {
    const auto positive = report("positive", damping, 4.0, 1000.0, 1.5);
    const auto negative = report("negative", damping, 4.0, -1000.0, 1.5);
    ASSERT_FALSE(positive.rows.empty());
    ASSERT_FALSE(negative.rows.empty());
    EXPECT_NEAR(positive.rows.back().plant_velocity,
                -negative.rows.back().plant_velocity, 0.01);
    EXPECT_NEAR(positive.rows.back().com_trim_deg,
                -negative.rows.back().com_trim_deg, 0.01);
  }

  for (const double limit_deg : {0.0, 3.0, 4.0, 5.0, 6.0}) {
    // Use a larger deterministic push here so the report exercises the
    // authority boundary rather than merely confirming that small motion is
    // unaffected by the candidate cap.
    const auto result = report("limit", 8.0, limit_deg, 1500.0, 3.0);
    ASSERT_FALSE(result.rows.empty());
    if (limit_deg > 0.0) {
      EXPECT_LE(std::abs(result.rows.back().velocity_pitch_request_limited_deg),
                limit_deg + 1e-6);
    }
    if (limit_deg == 3.0) {
      EXPECT_TRUE(std::any_of(result.rows.begin(), result.rows.end(), [](const auto& row) {
        return row.velocity_authority_limited > 0.5;
      }));
    }
  }

  for (const double damping : {4.0, 6.0, 8.0, 10.0, 12.0, 16.0}) {
    ConfigPid::values.velocity_damping_per_s = damping;
    ConfigPid::values.velocity_pitch_limit_deg = 4.0;
    for (const auto profile : {PhysicsProfile::DirectActuator,
                               PhysicsProfile::RetiredSimpleForce}) {
      const auto result = run_simulator_scenario_with_loaded_pid(
          velocity_reference_scenario(profile));
      const auto metrics = measure_controller_scenario(result, 1.0);
      const double velocity_rms = signal_rms_in_window(
          result, 10.0, 14.0, [](const auto& row) { return row.plant_velocity; });
      const double max_target = signal_peak_in_window(
          result, 0.0, 14.0, [](const auto& row) { return row.pitch_sp_deg; });
      std::cout << "velocity_drive damping=" << damping
                << " limit_deg=4 profile=" << BalancerSimulator::profile_name(profile)
                << " fell=" << result.fell << " velocity_rms_mps=" << velocity_rms
                << " pitch_tail_deg=" << metrics.tail_rms_deg
                << " command_rms_sps=" << metrics.command_rms_sps
                << " command_peak_sps=" << metrics.command_peak_sps
                << " max_pitch_target_deg=" << max_target
                << " final_velocity_mps=" << result.rows.back().plant_velocity << '\n';
    }
  }
}

TEST(SimulatorReferenceTest, InterruptedComTrimAcquisitionPausesAndResumes) {
  ScopedStateFeedbackConfig restore;
  ConfigPid::load(sim_pid_path());
  ConfigPid::values.pitch_gain = kOneThirtySecondPitchGain;
  ConfigPid::values.pitch_rate_gain = 350.0;
  ConfigPid::values.pitch_accel_gain = 0.0;
  ConfigPid::values.velocity_control_cutoff_hz = 3.0;
  // Use the upper sweep edge here to force the authority-limited branch; the
  // selected default is evaluated separately at the lower, ordinary-motion
  // gain. This keeps the regression about state protection rather than gain
  // selection.
  ConfigPid::values.velocity_damping_per_s = 16.0;
  ConfigPid::values.velocity_pitch_limit_deg = 4.0;
  ConfigPid::values.velocity_I = 0.001;

  SimulatorScenario scenario;
  scenario.name = "interrupted_com_acquisition";
  scenario.duration_s = 90.0;
  scenario.physics_profile = PhysicsProfile::DirectActuator;
  scenario.com_angle_offset_rad = 0.004;
  scenario.joy_segments = {
      SimulatorJoySegment{.start_s = 0.5, .duration_s = 2.0, .forward = 0.45},
  };
  scenario.disturbances.push_back(SimulatorDisturbance{
      .kind = SimulatorDisturbanceKind::Step,
      .start_s = 8.0,
      .duration_s = 2.0,
      .force_n = 1.0,
  });
  const auto result = run_simulator_scenario_with_loaded_pid(scenario);
  ASSERT_FALSE(result.rows.empty());

  double trim_before_motion = 0.0;
  double trim_during_motion_min = std::numeric_limits<double>::infinity();
  double trim_during_motion_max = -std::numeric_limits<double>::infinity();
  bool saw_untrusted_motion_block = false;
  bool saw_command_block = false;
  bool saw_authority_block = false;
  bool saw_trusted = false;
  bool saw_learning_after_recovery = false;
  for (const auto& row : result.rows) {
    if (row.sim_time_s < 0.5) trim_before_motion = std::max(trim_before_motion,
                                                              std::abs(row.com_trim_deg));
    if (row.sim_time_s >= 0.5 && row.sim_time_s < 12.0) {
      trim_during_motion_min = std::min(trim_during_motion_min, row.com_trim_deg);
      trim_during_motion_max = std::max(trim_during_motion_max, row.com_trim_deg);
      saw_untrusted_motion_block = saw_untrusted_motion_block ||
                                   (row.trim_trusted < 0.5 && row.trim_learning_allowed < 0.5);
      saw_command_block = saw_command_block ||
                          row.trim_learning_block_reason ==
                              static_cast<uint32_t>(ComTrimLearningBlockCommand);
      saw_authority_block = saw_authority_block || row.velocity_authority_limited > 0.5;
    }
    saw_trusted = saw_trusted || row.trim_trusted > 0.5;
    saw_learning_after_recovery =
        saw_learning_after_recovery || (row.sim_time_s > 30.0 &&
                                        row.trim_learning_enabled > 0.5);
  }
  std::cout << "com_acquisition_interrupted fell=" << result.fell
            << " trim_before_motion_deg=" << trim_before_motion
            << " trim_motion_span_deg=" << trim_during_motion_max - trim_during_motion_min
            << " final_trim_deg=" << result.rows.back().com_trim_deg
            << " saw_untrusted_motion_block=" << saw_untrusted_motion_block
            << " saw_command_block=" << saw_command_block
            << " saw_authority_block=" << saw_authority_block
            << " saw_trusted=" << saw_trusted
            << " saw_learning_after_recovery=" << saw_learning_after_recovery << '\n';
  EXPECT_FALSE(result.fell);
  EXPECT_TRUE(saw_untrusted_motion_block);
  EXPECT_TRUE(saw_command_block);
  EXPECT_TRUE(saw_authority_block);
  EXPECT_TRUE(saw_trusted);
  EXPECT_TRUE(saw_learning_after_recovery);
  EXPECT_LT(trim_during_motion_max - trim_during_motion_min, 0.25);
}

TEST(SimulatorReferenceTest, QuietRateMetricRejectsInstantaneousResidualCrossings) {
  ScopedStateFeedbackConfig restore;
  ConfigPid::load(sim_pid_path());
  ConfigPid::values.velocity_damping_per_s = 0.0;
  ConfigPid::values.velocity_pitch_limit_deg = 4.0;
  ConfigPid::values.velocity_I = 0.001;

  SimulatorScenario scenario;
  scenario.name = "quiet_metric_30hz_residual";
  scenario.duration_s = 8.0;
  scenario.physics_profile = PhysicsProfile::DirectActuator;
  scenario.gyro_pitch_disturbance_frequency_hz = 29.0;
  scenario.gyro_pitch_disturbance_amplitude_rad_s = 10.0 * M_PI / 180.0;
  const auto result = run_simulator_scenario_with_loaded_pid(scenario);
  ASSERT_FALSE(result.rows.empty());
  const auto& last = result.rows.back();
  std::cout << "quiet_metric residual_hz=29 rate_rms_dps="
            << last.trim_quiet_rate_rms_dps << " trusted=" << last.trim_trusted
            << " learning_allowed=" << last.trim_learning_allowed << '\n';
  EXPECT_LT(last.trim_quiet_rate_rms_dps, 10.0);
}

}  // namespace
