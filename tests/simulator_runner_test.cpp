#include "simulator/simulator_runner.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <random>

#include "services/imu/imu_pitch_estimator.h"
#include "services/main/config.h"
#include "services/motor/motor_runner.h"
#include "simulator/tuner_support.h"

namespace {

std::string sim_pid_path() {
  return (std::filesystem::path(BALANCER_REPO_ROOT) / "pid.conf").string();
}

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

double band_rms_pitch_in_window(const SimulatorRunResult& result, double start_s, double end_s,
                                int low_hz, int high_hz) {
  std::vector<double> samples;
  for (const auto& row : result.rows) {
    if (row.sim_time_s >= start_s && row.sim_time_s < end_s) {
      samples.push_back(row.plant_pitch_deg);
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
  constexpr int warmup_samples = static_cast<int>(0.1 * fs_hz);
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
  scenario.physics_profile = PhysicsProfile::Realistic;
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
    apparent_pitch[index] = estimate.sample.angle_rad;

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

TEST(SimulatorTransferTest, MandatoryNominalAndConservativeProfilesMeetAcceptanceGates) {
  ConfigPid::load(sim_pid_path());
  const auto scenarios = transfer_scenario_set();
  ASSERT_EQ(scenarios.size(), 10u);
  for (const auto& scenario : scenarios) {
    SCOPED_TRACE(scenario.name);
    const auto result = run_simulator_scenario_with_loaded_pid(scenario);
    const auto acceptance = evaluate_transfer_scenario(result);
    EXPECT_TRUE(acceptance.accepted) << [&]() {
      std::string joined;
      for (const auto& failure : acceptance.failures) {
        if (!joined.empty()) joined += ",";
        joined += failure;
      }
      return joined;
    }();
  }
}

TEST(SimulatorTransferTest, FastStrongDriveMeetsAcceptanceGates) {
  ConfigPid::load(sim_pid_path());
  const auto scenarios = transfer_scenario_set();
  const auto scenario = std::find_if(scenarios.begin(), scenarios.end(), [](const auto& value) {
    return value.name == "fast_strong_drive_bidirectional";
  });
  ASSERT_NE(scenario, scenarios.end());

  const auto acceptance =
      evaluate_transfer_scenario(run_simulator_scenario_with_loaded_pid(*scenario));
  EXPECT_TRUE(acceptance.accepted);
  EXPECT_TRUE(acceptance.failures.empty());
}

}  // namespace
