#include "simulator/simulator_runner.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <random>

#include "services/imu/pitch_lpf.h"
#include "services/main/config.h"

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

TEST(SimulatorRunnerTest, StaticRawImuNoiseIsReducedWithoutErasingPitchBias) {
  BalancerSimulator::Config cfg;
  cfg.initial_pitch_deg = 4.0;
  cfg.physics_profile = PhysicsProfile::Simplified;
  BalancerSimulator sim(cfg);

  PitchComplementaryFilter filter;
  std::mt19937 rng(909);
  std::normal_distribution<double> accel_noise(0.0, 0.25);
  std::normal_distribution<double> gyro_noise(0.0, 0.015);

  constexpr double fs_hz = Config::sampling_hz;
  // The accelerometer correction is intentionally slow (0.05 Hz) so linear
  // balancing acceleration is not mistaken for tilt. Give that path several
  // time constants before measuring its static-bias accuracy.
  constexpr int total_samples = static_cast<int>(30.0 * fs_hz);
  constexpr int warmup_samples = static_cast<int>(15.0 * fs_hz);
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
    filter.push_sample(raw.acc, raw.gyr, now);
    const ImuSample fused = filter.read_latest();
    if (i >= warmup_samples) {
      const double raw_err = raw_pitch_deg(raw.acc) - cfg.initial_pitch_deg;
      const double fused_err = fused.angle_rad * 180.0 / M_PI - cfg.initial_pitch_deg;
      raw_sq += raw_err * raw_err;
      fused_sq += fused_err * fused_err;
      fused_sum += fused.angle_rad * 180.0 / M_PI;
      ++count;
    }
  }

  ASSERT_GT(count, 0);
  const double raw_rms = std::sqrt(raw_sq / count);
  const double fused_rms = std::sqrt(fused_sq / count);
  EXPECT_LT(fused_rms, raw_rms * 0.55);
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

TEST(SimulatorRunnerTest, HardwareNominalConstantsRemainAuthoritative) {
  using Nominal = BalancerSimulator::HardwareNominal;
  EXPECT_DOUBLE_EQ(Nominal::gravity, 9.81);
  EXPECT_DOUBLE_EQ(Nominal::wheel_radius, 0.0412);
  EXPECT_DOUBLE_EQ(Nominal::motor_count, 2.0);
  EXPECT_DOUBLE_EQ(Nominal::motor_stall_torque_nm, 0.45);
  EXPECT_DOUBLE_EQ(Nominal::combined_stall_force_n, 22.5);
  EXPECT_DOUBLE_EQ(BalancerSimulator::physics_for_profile(PhysicsProfile::Realistic).max_force_n,
                   Nominal::combined_stall_force_n);
  EXPECT_DOUBLE_EQ(Nominal::total_mass_kg, 1.032);
  EXPECT_DOUBLE_EQ(Nominal::first_mass_moment_kg_m, 0.06192);
  EXPECT_DOUBLE_EQ(Nominal::pitch_inertia_about_axle_kg_m2, 0.0067552);
  EXPECT_DOUBLE_EQ(Nominal::steps_per_rev, 3200.0);
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
      .duration_s = 0.5,
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
  double max_target_velocity = 0.0;
  double max_turn_split = 0.0;
  for (const auto& row : result.rows) {
    if (row.sim_time_s >= 0.2 && row.sim_time_s <= 0.5) {
      max_pitch_sp = std::max(max_pitch_sp, std::abs(row.pitch_sp_deg));
      max_target_velocity = std::max(max_target_velocity, std::abs(row.target_velocity_sps));
      max_turn_split = std::max(max_turn_split, std::abs(row.left_sps - row.right_sps));
    }
  }

  EXPECT_GT(max_pitch_sp, 0.25);
  EXPECT_GT(max_target_velocity, 400.0);
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
    pitch_accels[index] = simulator.diagnostics().theta_ddot;
  }

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
  EXPECT_GT(simulator.diagnostics().x_ddot, 0.0);
  EXPECT_LT(simulator.diagnostics().theta_ddot, 0.0);
  EXPECT_NEAR(simulator.diagnostics().x_ddot, expected_x_accel, 2e-5);
  EXPECT_NEAR(simulator.diagnostics().theta_ddot, expected_pitch_accel, 2e-4);
}

TEST(SimulatorTransferTest, MandatoryNominalAndConservativeProfilesMeetAcceptanceGates) {
  ConfigPid::load(sim_pid_path());
  const auto scenarios = transfer_scenario_set();
  ASSERT_EQ(scenarios.size(), 10u);
  for (const auto& scenario : scenarios) {
    if (scenario.name == "fast_strong_drive_bidirectional") continue;
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
