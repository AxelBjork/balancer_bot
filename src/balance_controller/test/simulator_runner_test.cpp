#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <random>

#include "config.h"
#include "services/imu/pitch_lpf.h"
#include "simulator/simulator_runner.h"

namespace {

std::string sim_pid_path() {
  return (std::filesystem::path(BALANCER_REPO_ROOT) / "pid_sim.conf").string();
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
  return std::atan2(-acc[0], std::sqrt(acc[1] * acc[1] + acc[2] * acc[2])) * 180.0 / M_PI;
}

TEST(SimulatorRunnerTest, PositivePitchProducesCorrectiveWheelAndPlantResponse) {
  auto scenario = simulator_named_scenario("pitch_bias_pos", PhysicsProfile::Simplified);
  ASSERT_TRUE(scenario.has_value());
  scenario->duration_s = 0.05;

  const auto result = run_simulator_scenario(*scenario, sim_pid_path());
  ASSERT_FALSE(result.rows.empty());

  const auto it = std::find_if(result.rows.begin(), result.rows.end(), [](const auto& row) {
    return std::abs(row.f_app) > 1e-6;
  });
  ASSERT_NE(it, result.rows.end());

  EXPECT_GT(it->f_app, 0.0);
  EXPECT_LT(it->theta_ddot, 0.0);
}

TEST(SimulatorRunnerTest, NegativePitchProducesOppositeCorrectiveResponse) {
  auto scenario = simulator_named_scenario("pitch_bias_neg", PhysicsProfile::Simplified);
  ASSERT_TRUE(scenario.has_value());
  scenario->duration_s = 0.05;

  const auto result = run_simulator_scenario(*scenario, sim_pid_path());
  ASSERT_FALSE(result.rows.empty());

  const auto it = std::find_if(result.rows.begin(), result.rows.end(), [](const auto& row) {
    return std::abs(row.f_app) > 1e-6;
  });
  ASSERT_NE(it, result.rows.end());

  EXPECT_LT(it->f_app, 0.0);
  EXPECT_GT(it->theta_ddot, 0.0);
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
  constexpr int total_samples = static_cast<int>(8.0 * fs_hz);
  constexpr int warmup_samples = static_cast<int>(2.0 * fs_hz);
  const auto tick = std::chrono::nanoseconds{(long long)std::llround(1e9 / fs_hz)};
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

TEST(SimulatorRunnerTest, PositiveComOffsetBuildsNegativeLeanTrim) {
  auto scenario = simulator_named_scenario("com_offset_pos", PhysicsProfile::Realistic);
  ASSERT_TRUE(scenario.has_value());
  scenario->duration_s = 5.0;

  const auto result = run_simulator_scenario(*scenario, sim_pid_path());
  ASSERT_FALSE(result.rows.empty());

  EXPECT_FALSE(result.fell);
  EXPECT_GT(result.rows.back().pitch_trim_deg, 0.0);
}

TEST(SimulatorRunnerTest, NegativeComOffsetBuildsPositiveLeanTrim) {
  auto scenario = simulator_named_scenario("com_offset_neg", PhysicsProfile::Realistic);
  ASSERT_TRUE(scenario.has_value());
  scenario->duration_s = 5.0;

  const auto result = run_simulator_scenario(*scenario, sim_pid_path());
  ASSERT_FALSE(result.rows.empty());

  EXPECT_FALSE(result.fell);
  EXPECT_LT(result.rows.back().pitch_trim_deg, 0.0);
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

TEST(SimulatorRunnerTest, DriveForceUsesExplicitActuatorGainInsteadOfControllerScaling) {
  auto scenario = simulator_named_scenario("pitch_bias_pos", PhysicsProfile::Realistic);
  ASSERT_TRUE(scenario.has_value());
  scenario->duration_s = 0.05;

  const auto result = run_simulator_scenario(*scenario, sim_pid_path());
  ASSERT_FALSE(result.rows.empty());

  const auto it = std::find_if(result.rows.begin(), result.rows.end(), [](const auto& row) {
    return std::abs(row.target_wheel_velocity) > 1e-6;
  });
  ASSERT_NE(it, result.rows.end());
  EXPECT_NEAR(it->f_cmd,
              std::clamp(it->target_wheel_velocity * result.physics.drive_force_per_mps,
                         -result.physics.max_force_n, result.physics.max_force_n),
              1e-3);
}

TEST(SimulatorRunnerTest, SmallAngleLinearizedPlantIsControllableWithOverdampedPoleTargets) {
  const auto physics = BalancerSimulator::physics_for_profile(PhysicsProfile::Realistic);
  const auto model = BalancerSimulator::linearized_upright_model(physics);

  std::vector<std::array<double, 4>> controllability;
  std::array<double, 4> bk = model.B;
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

}  // namespace
