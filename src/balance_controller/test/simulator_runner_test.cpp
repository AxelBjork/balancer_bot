#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>

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
