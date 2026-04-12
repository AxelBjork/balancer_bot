#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>

#include "simulator/simulator_runner.h"

namespace {

std::string sim_pid_path() {
  return (std::filesystem::path(BALANCER_REPO_ROOT) / "pid_sim.conf").string();
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
  EXPECT_LT(result.rows.back().pitch_trim_deg, 0.0);
}

TEST(SimulatorRunnerTest, NegativeComOffsetBuildsPositiveLeanTrim) {
  auto scenario = simulator_named_scenario("com_offset_neg", PhysicsProfile::Realistic);
  ASSERT_TRUE(scenario.has_value());
  scenario->duration_s = 5.0;

  const auto result = run_simulator_scenario(*scenario, sim_pid_path());
  ASSERT_FALSE(result.rows.empty());

  EXPECT_FALSE(result.fell);
  EXPECT_GT(result.rows.back().pitch_trim_deg, 0.0);
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
      .forward = 0.0f,
      .turn = 0.0f,
      .forward_end = 0.2f,
      .turn_end = 0.0f,
  });

  const auto result = run_simulator_scenario(scenario, sim_pid_path());
  ASSERT_FALSE(result.rows.empty());

  double early_target = 0.0;
  double late_target = 0.0;
  int early_count = 0;
  int late_count = 0;
  for (const auto& row : result.rows) {
    if (row.sim_time_s >= 0.15 && row.sim_time_s < 0.25) {
      early_target += row.target_vel_sps;
      ++early_count;
    }
    if (row.sim_time_s >= 0.45 && row.sim_time_s < 0.55) {
      late_target += row.target_vel_sps;
      ++late_count;
    }
  }

  ASSERT_GT(early_count, 0);
  ASSERT_GT(late_count, 0);
  EXPECT_GT(late_target / late_count, early_target / early_count);
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
      .forward = 0.15f,
      .turn = 0.0f,
  });

  const auto result = run_simulator_scenario(scenario, sim_pid_path());
  ASSERT_FALSE(result.rows.empty());

  const auto tail = result.rows.back();
  EXPECT_GT(tail.target_vel_sps, 0.0);
}

}  // namespace
