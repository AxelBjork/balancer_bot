#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "afl_compat.h"
#include "fuzz_scenario_format.h"
#include "fuzz_support.h"
#include "messages/types.h"
#include "simulator/simulator_runner.h"

namespace {

volatile std::uint64_t g_sim_sink = 0;

float clamp_finite(float value, float lo, float hi, float fallback) {
  if (!std::isfinite(value)) {
    return fallback;
  }
  return std::clamp(value, lo, hi);
}

PhysicsProfile map_profile(uint8_t raw_profile) {
  if (raw_profile == 1) return PhysicsProfile::Realistic;
  if (raw_profile == 2) return PhysicsProfile::ActuatorStress;
  if (raw_profile == 3) return PhysicsProfile::DirectActuator;
  if (raw_profile == 4) return PhysicsProfile::RetiredSimpleForce;
  if (raw_profile == 5) return PhysicsProfile::RetiredNoSlipActuator;
  if (raw_profile == 6) return PhysicsProfile::StepperPhase;
  if (raw_profile == 7) return PhysicsProfile::StepperPhaseElectrical;
  return PhysicsProfile::Simplified;
}

SimulatorDisturbanceKind map_disturbance_kind(uint8_t raw_kind) {
  switch (raw_kind) {
    case 1:
      return SimulatorDisturbanceKind::Ramp;
    case 2:
      return SimulatorDisturbanceKind::HoldBias;
    default:
      return SimulatorDisturbanceKind::Step;
  }
}

SimulatorScenario decode_scenario(const fuzz::FuzzSimulatorScenarioV1& wire) {
  const auto transfer_scenarios = transfer_scenario_set();
  const bool uses_transfer_catalog = wire.version == fuzz::kFuzzScenarioVersion4 &&
                                     wire.transfer_scenario_index < transfer_scenarios.size();
  SimulatorScenario scenario = uses_transfer_catalog
                                   ? transfer_scenarios[wire.transfer_scenario_index]
                                   : SimulatorScenario{};
  scenario.name = uses_transfer_catalog ? "fuzz_transfer_" + scenario.name : "fuzz_custom";
  if (!uses_transfer_catalog) scenario.physics_profile = map_profile(wire.physics_profile);
  scenario.duration_s = clamp_finite(wire.duration_s, 0.01f, 2.0f, 0.25f);
  scenario.initial_pitch_deg = clamp_finite(wire.initial_pitch_deg, -70.0f, 70.0f, 0.0f);
  scenario.com_angle_offset_rad = clamp_finite(wire.com_angle_offset_rad, -0.05f, 0.05f, 0.0f);
  if (!scenario.physics_override.has_value()) {
    scenario.physics_override = BalancerSimulator::physics_for_profile(scenario.physics_profile);
  }
  scenario.physics_override->traction_coefficient *=
      static_cast<double>(clamp_finite(wire.traction_coefficient, 0.0f, 2.0f, 1.0f));
  scenario.imu_pitch_lag_s = clamp_finite(wire.imu_pitch_lag_s, 0.0f, 0.05f, 0.0f);

  const std::size_t disturbance_count =
      std::min<std::size_t>(wire.disturbance_count, fuzz::kMaxFuzzDisturbances);
  scenario.disturbances.reserve(disturbance_count);
  for (std::size_t idx = 0; idx < disturbance_count; ++idx) {
    const auto& disturbance = wire.disturbances[idx];
    SimulatorDisturbance sanitized{};
    sanitized.kind = map_disturbance_kind(disturbance.kind);
    sanitized.start_s =
        clamp_finite(disturbance.start_s, 0.0f, static_cast<float>(scenario.duration_s), 0.0f);
    sanitized.duration_s =
        clamp_finite(disturbance.duration_s, 0.0f, static_cast<float>(scenario.duration_s), 0.0f);
    sanitized.force_n = clamp_finite(disturbance.force_n, -5.0f, 5.0f, 0.0f);
    sanitized.com_bias_rad = clamp_finite(disturbance.com_bias_rad, -0.05f, 0.05f, 0.0f);
    sanitized.force_n_end =
        clamp_finite(disturbance.force_n_end, -5.0f, 5.0f, static_cast<float>(sanitized.force_n));
    sanitized.com_bias_rad_end = clamp_finite(disturbance.com_bias_rad_end, -0.05f, 0.05f,
                                              static_cast<float>(sanitized.com_bias_rad));
    scenario.disturbances.push_back(sanitized);
  }

  return scenario;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    return 1;
  }

  const std::string pid_path = fuzz::repo_path("pid.conf");
  ConfigPid::load(pid_path);
  while (__AFL_LOOP(1000)) {
    std::vector<uint8_t> input;
    if (!fuzz::read_binary_file(argv[1], input)) {
      return 1;
    }

    fuzz::FuzzSimulatorScenarioV1 wire{};
    fuzz::copy_prefix(input, wire);
    const SimulatorScenario scenario = decode_scenario(wire);
    const SimulatorRunResult result = run_simulator_scenario_with_loaded_pid(scenario);

    g_sim_sink = g_sim_sink ^ result.rows.size();
    g_sim_sink = g_sim_sink + (result.fell ? 17u : 3u);
    if (std::isfinite(result.final_pitch_deg)) {
      g_sim_sink =
          g_sim_sink + static_cast<std::uint64_t>(std::llround(std::abs(result.final_pitch_deg)));
    }
    if (std::isfinite(result.max_abs_pitch_deg)) {
      g_sim_sink = g_sim_sink + static_cast<std::uint64_t>(std::llround(result.max_abs_pitch_deg));
    }
  }

  return 0;
}
