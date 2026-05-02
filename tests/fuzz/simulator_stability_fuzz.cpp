#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "afl_compat.h"
#include "fuzz_scenario_format.h"
#include "fuzz_support.h"
#include "messages/types.h"
#include "simulator/simulator_runner.h"

namespace {

volatile std::uint64_t g_stability_sink = 0;

float clamp_finite(float value, float lo, float hi, float fallback) {
  if (!std::isfinite(value)) {
    return fallback;
  }
  return std::clamp(value, lo, hi);
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

void decode_base_scenario(const fuzz::FuzzSimulatorScenarioV1& wire, SimulatorScenario& scenario) {
  scenario.name = wire.version == fuzz::kFuzzScenarioVersion2 ? "stability_fuzz_v2" : "stability_fuzz_v1";
  scenario.physics_profile = PhysicsProfile::Realistic;
  scenario.duration_s = clamp_finite(wire.duration_s, 0.25f, 4.0f, 1.0f);
  scenario.initial_pitch_deg = clamp_finite(wire.initial_pitch_deg, -20.0f, 20.0f, 0.0f);
  scenario.com_angle_offset_rad =
      clamp_finite(wire.com_angle_offset_rad, -0.02f, 0.02f, 0.0f);
  scenario.wheel_slip_factor = clamp_finite(wire.wheel_slip_factor, 0.3f, 1.5f, 1.0f);
  scenario.velocity_feedback_scale =
      clamp_finite(wire.velocity_feedback_scale, 0.0f, 1.2f, 0.05f);
  scenario.velocity_feedback_tau_s =
      clamp_finite(wire.velocity_feedback_tau_s, 0.0f, 0.20f, 0.0f);
  scenario.imu_pitch_lag_s = clamp_finite(wire.imu_pitch_lag_s, 0.0f, 0.04f, 0.0f);

  const std::size_t disturbance_count =
      std::min<std::size_t>(wire.disturbance_count, fuzz::kMaxFuzzDisturbances);
  scenario.disturbances.reserve(disturbance_count);
  for (std::size_t idx = 0; idx < disturbance_count; ++idx) {
    const auto& disturbance = wire.disturbances[idx];
    SimulatorDisturbance sanitized{};
    sanitized.kind = map_disturbance_kind(disturbance.kind);
    sanitized.start_s =
        clamp_finite(disturbance.start_s, 0.0f, static_cast<float>(scenario.duration_s), 0.0f);
    sanitized.duration_s = clamp_finite(
        disturbance.duration_s, 0.0f, static_cast<float>(scenario.duration_s), 0.0f);
    sanitized.force_n = clamp_finite(disturbance.force_n, -3.0f, 3.0f, 0.0f);
    sanitized.com_bias_rad = clamp_finite(disturbance.com_bias_rad, -0.03f, 0.03f, 0.0f);
    sanitized.force_n_end =
        clamp_finite(disturbance.force_n_end, -3.0f, 3.0f, static_cast<float>(sanitized.force_n));
    sanitized.com_bias_rad_end =
        clamp_finite(disturbance.com_bias_rad_end, -0.03f, 0.03f, static_cast<float>(sanitized.com_bias_rad));
    scenario.disturbances.push_back(sanitized);
  }
}

SimulatorScenario decode_stability_scenario(const fuzz::FuzzSimulatorScenarioV2& wire) {
  SimulatorScenario scenario;
  decode_base_scenario(wire.base, scenario);

  const std::size_t joy_count =
      std::min<std::size_t>(wire.joy_segment_count, fuzz::kMaxFuzzJoySegments);
  scenario.joy_segments.reserve(joy_count);
  for (std::size_t idx = 0; idx < joy_count; ++idx) {
    const auto& segment = wire.joy_segments[idx];
    SimulatorJoySegment sanitized{};
    sanitized.start_s =
        clamp_finite(segment.start_s, 0.0f, static_cast<float>(scenario.duration_s), 0.0f);
    sanitized.duration_s =
        clamp_finite(segment.duration_s, 0.0f, static_cast<float>(scenario.duration_s), 0.0f);
    sanitized.forward = clamp_finite(segment.forward, -1.0f, 1.0f, 0.0f);
    sanitized.turn = clamp_finite(segment.turn, -1.0f, 1.0f, 0.0f);
    sanitized.forward_end = clamp_finite(segment.forward_end, -1.0f, 1.0f, sanitized.forward);
    sanitized.turn_end = clamp_finite(segment.turn_end, -1.0f, 1.0f, sanitized.turn);
    scenario.joy_segments.push_back(sanitized);
  }

  return scenario;
}

bool running_under_afl() {
  return std::getenv("__AFL_SHM_ID") != nullptr || std::getenv("AFL_DEBUG_CHILD") != nullptr;
}

[[noreturn]] void fail_stability(const char* reason, double value = 0.0) {
  if (!running_under_afl()) {
    std::fprintf(stderr, "stability fuzz failure: %s", reason);
    if (std::isfinite(value)) {
      std::fprintf(stderr, " (%.6f)", value);
    }
    std::fprintf(stderr, "\n");
  }
  std::abort();
}

void fail_if_bad_stability(const SimulatorRunResult& result) {
  if (result.rows.empty()) {
    fail_stability("empty result");
  }
  if (result.fell) {
    fail_stability("fall", result.max_abs_pitch_deg);
  }
  if (!std::isfinite(result.final_pitch_deg)) {
    fail_stability("non-finite final pitch");
  }
  if (!std::isfinite(result.max_abs_pitch_deg)) {
    fail_stability("non-finite max pitch");
  }
  if (result.max_abs_pitch_deg > 45.0) {
    fail_stability("excessive pitch", result.max_abs_pitch_deg);
  }

  double rail_samples = 0.0;
  double max_abs_velocity_mps = 0.0;
  double max_abs_motor_sps = 0.0;
  for (const auto& row : result.rows) {
    if (!std::isfinite(row.plant_pitch_deg) || !std::isfinite(row.plant_velocity) ||
        !std::isfinite(row.left_sps) || !std::isfinite(row.right_sps)) {
      fail_stability("non-finite row");
    }
    max_abs_velocity_mps = std::max(max_abs_velocity_mps, std::abs(row.plant_velocity));
    max_abs_motor_sps = std::max(max_abs_motor_sps, std::max(std::abs(row.left_sps), std::abs(row.right_sps)));
    rail_samples += row.command_saturated >= 0.5 ? 1.0 : 0.0;
  }

  const double rail_fraction = rail_samples / static_cast<double>(result.rows.size());
  if (rail_fraction > 0.95) {
    fail_stability("excessive rail fraction", rail_fraction);
  }
  if (max_abs_velocity_mps > 100.0) {
    fail_stability("excessive velocity", max_abs_velocity_mps);
  }
  if (max_abs_motor_sps > (kMaxSps + 1e-6)) {
    fail_stability("invalid motor command", max_abs_motor_sps);
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    return 1;
  }

  const std::string pid_path = fuzz::repo_path("pid_sim.conf");
  ConfigPid::load(pid_path);
  while (__AFL_LOOP(1000)) {
    std::vector<uint8_t> input;
    if (!fuzz::read_binary_file(argv[1], input)) {
      return 1;
    }

    fuzz::FuzzSimulatorScenarioV2 wire{};
    fuzz::copy_prefix(input, wire);
    if (wire.base.version != fuzz::kFuzzScenarioVersion2) {
      fuzz::FuzzSimulatorScenarioV1 legacy{};
      fuzz::copy_prefix(input, legacy);
      wire = {};
      wire.base = legacy;
      wire.base.version = fuzz::kFuzzScenarioVersion1;
    }
    const SimulatorScenario scenario = decode_stability_scenario(wire);
    const SimulatorRunResult result = run_simulator_scenario_with_loaded_pid(scenario);

    g_stability_sink ^= result.rows.size();
    g_stability_sink += static_cast<std::uint64_t>(std::llround(std::abs(result.final_pitch_deg)));
    fail_if_bad_stability(result);
  }

  return 0;
}
