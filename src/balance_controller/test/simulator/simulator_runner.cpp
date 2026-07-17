#include "simulator/simulator_runner.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <stdexcept>
#include <string>

#include "messages/types.h"
#include "services/main/config.h"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kFallPitchDeg = 75.0;
constexpr double kTickDtS = 1.0 / 400.0;

SimulatorScenario make_scenario(std::string name, double initial_pitch_deg,
                                double com_angle_offset_rad, PhysicsProfile physics_profile) {
  SimulatorScenario scenario;
  scenario.name = std::move(name);
  scenario.initial_pitch_deg = initial_pitch_deg;
  scenario.com_angle_offset_rad = com_angle_offset_rad;
  scenario.physics_profile = physics_profile;
  return scenario;
}

}  // namespace

SimulatorRunResult run_simulator_scenario_with_loaded_pid(const SimulatorScenario& scenario) {
  SimulatorEngine engine(scenario);
  SimulatorRunResult result;
  result.scenario = scenario;
  result.physics = engine.physics();
  result.rows.reserve(static_cast<size_t>(scenario.duration_s * Config::control_hz));

  const int steps = std::max(1, static_cast<int>(std::llround(scenario.duration_s / kTickDtS)));
  result.max_abs_pitch_deg = std::abs(engine.simulator().get_pitch()) * 180.0 / kPi;
  double continuous_saturation_s = 0.0;

  for (int i = 0; i < steps; ++i) {
    const SimulatorTimelineRow row = engine.step();
    result.rows.push_back(row);
    result.timeline_hash = update_simulator_timeline_hash(result.timeline_hash, row);
    result.max_abs_pitch_deg = std::max(result.max_abs_pitch_deg, std::abs(row.plant_pitch_deg));
    result.controller_fault_flags |= row.controller_fault_flags;
    if (row.actuator_fault > 0.5) ++result.actuator_fault_count;
    if (row.command_saturated > 0.5) {
      continuous_saturation_s += kTickDtS;
      result.max_continuous_saturation_s =
          std::max(result.max_continuous_saturation_s, continuous_saturation_s);
    } else {
      continuous_saturation_s = 0.0;
    }
  }

  result.final_pitch_deg = engine.simulator().get_pitch() * 180.0 / kPi;
  result.fell = result.max_abs_pitch_deg > kFallPitchDeg;
  double tail_squared = 0.0;
  const size_t tail_count = std::min(result.rows.size(), static_cast<size_t>(2.0 / kTickDtS));
  const size_t tail_begin = result.rows.size() - tail_count;
  for (size_t index = tail_begin; index < result.rows.size(); ++index) {
    tail_squared += result.rows[index].plant_pitch_deg * result.rows[index].plant_pitch_deg;
  }
  if (tail_count > 0) {
    result.tail_rms_pitch_deg = std::sqrt(tail_squared / static_cast<double>(tail_count));
  }
  return result;
}

uint64_t update_simulator_timeline_hash(uint64_t hash, const SimulatorTimelineRow& row) {
  const auto add_u64 = [&hash](uint64_t value) {
    for (int shift = 0; shift < 64; shift += 8) {
      hash ^= (value >> shift) & 0xffU;
      hash *= 1099511628211ULL;
    }
  };
  const auto add_double = [&add_u64](double value) { add_u64(std::bit_cast<uint64_t>(value)); };
  add_double(row.sim_time_s);
  add_u64(row.imu_timestamp_us);
  add_double(row.plant_pitch_deg);
  add_double(row.plant_pitch_rate_dps);
  add_double(row.plant_position);
  add_double(row.plant_velocity);
  add_double(row.u_sps);
  add_double(row.turn_sps);
  add_double(row.left_sps);
  add_double(row.right_sps);
  add_double(row.left_actual_steps);
  add_double(row.right_actual_steps);
  add_u64(row.controller_fault_flags);
  add_u64(row.controller_saturation_flags);
  return hash;
}

SimulatorRunResult run_simulator_scenario(const SimulatorScenario& scenario,
                                          const std::string& pid_config_path) {
  ConfigPid::load(pid_config_path);
  SimulatorRunResult result = run_simulator_scenario_with_loaded_pid(scenario);
  result.pid_config_path = pid_config_path;
  return result;
}

std::optional<SimulatorScenario> simulator_named_scenario(std::string_view name,
                                                          PhysicsProfile physics_profile) {
  if (name == "neutral_hold") {
    return make_scenario("neutral_hold", 0.0, 0.0, physics_profile);
  }
  if (name == "pitch_bias_pos") {
    return make_scenario("pitch_bias_pos", 0.10, 0.0, physics_profile);
  }
  if (name == "pitch_bias_neg") {
    return make_scenario("pitch_bias_neg", -0.10, 0.0, physics_profile);
  }
  if (name == "com_offset_pos") {
    return make_scenario("com_offset_pos", 0.0, 0.001, physics_profile);
  }
  if (name == "com_offset_neg") {
    return make_scenario("com_offset_neg", 0.0, -0.001, physics_profile);
  }
  if (name == "combined_bias_pos") {
    return make_scenario("combined_bias_pos", 0.10, 0.001, physics_profile);
  }
  if (name == "combined_bias_neg") {
    return make_scenario("combined_bias_neg", -0.10, -0.001, physics_profile);
  }
  if (name == "recovery_large_pitch_pos") {
    return make_scenario("recovery_large_pitch_pos", 0.25, 0.0, physics_profile);
  }
  if (name == "recovery_large_pitch_neg") {
    return make_scenario("recovery_large_pitch_neg", -0.25, 0.0, physics_profile);
  }
  if (name == "disturbance_pulse") {
    auto scenario = make_scenario("disturbance_pulse", 0.0, 0.0, physics_profile);
    scenario.disturbances.push_back(SimulatorDisturbance{
        .start_s = 1.0,
        .duration_s = 0.20,
        .force_n = 0.25,
    });
    return scenario;
  }
  if (name == "slow_push_recover") {
    auto scenario = make_scenario("slow_push_recover", 0.0, 0.0, physics_profile);
    scenario.duration_s = 20.0;
    scenario.disturbances = {
        SimulatorDisturbance{
            .kind = SimulatorDisturbanceKind::Ramp,
            .start_s = 1.0,
            .duration_s = 4.0,
            .force_n = 0.0,
            .com_bias_rad = 0.0,
            .force_n_end = 0.45,
            .com_bias_rad_end = 0.0,
        },
        SimulatorDisturbance{
            .kind = SimulatorDisturbanceKind::Ramp,
            .start_s = 5.0,
            .duration_s = 4.0,
            .force_n = 0.45,
            .com_bias_rad = 0.0,
            .force_n_end = 0.0,
            .com_bias_rad_end = 0.0,
        },
    };
    return scenario;
  }
  if (name == "slow_push_runaway") {
    auto scenario = make_scenario("slow_push_runaway", 0.0, 0.0, physics_profile);
    scenario.duration_s = 40.0;
    scenario.imu_pitch_lag_s = 0.01;
    scenario.disturbances = {
        SimulatorDisturbance{
            .kind = SimulatorDisturbanceKind::Ramp,
            .start_s = 1.0,
            .duration_s = 8.0,
            .force_n = 0.0,
            .com_bias_rad = 0.0,
            .force_n_end = 0.60,
            .com_bias_rad_end = 0.0,
        },
        SimulatorDisturbance{
            .kind = SimulatorDisturbanceKind::HoldBias,
            .start_s = 9.0,
            .duration_s = 0.0,
            .force_n = 0.60,
            .com_bias_rad = 0.0,
        },
    };
    return scenario;
  }
  if (name == "hold_bias_long_horizon") {
    auto scenario = make_scenario("hold_bias_long_horizon", 0.0, 0.0, physics_profile);
    scenario.duration_s = 40.0;
    scenario.imu_pitch_lag_s = 0.01;
    scenario.disturbances.push_back(SimulatorDisturbance{
        .kind = SimulatorDisturbanceKind::HoldBias,
        .start_s = 1.0,
        .duration_s = 0.0,
        .force_n = 0.0,
        .com_bias_rad = 0.02,
    });
    return scenario;
  }
  return std::nullopt;
}

std::vector<SimulatorScenario> simulator_scenario_set(std::string_view set_name,
                                                      PhysicsProfile physics_profile) {
  static constexpr std::array<std::string_view, 5> kRequired = {
      "neutral_hold", "pitch_bias_pos", "pitch_bias_neg", "com_offset_pos", "com_offset_neg",
  };
  static constexpr std::array<std::string_view, 5> kCapability = {
      "combined_bias_pos",        "combined_bias_neg", "recovery_large_pitch_pos",
      "recovery_large_pitch_neg", "disturbance_pulse",
  };
  static constexpr std::array<std::string_view, 3> kSlowPush = {
      "slow_push_recover",
      "slow_push_runaway",
      "hold_bias_long_horizon",
  };

  std::vector<SimulatorScenario> scenarios;
  auto add_names = [&](const auto& names) {
    for (std::string_view name : names) {
      auto scenario = simulator_named_scenario(name, physics_profile);
      if (!scenario.has_value()) {
        throw std::runtime_error("Unknown simulator scenario in set");
      }
      scenarios.push_back(*scenario);
    }
  };

  if (set_name == "required") {
    add_names(kRequired);
    return scenarios;
  }
  if (set_name == "capability") {
    add_names(kCapability);
    return scenarios;
  }
  if (set_name == "all") {
    add_names(kRequired);
    add_names(kCapability);
    add_names(kSlowPush);
    return scenarios;
  }
  if (set_name == "slow_push") {
    add_names(kSlowPush);
    return scenarios;
  }

  throw std::runtime_error("Unknown simulator scenario set: " + std::string(set_name));
}

std::vector<SimulatorScenario> transfer_scenario_set() {
  auto push_scenario = [](std::string name) {
    SimulatorScenario scenario;
    scenario.name = std::move(name);
    scenario.physics_profile = PhysicsProfile::Realistic;
    scenario.duration_s = 20.0;
    scenario.disturbances.push_back(SimulatorDisturbance{
        .kind = SimulatorDisturbanceKind::Step,
        .start_s = 1.0,
        .duration_s = 0.1,
        .force_n = 3.0,
    });
    return scenario;
  };

  std::vector<SimulatorScenario> scenarios;
  SimulatorScenario neutral;
  neutral.name = "nominal_neutral";
  neutral.physics_profile = PhysicsProfile::Realistic;
  neutral.duration_s = 20.0;
  scenarios.push_back(neutral);

  scenarios.push_back(push_scenario("nominal_push"));
  auto noisy = push_scenario("nominal_noisy_push");
  noisy.imu_noise_seed = 2026;
  noisy.accel_noise_std_mps2 = 0.20;
  noisy.gyro_noise_std_rad_s = 0.015;
  noisy.imu_pitch_lag_s = 0.01;
  scenarios.push_back(noisy);

  SimulatorScenario drive;
  drive.name = "nominal_drive_800_sps";
  drive.physics_profile = PhysicsProfile::Realistic;
  drive.duration_s = 12.0;
  const double forward = 0.05 + 0.95 * (800.0 / 1200.0);
  drive.joy_segments.push_back(SimulatorJoySegment{
      .start_s = 1.0, .duration_s = 4.0, .forward = 0.0, .forward_end = forward});
  drive.joy_segments.push_back(SimulatorJoySegment{
      .start_s = 5.0, .duration_s = 4.0, .forward = forward, .forward_end = forward});
  scenarios.push_back(drive);

  auto add_scaled = [&](std::string name, const auto& mutate) {
    auto scenario = push_scenario("margin_" + name);
    scenario.physics_override = BalancerSimulator::physics_for_profile(PhysicsProfile::Realistic);
    mutate(scenario);
    scenarios.push_back(std::move(scenario));
  };
  add_scaled("mass_low", [](auto& value) { value.mass_scale = 0.90; });
  add_scaled("mass_high", [](auto& value) { value.mass_scale = 1.10; });
  add_scaled("com_low", [](auto& value) { value.com_height_scale = 0.85; });
  add_scaled("com_high", [](auto& value) { value.com_height_scale = 1.15; });
  add_scaled("inertia_low", [](auto& value) { value.inertia_scale = 0.80; });
  add_scaled("inertia_high", [](auto& value) { value.inertia_scale = 1.20; });
  add_scaled("motor_low", [](auto& value) { value.physics_override->max_force_n *= 0.70; });
  add_scaled("motor_high", [](auto& value) { value.physics_override->max_force_n *= 1.20; });
  add_scaled("lag_low", [](auto& value) { value.physics_override->motor_tau_s = 0.004; });
  add_scaled("lag_high", [](auto& value) { value.physics_override->motor_tau_s = 0.020; });
  add_scaled("traction_low",
             [](auto& value) { value.physics_override->traction_coefficient = 0.60; });
  add_scaled("traction_high",
             [](auto& value) { value.physics_override->traction_coefficient = 1.20; });
  add_scaled("pitch_damping_low", [](auto& value) { value.physics_override->pitch_damping = 0.0; });
  add_scaled("pitch_damping_high",
             [](auto& value) { value.physics_override->pitch_damping = 0.04; });
  add_scaled("cart_drag_low", [](auto& value) { value.physics_override->cart_damping = 0.30; });
  add_scaled("cart_drag_high", [](auto& value) { value.physics_override->cart_damping = 2.0; });
  return scenarios;
}

TransferAcceptance evaluate_transfer_scenario(const SimulatorRunResult& result) {
  TransferAcceptance acceptance;
  const bool margin = result.scenario.name.rfind("margin_", 0) == 0;
  const auto reject = [&](bool condition, std::string reason) {
    if (condition) acceptance.failures.push_back(std::move(reason));
  };
  reject(result.fell, "fell");
  reject(result.max_abs_pitch_deg >= 15.0, "peak_pitch");
  reject(result.actuator_fault_count != 0, "actuator_fault");
  const uint32_t invalid_timing_or_runtime_faults =
      result.controller_fault_flags & ~static_cast<uint32_t>(ControllerFaultNoImu);
  reject(invalid_timing_or_runtime_faults != ControllerFaultNone, "controller_fault");
  reject(result.tail_rms_pitch_deg >= (margin ? 1.25 : 1.0), "tail_rms");
  reject(result.max_continuous_saturation_s >= (margin ? 0.3125 : 0.250), "continuous_saturation");

  if (result.scenario.name == "nominal_neutral") {
    double max_position = 0.0;
    for (const auto& row : result.rows) {
      max_position = std::max(max_position, std::abs(row.plant_position));
    }
    reject(max_position >= 0.35, "neutral_travel");
  } else if (result.scenario.name == "nominal_push") {
    const bool recovered = std::any_of(result.rows.begin(), result.rows.end(), [](const auto& row) {
      return row.sim_time_s >= 1.1 && row.sim_time_s <= 3.1 && std::abs(row.plant_pitch_deg) < 2.0;
    });
    reject(!recovered, "push_recovery");
  } else if (result.scenario.name == "nominal_drive_800_sps") {
    double drive_sum = 0.0;
    size_t drive_count = 0;
    double stopped_max = 0.0;
    for (const auto& row : result.rows) {
      if (row.sim_time_s >= 8.0 && row.sim_time_s <= 9.0) {
        drive_sum += row.plant_velocity;
        ++drive_count;
      }
      if (row.sim_time_s >= 11.0 && row.sim_time_s <= 12.0) {
        stopped_max = std::max(stopped_max, std::abs(row.plant_velocity));
      }
    }
    const double target_mps = 800.0 * BalancerSimulator::HardwareNominal::meters_per_step;
    const double mean_speed = drive_count > 0 ? drive_sum / static_cast<double>(drive_count) : 0.0;
    reject(drive_count == 0 || std::abs(mean_speed - target_mps) > 0.30 * target_mps,
           "drive_speed");
    reject(stopped_max >= 0.05, "stop_speed");
  }

  acceptance.accepted = acceptance.failures.empty();
  return acceptance;
}
