#include "simulator/simulator_runner.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>
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
  constexpr double kComOffsetRad = 0.002;
  constexpr double kDriveSps = 800.0;
  const double drive_command =
      Config::deadzone + (1.0 - Config::deadzone) * (kDriveSps / ConfigPid::drive_max_sps);

  enum class TransferProfile { Nominal, SlowWeak, FastStrong };

  const auto apply_profile = [](SimulatorScenario& scenario, TransferProfile profile) {
    scenario.physics_profile = PhysicsProfile::Realistic;
    if (profile == TransferProfile::Nominal) return;

    scenario.physics_override = BalancerSimulator::physics_for_profile(PhysicsProfile::Realistic);
    scenario.imu_noise_seed = profile == TransferProfile::SlowWeak ? 2026u : 2027u;
    scenario.accel_noise_std_mps2 = 0.20;
    scenario.gyro_noise_std_rad_s = 0.015;
    scenario.imu_pitch_lag_s = 0.010;

    if (profile == TransferProfile::SlowWeak) {
      scenario.total_mass_scale = 1.10;
      // A payload increases both total mass and axle pitch inertia; H remains the measured base
      // robot value because no payload COM geometry has been asserted for this margin sweep.
      scenario.pitch_inertia_scale = 1.30;
      scenario.physics_override->motor_tau_s = 0.020;
      scenario.physics_override->traction_coefficient = 0.60;
      scenario.physics_override->pitch_damping = 0.02;
      scenario.physics_override->cart_damping = 0.40;
    } else {
      scenario.total_mass_scale = 0.90;
      scenario.pitch_inertia_scale = 0.80;
      scenario.physics_override->motor_tau_s = 0.004;
      scenario.physics_override->traction_coefficient = 1.20;
      scenario.physics_override->pitch_damping = 0.04;
      scenario.physics_override->cart_damping = 2.0;
    }
  };

  const auto release_scenario = [&](std::string name, TransferProfile profile, double sign) {
    SimulatorScenario scenario;
    scenario.name = std::move(name);
    scenario.duration_s = 20.0;
    scenario.initial_pitch_deg = sign * 3.0;
    scenario.com_angle_offset_rad = sign * kComOffsetRad;
    apply_profile(scenario, profile);
    return scenario;
  };

  const auto push_scenario = [&](std::string name, TransferProfile profile) {
    SimulatorScenario scenario;
    scenario.name = std::move(name);
    scenario.duration_s = 20.0;
    scenario.disturbances.push_back(SimulatorDisturbance{
        .kind = SimulatorDisturbanceKind::Step,
        .start_s = 2.0,
        .duration_s = 0.1,
        .force_n = 3.0,
    });
    scenario.disturbances.push_back(SimulatorDisturbance{
        .kind = SimulatorDisturbanceKind::Step,
        .start_s = 10.0,
        .duration_s = 0.1,
        .force_n = -3.0,
    });
    apply_profile(scenario, profile);
    return scenario;
  };

  const auto drive_scenario = [&](std::string name, TransferProfile profile) {
    SimulatorScenario scenario;
    scenario.name = std::move(name);
    scenario.duration_s = 23.0;
    scenario.joy_segments.push_back(SimulatorJoySegment{
        .start_s = 8.0,
        .duration_s = 3.0,
        .forward = drive_command,
        .forward_end = drive_command,
    });
    scenario.joy_segments.push_back(SimulatorJoySegment{
        .start_s = 15.0,
        .duration_s = 3.0,
        .forward = -drive_command,
        .forward_end = -drive_command,
    });
    apply_profile(scenario, profile);
    return scenario;
  };

  return {
      release_scenario("nominal_release_pos", TransferProfile::Nominal, 1.0),
      release_scenario("nominal_release_neg", TransferProfile::Nominal, -1.0),
      push_scenario("nominal_push_symmetric", TransferProfile::Nominal),
      drive_scenario("nominal_drive_bidirectional", TransferProfile::Nominal),
      release_scenario("slow_weak_release", TransferProfile::SlowWeak, 1.0),
      push_scenario("slow_weak_push_symmetric", TransferProfile::SlowWeak),
      drive_scenario("slow_weak_drive_bidirectional", TransferProfile::SlowWeak),
      release_scenario("fast_strong_release", TransferProfile::FastStrong, -1.0),
      push_scenario("fast_strong_push_symmetric", TransferProfile::FastStrong),
      drive_scenario("fast_strong_drive_bidirectional", TransferProfile::FastStrong),
  };
}

TransferAcceptance evaluate_transfer_scenario(const SimulatorRunResult& result) {
  TransferAcceptance acceptance;
  const bool corner = result.scenario.name.rfind("nominal_", 0) != 0;
  const bool release = result.scenario.name.find("release") != std::string::npos;
  const bool push = result.scenario.name.find("push") != std::string::npos;
  const bool drive = result.scenario.name.find("drive") != std::string::npos;
  const auto reject = [&](bool condition, std::string reason) {
    if (condition) acceptance.failures.push_back(std::move(reason));
  };

  const auto window_rms = [&](double start_s, double end_s, const auto& select) {
    double squared = 0.0;
    size_t count = 0;
    for (const auto& row : result.rows) {
      if (row.sim_time_s < start_s || row.sim_time_s >= end_s) continue;
      const double value = select(row);
      squared += value * value;
      ++count;
    }
    return count > 0 ? std::sqrt(squared / static_cast<double>(count))
                     : std::numeric_limits<double>::infinity();
  };
  const auto window_mean = [&](double start_s, double end_s, const auto& select) {
    double sum = 0.0;
    size_t count = 0;
    for (const auto& row : result.rows) {
      if (row.sim_time_s < start_s || row.sim_time_s >= end_s) continue;
      sum += select(row);
      ++count;
    }
    return count > 0 ? sum / static_cast<double>(count)
                     : std::numeric_limits<double>::quiet_NaN();
  };

  reject(result.fell, "fell");
  reject(result.max_abs_pitch_deg >= 20.0, "peak_pitch");
  reject(result.actuator_fault_count != 0, "actuator_fault");
  const uint32_t invalid_timing_or_runtime_faults =
      result.controller_fault_flags & ~static_cast<uint32_t>(ControllerFaultNoImu);
  reject(invalid_timing_or_runtime_faults != ControllerFaultNone, "controller_fault");
  reject(result.tail_rms_pitch_deg >= (corner ? 1.5 : 1.0), "tail_rms");
  reject(result.max_continuous_saturation_s >= 0.500, "continuous_saturation");

  for (const auto& row : result.rows) {
    const bool finite =
        std::isfinite(row.plant_pitch_deg) && std::isfinite(row.plant_pitch_rate_dps) &&
        std::isfinite(row.plant_position) && std::isfinite(row.plant_velocity) &&
        std::isfinite(row.pitch_deg) && std::isfinite(row.pitch_rate_dps) &&
        std::isfinite(row.u_sps) && std::isfinite(row.measured_vel_sps);
    if (!finite) {
      reject(true, "non_finite");
      break;
    }
  }

  const double end_s = result.scenario.duration_s;
  const double preceding_pitch_rms =
      window_rms(end_s - 4.0, end_s - 2.0, [](const auto& row) { return row.plant_pitch_deg; });
  const double final_pitch_rms =
      window_rms(end_s - 2.0, end_s, [](const auto& row) { return row.plant_pitch_deg; });
  // The non-nominal profiles deliberately add IMU noise and lag. Their absolute tail-RMS gate
  // remains strict, but comparing adjacent noisy windows needs a larger noise floor.
  const double tail_growth_limit = corner ? 0.10 : 0.05;
  reject(final_pitch_rms > preceding_pitch_rms + tail_growth_limit, "growing_tail");

  if (release || push) {
    const double mean_neutral_speed = window_mean(
        end_s - 2.0, end_s, [](const auto& row) { return row.measured_vel_sps; });
    reject(!std::isfinite(mean_neutral_speed) || std::abs(mean_neutral_speed) >= 150.0,
           "neutral_speed");
  }

  if (push) {
    const auto recovered_after = [&](double disturbance_end_s) {
      return std::any_of(result.rows.begin(), result.rows.end(), [&](const auto& row) {
        return row.sim_time_s >= disturbance_end_s && row.sim_time_s <= disturbance_end_s + 3.0 &&
               std::abs(row.plant_pitch_deg) < 2.0;
      });
    };
    reject(!recovered_after(2.1) || !recovered_after(10.1), "push_recovery");
  }

  if (drive) {
    constexpr double target_sps = 800.0;
    const auto first_correct_output = [&](double start_s, double sign) {
      return std::any_of(result.rows.begin(), result.rows.end(), [&](const auto& row) {
        return row.sim_time_s >= start_s && row.sim_time_s <= start_s + 0.5 &&
               sign * row.u_sps > 1.0;
      });
    };
    const auto reaches_speed = [&](double start_s, double sign) {
      return std::any_of(result.rows.begin(), result.rows.end(), [&](const auto& row) {
        return row.sim_time_s >= start_s && row.sim_time_s <= start_s + 2.5 &&
               sign * row.measured_vel_sps >= 0.70 * target_sps;
      });
    };
    const double positive_speed = window_mean(
        9.5, 10.8, [](const auto& row) { return row.measured_vel_sps; });
    const double negative_speed = window_mean(
        16.5, 17.8, [](const auto& row) { return row.measured_vel_sps; });
    const auto reaches_stop = [&](double start_s) {
      return std::any_of(result.rows.begin(), result.rows.end(), [&](const auto& row) {
        return row.sim_time_s >= start_s && row.sim_time_s <= start_s + 1.5 &&
               std::abs(row.measured_vel_sps) < 200.0;
      });
    };
    const double symmetric_scale = 0.5 * (std::abs(positive_speed) + std::abs(negative_speed));

    reject(!first_correct_output(8.0, 1.0) || !first_correct_output(15.0, -1.0),
           "drive_direction");
    reject(!reaches_speed(8.0, 1.0) || !reaches_speed(15.0, -1.0), "drive_response");
    reject(!std::isfinite(symmetric_scale) || symmetric_scale <= 0.0 ||
               std::abs(positive_speed + negative_speed) > 0.30 * symmetric_scale,
           "drive_symmetry");
    reject(!reaches_stop(11.0) || !reaches_stop(18.0), "stop_speed");
  }

  acceptance.accepted = acceptance.failures.empty();
  return acceptance;
}
