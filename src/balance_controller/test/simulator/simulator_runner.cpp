#include "simulator/simulator_runner.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>

#include "config.h"
#include "types.h"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kFallPitchDeg = 75.0;
constexpr double kTickDtS = 1.0 / 400.0;

JoyCmd scenario_joystick_for_time(const SimulatorScenario& scenario, double sim_time_s) {
  if (!scenario.disturbance.has_value()) {
    return JoyCmd{0.0f, 0.0f};
  }

  const auto& d = *scenario.disturbance;
  if (sim_time_s >= d.start_s && sim_time_s < (d.start_s + d.duration_s)) {
    return JoyCmd{d.forward, d.turn};
  }
  return JoyCmd{0.0f, 0.0f};
}

SimulatorScenario make_scenario(std::string name,
                                double initial_pitch_deg,
                                double com_angle_offset_rad,
                                PhysicsProfile physics_profile) {
  SimulatorScenario scenario;
  scenario.name = std::move(name);
  scenario.initial_pitch_deg = initial_pitch_deg;
  scenario.com_angle_offset_rad = com_angle_offset_rad;
  scenario.physics_profile = physics_profile;
  return scenario;
}

}  // namespace

SimulatorRunResult run_simulator_scenario(const SimulatorScenario& scenario,
                                          const std::string& pid_config_path) {
  ConfigPid::load(pid_config_path);

  BalancerSimulator::Config sim_cfg;
  sim_cfg.com_angle_offset_rad = scenario.com_angle_offset_rad;
  sim_cfg.initial_pitch_deg = scenario.initial_pitch_deg;
  sim_cfg.physics_profile = scenario.physics_profile;
  sim_cfg.physics_override = scenario.physics_override;

  BalancerSimulator sim(sim_cfg);
  RateControllerCore core;

  float left_sps = 0.0f;
  float right_sps = 0.0f;
  Telemetry latest_telemetry{};
  bool have_telemetry = false;

  core.setMotorOutputs([&](float left, float right) {
    left_sps = left;
    right_sps = right;
    sim.set_motor_targets(left, right);
  });
  core.setVelocityFeedback([&]() { return sim.get_actual_speed_sps(); });
  core.setPositionFeedback([&]() { return static_cast<float>(sim.get_position()); });
  core.setTelemetrySink([&](const Telemetry& t) {
    latest_telemetry = t;
    have_telemetry = true;
  });

  SimulatorRunResult result;
  result.scenario = scenario;
  result.physics = sim.physics();
  result.pid_config_path = pid_config_path;
  result.rows.reserve(static_cast<size_t>(scenario.duration_s * Config::control_hz));

  const int steps = std::max(1, static_cast<int>(std::llround(scenario.duration_s / kTickDtS)));
  uint64_t sim_time_us = 0;
  result.max_abs_pitch_deg = std::abs(sim.get_pitch()) * 180.0 / kPi;

  for (int i = 0; i < steps; ++i) {
    const double sim_time_s = static_cast<double>(sim_time_us) / 1e6;
    core.setJoystick(scenario_joystick_for_time(scenario, sim_time_s));

    sim.step(kTickDtS);
    sim_time_us += static_cast<uint64_t>(kTickDtS * 1e6);
    const auto imu = sim.make_imu_payload(sim_time_us);

    ImuSample sample{};
    sample.angle_rad = imu.pitch_rad;
    sample.gyro_rad_s = imu.gyr[1];
    sample.yaw_rate_z = imu.gyr[2];
    sample.t = std::chrono::steady_clock::time_point(std::chrono::microseconds(imu.timestamp_us));

    core.pushImu(sample);
    core.step(kTickDtS, sample.t);

    const auto& diag = sim.diagnostics();
    SimulatorTimelineRow row{};
    row.sim_time_s = static_cast<double>(sim_time_us) / 1e6;
    row.left_sps = left_sps;
    row.right_sps = right_sps;
    row.plant_pitch_deg = sim.get_pitch() * 180.0 / kPi;
    row.plant_pitch_rate_dps = sim.state().pitch_rate * 180.0 / kPi;
    row.plant_position = sim.state().position;
    row.plant_velocity = sim.state().velocity;
    row.target_wheel_velocity = diag.target_wheel_velocity;
    row.actual_wheel_velocity = diag.actual_wheel_velocity;
    row.velocity_error = diag.velocity_error;
    row.f_cmd = diag.f_cmd;
    row.f_app = diag.f_app;
    row.x_ddot = diag.x_ddot;
    row.theta_ddot = diag.theta_ddot;
    row.command_saturated = (std::abs(left_sps) >= (0.99 * kMaxSps) ||
                             std::abs(right_sps) >= (0.99 * kMaxSps))
                                ? 1.0
                                : 0.0;
    row.force_saturated = diag.command_saturated ? 1.0 : 0.0;
    if (have_telemetry) {
      row.pitch_deg = latest_telemetry.pitch_deg;
      row.pitch_rate_dps = latest_telemetry.pitch_rate_dps;
      row.pitch_sp_deg = latest_telemetry.pitch_sp_deg;
      row.rate_sp_dps = latest_telemetry.rate_sp_dps;
      row.u_sps = latest_telemetry.u_sps;
      row.vel_error = latest_telemetry.vel_error;
      row.vel_i_term = latest_telemetry.vel_i_term;
      row.vel_p_term = latest_telemetry.vel_p_term;
      row.out_norm = latest_telemetry.out_norm;
    }
    result.rows.push_back(row);

    result.max_abs_pitch_deg = std::max(result.max_abs_pitch_deg, std::abs(row.plant_pitch_deg));
  }

  result.final_pitch_deg = sim.get_pitch() * 180.0 / kPi;
  result.fell = result.max_abs_pitch_deg > kFallPitchDeg;
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
    scenario.disturbance = SimulatorDisturbance{
        .start_s = 1.0,
        .duration_s = 0.20,
        .forward = 0.08f,
        .turn = 0.0f,
    };
    return scenario;
  }
  return std::nullopt;
}

std::vector<SimulatorScenario> simulator_scenario_set(std::string_view set_name,
                                                      PhysicsProfile physics_profile) {
  static constexpr std::array<std::string_view, 5> kRequired = {
      "neutral_hold",
      "pitch_bias_pos",
      "pitch_bias_neg",
      "com_offset_pos",
      "com_offset_neg",
  };
  static constexpr std::array<std::string_view, 5> kCapability = {
      "combined_bias_pos",
      "combined_bias_neg",
      "recovery_large_pitch_pos",
      "recovery_large_pitch_neg",
      "disturbance_pulse",
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
    return scenarios;
  }

  throw std::runtime_error("Unknown simulator scenario set: " + std::string(set_name));
}
