#include "simulator/simulator_runner.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <random>
#include <stdexcept>
#include <string>

#include "messages/types.h"
#include "services/imu/pitch_lpf.h"
#include "services/main/config.h"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kFallPitchDeg = 75.0;
constexpr double kTickDtS = 1.0 / 400.0;

struct DisturbanceSample {
  double force_n = 0.0;
  double com_bias_rad = 0.0;
};

DisturbanceSample disturbance_sample(const SimulatorDisturbance& disturbance, double sim_time_s) {
  if (sim_time_s < disturbance.start_s) {
    return {};
  }

  switch (disturbance.kind) {
    case SimulatorDisturbanceKind::Step: {
      if (disturbance.duration_s > 0.0 &&
          sim_time_s < (disturbance.start_s + disturbance.duration_s)) {
        return DisturbanceSample{
            .force_n = disturbance.force_n,
            .com_bias_rad = disturbance.com_bias_rad,
        };
      }
      break;
    }
    case SimulatorDisturbanceKind::Ramp: {
      if (disturbance.duration_s > 0.0 &&
          sim_time_s < (disturbance.start_s + disturbance.duration_s)) {
        const double progress =
            std::clamp((sim_time_s - disturbance.start_s) / disturbance.duration_s, 0.0, 1.0);
        return DisturbanceSample{
            .force_n =
                disturbance.force_n + (disturbance.force_n_end - disturbance.force_n) * progress,
            .com_bias_rad = disturbance.com_bias_rad +
                            (disturbance.com_bias_rad_end - disturbance.com_bias_rad) * progress,
        };
      }
      break;
    }
    case SimulatorDisturbanceKind::HoldBias: {
      const bool active = disturbance.duration_s <= 0.0 ||
                          sim_time_s < (disturbance.start_s + disturbance.duration_s);
      if (active) {
        return DisturbanceSample{
            .force_n = disturbance.force_n,
            .com_bias_rad = disturbance.com_bias_rad,
        };
      }
      break;
    }
  }

  return {};
}

DisturbanceSample scenario_disturbance_for_time(const SimulatorScenario& scenario,
                                                double sim_time_s) {
  DisturbanceSample total{};
  for (const auto& disturbance : scenario.disturbances) {
    const DisturbanceSample sample = disturbance_sample(disturbance, sim_time_s);
    total.force_n += sample.force_n;
    total.com_bias_rad += sample.com_bias_rad;
  }
  return total;
}

double raw_acc_pitch_deg(const std::array<double, 3>& acc) {
  return std::atan2(-acc[0], std::sqrt(acc[1] * acc[1] + acc[2] * acc[2])) * 180.0 / kPi;
}

class SimImuPipeline {
 public:
  explicit SimImuPipeline(const SimulatorScenario& scenario)
      : rng_(scenario.imu_noise_seed),
        accel_noise_std_(scenario.accel_noise_std_mps2),
        gyro_noise_std_(scenario.gyro_noise_std_rad_s),
        accel_noise_(0.0,
                     scenario.accel_noise_std_mps2 > 0.0 ? scenario.accel_noise_std_mps2 : 1.0),
        gyro_noise_(0.0,
                    scenario.gyro_noise_std_rad_s > 0.0 ? scenario.gyro_noise_std_rad_s : 1.0) {
  }

  ipc::ImuSamplePayload sample(const BalancerSimulator& sim, uint64_t sim_time_us) {
    auto payload = sim.make_imu_payload(sim_time_us);
    if (accel_noise_std_ > 0.0) {
      for (int i = 0; i < 3; ++i) {
        payload.acc[i] += accel_noise_(rng_);
      }
    }
    if (gyro_noise_std_ > 0.0) {
      for (int i = 0; i < 3; ++i) {
        payload.gyr[i] += gyro_noise_(rng_);
      }
    }
    return payload;
  }

 private:
  std::mt19937 rng_;
  double accel_noise_std_;
  double gyro_noise_std_;
  std::normal_distribution<double> accel_noise_;
  std::normal_distribution<double> gyro_noise_;
};

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
  BalancerSimulator::Config sim_cfg;
  sim_cfg.com_angle_offset_rad = scenario.com_angle_offset_rad;
  sim_cfg.initial_pitch_deg = scenario.initial_pitch_deg;
  sim_cfg.physics_profile = scenario.physics_profile;
  sim_cfg.physics_override = scenario.physics_override;
  sim_cfg.wheel_slip_factor = scenario.wheel_slip_factor;
  sim_cfg.velocity_feedback_scale = scenario.velocity_feedback_scale;
  sim_cfg.velocity_feedback_tau_s = scenario.velocity_feedback_tau_s;
  sim_cfg.velocity_feedback_model = scenario.velocity_feedback_model;
  sim_cfg.imu_pitch_lag_s = scenario.imu_pitch_lag_s;
  sim_cfg.imu_noise_seed = scenario.imu_noise_seed;
  sim_cfg.accel_noise_std_mps2 = scenario.accel_noise_std_mps2;
  sim_cfg.gyro_noise_std_rad_s = scenario.gyro_noise_std_rad_s;
  sim_cfg.accel_bias_mps2 = scenario.accel_bias_mps2;
  sim_cfg.gyro_bias_rad_s = scenario.gyro_bias_rad_s;

  BalancerSimulator sim(sim_cfg);
  SimImuPipeline imu_pipeline(scenario);
  RateControllerCore core;

  double left_sps = 0.0;
  double right_sps = 0.0;
  double left_actual_steps = 0.0;
  double right_actual_steps = 0.0;
  Telemetry latest_telemetry{};
  bool have_telemetry = false;

  core.setMotorOutputs([&](double left, double right) {
    left_sps = left;
    right_sps = right;
    sim.set_motor_targets(left, right);
  });
  core.setJoystick(JoyCmd{0.0f, 0.0f});
  core.setTelemetrySink([&](const Telemetry& t) {
    latest_telemetry = t;
    have_telemetry = true;
  });

  SimulatorRunResult result;
  result.scenario = scenario;
  result.physics = sim.physics();
  result.rows.reserve(static_cast<size_t>(scenario.duration_s * Config::control_hz));

  const int steps = std::max(1, static_cast<int>(std::llround(scenario.duration_s / kTickDtS)));
  uint64_t sim_time_us = 0;
  result.max_abs_pitch_deg = std::abs(sim.get_pitch()) * 180.0 / kPi;

  for (int i = 0; i < steps; ++i) {
    const double sim_time_s = static_cast<double>(sim_time_us) / 1e6;
    const DisturbanceSample disturbance = scenario_disturbance_for_time(scenario, sim_time_s);
    sim.set_external_force_n(disturbance.force_n);
    sim.set_external_com_bias_rad(disturbance.com_bias_rad);

    sim.step(kTickDtS);
    sim_time_us += static_cast<uint64_t>(kTickDtS * 1e6);
    const auto imu = imu_pipeline.sample(sim, sim_time_us);

    ImuSample sample{};
    sample.angle_rad = imu.pitch_rad;
    sample.gyro_rad_s = imu.pitch_rate_rad_s;
    sample.pitch_accel_rad_s2 = imu.pitch_accel_rad_s2;
    sample.yaw_rate_z = imu.gyr[2];
    sample.t = std::chrono::steady_clock::time_point(std::chrono::microseconds(imu.timestamp_us));

    core.pushImu(sample);
    core.updateOuterLoop(sim.get_corrected_axle_speed_sps(), kTickDtS);
    core.step(kTickDtS, sample.t);
    left_actual_steps += left_sps * kTickDtS;
    right_actual_steps += right_sps * kTickDtS;

    const auto& diag = sim.diagnostics();
    SimulatorTimelineRow row{};
    row.sim_time_s = static_cast<double>(sim_time_us) / 1e6;
    row.left_sps = left_sps;
    row.right_sps = right_sps;
    row.plant_pitch_deg = sim.get_pitch() * 180.0 / kPi;
    row.plant_pitch_rate_dps = sim.state().pitch_rate * 180.0 / kPi;
    row.plant_position = sim.state().position;
    row.plant_velocity = sim.state().velocity;
    row.raw_feedback_sps = sim.get_raw_feedback_sps();
    row.corrected_feedback_sps = sim.get_corrected_axle_speed_sps();
    row.target_wheel_velocity = diag.target_wheel_velocity;
    row.actual_wheel_velocity = diag.actual_wheel_velocity;
    row.velocity_error = diag.velocity_error;
    row.f_cmd = diag.f_cmd;
    row.f_app = diag.f_app;
    row.x_ddot = diag.x_ddot;
    row.theta_ddot = diag.theta_ddot;
    row.command_saturated =
        (std::abs(left_sps) >= (0.99 * kMaxSps) || std::abs(right_sps) >= (0.99 * kMaxSps)) ? 1.0
                                                                                            : 0.0;
    row.force_saturated = diag.command_saturated ? 1.0 : 0.0;
    if (have_telemetry) {
      row.pitch_deg = latest_telemetry.pitch_deg;
      row.pitch_rate_dps = latest_telemetry.pitch_rate_dps;
      row.filtered_pitch_rate_dps = latest_telemetry.filtered_pitch_rate_dps;
      row.raw_acc_pitch_deg = raw_acc_pitch_deg(imu.acc);
      row.fused_pitch_deg = imu.pitch_rad * 180.0 / kPi;
      row.gyro_pitch_rate_dps = imu.gyr[1] * 180.0 / kPi;
      row.pitch_sp_deg = latest_telemetry.pitch_sp_deg;
      row.u_sps = latest_telemetry.u_sps;
      row.vel_error = latest_telemetry.vel_error;
      row.vel_p_term = latest_telemetry.vel_p_term;
      row.pitch_ref_from_vel_deg = latest_telemetry.pitch_ref_from_vel_deg;
      row.pitch_trim_deg = latest_telemetry.pitch_trim_deg;
      row.trim_active = latest_telemetry.trim_active;
    }
    row.left_applied_sps = left_sps;
    row.right_applied_sps = right_sps;
    row.left_actual_steps = left_actual_steps;
    row.right_actual_steps = right_actual_steps;
    row.external_force_n = diag.external_force_n;
    row.external_com_bias_rad = diag.external_com_bias_rad;
    result.rows.push_back(row);

    result.max_abs_pitch_deg = std::max(result.max_abs_pitch_deg, std::abs(row.plant_pitch_deg));
  }

  result.final_pitch_deg = sim.get_pitch() * 180.0 / kPi;
  result.fell = result.max_abs_pitch_deg > kFallPitchDeg;
  return result;
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
    scenario.velocity_feedback_scale = 0.85;
    scenario.velocity_feedback_tau_s = 0.10;
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
    scenario.velocity_feedback_scale = 0.85;
    scenario.velocity_feedback_tau_s = 0.10;
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
