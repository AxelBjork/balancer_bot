#include "simulator/simulator_runner.h"

#include <algorithm>
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
  // Acceleration command, not a target wheel speed. This exercises symmetric
  // drive and braking without making the transfer catalog a full-throttle
  // plant-authority test.
  constexpr double drive_command = 0.20;

  enum class TransferProfile { Nominal, Secondary };

  const auto apply_profile = [](SimulatorScenario& scenario, TransferProfile profile) {
    scenario.physics_profile = profile == TransferProfile::Secondary
                                   ? PhysicsProfile::ActuatorStress
                                   : PhysicsProfile::DirectActuator;
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
        .force_n = 0.5,
    });
    scenario.disturbances.push_back(SimulatorDisturbance{
        .kind = SimulatorDisturbanceKind::Step,
        .start_s = 10.0,
        .duration_s = 0.1,
        .force_n = -0.5,
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
      release_scenario("actuator_stress_release", TransferProfile::Secondary, 1.0),
      push_scenario("actuator_stress_push_symmetric", TransferProfile::Secondary),
      drive_scenario("actuator_stress_drive_bidirectional", TransferProfile::Secondary),
  };
}

std::vector<SimulatorScenario> tuning_inner_scenario_set(PhysicsProfile physics_profile) {
  const auto nominal = [physics_profile](std::string name, double pitch_deg,
                                         double pitch_rate_dps) {
    SimulatorScenario scenario;
    scenario.name = std::move(name);
    scenario.duration_s = 4.0;
    scenario.initial_pitch_deg = pitch_deg;
    scenario.initial_pitch_rate_dps = pitch_rate_dps;
    scenario.physics_profile = physics_profile;
    return scenario;
  };
  return {
      nominal("tuning_inner_neutral", 0.0, 0.0),
      nominal("tuning_inner_release_pos", 1.5, 0.0),
      nominal("tuning_inner_release_neg", -1.5, 0.0),
      nominal("tuning_inner_rate_kick_pos", 0.0, 30.0),
      nominal("tuning_inner_rate_kick_neg", 0.0, -30.0),
  };
}

std::vector<SimulatorScenario> tuning_authority_scenario_set(PhysicsProfile physics_profile) {
  const auto release = [physics_profile](std::string name, double pitch_deg) {
    SimulatorScenario scenario;
    scenario.name = std::move(name);
    scenario.duration_s = 5.0;
    scenario.initial_pitch_deg = pitch_deg;
    scenario.physics_profile = physics_profile;
    return scenario;
  };
  const auto push = [physics_profile](std::string name, double force_n) {
    SimulatorScenario scenario;
    scenario.name = std::move(name);
    scenario.duration_s = 5.0;
    scenario.physics_profile = physics_profile;
    scenario.disturbances.push_back(SimulatorDisturbance{
        .kind = SimulatorDisturbanceKind::Step,
        .start_s = 1.0,
        .duration_s = 0.10,
        .force_n = force_n,
    });
    return scenario;
  };
  return {
      release("tuning_authority_release_pos", 6.0),
      release("tuning_authority_release_neg", -6.0),
      // The nominal plant's transfer suite establishes 0.5 N as a moderate,
      // recoverable 100 ms push.  Larger pushes are retained for transfer
      // margins rather than selecting the nominal controller shape.
      push("tuning_authority_push_pos", 0.5),
      push("tuning_authority_push_neg", -0.5),
  };
}

std::vector<SimulatorScenario> tuning_drive_scenario_set(PhysicsProfile physics_profile) {
  SimulatorScenario scenario;
  scenario.name = "tuning_drive_bidirectional";
  scenario.duration_s = 9.0;
  scenario.physics_profile = physics_profile;
  scenario.joy_segments = {
      SimulatorJoySegment{.start_s = 1.0, .duration_s = 2.0, .forward = 0.5, .forward_end = 0.5},
      SimulatorJoySegment{.start_s = 5.0, .duration_s = 2.0, .forward = -0.5, .forward_end = -0.5},
  };
  return {scenario};
}

std::vector<SimulatorScenario> tuning_velocity_scenario_set(PhysicsProfile physics_profile) {
  const auto release = [physics_profile](std::string name, double sign) {
    SimulatorScenario scenario;
    scenario.name = std::move(name);
    scenario.duration_s = 10.0;
    scenario.initial_velocity_mps = sign * 1500.0 * Config::meters_per_step;
    scenario.physics_profile = physics_profile;
    return scenario;
  };
  const auto push = [physics_profile](std::string name, double force_n) {
    SimulatorScenario scenario;
    scenario.name = std::move(name);
    scenario.duration_s = 10.0;
    scenario.physics_profile = physics_profile;
    scenario.disturbances.push_back(SimulatorDisturbance{
        .kind = SimulatorDisturbanceKind::Step,
        .start_s = 2.0,
        .duration_s = 0.10,
        .force_n = force_n,
    });
    return scenario;
  };

  std::vector<SimulatorScenario> scenarios;
  SimulatorScenario neutral;
  neutral.name = "tuning_velocity_neutral";
  neutral.duration_s = 6.0;
  neutral.physics_profile = physics_profile;
  scenarios.push_back(neutral);
  scenarios.push_back(release("tuning_velocity_release_pos", 1.0));
  scenarios.push_back(release("tuning_velocity_release_neg", -1.0));
  scenarios.push_back(push("tuning_velocity_push_pos", 0.5));
  scenarios.push_back(push("tuning_velocity_push_neg", -0.5));

  auto drive = tuning_drive_scenario_set(physics_profile);
  drive.front().name = "tuning_velocity_drive";
  drive.front().duration_s = 10.0;
  drive.front().disturbances.push_back(SimulatorDisturbance{
      .kind = SimulatorDisturbanceKind::Step,
      .start_s = 2.5,
      .duration_s = 0.10,
      .force_n = 0.5,
  });
  scenarios.insert(scenarios.end(), drive.begin(), drive.end());
  return scenarios;
}

std::vector<SimulatorScenario> tuning_motion_scenario_set(PhysicsProfile physics_profile) {
  // These scenarios exercise the planner contract directly. A normalized
  // command of 0.46 is approximately +0.05 m/s with the conservative 0.12
  // m/s user-speed cap; release is represented by the absence of a segment.
  constexpr double kHalfSpeedCommand = 0.46;
  constexpr double kFullSpeedCommand = 0.92;
  const auto motion = [physics_profile](std::string name, double duration_s) {
    SimulatorScenario scenario;
    scenario.name = std::move(name);
    scenario.duration_s = duration_s;
    scenario.physics_profile = physics_profile;
    return scenario;
  };
  auto positive = motion("motion_positive_hold_release", 8.0);
  positive.joy_segments = {{1.0, 3.0, kHalfSpeedCommand, 0.0, kHalfSpeedCommand, 0.0}};

  auto negative = motion("motion_negative_hold_release", 8.0);
  negative.joy_segments = {{1.0, 3.0, -kHalfSpeedCommand, 0.0, -kHalfSpeedCommand, 0.0}};

  auto reversal = motion("motion_reversal_through_zero", 9.0);
  reversal.joy_segments = {
      {1.0, 2.5, kHalfSpeedCommand, 0.0, kHalfSpeedCommand, 0.0},
      {5.0, 2.5, -kHalfSpeedCommand, 0.0, -kHalfSpeedCommand, 0.0},
  };

  auto ramp = motion("motion_analog_ramp", 8.0);
  ramp.joy_segments = {{1.0, 4.0, 0.0, 0.0, kHalfSpeedCommand, 0.0}};

  auto high_speed = motion("motion_high_speed_hold_release", 8.0);
  high_speed.joy_segments = {{1.0, 2.5, kFullSpeedCommand, 0.0, kFullSpeedCommand, 0.0}};

  auto translating_push = motion("motion_push_while_translating", 9.0);
  translating_push.joy_segments = {{1.0, 5.0, kHalfSpeedCommand, 0.0, kHalfSpeedCommand, 0.0}};
  translating_push.disturbances.push_back(SimulatorDisturbance{
      .kind = SimulatorDisturbanceKind::Step,
      .start_s = 3.0,
      .duration_s = 0.10,
      .force_n = 0.5,
  });

  auto signed_push = motion("motion_negative_push_while_translating", 9.0);
  signed_push.joy_segments = {{1.0, 5.0, -kHalfSpeedCommand, 0.0, -kHalfSpeedCommand, 0.0}};
  signed_push.disturbances.push_back(SimulatorDisturbance{
      .kind = SimulatorDisturbanceKind::Step,
      .start_s = 3.0,
      .duration_s = 0.10,
      .force_n = -0.5,
  });

  return {positive, negative, reversal, ramp, high_speed, translating_push, signed_push};
}

std::vector<SimulatorScenario> tuning_outer_motion_scenario_set(
    PhysicsProfile physics_profile, std::optional<double> cart_damping_override) {
  // This set is intentionally expressed in physical target velocities. The
  // simulator still receives the same normalized joystick command as the
  // production path, but the conversion below compensates for the configured
  // deadzone so that the planner's v_user is exactly the requested value when
  // drive_max_velocity_mps is the fixed 0.12 m/s tuning surface.
  constexpr double kUserSpeedMps = 0.12;
  constexpr double kDeadzone = Config::deadzone;
  const auto command_for_velocity = [](double velocity_mps) {
    if (std::abs(velocity_mps) <= 1e-12) return 0.0;
    const double magnitude = std::clamp(std::abs(velocity_mps) / kUserSpeedMps, 0.0, 1.0);
    return std::copysign(kDeadzone + magnitude * (1.0 - kDeadzone), velocity_mps);
  };
  // Match the existing fixed Python outer-loop behavioral surface while
  // obtaining every other parameter from the selected profile. This does not
  // alter StepperPhaseElectrical physics or duplicate its constants.
  std::optional<SimulatorPhysics> fixed_outer_physics;
  if (cart_damping_override.has_value()) {
    fixed_outer_physics = BalancerSimulator::physics_for_profile(physics_profile);
    fixed_outer_physics->cart_damping = std::max(0.0, *cart_damping_override);
  }
  const auto motion = [physics_profile, &fixed_outer_physics](std::string name,
                                                               double duration_s) {
    SimulatorScenario scenario;
    scenario.name = std::move(name);
    scenario.duration_s = duration_s;
    scenario.physics_profile = physics_profile;
    scenario.physics_override = fixed_outer_physics;
    return scenario;
  };
  const auto held = [&](std::string name, double target_mps, double duration_s,
                        double hold_duration_s) {
    auto scenario = motion(std::move(name), duration_s);
    scenario.joy_segments = {{1.0, hold_duration_s, command_for_velocity(target_mps), 0.0,
                              command_for_velocity(target_mps), 0.0}};
    return scenario;
  };

  auto positive_005 = held("motion_target_pos_005_release", 0.05, 12.0, 5.0);
  auto negative_005 = held("motion_target_neg_005_release", -0.05, 12.0, 5.0);
  auto positive_010 = held("motion_target_pos_010_release", 0.10, 14.0, 6.0);
  auto negative_010 = held("motion_target_neg_010_release", -0.10, 14.0, 6.0);

  auto reversal = motion("motion_target_reversal_pos010_neg010", 18.0);
  reversal.joy_segments = {
      {1.0, 4.0, command_for_velocity(0.10), 0.0, command_for_velocity(0.10), 0.0},
      {9.0, 5.0, command_for_velocity(-0.10), 0.0, command_for_velocity(-0.10), 0.0},
  };

  // Preserve the timing and command shape of the fixed behavioral drive/stop
  // surface, without changing the plant or its scenario assertions.
  auto drive_stop = motion("motion_fixed_drive_stop", 70.0);
  drive_stop.physics_override = fixed_outer_physics;
  drive_stop.joy_segments = {
      {2.0, 12.0, 0.45, 0.0, 0.45, 0.0},
      {12.0, 2.0, -0.55, 0.0, -0.55, 0.0},
  };

  auto push_positive = held("motion_target_pos_005_push", 0.05, 12.0, 8.0);
  push_positive.disturbances.push_back({
      .kind = SimulatorDisturbanceKind::Step, .start_s = 4.0, .duration_s = 0.15,
      .force_n = 0.5});
  auto push_negative = held("motion_target_neg_005_push", -0.05, 12.0, 8.0);
  push_negative.disturbances.push_back({
      .kind = SimulatorDisturbanceKind::Step, .start_s = 4.0, .duration_s = 0.15,
      .force_n = -0.5});

  auto initial_015 = motion("motion_initial_velocity_pos1500", 18.0);
  initial_015.physics_override = fixed_outer_physics;
  initial_015.initial_velocity_mps = 1500.0 * Config::meters_per_step;
  auto initial_neg_015 = motion("motion_initial_velocity_neg1500", 18.0);
  initial_neg_015.physics_override = fixed_outer_physics;
  initial_neg_015.initial_velocity_mps = -1500.0 * Config::meters_per_step;
  auto initial_035 = motion("motion_initial_velocity_pos3500", 24.0);
  initial_035.physics_override = fixed_outer_physics;
  initial_035.initial_velocity_mps = 3500.0 * Config::meters_per_step;
  auto initial_neg_035 = motion("motion_initial_velocity_neg3500", 24.0);
  initial_neg_035.physics_override = fixed_outer_physics;
  initial_neg_035.initial_velocity_mps = -3500.0 * Config::meters_per_step;

  // Compact guards keep the motion optimizer from trading away the known
  // attitude envelope. They remain ordinary simulator scenarios and do not
  // alter any fixed behavioral assertion.
  auto neutral = motion("guard_neutral_balance", 8.0);
  auto pitch_pos_1 = motion("guard_pitch_pos1", 8.0);
  pitch_pos_1.initial_pitch_deg = 1.0;
  auto pitch_neg_1 = motion("guard_pitch_neg1", 8.0);
  pitch_neg_1.initial_pitch_deg = -1.0;
  auto pitch_pos_4 = motion("guard_pitch_pos4", 10.0);
  pitch_pos_4.initial_pitch_deg = 4.0;
  auto pitch_neg_4 = motion("guard_pitch_neg4", 10.0);
  pitch_neg_4.initial_pitch_deg = -4.0;
  auto guard_push = motion("guard_push_recovery", 12.0);
  guard_push.disturbances.push_back({
      .kind = SimulatorDisturbanceKind::Step, .start_s = 3.0, .duration_s = 0.15,
      .force_n = 0.5});

  return {positive_005, negative_005, positive_010, negative_010, reversal, drive_stop,
          push_positive, push_negative, initial_015, initial_neg_015, initial_035,
          initial_neg_035, neutral, pitch_pos_1, pitch_neg_1, pitch_pos_4, pitch_neg_4,
          guard_push};
}

std::vector<SimulatorScenario> tuning_leaky_integral_scenario_set(
    PhysicsProfile physics_profile, std::optional<double> cart_damping_override) {
  // This is deliberately narrower than the broad outer-motion surface. It
  // gives a bounded integral several seconds of real hold time to accumulate
  // while keeping the planner, speed cap, and authority fixed for the P/I
  // comparison.
  constexpr double kUserSpeedMps = 0.12;
  constexpr double kDeadzone = Config::deadzone;
  const auto command_for_velocity = [](double velocity_mps) {
    if (std::abs(velocity_mps) <= 1e-12) return 0.0;
    const double magnitude = std::clamp(std::abs(velocity_mps) / kUserSpeedMps, 0.0, 1.0);
    return std::copysign(kDeadzone + magnitude * (1.0 - kDeadzone), velocity_mps);
  };

  std::optional<SimulatorPhysics> fixed_outer_physics;
  if (cart_damping_override.has_value()) {
    fixed_outer_physics = BalancerSimulator::physics_for_profile(physics_profile);
    fixed_outer_physics->cart_damping = std::max(0.0, *cart_damping_override);
  }
  const auto motion = [physics_profile, &fixed_outer_physics](std::string name,
                                                               double duration_s) {
    SimulatorScenario scenario;
    scenario.name = std::move(name);
    scenario.duration_s = duration_s;
    scenario.physics_profile = physics_profile;
    scenario.physics_override = fixed_outer_physics;
    return scenario;
  };
  const auto held = [&](std::string name, double target_mps, double duration_s,
                        double hold_duration_s) {
    auto scenario = motion(std::move(name), duration_s);
    scenario.joy_segments = {{1.0, hold_duration_s, command_for_velocity(target_mps), 0.0,
                              command_for_velocity(target_mps), 0.0}};
    return scenario;
  };

  auto positive_005 = held("motion_pi_pos005_hold_release", 0.05, 14.0, 8.0);
  auto negative_005 = held("motion_pi_neg005_hold_release", -0.05, 14.0, 8.0);
  auto positive_010 = held("motion_pi_pos010_hold_release", 0.10, 16.0, 8.0);
  auto negative_010 = held("motion_pi_neg010_hold_release", -0.10, 16.0, 8.0);
  auto long_positive = held("motion_pi_long_pos005_hold_release", 0.05, 28.0, 20.0);
  auto long_negative = held("motion_pi_long_neg005_hold_release", -0.05, 28.0, 20.0);

  auto reversal = motion("motion_pi_reversal_pos010_neg010", 24.0);
  reversal.joy_segments = {
      {1.0, 6.0, command_for_velocity(0.10), 0.0, command_for_velocity(0.10), 0.0},
      {11.0, 7.0, command_for_velocity(-0.10), 0.0, command_for_velocity(-0.10), 0.0},
  };
  auto reverse_reversal = motion("motion_pi_reversal_neg010_pos010", 24.0);
  reverse_reversal.joy_segments = {
      {1.0, 6.0, command_for_velocity(-0.10), 0.0, command_for_velocity(-0.10), 0.0},
      {11.0, 7.0, command_for_velocity(0.10), 0.0, command_for_velocity(0.10), 0.0},
  };

  auto push_positive = held("motion_pi_pos005_push", 0.05, 14.0, 10.0);
  push_positive.disturbances.push_back({
      .kind = SimulatorDisturbanceKind::Step, .start_s = 4.0, .duration_s = 0.15,
      .force_n = 0.5});
  auto push_negative = held("motion_pi_neg005_push", -0.05, 14.0, 10.0);
  push_negative.disturbances.push_back({
      .kind = SimulatorDisturbanceKind::Step, .start_s = 4.0, .duration_s = 0.15,
      .force_n = -0.5});

  auto neutral = motion("guard_pi_neutral_balance", 8.0);
  auto pitch_pos_1 = motion("guard_pi_pitch_pos1", 8.0);
  pitch_pos_1.initial_pitch_deg = 1.0;
  auto pitch_neg_1 = motion("guard_pi_pitch_neg1", 8.0);
  pitch_neg_1.initial_pitch_deg = -1.0;
  auto pitch_pos_4 = motion("guard_pi_pitch_pos4", 10.0);
  pitch_pos_4.initial_pitch_deg = 4.0;
  auto pitch_neg_4 = motion("guard_pi_pitch_neg4", 10.0);
  pitch_neg_4.initial_pitch_deg = -4.0;
  auto guard_push = motion("guard_pi_push_recovery", 12.0);
  guard_push.disturbances.push_back({
      .kind = SimulatorDisturbanceKind::Step, .start_s = 3.0, .duration_s = 0.15,
      .force_n = 0.5});

  return {positive_005, negative_005, positive_010, negative_010, long_positive,
          long_negative, reversal, reverse_reversal, push_positive, push_negative, neutral,
          pitch_pos_1, pitch_neg_1, pitch_pos_4, pitch_neg_4, guard_push};
}

std::vector<SimulatorScenario> tuning_distance_scenario_set(
    PhysicsProfile physics_profile, std::optional<double> cart_damping_override) {
  // This keeps the existing fixed full-forward command timing: one second of
  // quiet startup followed by five seconds of full command. The Python test
  // uses the same cart-damping override. The distance-only tuner retains two
  // seconds after release for diagnostic context, but scores only the signed
  // position change over [1, 6] seconds and does not inherit release gates.
  std::optional<SimulatorPhysics> physics;
  if (cart_damping_override.has_value()) {
    physics = BalancerSimulator::physics_for_profile(physics_profile);
    physics->cart_damping = std::max(0.0, *cart_damping_override);
  }

  SimulatorScenario scenario;
  scenario.name = "distance_full_forward_then_stop";
  scenario.duration_s = 8.0;
  scenario.physics_profile = physics_profile;
  scenario.physics_override = physics;
  scenario.joy_segments = {
      SimulatorJoySegment{.start_s = 1.0,
                           .duration_s = 5.0,
                           .forward = 1.0,
                           .turn = 0.0,
                           .forward_end = 1.0,
                           .turn_end = 0.0},
  };
  return {scenario};
}

std::vector<SimulatorScenario> tuning_speed_envelope_scenario_set(
    PhysicsProfile physics_profile, std::optional<double> cart_damping_override) {
  // Analog-like 0 -> 0.5 m/s -> 0 command.  With the 0.5 m/s drive cap this
  // trajectory has 0.5 m of ramp-in travel, 9 m of hold travel, and 0.5 m of
  // ramp-out travel: approximately 10 m of reference displacement.  It is a
  // benchmark for the electrical envelope, not a production acceptance gate.
  std::optional<SimulatorPhysics> physics;
  if (cart_damping_override.has_value()) {
    physics = BalancerSimulator::physics_for_profile(physics_profile);
    physics->cart_damping = std::max(0.0, *cart_damping_override);
  }

  SimulatorScenario scenario;
  scenario.name = "speed_envelope_05mps_ramp_hold_stop";
  scenario.duration_s = 28.0;
  scenario.physics_profile = physics_profile;
  scenario.physics_override = physics;
  scenario.joy_segments = {
      SimulatorJoySegment{.start_s = 1.0,
                           .duration_s = 2.0,
                           .forward = 0.0,
                           .turn = 0.0,
                           .forward_end = 1.0,
                           .turn_end = 0.0},
      SimulatorJoySegment{.start_s = 3.0,
                           .duration_s = 18.0,
                           .forward = 1.0,
                           .turn = 0.0,
                           .forward_end = 1.0,
                           .turn_end = 0.0},
      SimulatorJoySegment{.start_s = 21.0,
                           .duration_s = 2.0,
                           .forward = 1.0,
                           .turn = 0.0,
                           .forward_end = 0.0,
                           .turn_end = 0.0},
  };
  return {scenario};
}

std::vector<SimulatorScenario> tuning_trim_scenario_set(PhysicsProfile physics_profile) {
  const auto trim = [physics_profile](std::string name, double offset_rad) {
    SimulatorScenario scenario;
    scenario.name = std::move(name);
    scenario.duration_s = 15.0;
    scenario.com_angle_offset_rad = offset_rad;
    scenario.physics_profile = physics_profile;
    return scenario;
  };
  return {trim("tuning_trim_pos", 0.002), trim("tuning_trim_neg", -0.002)};
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
        std::isfinite(row.u_sps) && std::isfinite(row.corrected_axle_velocity_sps);
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
        end_s - 2.0, end_s, [](const auto& row) { return row.corrected_axle_velocity_sps; });
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
    // The drive segment is deliberately a 0.20 normalized command. Its
    // reference speed is now explicit SI configuration rather than an
    // acceleration/damping equilibrium inferred from legacy fields.
    const double normalized_drive =
        std::clamp((0.20 - Config::deadzone) / (1.0 - Config::deadzone), 0.0, 1.0);
    const double expected_speed_sps =
        normalized_drive * ConfigPid::values.drive_max_velocity_mps / Config::meters_per_step;
    const double target_sps = std::clamp(0.70 * expected_speed_sps, 50.0, 800.0);
    const auto first_correct_output = [&](double start_s, double sign) {
      return std::any_of(result.rows.begin(), result.rows.end(), [&](const auto& row) {
        return row.sim_time_s >= start_s && row.sim_time_s <= start_s + 0.5 &&
               sign * row.u_sps > 1.0;
      });
    };
    const auto reaches_speed = [&](double start_s, double sign) {
      return std::any_of(result.rows.begin(), result.rows.end(), [&](const auto& row) {
        return row.sim_time_s >= start_s && row.sim_time_s <= start_s + 2.5 &&
               sign * row.corrected_axle_velocity_sps >= 0.70 * target_sps;
      });
    };
    const double positive_speed = window_mean(
        9.5, 10.8, [](const auto& row) { return row.corrected_axle_velocity_sps; });
    const double negative_speed = window_mean(
        16.5, 17.8, [](const auto& row) { return row.corrected_axle_velocity_sps; });
    const auto reaches_stop = [&](double start_s) {
      return std::any_of(result.rows.begin(), result.rows.end(), [&](const auto& row) {
        return row.sim_time_s >= start_s && row.sim_time_s <= start_s + 1.5 &&
               std::abs(row.corrected_axle_velocity_sps) < 200.0;
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
