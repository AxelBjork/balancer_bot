#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "services/control/rate_controller_core.h"
#include "simulator/balancer_simulator.h"

struct SimulatorDisturbance {
  double start_s = 0.0;
  double duration_s = 0.0;
  float forward = 0.0f;
  float turn = 0.0f;
};

struct SimulatorScenario {
  std::string name;
  double initial_pitch_deg = 0.0;
  double com_angle_offset_rad = 0.0;
  double duration_s = 5.0;
  PhysicsProfile physics_profile = PhysicsProfile::Simplified;
  std::optional<SimulatorPhysics> physics_override;
  std::optional<SimulatorDisturbance> disturbance;
};

struct SimulatorTimelineRow {
  double sim_time_s = 0.0;
  double pitch_deg = 0.0;
  double pitch_rate_dps = 0.0;
  double pitch_sp_deg = 0.0;
  double rate_sp_dps = 0.0;
  double u_sps = 0.0;
  double left_sps = 0.0;
  double right_sps = 0.0;
  double vel_error = 0.0;
  double vel_i_term = 0.0;
  double vel_p_term = 0.0;
  double out_norm = 0.0;
  double plant_pitch_deg = 0.0;
  double plant_pitch_rate_dps = 0.0;
  double plant_position = 0.0;
  double plant_velocity = 0.0;
  double target_wheel_velocity = 0.0;
  double actual_wheel_velocity = 0.0;
  double velocity_error = 0.0;
  double f_cmd = 0.0;
  double f_app = 0.0;
  double x_ddot = 0.0;
  double theta_ddot = 0.0;
  double command_saturated = 0.0;
  double force_saturated = 0.0;
};

struct SimulatorRunResult {
  SimulatorScenario scenario;
  SimulatorPhysics physics;
  std::string pid_config_path;
  std::vector<SimulatorTimelineRow> rows;
  bool fell = false;
  double final_pitch_deg = 0.0;
  double max_abs_pitch_deg = 0.0;
};

SimulatorRunResult run_simulator_scenario(const SimulatorScenario& scenario,
                                          const std::string& pid_config_path);
std::optional<SimulatorScenario> simulator_named_scenario(std::string_view name,
                                                          PhysicsProfile physics_profile);
std::vector<SimulatorScenario> simulator_scenario_set(std::string_view set_name,
                                                      PhysicsProfile physics_profile);
