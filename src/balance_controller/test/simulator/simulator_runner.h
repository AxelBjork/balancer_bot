#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "services/control/rate_controller_core.h"
#include "simulator/balancer_simulator.h"

enum class SimulatorDisturbanceKind : uint8_t {
  Step = 0,
  Ramp = 1,
  HoldBias = 2,
};

struct SimulatorDisturbance {
  SimulatorDisturbanceKind kind = SimulatorDisturbanceKind::Step;
  double start_s = 0.0;
  double duration_s = 0.0;
  double force_n = 0.0;
  double com_bias_rad = 0.0;
  double force_n_end = 0.0;
  double com_bias_rad_end = 0.0;
};

struct SimulatorScenario {
  std::string name;
  double initial_pitch_deg = 0.0;
  double com_angle_offset_rad = 0.0;
  double duration_s = 5.0;
  PhysicsProfile physics_profile = PhysicsProfile::Simplified;
  std::optional<SimulatorPhysics> physics_override;
  std::vector<SimulatorDisturbance> disturbances;
  double wheel_slip_factor = 1.0;
  double velocity_feedback_scale = 1.0;
  double velocity_feedback_tau_s = 0.0;
  double imu_pitch_lag_s = 0.0;
  uint32_t imu_noise_seed = 0;
  double accel_noise_std_mps2 = 0.0;
  double gyro_noise_std_rad_s = 0.0;
  std::array<double, 3> accel_bias_mps2{};
  std::array<double, 3> gyro_bias_rad_s{};
};

struct SimulatorTimelineRow {
  double sim_time_s = 0.0;
  double pitch_deg = 0.0;
  double pitch_rate_dps = 0.0;
  double filtered_pitch_rate_dps = 0.0;
  double raw_acc_pitch_deg = 0.0;
  double fused_pitch_deg = 0.0;
  double gyro_pitch_rate_dps = 0.0;
  double pitch_sp_deg = 0.0;
  double u_sps = 0.0;
  double left_sps = 0.0;
  double right_sps = 0.0;
  double vel_error = 0.0;
  double vel_p_term = 0.0;
  double position_target_vel_sps = 0.0;
  double pitch_ref_from_vel_deg = 0.0;
  double pitch_ref_from_pos_deg = 0.0;
  double pitch_error_deg = 0.0;
  double pitch_trim_deg = 0.0;
  double trim_active = 0.0;
  double left_applied_sps = 0.0;
  double right_applied_sps = 0.0;
  double left_actual_steps = 0.0;
  double right_actual_steps = 0.0;
  double plant_pitch_deg = 0.0;
  double plant_pitch_rate_dps = 0.0;
  double plant_position = 0.0;
  double plant_velocity = 0.0;
  double target_wheel_velocity = 0.0;
  double actual_wheel_velocity = 0.0;
  double velocity_error = 0.0;
  double f_cmd = 0.0;
  double f_app = 0.0;
  double external_force_n = 0.0;
  double external_com_bias_rad = 0.0;
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
SimulatorRunResult run_simulator_scenario_with_loaded_pid(const SimulatorScenario& scenario);
std::optional<SimulatorScenario> simulator_named_scenario(std::string_view name,
                                                          PhysicsProfile physics_profile);
std::vector<SimulatorScenario> simulator_scenario_set(std::string_view set_name,
                                                      PhysicsProfile physics_profile);
