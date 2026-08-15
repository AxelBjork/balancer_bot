#pragma once

#include <cstdint>
#include <memory>
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

struct SimulatorJoySegment {
  double start_s = 0.0;
  double duration_s = 0.0;
  double forward = 0.0;
  double turn = 0.0;
  double forward_end = 0.0;
  double turn_end = 0.0;
};

struct SimulatorPitchAuthoritySegment {
  double start_s = 0.0;
  double duration_s = 0.0;
  double target_deg = 0.0;
  double com_trim_deg = 0.0;
};

struct SimulatorScenario {
  std::string name;
  double initial_pitch_deg = 0.0;
  double initial_pitch_rate_dps = 0.0;
  double initial_velocity_mps = 0.0;
  double com_angle_offset_rad = 0.0;
  double duration_s = 5.0;
  PhysicsProfile physics_profile = PhysicsProfile::Simplified;
  std::optional<SimulatorPhysics> physics_override;
  std::vector<SimulatorDisturbance> disturbances;
  std::vector<SimulatorJoySegment> joy_segments;
  std::vector<SimulatorPitchAuthoritySegment> pitch_authority_segments;
  double pitch_authority_refresh_dropout_start_s = 0.0;
  double pitch_authority_refresh_dropout_duration_s = 0.0;
  double total_mass_scale = 1.0;
  double pitch_inertia_scale = 1.0;
  double first_mass_moment_scale = 1.0;
  // Additional transport delay after sampling; production estimator filter
  // latency is already present through ImuService and is not included here.
  double imu_pitch_lag_s = 0.0;
  uint32_t imu_noise_seed = 0;
  double accel_noise_std_mps2 = 0.0;
  double gyro_noise_std_rad_s = 0.0;
  // Deterministic sensor-path disturbance for offline controller sensitivity
  // tests. These are not part of the simulator wire request or production
  // hardware model.
  double gyro_pitch_disturbance_frequency_hz = 0.0;
  double gyro_pitch_disturbance_amplitude_rad_s = 0.0;
  double imu_timestamp_jitter_us = 0.0;
  double imu_sample_loss_rate = 0.0;
  std::array<double, 3> accel_bias_mps2{};
  std::array<double, 3> gyro_bias_rad_s{};
  double velocity_estimator_bias_mps = 0.0;
  double velocity_estimator_bias_drift_mps_per_s = 0.0;
  double velocity_estimator_scale = 1.0;
  double velocity_estimator_latency_s = 0.0;
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
  double turn_sps = 0.0;
  double left_sps = 0.0;
  double right_sps = 0.0;
  double nominal_acceleration_mps2 = 0.0;
  double raw_completed_velocity_sps = 0.0;
  double corrected_axle_velocity_sps = 0.0;
  double velocity_control_sps = 0.0;
  double velocity_damping_acceleration_mps2 = 0.0;
  double com_trim_deg = 0.0;
  double trim_learning_enabled = 0.0;
  uint32_t trim_learning_block_reason = ComTrimLearningBlockFault;
  double pitch_error_deg = 0.0;
  double pitch_feedback_sps = 0.0;
  double pitch_rate_feedback_sps = 0.0;
  double pitch_accel_feedback_sps = 0.0;
  double velocity_pitch_target_deg = 0.0;
  double balance_unclamped_sps = 0.0;
  double active_pitch_gain_sps_per_rad = 0.0;
  double active_pitch_rate_gain_sps_per_rad_s = 0.0;
  double active_pitch_accel_gain_sps_per_rad_s2 = 0.0;
  double active_velocity_pitch_gain_rad_per_sps = 0.0;
  double active_velocity_control_cutoff_hz = 0.0;
  double active_velocity_observer_cutoff_hz = 0.0;
  double active_com_trim_gain_deg_per_sps_s = 0.0;
  double active_com_trim_limit_deg = 0.0;
  double active_velocity_pitch_limit_deg = 0.0;
  double active_accel_lpf_hz = 0.0;
  double active_gyro_lpf_hz = 0.0;
  double active_gyro_derivative_lpf_hz = 0.0;
  uint64_t active_config_generation = 0;
  double velocity_pitch_request_unclamped_deg = 0.0;
  double velocity_pitch_request_limited_deg = 0.0;
  double velocity_authority_limited = 0.0;
  double pitch_target_unclamped_deg = 0.0;
  uint32_t pitch_target_limit_reason = PitchTargetLimitNone;
  double trim_trusted = 0.0;
  double trim_learning_allowed = 0.0;
  double trim_quiet_rate_rms_dps = 0.0;
  double pitch_authority_diagnostic_active = 0.0;
  double pitch_authority_diagnostic_target_deg = 0.0;
  double pitch_authority_diagnostic_com_trim_deg = 0.0;
  double pitch_authority_diagnostic_remaining_s = 0.0;
  double pitch_authority_diagnostic_request_id = 0.0;
  double pitch_authority_diagnostic_command_age_ms = 0.0;
  double completed_step_acceleration_sps2 = 0.0;
  double actuator_fault = 0.0;
  double motor_update_dt_ms = 0.0;
  double motor_feedback_age_ms = 0.0;
  uint64_t imu_timestamp_us = 0;
  double left_slewed_sps = 0.0;
  double right_slewed_sps = 0.0;
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
  double desired_drive_force = 0.0;
  double limited_drive_force = 0.0;
  double applied_drive_force = 0.0;
  double desired_tire_force = 0.0;
  double external_force_n = 0.0;
  double external_com_bias_rad = 0.0;
  double x_ddot = 0.0;
  double theta_ddot = 0.0;
  double command_saturated = 0.0;
  uint32_t controller_fault_flags = 0;
  uint32_t controller_saturation_flags = 0;
  uint32_t actuator_saturation_flags = 0;
  double force_saturated = 0.0;
  double phase_saturated = 0.0;
  double motor_force_saturated = 0.0;
  double traction_saturated = 0.0;
  double phase_error_steps = 0.0;
  double missed_steps = 0.0;
  double traction_limit_n = 0.0;
  double motor_force_limit_n = 0.0;
  uint32_t seed = 0;
  double total_mass_scale = 1.0;
  double pitch_inertia_scale = 1.0;
};

struct SimulatorRunResult {
  SimulatorScenario scenario;
  SimulatorPhysics physics;
  std::string pid_config_path;
  std::vector<SimulatorTimelineRow> rows;
  bool fell = false;
  double final_pitch_deg = 0.0;
  double max_abs_pitch_deg = 0.0;
  double tail_rms_pitch_deg = 0.0;
  double max_continuous_saturation_s = 0.0;
  uint32_t actuator_fault_count = 0;
  uint32_t controller_fault_flags = 0;
  uint64_t timeline_hash = 1469598103934665603ULL;
};

struct TransferAcceptance {
  bool accepted = false;
  std::vector<std::string> failures;
};

class SimulatorEngine {
 public:
  explicit SimulatorEngine(const SimulatorScenario& scenario);
  ~SimulatorEngine();
  SimulatorEngine(SimulatorEngine&&) noexcept;
  SimulatorEngine& operator=(SimulatorEngine&&) noexcept;
  SimulatorEngine(const SimulatorEngine&) = delete;
  SimulatorEngine& operator=(const SimulatorEngine&) = delete;

  SimulatorTimelineRow step();
  void set_joystick(double forward, double turn);
  uint64_t current_time_us() const;
  const BalancerSimulator& simulator() const;
  const SimulatorPhysics& physics() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

SimulatorRunResult run_simulator_scenario(const SimulatorScenario& scenario,
                                          const std::string& pid_config_path);
SimulatorRunResult run_simulator_scenario_with_loaded_pid(const SimulatorScenario& scenario);
std::optional<SimulatorScenario> simulator_named_scenario(std::string_view name,
                                                          PhysicsProfile physics_profile);
std::vector<SimulatorScenario> simulator_scenario_set(std::string_view set_name,
                                                      PhysicsProfile physics_profile);
std::vector<SimulatorScenario> transfer_scenario_set();
std::vector<SimulatorScenario> tuning_inner_scenario_set();
std::vector<SimulatorScenario> tuning_authority_scenario_set();
std::vector<SimulatorScenario> tuning_velocity_scenario_set();
std::vector<SimulatorScenario> tuning_drive_scenario_set();
std::vector<SimulatorScenario> tuning_trim_scenario_set();
TransferAcceptance evaluate_transfer_scenario(const SimulatorRunResult& result);
uint64_t update_simulator_timeline_hash(uint64_t hash, const SimulatorTimelineRow& row);
