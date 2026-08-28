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
  // Simulator-only estimator fixture. Zero preserves normal startup, where
  // the first gravity sample seeds fused pitch at zero.
  double initial_fused_pitch_deg = 0.0;
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
  double user_velocity_mps = 0.0;
  double reference_velocity_mps = 0.0;
  double reference_acceleration_mps2 = 0.0;
  double reference_jerk_mps3 = 0.0;
  double velocity_feedback_estimate_mps = 0.0;
  double velocity_error_mps = 0.0;
  double velocity_feedback_acceleration_mps2 = 0.0;
  double velocity_p_acceleration_mps2 = 0.0;
  double velocity_i_acceleration_mps2 = 0.0;
  double velocity_integral_state_mps_s = 0.0;
  double acceleration_raw_mps2 = 0.0;
  double acceleration_cmd_mps2 = 0.0;
  double drive_pitch_target_deg = 0.0;
  double fixed_com_trim_deg = 0.0;
  double velocity_feedback_valid = 0.0;
  double velocity_feedback_active = 0.0;
  double outer_acceleration_limited = 0.0;
  double outer_pitch_target_limited = 0.0;
  double active_drive_max_velocity_mps = 0.0;
  double active_drive_max_acceleration_mps2 = 0.0;
  double active_drive_max_deceleration_mps2 = 0.0;
  double active_velocity_gain_per_s = 0.0;
  double active_velocity_feedback_cutoff_hz = 0.0;
  double active_outer_pitch_limit_deg = 0.0;
  double active_fixed_com_trim_deg = 0.0;
  double adaptive_com_trim_enabled = 0.0;
  double legacy_outer_fields_valid = 0.0;
  double final_pitch_target_deg = 0.0;
  double active_planner_max_acceleration_mps2 = 0.0;
  double active_planner_max_deceleration_mps2 = 0.0;
  double active_planner_max_jerk_mps3 = 0.0;
  double active_velocity_i_gain_per_s2 = 0.0;
  double active_velocity_i_leak_time_s = 0.0;
  double active_velocity_i_acceleration_limit_mps2 = 0.0;
  double planner_acceleration_limited = 0.0;
  double planner_jerk_limited = 0.0;
  double velocity_integral_limited = 0.0;
  double velocity_anti_windup_active = 0.0;
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
  // Simulator-only velocity provenance. The emitted rate is derived from the
  // physical MotorFeedback counters before any optional estimator fixture is
  // applied. The controller-facing rate includes that fixture, while the
  // production telemetry fields below remain the filtered observer stages.
  double emitted_step_velocity_sps = 0.0;
  double synthetic_estimator_velocity_sps = 0.0;
  double controller_feedback_velocity_sps = 0.0;
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
  // StepperPhase internal diagnostics. These are intentionally kept outside
  // the production telemetry payload until the actuator topology is settled.
  double stepper_commanded_microsteps_left = 0.0;
  double stepper_commanded_microsteps_right = 0.0;
  double stepper_commanded_field_angle_left_rad = 0.0;
  double stepper_commanded_field_angle_right_rad = 0.0;
  double stepper_commanded_field_electrical_angle_left_rad = 0.0;
  double stepper_commanded_field_electrical_angle_right_rad = 0.0;
  double stepper_commanded_field_velocity_mps = 0.0;
  double stepper_actual_relative_angle_left_rad = 0.0;
  double stepper_actual_relative_angle_right_rad = 0.0;
  double stepper_actual_rotor_electrical_angle_left_rad = 0.0;
  double stepper_actual_rotor_electrical_angle_right_rad = 0.0;
  double stepper_electrical_phase_error_left_rad = 0.0;
  double stepper_electrical_phase_error_right_rad = 0.0;
  double stepper_torque_left_nm = 0.0;
  double stepper_torque_right_nm = 0.0;
  double stepper_summed_torque_nm = 0.0;
  double stepper_actual_wheel_velocity_mps = 0.0;
  double stepper_chassis_velocity_mps = 0.0;
  double stepper_current_ref_a_left = 0.0;
  double stepper_current_ref_b_left = 0.0;
  double stepper_current_a_left = 0.0;
  double stepper_current_b_left = 0.0;
  double stepper_phase_voltage_a_left = 0.0;
  double stepper_phase_voltage_b_left = 0.0;
  double stepper_back_emf_a_left = 0.0;
  double stepper_back_emf_b_left = 0.0;
  double stepper_electrical_power_left_w = 0.0;
  double stepper_mechanical_power_left_w = 0.0;
  double stepper_resistive_loss_left_w = 0.0;
  double stepper_magnetic_energy_left_j = 0.0;
  double stepper_current_ref_a_right = 0.0;
  double stepper_current_ref_b_right = 0.0;
  double stepper_current_a_right = 0.0;
  double stepper_current_b_right = 0.0;
  double stepper_phase_voltage_a_right = 0.0;
  double stepper_phase_voltage_b_right = 0.0;
  double stepper_back_emf_a_right = 0.0;
  double stepper_back_emf_b_right = 0.0;
  double stepper_electrical_power_right_w = 0.0;
  double stepper_mechanical_power_right_w = 0.0;
  double stepper_resistive_loss_right_w = 0.0;
  double stepper_magnetic_energy_right_j = 0.0;
  double stepper_voltage_saturated_left = 0.0;
  double stepper_voltage_saturated_right = 0.0;
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
std::vector<SimulatorScenario> tuning_inner_scenario_set(
    PhysicsProfile physics_profile = PhysicsProfile::Realistic);
std::vector<SimulatorScenario> tuning_authority_scenario_set(
    PhysicsProfile physics_profile = PhysicsProfile::Realistic);
std::vector<SimulatorScenario> tuning_velocity_scenario_set(
    PhysicsProfile physics_profile = PhysicsProfile::Realistic);
std::vector<SimulatorScenario> tuning_drive_scenario_set(
    PhysicsProfile physics_profile = PhysicsProfile::Realistic);
std::vector<SimulatorScenario> tuning_motion_scenario_set(
    PhysicsProfile physics_profile = PhysicsProfile::StepperPhaseElectrical);
std::vector<SimulatorScenario> tuning_outer_motion_scenario_set(
    PhysicsProfile physics_profile = PhysicsProfile::StepperPhaseElectrical,
    std::optional<double> cart_damping_override = 1.0);
std::vector<SimulatorScenario> tuning_leaky_integral_scenario_set(
    PhysicsProfile physics_profile = PhysicsProfile::StepperPhaseElectrical,
    std::optional<double> cart_damping_override = 1.0);
std::vector<SimulatorScenario> tuning_distance_scenario_set(
    PhysicsProfile physics_profile = PhysicsProfile::StepperPhaseElectrical,
    std::optional<double> cart_damping_override = 1.0);
std::vector<SimulatorScenario> tuning_speed_envelope_scenario_set(
    PhysicsProfile physics_profile = PhysicsProfile::StepperPhaseElectrical,
    std::optional<double> cart_damping_override = 1.0);
std::vector<SimulatorScenario> tuning_trim_scenario_set(
    PhysicsProfile physics_profile = PhysicsProfile::Realistic);
TransferAcceptance evaluate_transfer_scenario(const SimulatorRunResult& result);
