#pragma once

#include <optional>
#include <string_view>

#include "messages/balancer_msgs.h"

enum class PhysicsProfile {
  Realistic,
  Simplified,
};

struct SimulatorPhysics {
  // Aggregate zero-speed tangential force from both motors at the wheel rim.
  // The torque-speed envelope reduces this ideal stall-force value with speed.
  double max_force_n = 22.5;
  double no_load_speed_mps = 1.2;
  double traction_coefficient = 1.0;
  double motor_velocity_damping = 8.0;
  double cart_damping = 1.0;
  double pitch_damping = 0.02;
  double motor_tau_s = 0.008;
  double phase_error_limit_steps = 16.0;
  double tire_stiffness_n_per_m = 3000.0;
  double tire_damping_n_s_per_m = 12.0;
  double wheel_equivalent_mass_kg = 0.10;
};

struct SimulatorConfig {
  double com_angle_offset_rad = 0.001;
  double initial_pitch_deg = 2.0;
  double initial_pitch_rate_dps = 0.0;
  PhysicsProfile physics_profile = PhysicsProfile::Realistic;
  std::optional<SimulatorPhysics> physics_override;
  double total_mass_scale = 1.0;
  double pitch_inertia_scale = 1.0;
  double imu_height_m = 0.070;
};

class BalancerSimulator {
 public:
  using Config = SimulatorConfig;

  struct State {
    double pitch = 0.0;
    double pitch_rate = 0.0;
    double position = 0.0;
    double velocity = 0.0;
  };

  struct Diagnostics {
    double target_wheel_velocity = 0.0;
    double actual_wheel_velocity = 0.0;
    double velocity_error = 0.0;
    double f_cmd = 0.0;
    double f_app = 0.0;
    double external_force_n = 0.0;
    double external_com_bias_rad = 0.0;
    double x_ddot = 0.0;
    double theta_ddot = 0.0;
    double phase_error_steps = 0.0;
    double missed_steps = 0.0;
    double traction_limit_n = 0.0;
    double motor_force_limit_n = 0.0;
    bool command_saturated = false;
  };

  struct LinearizedUprightModel {
    std::array<std::array<double, 4>, 4> A{};
    std::array<double, 4> horizontal_force_input{};
    std::array<double, 4> motor_force_input{};
  };

  explicit BalancerSimulator(const Config& cfg = Config());

  void set_motor_targets(double left_sps, double right_sps);
  void set_emitted_steps(double left_steps, double right_steps);
  void set_external_force_n(double force_n);
  void set_external_com_bias_rad(double com_bias_rad);
  void step(double dt_s);
  ipc::ImuRawPayload make_raw_imu_payload(uint64_t sim_time_us) const;

  const Config& config() const {
    return cfg_;
  }
  const SimulatorPhysics& physics() const {
    return physics_;
  }
  const State& state() const {
    return state_;
  }
  const Diagnostics& diagnostics() const {
    return diagnostics_;
  }
  double get_pitch() const {
    return state_.pitch;
  }
  double get_position() const {
    return state_.position;
  }

  static SimulatorPhysics physics_for_profile(PhysicsProfile profile);
  static std::string_view profile_name(PhysicsProfile profile);
  static LinearizedUprightModel linearized_upright_model(const SimulatorPhysics& physics);
  static std::array<double, 4> overdamped_candidate_poles(const SimulatorPhysics& physics);

 private:
  Config cfg_;
  SimulatorPhysics physics_{};
  State state_{};
  double left_target_sps_{0.0};
  double right_target_sps_{0.0};
  double actual_wheel_velocity_{0.0};
  double applied_drive_force_{0.0};
  double wheel_position_m_{0.0};
  double wheel_velocity_mps_{0.0};
  double emitted_steps_avg_{0.0};
  double missed_distance_m_{0.0};
  bool have_external_emitted_steps_{false};
  double external_force_n_{0.0};
  double external_com_bias_rad_{0.0};
  Diagnostics diagnostics_{};

 public:
  struct HardwareNominal {
    static constexpr double gravity = 9.81;
    static constexpr double wheel_radius = 0.0824 / 2.0;
    static constexpr double motor_count = 2.0;
    static constexpr double motor_stall_torque_nm = 0.45;
    static constexpr double combined_stall_force_n =
        motor_count * motor_stall_torque_nm / wheel_radius;
    static constexpr double total_mass_kg = 1.032;
    // Measured complete-robot COM height is 60 mm above the axle: H = T * z_c.
    static constexpr double first_mass_moment_kg_m = 0.06192;
    // Provisional legacy-derived value; replace after the physical-pendulum measurement.
    static constexpr double pitch_inertia_about_axle_kg_m2 = 0.0067552;
    static constexpr double steps_per_rev = 200.0 * 16.0;
    static constexpr double meters_per_step =
        2.0 * 3.14159265358979323846 * wheel_radius / steps_per_rev;
  };
};
