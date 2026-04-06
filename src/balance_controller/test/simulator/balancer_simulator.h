#pragma once

#include <optional>
#include <string_view>

#include "messages/balancer_msgs.h"

enum class PhysicsProfile {
  Realistic,
  Simplified,
};

struct SimulatorPhysics {
  double driver_kp = 500.0;
  double max_force_n = 20.0;
  double cart_damping = 2.0;
  double pitch_damping = 0.05;
  double motor_tau_s = 0.0;
};

struct SimulatorConfig {
  double com_angle_offset_rad = 0.001;
  double initial_pitch_deg = 2.0;
  PhysicsProfile physics_profile = PhysicsProfile::Realistic;
  std::optional<SimulatorPhysics> physics_override;
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
    double x_ddot = 0.0;
    double theta_ddot = 0.0;
    bool command_saturated = false;
  };

  explicit BalancerSimulator(const Config& cfg = Config());

  void set_motor_targets(float left_sps, float right_sps);
  void step(double dt_s);
  ipc::ImuSamplePayload make_imu_payload(uint64_t sim_time_us) const;

  const Config& config() const { return cfg_; }
  const SimulatorPhysics& physics() const { return physics_; }
  const State& state() const { return state_; }
  const Diagnostics& diagnostics() const { return diagnostics_; }
  double get_pitch() const { return state_.pitch; }
  double get_position() const { return state_.position; }
  float get_actual_speed_sps() const;

  static SimulatorPhysics physics_for_profile(PhysicsProfile profile);
  static std::string_view profile_name(PhysicsProfile profile);

 private:
  Config cfg_;
  SimulatorPhysics physics_{};
  State state_{};
  float left_target_sps_{0.0f};
  float right_target_sps_{0.0f};
  double actual_wheel_velocity_{0.0};
  Diagnostics diagnostics_{};

  static constexpr double gravity = 9.81;
  static constexpr double wheel_radius = 0.080 / 2.0;
  static constexpr double robot_mass = 1.032;
  static constexpr double wheel_mass = 0.050;
  static constexpr double cart_mass = 2.0 * wheel_mass;
  static constexpr double body_mass = robot_mass - cart_mass;
  static constexpr double center_of_mass_height = 0.06;
  static constexpr double I_com = 0.0034;
  static constexpr double steps_per_rev = 200.0 * 16.0;
};
