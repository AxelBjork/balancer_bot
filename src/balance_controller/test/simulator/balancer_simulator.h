#pragma once

#include "balancer_msgs.h"

struct SimulatorConfig {
  double com_angle_offset_rad = 0.001;
  double initial_pitch_deg = 2.0;
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

  explicit BalancerSimulator(const Config& cfg = Config());

  void set_motor_targets(float left_sps, float right_sps);
  void step(double dt_s);
  ipc::ImuSamplePayload make_imu_payload(uint64_t sim_time_us) const;

  const State& state() const { return state_; }
  double get_pitch() const { return state_.pitch; }
  double get_position() const { return state_.position; }
  float get_actual_speed_sps() const;

 private:
  Config cfg_;
  State state_{};
  float left_target_sps_{0.0f};
  float right_target_sps_{0.0f};
  double last_x_ddot_{0.0};

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
