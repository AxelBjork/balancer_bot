#include "balancer_simulator.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kAccelScale = 16384.0 / 9.81;
constexpr double kGyroScale = 6550.0;

double steps_per_sec_to_wheel_velocity(double steps_per_sec, double steps_per_rev, double wheel_radius) {
  // Controller convention is negative wheel SPS for forward corrective motion.
  return -(steps_per_sec / steps_per_rev) * 2.0 * kPi * wheel_radius;
}

}  // namespace

SimulatorPhysics BalancerSimulator::physics_for_profile(PhysicsProfile profile) {
  switch (profile) {
    case PhysicsProfile::Simplified:
      return SimulatorPhysics{
          .driver_kp = 90.0,
          .max_force_n = 4.0,
          .cart_damping = 12.0,
          .pitch_damping = 1.5,
          .motor_tau_s = 0.05,
      };
    case PhysicsProfile::Realistic:
    default:
      return SimulatorPhysics{
          .driver_kp = 192.5,
          .max_force_n = 8.0,
          .cart_damping = 9.5,
          .pitch_damping = 1.1375,
          .motor_tau_s = 0.0375,
      };
  }
}

std::string_view BalancerSimulator::profile_name(PhysicsProfile profile) {
  switch (profile) {
    case PhysicsProfile::Simplified:
      return "simplified";
    case PhysicsProfile::Realistic:
    default:
      return "realistic";
  }
}

BalancerSimulator::BalancerSimulator(const Config& cfg) : cfg_(cfg) {
  physics_ = cfg_.physics_override.value_or(physics_for_profile(cfg_.physics_profile));
  state_.pitch = cfg_.initial_pitch_deg * kPi / 180.0;
}

void BalancerSimulator::set_motor_targets(float left_sps, float right_sps) {
  left_target_sps_ = left_sps;
  right_target_sps_ = right_sps;
}

void BalancerSimulator::step(double dt_s) {
  const double avg_steps_per_sec = 0.5 * (left_target_sps_ + right_target_sps_);
  const double target_wheel_velocity =
      steps_per_sec_to_wheel_velocity(avg_steps_per_sec, steps_per_rev, wheel_radius);

  if (physics_.motor_tau_s > 0.0) {
    const double alpha = std::clamp(dt_s / (physics_.motor_tau_s + dt_s), 0.0, 1.0);
    actual_wheel_velocity_ += alpha * (target_wheel_velocity - actual_wheel_velocity_);
  } else {
    actual_wheel_velocity_ = target_wheel_velocity;
  }

  const double current_wheel_v = state_.velocity;
  const double v_err = actual_wheel_velocity_ - current_wheel_v;
  const double F_cmd = physics_.driver_kp * v_err;
  const double F_app = std::clamp(F_cmd, -physics_.max_force_n, physics_.max_force_n);

  const double Q = state_.pitch + cfg_.com_angle_offset_rad;
  const double Q_dot = state_.pitch_rate;
  const double sQ = std::sin(Q);
  const double cQ = std::cos(Q);

  const double M = cart_mass;
  const double m = body_mass;
  const double l = center_of_mass_height;
  const double I = I_com;

  const double d11 = M + m;
  const double d12 = m * l * cQ;
  const double d21 = m * l * cQ;
  const double d22 = I + m * l * l;

  const double rhs1 = F_app + m * l * Q_dot * Q_dot * sQ - physics_.cart_damping * state_.velocity;
  const double rhs2 = m * gravity * l * sQ - physics_.pitch_damping * state_.pitch_rate;

  const double det = d11 * d22 - d12 * d21;
  const double x_ddot = (d22 * rhs1 - d12 * rhs2) / det;
  const double theta_ddot = (d11 * rhs2 - d21 * rhs1) / det;

  state_.velocity += x_ddot * dt_s;
  state_.position += state_.velocity * dt_s;
  state_.pitch_rate += theta_ddot * dt_s;
  state_.pitch += state_.pitch_rate * dt_s;

  if (std::abs(state_.pitch) > kPi / 2.0) {
    state_.pitch = state_.pitch > 0 ? kPi / 2.0 : -kPi / 2.0;
    state_.pitch_rate = 0.0;
    state_.velocity = 0.0;
  }

  diagnostics_.target_wheel_velocity = target_wheel_velocity;
  diagnostics_.actual_wheel_velocity = actual_wheel_velocity_;
  diagnostics_.velocity_error = v_err;
  diagnostics_.f_cmd = F_cmd;
  diagnostics_.f_app = F_app;
  diagnostics_.x_ddot = x_ddot;
  diagnostics_.theta_ddot = theta_ddot;
  diagnostics_.command_saturated = std::abs(F_app) >= physics_.max_force_n * 0.999;
}

ipc::ImuSamplePayload BalancerSimulator::make_imu_payload(uint64_t sim_time_us) const {
  ipc::ImuSamplePayload payload{};

  const double Q = state_.pitch + cfg_.com_angle_offset_rad;
  const double ax_mps2 = diagnostics_.x_ddot * std::cos(Q) + gravity * std::sin(Q);
  const double az_mps2 = -diagnostics_.x_ddot * std::sin(Q) + gravity * std::cos(Q);

  payload.pitch_rad = state_.pitch;
  payload.acc = {
      -ax_mps2,
      0.0,
      -az_mps2,
  };
  payload.gyr = {
      0.0,
      state_.pitch_rate,
      0.0,
  };
  payload.timestamp_us = sim_time_us;

  (void)kAccelScale;
  (void)kGyroScale;
  return payload;
}

float BalancerSimulator::get_actual_speed_sps() const {
  const double cart_rev_per_sec = state_.velocity / (2.0 * kPi * wheel_radius);
  return static_cast<float>(cart_rev_per_sec * steps_per_rev);
}
