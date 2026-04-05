#include "balancer_simulator.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kAccelScale = 16384.0 / 9.81;
constexpr double kGyroScale = 6550.0;

}  // namespace

BalancerSimulator::BalancerSimulator(const Config& cfg) : cfg_(cfg) {
  state_.pitch = cfg_.initial_pitch_deg * kPi / 180.0;
}

void BalancerSimulator::set_motor_targets(float left_sps, float right_sps) {
  left_target_sps_ = left_sps;
  right_target_sps_ = right_sps;
}

void BalancerSimulator::step(double dt_s) {
  const double avg_steps_per_sec = 0.5 * (left_target_sps_ + right_target_sps_);
  const double target_wheel_velocity =
      (avg_steps_per_sec / steps_per_rev) * 2.0 * kPi * wheel_radius;

  const double current_wheel_v = state_.velocity;
  const double v_err = target_wheel_velocity - current_wheel_v;

  constexpr double kDriverP = 500.0;
  constexpr double kMaxForce = 20.0;
  const double F_cmd = kDriverP * v_err;
  const double F_app = std::clamp(F_cmd, -kMaxForce, kMaxForce);

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

  constexpr double c_x = 2.0;
  constexpr double c_th = 0.05;

  const double rhs1 = F_app + m * l * Q_dot * Q_dot * sQ - c_x * state_.velocity;
  const double rhs2 = m * gravity * l * sQ - c_th * state_.pitch_rate;

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

  last_x_ddot_ = x_ddot;
}

ipc::ImuSamplePayload BalancerSimulator::make_imu_payload(uint64_t sim_time_us) const {
  ipc::ImuSamplePayload payload{};

  const double Q = state_.pitch + cfg_.com_angle_offset_rad;
  const double ax_mps2 = last_x_ddot_ * std::cos(Q) + gravity * std::sin(Q);
  const double az_mps2 = -last_x_ddot_ * std::sin(Q) + gravity * std::cos(Q);

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
  const double rev_per_sec = state_.velocity / (2.0 * kPi * wheel_radius);
  return static_cast<float>(rev_per_sec * steps_per_rev);
}
