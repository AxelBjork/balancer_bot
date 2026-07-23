#include "balancer_simulator.h"

#include <algorithm>
#include <cmath>

#include "services/control/rate_controller_core.h"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kAccelScale = 16384.0 / 9.81;
constexpr double kGyroScale = 6550.0;
constexpr double kReferenceWheelVelocityMps =
    (2.0 * kPi * 0.040) / 1.0;  // one wheel rev per second for the configured geometry

double steps_per_sec_to_wheel_velocity(double steps_per_sec, double steps_per_rev,
                                       double wheel_radius) {
  // Controller convention is negative wheel SPS for forward corrective motion.
  return -(steps_per_sec / steps_per_rev) * 2.0 * kPi * wheel_radius;
}

}  // namespace

SimulatorPhysics BalancerSimulator::physics_for_profile(PhysicsProfile profile) {
  switch (profile) {
    case PhysicsProfile::Simplified:
      return SimulatorPhysics{
          .drive_force_per_mps = 90.0 / kReferenceWheelVelocityMps,
          .max_force_n = 4.0,
          .cart_damping = 0.4,
          .pitch_damping = 1.5,
          .motor_tau_s = 0.10,
      };
    case PhysicsProfile::Realistic:
    default:
      return SimulatorPhysics{
          .drive_force_per_mps = 192.5 / kReferenceWheelVelocityMps,
          .max_force_n = 200.0,
          .cart_damping = 1.0,
          .pitch_damping = 1.1375,
          .motor_tau_s = 0.15,
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
  imu_pitch_ = state_.pitch;
  imu_pitch_rate_ = state_.pitch_rate;
}

void BalancerSimulator::set_motor_targets(double left_sps, double right_sps) {
  left_target_sps_ = left_sps;
  right_target_sps_ = right_sps;
}

void BalancerSimulator::set_external_force_n(double force_n) {
  external_force_n_ = force_n;
}

void BalancerSimulator::set_external_com_bias_rad(double com_bias_rad) {
  external_com_bias_rad_ = com_bias_rad;
}

void BalancerSimulator::step(double dt_s) {
  const double avg_steps_per_sec = 0.5 * (left_target_sps_ + right_target_sps_);
  const double target_wheel_velocity =
      steps_per_sec_to_wheel_velocity(avg_steps_per_sec, steps_per_rev, wheel_radius);
  const double desired_force = std::clamp(target_wheel_velocity * physics_.drive_force_per_mps,
                                          -physics_.max_force_n, physics_.max_force_n);

  if (physics_.motor_tau_s > 0.0) {
    const double alpha = std::clamp(dt_s / (physics_.motor_tau_s + dt_s), 0.0, 1.0);
    actual_wheel_velocity_ += alpha * (target_wheel_velocity - actual_wheel_velocity_);
  } else {
    actual_wheel_velocity_ = target_wheel_velocity;
  }

  const double force_alpha = (physics_.motor_tau_s > 0.0)
                                 ? std::clamp(dt_s / (physics_.motor_tau_s + dt_s), 0.0, 1.0)
                                 : 1.0;
  applied_drive_force_ += force_alpha * (desired_force - applied_drive_force_);
  const double F_cmd = desired_force;
  const double F_app = std::clamp(applied_drive_force_ * cfg_.wheel_slip_factor,
                                  -physics_.max_force_n, physics_.max_force_n);
  const double total_force = F_app + external_force_n_;

  const double Q = state_.pitch + cfg_.com_angle_offset_rad + external_com_bias_rad_;
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

  const double rhs1 =
      total_force + m * l * Q_dot * Q_dot * sQ - physics_.cart_damping * state_.velocity;
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

  const double axle_sps = state_.velocity / (2.0 * kPi * wheel_radius) * steps_per_rev;
  // q_dot = u_dot - r*theta_dot (expressed as wheel angular velocity).
  // This intentionally includes pitch motion in the encoder-like stream.
  const double motor_relative_sps = axle_sps - steps_per_rev * state_.pitch_rate / (2.0 * kPi);
  const double measured_velocity_target_sps =
      cfg_.velocity_feedback_scale *
      (cfg_.velocity_feedback_model == VelocityFeedbackModel::MotorRelative ? motor_relative_sps
                                                                            : axle_sps);
  if (cfg_.velocity_feedback_tau_s > 0.0) {
    const double alpha = std::clamp(dt_s / (cfg_.velocity_feedback_tau_s + dt_s), 0.0, 1.0);
    measured_velocity_sps_ += alpha * (measured_velocity_target_sps - measured_velocity_sps_);
  } else {
    measured_velocity_sps_ = measured_velocity_target_sps;
  }

  if (cfg_.imu_pitch_lag_s > 0.0) {
    const double alpha = std::clamp(dt_s / (cfg_.imu_pitch_lag_s + dt_s), 0.0, 1.0);
    imu_pitch_ += alpha * (state_.pitch - imu_pitch_);
    imu_pitch_rate_ += alpha * (state_.pitch_rate - imu_pitch_rate_);
  } else {
    imu_pitch_ = state_.pitch;
    imu_pitch_rate_ = state_.pitch_rate;
  }

  diagnostics_.target_wheel_velocity = target_wheel_velocity;
  diagnostics_.actual_wheel_velocity = actual_wheel_velocity_;
  diagnostics_.velocity_error = target_wheel_velocity - state_.velocity;
  diagnostics_.f_cmd = F_cmd;
  diagnostics_.f_app = F_app;
  diagnostics_.external_force_n = external_force_n_;
  diagnostics_.external_com_bias_rad = external_com_bias_rad_;
  diagnostics_.x_ddot = x_ddot;
  diagnostics_.theta_ddot = theta_ddot;
  diagnostics_.command_saturated = (std::abs(F_cmd) >= physics_.max_force_n * 0.999) ||
                                   (std::abs(F_app) >= physics_.max_force_n * 0.999);
}

ipc::ImuRawPayload BalancerSimulator::make_raw_imu_payload(uint64_t sim_time_us) const {
  const double body_pitch = state_.pitch;

  ipc::ImuRawPayload payload{};
  // Keep this first hardware-like filter pass focused on sensor noise and pitch bias.
  // Translational acceleration remains exposed separately through plant diagnostics.
  payload.acc = {
      -gravity * std::sin(body_pitch),
      0.0,
      -gravity * std::cos(body_pitch),
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

ipc::ImuSamplePayload BalancerSimulator::make_imu_payload(uint64_t sim_time_us) const {
  const ipc::ImuRawPayload raw = make_raw_imu_payload(sim_time_us);
  ipc::ImuSamplePayload payload{};
  payload.pitch_rad = imu_pitch_;
  payload.pitch_rate_rad_s = imu_pitch_rate_;
  payload.pitch_accel_rad_s2 = diagnostics_.theta_ddot;
  payload.acc = raw.acc;
  payload.gyr = raw.gyr;
  payload.timestamp_us = raw.timestamp_us;
  return payload;
}

double BalancerSimulator::get_actual_speed_sps() const {
  return measured_velocity_sps_;
}

double BalancerSimulator::get_corrected_axle_speed_sps() const {
  if (cfg_.velocity_feedback_model == VelocityFeedbackModel::MotorRelative) {
    return measured_velocity_sps_ +
           cfg_.velocity_feedback_scale * steps_per_rev * state_.pitch_rate / (2.0 * kPi);
  }
  return measured_velocity_sps_;
}

BalancerSimulator::LinearizedUprightModel BalancerSimulator::linearized_upright_model(
    const SimulatorPhysics& physics) {
  (void)physics;
  LinearizedUprightModel model{};

  const double M = cart_mass;
  const double m = body_mass;
  const double l = center_of_mass_height;
  const double I = I_com;

  const double d11 = M + m;
  const double d12 = m * l;
  const double d21 = m * l;
  const double d22 = I + m * l * l;
  const double det = d11 * d22 - d12 * d21;

  const double v_coeff = -(d22 * physics.cart_damping) / det;
  const double theta_coeff = -(d12 * m * gravity * l) / det;
  const double theta_dot_coeff = (d12 * physics.pitch_damping) / det;
  const double force_to_x_ddot = d22 / det;

  const double q_v_coeff = (d21 * physics.cart_damping) / det;
  const double q_theta_coeff = (d11 * m * gravity * l) / det;
  const double q_theta_dot_coeff = -(d11 * physics.pitch_damping) / det;
  const double force_to_q_ddot = -d21 / det;

  model.A = {{
      {{0.0, 1.0, 0.0, 0.0}},
      {{0.0, v_coeff, theta_coeff, theta_dot_coeff}},
      {{0.0, 0.0, 0.0, 1.0}},
      {{0.0, q_v_coeff, q_theta_coeff, q_theta_dot_coeff}},
  }};
  model.B = {{0.0, force_to_x_ddot, 0.0, force_to_q_ddot}};
  return model;
}

std::array<double, 4> BalancerSimulator::overdamped_candidate_poles(
    const SimulatorPhysics& physics) {
  const auto model = linearized_upright_model(physics);
  const double open_loop_pitch_rate = std::sqrt(std::max(0.0, model.A[3][2]));
  const double actuator_limited_rate = 0.5 / std::max(physics.motor_tau_s, 0.02);
  const double base =
      std::clamp(std::min(1.25 * open_loop_pitch_rate, actuator_limited_rate), 1.0, 8.0);
  return {{-0.45 * base, -0.75 * base, -1.05 * base, -1.35 * base}};
}
