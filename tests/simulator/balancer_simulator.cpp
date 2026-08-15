#include "balancer_simulator.h"

#include <algorithm>
#include <cmath>

#include "services/control/rate_controller_core.h"

namespace {

constexpr double kPi = 3.14159265358979323846;
double steps_per_sec_to_wheel_velocity(double steps_per_sec, double steps_per_rev,
                                       double wheel_radius) {
  return (steps_per_sec / steps_per_rev) * 2.0 * kPi * wheel_radius;
}

}  // namespace

SimulatorPhysics BalancerSimulator::physics_for_profile(PhysicsProfile profile) {
  switch (profile) {
    case PhysicsProfile::Simplified:
      return SimulatorPhysics{
          .max_force_n = 8.0,
          .no_load_speed_mps = 1.6,
          .traction_coefficient = 1.2,
          .motor_velocity_damping = 30.0,
          .cart_damping = 0.4,
          .pitch_damping = 0.04,
          .motor_tau_s = 0.004,
          .phase_error_limit_steps = 16.0,
          .tire_stiffness_n_per_m = 2500.0,
          .tire_damping_n_s_per_m = 30.0,
          .wheel_equivalent_mass_kg = 0.10,
      };
    case PhysicsProfile::Realistic:
    default:
      return SimulatorPhysics{
          .max_force_n = HardwareNominal::combined_stall_force_n,
          .no_load_speed_mps = 1.2,
          .traction_coefficient = 1.0,
          .motor_velocity_damping = 40.0,
          .cart_damping = 1.0,
          .pitch_damping = 0.02,
          .motor_tau_s = 0.008,
          .phase_error_limit_steps = 16.0,
          .tire_stiffness_n_per_m = 3000.0,
          .tire_damping_n_s_per_m = 35.0,
          .wheel_equivalent_mass_kg = 0.10,
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
  state_.pitch_rate = cfg_.initial_pitch_rate_dps * kPi / 180.0;
}

void BalancerSimulator::set_motor_targets(double left_sps, double right_sps) {
  left_target_sps_ = left_sps;
  right_target_sps_ = right_sps;
}

void BalancerSimulator::set_emitted_steps(double left_steps, double right_steps) {
  emitted_steps_avg_ = 0.5 * (left_steps + right_steps);
  have_external_emitted_steps_ = true;
}

void BalancerSimulator::set_external_force_n(double force_n) {
  external_force_n_ = force_n;
}

void BalancerSimulator::set_external_com_bias_rad(double com_bias_rad) {
  external_com_bias_rad_ = com_bias_rad;
}

void BalancerSimulator::step(double dt_s) {
  using Nominal = HardwareNominal;
  const double avg_steps_per_sec = 0.5 * (left_target_sps_ + right_target_sps_);
  const double target_wheel_velocity = steps_per_sec_to_wheel_velocity(
      avg_steps_per_sec, Nominal::steps_per_rev, Nominal::wheel_radius);
  if (!have_external_emitted_steps_) {
    emitted_steps_avg_ += avg_steps_per_sec * dt_s;
  }
  const double emitted_position_m = emitted_steps_avg_ * Nominal::meters_per_step;
  const double initial_pitch_rad = cfg_.initial_pitch_deg * kPi / 180.0;
  const double relative_wheel_position_m =
      wheel_position_m_ - Nominal::wheel_radius * (state_.pitch - initial_pitch_rad);
  const double relative_wheel_velocity_mps =
      wheel_velocity_mps_ - Nominal::wheel_radius * state_.pitch_rate;
  const double speed_fraction = physics_.no_load_speed_mps > 0.0
                                    ? std::abs(relative_wheel_velocity_mps) /
                                          physics_.no_load_speed_mps
                                    : 1.0;
  const double motor_force_limit =
      physics_.max_force_n * std::clamp(1.0 - speed_fraction, 0.0, 1.0);
  const double scaled_robot_mass = Nominal::total_mass_kg * std::max(0.1, cfg_.total_mass_scale);
  const double traction_limit =
      std::max(0.0, physics_.traction_coefficient) * scaled_robot_mass * Nominal::gravity;

  double effective_command_position_m = emitted_position_m - missed_distance_m_;
  double phase_error_m = effective_command_position_m - relative_wheel_position_m;
  const double phase_limit_m =
      std::max(1.0, physics_.phase_error_limit_steps) * Nominal::meters_per_step;
  if (std::abs(phase_error_m) > phase_limit_m) {
    const double clamped_error = std::clamp(phase_error_m, -phase_limit_m, phase_limit_m);
    missed_distance_m_ += phase_error_m - clamped_error;
    effective_command_position_m = emitted_position_m - missed_distance_m_;
    phase_error_m = effective_command_position_m - relative_wheel_position_m;
  }

  const double phase_stiffness = physics_.max_force_n / phase_limit_m;
  const double desired_force =
      phase_stiffness * phase_error_m +
      physics_.motor_velocity_damping *
          (target_wheel_velocity - relative_wheel_velocity_mps);
  const double limited_motor_force =
      std::clamp(desired_force, -motor_force_limit, motor_force_limit);

  const double force_alpha = (physics_.motor_tau_s > 0.0)
                                 ? std::clamp(dt_s / (physics_.motor_tau_s + dt_s), 0.0, 1.0)
                                 : 1.0;
  applied_drive_force_ += force_alpha * (limited_motor_force - applied_drive_force_);
  const double desired_tire_force =
      physics_.tire_stiffness_n_per_m * (wheel_position_m_ - state_.position) +
      physics_.tire_damping_n_s_per_m * (wheel_velocity_mps_ - state_.velocity);
  const double tire_force = std::clamp(desired_tire_force, -traction_limit, traction_limit);
  const double F_cmd = desired_force;
  const double F_app = tire_force;
  const double total_force = F_app + external_force_n_;

  const double Q = state_.pitch + cfg_.com_angle_offset_rad + external_com_bias_rad_;
  const double Q_dot = state_.pitch_rate;
  const double sQ = std::sin(Q);
  const double cQ = std::cos(Q);

  const double T = Nominal::total_mass_kg * std::max(0.1, cfg_.total_mass_scale);
  const double H = Nominal::first_mass_moment_kg_m;
  const double J = Nominal::pitch_inertia_about_axle_kg_m2 *
                   std::max(0.1, cfg_.pitch_inertia_scale);

  const double d11 = T;
  const double d12 = H * cQ;
  const double d21 = H * cQ;
  const double d22 = J;

  const double rhs1 =
      total_force + H * Q_dot * Q_dot * sQ - physics_.cart_damping * state_.velocity;
  const double motor_reaction_torque = applied_drive_force_ * Nominal::wheel_radius;
  // Disturbances model a horizontal push at the robot COM.  In this 2D plant,
  // that force creates both a translational force and a pitch moment about the axle.
  const double com_height_m = Nominal::first_mass_moment_kg_m / Nominal::total_mass_kg;
  const double external_force_pitch_moment = external_force_n_ * com_height_m * cQ;
  const double rhs2 = Nominal::gravity * H * sQ -
                      physics_.pitch_damping * state_.pitch_rate - motor_reaction_torque +
                      external_force_pitch_moment;

  const double det = d11 * d22 - d12 * d21;
  const double x_ddot = (d22 * rhs1 - d12 * rhs2) / det;
  const double theta_ddot = (d11 * rhs2 - d21 * rhs1) / det;

  state_.velocity += x_ddot * dt_s;
  state_.position += state_.velocity * dt_s;
  state_.pitch_rate += theta_ddot * dt_s;
  state_.pitch += state_.pitch_rate * dt_s;

  const double wheel_mass = std::max(0.01, physics_.wheel_equivalent_mass_kg);
  const double wheel_accel = (applied_drive_force_ - tire_force) / wheel_mass;
  wheel_velocity_mps_ += wheel_accel * dt_s;
  wheel_position_m_ += wheel_velocity_mps_ * dt_s;
  actual_wheel_velocity_ =
      wheel_velocity_mps_ - Nominal::wheel_radius * state_.pitch_rate;

  if (std::abs(state_.pitch) > kPi / 2.0) {
    state_.pitch = state_.pitch > 0 ? kPi / 2.0 : -kPi / 2.0;
    state_.pitch_rate = 0.0;
    state_.velocity = 0.0;
  }

  diagnostics_.target_wheel_velocity = target_wheel_velocity;
  diagnostics_.actual_wheel_velocity = actual_wheel_velocity_;
  diagnostics_.velocity_error = target_wheel_velocity - actual_wheel_velocity_;
  diagnostics_.f_cmd = F_cmd;
  diagnostics_.f_app = F_app;
  diagnostics_.external_force_n = external_force_n_;
  diagnostics_.external_com_bias_rad = external_com_bias_rad_;
  diagnostics_.x_ddot = x_ddot;
  diagnostics_.theta_ddot = theta_ddot;
  diagnostics_.phase_error_steps = phase_error_m / Nominal::meters_per_step;
  diagnostics_.missed_steps = missed_distance_m_ / Nominal::meters_per_step;
  diagnostics_.traction_limit_n = traction_limit;
  diagnostics_.motor_force_limit_n = motor_force_limit;
  diagnostics_.command_saturated = motor_force_limit <= 0.0 ||
                                   std::abs(F_cmd) >= motor_force_limit * 0.999 ||
                                   std::abs(desired_tire_force) >= traction_limit * 0.999;
}

ipc::ImuRawPayload BalancerSimulator::make_raw_imu_payload(uint64_t sim_time_us) const {
  using Nominal = HardwareNominal;
  const double body_pitch = state_.pitch;
  const double imu_height = std::max(0.0, cfg_.imu_height_m);
  const double inertial_x_accel =
      diagnostics_.x_ddot + imu_height * std::cos(body_pitch) * diagnostics_.theta_ddot -
      imu_height * std::sin(body_pitch) * state_.pitch_rate * state_.pitch_rate;
  const double inertial_z_accel =
      -imu_height * std::sin(body_pitch) * diagnostics_.theta_ddot -
      imu_height * std::cos(body_pitch) * state_.pitch_rate * state_.pitch_rate;
  const double body_x_accel =
      std::cos(body_pitch) * inertial_x_accel - std::sin(body_pitch) * inertial_z_accel;
  const double body_z_accel =
      std::sin(body_pitch) * inertial_x_accel + std::cos(body_pitch) * inertial_z_accel;

  ipc::ImuRawPayload payload{};
  payload.acc = {
      body_x_accel - Nominal::gravity * std::sin(body_pitch),
      0.0,
      body_z_accel - Nominal::gravity * std::cos(body_pitch),
  };
  payload.gyr = {
      0.0,
      state_.pitch_rate,
      0.0,
  };
  payload.timestamp_us = sim_time_us;
  return payload;
}

BalancerSimulator::LinearizedUprightModel BalancerSimulator::linearized_upright_model(
    const SimulatorPhysics& physics) {
  (void)physics;
  LinearizedUprightModel model{};

  const double T = HardwareNominal::total_mass_kg;
  const double H = HardwareNominal::first_mass_moment_kg_m;
  const double J = HardwareNominal::pitch_inertia_about_axle_kg_m2;

  const double d11 = T;
  const double d12 = H;
  const double d21 = H;
  const double d22 = J;
  const double det = d11 * d22 - d12 * d21;

  const double v_coeff = -(d22 * physics.cart_damping) / det;
  const double theta_coeff = -(d12 * HardwareNominal::gravity * H) / det;
  const double theta_dot_coeff = (d12 * physics.pitch_damping) / det;
  // Horizontal disturbances act at the COM. At upright their lever-arm
  // moment cancels the axle-force-induced angular acceleration, leaving the
  // expected translational acceleration of the complete robot mass.
  const double com_height_m = H / T;
  const double horizontal_force_to_x_ddot = d22 / det - d12 * com_height_m / det;
  const double motor_force_to_x_ddot = d12 * HardwareNominal::wheel_radius / det;

  const double q_v_coeff = (d21 * physics.cart_damping) / det;
  const double q_theta_coeff = (d11 * HardwareNominal::gravity * H) / det;
  const double q_theta_dot_coeff = -(d11 * physics.pitch_damping) / det;
  const double horizontal_force_to_q_ddot = -d21 / det + d11 * com_height_m / det;
  const double motor_force_to_q_ddot = -d11 * HardwareNominal::wheel_radius / det;

  model.A = {{
      {{0.0, 1.0, 0.0, 0.0}},
      {{0.0, v_coeff, theta_coeff, theta_dot_coeff}},
      {{0.0, 0.0, 0.0, 1.0}},
      {{0.0, q_v_coeff, q_theta_coeff, q_theta_dot_coeff}},
  }};
  model.horizontal_force_input =
      {{0.0, horizontal_force_to_x_ddot, 0.0, horizontal_force_to_q_ddot}};
  model.motor_force_input =
      {{0.0, motor_force_to_x_ddot, 0.0, motor_force_to_q_ddot}};
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
