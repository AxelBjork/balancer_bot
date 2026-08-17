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

struct StepperSnapshot {
  double commanded_microsteps = 0.0;
  double commanded_field_angle_rad = 0.0;
  double commanded_field_electrical_angle_rad = 0.0;
  double commanded_field_velocity_mps = 0.0;
  double actual_relative_angle_rad = 0.0;
  double actual_relative_mechanical_velocity_mps = 0.0;
  double actual_rotor_electrical_angle_rad = 0.0;
  double electrical_phase_error_rad = 0.0;
  double torque_nm = 0.0;
  double current_ref_a = 0.0;
  double current_ref_b = 0.0;
  double current_a = 0.0;
  double current_b = 0.0;
  double phase_voltage_a = 0.0;
  double phase_voltage_b = 0.0;
  double back_emf_a = 0.0;
  double back_emf_b = 0.0;
  double electrical_power_w = 0.0;
  double mechanical_power_w = 0.0;
  double resistive_loss_w = 0.0;
  double magnetic_energy_j = 0.0;
  bool voltage_saturated = false;
};

StepperSnapshot make_stepper_snapshot(const stepper_phase::MotorOutput& output) {
  return StepperSnapshot{
      .commanded_microsteps = output.commanded_microstep_position,
      .commanded_field_angle_rad = output.commanded_mechanical_angle_rad,
      .commanded_field_electrical_angle_rad = output.commanded_field_electrical_angle_rad,
      .commanded_field_velocity_mps = output.commanded_field_velocity_mps,
      .actual_relative_angle_rad = output.actual_relative_mechanical_angle_rad,
      .actual_relative_mechanical_velocity_mps =
          output.actual_relative_mechanical_velocity_mps,
      .actual_rotor_electrical_angle_rad = output.actual_rotor_electrical_angle_rad,
      .electrical_phase_error_rad = output.electrical_phase_error_rad,
      .torque_nm = output.torque_nm,
  };
}

StepperSnapshot make_stepper_snapshot(const stepper_phase::ElectricalMotorOutput& output) {
  return StepperSnapshot{
      .commanded_microsteps = output.commanded_microstep_position,
      .commanded_field_angle_rad = output.commanded_mechanical_angle_rad,
      .commanded_field_electrical_angle_rad = output.commanded_field_electrical_angle_rad,
      .commanded_field_velocity_mps = output.commanded_field_velocity_mps,
      .actual_relative_angle_rad = output.actual_relative_mechanical_angle_rad,
      .actual_relative_mechanical_velocity_mps =
          output.actual_relative_mechanical_velocity_mps,
      .actual_rotor_electrical_angle_rad = output.actual_rotor_electrical_angle_rad,
      .electrical_phase_error_rad = output.electrical_phase_error_rad,
      .torque_nm = output.torque_nm,
      .current_ref_a = output.current_ref_a,
      .current_ref_b = output.current_ref_b,
      .current_a = output.current_a,
      .current_b = output.current_b,
      .phase_voltage_a = output.phase_voltage_a,
      .phase_voltage_b = output.phase_voltage_b,
      .back_emf_a = output.back_emf_a,
      .back_emf_b = output.back_emf_b,
      .electrical_power_w = output.electrical_power_w,
      .mechanical_power_w = output.mechanical_power_w,
      .resistive_loss_w = output.resistive_loss_w,
      .magnetic_energy_j = output.magnetic_energy_j,
      .voltage_saturated = output.voltage_saturated,
  };
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
      return SimulatorPhysics{
          .max_force_n = HardwareNominal::combined_stall_force_n,
          .no_load_speed_mps = 1.2,
          .traction_coefficient = 1.0,
          .motor_velocity_damping = 40.0,
          // Keep nominal chassis damping at the original baseline until a separate
          // plant-damping model is justified by hardware evidence.
          .cart_damping = 1.0,
          .pitch_damping = 0.02,
          // Provisional nominal response anchored to the approximately 2 ms
          // electrical time constant. Pulse-frame and command effects remain
          // represented separately by the motor runner and phase model.
          .motor_tau_s = 0.002,
          .phase_error_limit_steps = 16.0,
          .tire_stiffness_n_per_m = 3000.0,
          .tire_damping_n_s_per_m = 35.0,
          .wheel_equivalent_mass_kg = 0.10,
      };
    case PhysicsProfile::ActuatorStress:
      return SimulatorPhysics{
          .max_force_n = HardwareNominal::combined_stall_force_n,
          .no_load_speed_mps = 1.2,
          .traction_coefficient = 1.0,
          .motor_velocity_damping = 40.0,
          .cart_damping = 1.0,
          .pitch_damping = 0.02,
          // Aggregate robustness case for unmodeled command, pulse, and
          // drivetrain delay. It is not the approximately 2 ms electrical tau.
          .motor_tau_s = 0.020,
          .phase_error_limit_steps = 16.0,
          .tire_stiffness_n_per_m = 3000.0,
          .tire_damping_n_s_per_m = 35.0,
          .wheel_equivalent_mass_kg = 0.10,
      };
    case PhysicsProfile::DirectActuator:
      return SimulatorPhysics{
          .max_force_n = HardwareNominal::combined_stall_force_n,
          .no_load_speed_mps = 1.2,
          .traction_coefficient = 1.0,
          .motor_velocity_damping = 0.0,
          // The ideal no-slip reference uses a strong low-frequency
          // translational return so completed wheel displacement and chassis
          // velocity remain in the same nominal regime. This is not a
          // hardware damping estimate.
          .cart_damping = 40.0,
          .pitch_damping = 0.02,
          // Ideal-force reference: no modeled command lag or phase-position
          // state is included in the attitude-controller authority path. Its
          // N/SPS authority is intentionally independent of wheel kinematics;
          // physical 1/32 kinematics belong to StepperPhase.
          .motor_tau_s = 0.0,
          .phase_error_limit_steps = 16.0,
          .tire_stiffness_n_per_m = 3000.0,
          .tire_damping_n_s_per_m = 35.0,
          .wheel_equivalent_mass_kg = 0.10,
          .direct_force = true,
          .direct_force_per_sps = 80.0 * HardwareNominal::meters_per_step,
      };
    case PhysicsProfile::RetiredSimpleForce:
      // Numeric profile value 4 is accepted only for source compatibility
      // with old offline callers.  It no longer recreates the 150 ms
      // SPS-to-force experiment.
      return physics_for_profile(PhysicsProfile::DirectActuator);
    case PhysicsProfile::RetiredNoSlipActuator:
      // Numeric profile value 5 is likewise a migration alias.  The
      // provisional speed-limited SPS-to-force model has been removed; callers
      // must use StepperPhaseElectrical for a physical actuator path.
      return physics_for_profile(PhysicsProfile::StepperPhaseElectrical);
    case PhysicsProfile::StepperPhase:
      return SimulatorPhysics{
          // The first-stage topology model is torque-limited by the ideal
          // holding-torque scale, not by an SPS-to-force coefficient.
          .max_force_n = HardwareNominal::stepper_combined_force_n,
          .no_load_speed_mps = 0.0,
          .traction_coefficient = 1.0,
          .motor_velocity_damping = 0.0,
          .cart_damping = 1.0,
          .pitch_damping = 0.02,
          .motor_tau_s = 0.0,
          .phase_error_limit_steps = 0.0,
          .tire_stiffness_n_per_m = 0.0,
          .tire_damping_n_s_per_m = 0.0,
          .wheel_equivalent_mass_kg = 0.0,
          .direct_force = false,
          .direct_force_per_sps = 0.0,
          .command_delay_s = 0.0,
          .speed_dependent_force_limit = false,
          .force_from_velocity_error = false,
          .stepper_phase = true,
          .stepper_rotating_inertia_kg_m2_per_motor =
              HardwareNominal::stepper_rotating_inertia_kg_m2_per_motor,
          .stepper_motor_relative_damping_nm_s_per_rad = 0.0,
          .stepper_current_limit_a = HardwareNominal::stepper_current_limit_a,
          .stepper_bus_voltage_v = 11.1,
          .max_physical_integration_step_s = 62.5e-6,
      };
    case PhysicsProfile::StepperPhaseElectrical:
      return SimulatorPhysics{
          // Electrical StepperPhase uses the same rigid no-slip equations as
          // the ideal-current reference.  The actuator itself owns the
          // current, back-EMF, and bus-voltage states.
          .max_force_n = HardwareNominal::stepper_combined_force_n,
          .no_load_speed_mps = 0.0,
          .traction_coefficient = 1.0,
          .motor_velocity_damping = 0.0,
          .cart_damping = 1.0,
          .pitch_damping = 0.02,
          .motor_tau_s = 0.0,
          .phase_error_limit_steps = 0.0,
          .tire_stiffness_n_per_m = 0.0,
          .tire_damping_n_s_per_m = 0.0,
          .wheel_equivalent_mass_kg = 0.0,
          .direct_force = false,
          .direct_force_per_sps = 0.0,
          .command_delay_s = 0.0,
          .speed_dependent_force_limit = false,
          .force_from_velocity_error = false,
          .stepper_phase = true,
          .stepper_phase_electrical = true,
          .stepper_rotating_inertia_kg_m2_per_motor =
              HardwareNominal::stepper_rotating_inertia_kg_m2_per_motor,
          .stepper_motor_relative_damping_nm_s_per_rad =
              HardwareNominal::stepper_motor_relative_damping_nm_s_per_rad,
          .stepper_current_limit_a = HardwareNominal::stepper_current_limit_a,
          .stepper_bus_voltage_v = 11.1,
          .max_physical_integration_step_s = 62.5e-6,
      };
    default:
      return physics_for_profile(PhysicsProfile::Realistic);
  }
}

std::string_view BalancerSimulator::profile_name(PhysicsProfile profile) {
  switch (profile) {
    case PhysicsProfile::Simplified:
      return "simplified";
    case PhysicsProfile::Realistic:
      return "realistic";
    case PhysicsProfile::ActuatorStress:
      return "actuator_stress";
    case PhysicsProfile::DirectActuator:
      return "direct_actuator";
    case PhysicsProfile::RetiredSimpleForce:
      return "retired_simple_force->direct_actuator";
    case PhysicsProfile::RetiredNoSlipActuator:
      return "retired_no_slip_actuator->stepper_phase_electrical";
    case PhysicsProfile::StepperPhase:
      return "stepper_phase";
    case PhysicsProfile::StepperPhaseElectrical:
      return "stepper_phase_electrical";
    default:
      return "realistic";
  }
}

BalancerSimulator::StepperMassMatrix BalancerSimulator::stepper_mass_matrix(
    const SimulatorPhysics& physics, double pitch_rad, double total_mass_scale,
    double first_mass_moment_scale, double pitch_inertia_scale) {
  return stepper_mass_matrix_with_cosine(
      physics, std::cos(pitch_rad), total_mass_scale, first_mass_moment_scale,
      pitch_inertia_scale);
}

BalancerSimulator::StepperMassMatrix BalancerSimulator::stepper_mass_matrix_with_cosine(
    const SimulatorPhysics& physics, double cos_pitch, double total_mass_scale,
    double first_mass_moment_scale, double pitch_inertia_scale) {
  const double total_mass = HardwareNominal::total_mass_kg * std::max(0.1, total_mass_scale);
  const double first_mass_moment =
      HardwareNominal::first_mass_moment_kg_m * std::max(0.1, first_mass_moment_scale);
  const double pitch_inertia =
      HardwareNominal::pitch_inertia_about_axle_kg_m2 * std::max(0.1, pitch_inertia_scale);
  const double radius = HardwareNominal::stepper_phase_wheel_radius;
  const double rotating_inertia =
      std::max(0.0, physics.stepper_rotating_inertia_kg_m2_per_motor);

  // T = 1/2 (M + 2 I_w/r^2) x_dot^2 + H cos(theta) x_dot theta_dot
  //     + 1/2 J theta_dot^2.  I_w is the absolute wheel/rotor inertia;
  // theta_relative = x/r - theta is not a kinetic-energy coordinate.
  const double d11 = total_mass + 2.0 * rotating_inertia / (radius * radius);
  const double d12 = first_mass_moment * cos_pitch;
  const double d22 = pitch_inertia;
  return StepperMassMatrix{.d11 = d11,
                           .d12 = d12,
                           .d22 = d22,
                           .determinant = d11 * d22 - d12 * d12};
}

BalancerSimulator::BalancerSimulator(const Config& cfg)
    : cfg_(cfg),
      stepper_phase_actuator_(stepper_phase::Parameters{
          .wheel_radius_m = HardwareNominal::stepper_phase_wheel_radius,
          .full_steps_per_revolution = static_cast<int>(HardwareNominal::full_steps_per_rev),
          .microsteps_per_full_step =
              static_cast<int>(HardwareNominal::stepper_phase_microsteps_per_full_step),
          .current_limit_a = HardwareNominal::stepper_current_limit_a,
          .torque_constant_nm_per_a = HardwareNominal::stepper_torque_constant_nm_per_a,
      }),
      stepper_phase_electrical_actuator_(stepper_phase::ElectricalParameters{
          .wheel_radius_m = HardwareNominal::stepper_phase_wheel_radius,
          .full_steps_per_revolution = static_cast<int>(HardwareNominal::full_steps_per_rev),
          .microsteps_per_full_step =
              static_cast<int>(HardwareNominal::stepper_phase_microsteps_per_full_step),
          .phase_resistance_ohm = 2.3,
          .phase_inductance_h = 0.0044,
          .current_limit_a = HardwareNominal::stepper_current_limit_a,
          .bus_voltage_v = 11.1,
          .torque_constant_nm_per_a = HardwareNominal::stepper_torque_constant_nm_per_a,
          .back_emf_constant_v_per_rad_s =
              HardwareNominal::stepper_back_emf_constant_v_per_rad_s,
  }) {
  physics_ = cfg_.physics_override.value_or(physics_for_profile(cfg_.physics_profile));
  if (physics_.stepper_phase) {
    auto ideal_parameters = stepper_phase_actuator_.parameters();
    ideal_parameters.current_limit_a = physics_.stepper_current_limit_a;
    stepper_phase_actuator_.set_parameters(ideal_parameters);
  }
  if (physics_.stepper_phase_electrical) {
    auto electrical_parameters = stepper_phase_electrical_actuator_.parameters();
    electrical_parameters.current_limit_a = physics_.stepper_current_limit_a;
    electrical_parameters.bus_voltage_v = physics_.stepper_bus_voltage_v;
    stepper_phase_electrical_actuator_.set_parameters(electrical_parameters);
  }
  state_.pitch = cfg_.initial_pitch_deg * kPi / 180.0;
  state_.pitch_rate = cfg_.initial_pitch_rate_dps * kPi / 180.0;
  state_.velocity = cfg_.initial_velocity_mps;
  if (physics_.stepper_phase) {
    // The default release has the rotor phase aligned with the commanded
    // field (relative rotor angle zero).  With q = x/r - pitch, the absolute
    // wheel coordinate must therefore start at r * pitch; x's origin is
    // arbitrary, but the no-slip coordinate relation is not.
    state_.position = HardwareNominal::stepper_phase_wheel_radius * state_.pitch;
  }
  wheel_velocity_mps_ = cfg_.initial_velocity_mps;
}

void BalancerSimulator::set_motor_targets(double left_sps, double right_sps) {
  left_target_sps_ = left_sps;
  right_target_sps_ = right_sps;
}

void BalancerSimulator::set_emitted_steps(double left_steps, double right_steps) {
  set_emitted_motor_steps(left_steps, right_steps);
}

void BalancerSimulator::set_emitted_motor_steps(double left_steps, double right_steps) {
  emitted_left_steps_ = left_steps;
  emitted_right_steps_ = right_steps;
  emitted_steps_avg_ = 0.5 * (left_steps + right_steps);
  have_external_emitted_steps_ = true;
  have_external_emitted_step_indices_ = false;
}

void BalancerSimulator::set_emitted_motor_step_indices(std::int64_t left_steps,
                                                        std::int64_t right_steps) {
  emitted_left_steps_ = static_cast<double>(left_steps);
  emitted_right_steps_ = static_cast<double>(right_steps);
  emitted_steps_avg_ = 0.5 * (emitted_left_steps_ + emitted_right_steps_);
  have_external_emitted_steps_ = true;
  have_external_emitted_step_indices_ = true;
}

void BalancerSimulator::set_stepper_direct_torque_for_test(double left_torque_nm,
                                                            double right_torque_nm) {
  stepper_direct_torque_for_test_ = std::array<double, 2>{left_torque_nm, right_torque_nm};
}

void BalancerSimulator::clear_stepper_direct_torque_for_test() {
  stepper_direct_torque_for_test_.reset();
}

void BalancerSimulator::set_stepper_relative_angles_for_test(double left_angle_rad,
                                                              double right_angle_rad) {
  stepper_phase_actuator_.set_actual_relative_angles_for_test(left_angle_rad,
                                                              right_angle_rad);
  stepper_phase_electrical_actuator_.set_actual_relative_angles_for_test(left_angle_rad,
                                                                          right_angle_rad);
  // Keep the hidden actuator phase state and the no-slip generalized
  // coordinates consistent for isolated ringdown/passivity fixtures.  The
  // production path starts at zero relative displacement.
  const double mean_relative_angle_rad = 0.5 * (left_angle_rad + right_angle_rad);
  state_.position = HardwareNominal::stepper_phase_wheel_radius *
                    (mean_relative_angle_rad + state_.pitch);
}

void BalancerSimulator::set_external_force_n(double force_n) {
  external_force_n_ = force_n;
}

void BalancerSimulator::set_external_com_bias_rad(double com_bias_rad) {
  external_com_bias_rad_ = com_bias_rad;
}

void BalancerSimulator::step(double dt_s) {
  const double duration_s = std::max(0.0, dt_s);
  const double max_step_s = physics_.max_physical_integration_step_s;
  if (physics_.stepper_phase && max_step_s > 0.0 && duration_s > max_step_s) {
    double remaining_s = duration_s;
    while (remaining_s > 0.0) {
      const double step_s = std::min(remaining_s, max_step_s);
      step_once(step_s);
      remaining_s -= step_s;
    }
    return;
  }
  step_once(duration_s);
}

void BalancerSimulator::step_once(double dt_s) {
  using Nominal = HardwareNominal;
  const double avg_steps_per_sec = 0.5 * (left_target_sps_ + right_target_sps_);
  const double target_wheel_velocity = steps_per_sec_to_wheel_velocity(
      avg_steps_per_sec, Nominal::steps_per_rev, Nominal::wheel_radius);
  if (physics_.stepper_phase) {
    // The motor runner supplies cumulative STEP-edge positions.  Direct
    // BalancerSimulator users do not have a runner, so retain a deliberately
    // small fallback that advances those positions from the requested SPS.
    // The engine path always takes the exact emitted-step branch.
    if (physics_.stepper_phase_continuous_field) {
      // This is deliberately a debug-only comparison path. It uses the same
      // phase actuator and mechanical equations as the discrete model, but
      // removes only STEP quantization/timing from commanded field motion.
      continuous_field_left_steps_ += left_target_sps_ * dt_s;
      continuous_field_right_steps_ += right_target_sps_ * dt_s;
      emitted_left_steps_ = continuous_field_left_steps_;
      emitted_right_steps_ = continuous_field_right_steps_;
      emitted_steps_avg_ = 0.5 * (emitted_left_steps_ + emitted_right_steps_);
      have_external_emitted_step_indices_ = false;
    } else if (!have_external_emitted_steps_) {
      emitted_left_steps_ += left_target_sps_ * dt_s;
      emitted_right_steps_ += right_target_sps_ * dt_s;
      emitted_steps_avg_ = 0.5 * (emitted_left_steps_ + emitted_right_steps_);
      have_external_emitted_step_indices_ = false;
    }
    StepperSnapshot left_snapshot{};
    StepperSnapshot right_snapshot{};
    if (physics_.stepper_phase_electrical) {
      if (have_external_emitted_step_indices_) {
        stepper_phase_electrical_actuator_.set_commanded_microstep_indices(
            static_cast<std::int64_t>(emitted_left_steps_),
            static_cast<std::int64_t>(emitted_right_steps_));
      } else {
        stepper_phase_electrical_actuator_.set_commanded_microstep_positions(
            emitted_left_steps_, emitted_right_steps_);
      }
      const auto actuator = stepper_phase_electrical_actuator_.evaluate(
          dt_s, state_.velocity, state_.pitch_rate);
      left_snapshot = make_stepper_snapshot(actuator.left);
      right_snapshot = make_stepper_snapshot(actuator.right);
    } else {
      if (have_external_emitted_step_indices_) {
        stepper_phase_actuator_.set_commanded_microstep_indices(
            static_cast<std::int64_t>(emitted_left_steps_),
            static_cast<std::int64_t>(emitted_right_steps_));
      } else {
        stepper_phase_actuator_.set_commanded_microstep_positions(emitted_left_steps_,
                                                                   emitted_right_steps_);
      }
      const auto actuator = stepper_phase_actuator_.evaluate(
          dt_s, state_.velocity, state_.pitch_rate);
      left_snapshot = make_stepper_snapshot(actuator.left);
      right_snapshot = make_stepper_snapshot(actuator.right);
    }
    double total_torque_nm = 0.0;
    const double left_phase_torque_nm = stepper_direct_torque_for_test_.has_value()
                                      ? (*stepper_direct_torque_for_test_)[0]
                                      : left_snapshot.torque_nm;
    const double right_phase_torque_nm = stepper_direct_torque_for_test_.has_value()
                                       ? (*stepper_direct_torque_for_test_)[1]
                                       : right_snapshot.torque_nm;
    const double rotor_radius = Nominal::stepper_phase_wheel_radius;
    const double left_relative_velocity_rad_s =
        left_snapshot.actual_relative_mechanical_velocity_mps / rotor_radius;
    const double right_relative_velocity_rad_s =
        right_snapshot.actual_relative_mechanical_velocity_mps / rotor_radius;
    const double damping_coefficient =
        std::max(0.0, physics_.stepper_motor_relative_damping_nm_s_per_rad);
    const double left_damping_torque_nm = -damping_coefficient * left_relative_velocity_rad_s;
    const double right_damping_torque_nm = -damping_coefficient * right_relative_velocity_rad_s;
    const double left_torque_nm = left_phase_torque_nm + left_damping_torque_nm;
    const double right_torque_nm = right_phase_torque_nm + right_damping_torque_nm;
    total_torque_nm = left_torque_nm + right_torque_nm;
    const double phase_torque_sum_nm = left_phase_torque_nm + right_phase_torque_nm;
    const double applied_force_n = total_torque_nm / Nominal::stepper_phase_wheel_radius;
    const double T = Nominal::total_mass_kg * std::max(0.1, cfg_.total_mass_scale);
    const double Q = state_.pitch + cfg_.com_angle_offset_rad + external_com_bias_rad_;
    const double Q_dot = state_.pitch_rate;
    const double sQ = std::sin(Q);
    const double cQ = std::cos(Q);
    const double H = Nominal::first_mass_moment_kg_m *
                     std::max(0.1, cfg_.first_mass_moment_scale);
    const auto mass_matrix = stepper_mass_matrix_with_cosine(
        physics_, cQ, cfg_.total_mass_scale, cfg_.first_mass_moment_scale,
        cfg_.pitch_inertia_scale);
    const double d11 = mass_matrix.d11;
    const double d12 = mass_matrix.d12;
    const double d21 = d12;
    const double d22 = mass_matrix.d22;
    const double rhs1 = applied_force_n + external_force_n_ + H * Q_dot * Q_dot * sQ -
                        physics_.cart_damping * state_.velocity;
    const double com_height_m = H / T;
    const double external_force_pitch_moment = external_force_n_ * com_height_m * cQ;
    // The motor torque enters the constrained equations once: +T/r in the
    // translation equation and the equal/opposite -T reaction on the body.
    const double rhs2 = physics_.gravity_mps2 * H * sQ -
                        physics_.pitch_damping * state_.pitch_rate -
                        total_torque_nm + external_force_pitch_moment;
    const double det = mass_matrix.determinant;
    const double x_ddot = (d22 * rhs1 - d12 * rhs2) / det;
    const double theta_ddot = (d11 * rhs2 - d21 * rhs1) / det;
    const double velocity_before = state_.velocity;
    const double pitch_rate_before = state_.pitch_rate;
    state_.velocity += x_ddot * dt_s;
    state_.position += 0.5 * (velocity_before + state_.velocity) * dt_s;
    state_.pitch_rate += theta_ddot * dt_s;
    state_.pitch += 0.5 * (pitch_rate_before + state_.pitch_rate) * dt_s;
    if (std::abs(state_.pitch) > kPi / 2.0) {
      state_.pitch = state_.pitch > 0 ? kPi / 2.0 : -kPi / 2.0;
      state_.pitch_rate = 0.0;
      state_.velocity = 0.0;
    }
    if (physics_.stepper_phase_electrical) {
      stepper_phase_electrical_actuator_.advance_mechanical_state(
          dt_s, velocity_before, pitch_rate_before, state_.velocity, state_.pitch_rate);
    } else {
      stepper_phase_actuator_.advance_mechanical_state(
          dt_s, velocity_before, pitch_rate_before, state_.velocity, state_.pitch_rate);
    }

    const double average_phase_error =
        0.5 * (left_snapshot.electrical_phase_error_rad +
               right_snapshot.electrical_phase_error_rad);
    const double average_torque = phase_torque_sum_nm;
    actual_wheel_velocity_ =
        state_.velocity - Nominal::stepper_phase_wheel_radius * state_.pitch_rate;
    diagnostics_.target_wheel_velocity =
        0.5 * (left_snapshot.commanded_field_velocity_mps +
               right_snapshot.commanded_field_velocity_mps);
    diagnostics_.actual_wheel_velocity = actual_wheel_velocity_;
    diagnostics_.velocity_error = diagnostics_.target_wheel_velocity - actual_wheel_velocity_;
    diagnostics_.f_cmd = applied_force_n;
    diagnostics_.f_app = applied_force_n;
    diagnostics_.desired_drive_force = applied_force_n;
    diagnostics_.limited_drive_force = applied_force_n;
    diagnostics_.applied_drive_force = applied_force_n;
    diagnostics_.desired_tire_force = 0.0;
    diagnostics_.external_force_n = external_force_n_;
    diagnostics_.external_com_bias_rad = external_com_bias_rad_;
    diagnostics_.x_ddot = x_ddot;
    diagnostics_.theta_ddot = theta_ddot;
    diagnostics_.phase_error_steps = average_phase_error /
                                     Nominal::stepper_phase_electrical_radians_per_step;
    diagnostics_.missed_steps = diagnostics_.phase_error_steps;
    diagnostics_.traction_limit_n = Nominal::stepper_combined_force_n;
    diagnostics_.motor_force_limit_n = Nominal::stepper_combined_force_n;
    diagnostics_.command_saturated = false;
    diagnostics_.phase_saturated = false;
    diagnostics_.motor_force_saturated = false;
    diagnostics_.traction_saturated = false;
    diagnostics_.stepper_commanded_microsteps_left = left_snapshot.commanded_microsteps;
    diagnostics_.stepper_commanded_microsteps_right = right_snapshot.commanded_microsteps;
    diagnostics_.stepper_commanded_field_angle_left_rad =
        left_snapshot.commanded_field_angle_rad;
    diagnostics_.stepper_commanded_field_angle_right_rad =
        right_snapshot.commanded_field_angle_rad;
    diagnostics_.stepper_commanded_field_electrical_angle_left_rad =
        left_snapshot.commanded_field_electrical_angle_rad;
    diagnostics_.stepper_commanded_field_electrical_angle_right_rad =
        right_snapshot.commanded_field_electrical_angle_rad;
    diagnostics_.stepper_commanded_field_velocity_mps = diagnostics_.target_wheel_velocity;
    diagnostics_.stepper_actual_relative_angle_left_rad =
        left_snapshot.actual_relative_angle_rad;
    diagnostics_.stepper_actual_relative_angle_right_rad =
        right_snapshot.actual_relative_angle_rad;
    diagnostics_.stepper_actual_rotor_electrical_angle_left_rad =
        left_snapshot.actual_rotor_electrical_angle_rad;
    diagnostics_.stepper_actual_rotor_electrical_angle_right_rad =
        right_snapshot.actual_rotor_electrical_angle_rad;
    diagnostics_.stepper_electrical_phase_error_left_rad =
        left_snapshot.electrical_phase_error_rad;
    diagnostics_.stepper_electrical_phase_error_right_rad =
        right_snapshot.electrical_phase_error_rad;
    diagnostics_.stepper_torque_left_nm = left_phase_torque_nm;
    diagnostics_.stepper_torque_right_nm = right_phase_torque_nm;
    diagnostics_.stepper_summed_torque_nm = average_torque;
    diagnostics_.stepper_damping_torque_left_nm = left_damping_torque_nm;
    diagnostics_.stepper_damping_torque_right_nm = right_damping_torque_nm;
    diagnostics_.stepper_applied_torque_left_nm = left_torque_nm;
    diagnostics_.stepper_applied_torque_right_nm = right_torque_nm;
    diagnostics_.stepper_motor_relative_velocity_left_rad_s = left_relative_velocity_rad_s;
    diagnostics_.stepper_motor_relative_velocity_right_rad_s = right_relative_velocity_rad_s;
    diagnostics_.stepper_actual_wheel_velocity_mps = actual_wheel_velocity_;
    diagnostics_.stepper_chassis_velocity_mps = state_.velocity;
    diagnostics_.stepper_current_ref_a_left = left_snapshot.current_ref_a;
    diagnostics_.stepper_current_ref_b_left = left_snapshot.current_ref_b;
    diagnostics_.stepper_current_a_left = left_snapshot.current_a;
    diagnostics_.stepper_current_b_left = left_snapshot.current_b;
    diagnostics_.stepper_phase_voltage_a_left = left_snapshot.phase_voltage_a;
    diagnostics_.stepper_phase_voltage_b_left = left_snapshot.phase_voltage_b;
    diagnostics_.stepper_back_emf_a_left = left_snapshot.back_emf_a;
    diagnostics_.stepper_back_emf_b_left = left_snapshot.back_emf_b;
    diagnostics_.stepper_electrical_power_left_w = left_snapshot.electrical_power_w;
    diagnostics_.stepper_mechanical_power_left_w = left_snapshot.mechanical_power_w;
    diagnostics_.stepper_resistive_loss_left_w = left_snapshot.resistive_loss_w;
    diagnostics_.stepper_magnetic_energy_left_j = left_snapshot.magnetic_energy_j;
    diagnostics_.stepper_current_ref_a_right = right_snapshot.current_ref_a;
    diagnostics_.stepper_current_ref_b_right = right_snapshot.current_ref_b;
    diagnostics_.stepper_current_a_right = right_snapshot.current_a;
    diagnostics_.stepper_current_b_right = right_snapshot.current_b;
    diagnostics_.stepper_phase_voltage_a_right = right_snapshot.phase_voltage_a;
    diagnostics_.stepper_phase_voltage_b_right = right_snapshot.phase_voltage_b;
    diagnostics_.stepper_back_emf_a_right = right_snapshot.back_emf_a;
    diagnostics_.stepper_back_emf_b_right = right_snapshot.back_emf_b;
    diagnostics_.stepper_electrical_power_right_w = right_snapshot.electrical_power_w;
    diagnostics_.stepper_mechanical_power_right_w = right_snapshot.mechanical_power_w;
    diagnostics_.stepper_resistive_loss_right_w = right_snapshot.resistive_loss_w;
    diagnostics_.stepper_magnetic_energy_right_j = right_snapshot.magnetic_energy_j;
    diagnostics_.stepper_voltage_saturated_left = left_snapshot.voltage_saturated;
    diagnostics_.stepper_voltage_saturated_right = right_snapshot.voltage_saturated;
    return;
  }
  if (physics_.direct_force) {
    actuator_time_s_ += std::max(0.0, dt_s);
    double actuator_command_sps = avg_steps_per_sec;
    if (physics_.command_delay_s > 0.0) {
      command_history_.push_back(DelayedCommand{actuator_time_s_, avg_steps_per_sec});
      const double target_time_s = actuator_time_s_ - physics_.command_delay_s;
      while (command_history_.size() >= 2 &&
             command_history_[1].time_s <= target_time_s) {
        command_history_.pop_front();
      }
      if (!command_history_.empty() && command_history_.front().time_s <= target_time_s) {
        delayed_command_sps_ = command_history_.front().command_sps;
      }
      actuator_command_sps = delayed_command_sps_;
    } else {
      command_history_.clear();
      delayed_command_sps_ = avg_steps_per_sec;
    }
    const double T = Nominal::total_mass_kg * std::max(0.1, cfg_.total_mass_scale);
    const double delayed_target_wheel_velocity = steps_per_sec_to_wheel_velocity(
        actuator_command_sps, Nominal::steps_per_rev, Nominal::wheel_radius);
    const double relative_wheel_velocity =
        state_.velocity - Nominal::wheel_radius * state_.pitch_rate;
    const double desired_force = physics_.force_from_velocity_error
                                     ? physics_.motor_velocity_damping *
                                           (delayed_target_wheel_velocity -
                                            relative_wheel_velocity)
                                     : physics_.direct_force_per_sps * actuator_command_sps;
    const double traction_limit =
        std::max(0.0, physics_.traction_coefficient) * T * physics_.gravity_mps2;
    const double fixed_motor_limit = std::max(0.0, physics_.max_force_n);
    const double speed_fraction = physics_.speed_dependent_force_limit &&
                                          physics_.no_load_speed_mps > 0.0
                                      ? std::abs(relative_wheel_velocity) /
                                            physics_.no_load_speed_mps
                                      : 0.0;
    const double speed_motor_limit = physics_.speed_dependent_force_limit
                                         ? fixed_motor_limit *
                                               std::clamp(1.0 - speed_fraction, 0.0, 1.0)
                                         : fixed_motor_limit;
    const double motor_limit = std::min(fixed_motor_limit, speed_motor_limit);
    const double available_force = std::min(motor_limit, traction_limit);
    const double limited_force =
        std::clamp(desired_force, -available_force, available_force);
    const double force_alpha = physics_.motor_tau_s > 0.0
                                   ? std::clamp(dt_s / (physics_.motor_tau_s + dt_s), 0.0, 1.0)
                                   : 1.0;
    applied_drive_force_ += force_alpha * (limited_force - applied_drive_force_);

    const double Q = state_.pitch + cfg_.com_angle_offset_rad + external_com_bias_rad_;
    const double Q_dot = state_.pitch_rate;
    const double sQ = std::sin(Q);
    const double cQ = std::cos(Q);
    const double H = Nominal::first_mass_moment_kg_m *
                     std::max(0.1, cfg_.first_mass_moment_scale);
    const double J = Nominal::pitch_inertia_about_axle_kg_m2 *
                     std::max(0.1, cfg_.pitch_inertia_scale);
    const double d11 = T;
    const double d12 = H * cQ;
    const double d21 = H * cQ;
    const double d22 = J;
    const double rhs1 = applied_drive_force_ + external_force_n_ + H * Q_dot * Q_dot * sQ -
                        physics_.cart_damping * state_.velocity;
    const double motor_reaction_torque = applied_drive_force_ * Nominal::wheel_radius;
    const double com_height_m = H / T;
    const double external_force_pitch_moment = external_force_n_ * com_height_m * cQ;
    const double rhs2 = physics_.gravity_mps2 * H * sQ - physics_.pitch_damping * state_.pitch_rate -
                        motor_reaction_torque + external_force_pitch_moment;
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

    // In the no-slip reference, chassis translation is the completed-wheel
    // displacement seen by the outer-loop observer.
    actual_wheel_velocity_ = state_.velocity;
    diagnostics_.target_wheel_velocity = target_wheel_velocity;
    diagnostics_.actual_wheel_velocity = actual_wheel_velocity_;
    diagnostics_.velocity_error = target_wheel_velocity - actual_wheel_velocity_;
    diagnostics_.f_cmd = desired_force;
    diagnostics_.f_app = applied_drive_force_;
    diagnostics_.desired_drive_force = desired_force;
    diagnostics_.limited_drive_force = limited_force;
    diagnostics_.applied_drive_force = applied_drive_force_;
    diagnostics_.desired_tire_force = 0.0;
    diagnostics_.external_force_n = external_force_n_;
    diagnostics_.external_com_bias_rad = external_com_bias_rad_;
    diagnostics_.x_ddot = x_ddot;
    diagnostics_.theta_ddot = theta_ddot;
    diagnostics_.phase_error_steps = 0.0;
    diagnostics_.missed_steps = 0.0;
    diagnostics_.traction_limit_n = traction_limit;
    diagnostics_.motor_force_limit_n = motor_limit;
    diagnostics_.command_saturated = available_force <= 0.0 ||
                                     std::abs(desired_force) >= available_force * 0.999;
    diagnostics_.phase_saturated = false;
    diagnostics_.motor_force_saturated = motor_limit <= 0.0 ||
                                         std::abs(desired_force) >= motor_limit * 0.999;
    diagnostics_.traction_saturated = traction_limit <= 0.0 ||
                                      std::abs(desired_force) >= traction_limit * 0.999;
    return;
  }
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
      std::max(0.0, physics_.traction_coefficient) * scaled_robot_mass * physics_.gravity_mps2;

  double effective_command_position_m = emitted_position_m - missed_distance_m_;
  double phase_error_m = effective_command_position_m - relative_wheel_position_m;
  const double phase_limit_m =
      std::max(1.0, physics_.phase_error_limit_steps) * Nominal::meters_per_step;
  const bool phase_saturated = std::abs(phase_error_m) > phase_limit_m;
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
  const bool motor_force_saturated =
      motor_force_limit <= 0.0 || std::abs(desired_force) >= motor_force_limit * 0.999;

  const double force_alpha = (physics_.motor_tau_s > 0.0)
                                 ? std::clamp(dt_s / (physics_.motor_tau_s + dt_s), 0.0, 1.0)
                                 : 1.0;
  applied_drive_force_ += force_alpha * (limited_motor_force - applied_drive_force_);
  const double desired_tire_force =
      physics_.tire_stiffness_n_per_m * (wheel_position_m_ - state_.position) +
      physics_.tire_damping_n_s_per_m * (wheel_velocity_mps_ - state_.velocity);
  const double tire_force = std::clamp(desired_tire_force, -traction_limit, traction_limit);
  const bool traction_saturated =
      traction_limit <= 0.0 || std::abs(desired_tire_force) >= traction_limit * 0.999;
  const double F_cmd = desired_force;
  const double F_app = tire_force;
  const double total_force = F_app + external_force_n_;

  const double Q = state_.pitch + cfg_.com_angle_offset_rad + external_com_bias_rad_;
  const double Q_dot = state_.pitch_rate;
  const double sQ = std::sin(Q);
  const double cQ = std::cos(Q);

  const double T = Nominal::total_mass_kg * std::max(0.1, cfg_.total_mass_scale);
  const double H = Nominal::first_mass_moment_kg_m *
                   std::max(0.1, cfg_.first_mass_moment_scale);
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
  const double com_height_m = H / T;
  const double external_force_pitch_moment = external_force_n_ * com_height_m * cQ;
  const double rhs2 = physics_.gravity_mps2 * H * sQ -
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
  diagnostics_.desired_drive_force = desired_force;
  diagnostics_.limited_drive_force = limited_motor_force;
  diagnostics_.applied_drive_force = applied_drive_force_;
  diagnostics_.desired_tire_force = desired_tire_force;
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
  diagnostics_.phase_saturated = phase_saturated;
  diagnostics_.motor_force_saturated = motor_force_saturated;
  diagnostics_.traction_saturated = traction_saturated;
}

ipc::ImuRawPayload BalancerSimulator::make_raw_imu_payload(uint64_t sim_time_us) const {
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
      body_x_accel - physics_.gravity_mps2 * std::sin(body_pitch),
      0.0,
      body_z_accel - physics_.gravity_mps2 * std::cos(body_pitch),
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

  const double rotating_inertia =
      std::max(0.0, physics.stepper_rotating_inertia_kg_m2_per_motor);
  const double d11 = T +
                     (physics.stepper_phase ? 2.0 * rotating_inertia /
                                                 (HardwareNominal::stepper_phase_wheel_radius *
                                                  HardwareNominal::stepper_phase_wheel_radius)
                                             : 0.0);
  // Attached wheel/rotor inertia has absolute wheel speed x_dot/r.  The
  // relative rotor/stator angle is a magnetic coordinate and contributes no
  // inertial cross terms when the no-slip constraint is applied.
  const double d12 = H;
  const double d21 = d12;
  const double d22 = J;
  const double det = d11 * d22 - d12 * d21;

  const double v_coeff = -(d22 * physics.cart_damping) / det;
  const double theta_coeff = -(d12 * physics.gravity_mps2 * H) / det;
  const double theta_dot_coeff = (d12 * physics.pitch_damping) / det;
  // Horizontal disturbances act at the COM. At upright their lever-arm
  // moment cancels the axle-force-induced angular acceleration, leaving the
  // expected translational acceleration of the complete robot mass.
  const double com_height_m = H / T;
  const double horizontal_force_to_x_ddot = d22 / det - d12 * com_height_m / det;
  const double motor_force_to_x_ddot = d12 * HardwareNominal::wheel_radius / det;

  const double q_v_coeff = (d21 * physics.cart_damping) / det;
  const double q_theta_coeff = (d11 * physics.gravity_mps2 * H) / det;
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
