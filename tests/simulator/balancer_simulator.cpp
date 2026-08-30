#include "balancer_simulator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

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
  double unwrapped_electrical_phase_error_rad = 0.0;
  double field_rotor_relative_velocity_rad_s = 0.0;
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
      .unwrapped_electrical_phase_error_rad = output.unwrapped_electrical_phase_error_rad,
      .field_rotor_relative_velocity_rad_s = output.field_rotor_relative_velocity_rad_s,
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
      .unwrapped_electrical_phase_error_rad = output.unwrapped_electrical_phase_error_rad,
      .field_rotor_relative_velocity_rad_s = output.field_rotor_relative_velocity_rad_s,
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

template <std::size_t N>
bool solve_linear_system(std::array<std::array<double, N + 1>, N> augmented,
                         std::array<double, N>* solution) {
  for (std::size_t column = 0; column < N; ++column) {
    std::size_t pivot = column;
    double pivot_magnitude = std::abs(augmented[pivot][column]);
    for (std::size_t row = column + 1; row < N; ++row) {
      const double magnitude = std::abs(augmented[row][column]);
      if (magnitude > pivot_magnitude) {
        pivot = row;
        pivot_magnitude = magnitude;
      }
    }
    if (pivot_magnitude < 1.0e-14) return false;
    if (pivot != column) std::swap(augmented[pivot], augmented[column]);

    const double diagonal = augmented[column][column];
    for (std::size_t entry = column; entry <= N; ++entry) {
      augmented[column][entry] /= diagonal;
    }
    for (std::size_t row = 0; row < N; ++row) {
      if (row == column) continue;
      const double factor = augmented[row][column];
      if (factor == 0.0) continue;
      for (std::size_t entry = column; entry <= N; ++entry) {
        augmented[row][entry] -= factor * augmented[column][entry];
      }
    }
  }
  for (std::size_t row = 0; row < N; ++row) {
    (*solution)[row] = augmented[row][N];
  }
  return true;
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
          // Electrical StepperPhase uses an independent wheel/rotor state
          // coupled to chassis motion through the contact model below. The
          // actuator itself owns the current, back-EMF, and bus-voltage
          // states.
          .max_force_n = HardwareNominal::stepper_combined_force_n,
          .no_load_speed_mps = 0.0,
          .traction_coefficient = 1.0,
          .motor_velocity_damping = 0.0,
          .cart_damping = 1.0,
          // Hardware steady-drive captures imply approximately 0.4 N of
          // velocity-independent wheel-ground resistance after the existing
          // 1 N*s/m chassis term is accounted for at 0.3 m/s.
          .rolling_resistance_force_n = 0.4,
          // The same force is the conservative first breakaway bracket: it
          // lets the electrical plant hold a small lean at zero speed, as the
          // hardware does, without changing the moving-force estimate.
          .static_breakaway_force_n = 0.4,
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
  if (physics_.stepper_phase_electrical) {
    const double radius = HardwareNominal::stepper_phase_wheel_radius;
    stepper_left_relative_velocity_mps_ =
        cfg_.initial_velocity_mps - radius * state_.pitch_rate;
    stepper_right_relative_velocity_mps_ = stepper_left_relative_velocity_mps_;
  }
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
  if (physics_.stepper_phase_electrical) {
    const double radius = HardwareNominal::stepper_phase_wheel_radius;
    const double relative_velocity = state_.velocity - radius * state_.pitch_rate;
    stepper_left_relative_velocity_mps_ = relative_velocity;
    stepper_right_relative_velocity_mps_ = relative_velocity;
    stepper_contact_sticking_ = true;
    stepper_have_cycle_indices_ = false;
    stepper_accumulated_cycle_slips_left_ = 0;
    stepper_accumulated_cycle_slips_right_ = 0;
    stepper_accumulated_slip_distance_m_ = 0.0;
  }
}

void BalancerSimulator::set_external_force_n(double force_n) {
  external_force_n_ = force_n;
}

void BalancerSimulator::set_external_com_bias_rad(double com_bias_rad) {
  external_com_bias_rad_ = com_bias_rad;
}

void BalancerSimulator::place_on_brace_for_test(double pitch_deg) {
  // This is a simulator fixture for an already-resting robot.  It changes
  // only the physical state; the controller and estimator continue from
  // their existing states on the next simulated sample.
  state_.pitch = std::clamp(pitch_deg * kPi / 180.0, -0.5 * kPi, 0.5 * kPi);
  state_.pitch_rate = 0.0;
  state_.velocity = 0.0;
  wheel_velocity_mps_ = 0.0;
  if (physics_.stepper_phase_electrical) {
    stepper_left_relative_velocity_mps_ = 0.0;
    stepper_right_relative_velocity_mps_ = 0.0;
    stepper_contact_sticking_ = true;
  }
}

void BalancerSimulator::step(double dt_s) {
  using Nominal = HardwareNominal;
  const double duration_s = std::max(0.0, dt_s);
  if (physics_.stepper_phase_electrical) {
    // The engine presents cumulative STEP positions at event boundaries. The
    // field changes discretely at those boundaries; keep that command fixed
    // while resolving the electrical/mechanical state with smaller physical
    // substeps. The interval-average field speed is passed separately for
    // diagnostics, so polling boundaries cannot change the physical result.
    double end_left_steps = emitted_left_steps_;
    double end_right_steps = emitted_right_steps_;
    if (physics_.stepper_phase_continuous_field || !have_external_emitted_steps_) {
      const auto& previous_output = stepper_phase_electrical_actuator_.output();
      end_left_steps = previous_output.left.commanded_microstep_position +
                       left_target_sps_ * duration_s;
      end_right_steps = previous_output.right.commanded_microstep_position +
                        right_target_sps_ * duration_s;
      emitted_left_steps_ = end_left_steps;
      emitted_right_steps_ = end_right_steps;
      emitted_steps_avg_ = 0.5 * (end_left_steps + end_right_steps);
      continuous_field_left_steps_ = end_left_steps;
      continuous_field_right_steps_ = end_right_steps;
      have_external_emitted_step_indices_ = false;
    }

    const auto& previous_output = stepper_phase_electrical_actuator_.output();
    const double field_velocity_left_mps =
        duration_s > 0.0
            ? (end_left_steps - previous_output.left.commanded_microstep_position) *
                  Nominal::stepper_phase_meters_per_step / duration_s
            : 0.0;
    const double field_velocity_right_mps =
        duration_s > 0.0
            ? (end_right_steps - previous_output.right.commanded_microstep_position) *
                  Nominal::stepper_phase_meters_per_step / duration_s
            : 0.0;

    if (duration_s <= 0.0) {
      step_stepper_electrical_once(0.0, end_left_steps, end_right_steps, 0.0, 0.0);
    } else {
      const double max_step_s = physics_.max_physical_integration_step_s;
      const double physical_step_s = max_step_s > 0.0 ? max_step_s : duration_s;
      double elapsed_s = 0.0;
      while (elapsed_s < duration_s) {
        const double step_s = std::min(duration_s - elapsed_s, physical_step_s);
        step_stepper_electrical_once(step_s, end_left_steps, end_right_steps,
                                     field_velocity_left_mps, field_velocity_right_mps);
        elapsed_s += step_s;
      }
    }
    return;
  }
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

void BalancerSimulator::step_stepper_electrical_once(double dt_s, double commanded_left_steps,
                                                     double commanded_right_steps,
                                                     double field_velocity_left_mps,
                                                     double field_velocity_right_mps) {
  using Nominal = HardwareNominal;

  // The cumulative STEP position is an event-driven field command. It is
  // held constant for this physical substep; field speed is supplied as an
  // interval-average diagnostic below.
  stepper_phase_electrical_actuator_.set_commanded_microstep_positions(
      commanded_left_steps, commanded_right_steps);
  const auto actuator = stepper_phase_electrical_actuator_.evaluate_relative_with_field_velocity(
      dt_s, stepper_left_relative_velocity_mps_, stepper_right_relative_velocity_mps_,
      field_velocity_left_mps, field_velocity_right_mps);
  const StepperSnapshot left_snapshot = make_stepper_snapshot(actuator.left);
  const StepperSnapshot right_snapshot = make_stepper_snapshot(actuator.right);

  const double left_phase_torque_nm = stepper_direct_torque_for_test_.has_value()
                                         ? (*stepper_direct_torque_for_test_)[0]
                                         : left_snapshot.torque_nm;
  const double right_phase_torque_nm = stepper_direct_torque_for_test_.has_value()
                                          ? (*stepper_direct_torque_for_test_)[1]
                                          : right_snapshot.torque_nm;
  const double radius = Nominal::stepper_phase_wheel_radius;
  const double damping_coefficient =
      std::max(0.0, physics_.stepper_motor_relative_damping_nm_s_per_rad);
  const double left_relative_rate = stepper_left_relative_velocity_mps_ / radius;
  const double right_relative_rate = stepper_right_relative_velocity_mps_ / radius;
  const double left_damping_torque_nm = -damping_coefficient * left_relative_rate;
  const double right_damping_torque_nm = -damping_coefficient * right_relative_rate;
  const double left_torque_nm = left_phase_torque_nm + left_damping_torque_nm;
  const double right_torque_nm = right_phase_torque_nm + right_damping_torque_nm;
  const double total_torque_nm = left_torque_nm + right_torque_nm;

  const double total_mass = Nominal::total_mass_kg * std::max(0.1, cfg_.total_mass_scale);
  const double first_mass_moment =
      Nominal::first_mass_moment_kg_m * std::max(0.1, cfg_.first_mass_moment_scale);
  const double Q = state_.pitch + cfg_.com_angle_offset_rad + external_com_bias_rad_;
  const double Q_dot = state_.pitch_rate;
  const double sQ = std::sin(Q);
  const double cQ = std::cos(Q);
  const double pitch_inertia =
      Nominal::pitch_inertia_about_axle_kg_m2 * std::max(0.1, cfg_.pitch_inertia_scale);
  const double wheel_inertia =
      std::max(1.0e-12, physics_.stepper_rotating_inertia_kg_m2_per_motor);
  const double body_x_rhs = external_force_n_ + first_mass_moment * Q_dot * Q_dot * sQ -
                            physics_.cart_damping * state_.velocity;
  const double com_height_m = first_mass_moment / total_mass;
  const double brace_angle_rad =
      std::clamp(std::abs(cfg_.brace_pitch_deg), 0.0, 89.0) * kPi / 180.0;
  const double brace_penetration_rad =
      cfg_.brace_enabled ? std::max(0.0, std::abs(state_.pitch) - brace_angle_rad) : 0.0;
  const double pitch_sign = state_.pitch >= 0.0 ? 1.0 : -1.0;
  const double outward_rate_rad_s =
      cfg_.brace_enabled && std::abs(state_.pitch) >= brace_angle_rad &&
              state_.pitch * state_.pitch_rate > 0.0
          ? std::abs(state_.pitch_rate)
          : 0.0;
  const double brace_torque_nm =
      cfg_.brace_enabled && (brace_penetration_rad > 0.0 || outward_rate_rad_s > 0.0)
          ? -pitch_sign * (std::max(0.0, cfg_.brace_stiffness_nm_per_rad) *
                               brace_penetration_rad +
                           std::max(0.0, cfg_.brace_damping_nm_s_per_rad) *
                               outward_rate_rad_s)
          : 0.0;
  const bool brace_contact_active = cfg_.brace_enabled &&
                                    (brace_penetration_rad > 0.0 || outward_rate_rad_s > 0.0);
  const double body_pitch_rhs = physics_.gravity_mps2 * first_mass_moment * sQ -
                                physics_.pitch_damping * Q_dot +
                                external_force_n_ * com_height_m * cQ + brace_torque_nm;

  const double body_normal_force =
      std::max(0.0, total_mass * std::max(0.0, physics_.gravity_mps2) * cQ);
  const double traction_limit_each =
      std::max(0.0, physics_.traction_coefficient) * body_normal_force * 0.5;
  const double rolling_force_each =
      std::max(0.0, physics_.rolling_resistance_force_n) * 0.5;

  // The four mechanical coordinates are chassis translation, body pitch, and
  // the two wheel/rotor angles relative to the body.  The wheel's absolute
  // angular rate is alpha_dot + pitch_dot, so its ground contact velocity is
  // x_dot - r * (alpha_dot + pitch_dot).
  const auto solve_fixed_contact = [&](double left_contact_force_n,
                                       double right_contact_force_n,
                                       double rolling_resistance_force_n) {
    std::array<std::array<double, 5>, 4> equations{{
        {{total_mass, first_mass_moment * cQ, 0.0, 0.0,
          body_x_rhs + rolling_resistance_force_n + left_contact_force_n +
              right_contact_force_n}},
        {{first_mass_moment * cQ, pitch_inertia + 2.0 * wheel_inertia, wheel_inertia,
          wheel_inertia,
          body_pitch_rhs - radius * (left_contact_force_n + right_contact_force_n)}},
        {{0.0, wheel_inertia, wheel_inertia, 0.0,
          left_torque_nm - radius * left_contact_force_n}},
        {{0.0, wheel_inertia, 0.0, wheel_inertia,
          right_torque_nm - radius * right_contact_force_n}},
    }};
    std::array<double, 4> solution{};
    const bool valid = solve_linear_system<4>(equations, &solution);
    return std::pair<bool, std::array<double, 4>>{valid, solution};
  };

  // Solve the acceleration-level no-slip constraints while retaining the two
  // contact forces as Lagrange multipliers. The solved C forces are the tire
  // traction forces. The known
  // rolling/Coulomb resistance is an independent generalized chassis force;
  // it is never included in the traction limit comparison.
  const auto solve_sticking = [&](double rolling_resistance_force_n) {
    std::array<std::array<double, 7>, 6> equations{{
        {{total_mass, first_mass_moment * cQ, 0.0, 0.0, -1.0, -1.0,
          body_x_rhs + rolling_resistance_force_n}},
        {{first_mass_moment * cQ, pitch_inertia + 2.0 * wheel_inertia, wheel_inertia,
          wheel_inertia, radius, radius, body_pitch_rhs}},
        {{0.0, wheel_inertia, wheel_inertia, 0.0, radius, 0.0,
          left_torque_nm}},
        {{0.0, wheel_inertia, 0.0, wheel_inertia, 0.0, radius,
          right_torque_nm}},
        {{1.0, -radius, -radius, 0.0, 0.0, 0.0, 0.0}},
        {{1.0, -radius, 0.0, -radius, 0.0, 0.0, 0.0}},
    }};
    std::array<double, 6> solution{};
    const bool valid = solve_linear_system<6>(equations, &solution);
    return std::pair<bool, std::array<double, 6>>{valid, solution};
  };

  std::array<double, 4> acceleration{};
  double requested_left_contact_n = 0.0;
  double requested_right_contact_n = 0.0;
  double actual_left_contact_n = 0.0;
  double actual_right_contact_n = 0.0;
  bool static_contact = false;
  bool sticking = false;

  const auto mass_matrix = stepper_mass_matrix_with_cosine(
      physics_, cQ, cfg_.total_mass_scale, cfg_.first_mass_moment_scale,
      cfg_.pitch_inertia_scale);
  const double total_motor_force_n = total_torque_nm / radius;
  const double reduced_pitch_rhs = body_pitch_rhs - total_torque_nm;
  const double reduced_force_rhs = body_x_rhs + total_motor_force_n;
  const bool near_zero_contact_velocity =
      std::abs(state_.velocity) <= 1.0e-4 &&
      std::abs(stepper_left_relative_velocity_mps_ + radius * state_.pitch_rate) <= 1.0e-4 &&
      std::abs(stepper_right_relative_velocity_mps_ + radius * state_.pitch_rate) <= 1.0e-4;
  const double required_static_total_force_n =
      mass_matrix.d22 > 1.0e-12 ? mass_matrix.d12 * reduced_pitch_rhs / mass_matrix.d22 : 0.0;
  const double required_static_resistance_n =
      required_static_total_force_n - reduced_force_rhs;
  const double total_traction_limit_n = 2.0 * traction_limit_each;
  if (cfg_.stepper_lock_pitch_for_test) {
    // The isolated actuator envelope holds the body pitch coordinate fixed by
    // an ideal external fixture.  This removes balance-controller dynamics
    // from the contact experiment while retaining chassis translation and the
    // two independent wheel/rotor coordinates.
    const auto solve_locked_fixed_contact = [&](double left_contact_force_n,
                                                double right_contact_force_n,
                                                double rolling_resistance_force_n) {
      std::array<std::array<double, 4>, 3> equations{{
          {{total_mass, 0.0, 0.0,
            body_x_rhs + rolling_resistance_force_n + left_contact_force_n +
                right_contact_force_n}},
        {{0.0, wheel_inertia, 0.0,
            left_torque_nm - radius * left_contact_force_n}},
          {{0.0, 0.0, wheel_inertia,
            right_torque_nm - radius * right_contact_force_n}},
      }};
      std::array<double, 3> solution{};
      const bool valid = solve_linear_system<3>(equations, &solution);
      return std::pair<bool, std::array<double, 3>>{valid, solution};
    };
    const auto solve_locked_sticking = [&](double rolling_resistance_force_n) {
      std::array<std::array<double, 6>, 5> equations{{
          {{total_mass, 0.0, 0.0, -1.0, -1.0,
            body_x_rhs + rolling_resistance_force_n}},
          {{0.0, wheel_inertia, 0.0, radius, 0.0,
            left_torque_nm}},
          {{0.0, 0.0, wheel_inertia, 0.0, radius,
            right_torque_nm}},
          {{1.0, -radius, 0.0, 0.0, 0.0, 0.0}},
          {{1.0, 0.0, -radius, 0.0, 0.0, 0.0}},
      }};
      std::array<double, 5> solution{};
      const bool valid = solve_linear_system<5>(equations, &solution);
      return std::pair<bool, std::array<double, 5>>{valid, solution};
    };

    const bool near_zero_locked_velocity =
        std::abs(state_.velocity) <= 1.0e-4 &&
        std::abs(stepper_left_relative_velocity_mps_) <= 1.0e-4 &&
        std::abs(stepper_right_relative_velocity_mps_) <= 1.0e-4;
    const double static_left_contact = left_torque_nm / radius;
    const double static_right_contact = right_torque_nm / radius;
    if (near_zero_locked_velocity && physics_.static_breakaway_force_n > 0.0 &&
        std::abs(static_left_contact) <= physics_.static_breakaway_force_n &&
        std::abs(static_right_contact) <= physics_.static_breakaway_force_n &&
        std::abs(static_left_contact) <= traction_limit_each + 1.0e-10 &&
        std::abs(static_right_contact) <= traction_limit_each + 1.0e-10) {
      static_contact = true;
      sticking = true;
      acceleration = {};
      requested_left_contact_n = static_left_contact;
      requested_right_contact_n = static_right_contact;
      actual_left_contact_n = requested_left_contact_n;
      actual_right_contact_n = requested_right_contact_n;
    } else {
      const auto free_motion = solve_locked_fixed_contact(0.0, 0.0, 0.0);
      const double free_x_ddot = free_motion.first ? free_motion.second[0] : 0.0;
      double rolling_resistance_n = 0.0;
      if (std::abs(state_.velocity) > 1.0e-4) {
        rolling_resistance_n = -std::copysign(2.0 * rolling_force_each, state_.velocity);
      } else if (std::abs(free_x_ddot) > 1.0e-12) {
        rolling_resistance_n = -std::copysign(2.0 * rolling_force_each, free_x_ddot);
      }
      const auto constrained = solve_locked_sticking(rolling_resistance_n);
      if (constrained.first) {
        acceleration = {constrained.second[0], 0.0, constrained.second[1],
                        constrained.second[2]};
        requested_left_contact_n = constrained.second[3];
        requested_right_contact_n = constrained.second[4];
        sticking = std::abs(requested_left_contact_n) <= traction_limit_each + 1.0e-10 &&
                   std::abs(requested_right_contact_n) <= traction_limit_each + 1.0e-10;
      }
      if (!sticking) {
        actual_left_contact_n = std::clamp(requested_left_contact_n, -traction_limit_each,
                                           traction_limit_each);
        actual_right_contact_n = std::clamp(requested_right_contact_n, -traction_limit_each,
                                           traction_limit_each);
        const auto slipping =
            solve_locked_fixed_contact(actual_left_contact_n, actual_right_contact_n,
                                       rolling_resistance_n);
        if (slipping.first) {
          acceleration = {slipping.second[0], 0.0, slipping.second[1], slipping.second[2]};
        } else {
          acceleration = {};
        }
      } else {
        actual_left_contact_n = requested_left_contact_n;
        actual_right_contact_n = requested_right_contact_n;
      }
    }
  } else if (near_zero_contact_velocity && physics_.static_breakaway_force_n > 0.0 &&
             std::abs(required_static_resistance_n) <=
                 std::max(0.0, physics_.static_breakaway_force_n) &&
             std::abs(required_static_total_force_n) <= total_traction_limit_n + 1.0e-10) {
    // Preserve the useful low-speed/static behavior of the prior reduction:
    // a finite static resistance can hold the chassis at x_ddot=0, while the
    // pitch coordinate remains free to respond to the net pitch moment.
    static_contact = true;
    sticking = true;
    acceleration[0] = 0.0;
    acceleration[1] = reduced_pitch_rhs / mass_matrix.d22;
    acceleration[2] = -acceleration[1];
    acceleration[3] = -acceleration[1];
    requested_left_contact_n = 0.5 * required_static_total_force_n;
    requested_right_contact_n = requested_left_contact_n;
    actual_left_contact_n = requested_left_contact_n;
    actual_right_contact_n = requested_right_contact_n;
  } else {
    const auto free_motion = solve_fixed_contact(0.0, 0.0, 0.0);
    const double free_x_ddot = free_motion.first ? free_motion.second[0] : 0.0;
    double rolling_resistance_n = 0.0;
    if (std::abs(state_.velocity) > 1.0e-4) {
      rolling_resistance_n = -std::copysign(2.0 * rolling_force_each, state_.velocity);
    } else if (std::abs(free_x_ddot) > 1.0e-12) {
      rolling_resistance_n = -std::copysign(2.0 * rolling_force_each, free_x_ddot);
    }

    const auto constrained = solve_sticking(rolling_resistance_n);
    if (constrained.first) {
      acceleration = {constrained.second[0], constrained.second[1], constrained.second[2],
                      constrained.second[3]};
      requested_left_contact_n = constrained.second[4];
      requested_right_contact_n = constrained.second[5];
      sticking = std::abs(requested_left_contact_n) <= traction_limit_each + 1.0e-10 &&
                 std::abs(requested_right_contact_n) <= traction_limit_each + 1.0e-10;
    }

    if (!sticking) {
      // Once the sticking multiplier exceeds the tire envelope, the contact
      // force is clipped and the wheel/rotor coordinates are allowed to run
      // independently.  This is the only path that creates longitudinal slip.
      const double requested_left = requested_left_contact_n;
      const double requested_right = requested_right_contact_n;
      actual_left_contact_n = std::clamp(requested_left, -traction_limit_each,
                                         traction_limit_each);
      actual_right_contact_n = std::clamp(requested_right, -traction_limit_each,
                                           traction_limit_each);
      const auto slipping = solve_fixed_contact(actual_left_contact_n, actual_right_contact_n,
                                                rolling_resistance_n);
      if (slipping.first) {
        acceleration = slipping.second;
      } else {
        acceleration = {};
      }
    } else {
      actual_left_contact_n = requested_left_contact_n;
      actual_right_contact_n = requested_right_contact_n;
    }
  }

  const double velocity_before = state_.velocity;
  const double position_before = state_.position;
  const double pitch_rate_before = state_.pitch_rate;
  const double left_relative_velocity_before = stepper_left_relative_velocity_mps_;
  const double right_relative_velocity_before = stepper_right_relative_velocity_mps_;
  const double slip_before =
      state_.velocity -
      (0.5 * (stepper_left_relative_velocity_mps_ +
               stepper_right_relative_velocity_mps_) +
       radius * state_.pitch_rate);

  state_.velocity += acceleration[0] * dt_s;
  state_.position += 0.5 * (velocity_before + state_.velocity) * dt_s;
  if (static_contact) {
    state_.velocity = 0.0;
    state_.position = position_before;
  }
  if (!cfg_.stepper_lock_pitch_for_test) {
    state_.pitch_rate += acceleration[1] * dt_s;
    state_.pitch += 0.5 * (pitch_rate_before + state_.pitch_rate) * dt_s;
  }
  if (!cfg_.stepper_lock_pitch_for_test && std::abs(state_.pitch) > kPi / 2.0) {
    state_.pitch = state_.pitch > 0 ? kPi / 2.0 : -kPi / 2.0;
    state_.pitch_rate = 0.0;
    state_.velocity = 0.0;
  }

  if (static_contact || sticking) {
    stepper_left_relative_velocity_mps_ = state_.velocity - radius * state_.pitch_rate;
    stepper_right_relative_velocity_mps_ = stepper_left_relative_velocity_mps_;
  } else {
    // The solved alpha accelerations are angular (rad/s^2), while the
    // independent state is deliberately stored as relative tangential
    // velocity (m/s) for the actuator/back-EMF interface.
    stepper_left_relative_velocity_mps_ += radius * acceleration[2] * dt_s;
    stepper_right_relative_velocity_mps_ += radius * acceleration[3] * dt_s;
  }
  const double slip_after =
      state_.velocity -
      (0.5 * (stepper_left_relative_velocity_mps_ +
               stepper_right_relative_velocity_mps_) +
       radius * state_.pitch_rate);
  if (dt_s > 0.0) {
    stepper_accumulated_slip_distance_m_ +=
        0.5 * (std::abs(slip_before) + std::abs(slip_after)) * dt_s;
  }
  stepper_contact_sticking_ = sticking && std::abs(slip_after) <= 1.0e-6;
  stepper_phase_electrical_actuator_.advance_relative_mechanical_state(
      dt_s, left_relative_velocity_before, right_relative_velocity_before,
      stepper_left_relative_velocity_mps_, stepper_right_relative_velocity_mps_);

  const auto update_cycle_counter = [&](double unwrapped_phase_error_rad,
                                        std::int64_t* last_cycle_index,
                                        std::uint64_t* accumulated_cycles) {
    const auto cycle_index = static_cast<std::int64_t>(std::floor(
        (unwrapped_phase_error_rad + kPi) / (2.0 * kPi)));
    if (stepper_have_cycle_indices_) {
      const auto difference = cycle_index - *last_cycle_index;
      if (difference != 0) {
        *accumulated_cycles += static_cast<std::uint64_t>(std::abs(difference));
      }
    }
    *last_cycle_index = cycle_index;
    return cycle_index;
  };
  const auto left_cycle_index = update_cycle_counter(
      left_snapshot.unwrapped_electrical_phase_error_rad, &stepper_last_cycle_index_left_,
      &stepper_accumulated_cycle_slips_left_);
  const auto right_cycle_index = update_cycle_counter(
      right_snapshot.unwrapped_electrical_phase_error_rad, &stepper_last_cycle_index_right_,
      &stepper_accumulated_cycle_slips_right_);
  stepper_have_cycle_indices_ = true;

  const double average_phase_error =
      0.5 * (left_snapshot.electrical_phase_error_rad +
             right_snapshot.electrical_phase_error_rad);
  const double wheel_surface_velocity =
      0.5 * (stepper_left_relative_velocity_mps_ +
             stepper_right_relative_velocity_mps_) +
       radius * state_.pitch_rate;
  // In the independent-wheel model this is the actual wheel/rim velocity.
  // It equals chassis velocity while sticking, but intentionally diverges
  // from it when the contact model is slipping.
  actual_wheel_velocity_ = wheel_surface_velocity;
  const double requested_contact_total =
      requested_left_contact_n + requested_right_contact_n;
  const double actual_contact_total = actual_left_contact_n + actual_right_contact_n;
  const double requested_traction_utilization =
      traction_limit_each > 1.0e-12
          ? std::max(std::abs(requested_left_contact_n),
                     std::abs(requested_right_contact_n)) /
                traction_limit_each
          : (std::max(std::abs(requested_left_contact_n),
                      std::abs(requested_right_contact_n)) > 1.0e-12
                 ? std::numeric_limits<double>::infinity()
                 : 0.0);

  diagnostics_.target_wheel_velocity =
      0.5 * (left_snapshot.commanded_field_velocity_mps +
             right_snapshot.commanded_field_velocity_mps);
  diagnostics_.actual_wheel_velocity = actual_wheel_velocity_;
  diagnostics_.velocity_error = diagnostics_.target_wheel_velocity - actual_wheel_velocity_;
  diagnostics_.f_cmd = total_torque_nm / radius;
  diagnostics_.f_app = actual_contact_total;
  diagnostics_.desired_drive_force = total_torque_nm / radius;
  diagnostics_.limited_drive_force = total_torque_nm / radius;
  diagnostics_.applied_drive_force = total_torque_nm / radius;
  diagnostics_.desired_tire_force = requested_contact_total;
  diagnostics_.external_force_n = external_force_n_;
  diagnostics_.external_com_bias_rad = external_com_bias_rad_;
  diagnostics_.x_ddot = acceleration[0];
  diagnostics_.theta_ddot = acceleration[1];
  diagnostics_.phase_error_steps = average_phase_error /
                                   Nominal::stepper_phase_electrical_radians_per_step;
  diagnostics_.missed_steps = diagnostics_.phase_error_steps;
  diagnostics_.traction_limit_n = total_traction_limit_n;
  diagnostics_.motor_force_limit_n = Nominal::stepper_combined_force_n;
  diagnostics_.command_saturated = false;
  diagnostics_.phase_saturated = false;
  diagnostics_.motor_force_saturated = false;
  diagnostics_.traction_saturated =
      std::abs(requested_left_contact_n) > traction_limit_each + 1.0e-10 ||
      std::abs(requested_right_contact_n) > traction_limit_each + 1.0e-10;
  diagnostics_.stepper_commanded_microsteps_left = left_snapshot.commanded_microsteps;
  diagnostics_.stepper_commanded_microsteps_right = right_snapshot.commanded_microsteps;
  diagnostics_.stepper_commanded_field_angle_left_rad = left_snapshot.commanded_field_angle_rad;
  diagnostics_.stepper_commanded_field_angle_right_rad = right_snapshot.commanded_field_angle_rad;
  diagnostics_.stepper_commanded_field_electrical_angle_left_rad =
      left_snapshot.commanded_field_electrical_angle_rad;
  diagnostics_.stepper_commanded_field_electrical_angle_right_rad =
      right_snapshot.commanded_field_electrical_angle_rad;
  diagnostics_.stepper_commanded_field_velocity_mps = diagnostics_.target_wheel_velocity;
  diagnostics_.stepper_actual_relative_angle_left_rad = left_snapshot.actual_relative_angle_rad;
  diagnostics_.stepper_actual_relative_angle_right_rad = right_snapshot.actual_relative_angle_rad;
  diagnostics_.stepper_actual_rotor_electrical_angle_left_rad =
      left_snapshot.actual_rotor_electrical_angle_rad;
  diagnostics_.stepper_actual_rotor_electrical_angle_right_rad =
      right_snapshot.actual_rotor_electrical_angle_rad;
  diagnostics_.stepper_electrical_phase_error_left_rad = left_snapshot.electrical_phase_error_rad;
  diagnostics_.stepper_electrical_phase_error_right_rad = right_snapshot.electrical_phase_error_rad;
  diagnostics_.stepper_torque_left_nm = left_phase_torque_nm;
  diagnostics_.stepper_torque_right_nm = right_phase_torque_nm;
  diagnostics_.stepper_summed_torque_nm = left_phase_torque_nm + right_phase_torque_nm;
  diagnostics_.stepper_damping_torque_left_nm = left_damping_torque_nm;
  diagnostics_.stepper_damping_torque_right_nm = right_damping_torque_nm;
  diagnostics_.stepper_applied_torque_left_nm = left_torque_nm;
  diagnostics_.stepper_applied_torque_right_nm = right_torque_nm;
  diagnostics_.stepper_motor_relative_velocity_left_rad_s = left_relative_rate;
  diagnostics_.stepper_motor_relative_velocity_right_rad_s = right_relative_rate;
  diagnostics_.stepper_actual_wheel_velocity_mps = actual_wheel_velocity_;
  diagnostics_.stepper_chassis_velocity_mps = state_.velocity;
  diagnostics_.stepper_wheel_surface_velocity_mps = wheel_surface_velocity;
  diagnostics_.stepper_wheel_angle_left_rad =
      left_snapshot.actual_relative_angle_rad + state_.pitch;
  diagnostics_.stepper_wheel_angle_right_rad =
      right_snapshot.actual_relative_angle_rad + state_.pitch;
  diagnostics_.stepper_wheel_angular_velocity_left_rad_s =
      stepper_left_relative_velocity_mps_ / radius + state_.pitch_rate;
  diagnostics_.stepper_wheel_angular_velocity_right_rad_s =
      stepper_right_relative_velocity_mps_ / radius + state_.pitch_rate;
  diagnostics_.stepper_chassis_ground_velocity_mps = state_.velocity;
  diagnostics_.stepper_slip_velocity_mps =
      state_.velocity - wheel_surface_velocity;
  diagnostics_.stepper_accumulated_slip_distance_m = stepper_accumulated_slip_distance_m_;
  diagnostics_.stepper_wheel_acceleration_mps2 =
      0.5 * radius * (acceleration[2] + acceleration[3]) + radius * acceleration[1];
  diagnostics_.stepper_requested_contact_force_n = requested_contact_total;
  diagnostics_.stepper_actual_contact_force_n = actual_contact_total;
  diagnostics_.stepper_traction_limit_n = total_traction_limit_n;
  diagnostics_.stepper_traction_utilization = requested_traction_utilization;
  diagnostics_.stepper_requested_contact_force_left_n = requested_left_contact_n;
  diagnostics_.stepper_requested_contact_force_right_n = requested_right_contact_n;
  diagnostics_.stepper_actual_contact_force_left_n = actual_left_contact_n;
  diagnostics_.stepper_actual_contact_force_right_n = actual_right_contact_n;
  diagnostics_.stepper_traction_limit_left_n = traction_limit_each;
  diagnostics_.stepper_traction_limit_right_n = traction_limit_each;
  diagnostics_.stepper_traction_saturated = diagnostics_.traction_saturated;
  diagnostics_.stepper_contact_sticking = sticking;
  diagnostics_.stepper_static_contact_active = static_contact;
  diagnostics_.stepper_unwrapped_electrical_phase_error_left_rad =
      left_snapshot.unwrapped_electrical_phase_error_rad;
  diagnostics_.stepper_unwrapped_electrical_phase_error_right_rad =
      right_snapshot.unwrapped_electrical_phase_error_rad;
  diagnostics_.stepper_field_rotor_relative_velocity_left_rad_s =
      left_snapshot.field_rotor_relative_velocity_rad_s;
  diagnostics_.stepper_field_rotor_relative_velocity_right_rad_s =
      right_snapshot.field_rotor_relative_velocity_rad_s;
  diagnostics_.stepper_electrical_cycle_index_left = static_cast<double>(left_cycle_index);
  diagnostics_.stepper_electrical_cycle_index_right = static_cast<double>(right_cycle_index);
  diagnostics_.stepper_accumulated_electrical_cycle_slips_left =
      static_cast<double>(stepper_accumulated_cycle_slips_left_);
  diagnostics_.stepper_accumulated_electrical_cycle_slips_right =
      static_cast<double>(stepper_accumulated_cycle_slips_right_);
  diagnostics_.stepper_electrical_cycle_slipped_left =
      stepper_accumulated_cycle_slips_left_ != 0;
  diagnostics_.stepper_electrical_cycle_slipped_right =
      stepper_accumulated_cycle_slips_right_ != 0;
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
  diagnostics_.brace_contact_active = brace_contact_active;
  diagnostics_.brace_penetration_rad = brace_penetration_rad;
  diagnostics_.brace_torque_nm = brace_torque_nm;
}

void BalancerSimulator::step_once(double dt_s) {
  using Nominal = HardwareNominal;
  const double avg_steps_per_sec = 0.5 * (left_target_sps_ + right_target_sps_);
  const double target_wheel_velocity = steps_per_sec_to_wheel_velocity(
      avg_steps_per_sec, Nominal::steps_per_rev, Nominal::wheel_radius);
  if (physics_.stepper_phase_electrical) {
    step_stepper_electrical_once(dt_s, emitted_left_steps_, emitted_right_steps_, 0.0, 0.0);
    return;
  }
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
    const double rhs1_without_contact =
        applied_force_n + external_force_n_ + H * Q_dot * Q_dot * sQ -
        physics_.cart_damping * state_.velocity;
    const double com_height_m = H / T;
    const double external_force_pitch_moment = external_force_n_ * com_height_m * cQ;
    // The motor torque enters the constrained equations once: +T/r in the
    // translation equation and the equal/opposite -T reaction on the body.
    const double rhs2 = physics_.gravity_mps2 * H * sQ -
                        physics_.pitch_damping * state_.pitch_rate -
                        total_torque_nm + external_force_pitch_moment;
    const double det = mass_matrix.determinant;
    const double rolling_resistance_force_n =
        std::max(0.0, physics_.rolling_resistance_force_n);
    const double static_breakaway_force_n =
        std::max(0.0, physics_.static_breakaway_force_n);
    constexpr double kStaticContactVelocityMps = 1.0e-4;
    bool static_contact = false;
    double contact_resistance_force_n = 0.0;
    if (static_breakaway_force_n > 0.0 &&
        std::abs(state_.velocity) <= kStaticContactVelocityMps) {
      // Solve the constrained x_ddot=0 equation for the contact force.  This
      // allows the same body/chassis mass matrix to determine how much of the
      // applied pitch torque the ground can hold, instead of treating static
      // friction as an unrelated acceleration clamp.
      const double required_static_contact =
          d12 * rhs2 / d22 - rhs1_without_contact;
      if (std::abs(required_static_contact) <= static_breakaway_force_n) {
        contact_resistance_force_n = required_static_contact;
        static_contact = true;
      }
    }
    if (!static_contact) {
      if (std::abs(state_.velocity) > kStaticContactVelocityMps) {
        contact_resistance_force_n =
            -std::copysign(rolling_resistance_force_n, state_.velocity);
      } else if (rolling_resistance_force_n > 0.0) {
        const double free_x_ddot =
            (d22 * rhs1_without_contact - d12 * rhs2) / det;
        if (std::abs(free_x_ddot) > 1.0e-12) {
          contact_resistance_force_n =
              -std::copysign(rolling_resistance_force_n, free_x_ddot);
        }
      }
    }
    const double rhs1 = rhs1_without_contact + contact_resistance_force_n;
    const double x_ddot = (d22 * rhs1 - d12 * rhs2) / det;
    const double theta_ddot = (d11 * rhs2 - d21 * rhs1) / det;
    const double velocity_before = state_.velocity;
    const double position_before = state_.position;
    const double pitch_rate_before = state_.pitch_rate;
    state_.velocity += x_ddot * dt_s;
    state_.position += 0.5 * (velocity_before + state_.velocity) * dt_s;
    if (static_contact) {
      state_.velocity = 0.0;
      state_.position = position_before;
    }
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
