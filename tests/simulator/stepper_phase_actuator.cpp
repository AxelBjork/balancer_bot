#include "stepper_phase_actuator.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace stepper_phase {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr std::int64_t kNominalPhaseStates = 128;

// The nominal electrical command grid is 128 states per electrical cycle.
// Store one quadrant as literals so the hot path performs no command-angle
// trigonometry and does not depend on constexpr libm support in the cross
// compiler. The remaining quadrants are obtained by sign/axis symmetry.
constexpr std::array<PhaseCommand, 32> kNominalPhaseQuarter = {{
    PhaseCommand{1.0, 0.0},
    PhaseCommand{0.99879545620517241, 0.049067674327418015},
    PhaseCommand{0.99518472667219693, 0.098017140329560604},
    PhaseCommand{0.98917650996478101, 0.14673047445536175},
    PhaseCommand{0.98078528040323043, 0.19509032201612825},
    PhaseCommand{0.97003125319454397, 0.24298017990326387},
    PhaseCommand{0.95694033573220882, 0.29028467725446233},
    PhaseCommand{0.94154406518302081, 0.33688985339222005},
    PhaseCommand{0.92387953251128674, 0.38268343236508978},
    PhaseCommand{0.90398929312344334, 0.42755509343028208},
    PhaseCommand{0.88192126434835505, 0.47139673682599764},
    PhaseCommand{0.85772861000027212, 0.51410274419322166},
    PhaseCommand{0.83146961230254524, 0.55557023301960218},
    PhaseCommand{0.80320753148064494, 0.59569930449243336},
    PhaseCommand{0.77301045336273699, 0.63439328416364549},
    PhaseCommand{0.74095112535495911, 0.67155895484701833},
    PhaseCommand{0.70710678118654757, 0.70710678118654746},
    PhaseCommand{0.67155895484701833, 0.74095112535495911},
    PhaseCommand{0.63439328416364549, 0.77301045336273699},
    PhaseCommand{0.59569930449243347, 0.80320753148064483},
    PhaseCommand{0.55557023301960229, 0.83146961230254524},
    PhaseCommand{0.51410274419322166, 0.85772861000027212},
    PhaseCommand{0.47139673682599781, 0.88192126434835494},
    PhaseCommand{0.42755509312344334, 0.90398929312344334},
    PhaseCommand{0.38268343236508984, 0.92387953251128674},
    PhaseCommand{0.33688985339222005, 0.94154406518302081},
    PhaseCommand{0.29028467725446233, 0.95694033573220894},
    PhaseCommand{0.24298017990326398, 0.97003125319454397},
    PhaseCommand{0.19509032201612833, 0.98078528040323043},
    PhaseCommand{0.14673047445536175, 0.98917650996478101},
    PhaseCommand{0.09801714032956077, 0.99518472667219682},
    PhaseCommand{0.049067674327418126, 0.99879545620517241},
}};

double positive_mod(double value, double modulus) {
  const double result = std::fmod(value, modulus);
  return result < 0.0 ? result + modulus : result;
}

std::int64_t positive_mod(std::int64_t value, std::int64_t modulus) {
  const std::int64_t result = value % modulus;
  return result < 0 ? result + modulus : result;
}

double wrap_pi_fmod(double angle_rad) {
  const double wrapped = std::fmod(angle_rad + kPi, kTwoPi);
  const double normalized = wrapped < 0.0 ? wrapped + kTwoPi : wrapped;
  return normalized - kPi;
}

double fast_wrap_pi(double angle_rad) {
  // Most simulated phase errors are already within one turn. Keep the exact
  // old boundary behavior while avoiding fmod for that common case.
  if (angle_rad >= -kPi && angle_rad < kPi) return angle_rad;
  if (angle_rad >= kPi && angle_rad < 3.0 * kPi) return angle_rad - kTwoPi;
  if (angle_rad >= -3.0 * kPi && angle_rad < -kPi) return angle_rad + kTwoPi;
  return wrap_pi_fmod(angle_rad);
}

bool uses_nominal_phase_grid(int full_steps_per_revolution,
                             int microsteps_per_full_step) {
  return full_steps_per_revolution == Config::motor_full_steps_per_rev &&
         microsteps_per_full_step == Config::microsteps_per_full_step;
}

PhaseCommand nominal_phase_for_index(std::int64_t index) {
  const bool negative = index < 0;
  const std::uint64_t magnitude =
      negative ? static_cast<std::uint64_t>(-(index + 1)) + 1U
               : static_cast<std::uint64_t>(index);
  const std::int64_t wrapped =
      static_cast<std::int64_t>(magnitude % static_cast<std::uint64_t>(kNominalPhaseStates));
  const std::size_t quadrant = static_cast<std::size_t>(wrapped / 32);
  const std::size_t offset = static_cast<std::size_t>(wrapped % 32);
  const PhaseCommand phase = kNominalPhaseQuarter[offset];
  PhaseCommand positive_phase{};
  switch (quadrant) {
    case 0:
      positive_phase = phase;
      break;
    case 1:
      positive_phase = PhaseCommand{-phase.b, phase.a};
      break;
    case 2:
      positive_phase = PhaseCommand{-phase.a, -phase.b};
      break;
    default:
      positive_phase = PhaseCommand{phase.b, -phase.a};
      break;
  }
  return negative ? PhaseCommand{positive_phase.a, -positive_phase.b} : positive_phase;
}

PhaseCommand phase_for_command(
    int full_steps_per_revolution, int microsteps_per_full_step,
    std::int64_t commanded_index, double commanded_electrical_angle_rad) {
  if (uses_nominal_phase_grid(full_steps_per_revolution, microsteps_per_full_step)) {
    return nominal_phase_for_index(commanded_index);
  }
  return PhaseCommand{std::cos(commanded_electrical_angle_rad),
                      std::sin(commanded_electrical_angle_rad)};
}

double torque_for_rotor_components(double rotor_sin, double rotor_cos, double current_a,
                                   double current_b, double torque_constant_nm_per_a) {
  return torque_constant_nm_per_a * (-current_a * rotor_sin + current_b * rotor_cos);
}

}  // namespace

Actuator::Actuator(const Parameters& parameters) : parameters_(parameters) {
  reset();
}

void Actuator::set_parameters(const Parameters& parameters, bool reset_state) {
  parameters_ = parameters;
  if (reset_state) reset();
}

void Actuator::reset(double left_actual_relative_angle_rad,
                     double right_actual_relative_angle_rad) {
  left_state_.actual_relative_angle_rad = left_actual_relative_angle_rad;
  right_state_.actual_relative_angle_rad = right_actual_relative_angle_rad;
  commanded_left_steps_ = 0.0;
  commanded_right_steps_ = 0.0;
  previous_left_steps_ = 0.0;
  previous_right_steps_ = 0.0;
  commanded_left_step_index_ = 0;
  commanded_right_step_index_ = 0;
  have_commanded_indices_ = false;
  output_ = {};
}

void Actuator::set_commanded_microstep_positions(double left_steps, double right_steps) {
  commanded_left_steps_ = left_steps;
  commanded_right_steps_ = right_steps;
  have_commanded_indices_ = false;
}

void Actuator::set_commanded_microstep_indices(std::int64_t left_steps,
                                                std::int64_t right_steps) {
  commanded_left_step_index_ = left_steps;
  commanded_right_step_index_ = right_steps;
  commanded_left_steps_ = static_cast<double>(left_steps);
  commanded_right_steps_ = static_cast<double>(right_steps);
  have_commanded_indices_ = true;
}

double Actuator::mechanical_radians_per_step() const {
  return kTwoPi /
         (static_cast<double>(parameters_.full_steps_per_revolution) *
          static_cast<double>(parameters_.microsteps_per_full_step));
}

double Actuator::meters_per_step() const {
  return parameters_.wheel_radius_m * mechanical_radians_per_step();
}

double Actuator::electrical_cycles_per_mechanical_revolution() const {
  // A 1.8-degree two-phase stepper takes four full steps per electrical
  // cycle: 200 / 4 = 50 cycles per mechanical revolution.
  return static_cast<double>(parameters_.full_steps_per_revolution) / 4.0;
}

double Actuator::electrical_radians_per_step() const {
  return mechanical_radians_per_step() * electrical_cycles_per_mechanical_revolution();
}

double Actuator::wrap_pi(double angle_rad) {
  const double wrapped = std::fmod(angle_rad + kPi, kTwoPi);
  const double normalized = wrapped < 0.0 ? wrapped + kTwoPi : wrapped;
  return normalized - kPi;
}

PhaseCommand Actuator::phase_for_electrical_angle(double angle_rad) {
  // This normalized sine/cosine sequence is the first-stage equivalent of
  // the indexed DRV8825 microstep current table.  Exact driver-table
  // quantization and harmonics are intentionally deferred.
  return PhaseCommand{std::cos(angle_rad), std::sin(angle_rad)};
}

double Actuator::torque_for_phase_error(double phase_error_rad, double holding_torque_nm) {
  return holding_torque_nm * std::sin(phase_error_rad);
}

void Actuator::set_actual_relative_angles_for_test(double left_angle_rad,
                                                    double right_angle_rad) {
  left_state_.actual_relative_angle_rad = left_angle_rad;
  right_state_.actual_relative_angle_rad = right_angle_rad;
}

MotorOutput Actuator::evaluate_motor(double commanded_steps, double previous_commanded_steps,
                                     MotorState& state, double dt_s,
                                     double chassis_velocity_mps,
                                     double body_pitch_rate_rad_s, bool have_commanded_index,
                                     std::int64_t commanded_index) const {
  MotorOutput output{};
  output.commanded_microstep_position = commanded_steps;
  if (have_commanded_index) {
    const auto step_count = static_cast<std::int64_t>(parameters_.full_steps_per_revolution) *
                            static_cast<std::int64_t>(parameters_.microsteps_per_full_step);
    output.commanded_microstep_index =
        step_count > 0 ? positive_mod(commanded_index, step_count) : 0;
  } else {
    const double step_count = static_cast<double>(parameters_.full_steps_per_revolution) *
                              static_cast<double>(parameters_.microsteps_per_full_step);
    const auto rounded_index = static_cast<std::int64_t>(std::llround(commanded_steps));
    output.commanded_microstep_index = static_cast<std::int64_t>(
        std::llround(positive_mod(static_cast<double>(rounded_index), step_count)));
  }
  output.commanded_mechanical_angle_rad = commanded_steps * mechanical_radians_per_step();
  output.commanded_field_electrical_angle_rad =
      output.commanded_mechanical_angle_rad * electrical_cycles_per_mechanical_revolution();
  output.commanded_field_velocity_mps =
      dt_s > 0.0 ? (commanded_steps - previous_commanded_steps) * meters_per_step() / dt_s : 0.0;
  output.actual_relative_mechanical_angle_rad = state.actual_relative_angle_rad;
  output.actual_relative_mechanical_velocity_mps =
      chassis_velocity_mps - parameters_.wheel_radius_m * body_pitch_rate_rad_s;
  output.actual_rotor_electrical_angle_rad =
      output.actual_relative_mechanical_angle_rad * electrical_cycles_per_mechanical_revolution();
  const double raw_phase_error = output.commanded_field_electrical_angle_rad -
                                 output.actual_rotor_electrical_angle_rad;
  output.electrical_phase_error_rad = fast_wrap_pi(raw_phase_error);
  output.phase = have_commanded_index
                     ? phase_for_command(parameters_.full_steps_per_revolution,
                                         parameters_.microsteps_per_full_step, commanded_index,
                                         output.commanded_field_electrical_angle_rad)
                     : phase_for_electrical_angle(output.commanded_field_electrical_angle_rad);
  output.torque_nm =
      torque_for_phase_error(output.electrical_phase_error_rad,
                             parameters_.torque_constant_nm_per_a *
                                 std::max(0.0, parameters_.current_limit_a));
  return output;
}

Output Actuator::evaluate(double dt_s, double chassis_velocity_mps,
                          double body_pitch_rate_rad_s) {
  output_.left = evaluate_motor(commanded_left_steps_, previous_left_steps_, left_state_, dt_s,
                                chassis_velocity_mps, body_pitch_rate_rad_s,
                                have_commanded_indices_, commanded_left_step_index_);
  output_.right = evaluate_motor(commanded_right_steps_, previous_right_steps_, right_state_, dt_s,
                                 chassis_velocity_mps, body_pitch_rate_rad_s,
                                 have_commanded_indices_, commanded_right_step_index_);
  output_.summed_torque_nm = output_.left.torque_nm + output_.right.torque_nm;
  previous_left_steps_ = commanded_left_steps_;
  previous_right_steps_ = commanded_right_steps_;
  return output_;
}

void Actuator::advance_mechanical_state(double dt_s, double chassis_velocity_before_mps,
                                        double body_pitch_rate_before_rad_s,
                                        double chassis_velocity_after_mps,
                                        double body_pitch_rate_after_rad_s) {
  if (dt_s <= 0.0) return;
  const double before_left =
      chassis_velocity_before_mps - parameters_.wheel_radius_m * body_pitch_rate_before_rad_s;
  const double after_left =
      chassis_velocity_after_mps - parameters_.wheel_radius_m * body_pitch_rate_after_rad_s;
  const double delta_angle = 0.5 * (before_left + after_left) / parameters_.wheel_radius_m * dt_s;
  left_state_.actual_relative_angle_rad += delta_angle;
  right_state_.actual_relative_angle_rad += delta_angle;
}

ElectricalActuator::ElectricalActuator(const ElectricalParameters& parameters)
    : parameters_(parameters) {
  reset();
}

void ElectricalActuator::set_parameters(const ElectricalParameters& parameters,
                                         bool reset_state) {
  parameters_ = parameters;
  if (reset_state) reset();
}

void ElectricalActuator::reset(double left_actual_relative_angle_rad,
                               double right_actual_relative_angle_rad) {
  left_state_.actual_relative_angle_rad = left_actual_relative_angle_rad;
  right_state_.actual_relative_angle_rad = right_actual_relative_angle_rad;
  commanded_left_steps_ = 0.0;
  commanded_right_steps_ = 0.0;
  previous_left_steps_ = 0.0;
  previous_right_steps_ = 0.0;
  commanded_left_step_index_ = 0;
  commanded_right_step_index_ = 0;
  have_commanded_indices_ = false;
  const double current_limit = std::max(0.0, parameters_.current_limit_a);
  left_state_.current_a = current_limit;
  left_state_.current_b = 0.0;
  right_state_.current_a = current_limit;
  right_state_.current_b = 0.0;
  output_ = {};
}

void ElectricalActuator::set_commanded_microstep_positions(double left_steps,
                                                            double right_steps) {
  commanded_left_steps_ = left_steps;
  commanded_right_steps_ = right_steps;
  have_commanded_indices_ = false;
}

void ElectricalActuator::set_commanded_microstep_indices(std::int64_t left_steps,
                                                          std::int64_t right_steps) {
  commanded_left_step_index_ = left_steps;
  commanded_right_step_index_ = right_steps;
  commanded_left_steps_ = static_cast<double>(left_steps);
  commanded_right_steps_ = static_cast<double>(right_steps);
  have_commanded_indices_ = true;
}

double ElectricalActuator::mechanical_radians_per_step() const {
  return kTwoPi /
         (static_cast<double>(parameters_.full_steps_per_revolution) *
          static_cast<double>(parameters_.microsteps_per_full_step));
}

double ElectricalActuator::meters_per_step() const {
  return parameters_.wheel_radius_m * mechanical_radians_per_step();
}

double ElectricalActuator::electrical_cycles_per_mechanical_revolution() const {
  return static_cast<double>(parameters_.full_steps_per_revolution) / 4.0;
}

double ElectricalActuator::electrical_radians_per_step() const {
  return mechanical_radians_per_step() * electrical_cycles_per_mechanical_revolution();
}

double ElectricalActuator::wrap_pi(double angle_rad) {
  return wrap_pi_fmod(angle_rad);
}

PhaseCommand ElectricalActuator::phase_for_electrical_angle(double angle_rad) {
  return PhaseCommand{std::cos(angle_rad), std::sin(angle_rad)};
}

double ElectricalActuator::torque_for_currents(double rotor_electrical_angle_rad,
                                                double current_a, double current_b,
                                                double torque_constant_nm_per_a) {
  return torque_for_rotor_components(std::sin(rotor_electrical_angle_rad),
                                     std::cos(rotor_electrical_angle_rad), current_a, current_b,
                                     torque_constant_nm_per_a);
}

double ElectricalActuator::back_emf_power(double omega_motor_rad_s,
                                          double rotor_electrical_angle_rad,
                                          double current_a, double current_b,
                                          double back_emf_constant_v_per_rad_s) {
  return back_emf_constant_v_per_rad_s * omega_motor_rad_s *
         (-current_a * std::sin(rotor_electrical_angle_rad) +
          current_b * std::cos(rotor_electrical_angle_rad));
}

double ElectricalActuator::update_current(double current, double current_ref, double back_emf,
                                          const CurrentUpdateCoefficients& coefficients,
                                          double* phase_voltage,
                                          bool* voltage_saturated) const {
  if (!coefficients.active) {
    if (phase_voltage != nullptr) *phase_voltage = 0.0;
    if (voltage_saturated != nullptr) *voltage_saturated = false;
    return current;
  }

  const auto update_for_voltage = [&](double voltage) {
    const double steady_current = (voltage - back_emf) * coefficients.inverse_resistance;
    return steady_current + (current - steady_current) * coefficients.decay;
  };
  const double next_plus = update_for_voltage(coefficients.bus_voltage);
  const double next_minus = update_for_voltage(-coefficients.bus_voltage);
  const double lower = std::min(next_minus, next_plus);
  const double upper = std::max(next_minus, next_plus);
  const double next = std::clamp(current_ref, lower, upper);

  double voltage = 0.0;
  bool saturated = false;
  const double denominator = coefficients.one_minus_decay;
  if (next <= lower + 1e-12) {
    voltage = next_minus <= next_plus ? -coefficients.bus_voltage : coefficients.bus_voltage;
    saturated = true;
  } else if (next >= upper - 1e-12) {
    voltage = next_plus >= next_minus ? coefficients.bus_voltage : -coefficients.bus_voltage;
    saturated = true;
  } else if (denominator > 1e-15) {
    const double steady_current =
        (next - current * coefficients.decay) / denominator;
    voltage = std::clamp(coefficients.resistance * steady_current + back_emf,
                         -coefficients.bus_voltage, coefficients.bus_voltage);
    saturated = std::abs(voltage) >= coefficients.bus_voltage - 1e-12;
  }
  if (phase_voltage != nullptr) *phase_voltage = voltage;
  if (voltage_saturated != nullptr) *voltage_saturated = saturated;
  return next;
}

ElectricalActuator::CurrentUpdateCoefficients
ElectricalActuator::make_current_update_coefficients(double dt_s) const {
  CurrentUpdateCoefficients coefficients;
  coefficients.resistance = std::max(1e-12, parameters_.phase_resistance_ohm);
  coefficients.inverse_resistance = 1.0 / coefficients.resistance;
  coefficients.bus_voltage = std::max(0.0, parameters_.bus_voltage_v);
  if (dt_s <= 0.0 || coefficients.bus_voltage <= 0.0) {
    return coefficients;
  }

  const double inductance = std::max(1e-12, parameters_.phase_inductance_h);
  coefficients.decay =
      std::exp(-coefficients.resistance * dt_s / inductance);
  coefficients.one_minus_decay = 1.0 - coefficients.decay;
  coefficients.active = true;
  return coefficients;
}

void ElectricalActuator::set_actual_relative_angles_for_test(double left_angle_rad,
                                                              double right_angle_rad) {
  left_state_.actual_relative_angle_rad = left_angle_rad;
  right_state_.actual_relative_angle_rad = right_angle_rad;
}

ElectricalMotorOutput ElectricalActuator::evaluate_motor(
    double commanded_steps, double previous_commanded_steps, MotorState& state, double dt_s,
    double chassis_velocity_mps, double body_pitch_rate_rad_s, bool have_commanded_index,
    std::int64_t commanded_index,
    const CurrentUpdateCoefficients& coefficients) const {
  ElectricalMotorOutput output{};
  output.commanded_microstep_position = commanded_steps;
  if (have_commanded_index) {
    const auto step_count = static_cast<std::int64_t>(parameters_.full_steps_per_revolution) *
                            static_cast<std::int64_t>(parameters_.microsteps_per_full_step);
    output.commanded_microstep_index =
        step_count > 0 ? positive_mod(commanded_index, step_count) : 0;
  } else {
    const double step_count = static_cast<double>(parameters_.full_steps_per_revolution) *
                              static_cast<double>(parameters_.microsteps_per_full_step);
    const auto rounded_index = static_cast<std::int64_t>(std::llround(commanded_steps));
    output.commanded_microstep_index = static_cast<std::int64_t>(
        std::llround(positive_mod(static_cast<double>(rounded_index), step_count)));
  }
  output.commanded_mechanical_angle_rad = commanded_steps * mechanical_radians_per_step();
  output.commanded_field_electrical_angle_rad =
      output.commanded_mechanical_angle_rad * electrical_cycles_per_mechanical_revolution();
  output.commanded_field_velocity_mps =
      dt_s > 0.0 ? (commanded_steps - previous_commanded_steps) * meters_per_step() / dt_s : 0.0;
  output.actual_relative_mechanical_angle_rad = state.actual_relative_angle_rad;
  output.actual_relative_mechanical_velocity_mps =
      chassis_velocity_mps - parameters_.wheel_radius_m * body_pitch_rate_rad_s;
  output.actual_rotor_electrical_angle_rad =
      output.actual_relative_mechanical_angle_rad * electrical_cycles_per_mechanical_revolution();
  const double raw_phase_error = output.commanded_field_electrical_angle_rad -
                                 output.actual_rotor_electrical_angle_rad;
  output.electrical_phase_error_rad = fast_wrap_pi(raw_phase_error);
  output.phase = have_commanded_index
                     ? phase_for_command(parameters_.full_steps_per_revolution,
                                         parameters_.microsteps_per_full_step, commanded_index,
                                         output.commanded_field_electrical_angle_rad)
                     : phase_for_electrical_angle(output.commanded_field_electrical_angle_rad);
  output.current_ref_a = parameters_.current_limit_a * output.phase.a;
  output.current_ref_b = parameters_.current_limit_a * output.phase.b;

  const double omega_motor = output.actual_relative_mechanical_velocity_mps /
                             parameters_.wheel_radius_m;
  const double rotor_sin = std::sin(output.actual_rotor_electrical_angle_rad);
  const double rotor_cos = std::cos(output.actual_rotor_electrical_angle_rad);
  output.back_emf_a = -parameters_.back_emf_constant_v_per_rad_s * omega_motor *
                      rotor_sin;
  output.back_emf_b = parameters_.back_emf_constant_v_per_rad_s * omega_motor *
                      rotor_cos;
  bool voltage_saturated_a = false;
  bool voltage_saturated_b = false;
  state.current_a = update_current(state.current_a, output.current_ref_a, output.back_emf_a,
                                    coefficients, &output.phase_voltage_a,
                                    &voltage_saturated_a);
  state.current_b = update_current(state.current_b, output.current_ref_b, output.back_emf_b,
                                    coefficients, &output.phase_voltage_b,
                                    &voltage_saturated_b);
  output.current_a = state.current_a;
  output.current_b = state.current_b;
  output.voltage_saturated = voltage_saturated_a || voltage_saturated_b;
  output.torque_nm = torque_for_rotor_components(
      rotor_sin, rotor_cos, output.current_a, output.current_b,
      parameters_.torque_constant_nm_per_a);
  output.electrical_power_w = output.phase_voltage_a * output.current_a +
                              output.phase_voltage_b * output.current_b;
  output.mechanical_power_w = output.torque_nm * omega_motor;
  output.resistive_loss_w = parameters_.phase_resistance_ohm *
                            (output.current_a * output.current_a +
                             output.current_b * output.current_b);
  output.magnetic_energy_j = 0.5 * parameters_.phase_inductance_h *
                             (output.current_a * output.current_a +
                              output.current_b * output.current_b);
  return output;
}

ElectricalOutput ElectricalActuator::evaluate(double dt_s, double chassis_velocity_mps,
                                              double body_pitch_rate_rad_s) {
  const CurrentUpdateCoefficients coefficients = make_current_update_coefficients(dt_s);
  output_.left = evaluate_motor(commanded_left_steps_, previous_left_steps_, left_state_, dt_s,
                                chassis_velocity_mps, body_pitch_rate_rad_s,
                                have_commanded_indices_, commanded_left_step_index_,
                                coefficients);
  output_.right = evaluate_motor(commanded_right_steps_, previous_right_steps_, right_state_, dt_s,
                                 chassis_velocity_mps, body_pitch_rate_rad_s,
                                 have_commanded_indices_, commanded_right_step_index_,
                                 coefficients);
  output_.summed_torque_nm = output_.left.torque_nm + output_.right.torque_nm;
  previous_left_steps_ = commanded_left_steps_;
  previous_right_steps_ = commanded_right_steps_;
  return output_;
}

void ElectricalActuator::advance_mechanical_state(double dt_s,
                                                  double chassis_velocity_before_mps,
                                                  double body_pitch_rate_before_rad_s,
                                                  double chassis_velocity_after_mps,
                                                  double body_pitch_rate_after_rad_s) {
  if (dt_s <= 0.0) return;
  const double before = chassis_velocity_before_mps -
                        parameters_.wheel_radius_m * body_pitch_rate_before_rad_s;
  const double after = chassis_velocity_after_mps -
                       parameters_.wheel_radius_m * body_pitch_rate_after_rad_s;
  const double delta_angle = 0.5 * (before + after) / parameters_.wheel_radius_m * dt_s;
  left_state_.actual_relative_angle_rad += delta_angle;
  right_state_.actual_relative_angle_rad += delta_angle;
}

}  // namespace stepper_phase
