#include "stepper_phase_actuator.h"

#include <algorithm>
#include <cmath>

namespace stepper_phase {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

double positive_mod(double value, double modulus) {
  const double result = std::fmod(value, modulus);
  return result < 0.0 ? result + modulus : result;
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
  output_ = {};
}

void Actuator::set_commanded_microstep_positions(double left_steps, double right_steps) {
  commanded_left_steps_ = left_steps;
  commanded_right_steps_ = right_steps;
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
                                     double body_pitch_rate_rad_s) const {
  MotorOutput output{};
  output.commanded_microstep_position = commanded_steps;
  const double step_count = static_cast<double>(parameters_.full_steps_per_revolution) *
                            static_cast<double>(parameters_.microsteps_per_full_step);
  const auto rounded_index = static_cast<std::int64_t>(std::llround(commanded_steps));
  const auto wrapped_index = static_cast<std::int64_t>(
      std::llround(positive_mod(static_cast<double>(rounded_index), step_count)));
  output.commanded_microstep_index = wrapped_index;
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
  output.electrical_phase_error_rad = wrap_pi(
      output.commanded_field_electrical_angle_rad - output.actual_rotor_electrical_angle_rad);
  output.phase = phase_for_electrical_angle(output.commanded_field_electrical_angle_rad);
  output.torque_nm =
      torque_for_phase_error(output.electrical_phase_error_rad,
                             parameters_.torque_constant_nm_per_a *
                                 std::max(0.0, parameters_.current_limit_a));
  return output;
}

Output Actuator::evaluate(double dt_s, double chassis_velocity_mps,
                          double body_pitch_rate_rad_s) {
  output_.left = evaluate_motor(commanded_left_steps_, previous_left_steps_, left_state_, dt_s,
                                chassis_velocity_mps, body_pitch_rate_rad_s);
  output_.right = evaluate_motor(commanded_right_steps_, previous_right_steps_, right_state_, dt_s,
                                 chassis_velocity_mps, body_pitch_rate_rad_s);
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
  const double wrapped = std::fmod(angle_rad + kPi, kTwoPi);
  const double normalized = wrapped < 0.0 ? wrapped + kTwoPi : wrapped;
  return normalized - kPi;
}

PhaseCommand ElectricalActuator::phase_for_electrical_angle(double angle_rad) {
  return PhaseCommand{std::cos(angle_rad), std::sin(angle_rad)};
}

double ElectricalActuator::torque_for_currents(double rotor_electrical_angle_rad,
                                                double current_a, double current_b,
                                                double torque_constant_nm_per_a) {
  return torque_constant_nm_per_a *
         (-current_a * std::sin(rotor_electrical_angle_rad) +
          current_b * std::cos(rotor_electrical_angle_rad));
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
                                          double dt_s, double* phase_voltage,
                                          bool* voltage_saturated) const {
  const double resistance = std::max(1e-12, parameters_.phase_resistance_ohm);
  const double inductance = std::max(1e-12, parameters_.phase_inductance_h);
  const double bus_voltage = std::max(0.0, parameters_.bus_voltage_v);
  if (dt_s <= 0.0 || bus_voltage <= 0.0) {
    if (phase_voltage != nullptr) *phase_voltage = 0.0;
    if (voltage_saturated != nullptr) *voltage_saturated = false;
    return current;
  }

  const double decay = std::exp(-resistance * dt_s / inductance);
  const auto update_for_voltage = [&](double voltage) {
    const double steady_current = (voltage - back_emf) / resistance;
    return steady_current + (current - steady_current) * decay;
  };
  const double next_plus = update_for_voltage(bus_voltage);
  const double next_minus = update_for_voltage(-bus_voltage);
  const double lower = std::min(next_minus, next_plus);
  const double upper = std::max(next_minus, next_plus);
  const double next = std::clamp(current_ref, lower, upper);

  double voltage = 0.0;
  bool saturated = false;
  const double denominator = 1.0 - decay;
  if (next <= lower + 1e-12) {
    voltage = next_minus <= next_plus ? -bus_voltage : bus_voltage;
    saturated = true;
  } else if (next >= upper - 1e-12) {
    voltage = next_plus >= next_minus ? bus_voltage : -bus_voltage;
    saturated = true;
  } else if (denominator > 1e-15) {
    const double steady_current = (next - current * decay) / denominator;
    voltage = std::clamp(resistance * steady_current + back_emf,
                         -bus_voltage, bus_voltage);
    saturated = std::abs(voltage) >= bus_voltage - 1e-12;
  }
  if (phase_voltage != nullptr) *phase_voltage = voltage;
  if (voltage_saturated != nullptr) *voltage_saturated = saturated;
  return next;
}

void ElectricalActuator::set_actual_relative_angles_for_test(double left_angle_rad,
                                                              double right_angle_rad) {
  left_state_.actual_relative_angle_rad = left_angle_rad;
  right_state_.actual_relative_angle_rad = right_angle_rad;
}

ElectricalMotorOutput ElectricalActuator::evaluate_motor(
    double commanded_steps, double previous_commanded_steps, MotorState& state, double dt_s,
    double chassis_velocity_mps, double body_pitch_rate_rad_s) const {
  ElectricalMotorOutput output{};
  output.commanded_microstep_position = commanded_steps;
  const double step_count = static_cast<double>(parameters_.full_steps_per_revolution) *
                            static_cast<double>(parameters_.microsteps_per_full_step);
  const auto rounded_index = static_cast<std::int64_t>(std::llround(commanded_steps));
  output.commanded_microstep_index = static_cast<std::int64_t>(
      std::llround(positive_mod(static_cast<double>(rounded_index), step_count)));
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
  output.electrical_phase_error_rad = wrap_pi(
      output.commanded_field_electrical_angle_rad - output.actual_rotor_electrical_angle_rad);
  output.phase = phase_for_electrical_angle(output.commanded_field_electrical_angle_rad);
  output.current_ref_a = parameters_.current_limit_a * output.phase.a;
  output.current_ref_b = parameters_.current_limit_a * output.phase.b;

  const double omega_motor = output.actual_relative_mechanical_velocity_mps /
                             parameters_.wheel_radius_m;
  output.back_emf_a = -parameters_.back_emf_constant_v_per_rad_s * omega_motor *
                      std::sin(output.actual_rotor_electrical_angle_rad);
  output.back_emf_b = parameters_.back_emf_constant_v_per_rad_s * omega_motor *
                      std::cos(output.actual_rotor_electrical_angle_rad);
  bool voltage_saturated_a = false;
  bool voltage_saturated_b = false;
  state.current_a = update_current(state.current_a, output.current_ref_a, output.back_emf_a,
                                    dt_s, &output.phase_voltage_a, &voltage_saturated_a);
  state.current_b = update_current(state.current_b, output.current_ref_b, output.back_emf_b,
                                    dt_s, &output.phase_voltage_b, &voltage_saturated_b);
  output.current_a = state.current_a;
  output.current_b = state.current_b;
  output.voltage_saturated = voltage_saturated_a || voltage_saturated_b;
  output.torque_nm = torque_for_currents(output.actual_rotor_electrical_angle_rad,
                                         output.current_a, output.current_b,
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
  output_.left = evaluate_motor(commanded_left_steps_, previous_left_steps_, left_state_, dt_s,
                                chassis_velocity_mps, body_pitch_rate_rad_s);
  output_.right = evaluate_motor(commanded_right_steps_, previous_right_steps_, right_state_, dt_s,
                                 chassis_velocity_mps, body_pitch_rate_rad_s);
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
