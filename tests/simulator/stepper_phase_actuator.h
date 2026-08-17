#pragma once

#include <cstdint>

#include "services/main/config.h"

namespace stepper_phase {

// Ideal-current diagnostic actuator for a two-phase stepper.  The current
// amplitude is the norm of the two winding-current vector, not the current in
// one winding.  This is the same convention used by ElectricalParameters.
// The maintained electrical variant below adds bounded R/L and back-EMF
// current evolution; this class isolates the phase/mechanical topology.
struct Parameters {
  double wheel_radius_m = Config::wheel_radius_m;
  int full_steps_per_revolution = Config::motor_full_steps_per_rev;
  int microsteps_per_full_step = Config::microsteps_per_full_step;
  // Nominal DRV8825 full-scale vector-current setting.  At a given field
  // angle the two phase references have this Euclidean norm.
  double current_limit_a = 1.065;
  // The motor's 0.45 N m / 1.5 A-per-winding holding datum maps to the
  // vector convention as 0.45 / (sqrt(2) * 1.5) N m/A.
  double torque_constant_nm_per_a = 0.21213203435596426;
};

struct PhaseCommand {
  double a = 0.0;
  double b = 0.0;
};

struct MotorOutput {
  double commanded_microstep_position = 0.0;
  std::int64_t commanded_microstep_index = 0;
  double commanded_mechanical_angle_rad = 0.0;
  double commanded_field_electrical_angle_rad = 0.0;
  double commanded_field_velocity_mps = 0.0;
  double actual_relative_mechanical_angle_rad = 0.0;
  double actual_relative_mechanical_velocity_mps = 0.0;
  double actual_rotor_electrical_angle_rad = 0.0;
  double electrical_phase_error_rad = 0.0;
  PhaseCommand phase{};
  double torque_nm = 0.0;
};

struct Output {
  MotorOutput left{};
  MotorOutput right{};
  double summed_torque_nm = 0.0;
};

// First electrical/driver stage. This keeps the verified STEP/magnetic-field
// topology but replaces instantaneous current tracking with a best-case
// averaged bridge bounded by R/L, back-EMF, and bus voltage.
struct ElectricalParameters {
  double wheel_radius_m = Config::wheel_radius_m;
  int full_steps_per_revolution = Config::motor_full_steps_per_rev;
  int microsteps_per_full_step = Config::microsteps_per_full_step;
  double phase_resistance_ohm = 2.3;
  double phase_inductance_h = 0.0044;
  // Nominal measured Waveshare/DRV8825 setting. The motor's 1.5 A rating is
  // not the configured driver limit.
  double current_limit_a = 1.065;
  double bus_voltage_v = 11.1;
  // These use the same vector-current convention as Parameters and
  // mechanical-relative-speed SI units.  Energy consistency requires the
  // numerical Kt and Ke values to agree in this representation.
  double torque_constant_nm_per_a = 0.21213203435596426;
  double back_emf_constant_v_per_rad_s = 0.21213203435596426;
};

struct ElectricalMotorOutput {
  double commanded_microstep_position = 0.0;
  std::int64_t commanded_microstep_index = 0;
  double commanded_mechanical_angle_rad = 0.0;
  double commanded_field_electrical_angle_rad = 0.0;
  double commanded_field_velocity_mps = 0.0;
  double actual_relative_mechanical_angle_rad = 0.0;
  double actual_relative_mechanical_velocity_mps = 0.0;
  double actual_rotor_electrical_angle_rad = 0.0;
  double electrical_phase_error_rad = 0.0;
  PhaseCommand phase{};
  double current_ref_a = 0.0;
  double current_ref_b = 0.0;
  double current_a = 0.0;
  double current_b = 0.0;
  double phase_voltage_a = 0.0;
  double phase_voltage_b = 0.0;
  double back_emf_a = 0.0;
  double back_emf_b = 0.0;
  double torque_nm = 0.0;
  double electrical_power_w = 0.0;
  double mechanical_power_w = 0.0;
  double resistive_loss_w = 0.0;
  double magnetic_energy_j = 0.0;
  bool voltage_saturated = false;
};

struct ElectricalOutput {
  ElectricalMotorOutput left{};
  ElectricalMotorOutput right{};
  double summed_torque_nm = 0.0;
};

class Actuator {
 public:
  explicit Actuator(const Parameters& parameters = Parameters());

  void set_parameters(const Parameters& parameters, bool reset_state = true);

  void reset(double left_actual_relative_angle_rad = 0.0,
             double right_actual_relative_angle_rad = 0.0);

  // These positions are the cumulative STEP-edge positions emitted by the
  // motor runner.  They are not a force command and are not integrated a
  // second time inside this actuator.
  void set_commanded_microstep_positions(double left_steps, double right_steps);
  // Fast path for the simulator's timestamped STEP stream. The positions are
  // already integral, so no floating-point rounding is needed in the actuator.
  void set_commanded_microstep_indices(std::int64_t left_steps,
                                       std::int64_t right_steps);

  Output evaluate(double dt_s, double chassis_velocity_mps, double body_pitch_rate_rad_s);

  // The no-slip plant supplies the mechanical wheel/body relative velocity.
  // There is deliberately no independent rotor-inertia state in this model;
  // the attached wheel/rotor follows the constrained mechanical coordinate,
  // while its phase angle remains distinct from its absolute wheel angle.
  void advance_mechanical_state(double dt_s, double chassis_velocity_before_mps,
                                double body_pitch_rate_before_rad_s,
                                double chassis_velocity_after_mps,
                                double body_pitch_rate_after_rad_s);

  const Parameters& parameters() const {
    return parameters_;
  }

  const Output& output() const {
    return output_;
  }

  // Pure helpers are exposed so actuator invariants can be tested without
  // coupling those tests to the rigid-body integrator.
  double mechanical_radians_per_step() const;
  double meters_per_step() const;
  double electrical_cycles_per_mechanical_revolution() const;
  double electrical_radians_per_step() const;
  static double wrap_pi(double angle_rad);
  static PhaseCommand phase_for_electrical_angle(double angle_rad);
  static double torque_for_phase_error(double phase_error_rad, double holding_torque_nm);

  // Test/diagnostic hook for a prescribed mechanical back-drive.  Production
  // simulation updates this state only through advance_mechanical_state().
  void set_actual_relative_angles_for_test(double left_angle_rad, double right_angle_rad);

 private:
  struct MotorState {
    double actual_relative_angle_rad = 0.0;
  };

  MotorOutput evaluate_motor(double commanded_steps, double previous_commanded_steps,
                             MotorState& state, double dt_s, double chassis_velocity_mps,
                             double body_pitch_rate_rad_s, bool have_commanded_index,
                             std::int64_t commanded_index) const;

  Parameters parameters_{};
  MotorState left_state_{};
  MotorState right_state_{};
  double commanded_left_steps_ = 0.0;
  double commanded_right_steps_ = 0.0;
  double previous_left_steps_ = 0.0;
  double previous_right_steps_ = 0.0;
  std::int64_t commanded_left_step_index_ = 0;
  std::int64_t commanded_right_step_index_ = 0;
  bool have_commanded_indices_ = false;
  Output output_{};
};

class ElectricalActuator {
 public:
  explicit ElectricalActuator(const ElectricalParameters& parameters = ElectricalParameters());

  void set_parameters(const ElectricalParameters& parameters, bool reset_state = true);

  void reset(double left_actual_relative_angle_rad = 0.0,
             double right_actual_relative_angle_rad = 0.0);
  void set_commanded_microstep_positions(double left_steps, double right_steps);
  // Fast path for the simulator's timestamped STEP stream. The positions are
  // already integral, so no floating-point rounding is needed in the actuator.
  void set_commanded_microstep_indices(std::int64_t left_steps,
                                       std::int64_t right_steps);
  ElectricalOutput evaluate(double dt_s, double chassis_velocity_mps,
                            double body_pitch_rate_rad_s);
  void advance_mechanical_state(double dt_s, double chassis_velocity_before_mps,
                                double body_pitch_rate_before_rad_s,
                                double chassis_velocity_after_mps,
                                double body_pitch_rate_after_rad_s);

  const ElectricalParameters& parameters() const {
    return parameters_;
  }
  const ElectricalOutput& output() const {
    return output_;
  }

  double mechanical_radians_per_step() const;
  double meters_per_step() const;
  double electrical_cycles_per_mechanical_revolution() const;
  double electrical_radians_per_step() const;
  static double wrap_pi(double angle_rad);
  static PhaseCommand phase_for_electrical_angle(double angle_rad);
  static double torque_for_currents(double rotor_electrical_angle_rad, double current_a,
                                    double current_b, double torque_constant_nm_per_a);
  static double back_emf_power(double omega_motor_rad_s, double rotor_electrical_angle_rad,
                               double current_a, double current_b,
                               double back_emf_constant_v_per_rad_s);

  // Test/diagnostic hook for a prescribed mechanical back-drive.
  void set_actual_relative_angles_for_test(double left_angle_rad, double right_angle_rad);

 private:
  struct CurrentUpdateCoefficients {
    double resistance = 0.0;
    double inverse_resistance = 0.0;
    double bus_voltage = 0.0;
    double decay = 1.0;
    double one_minus_decay = 0.0;
    bool active = false;
  };

  struct MotorState {
    double actual_relative_angle_rad = 0.0;
    double current_a = 0.0;
    double current_b = 0.0;
  };

  ElectricalMotorOutput evaluate_motor(double commanded_steps,
                                        double previous_commanded_steps,
                                        MotorState& state, double dt_s,
                                        double chassis_velocity_mps,
                                        double body_pitch_rate_rad_s,
                                        bool have_commanded_index,
                                        std::int64_t commanded_index,
                                        const CurrentUpdateCoefficients& coefficients) const;
  CurrentUpdateCoefficients make_current_update_coefficients(double dt_s) const;
  double update_current(double current_a, double current_ref, double back_emf,
                        const CurrentUpdateCoefficients& coefficients, double* phase_voltage,
                        bool* voltage_saturated) const;

  ElectricalParameters parameters_{};
  MotorState left_state_{};
  MotorState right_state_{};
  double commanded_left_steps_ = 0.0;
  double commanded_right_steps_ = 0.0;
  double previous_left_steps_ = 0.0;
  double previous_right_steps_ = 0.0;
  std::int64_t commanded_left_step_index_ = 0;
  std::int64_t commanded_right_step_index_ = 0;
  bool have_commanded_indices_ = false;
  ElectricalOutput output_{};
};

}  // namespace stepper_phase
