#include "simulator/balancer_simulator.h"
#include "simulator/simulator_runner.h"
#include "simulator/stepper_phase_actuator.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <utility>
#include <vector>

#include "services/main/config.h"

// These actuator tests use old zeroed-outer-loop setup spellings; keep the
// test source buildable while the behavioral assertions move to v12 fields.
#define velocity_damping_per_s velocity_gain_per_s
#define velocity_pitch_limit_deg outer_pitch_limit_deg
#define velocity_I adaptive_com_trim_gain_deg_per_mps_s

namespace {

constexpr double kPi = 3.14159265358979323846;

struct ScopedPidValues {
  ConfigPidValues values = ConfigPid::numeric_values();

  ~ScopedPidValues() {
    ConfigPid::apply_numeric(values);
  }
};

stepper_phase::Parameters nominal_stepper_parameters() {
  return stepper_phase::Parameters{
      .wheel_radius_m = BalancerSimulator::HardwareNominal::stepper_phase_wheel_radius,
      .full_steps_per_revolution = Config::motor_full_steps_per_rev,
      .microsteps_per_full_step = Config::microsteps_per_full_step,
      .current_limit_a = BalancerSimulator::HardwareNominal::stepper_current_limit_a,
      .torque_constant_nm_per_a =
          BalancerSimulator::HardwareNominal::stepper_torque_constant_nm_per_a,
  };
}

stepper_phase::ElectricalParameters nominal_electrical_parameters(double current_limit_a = 1.065,
                                                                  double bus_voltage_v = 11.1) {
  return stepper_phase::ElectricalParameters{
      .wheel_radius_m = BalancerSimulator::HardwareNominal::stepper_phase_wheel_radius,
      .full_steps_per_revolution = Config::motor_full_steps_per_rev,
      .microsteps_per_full_step = Config::microsteps_per_full_step,
      .phase_resistance_ohm = 2.3,
      .phase_inductance_h = 0.0044,
      .current_limit_a = current_limit_a,
      .bus_voltage_v = bus_voltage_v,
      .torque_constant_nm_per_a =
          BalancerSimulator::HardwareNominal::stepper_torque_constant_nm_per_a,
      .back_emf_constant_v_per_rad_s =
          BalancerSimulator::HardwareNominal::stepper_back_emf_constant_v_per_rad_s,
  };
}

SimulatorScenario stepper_recovery_scenario(double pitch_deg, double pitch_rate_dps,
                                            double velocity_sps, bool continuous_field = false) {
  SimulatorScenario scenario;
  scenario.name = "stepper_phase_recovery";
  scenario.physics_profile = PhysicsProfile::StepperPhase;
  scenario.duration_s = 2.0;
  scenario.initial_pitch_deg = pitch_deg;
  scenario.initial_fused_pitch_deg = pitch_deg;
  scenario.initial_pitch_rate_dps = pitch_rate_dps;
  scenario.initial_velocity_mps =
      velocity_sps * BalancerSimulator::HardwareNominal::stepper_phase_meters_per_step;
  scenario.com_angle_offset_rad = 0.0;
  if (continuous_field) {
    scenario.physics_override = BalancerSimulator::physics_for_profile(
        PhysicsProfile::StepperPhase);
    scenario.physics_override->stepper_phase_continuous_field = true;
  }
  return scenario;
}

struct PhaseModeLinearization {
  double effective_inertia_kg_m2 = 0.0;
  double magnetic_stiffness_nm_per_rad = 0.0;
  double effective_damping_nm_s_per_rad = 0.0;
  double natural_frequency_rad_s = 0.0;
  double natural_frequency_hz = 0.0;
  double period_s = 0.0;
  double damping_ratio = 0.0;
};

PhaseModeLinearization linearize_motor_relative_phase(const SimulatorPhysics& physics,
                                                       double current_limit_a,
                                                       double motor_damping = 0.0) {
  using Nominal = BalancerSimulator::HardwareNominal;
  const double mass = Nominal::total_mass_kg;
  const double first_moment = Nominal::first_mass_moment_kg_m;
  const double pitch_inertia = Nominal::pitch_inertia_about_axle_kg_m2;
  const double radius = Nominal::stepper_phase_wheel_radius;
  const double rotor_inertia =
      std::max(0.0, physics.stepper_rotating_inertia_kg_m2_per_motor);
  const double d11 = mass + 2.0 * rotor_inertia / (radius * radius);
  // The wheel/rotor inertia is tied to absolute wheel rotation x/r.  The
  // relative rotor/stator coordinate is only the magnetic phase coordinate.
  const double d12 = first_moment;
  const double d22 = pitch_inertia;
  const double determinant = d11 * d22 - d12 * d12;
  const double inverse_projected_inertia =
      (d22 / (radius * radius) + 2.0 * d12 / radius + d11) / determinant;
  const double effective_inertia = 1.0 / inverse_projected_inertia;
  constexpr double kElectricalCyclesPerMechanicalRevolution = 50.0;
  const double magnetic_stiffness =
      2.0 * BalancerSimulator::HardwareNominal::stepper_torque_constant_nm_per_a *
          std::max(0.0, current_limit_a) *
      kElectricalCyclesPerMechanicalRevolution;

  // z_dot = D^-1 a * J_eff * theta_dot for the pure relative-coordinate
  // mode, with a = [1/r, -1]. Project the existing chassis/pitch damping onto
  // that mode rather than inventing a second damping model.
  const double mode_x_per_theta_dot =
      effective_inertia * (d22 / radius + d12) / determinant;
  const double mode_q_per_theta_dot =
      effective_inertia * (-d12 / radius - d11) / determinant;
  const double base_damping =
      physics.cart_damping * mode_x_per_theta_dot * mode_x_per_theta_dot +
      physics.pitch_damping * mode_q_per_theta_dot * mode_q_per_theta_dot;
  const double total_damping = base_damping + 2.0 * std::max(0.0, motor_damping);
  const double natural_frequency_rad_s =
      magnetic_stiffness > 0.0 ? std::sqrt(magnetic_stiffness / effective_inertia) : 0.0;
  const double natural_frequency_hz = natural_frequency_rad_s / (2.0 * kPi);
  const double damping_denominator =
      2.0 * std::sqrt(std::max(0.0, effective_inertia * magnetic_stiffness));
  return PhaseModeLinearization{
      .effective_inertia_kg_m2 = effective_inertia,
      .magnetic_stiffness_nm_per_rad = magnetic_stiffness,
      .effective_damping_nm_s_per_rad = total_damping,
      .natural_frequency_rad_s = natural_frequency_rad_s,
      .natural_frequency_hz = natural_frequency_hz,
      .period_s = natural_frequency_rad_s > 0.0 ? 2.0 * kPi / natural_frequency_rad_s : 0.0,
      .damping_ratio = damping_denominator > 0.0 ? total_damping / damping_denominator : 0.0,
  };
}

struct PhaseRingdownSummary {
  double frequency_hz = 0.0;
  double damping_ratio = 0.0;
  double initial_phase_deg = 0.0;
  double peak_torque_nm_per_motor = 0.0;
  double maximum_current_error_a = 0.0;
  double final_phase_deg = 0.0;
  std::size_t zero_crossings = 0;
};

struct PhaseMassTerms {
  double d11 = 0.0;
  double d12 = 0.0;
  double d22 = 0.0;
  double determinant = 0.0;
};

PhaseMassTerms phase_mass_terms(const SimulatorPhysics& physics, double pitch_rad) {
  const auto matrix = BalancerSimulator::stepper_mass_matrix(physics, pitch_rad);
  return PhaseMassTerms{matrix.d11, matrix.d12, matrix.d22, matrix.determinant};
}

struct BalanceAcceleration {
  double x_ddot = 0.0;
  double pitch_ddot = 0.0;
};

BalanceAcceleration independently_expected_stepper_acceleration(
    const SimulatorPhysics& physics, double pitch_rad, double total_motor_torque_nm,
    double external_force_n = 0.0) {
  using Nominal = BalancerSimulator::HardwareNominal;
  // Keep this calculation independent of BalancerSimulator::stepper_mass_matrix().
  // The purpose of the invariant is to compare two derivations, not to call the
  // production helper twice.
  const double radius = Nominal::stepper_phase_wheel_radius;
  const double rotating_inertia =
      std::max(0.0, physics.stepper_rotating_inertia_kg_m2_per_motor);
  const double d11 = Nominal::total_mass_kg + 2.0 * rotating_inertia / (radius * radius);
  const double d12 = Nominal::first_mass_moment_kg_m * std::cos(pitch_rad);
  const double d22 = Nominal::pitch_inertia_about_axle_kg_m2;
  const double determinant = d11 * d22 - d12 * d12;
  const double H = Nominal::first_mass_moment_kg_m;
  const double total_mass = Nominal::total_mass_kg;
  const double pitch_rate = 0.0;
  const double rhs_x = total_motor_torque_nm / radius +
                       external_force_n - physics.cart_damping * 0.0;
  const double external_pitch_moment = external_force_n * (H / total_mass) * std::cos(pitch_rad);
  const double rhs_pitch = physics.gravity_mps2 * H * std::sin(pitch_rad) -
                           physics.pitch_damping * pitch_rate - total_motor_torque_nm +
                           external_pitch_moment;
  return BalanceAcceleration{
      .x_ddot = (d22 * rhs_x - d12 * rhs_pitch) / determinant,
      .pitch_ddot = (d11 * rhs_pitch - d12 * rhs_x) / determinant,
  };
}

PhaseRingdownSummary run_fixed_field_ringdown(const SimulatorPhysics& physics,
                                               bool electrical, double current_limit_a,
                                               double initial_phase_deg,
                                               double motor_damping_nm_s_per_rad,
                                               double dt_s = 1.0e-5,
                                               double duration_s = 0.20) {
  using Nominal = BalancerSimulator::HardwareNominal;
  const double radius = Nominal::stepper_phase_wheel_radius;
  const double electrical_cycles = Nominal::stepper_phase_electrical_cycles_per_rev;
  const auto mass_terms = phase_mass_terms(physics, 0.0);
  const double projected_inertia =
      1.0 / (mass_terms.d22 / (radius * radius) + 2.0 * mass_terms.d12 / radius +
             mass_terms.d11) * mass_terms.determinant;
  const double phase_error_rad = initial_phase_deg * kPi / 180.0;
  const double theta_initial = -phase_error_rad / electrical_cycles;
  const double mode_x =
      projected_inertia * (mass_terms.d22 / radius + mass_terms.d12) /
      mass_terms.determinant;
  const double mode_q =
      projected_inertia * (-mass_terms.d12 / radius - mass_terms.d11) /
      mass_terms.determinant;
  double x = mode_x * theta_initial;
  double q = mode_q * theta_initial;
  double x_dot = 0.0;
  double q_dot = 0.0;

  stepper_phase::Actuator ideal_actuator(stepper_phase::Parameters{
      .wheel_radius_m = radius,
      .full_steps_per_revolution = 200,
      .microsteps_per_full_step = 32,
      .current_limit_a = current_limit_a,
      .torque_constant_nm_per_a =
          BalancerSimulator::HardwareNominal::stepper_torque_constant_nm_per_a,
  });
  stepper_phase::ElectricalActuator electrical_actuator(nominal_electrical_parameters(
      current_limit_a, physics.stepper_bus_voltage_v));
  ideal_actuator.set_commanded_microstep_positions(0.0, 0.0);
  electrical_actuator.set_commanded_microstep_positions(0.0, 0.0);

  std::vector<double> times;
  std::vector<double> phases;
  times.reserve(static_cast<std::size_t>(duration_s / dt_s));
  phases.reserve(static_cast<std::size_t>(duration_s / dt_s));
  double peak_torque = 0.0;
  double max_current_error = 0.0;
  const std::size_t sample_count = static_cast<std::size_t>(duration_s / dt_s);
  for (std::size_t sample = 0; sample < sample_count; ++sample) {
    const double theta_motor = x / radius - q;
    if (electrical) {
      electrical_actuator.set_actual_relative_angles_for_test(theta_motor, theta_motor);
    } else {
      ideal_actuator.set_actual_relative_angles_for_test(theta_motor, theta_motor);
    }
    double phase_error = 0.0;
    double torque_per_motor = 0.0;
    double current_error = 0.0;
    if (electrical) {
      const auto output = electrical_actuator.evaluate(dt_s, x_dot, q_dot);
      phase_error = output.left.electrical_phase_error_rad;
      torque_per_motor = output.left.torque_nm;
      current_error = std::max(std::abs(output.left.current_ref_a - output.left.current_a),
                               std::abs(output.left.current_ref_b - output.left.current_b));
    } else {
      const auto output = ideal_actuator.evaluate(dt_s, x_dot, q_dot);
      phase_error = output.left.electrical_phase_error_rad;
      torque_per_motor = output.left.torque_nm;
    }
    const double relative_velocity_rad_s = x_dot / radius - q_dot;
    const double total_torque =
        2.0 * (torque_per_motor - motor_damping_nm_s_per_rad * relative_velocity_rad_s);
    const double pitch_sin = std::sin(q);
    const double rhs_x = total_torque / radius +
                         Nominal::first_mass_moment_kg_m * q_dot * q_dot * pitch_sin -
                         physics.cart_damping * x_dot;
    const double rhs_q = Nominal::gravity * Nominal::first_mass_moment_kg_m * pitch_sin -
                         physics.pitch_damping * q_dot - total_torque;
    const double x_ddot =
        (mass_terms.d22 * rhs_x - mass_terms.d12 * rhs_q) / mass_terms.determinant;
    const double q_ddot =
        (-mass_terms.d12 * rhs_x + mass_terms.d11 * rhs_q) / mass_terms.determinant;
    x_dot += x_ddot * dt_s;
    q_dot += q_ddot * dt_s;
    x += x_dot * dt_s;
    q += q_dot * dt_s;
    times.push_back(static_cast<double>(sample) * dt_s);
    phases.push_back(phase_error);
    peak_torque = std::max(peak_torque, std::abs(torque_per_motor));
    max_current_error = std::max(max_current_error, current_error);
  }

  std::vector<double> zero_crossings;
  for (std::size_t i = 1; i < phases.size(); ++i) {
    if (phases[i - 1] == 0.0 || phases[i - 1] * phases[i] < 0.0) {
      const double denominator = phases[i] - phases[i - 1];
      const double fraction = std::abs(denominator) > 1e-15
                                  ? -phases[i - 1] / denominator
                                  : 0.0;
      zero_crossings.push_back(times[i - 1] + fraction * dt_s);
    }
  }
  double frequency_hz = 0.0;
  if (zero_crossings.size() >= 3) {
    double period_sum = 0.0;
    std::size_t periods = 0;
    for (std::size_t i = 2; i < zero_crossings.size(); ++i) {
      period_sum += zero_crossings[i] - zero_crossings[i - 2];
      ++periods;
    }
    if (periods > 0 && period_sum > 0.0) {
      frequency_hz = static_cast<double>(periods) / period_sum;
    }
  }
  std::vector<std::pair<double, double>> peaks;
  for (std::size_t i = 1; i + 1 < phases.size(); ++i) {
    const bool local_max = phases[i] >= phases[i - 1] && phases[i] >= phases[i + 1];
    const bool local_min = phases[i] <= phases[i - 1] && phases[i] <= phases[i + 1];
    if ((local_max || local_min) && std::abs(phases[i]) > 1e-8) {
      peaks.emplace_back(times[i], std::abs(phases[i]));
    }
  }
  double damping_ratio = 0.0;
  if (peaks.size() >= 3 && frequency_hz > 0.0) {
    double sum_t = 0.0;
    double sum_log = 0.0;
    double sum_tt = 0.0;
    double sum_tlog = 0.0;
    for (const auto& [time, amplitude] : peaks) {
      const double log_amplitude = std::log(amplitude);
      sum_t += time;
      sum_log += log_amplitude;
      sum_tt += time * time;
      sum_tlog += time * log_amplitude;
    }
    const double count = static_cast<double>(peaks.size());
    const double denominator = count * sum_tt - sum_t * sum_t;
    const double slope = std::abs(denominator) > 1e-15
                             ? (count * sum_tlog - sum_t * sum_log) / denominator
                             : 0.0;
    const double decay_rate = std::max(0.0, -slope);
    const double damped_frequency_rad_s = 2.0 * kPi * frequency_hz;
    damping_ratio = decay_rate /
                    std::sqrt(decay_rate * decay_rate +
                              damped_frequency_rad_s * damped_frequency_rad_s);
  }
  return PhaseRingdownSummary{
      .frequency_hz = frequency_hz,
      .damping_ratio = damping_ratio,
      .initial_phase_deg = initial_phase_deg,
      .peak_torque_nm_per_motor = peak_torque,
      .maximum_current_error_a = max_current_error,
      .final_phase_deg = phases.empty() ? 0.0 : phases.back() * 180.0 / kPi,
      .zero_crossings = zero_crossings.size(),
  };
}

struct FreeWheelRingdownSummary {
  int step_events = 0;
  double first_step_time_s = 0.0;
  double second_step_time_s = 0.0;
  double commanded_mechanical_displacement_rad = 0.0;
  double final_mechanical_angle_rad = 0.0;
  double frequency_hz = 0.0;
  double damping_ratio = 0.0;
  double peak_torque_nm = 0.0;
};

// Deterministic optical-fixture proxy: the stator/body is fixed, one wheel is
// free, and one electrical StepperPhase motor receives timestamped 1/32 STEP
// edges. The wheel angle is the actuator's verified relative mechanical angle;
// no extra force or force-lag model is involved.
FreeWheelRingdownSummary run_free_wheel_fixture(int signed_step_count,
                                                double dt_s = 1.0e-6,
                                                double duration_s = 0.20,
                                                bool electrical = true) {
  using Nominal = BalancerSimulator::HardwareNominal;
  const auto parameters = nominal_electrical_parameters();
  stepper_phase::Actuator ideal_actuator(stepper_phase::Parameters{
      .wheel_radius_m = parameters.wheel_radius_m,
      .full_steps_per_revolution = parameters.full_steps_per_revolution,
      .microsteps_per_full_step = parameters.microsteps_per_full_step,
      .current_limit_a = parameters.current_limit_a,
      .torque_constant_nm_per_a = parameters.torque_constant_nm_per_a,
  });
  stepper_phase::ElectricalActuator actuator(parameters);
  const double wheel_inertia = Nominal::stepper_rotating_inertia_kg_m2_per_motor;
  const double damping = Nominal::stepper_motor_relative_damping_nm_s_per_rad;
  const int magnitude = std::abs(signed_step_count);
  const double direction = signed_step_count < 0 ? -1.0 : 1.0;
  const int second_step_sample = 100;  // 100 us edge separation, as measured.
  double actual_angle_rad = 0.0;
  double wheel_omega_rad_s = 0.0;
  int emitted_steps = 0;
  std::vector<double> times;
  std::vector<double> phase_errors;
  times.reserve(static_cast<std::size_t>(duration_s / dt_s));
  phase_errors.reserve(times.capacity());

  const std::size_t sample_count = static_cast<std::size_t>(duration_s / dt_s);
  for (std::size_t sample = 0; sample < sample_count; ++sample) {
    if (magnitude >= 1 && emitted_steps == 0) {
      emitted_steps = 1;
    }
    if (magnitude >= 2 && static_cast<int>(sample) >= second_step_sample && emitted_steps == 1) {
      emitted_steps = 2;
    }
    const double commanded_steps = direction * static_cast<double>(emitted_steps);
    double motor_torque_nm = 0.0;
    double phase_error_rad = 0.0;
    if (electrical) {
      actuator.set_commanded_microstep_positions(commanded_steps, 0.0);
      actuator.set_actual_relative_angles_for_test(actual_angle_rad, 0.0);
      const auto output = actuator.evaluate(
          dt_s, wheel_omega_rad_s * parameters.wheel_radius_m, 0.0);
      motor_torque_nm = output.left.torque_nm;
      phase_error_rad = output.left.electrical_phase_error_rad;
    } else {
      ideal_actuator.set_commanded_microstep_positions(commanded_steps, 0.0);
      ideal_actuator.set_actual_relative_angles_for_test(actual_angle_rad, 0.0);
      const auto output = ideal_actuator.evaluate(
          dt_s, wheel_omega_rad_s * parameters.wheel_radius_m, 0.0);
      motor_torque_nm = output.left.torque_nm;
      phase_error_rad = output.left.electrical_phase_error_rad;
    }
    const double torque_nm = motor_torque_nm - damping * wheel_omega_rad_s;
    wheel_omega_rad_s += torque_nm / wheel_inertia * dt_s;
    actual_angle_rad += wheel_omega_rad_s * dt_s;
    times.push_back(static_cast<double>(sample) * dt_s);
    phase_errors.push_back(phase_error_rad);
  }

  std::vector<double> zero_crossings;
  for (std::size_t index = 1; index < phase_errors.size(); ++index) {
    if (phase_errors[index - 1] * phase_errors[index] < 0.0) {
      const double denominator = phase_errors[index] - phase_errors[index - 1];
      const double fraction = -phase_errors[index - 1] / denominator;
      zero_crossings.push_back(times[index - 1] + fraction * dt_s);
    }
  }
  double frequency_hz = 0.0;
  if (zero_crossings.size() >= 3) {
    double period_sum = 0.0;
    for (std::size_t index = 2; index < zero_crossings.size(); ++index) {
      period_sum += zero_crossings[index] - zero_crossings[index - 2];
    }
    frequency_hz = static_cast<double>(zero_crossings.size() - 2U) / period_sum;
  }
  std::vector<std::pair<double, double>> peaks;
  for (std::size_t index = 1; index + 1 < phase_errors.size(); ++index) {
    const bool maximum = phase_errors[index] >= phase_errors[index - 1] &&
                         phase_errors[index] >= phase_errors[index + 1];
    const bool minimum = phase_errors[index] <= phase_errors[index - 1] &&
                         phase_errors[index] <= phase_errors[index + 1];
    if ((maximum || minimum) && std::abs(phase_errors[index]) > 1.0e-8) {
      peaks.emplace_back(times[index], std::abs(phase_errors[index]));
    }
  }
  double damping_ratio = 0.0;
  if (peaks.size() >= 3U && frequency_hz > 0.0) {
    double first_time = peaks.front().first;
    double first_log = std::log(peaks.front().second);
    double last_time = peaks.back().first;
    double last_log = std::log(peaks.back().second);
    const double decay_rate = std::max(0.0, (first_log - last_log) /
                                                  std::max(1.0e-12, last_time - first_time));
    const double damped_frequency_rad_s = 2.0 * kPi * frequency_hz;
    damping_ratio = decay_rate /
                    std::sqrt(decay_rate * decay_rate + damped_frequency_rad_s *
                                                            damped_frequency_rad_s);
  }
  return FreeWheelRingdownSummary{
      .step_events = emitted_steps,
      .first_step_time_s = magnitude >= 1 ? 0.0 : -1.0,
      .second_step_time_s = magnitude >= 2 ? second_step_sample * dt_s : -1.0,
      .commanded_mechanical_displacement_rad =
          static_cast<double>(signed_step_count) * actuator.mechanical_radians_per_step(),
      .final_mechanical_angle_rad = actual_angle_rad,
      .frequency_hz = frequency_hz,
      .damping_ratio = damping_ratio,
      .peak_torque_nm = parameters.torque_constant_nm_per_a * parameters.current_limit_a,
  };
}

TEST(StepperPhaseActuatorTest, HardwareKinematicsUseAuthoritativeOneThirtySecondMicrosteps) {
  const auto parameters = nominal_stepper_parameters();
  stepper_phase::Actuator actuator(parameters);
  constexpr double kSps = 8000.0;

  EXPECT_NEAR(BalancerSimulator::HardwareNominal::stepper_phase_steps_per_rev,
              Config::steps_per_rev, 1e-12);
  EXPECT_NEAR(BalancerSimulator::HardwareNominal::stepper_phase_meters_per_step,
              Config::meters_per_step, 1e-15);
  EXPECT_NEAR(actuator.mechanical_radians_per_step(), 2.0 * kPi / 6400.0, 1e-15);
  EXPECT_NEAR(actuator.meters_per_step(), Config::meters_per_step, 1e-15);
  EXPECT_NEAR(3200.0 * actuator.meters_per_step(),
              0.5 * Config::wheel_diam_m * kPi, 1e-12);
  EXPECT_NEAR(6400.0 * actuator.meters_per_step(), Config::wheel_diam_m * kPi, 1e-12);
  EXPECT_NEAR(kSps / 6400.0 * 60.0, 75.0, 1e-12);
  EXPECT_NEAR(kSps * actuator.meters_per_step(), 0.323584, 5e-4);

  actuator.set_commanded_microstep_positions(1.0, 1.0);
  const auto one_step = actuator.evaluate(1.0, 0.0, 0.0);
  EXPECT_EQ(one_step.left.commanded_microstep_index, 1);
  EXPECT_NEAR(one_step.left.commanded_mechanical_angle_rad, 0.05625 * kPi / 180.0,
              1e-15);
  EXPECT_NEAR(one_step.left.commanded_field_electrical_angle_rad,
              2.0 * kPi / BalancerSimulator::HardwareNominal::
                  stepper_phase_reference_lut_states_per_cycle,
              1e-14);
  const auto one_lut_state = stepper_phase::Actuator::phase_for_electrical_angle(
      2.0 * kPi /
      BalancerSimulator::HardwareNominal::stepper_phase_reference_lut_states_per_cycle);
  EXPECT_NEAR(one_step.left.phase.a, one_lut_state.a, 1e-12);
  EXPECT_NEAR(one_step.left.phase.b, one_lut_state.b, 1e-12);

  actuator.set_commanded_microstep_positions(6400.0, 6400.0);
  const auto output = actuator.evaluate(1.0, 0.0, 0.0);
  EXPECT_EQ(output.left.commanded_microstep_index, 0);
  EXPECT_NEAR(output.left.commanded_mechanical_angle_rad, 2.0 * kPi, 1e-12);
  EXPECT_NEAR(output.left.commanded_field_electrical_angle_rad, 100.0 * kPi, 1e-10);
  EXPECT_NEAR(BalancerSimulator::HardwareNominal::stepper_phase_lut_state_stride, 1.0,
              1e-12);
}

TEST(StepperPhaseActuatorTest, IntegerCommandPathWrapsSignedStepIndices) {
  const auto parameters = nominal_stepper_parameters();
  stepper_phase::Actuator actuator(parameters);
  actuator.set_commanded_microstep_indices(-1, 6400 + 17);
  const auto output = actuator.evaluate(1.0e-4, 0.0, 0.0);

  EXPECT_EQ(output.left.commanded_microstep_index, 6399);
  EXPECT_EQ(output.right.commanded_microstep_index, 17);
  const double step_angle = actuator.electrical_radians_per_step();
  const auto expected_left = stepper_phase::Actuator::phase_for_electrical_angle(-step_angle);
  const auto expected_right = stepper_phase::Actuator::phase_for_electrical_angle(17.0 * step_angle);
  EXPECT_NEAR(output.left.phase.a, expected_left.a, 1.0e-14);
  EXPECT_NEAR(output.left.phase.b, expected_left.b, 1.0e-14);
  EXPECT_NEAR(output.right.phase.a, expected_right.a, 1.0e-14);
  EXPECT_NEAR(output.right.phase.b, expected_right.b, 1.0e-14);
}

TEST(StepperPhaseActuatorTest, OneEighthToOneThirtySecondPreservesFieldMotionAtFourfoldSps) {
  constexpr double radius_m = Config::wheel_radius_m;
  constexpr double one_eighth_steps_per_rev =
      static_cast<double>(Config::motor_full_steps_per_rev * 8);
  constexpr double one_thirty_second_steps_per_rev = Config::steps_per_rev;
  const auto field_speed_mps = [radius_m](double sps, double steps_per_rev) {
    return sps * (2.0 * kPi * radius_m / steps_per_rev);
  };
  const auto field_speed_rad_s = [](double sps, double steps_per_rev) {
    return sps * (2.0 * kPi / steps_per_rev);
  };

  EXPECT_DOUBLE_EQ(one_thirty_second_steps_per_rev / one_eighth_steps_per_rev, 4.0);
  EXPECT_NEAR(field_speed_mps(6000.0, one_eighth_steps_per_rev),
              field_speed_mps(24000.0, one_thirty_second_steps_per_rev), 1e-15);
  EXPECT_NEAR(field_speed_rad_s(350.0, one_eighth_steps_per_rev),
              field_speed_rad_s(1400.0, one_thirty_second_steps_per_rev), 1e-15);
  EXPECT_NEAR(field_speed_rad_s(6000.0, one_eighth_steps_per_rev),
              field_speed_rad_s(24000.0, one_thirty_second_steps_per_rev), 1e-15);
}

TEST(StepperPhaseActuatorTest, TorqueConstantUsesTwoPhaseVectorNormalization) {
  using Nominal = BalancerSimulator::HardwareNominal;
  const double expected_constant =
      Nominal::motor_stall_torque_nm /
      (Nominal::stepper_sqrt_two * Nominal::motor_rated_phase_current_a);
  EXPECT_NEAR(Nominal::stepper_torque_constant_nm_per_a, expected_constant, 1e-15);
  EXPECT_NEAR(Nominal::stepper_back_emf_constant_v_per_rad_s,
              Nominal::stepper_torque_constant_nm_per_a, 1e-15);
  EXPECT_NEAR(Nominal::stepper_peak_torque_nm_per_motor,
              Nominal::stepper_torque_constant_nm_per_a * Nominal::stepper_current_limit_a,
              1e-15);
  const auto phase = stepper_phase::Actuator::phase_for_electrical_angle(kPi / 4.0);
  EXPECT_NEAR(std::hypot(phase.a, phase.b), 1.0, 1e-15);
}

TEST(StepperPhaseElectricalTest, UsesAuthoritativeGeometryAndOneThirtySecondStepStride) {
  const auto parameters = nominal_electrical_parameters();
  stepper_phase::ElectricalActuator actuator(parameters);

  EXPECT_EQ(parameters.full_steps_per_revolution * parameters.microsteps_per_full_step,
            Config::commanded_steps_per_rev);
  EXPECT_NEAR(actuator.mechanical_radians_per_step(), 2.0 * kPi / 6400.0, 1e-15);
  EXPECT_NEAR(actuator.meters_per_step(), Config::meters_per_step, 1e-15);
  EXPECT_NEAR(0.5 * Config::commanded_steps_per_rev * actuator.meters_per_step(),
              kPi * parameters.wheel_radius_m,
              1e-12);
  EXPECT_NEAR(Config::commanded_steps_per_rev * actuator.meters_per_step(),
              2.0 * kPi * parameters.wheel_radius_m,
              1e-12);
  EXPECT_NEAR(actuator.electrical_radians_per_step(), 2.0 * kPi / 128.0, 1e-14);

  actuator.set_commanded_microstep_positions(1.0, 1.0);
  const auto output = actuator.evaluate(1.0, 0.0, 0.0);
  EXPECT_EQ(output.left.commanded_microstep_index, 1);
  EXPECT_NEAR(output.left.commanded_mechanical_angle_rad, 0.05625 * kPi / 180.0, 1e-15);
  EXPECT_NEAR(output.left.commanded_field_electrical_angle_rad, 2.8125 * kPi / 180.0,
              1e-14);
}

TEST(StepperPhaseElectricalTest, IntegerCommandPathUsesNominalLutAndCustomFallback) {
  auto nominal = nominal_electrical_parameters();
  stepper_phase::ElectricalActuator nominal_actuator(nominal);
  nominal_actuator.set_commanded_microstep_indices(-1, 17);
  const auto nominal_output = nominal_actuator.evaluate(1.0e-4, 0.0, 0.0);
  const double nominal_step_angle = nominal_actuator.electrical_radians_per_step();
  const auto expected_negative =
      stepper_phase::ElectricalActuator::phase_for_electrical_angle(-nominal_step_angle);
  EXPECT_EQ(nominal_output.left.commanded_microstep_index, 6399);
  EXPECT_NEAR(nominal_output.left.phase.a, expected_negative.a, 1.0e-14);
  EXPECT_NEAR(nominal_output.left.phase.b, expected_negative.b, 1.0e-14);

  auto custom = nominal;
  custom.microsteps_per_full_step = 16;
  stepper_phase::ElectricalActuator custom_actuator(custom);
  custom_actuator.set_commanded_microstep_indices(3, 3);
  const auto custom_output = custom_actuator.evaluate(1.0e-4, 0.0, 0.0);
  const auto expected_custom =
      stepper_phase::ElectricalActuator::phase_for_electrical_angle(
          custom_output.left.commanded_field_electrical_angle_rad);
  EXPECT_NEAR(custom_output.left.phase.a, expected_custom.a, 1.0e-14);
  EXPECT_NEAR(custom_output.left.phase.b, expected_custom.b, 1.0e-14);
}

TEST(StepperPhaseElectricalTest, FreeWheelFixtureUsesTimestampedOneThirtySecondEvents) {
  const auto one_ideal = run_free_wheel_fixture(1, 1.0e-6, 0.20, false);
  const auto one_positive = run_free_wheel_fixture(1);
  const auto two_positive = run_free_wheel_fixture(2);
  const auto one_negative = run_free_wheel_fixture(-1);

  const double expected_step = 2.0 * kPi / 6400.0;
  EXPECT_EQ(one_positive.step_events, 1);
  EXPECT_EQ(two_positive.step_events, 2);
  EXPECT_EQ(one_negative.step_events, 1);
  EXPECT_NEAR(one_positive.commanded_mechanical_displacement_rad, expected_step, 1e-15);
  EXPECT_NEAR(two_positive.commanded_mechanical_displacement_rad, 2.0 * expected_step,
              1e-15);
  EXPECT_NEAR(one_negative.commanded_mechanical_displacement_rad, -expected_step, 1e-15);
  EXPECT_NEAR(two_positive.second_step_time_s - two_positive.first_step_time_s, 100.0e-6,
              1e-15);

  // The fixed-stator estimate is intentionally a comparison, not a fitted
  // assertion: Kphase=Kt*I*Ne and the measured rotating inertia predict a
  // roughly 94 Hz unloaded mode before any unmodeled fixture loss.
  const auto parameters = nominal_electrical_parameters();
  const double stiffness =
      parameters.torque_constant_nm_per_a * parameters.current_limit_a * 50.0;
  const double predicted_hz =
      std::sqrt(stiffness / BalancerSimulator::HardwareNominal::
                              stepper_rotating_inertia_kg_m2_per_motor) /
      (2.0 * kPi);
  EXPECT_NEAR(one_positive.frequency_hz, predicted_hz, 2.0);
  EXPECT_NEAR(two_positive.frequency_hz, predicted_hz, 2.0);
  EXPECT_NEAR(one_negative.frequency_hz, predicted_hz, 2.0);
  EXPECT_GT(one_positive.damping_ratio, 0.02);
  EXPECT_LT(one_positive.damping_ratio, 0.12);
  EXPECT_NEAR(one_positive.peak_torque_nm,
              parameters.torque_constant_nm_per_a * parameters.current_limit_a, 1e-12);
  EXPECT_NEAR(one_ideal.frequency_hz, predicted_hz, 2.0);
  EXPECT_NEAR(one_ideal.frequency_hz, one_positive.frequency_hz, 0.2);
  EXPECT_GT(predicted_hz, 93.0);
  EXPECT_LT(predicted_hz, 95.0);
  std::cout << "stepper_free_wheel_fixture predicted_hz=" << predicted_hz
            << " ideal_hz=" << one_ideal.frequency_hz
            << " one_hz=" << one_positive.frequency_hz
            << " two_hz=" << two_positive.frequency_hz
            << " negative_hz=" << one_negative.frequency_hz
            << " zeta=" << one_positive.damping_ratio
            << " two_step_spacing_us="
            << (two_positive.second_step_time_s - two_positive.first_step_time_s) * 1e6
            << '\n';
}

TEST(StepperPhaseElectricalTest, IdealCurrentTorqueEquationMatchesReferenceCurve) {
  const auto parameters = nominal_electrical_parameters();
  const double field_electrical_angle = 17.0 * kPi / 180.0;
  const double phase_error = 31.0 * kPi / 180.0;
  const auto phase = stepper_phase::ElectricalActuator::phase_for_electrical_angle(
      field_electrical_angle);
  const double rotor_electrical_angle = field_electrical_angle - phase_error;
  const double actual_torque = stepper_phase::ElectricalActuator::torque_for_currents(
      rotor_electrical_angle, parameters.current_limit_a * phase.a,
      parameters.current_limit_a * phase.b, parameters.torque_constant_nm_per_a);
  const double expected_torque = parameters.torque_constant_nm_per_a * parameters.current_limit_a *
                                 std::sin(phase_error);
  EXPECT_NEAR(actual_torque, expected_torque, 1e-12);
}

TEST(StepperPhaseElectricalTest, StaticPrescribedPhaseTorqueHasCorrectAmplitudeAndSymmetry) {
  const auto parameters = nominal_electrical_parameters();
  const double field_electrical_angle = 37.0 * kPi / 180.0;
  for (const double phase_error_rad : {-1.0, -0.4, -0.1, 0.1, 0.4, 1.0}) {
    stepper_phase::ElectricalActuator actuator(parameters);
    const double commanded_steps = field_electrical_angle / actuator.electrical_radians_per_step();
    const double actual_relative_angle =
        (field_electrical_angle - phase_error_rad) /
        actuator.electrical_cycles_per_mechanical_revolution();
    actuator.set_commanded_microstep_positions(commanded_steps, commanded_steps);
    actuator.set_actual_relative_angles_for_test(actual_relative_angle, actual_relative_angle);
    stepper_phase::ElectricalOutput output{};
    for (int sample = 0; sample < 1000; ++sample) {
      output = actuator.evaluate(1.0e-4, 0.0, 0.0);
    }
    const double expected_torque = parameters.torque_constant_nm_per_a *
                                   parameters.current_limit_a * std::sin(phase_error_rad);
    EXPECT_NEAR(output.left.electrical_phase_error_rad, phase_error_rad, 1.0e-10);
    EXPECT_NEAR(output.left.torque_nm, expected_torque, 1.0e-7);
    EXPECT_NEAR(output.right.torque_nm, expected_torque, 1.0e-7);
    EXPECT_LE(std::abs(output.left.current_a), parameters.current_limit_a + 1.0e-12);
    EXPECT_LE(std::abs(output.left.current_b), parameters.current_limit_a + 1.0e-12);
  }
}

TEST(StepperPhaseElectricalTest, ElectricalPowerMatchesTorqueBackEmfAndCopperLoss) {
  const auto parameters = nominal_electrical_parameters();
  constexpr double dt_s = 1.0e-6;
  constexpr int samples = 2000;
  constexpr double omega_rad_s = 2.0;
  const double electrical_cycles =
      parameters.full_steps_per_revolution / 4.0;
  double positive_mechanical_work = 0.0;
  double negative_mechanical_work = 0.0;
  double positive_electrical_work = 0.0;
  double negative_electrical_work = 0.0;
  double positive_resistive_work = 0.0;
  double negative_resistive_work = 0.0;

  for (const double direction : {1.0, -1.0}) {
    stepper_phase::ElectricalActuator actuator(parameters);
    actuator.set_commanded_microstep_positions(0.0, 0.0);
    const double initial_relative_angle = -0.25 / electrical_cycles;
    actuator.set_actual_relative_angles_for_test(initial_relative_angle,
                                                 initial_relative_angle);
    auto initial = actuator.evaluate(0.0, 0.0, 0.0);
    const double initial_magnetic_energy = initial.left.magnetic_energy_j;
    double electrical_work = 0.0;
    double mechanical_work = 0.0;
    double resistive_work = 0.0;
    double relative_angle = initial_relative_angle;
    for (int sample = 0; sample < samples; ++sample) {
      actuator.set_actual_relative_angles_for_test(relative_angle, relative_angle);
      const auto output = actuator.evaluate(
          dt_s, direction * omega_rad_s * parameters.wheel_radius_m, 0.0);
      electrical_work += output.left.electrical_power_w * dt_s;
      mechanical_work += output.left.mechanical_power_w * dt_s;
      resistive_work += output.left.resistive_loss_w * dt_s;
      relative_angle += direction * omega_rad_s * dt_s;
    }
    actuator.set_actual_relative_angles_for_test(relative_angle, relative_angle);
    const auto final = actuator.evaluate(0.0, direction * omega_rad_s * parameters.wheel_radius_m,
                                         0.0);
    const double magnetic_energy_change = final.left.magnetic_energy_j - initial_magnetic_energy;
    EXPECT_NEAR(electrical_work,
                mechanical_work + resistive_work + magnetic_energy_change, 2.0e-5);
    EXPECT_NEAR(final.left.back_emf_b, direction *
                                           parameters.back_emf_constant_v_per_rad_s *
                                           omega_rad_s *
                                           std::cos(final.left.actual_rotor_electrical_angle_rad),
                1.0e-9);
    if (direction > 0.0) {
      positive_mechanical_work = mechanical_work;
      positive_electrical_work = electrical_work;
      positive_resistive_work = resistive_work;
    } else {
      negative_mechanical_work = mechanical_work;
      negative_electrical_work = electrical_work;
      negative_resistive_work = resistive_work;
    }
  }

  EXPECT_GT(positive_mechanical_work, 0.0);
  EXPECT_LT(negative_mechanical_work, 0.0);
  EXPECT_GT(positive_electrical_work, positive_resistive_work);
  EXPECT_LT(negative_electrical_work, negative_resistive_work);
}

TEST(StepperPhaseElectricalTest, OneStepCurrentAndTorqueRiseAreVoltageLimited) {
  const auto parameters = nominal_electrical_parameters();
  stepper_phase::ElectricalActuator actuator(parameters);
  actuator.set_commanded_microstep_positions(1.0, 1.0);

  const double ideal_torque = parameters.torque_constant_nm_per_a * parameters.current_limit_a *
                              std::sin(actuator.electrical_radians_per_step());
  double first_torque = 0.0;
  double t50_s = -1.0;
  double t90_s = -1.0;
  bool any_voltage_saturated = false;
  stepper_phase::ElectricalOutput output{};
  constexpr double dt_s = 1e-5;
  for (int sample = 0; sample < 5000; ++sample) {
    output = actuator.evaluate(dt_s, 0.0, 0.0);
    any_voltage_saturated |= output.left.voltage_saturated;
    if (sample == 0) first_torque = output.left.torque_nm;
    const double elapsed_s = (sample + 1) * dt_s;
    if (t50_s < 0.0 && output.left.torque_nm >= 0.50 * ideal_torque) t50_s = elapsed_s;
    if (t90_s < 0.0 && output.left.torque_nm >= 0.90 * ideal_torque) t90_s = elapsed_s;
  }

  EXPECT_GT(ideal_torque, 0.0);
  EXPECT_GE(first_torque, 0.0);
  EXPECT_LT(first_torque, 0.5 * ideal_torque);
  EXPECT_GE(t50_s, 0.0);
  EXPECT_GE(t90_s, t50_s);
  EXPECT_NEAR(output.left.torque_nm, ideal_torque, 1e-5);
  EXPECT_TRUE(any_voltage_saturated);
  std::cout << "stepper_electrical_one_step ideal_torque_nm=" << ideal_torque
            << " first_torque_nm=" << first_torque << " t50_ms=" << t50_s * 1000.0
            << " t90_ms=" << t90_s * 1000.0 << " final_torque_nm=" << output.left.torque_nm
            << '\n';
}

TEST(StepperPhaseElectricalTest, HoldingAndBackDrivePersistAtZeroStepRate) {
  const auto parameters = nominal_electrical_parameters();
  stepper_phase::ElectricalActuator actuator(parameters);
  actuator.set_commanded_microstep_positions(0.0, 0.0);
  auto aligned = actuator.evaluate(1e-4, 0.0, 0.0);
  EXPECT_NEAR(aligned.left.torque_nm, 0.0, 1e-12);

  // The field remains energized after STEP stops.  A 90-degree electrical
  // displacement therefore produces a finite restoring torque at zero SPS.
  actuator.set_actual_relative_angles_for_test(kPi / (2.0 * 50.0), kPi / (2.0 * 50.0));
  for (int sample = 0; sample < 1000; ++sample) {
    aligned = actuator.evaluate(1e-4, 0.0, 0.0);
  }
  EXPECT_NEAR(aligned.left.commanded_field_velocity_mps, 0.0, 1e-12);
  EXPECT_NEAR(aligned.left.torque_nm, -parameters.torque_constant_nm_per_a *
                                      parameters.current_limit_a,
              1e-6);
  EXPECT_LT(aligned.left.torque_nm, 0.0);
}

TEST(StepperPhaseElectricalTest, BackEmfReversesAndPowerMatchesMechanicalConvention) {
  const auto parameters = nominal_electrical_parameters();
  stepper_phase::ElectricalActuator actuator(parameters);
  actuator.set_commanded_microstep_positions(0.0, 0.0);
  const double omega = 40.0;
  auto positive = actuator.evaluate(1e-4, omega * parameters.wheel_radius_m, 0.0);
  auto negative = actuator.evaluate(1e-4, -omega * parameters.wheel_radius_m, 0.0);
  EXPECT_NEAR(positive.left.back_emf_a, 0.0, 1e-12);
  EXPECT_GT(positive.left.back_emf_b, 0.0);
  EXPECT_NEAR(negative.left.back_emf_b, -positive.left.back_emf_b, 1e-12);

  const double arbitrary_projection = stepper_phase::ElectricalActuator::back_emf_power(
      omega, 0.4, 0.7, -0.3, parameters.back_emf_constant_v_per_rad_s);
  const double corresponding_torque = stepper_phase::ElectricalActuator::torque_for_currents(
      0.4, 0.7, -0.3, parameters.torque_constant_nm_per_a);
  EXPECT_NEAR(arbitrary_projection, omega * corresponding_torque, 1e-12);
}

TEST(StepperPhaseElectricalTest, CurrentLimitSetsStaticTorqueWithoutSeparateTorqueClamp) {
  for (const double current_limit : {1.0, 1.2, 1.5}) {
    const auto parameters = nominal_electrical_parameters(current_limit);
    stepper_phase::ElectricalActuator actuator(parameters);
    actuator.set_commanded_microstep_positions(0.0, 0.0);
    actuator.set_actual_relative_angles_for_test(kPi / (2.0 * 50.0), kPi / (2.0 * 50.0));
    auto output = actuator.evaluate(1e-4, 0.0, 0.0);
    EXPECT_NEAR(std::abs(output.left.torque_nm), parameters.torque_constant_nm_per_a *
                                                      current_limit,
                1e-9);
    EXPECT_NEAR(std::abs(output.right.torque_nm), parameters.torque_constant_nm_per_a *
                                                       current_limit,
                1e-9);
  }
}

TEST(StepperPhaseElectricalTest, HigherBusVoltageShortensOneStepCurrentRise) {
  auto time_to_ninety_percent = [](double bus_voltage_v) {
    const auto parameters = nominal_electrical_parameters(
        BalancerSimulator::HardwareNominal::stepper_current_limit_a, bus_voltage_v);
    stepper_phase::ElectricalActuator actuator(parameters);
    actuator.set_commanded_microstep_positions(1.0, 1.0);
    const double ideal_torque = parameters.torque_constant_nm_per_a * parameters.current_limit_a *
                                std::sin(actuator.electrical_radians_per_step());
    constexpr double dt_s = 1e-6;
    for (int sample = 0; sample < 1000; ++sample) {
      const auto output = actuator.evaluate(dt_s, 0.0, 0.0);
      if (output.left.torque_nm >= 0.90 * ideal_torque) return (sample + 1) * dt_s;
    }
    return 1.0;
  };

  const double t10 = time_to_ninety_percent(10.0);
  const double t111 = time_to_ninety_percent(11.1);
  const double t126 = time_to_ninety_percent(12.6);
  EXPECT_LT(t126, t111);
  EXPECT_LT(t111, t10);
  std::cout << "stepper_electrical_bus_sensitivity t10_us=" << t10 * 1e6
            << " t11_1_us=" << t111 * 1e6 << " t12_6_us=" << t126 * 1e6 << '\n';
}

TEST(StepperPhaseElectricalTest, ElectricalUpdateConvergesWhenTimeStepIsHalved) {
  auto torque_after = [](double dt_s) {
    const auto parameters = nominal_electrical_parameters();
    stepper_phase::ElectricalActuator actuator(parameters);
    actuator.set_commanded_microstep_positions(1.0, 1.0);
    const int samples = static_cast<int>(std::llround(0.002 / dt_s));
    stepper_phase::ElectricalOutput output{};
    for (int sample = 0; sample < samples; ++sample) {
      output = actuator.evaluate(dt_s, 0.0, 0.0);
    }
    return output.left.torque_nm;
  };

  const double coarse = torque_after(1e-5);
  const double fine = torque_after(5e-6);
  EXPECT_NEAR(coarse, fine, 1e-7);
}

TEST(StepperPhaseElectricalTest, ConstantFieldSpeedHasNoPersistentSpsForce) {
  const auto parameters = nominal_electrical_parameters();
  constexpr double dt_s = 1e-4;
  double commanded_steps = 0.0;
  double max_low_speed_error = 0.0;
  double max_high_speed_error = 0.0;
  bool high_speed_voltage_limited = false;

  for (const double steps_per_second : {100.0, 1000.0, 8000.0, 16000.0, 32000.0,
                                        64000.0}) {
    stepper_phase::ElectricalActuator actuator(parameters);
    commanded_steps = 0.0;
    for (int sample = 0; sample < 1000; ++sample) {
      commanded_steps += steps_per_second * dt_s;
      actuator.set_commanded_microstep_positions(commanded_steps, commanded_steps);
      actuator.set_actual_relative_angles_for_test(
          commanded_steps * actuator.mechanical_radians_per_step(),
          commanded_steps * actuator.mechanical_radians_per_step());
      const auto output = actuator.evaluate(
          dt_s, steps_per_second * actuator.meters_per_step(), 0.0);
      const double error = std::abs(output.left.current_ref_a - output.left.current_a) +
                           std::abs(output.left.current_ref_b - output.left.current_b);
      if (steps_per_second <= 1000.0) {
        max_low_speed_error = std::max(max_low_speed_error, error);
      } else {
        max_high_speed_error = std::max(max_high_speed_error, error);
      }
      if (steps_per_second >= 8000.0) high_speed_voltage_limited |= output.left.voltage_saturated;
    }
  }

  EXPECT_LT(max_low_speed_error, 0.05);
  EXPECT_GT(max_high_speed_error, max_low_speed_error);
  EXPECT_TRUE(high_speed_voltage_limited);
  std::cout << "stepper_electrical_speed_tracking low_speed_current_error="
            << max_low_speed_error << " high_speed_current_error=" << max_high_speed_error
            << " high_speed_voltage_limited=" << high_speed_voltage_limited << '\n';
}

TEST(StepperPhaseActuatorTest, HoldingAndBackDriveTorquePersistAtZeroStepRate) {
  stepper_phase::Actuator actuator(nominal_stepper_parameters());
  actuator.set_commanded_microstep_positions(0.0, 0.0);
  const auto aligned = actuator.evaluate(0.01, 0.0, 0.0);
  EXPECT_NEAR(aligned.left.torque_nm, 0.0, 1e-12);

  actuator.set_commanded_microstep_positions(16.0, 16.0);
  const auto advanced = actuator.evaluate(0.01, 0.0, 0.0);
  EXPECT_GT(advanced.left.torque_nm, 0.0);

  // No new STEP edges: the magnetic equilibrium remains where it was.
  const auto held = actuator.evaluate(0.01, 0.0, 0.0);
  EXPECT_NEAR(held.left.commanded_field_velocity_mps, 0.0, 1e-12);
  EXPECT_NEAR(held.left.torque_nm, advanced.left.torque_nm, 1e-12);

  // A positive wheel-relative displacement is opposed by negative torque.
  actuator.set_commanded_microstep_positions(0.0, 0.0);
  actuator.set_actual_relative_angles_for_test(
      actuator.mechanical_radians_per_step(), actuator.mechanical_radians_per_step());
  const auto back_driven = actuator.evaluate(0.01, 0.0, 0.0);
  EXPECT_LT(back_driven.left.torque_nm, 0.0);
  EXPECT_LE(std::abs(back_driven.left.torque_nm),
            nominal_stepper_parameters().current_limit_a *
                    nominal_stepper_parameters().torque_constant_nm_per_a +
                1e-12);
}

TEST(StepperPhaseActuatorTest, PhaseTorqueHasBoundedSinusoidalDirectionAndSymmetry) {
  EXPECT_NEAR(stepper_phase::Actuator::torque_for_phase_error(0.0, 0.45), 0.0, 1e-12);
  EXPECT_NEAR(stepper_phase::Actuator::torque_for_phase_error(kPi / 2.0, 0.45), 0.45, 1e-12);
  EXPECT_NEAR(stepper_phase::Actuator::torque_for_phase_error(-kPi / 2.0, 0.45), -0.45, 1e-12);
  EXPECT_NEAR(stepper_phase::Actuator::torque_for_phase_error(kPi, 0.45), 0.0, 1e-12);
  EXPECT_NEAR(stepper_phase::Actuator::torque_for_phase_error(0.2, 0.45),
              -stepper_phase::Actuator::torque_for_phase_error(-0.2, 0.45), 1e-12);

  const auto phase = stepper_phase::Actuator::phase_for_electrical_angle(kPi / 3.0);
  EXPECT_NEAR(phase.a, 0.5, 1e-12);
  EXPECT_NEAR(phase.b, std::sqrt(3.0) / 2.0, 1e-12);
}

TEST(StepperPhaseActuatorTest, BodyMotionChangesMotorRelativePhaseWithoutSteps) {
  stepper_phase::Actuator actuator(nominal_stepper_parameters());
  actuator.set_commanded_microstep_positions(0.0, 0.0);
  actuator.evaluate(0.01, 0.0, 0.0);
  actuator.advance_mechanical_state(0.01, 0.0, 1.0, 0.0, 1.0);
  const auto output = actuator.evaluate(0.01, 0.0, 1.0);
  EXPECT_NEAR(output.left.actual_relative_mechanical_angle_rad, -0.01, 1e-12);
  EXPECT_LT(output.left.actual_relative_mechanical_angle_rad, 0.0);
  EXPECT_GT(output.left.electrical_phase_error_rad, 0.0);
  EXPECT_GT(output.left.torque_nm, 0.0);
}

TEST(StepperPhaseActuatorTest, PureRollingTranslationChangesRelativeRotorAngle) {
  stepper_phase::Actuator actuator(nominal_stepper_parameters());
  actuator.set_commanded_microstep_positions(0.0, 0.0);
  actuator.evaluate(0.01, 0.0, 0.0);
  actuator.advance_mechanical_state(0.01, 0.10, 0.0, 0.10, 0.0);
  const auto output = actuator.evaluate(0.01, 0.10, 0.0);

  const double expected_angle = 0.10 * 0.01 / actuator.parameters().wheel_radius_m;
  EXPECT_NEAR(output.left.actual_relative_mechanical_angle_rad, expected_angle, 1e-12);
  EXPECT_LT(output.left.electrical_phase_error_rad, 0.0);
  EXPECT_LT(output.left.torque_nm, 0.0);
}

TEST(StepperPhaseModelTest, PositivePhaseTorqueHasCorrectGeneralizedForceSigns) {
  BalancerSimulator::Config config;
  config.physics_profile = PhysicsProfile::StepperPhase;
  config.initial_pitch_deg = 0.0;
  config.initial_pitch_rate_dps = 0.0;
  config.initial_velocity_mps = 0.0;
  config.com_angle_offset_rad = 0.0;

  BalancerSimulator simulator(config);
  // One positive emitted STEP creates a positive field/rotor phase error.
  simulator.set_emitted_motor_steps(1.0, 1.0);
  simulator.step(0.0025);

  EXPECT_GT(simulator.diagnostics().stepper_summed_torque_nm, 0.0);
  EXPECT_GT(simulator.state().velocity, 0.0);
  EXPECT_LT(simulator.state().pitch_rate, 0.0);
}

TEST(StepperPhaseModelTest, SingleOneThirtySecondStepHasAnalyticalTorqueAndAccelerationSigns) {
  BalancerSimulator::Config config;
  config.physics_profile = PhysicsProfile::StepperPhase;
  config.initial_pitch_deg = 1.0;
  config.initial_pitch_rate_dps = 0.0;
  config.initial_velocity_mps = 0.0;
  config.com_angle_offset_rad = 0.0;

  BalancerSimulator simulator(config);
  simulator.set_emitted_motor_steps(1.0, 1.0);
  simulator.step(1e-6);

  const double expected_electrical_step = 2.0 * kPi / 128.0;
  const double expected_motor_torque =
      BalancerSimulator::HardwareNominal::stepper_peak_torque_nm_per_motor *
      std::sin(expected_electrical_step);
  const auto& diagnostics = simulator.diagnostics();
  EXPECT_NEAR(diagnostics.stepper_commanded_field_angle_left_rad,
              0.05625 * kPi / 180.0, 1e-15);
  EXPECT_NEAR(diagnostics.stepper_commanded_field_electrical_angle_left_rad,
              expected_electrical_step, 1e-14);
  EXPECT_NEAR(diagnostics.stepper_electrical_phase_error_left_rad,
              expected_electrical_step, 1e-14);
  EXPECT_NEAR(diagnostics.stepper_torque_left_nm, expected_motor_torque, 1e-12);
  EXPECT_NEAR(diagnostics.stepper_torque_right_nm, expected_motor_torque, 1e-12);
  EXPECT_NEAR(diagnostics.stepper_summed_torque_nm, 2.0 * expected_motor_torque, 1e-12);
  EXPECT_GT(diagnostics.x_ddot, 0.0);
  EXPECT_LT(diagnostics.theta_ddot, 0.0);

  std::cout << "stepper_single_step"
            << " field_mechanical_deg="
            << diagnostics.stepper_commanded_field_angle_left_rad * 180.0 / kPi
            << " field_electrical_deg="
            << diagnostics.stepper_commanded_field_electrical_angle_left_rad * 180.0 / kPi
            << " phase_error_deg="
            << diagnostics.stepper_electrical_phase_error_left_rad * 180.0 / kPi
            << " torque_per_motor_nm=" << diagnostics.stepper_torque_left_nm
            << " total_torque_nm=" << diagnostics.stepper_summed_torque_nm
            << " x_ddot_mps2=" << diagnostics.x_ddot
            << " pitch_ddot_rad_s2=" << diagnostics.theta_ddot << '\n';
}

TEST(StepperPhaseModelTest, DirectTorqueMatchesPhaseTorqueAtMechanicalInterface) {
  BalancerSimulator::Config config;
  config.physics_profile = PhysicsProfile::StepperPhase;
  config.initial_pitch_deg = 0.0;
  config.initial_pitch_rate_dps = 0.0;
  config.initial_velocity_mps = 0.0;
  config.com_angle_offset_rad = 0.0;

  BalancerSimulator phase_simulator(config);
  phase_simulator.set_emitted_motor_steps(1.0, 1.0);
  phase_simulator.step(1e-6);
  const double left_torque = phase_simulator.diagnostics().stepper_torque_left_nm;
  const double right_torque = phase_simulator.diagnostics().stepper_torque_right_nm;

  BalancerSimulator direct_simulator(config);
  direct_simulator.set_stepper_direct_torque_for_test(left_torque, right_torque);
  direct_simulator.step(1e-6);

  EXPECT_NEAR(direct_simulator.diagnostics().x_ddot, phase_simulator.diagnostics().x_ddot,
              1e-10);
  EXPECT_NEAR(direct_simulator.diagnostics().theta_ddot,
              phase_simulator.diagnostics().theta_ddot, 1e-10);
  EXPECT_NEAR(direct_simulator.state().velocity, phase_simulator.state().velocity, 1e-14);
  EXPECT_NEAR(direct_simulator.state().pitch_rate, phase_simulator.state().pitch_rate, 1e-14);
  EXPECT_NEAR(direct_simulator.diagnostics().stepper_torque_left_nm, left_torque, 1e-12);
  EXPECT_NEAR(direct_simulator.diagnostics().stepper_torque_right_nm, right_torque, 1e-12);
}

TEST(StepperPhaseModelTest, MassMatrixUsesAbsoluteWheelRotorKineticCoordinate) {
  using Nominal = BalancerSimulator::HardwareNominal;
  auto physics = BalancerSimulator::physics_for_profile(PhysicsProfile::StepperPhase);
  const auto matrix = BalancerSimulator::stepper_mass_matrix(physics, 0.37);
  const double expected_d11 =
      Nominal::total_mass_kg +
      2.0 * Nominal::stepper_rotating_inertia_kg_m2_per_motor /
          (Nominal::stepper_phase_wheel_radius * Nominal::stepper_phase_wheel_radius);
  const double expected_d12 =
      Nominal::first_mass_moment_kg_m * std::cos(0.37);
  const double expected_d22 = Nominal::pitch_inertia_about_axle_kg_m2;
  EXPECT_NEAR(matrix.d11, expected_d11, 1e-15);
  EXPECT_NEAR(matrix.d12, expected_d12, 1e-15);
  EXPECT_NEAR(matrix.d22, expected_d22, 1e-15);
  EXPECT_GT(matrix.determinant, 0.0);

  auto no_rotating_inertia = physics;
  no_rotating_inertia.stepper_rotating_inertia_kg_m2_per_motor = 0.0;
  const auto without_wheel_inertia =
      BalancerSimulator::stepper_mass_matrix(no_rotating_inertia, 0.37);
  EXPECT_GT(matrix.d11, without_wheel_inertia.d11);
  EXPECT_DOUBLE_EQ(matrix.d12, without_wheel_inertia.d12);
  EXPECT_DOUBLE_EQ(matrix.d22, without_wheel_inertia.d22);

  const double x_dot = 0.27;
  const double theta_dot = -1.4;
  const double explicit_energy =
      0.5 * expected_d11 * x_dot * x_dot + expected_d12 * x_dot * theta_dot +
      0.5 * expected_d22 * theta_dot * theta_dot;
  const double matrix_energy =
      0.5 * (matrix.d11 * x_dot * x_dot +
             2.0 * matrix.d12 * x_dot * theta_dot +
             matrix.d22 * theta_dot * theta_dot);
  EXPECT_NEAR(matrix_energy, explicit_energy, 1e-15);
}

TEST(StepperPhaseModelTest, MassMatrixScalesRemainCorrectAroundNominal) {
  using Nominal = BalancerSimulator::HardwareNominal;
  const auto physics = BalancerSimulator::physics_for_profile(PhysicsProfile::StepperPhase);
  constexpr double pitch_rad = 0.37;
  for (const double scale : {0.9, 1.0, 1.1}) {
    const auto matrix = BalancerSimulator::stepper_mass_matrix(
        physics, pitch_rad, scale, scale, scale);
    const double total_mass = Nominal::total_mass_kg * scale;
    const double first_mass_moment = Nominal::first_mass_moment_kg_m * scale;
    const double pitch_inertia = Nominal::pitch_inertia_about_axle_kg_m2 * scale;
    const double rotating_inertia = Nominal::stepper_rotating_inertia_kg_m2_per_motor;
    const double radius = Nominal::stepper_phase_wheel_radius;
    const double expected_d11 = total_mass + 2.0 * rotating_inertia / (radius * radius);
    const double expected_d12 = first_mass_moment * std::cos(pitch_rad);
    const double expected_d22 = pitch_inertia;
    EXPECT_NEAR(matrix.d11, expected_d11, 1.0e-15);
    EXPECT_NEAR(matrix.d12, expected_d12, 1.0e-15);
    EXPECT_NEAR(matrix.d22, expected_d22, 1.0e-15);
    EXPECT_NEAR(matrix.determinant, expected_d11 * expected_d22 - expected_d12 * expected_d12,
                1.0e-15);
  }
}

TEST(StepperPhaseModelTest, GravityOnlyAccelerationMatchesIndependentBalanceDerivation) {
  auto physics = BalancerSimulator::physics_for_profile(PhysicsProfile::StepperPhase);
  physics.cart_damping = 0.0;
  physics.pitch_damping = 0.0;
  physics.stepper_motor_relative_damping_nm_s_per_rad = 0.0;

  for (const double pitch_deg : {-0.10, 0.10}) {
    BalancerSimulator::Config config;
    config.physics_profile = PhysicsProfile::StepperPhase;
    config.physics_override = physics;
    config.initial_pitch_deg = pitch_deg;
    config.initial_pitch_rate_dps = 0.0;
    config.initial_velocity_mps = 0.0;
    config.com_angle_offset_rad = 0.0;
    BalancerSimulator simulator(config);

    const double pitch_rad = pitch_deg * kPi / 180.0;
    const auto expected = independently_expected_stepper_acceleration(
        physics, pitch_rad, 0.0);
    simulator.step(1.0e-8);

    EXPECT_NEAR(simulator.diagnostics().x_ddot, expected.x_ddot, 1.0e-8) << pitch_deg;
    EXPECT_NEAR(simulator.diagnostics().theta_ddot, expected.pitch_ddot, 1.0e-7)
        << pitch_deg;
    EXPECT_EQ(std::signbit(simulator.diagnostics().x_ddot), std::signbit(expected.x_ddot));
    EXPECT_EQ(std::signbit(simulator.diagnostics().theta_ddot),
              std::signbit(expected.pitch_ddot));
  }
}

TEST(StepperPhaseModelTest, KnownMotorTorqueMatchesIndependentBalanceDerivation) {
  auto physics = BalancerSimulator::physics_for_profile(PhysicsProfile::StepperPhase);
  physics.gravity_mps2 = 0.0;
  physics.cart_damping = 0.0;
  physics.pitch_damping = 0.0;
  physics.stepper_motor_relative_damping_nm_s_per_rad = 0.0;

  for (const double torque_per_motor_nm : {-0.020, 0.020, -0.10, 0.10}) {
    BalancerSimulator::Config config;
    config.physics_profile = PhysicsProfile::StepperPhase;
    config.physics_override = physics;
    config.initial_pitch_deg = 0.0;
    config.initial_pitch_rate_dps = 0.0;
    config.initial_velocity_mps = 0.0;
    config.com_angle_offset_rad = 0.0;
    BalancerSimulator simulator(config);
    simulator.set_stepper_direct_torque_for_test(torque_per_motor_nm, torque_per_motor_nm);

    const auto expected = independently_expected_stepper_acceleration(
        physics, 0.0, 2.0 * torque_per_motor_nm);
    simulator.step(1.0e-8);

    EXPECT_NEAR(simulator.diagnostics().x_ddot, expected.x_ddot, 1.0e-8)
        << torque_per_motor_nm;
    EXPECT_NEAR(simulator.diagnostics().theta_ddot, expected.pitch_ddot, 1.0e-7)
        << torque_per_motor_nm;
    EXPECT_EQ(std::signbit(simulator.diagnostics().x_ddot), std::signbit(expected.x_ddot));
    EXPECT_EQ(std::signbit(simulator.diagnostics().theta_ddot),
              std::signbit(expected.pitch_ddot));
  }
}

TEST(StepperPhaseModelTest, QuasiStaticLeanTorqueHasExpectedAngularEquilibrium) {
  using Nominal = BalancerSimulator::HardwareNominal;
  auto physics = BalancerSimulator::physics_for_profile(PhysicsProfile::StepperPhase);
  physics.cart_damping = 0.0;
  physics.pitch_damping = 0.0;
  physics.stepper_motor_relative_damping_nm_s_per_rad = 0.0;

  const double available_total_torque = 2.0 * Nominal::stepper_peak_torque_nm_per_motor;
  for (const double pitch_deg : {0.5, 1.0, 2.0, 4.0}) {
    const double pitch_rad = pitch_deg * kPi / 180.0;
    const auto matrix = BalancerSimulator::stepper_mass_matrix(physics, pitch_rad);
    // This is the constant-pitch (theta_ddot=0) solution of the two
    // generalized equations. It is intentionally quasi-static: the wheel
    // coordinate accelerates while the body angle remains fixed.
    const double required_total_torque =
        Nominal::gravity * Nominal::first_mass_moment_kg_m * std::sin(pitch_rad) /
        (1.0 + matrix.d12 / (Nominal::stepper_phase_wheel_radius * matrix.d11));
    ASSERT_LT(std::abs(required_total_torque), available_total_torque);

    BalancerSimulator::Config config;
    config.physics_profile = PhysicsProfile::StepperPhase;
    config.physics_override = physics;
    config.initial_pitch_deg = pitch_deg;
    config.initial_pitch_rate_dps = 0.0;
    config.initial_velocity_mps = 0.0;
    config.com_angle_offset_rad = 0.0;
    BalancerSimulator simulator(config);
    simulator.set_stepper_direct_torque_for_test(0.5 * required_total_torque,
                                                 0.5 * required_total_torque);
    simulator.step(1.0e-8);

    const double expected_x_ddot = required_total_torque /
                                   (Nominal::stepper_phase_wheel_radius * matrix.d11);
    EXPECT_NEAR(simulator.diagnostics().theta_ddot, 0.0, 1.0e-7) << pitch_deg;
    EXPECT_NEAR(simulator.diagnostics().x_ddot, expected_x_ddot, 1.0e-7) << pitch_deg;

    for (int sample = 0; sample < 1000; ++sample) {
      simulator.step(1.0e-5);
    }
    EXPECT_NEAR(simulator.state().pitch, pitch_rad, 1.0e-8) << pitch_deg;
    std::cout << "stepper_quasi_static_lean pitch_deg=" << pitch_deg
              << " required_total_torque_nm=" << required_total_torque
              << " required_force_n=" << required_total_torque /
                                               Nominal::stepper_phase_wheel_radius
              << " available_total_torque_nm=" << available_total_torque
              << " x_ddot_mps2=" << expected_x_ddot << '\n';
  }
}

TEST(StepperPhaseModelTest, InitialStepperReleaseSatisfiesNoSlipPhaseCoordinate) {
  BalancerSimulator::Config config;
  config.physics_profile = PhysicsProfile::StepperPhaseElectrical;
  config.initial_pitch_deg = 3.0;
  config.initial_pitch_rate_dps = 0.0;
  config.initial_velocity_mps = 0.0;
  config.com_angle_offset_rad = 0.0;
  BalancerSimulator simulator(config);
  simulator.step(0.0);

  const double pitch = simulator.state().pitch;
  const double radius = BalancerSimulator::HardwareNominal::stepper_phase_wheel_radius;
  EXPECT_NEAR(simulator.state().position / radius - pitch, 0.0, 1e-15);
  EXPECT_NEAR(
      simulator.stepper_phase_electrical_output().left.actual_relative_mechanical_angle_rad,
      0.0, 1e-15);
}

double ideal_stepper_mechanical_energy(const BalancerSimulator& simulator) {
  const auto& state = simulator.state();
  const auto& physics = simulator.physics();
  const auto matrix = BalancerSimulator::stepper_mass_matrix(
      physics, state.pitch + simulator.config().com_angle_offset_rad);
  return 0.5 * (matrix.d11 * state.velocity * state.velocity +
                2.0 * matrix.d12 * state.velocity * state.pitch_rate +
                matrix.d22 * state.pitch_rate * state.pitch_rate);
}

double ideal_stepper_magnetic_potential(const BalancerSimulator& simulator) {
  using Nominal = BalancerSimulator::HardwareNominal;
  const double magnetic_scale = Nominal::stepper_peak_torque_nm_per_motor;
  const double electrical_cycles = Nominal::stepper_phase_electrical_cycles_per_rev;
  const auto& output = simulator.stepper_phase_output();
  return -magnetic_scale / electrical_cycles *
         (std::cos(output.left.electrical_phase_error_rad) +
          std::cos(output.right.electrical_phase_error_rad));
}

double ideal_stepper_total_energy(const BalancerSimulator& simulator) {
  return ideal_stepper_mechanical_energy(simulator) +
         ideal_stepper_magnetic_potential(simulator);
}

TEST(StepperPhaseEnergyTest, GravityDisabledCoupledPlantIsConservativeAndSymmetric) {
  auto make_simulator = [](double signed_phase_deg, double damping) {
    BalancerSimulator::Config config;
    config.physics_profile = PhysicsProfile::StepperPhase;
    config.initial_pitch_deg = 0.0;
    config.initial_pitch_rate_dps = 0.0;
    config.initial_velocity_mps = 0.0;
    config.com_angle_offset_rad = 0.0;
    auto physics = BalancerSimulator::physics_for_profile(PhysicsProfile::StepperPhase);
    physics.gravity_mps2 = 0.0;
    physics.cart_damping = 0.0;
    physics.pitch_damping = 0.0;
    physics.stepper_motor_relative_damping_nm_s_per_rad = damping;
    config.physics_override = physics;
    BalancerSimulator simulator(config);
    simulator.set_emitted_motor_steps(0.0, 0.0);
    const double phase_rad = signed_phase_deg * kPi / 180.0;
    simulator.set_stepper_relative_angles_for_test(
        -phase_rad / BalancerSimulator::HardwareNominal::stepper_phase_electrical_cycles_per_rev,
        -phase_rad / BalancerSimulator::HardwareNominal::stepper_phase_electrical_cycles_per_rev);
    simulator.step(0.0);
    return simulator;
  };

  auto positive = make_simulator(5.0, 0.0);
  auto negative = make_simulator(-5.0, 0.0);
  const double initial_energy = ideal_stepper_total_energy(positive);
  double maximum_energy_error = 0.0;
  constexpr double dt_s = 25.0e-6;
  for (int sample = 0; sample < 4000; ++sample) {
    positive.step(dt_s);
    negative.step(dt_s);
    maximum_energy_error = std::max(
        maximum_energy_error, std::abs(ideal_stepper_total_energy(positive) - initial_energy));
  }

  EXPECT_LT(maximum_energy_error, 2.0e-5);
  EXPECT_NEAR(positive.state().pitch, -negative.state().pitch, 2.0e-10);
  EXPECT_NEAR(positive.state().pitch_rate, -negative.state().pitch_rate, 2.0e-9);
  EXPECT_NEAR(positive.state().velocity, -negative.state().velocity, 2.0e-10);
  EXPECT_NEAR(ideal_stepper_total_energy(positive), ideal_stepper_total_energy(negative),
              2.0e-5);

  auto damped = make_simulator(5.0, 0.01);
  const double damped_initial_energy = ideal_stepper_total_energy(damped);
  double previous_energy = damped_initial_energy;
  int energy_increases = 0;
  for (int sample = 0; sample < 4000; ++sample) {
    damped.step(dt_s);
    const double energy = ideal_stepper_total_energy(damped);
    if (energy > previous_energy + 1.0e-8) ++energy_increases;
    previous_energy = energy;
  }
  EXPECT_LT(ideal_stepper_total_energy(damped), damped_initial_energy);
  EXPECT_LT(energy_increases, 20);
}

TEST(StepperPhaseModelTest, FixedPhaseTorqueSweepMatchesSineAndMechanicalSigns) {
  BalancerSimulator::Config config;
  config.physics_profile = PhysicsProfile::StepperPhase;
  config.initial_pitch_deg = 0.0;
  config.initial_pitch_rate_dps = 0.0;
  config.initial_velocity_mps = 0.0;
  config.com_angle_offset_rad = 0.0;

  for (const double phase_error_deg : {-2.8125, -5.625, -11.25, -22.5, -45.0, -90.0,
                                       2.8125, 5.625, 11.25, 22.5, 45.0, 90.0}) {
    const double phase_error_rad = phase_error_deg * kPi / 180.0;
    const double expected_torque =
        BalancerSimulator::HardwareNominal::stepper_peak_torque_nm_per_motor *
        std::sin(phase_error_rad);
    BalancerSimulator simulator(config);
    simulator.set_stepper_direct_torque_for_test(expected_torque, expected_torque);
    simulator.step(1e-6);

    EXPECT_NEAR(simulator.diagnostics().stepper_torque_left_nm, expected_torque, 1e-12);
    EXPECT_NEAR(simulator.diagnostics().stepper_torque_right_nm, expected_torque, 1e-12);
    EXPECT_NEAR(simulator.diagnostics().stepper_summed_torque_nm, 2.0 * expected_torque,
                1e-12);
    if (phase_error_deg > 0.0) {
      EXPECT_GT(simulator.diagnostics().x_ddot, 0.0);
      EXPECT_LT(simulator.diagnostics().theta_ddot, 0.0);
    } else {
      EXPECT_LT(simulator.diagnostics().x_ddot, 0.0);
      EXPECT_GT(simulator.diagnostics().theta_ddot, 0.0);
    }
    std::cout << "stepper_fixed_phase phase_error_deg=" << phase_error_deg
              << " expected_torque_per_motor_nm=" << expected_torque
              << " actual_torque_per_motor_nm="
              << simulator.diagnostics().stepper_torque_left_nm
              << " x_ddot_mps2=" << simulator.diagnostics().x_ddot
              << " pitch_ddot_rad_s2=" << simulator.diagnostics().theta_ddot << '\n';
  }
}

TEST(StepperPhaseModelTest, GravityMomentAndAvailableTorqueAreExplicitlyCompared) {
  const double electrical_step =
      BalancerSimulator::HardwareNominal::stepper_phase_electrical_radians_per_step;
  const double one_step_torque_per_motor =
      BalancerSimulator::HardwareNominal::stepper_peak_torque_nm_per_motor *
      std::sin(electrical_step);
  const double two_step_torque_per_motor =
      BalancerSimulator::HardwareNominal::stepper_peak_torque_nm_per_motor *
      std::sin(2.0 * electrical_step);
  const double four_step_torque_per_motor =
      BalancerSimulator::HardwareNominal::stepper_peak_torque_nm_per_motor *
      std::sin(4.0 * electrical_step);
  const double max_total_torque =
      2.0 * BalancerSimulator::HardwareNominal::stepper_peak_torque_nm_per_motor;

  for (const double pitch_deg : {1.0, 2.0, 4.0, 6.0}) {
    const double gravity_moment =
        BalancerSimulator::HardwareNominal::gravity *
        BalancerSimulator::HardwareNominal::first_mass_moment_kg_m *
        std::sin(pitch_deg * kPi / 180.0);
    EXPECT_GT(max_total_torque, gravity_moment);
    std::cout << "stepper_authority pitch_deg=" << pitch_deg
              << " gravity_moment_nm=" << gravity_moment
              << " one_step_total_torque_nm=" << 2.0 * one_step_torque_per_motor
              << " two_step_total_torque_nm=" << 2.0 * two_step_torque_per_motor
              << " four_step_total_torque_nm=" << 2.0 * four_step_torque_per_motor
              << " max_total_torque_nm=" << max_total_torque << '\n';
  }
}

TEST(StepperPhaseModelTest, DampingOffPreservesInstantaneousTorqueSigns) {
  BalancerSimulator::Config config;
  config.physics_profile = PhysicsProfile::StepperPhase;
  config.initial_pitch_deg = 1.0;
  config.initial_pitch_rate_dps = 0.0;
  config.initial_velocity_mps = 0.0;
  config.com_angle_offset_rad = 0.0;
  auto no_damping = BalancerSimulator::physics_for_profile(PhysicsProfile::StepperPhase);
  no_damping.cart_damping = 0.0;
  no_damping.pitch_damping = 0.0;
  config.physics_override = no_damping;

  BalancerSimulator simulator(config);
  simulator.set_emitted_motor_steps(1.0, 1.0);
  simulator.step(1e-6);
  EXPECT_GT(simulator.diagnostics().x_ddot, 0.0);
  EXPECT_LT(simulator.diagnostics().theta_ddot, 0.0);
  EXPECT_GT(simulator.diagnostics().stepper_summed_torque_nm, 0.0);
}

TEST(StepperPhaseActuatorTest, CommandDoesNotForceRotorToFollowAnAggressiveField) {
  stepper_phase::Actuator actuator(nominal_stepper_parameters());
  actuator.set_commanded_microstep_positions(12000.0, 12000.0);
  const auto output = actuator.evaluate(0.001, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(output.left.actual_relative_mechanical_angle_rad, 0.0);
  EXPECT_DOUBLE_EQ(output.right.actual_relative_mechanical_angle_rad, 0.0);
  EXPECT_LE(std::abs(output.summed_torque_nm), 0.90 + 1e-12);
}

TEST(StepperPhaseActuatorTest, ConstantStepRateIsFieldMotionNotAnInjectedForce) {
  stepper_phase::Actuator actuator(nominal_stepper_parameters());
  constexpr double dt_s = 0.001;
  constexpr double steps_per_second = 8000.0;
  double commanded_steps = 0.0;
  double maximum_torque = 0.0;
  for (int sample = 0; sample < 100; ++sample) {
    commanded_steps += steps_per_second * dt_s;
    actuator.set_commanded_microstep_positions(commanded_steps, commanded_steps);
    // If the rotor has caught the field, continuing to issue STEP pulses does
    // not itself create a force. The phase model is the only torque source.
    actuator.set_actual_relative_angles_for_test(
        commanded_steps * actuator.mechanical_radians_per_step(),
        commanded_steps * actuator.mechanical_radians_per_step());
    const auto output = actuator.evaluate(dt_s, 0.0, 0.0);
    maximum_torque = std::max(maximum_torque, std::abs(output.summed_torque_nm));
    EXPECT_NEAR(output.left.commanded_field_velocity_mps,
                steps_per_second * actuator.meters_per_step(), 1e-12);
    EXPECT_NEAR(output.summed_torque_nm, 0.0, 1e-10);
  }
  EXPECT_LE(maximum_torque, 0.90 + 1e-12);
}

TEST(StepperPhaseModelTest, RecoveryStatesExposeFieldRotorAndMechanicalVelocitySeparately) {
  ScopedPidValues restore;
  ConfigPid::load((std::filesystem::path(BALANCER_REPO_ROOT) / "pid.conf").string());
  ConfigPid::values.velocity_damping_per_s = 0.0;
  ConfigPid::values.velocity_I = 0.0;
  ConfigPid::values.velocity_pitch_limit_deg = 0.0;
  ConfigPid::values.pitch_rate_gain = 350.0;
  ConfigPid::values.pitch_accel_gain = 0.0;

  struct State {
    const char* name;
    double pitch_deg;
    double pitch_rate_dps;
    double velocity_sps;
  };
  const std::array<State, 12> states = {{
      {"small", 1.0, 0.0, 0.0},
      {"small_negative", -1.0, 0.0, 0.0},
      {"moderate_rate", 2.0, 20.0, 0.0},
      {"moderate_rate_negative", -2.0, -20.0, 0.0},
      {"pitch4_speed500", 4.0, 0.0, 500.0},
      {"pitch4_speed_negative", -4.0, 0.0, -500.0},
      {"pitch6_speed1000", 6.0, 0.0, 1000.0},
      {"pitch6_speed_negative", -6.0, 0.0, -1000.0},
      {"state_a", 4.3, 0.0, 964.0},
      {"state_a_negative", -4.3, 0.0, -964.0},
      {"state_b", 9.0, 0.0, 1800.0},
      {"state_b_negative", -9.0, 0.0, -1800.0},
  }};

  for (const double pitch_gain : {6000.0, 8000.0}) {
    ConfigPid::values.pitch_gain = pitch_gain;
    for (const auto& state : states) {
        const auto result = run_simulator_scenario_with_loaded_pid(
            stepper_recovery_scenario(state.pitch_deg, state.pitch_rate_dps, state.velocity_sps));
        ASSERT_FALSE(result.rows.empty());
      double max_torque = 0.0;
      double max_phase_error = 0.0;
      double max_velocity_difference = 0.0;
      for (const auto& row : result.rows) {
        max_torque = std::max(max_torque, std::abs(row.stepper_summed_torque_nm));
        max_phase_error = std::max(
            max_phase_error,
            std::max(std::abs(row.stepper_electrical_phase_error_left_rad),
                     std::abs(row.stepper_electrical_phase_error_right_rad)));
        max_velocity_difference = std::max(
            max_velocity_difference,
            std::abs(row.stepper_commanded_field_velocity_mps -
                     row.stepper_actual_wheel_velocity_mps));
      }
      EXPECT_LE(max_torque, 0.90 + 1e-9);
      EXPECT_TRUE(std::isfinite(max_phase_error));
      EXPECT_TRUE(std::isfinite(max_velocity_difference));
      std::cout << "stepper_recovery kp=" << pitch_gain << " state=" << state.name
                << " fell=" << result.fell << " peak_pitch_deg=" << result.max_abs_pitch_deg
                << " tail_rms_deg=" << result.tail_rms_pitch_deg
                << " max_torque_nm=" << max_torque
                << " max_phase_error_rad=" << max_phase_error
                << " max_field_wheel_velocity_difference_mps=" << max_velocity_difference
                << " final_chassis_velocity_sps="
                << result.rows.back().stepper_chassis_velocity_mps /
                       BalancerSimulator::HardwareNominal::stepper_phase_meters_per_step
                << '\n';
    }
  }
}

TEST(StepperPhaseModelTest, ContinuousAndDiscreteFieldRecoveryAreReportedSeparately) {
  ScopedPidValues restore;
  ConfigPid::load((std::filesystem::path(BALANCER_REPO_ROOT) / "pid.conf").string());
  ConfigPid::values.velocity_damping_per_s = 0.0;
  ConfigPid::values.velocity_I = 0.0;
  ConfigPid::values.velocity_pitch_limit_deg = 0.0;
  ConfigPid::values.pitch_gain = 8000.0;
  ConfigPid::values.pitch_rate_gain = 350.0;
  ConfigPid::values.pitch_accel_gain = 0.0;

  for (const double pitch_deg : {1.0, -1.0}) {
    const auto discrete = run_simulator_scenario_with_loaded_pid(
        stepper_recovery_scenario(pitch_deg, 0.0, 0.0, false));
    const auto continuous = run_simulator_scenario_with_loaded_pid(
        stepper_recovery_scenario(pitch_deg, 0.0, 0.0, true));
    ASSERT_FALSE(discrete.rows.empty());
    ASSERT_FALSE(continuous.rows.empty());

    auto max_phase = [](const SimulatorRunResult& result) {
      double value = 0.0;
      for (const auto& row : result.rows) {
        value = std::max(value,
                         std::max(std::abs(row.stepper_electrical_phase_error_left_rad),
                                  std::abs(row.stepper_electrical_phase_error_right_rad)));
      }
      return value;
    };
    std::cout << "stepper_field_compare pitch_deg=" << pitch_deg
              << " discrete_fell=" << discrete.fell
              << " continuous_fell=" << continuous.fell
              << " discrete_peak_phase_rad=" << max_phase(discrete)
              << " continuous_peak_phase_rad=" << max_phase(continuous)
              << " discrete_peak_pitch_deg=" << discrete.max_abs_pitch_deg
              << " continuous_peak_pitch_deg=" << continuous.max_abs_pitch_deg << '\n';
  }
}

TEST(StepperPhaseModelTest, FirstHundredMillisecondsExposeCausalResponseTrace) {
  ScopedPidValues restore;
  ConfigPid::load((std::filesystem::path(BALANCER_REPO_ROOT) / "pid.conf").string());
  ConfigPid::values.velocity_damping_per_s = 0.0;
  ConfigPid::values.velocity_I = 0.0;
  ConfigPid::values.velocity_pitch_limit_deg = 0.0;
  ConfigPid::values.pitch_gain = 8000.0;
  ConfigPid::values.pitch_rate_gain = 350.0;
  ConfigPid::values.pitch_accel_gain = 0.0;

  auto scenario = stepper_recovery_scenario(1.0, 0.0, 0.0);
  scenario.duration_s = 0.100;
  const auto result = run_simulator_scenario_with_loaded_pid(scenario);
  ASSERT_FALSE(result.rows.empty());

  std::cout << "stepper_causal_trace columns=time_s,pitch_deg,pitch_rate_dps,fused_pitch_deg,"
                "pitch_error_deg,pitch_feedback_sps,rate_feedback_sps,left_sps,"
                "field_mechanical_rad,field_electrical_rad,relative_mechanical_rad,"
                "rotor_electrical_rad,phase_error_rad,torque_nm,x_ddot_mps2,pitch_ddot_rad_s2"
             << '\n';
  for (const auto& row : result.rows) {
    std::cout << "stepper_causal_trace row=" << row.sim_time_s << ',' << row.plant_pitch_deg
              << ',' << row.plant_pitch_rate_dps << ',' << row.fused_pitch_deg << ','
              << row.pitch_error_deg << ',' << row.pitch_feedback_sps << ','
              << row.pitch_rate_feedback_sps << ',' << row.left_sps << ','
              << row.stepper_commanded_field_angle_left_rad << ','
              << row.stepper_commanded_field_electrical_angle_left_rad << ','
              << row.stepper_actual_relative_angle_left_rad << ','
              << row.stepper_actual_rotor_electrical_angle_left_rad << ','
              << row.stepper_electrical_phase_error_left_rad << ','
              << row.stepper_torque_left_nm << ',' << row.x_ddot << ',' << row.theta_ddot
              << '\n';
  }
}

TEST(StepperPhaseModelTest, RecoveryLadderReportsTheFirstFailingActuatorLayer) {
  ScopedPidValues restore;
  ConfigPid::load((std::filesystem::path(BALANCER_REPO_ROOT) / "pid.conf").string());
  ConfigPid::values.velocity_damping_per_s = 0.0;
  ConfigPid::values.velocity_I = 0.0;
  ConfigPid::values.velocity_pitch_limit_deg = 0.0;
  ConfigPid::values.pitch_gain = 8000.0;
  ConfigPid::values.pitch_rate_gain = 350.0;
  ConfigPid::values.pitch_accel_gain = 0.0;

  auto ideal = stepper_recovery_scenario(1.0, 0.0, 0.0);
  ideal.physics_profile = PhysicsProfile::DirectActuator;
  auto continuous = stepper_recovery_scenario(1.0, 0.0, 0.0, true);
  auto discrete = stepper_recovery_scenario(1.0, 0.0, 0.0, false);
  const auto ideal_result = run_simulator_scenario_with_loaded_pid(ideal);
  const auto continuous_result = run_simulator_scenario_with_loaded_pid(continuous);
  const auto discrete_result = run_simulator_scenario_with_loaded_pid(discrete);

  // Layer B is intentionally represented by the fixed-phase analytical torque
  // sweep above: it validates phase -> torque -> generalized force, but has no
  // field trajectory and therefore is not a closed-loop recovery scenario.
  std::cout << "stepper_recovery_ladder layer=A_direct_force fell=" << ideal_result.fell
            << " peak_pitch_deg=" << ideal_result.max_abs_pitch_deg << '\n';
  std::cout << "stepper_recovery_ladder layer=B_fixed_phase status=analytical_only\n";
  std::cout << "stepper_recovery_ladder layer=C_continuous_field fell="
            << continuous_result.fell
            << " peak_pitch_deg=" << continuous_result.max_abs_pitch_deg << '\n';
  std::cout << "stepper_recovery_ladder layer=D_discrete_stepper_phase fell="
            << discrete_result.fell
            << " peak_pitch_deg=" << discrete_result.max_abs_pitch_deg << '\n';
}

TEST(StepperPhaseModelTest, ReferenceProfilesAndCompletedStepVelocityAreCompared) {
  ScopedPidValues restore;
  ConfigPid::load((std::filesystem::path(BALANCER_REPO_ROOT) / "pid.conf").string());
  ConfigPid::values.velocity_damping_per_s = 0.0;
  ConfigPid::values.velocity_I = 0.0;
  ConfigPid::values.velocity_pitch_limit_deg = 0.0;
  ConfigPid::values.pitch_rate_gain = 350.0;
  ConfigPid::values.pitch_accel_gain = 0.0;

  const std::array<const char*, 3> profile_names = {
      "direct_actuator", "retired_no_slip_alias", "stepper_phase"};
  const std::array<PhysicsProfile, 3> profiles = {
      PhysicsProfile::DirectActuator, PhysicsProfile::RetiredNoSlipActuator,
      PhysicsProfile::StepperPhase};
  for (const double pitch_gain : {6000.0, 8000.0}) {
    ConfigPid::values.pitch_gain = pitch_gain;
    for (std::size_t profile_index = 0; profile_index < profiles.size(); ++profile_index) {
      for (const auto& state : std::array<std::array<double, 3>, 2>{
               {{{4.3, 0.0, 964.0}}, {{9.0, 0.0, 1800.0}}}}) {
        auto scenario = stepper_recovery_scenario(state[0], state[1], state[2]);
        scenario.physics_profile = profiles[profile_index];
        const auto result = run_simulator_scenario_with_loaded_pid(scenario);
        ASSERT_FALSE(result.rows.empty());
        double max_completed_velocity_difference = 0.0;
        double max_phase_error = 0.0;
        double max_field_velocity_difference = 0.0;
        double max_chassis_velocity_sps = 0.0;
        double completed_vs_wheel_squared_error = 0.0;
        std::vector<double> completed_velocity_mps;
        std::vector<double> wheel_velocity_mps;
        completed_velocity_mps.reserve(result.rows.size());
        wheel_velocity_mps.reserve(result.rows.size());
        for (const auto& row : result.rows) {
          const double completed_velocity =
              row.raw_completed_velocity_sps *
              BalancerSimulator::HardwareNominal::stepper_phase_meters_per_step;
          const double wheel_velocity = row.stepper_actual_wheel_velocity_mps;
          completed_velocity_mps.push_back(completed_velocity);
          wheel_velocity_mps.push_back(wheel_velocity);
          max_completed_velocity_difference =
              std::max(max_completed_velocity_difference,
                       std::abs(completed_velocity - wheel_velocity));
          completed_vs_wheel_squared_error +=
              (completed_velocity - wheel_velocity) * (completed_velocity - wheel_velocity);
          max_phase_error = std::max(
              max_phase_error,
              std::max(std::abs(row.stepper_electrical_phase_error_left_rad),
                       std::abs(row.stepper_electrical_phase_error_right_rad)));
          max_field_velocity_difference = std::max(
              max_field_velocity_difference,
              std::abs(row.stepper_commanded_field_velocity_mps -
                       row.stepper_actual_wheel_velocity_mps));
          max_chassis_velocity_sps = std::max(
              max_chassis_velocity_sps,
                  std::abs(row.plant_velocity) /
                  BalancerSimulator::HardwareNominal::stepper_phase_meters_per_step);
        }
        const double completed_vs_wheel_rms =
            std::sqrt(completed_vs_wheel_squared_error /
                      static_cast<double>(result.rows.size()));

        // These are transient recovery traces, so a sinusoidal phase is not a
        // meaningful single number. Report the best zero-mean cross-correlation
        // lag over a short diagnostic window instead. Positive lag means the
        // wheel trace is compared against a later completed-step sample.
        int best_lag_samples = 0;
        double best_correlation = -1.0;
        constexpr int kMaxLagSamples = 50;
        for (int lag = -kMaxLagSamples; lag <= kMaxLagSamples; ++lag) {
          const std::size_t first_completed = lag < 0 ? static_cast<std::size_t>(-lag) : 0;
          const std::size_t first_wheel = lag > 0 ? static_cast<std::size_t>(lag) : 0;
          if (first_completed >= completed_velocity_mps.size() ||
              first_wheel >= wheel_velocity_mps.size()) {
            continue;
          }
          const std::size_t count = std::min(completed_velocity_mps.size() - first_completed,
                                             wheel_velocity_mps.size() - first_wheel);
          if (count < 2) {
            continue;
          }
          double completed_mean = 0.0;
          double wheel_mean = 0.0;
          for (std::size_t i = 0; i < count; ++i) {
            completed_mean += completed_velocity_mps[first_completed + i];
            wheel_mean += wheel_velocity_mps[first_wheel + i];
          }
          completed_mean /= static_cast<double>(count);
          wheel_mean /= static_cast<double>(count);
          double numerator = 0.0;
          double completed_energy = 0.0;
          double wheel_energy = 0.0;
          for (std::size_t i = 0; i < count; ++i) {
            const double completed_delta =
                completed_velocity_mps[first_completed + i] - completed_mean;
            const double wheel_delta = wheel_velocity_mps[first_wheel + i] - wheel_mean;
            numerator += completed_delta * wheel_delta;
            completed_energy += completed_delta * completed_delta;
            wheel_energy += wheel_delta * wheel_delta;
          }
          const double denominator = std::sqrt(completed_energy * wheel_energy);
          if (denominator > 1e-12) {
            const double correlation = numerator / denominator;
            if (correlation > best_correlation) {
              best_correlation = correlation;
              best_lag_samples = lag;
            }
          }
        }
        const double sample_dt_s = result.rows.size() > 1
                                       ? result.rows[1].sim_time_s - result.rows[0].sim_time_s
                                       : 0.0;
        std::cout << "stepper_profile_compare kp=" << pitch_gain
                  << " profile=" << profile_names[profile_index]
                  << " state_pitch_deg=" << state[0] << " state_velocity_sps=" << state[2]
                  << " fell=" << result.fell << " peak_pitch_deg=" << result.max_abs_pitch_deg
                  << " final_velocity_sps="
                  << result.rows.back().plant_velocity /
                         BalancerSimulator::HardwareNominal::stepper_phase_meters_per_step
                  << " max_chassis_velocity_sps=" << max_chassis_velocity_sps
                  << " max_phase_error_rad=" << max_phase_error
                  << " max_field_vs_wheel_velocity_mps=" << max_field_velocity_difference
                  << " max_completed_vs_wheel_velocity_mps="
                  << max_completed_velocity_difference
                  << " rms_completed_vs_wheel_velocity_mps=" << completed_vs_wheel_rms
                  << " best_completed_wheel_lag_ms="
                  << best_lag_samples * sample_dt_s * 1000.0
                  << " best_completed_wheel_correlation=" << best_correlation << '\n';
      }
    }
  }
}

TEST(StepperPhaseElectricalModelTest, RecoveryLadderAndVelocityDivergenceAreReported) {
  ScopedPidValues restore;
  ConfigPid::load((std::filesystem::path(BALANCER_REPO_ROOT) / "pid.conf").string());
  ConfigPid::values.velocity_damping_per_s = 0.0;
  ConfigPid::values.velocity_I = 0.0;
  ConfigPid::values.velocity_pitch_limit_deg = 0.0;
  ConfigPid::values.pitch_rate_gain = 350.0;
  ConfigPid::values.pitch_accel_gain = 0.0;

  const std::array<const char*, 3> profile_names = {
      "direct_actuator", "stepper_phase_ideal_current", "stepper_phase_electrical"};
  const std::array<PhysicsProfile, 3> profiles = {
      PhysicsProfile::DirectActuator, PhysicsProfile::StepperPhase,
      PhysicsProfile::StepperPhaseElectrical};
  const std::array<double, 4> pitch_cases = {1.0, 2.0, 4.0, 6.0};
  bool electrical_both_one_degree_recovered = true;

  for (const double pitch_gain : {6000.0, 8000.0}) {
    ConfigPid::values.pitch_gain = pitch_gain;
    for (std::size_t profile_index = 0; profile_index < profiles.size(); ++profile_index) {
      for (const double pitch_deg : pitch_cases) {
        for (const double sign : {-1.0, 1.0}) {
          auto scenario = stepper_recovery_scenario(sign * pitch_deg, 0.0, 0.0);
          scenario.physics_profile = profiles[profile_index];
          const auto result = run_simulator_scenario_with_loaded_pid(scenario);
          ASSERT_FALSE(result.rows.empty());

          double max_current_error = 0.0;
          double max_phase_error = 0.0;
          double max_field_wheel_velocity_error = 0.0;
          double max_torque = 0.0;
          int voltage_saturated_samples = 0;
          for (const auto& row : result.rows) {
            max_current_error = std::max(
                max_current_error,
                std::max(std::abs(row.stepper_current_ref_a_left - row.stepper_current_a_left) +
                             std::abs(row.stepper_current_ref_b_left - row.stepper_current_b_left),
                         std::abs(row.stepper_current_ref_a_right - row.stepper_current_a_right) +
                             std::abs(row.stepper_current_ref_b_right - row.stepper_current_b_right)));
            max_phase_error = std::max(
                max_phase_error,
                std::max(std::abs(row.stepper_electrical_phase_error_left_rad),
                         std::abs(row.stepper_electrical_phase_error_right_rad)));
            max_field_wheel_velocity_error = std::max(
                max_field_wheel_velocity_error,
                std::abs(row.stepper_commanded_field_velocity_mps -
                         row.stepper_actual_wheel_velocity_mps));
            max_torque = std::max(max_torque,
                                  std::max(std::abs(row.stepper_torque_left_nm),
                                           std::abs(row.stepper_torque_right_nm)));
            if (row.stepper_voltage_saturated_left > 0.5 ||
                row.stepper_voltage_saturated_right > 0.5) {
              ++voltage_saturated_samples;
            }
          }
          EXPECT_TRUE(std::isfinite(max_current_error));
          EXPECT_TRUE(std::isfinite(max_phase_error));
          EXPECT_TRUE(std::isfinite(max_field_wheel_velocity_error));
          EXPECT_LE(max_torque,
                    BalancerSimulator::HardwareNominal::stepper_peak_torque_nm_per_motor +
                        1e-9);
          if (profiles[profile_index] == PhysicsProfile::StepperPhaseElectrical &&
              pitch_deg == 1.0) {
            electrical_both_one_degree_recovered &= !result.fell;
          }
          std::cout << "stepper_electrical_recovery kp=" << pitch_gain
                    << " profile=" << profile_names[profile_index]
                    << " pitch_deg=" << sign * pitch_deg << " fell=" << result.fell
                    << " peak_pitch_deg=" << result.max_abs_pitch_deg
                    << " tail_rms_deg=" << result.tail_rms_pitch_deg
                    << " max_current_error_a=" << max_current_error
                    << " max_phase_error_rad=" << max_phase_error
                    << " max_field_wheel_velocity_error_mps="
                    << max_field_wheel_velocity_error
                    << " max_torque_nm_per_motor=" << max_torque
                    << " voltage_saturated_fraction="
                    << static_cast<double>(voltage_saturated_samples) /
                           static_cast<double>(result.rows.size())
                    << '\n';
        }
      }
    }
  }

  // Hardware-like states are intentionally only interpreted if the new
  // profile first demonstrates ordinary one-degree recovery.  Otherwise the
  // failure is already a lower-layer diagnostic and the larger states would
  // not add a meaningful controller comparison.
  if (electrical_both_one_degree_recovered) {
    for (const double pitch_gain : {6000.0, 8000.0}) {
      ConfigPid::values.pitch_gain = pitch_gain;
      for (const auto& state : std::array<std::array<double, 3>, 2>{
               {{{4.3, 0.0, 964.0}}, {{9.0, 0.0, 1800.0}}}}) {
        auto scenario = stepper_recovery_scenario(state[0], 0.0, state[2]);
        scenario.physics_profile = PhysicsProfile::StepperPhaseElectrical;
        const auto result = run_simulator_scenario_with_loaded_pid(scenario);
        std::cout << "stepper_electrical_hardware_state kp=" << pitch_gain
                  << " pitch_deg=" << state[0] << " velocity_sps=" << state[2]
                  << " fell=" << result.fell << " peak_pitch_deg=" << result.max_abs_pitch_deg
                  << " final_velocity_sps="
                  << result.rows.back().plant_velocity /
                         BalancerSimulator::HardwareNominal::stepper_phase_meters_per_step
                  << '\n';
      }
    }
  } else {
    std::cout << "stepper_electrical_hardware_state status=skipped_until_one_degree_recovers\n";
  }
}

TEST(StepperPhaseElectricalModelTest, ElectricalTorqueUsesTheSameMechanicalInterfaceAsDirectTorque) {
  BalancerSimulator::Config config;
  config.physics_profile = PhysicsProfile::StepperPhaseElectrical;
  config.initial_pitch_deg = 0.0;
  config.initial_pitch_rate_dps = 0.0;
  config.initial_velocity_mps = 0.0;
  config.com_angle_offset_rad = 0.0;

  BalancerSimulator electrical(config);
  electrical.set_emitted_motor_steps(1.0, 1.0);
  electrical.step(1e-6);
  const double left_torque = electrical.diagnostics().stepper_torque_left_nm;
  const double right_torque = electrical.diagnostics().stepper_torque_right_nm;

  BalancerSimulator direct(config);
  direct.set_stepper_direct_torque_for_test(left_torque, right_torque);
  direct.step(1e-6);

  EXPECT_NEAR(direct.diagnostics().x_ddot, electrical.diagnostics().x_ddot, 1e-10);
  EXPECT_NEAR(direct.diagnostics().theta_ddot, electrical.diagnostics().theta_ddot, 1e-10);
  EXPECT_NEAR(direct.state().velocity, electrical.state().velocity, 1e-14);
  EXPECT_NEAR(direct.state().pitch_rate, electrical.state().pitch_rate, 1e-14);
}

TEST(StepperPhaseCharacterizationTest, LinearizedPhaseModeMatchesFixedFieldRingdown) {
  const auto physics =
      BalancerSimulator::physics_for_profile(PhysicsProfile::StepperPhaseElectrical);
  const double nominal_current =
      BalancerSimulator::HardwareNominal::stepper_current_limit_a;
  const double nominal_motor_damping =
      physics.stepper_motor_relative_damping_nm_s_per_rad;
  const auto with_damping = linearize_motor_relative_phase(
      physics, nominal_current, nominal_motor_damping);
  auto no_damping_physics = physics;
  no_damping_physics.cart_damping = 0.0;
  no_damping_physics.pitch_damping = 0.0;
  no_damping_physics.stepper_motor_relative_damping_nm_s_per_rad = 0.0;
  const auto without_damping = linearize_motor_relative_phase(
      no_damping_physics, nominal_current, 0.0);

  EXPECT_NEAR(with_damping.effective_inertia_kg_m2, 0.00014593343357, 2e-12);
  EXPECT_NEAR(with_damping.magnetic_stiffness_nm_per_rad, 22.59206165891019, 1e-12);
  EXPECT_NEAR(with_damping.natural_frequency_hz, 62.6211, 0.02);
  EXPECT_NEAR(without_damping.damping_ratio, 0.0, 1e-12);
  const auto measured_zero_damping =
      run_fixed_field_ringdown(
          no_damping_physics, true,
          BalancerSimulator::HardwareNominal::stepper_current_limit_a, 2.0, 0.0);
  EXPECT_NEAR(measured_zero_damping.frequency_hz, without_damping.natural_frequency_hz, 1.5);

  std::cout << "stepper_phase_mode inertia_kg_m2="
            << with_damping.effective_inertia_kg_m2
            << " stiffness_nm_per_rad=" << with_damping.magnetic_stiffness_nm_per_rad
            << " frequency_hz=" << with_damping.natural_frequency_hz
            << " period_ms=" << with_damping.period_s * 1000.0
            << " damping_ratio_nominal=" << with_damping.damping_ratio
            << " damping_ratio_zero_empirical=" << without_damping.damping_ratio
            << " measured_zero_damping_zeta=" << measured_zero_damping.damping_ratio
            << " measured_zero_damping_frequency_hz=" << measured_zero_damping.frequency_hz << '\n';

  for (const double phase_deg : {1.0, 2.0, 5.0}) {
    const auto ideal = run_fixed_field_ringdown(
        physics, false, BalancerSimulator::HardwareNominal::stepper_current_limit_a,
        phase_deg, nominal_motor_damping);
    const auto electrical = run_fixed_field_ringdown(
        physics, true, BalancerSimulator::HardwareNominal::stepper_current_limit_a,
        phase_deg, nominal_motor_damping);
    EXPECT_NEAR(ideal.frequency_hz, with_damping.natural_frequency_hz, 1.5);
    EXPECT_NEAR(electrical.frequency_hz, with_damping.natural_frequency_hz, 1.5);
    EXPECT_LT(electrical.maximum_current_error_a, 0.02);
    std::cout << "stepper_phase_ringdown phase_deg=" << phase_deg
              << " ideal_frequency_hz=" << ideal.frequency_hz
              << " electrical_frequency_hz=" << electrical.frequency_hz
              << " ideal_zeta=" << ideal.damping_ratio
              << " electrical_zeta=" << electrical.damping_ratio
              << " ideal_peak_torque_nm=" << ideal.peak_torque_nm_per_motor
              << " electrical_peak_torque_nm=" << electrical.peak_torque_nm_per_motor
              << " electrical_max_current_error_a=" << electrical.maximum_current_error_a
              << " zero_crossings=" << electrical.zero_crossings << '\n';
  }

  for (const double motor_damping : {0.0, 0.005, 0.01, 0.02, 0.05}) {
    const auto predicted =
        linearize_motor_relative_phase(
            physics, BalancerSimulator::HardwareNominal::stepper_current_limit_a,
            motor_damping);
    const auto measured =
        run_fixed_field_ringdown(
            physics, true, BalancerSimulator::HardwareNominal::stepper_current_limit_a,
            2.0, motor_damping);
    std::cout << "stepper_phase_ringdown_damping motor_damping_nm_s_per_rad="
              << motor_damping << " predicted_zeta=" << predicted.damping_ratio
              << " measured_zeta=" << measured.damping_ratio
              << " measured_frequency_hz=" << measured.frequency_hz << '\n';
  }
  const auto nominal_step = run_fixed_field_ringdown(
      physics, true, BalancerSimulator::HardwareNominal::stepper_current_limit_a,
      2.0, nominal_motor_damping, 1.0e-5);
  const auto half_step = run_fixed_field_ringdown(
      physics, true, BalancerSimulator::HardwareNominal::stepper_current_limit_a,
      2.0, nominal_motor_damping, 5.0e-6);
  EXPECT_NEAR(nominal_step.frequency_hz, half_step.frequency_hz, 0.05);
  EXPECT_NEAR(nominal_step.damping_ratio, half_step.damping_ratio, 0.001);
  std::cout << "stepper_phase_ringdown_convergence dt_us=10 frequency_hz="
            << nominal_step.frequency_hz << " zeta=" << nominal_step.damping_ratio
            << " dt_us=5 frequency_hz=" << half_step.frequency_hz
            << " zeta=" << half_step.damping_ratio << '\n';
}

TEST(StepperPhaseCharacterizationTest, RotorInertiaInventoryAndCurrentScalingAreExplicit) {
  const auto physics =
      BalancerSimulator::physics_for_profile(PhysicsProfile::StepperPhaseElectrical);
  const double rotor_inertia_per_motor =
      BalancerSimulator::HardwareNominal::stepper_rotating_inertia_kg_m2_per_motor;
  const double radius = BalancerSimulator::HardwareNominal::stepper_phase_wheel_radius;
  const double equivalent_translational_mass_total =
      2.0 * rotor_inertia_per_motor / (radius * radius);
  std::cout << "stepper_inertia_inventory rotating_per_motor_kg_m2="
            << rotor_inertia_per_motor
            << " rotating_pitch_total_kg_m2=" << 2.0 * rotor_inertia_per_motor
            << " rotor_equivalent_mass_total_kg=" << equivalent_translational_mass_total
            << " wheel_hub_inertia_included=true\n";

  double previous_frequency = 0.0;
  for (const double current_limit_a : {0.5, 0.75, 1.0, 1.2, 1.5}) {
    const auto expected = linearize_motor_relative_phase(physics, current_limit_a);
    const auto ringdown =
        run_fixed_field_ringdown(physics, true, current_limit_a, 2.0, 0.0);
    EXPECT_NEAR(ringdown.frequency_hz, expected.natural_frequency_hz, 1.5);
    EXPECT_GT(ringdown.frequency_hz, previous_frequency);
    previous_frequency = ringdown.frequency_hz;
    std::cout << "stepper_phase_current_sensitivity current_a=" << current_limit_a
              << " predicted_frequency_hz=" << expected.natural_frequency_hz
              << " measured_frequency_hz=" << ringdown.frequency_hz
              << " peak_torque_nm_per_motor=" << ringdown.peak_torque_nm_per_motor
              << " damping_ratio=" << ringdown.damping_ratio << '\n';
  }
}

TEST(StepperPhaseCharacterizationTest, DiagnosticMotorDampingQuantifiesRecoveryRequirement) {
  ScopedPidValues restore;
  ConfigPid::load((std::filesystem::path(BALANCER_REPO_ROOT) / "pid.conf").string());
  ConfigPid::values.velocity_damping_per_s = 0.0;
  ConfigPid::values.velocity_I = 0.0;
  ConfigPid::values.velocity_pitch_limit_deg = 0.0;
  ConfigPid::values.pitch_rate_gain = 350.0;
  ConfigPid::values.pitch_accel_gain = 0.0;

  const auto base_physics =
      BalancerSimulator::physics_for_profile(PhysicsProfile::StepperPhaseElectrical);
  for (const double motor_damping : {0.0, 0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0,
                                     2.0, 5.0, 10.0, 20.0}) {
    const auto mode = linearize_motor_relative_phase(
        base_physics, BalancerSimulator::HardwareNominal::stepper_current_limit_a,
        motor_damping);
    std::cout << "stepper_phase_damping_sweep motor_damping_nm_s_per_rad="
              << motor_damping << " predicted_zeta=" << mode.damping_ratio
              << " ringdown_frequency_hz=" << mode.natural_frequency_hz;
    for (const double pitch_deg : {1.0, 2.0, 4.0}) {
      auto scenario = stepper_recovery_scenario(pitch_deg, 0.0, 0.0);
      scenario.physics_profile = PhysicsProfile::StepperPhaseElectrical;
      scenario.physics_override = base_physics;
      scenario.physics_override->stepper_motor_relative_damping_nm_s_per_rad = motor_damping;
      const auto result = run_simulator_scenario_with_loaded_pid(scenario);
      double max_applied_force = 0.0;
      double max_relative_velocity = 0.0;
      for (const auto& row : result.rows) {
        max_applied_force = std::max(max_applied_force, std::abs(row.applied_drive_force));
        max_relative_velocity = std::max(
            max_relative_velocity,
            std::abs(row.stepper_actual_wheel_velocity_mps) /
                BalancerSimulator::HardwareNominal::stepper_phase_wheel_radius);
      }
      std::cout << " pitch_deg=" << pitch_deg << " fell=" << result.fell
                << " peak_pitch_deg=" << result.max_abs_pitch_deg
                << " max_applied_torque_nm="
                << max_applied_force * BalancerSimulator::HardwareNominal::stepper_phase_wheel_radius
                << " max_relative_velocity_rad_s=" << max_relative_velocity;
    }
    std::cout << '\n';
  }
}

TEST(StepperPhaseCharacterizationTest, CurrentLimitRecoverySensitivityIsReported) {
  ScopedPidValues restore;
  ConfigPid::load((std::filesystem::path(BALANCER_REPO_ROOT) / "pid.conf").string());
  ConfigPid::values.velocity_damping_per_s = 0.0;
  ConfigPid::values.velocity_I = 0.0;
  ConfigPid::values.velocity_pitch_limit_deg = 0.0;
  ConfigPid::values.pitch_rate_gain = 350.0;
  ConfigPid::values.pitch_accel_gain = 0.0;

  const auto base_physics =
      BalancerSimulator::physics_for_profile(PhysicsProfile::StepperPhaseElectrical);
  for (const double pitch_gain : {6000.0, 8000.0}) {
    ConfigPid::values.pitch_gain = pitch_gain;
    for (const double current_limit_a : {0.5, 0.75, 1.0, 1.2, 1.5}) {
      auto scenario = stepper_recovery_scenario(1.0, 0.0, 0.0);
      scenario.physics_profile = PhysicsProfile::StepperPhaseElectrical;
      scenario.physics_override = base_physics;
      scenario.physics_override->stepper_current_limit_a = current_limit_a;
      const auto result = run_simulator_scenario_with_loaded_pid(scenario);
      double max_phase = 0.0;
      double max_torque = 0.0;
      for (const auto& row : result.rows) {
        max_phase = std::max(max_phase,
                             std::max(std::abs(row.stepper_electrical_phase_error_left_rad),
                                      std::abs(row.stepper_electrical_phase_error_right_rad)));
        max_torque = std::max(max_torque,
                              std::max(std::abs(row.stepper_torque_left_nm),
                                       std::abs(row.stepper_torque_right_nm)));
      }
      std::cout << "stepper_phase_recovery_current kp=" << pitch_gain
                << " current_a=" << current_limit_a << " fell=" << result.fell
                << " max_phase_rad=" << max_phase
                << " peak_torque_nm_per_motor=" << max_torque << '\n';
    }
  }
}

struct SimulatorRingdownSample {
  double frequency_hz = 0.0;
  double initial_abs_phase_rad = 0.0;
  double final_abs_phase_rad = 0.0;
};

SimulatorRingdownSample run_converged_simulator_ringdown(double max_physical_step_s) {
  auto physics =
      BalancerSimulator::physics_for_profile(PhysicsProfile::StepperPhaseElectrical);
  physics.max_physical_integration_step_s = max_physical_step_s;

  BalancerSimulator::Config config;
  config.physics_profile = PhysicsProfile::StepperPhaseElectrical;
  config.physics_override = physics;
  config.initial_pitch_deg = 0.0;
  config.com_angle_offset_rad = 0.0;
  BalancerSimulator simulator(config);
  simulator.set_emitted_motor_steps(0.0, 0.0);
  const double initial_phase_rad = 2.0 * kPi / 180.0;
  simulator.set_stepper_relative_angles_for_test(
      -initial_phase_rad / BalancerSimulator::HardwareNominal::stepper_phase_electrical_cycles_per_rev,
      -initial_phase_rad / BalancerSimulator::HardwareNominal::stepper_phase_electrical_cycles_per_rev);

  constexpr double sample_dt_s = 0.0005;
  constexpr double duration_s = 0.20;
  std::vector<double> phases;
  phases.reserve(static_cast<std::size_t>(duration_s / sample_dt_s));
  for (double elapsed_s = 0.0; elapsed_s < duration_s; elapsed_s += sample_dt_s) {
    simulator.step(sample_dt_s);
    phases.push_back(simulator.stepper_phase_electrical_output().left.electrical_phase_error_rad);
  }

  std::vector<double> positive_crossings;
  for (std::size_t index = 1; index < phases.size(); ++index) {
    if (phases[index - 1] >= 0.0 && phases[index] < 0.0) {
      positive_crossings.push_back(static_cast<double>(index) * sample_dt_s);
    }
  }
  double frequency_hz = 0.0;
  if (positive_crossings.size() >= 2U) {
    const double period_s = (positive_crossings.back() - positive_crossings.front()) /
                            static_cast<double>(positive_crossings.size() - 1U);
    frequency_hz = period_s > 0.0 ? 1.0 / period_s : 0.0;
  }
  const double final_abs_phase = phases.empty() ? 0.0 : std::abs(phases.back());
  return SimulatorRingdownSample{
      .frequency_hz = frequency_hz,
      .initial_abs_phase_rad = std::abs(phases.front()),
      .final_abs_phase_rad = final_abs_phase,
  };
}

TEST(StepperPhaseCharacterizationTest, OneThirtySecondSpsGainSanityIsReported) {
  ScopedPidValues restore;
  ConfigPid::load((std::filesystem::path(BALANCER_REPO_ROOT) / "pid.conf").string());
  ConfigPid::values.velocity_damping_per_s = 0.0;
  ConfigPid::values.velocity_I = 0.0;
  ConfigPid::values.velocity_pitch_limit_deg = 0.0;
  ConfigPid::values.pitch_accel_gain = 0.0;

  SimulatorScenario scenario;
  scenario.name = "one_thirty_second_gain_sanity";
  scenario.physics_profile = PhysicsProfile::DirectActuator;
  scenario.duration_s = 3.0;
  scenario.initial_pitch_deg = 2.0;
  scenario.initial_fused_pitch_deg = 2.0;
  scenario.com_angle_offset_rad = 0.0;

  for (const auto [pitch_gain, pitch_rate_gain] :
       std::array<std::pair<double, double>, 5>{{
           {6000.0, 350.0}, {8000.0, 350.0}, {12000.0, 350.0},
           {12000.0, 700.0}, {16000.0, 700.0}}}) {
    ConfigPid::values.pitch_gain = pitch_gain;
    ConfigPid::values.pitch_rate_gain = pitch_rate_gain;
    const auto result = run_simulator_scenario_with_loaded_pid(scenario);
    std::cout << "stepper_one_thirty_second_gain_sanity pitch_gain=" << pitch_gain
              << " pitch_rate_gain=" << pitch_rate_gain << " fell=" << result.fell
              << " peak_pitch_deg=" << result.max_abs_pitch_deg
              << " tail_rms_deg=" << result.tail_rms_pitch_deg
              << " controller_fault_flags=" << result.controller_fault_flags << '\n';
  }
}

TEST(StepperPhaseCharacterizationTest, SimulatorPhysicalStepConvergesAtBodyCoupledPhaseMode) {
  const auto physics =
      BalancerSimulator::physics_for_profile(PhysicsProfile::StepperPhaseElectrical);
  const auto predicted = linearize_motor_relative_phase(
      physics, BalancerSimulator::HardwareNominal::stepper_current_limit_a,
      physics.stepper_motor_relative_damping_nm_s_per_rad);
  std::cout << "stepper_sim_ringdown predicted_hz=" << predicted.natural_frequency_hz;
  const std::array<double, 4> physical_steps_s = {500.0e-6, 250.0e-6, 125.0e-6, 62.5e-6};
  std::array<SimulatorRingdownSample, 4> samples{};
  for (std::size_t index = 0; index < physical_steps_s.size(); ++index) {
    samples[index] = run_converged_simulator_ringdown(physical_steps_s[index]);
    std::cout << " max_step_us=" << physical_steps_s[index] * 1e6
              << " measured_hz=" << samples[index].frequency_hz
              << " initial_phase_rad=" << samples[index].initial_abs_phase_rad
              << " final_phase_rad=" << samples[index].final_abs_phase_rad;
  }
  std::cout << '\n';

  // A 500 us step is intentionally included as a coarse boundary.  It is
  // only about 21 samples per phase cycle and shows the expected numerical
  // distortion; 250 us and below are the converged validation range.
  for (std::size_t index = 1; index < samples.size(); ++index) {
    const auto& sample = samples[index];
    ASSERT_GT(sample.frequency_hz, 0.0);
    EXPECT_NEAR(sample.frequency_hz, predicted.natural_frequency_hz, 2.0);
  }
  EXPECT_NEAR(samples[2].frequency_hz, samples[3].frequency_hz, 0.5);
  EXPECT_LT(samples[3].final_abs_phase_rad, samples[3].initial_abs_phase_rad);
}

struct EventReplayState {
  double pitch_rad = 0.0;
  double pitch_rate_rad_s = 0.0;
  double position_m = 0.0;
  double velocity_mps = 0.0;
  double phase_error_rad = 0.0;
};

EventReplayState replay_stepper_events(const std::vector<uint64_t>& polling_boundaries_us) {
  auto physics =
      BalancerSimulator::physics_for_profile(PhysicsProfile::StepperPhaseElectrical);
  physics.max_physical_integration_step_s = 62.5e-6;
  BalancerSimulator::Config config;
  config.physics_profile = PhysicsProfile::StepperPhaseElectrical;
  config.physics_override = physics;
  config.initial_pitch_deg = 1.0;
  BalancerSimulator simulator(config);

  // This is a fixed actuator event stream, not an SPS command.  The two
  // replay harnesses below differ only in how they group the same events.
  constexpr uint64_t kEventSpacingUs = 500;
  constexpr int kEventCount = 20;
  uint64_t now_us = 0;
  int emitted_steps = 0;
  std::size_t event_index = 0;
  simulator.set_emitted_motor_steps(0.0, 0.0);
  for (const uint64_t boundary_us : polling_boundaries_us) {
    while (event_index < static_cast<std::size_t>(kEventCount) &&
           (event_index + 1U) * kEventSpacingUs <= boundary_us) {
      const uint64_t event_us = (event_index + 1U) * kEventSpacingUs;
      simulator.step(static_cast<double>(event_us - now_us) / 1e6);
      now_us = event_us;
      ++emitted_steps;
      simulator.set_emitted_motor_steps(emitted_steps, emitted_steps);
      ++event_index;
    }
    simulator.step(static_cast<double>(boundary_us - now_us) / 1e6);
    now_us = boundary_us;
  }

  const auto& state = simulator.state();
  return EventReplayState{
      .pitch_rad = state.pitch,
      .pitch_rate_rad_s = state.pitch_rate,
      .position_m = state.position,
      .velocity_mps = state.velocity,
      .phase_error_rad = simulator.stepper_phase_electrical_output().left.electrical_phase_error_rad,
  };
}

TEST(SimulatorClockIsolationTest, SameStepEventStreamIsIndependentOfPollingBoundaries) {
  const auto controller_like = replay_stepper_events({2500, 5000, 7500, 10000});
  const auto finer_polling = replay_stepper_events({1250, 2500, 3750, 5000,
                                                    6250, 7500, 8750, 10000});

  EXPECT_NEAR(controller_like.pitch_rad, finer_polling.pitch_rad, 1e-12);
  EXPECT_NEAR(controller_like.pitch_rate_rad_s, finer_polling.pitch_rate_rad_s, 1e-10);
  EXPECT_NEAR(controller_like.position_m, finer_polling.position_m, 1e-12);
  EXPECT_NEAR(controller_like.velocity_mps, finer_polling.velocity_mps, 1e-10);
  EXPECT_NEAR(controller_like.phase_error_rad, finer_polling.phase_error_rad, 1e-12);
}

TEST(StepperPhaseElectricalTuningTest, CorrectedPlantGainRegionAndRecoveryFrontier) {
  ScopedPidValues restore;
  ConfigPid::load((std::filesystem::path(BALANCER_REPO_ROOT) / "pid.conf").string());
  ConfigPid::values.velocity_damping_per_s = 0.0;
  ConfigPid::values.velocity_I = 0.0;
  ConfigPid::values.velocity_pitch_limit_deg = 0.0;
  ConfigPid::values.pitch_accel_gain = 0.0;
  ConfigPid::values.balance_max_sps = Config::nominal_balance_max_sps;

  struct Metrics {
    SimulatorRunResult result;
    double requested_peak_sps = 0.0;
    double requested_unclamped_peak_sps = 0.0;
    double requested_p95_sps = 0.0;
    double requested_p99_sps = 0.0;
    double emitted_peak_sps = 0.0;
    double emitted_p95_sps = 0.0;
    double emitted_p99_sps = 0.0;
    double completed_p95_sps = 0.0;
    double completed_p99_sps = 0.0;
    double phase_peak_rad = 0.0;
    double torque_peak_nm_per_motor = 0.0;
    double field_wheel_velocity_difference_peak_mps = 0.0;
    double current_error_peak_a = 0.0;
    double voltage_saturated_s = 0.0;
    double command_saturated_s = 0.0;
    double first_torque_reversal_ms = -1.0;
  };

  const auto percentile = [](std::vector<double> values, double fraction) {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const double position = fraction * static_cast<double>(values.size() - 1U);
    const auto lower = static_cast<std::size_t>(position);
    const auto upper = std::min(values.size() - 1U, lower + 1U);
    const double weight = position - static_cast<double>(lower);
    return values[lower] * (1.0 - weight) + values[upper] * weight;
  };

  const auto run = [&](double pitch_gain, double pitch_rate_gain, double pitch_deg,
                       double duration_s,
                       double balance_limit_sps = Config::nominal_balance_max_sps) {
    ConfigPid::values.pitch_gain = pitch_gain;
    ConfigPid::values.pitch_rate_gain = pitch_rate_gain;
    ConfigPid::values.balance_max_sps = balance_limit_sps;
    auto scenario = stepper_recovery_scenario(pitch_deg, 0.0, 0.0);
    scenario.physics_profile = PhysicsProfile::StepperPhaseElectrical;
    scenario.duration_s = duration_s;
    Metrics metrics{.result = run_simulator_scenario_with_loaded_pid(scenario)};
    std::vector<double> completed_sps;
    std::vector<double> requested_sps;
    std::vector<double> emitted_sps;
    double previous_torque = 0.0;
    bool have_torque_sign = false;
    for (std::size_t index = 0; index < metrics.result.rows.size(); ++index) {
      const auto& row = metrics.result.rows[index];
      metrics.requested_peak_sps = std::max(metrics.requested_peak_sps, std::abs(row.u_sps));
      requested_sps.push_back(std::abs(row.u_sps));
      metrics.requested_unclamped_peak_sps =
          std::max(metrics.requested_unclamped_peak_sps, std::abs(row.balance_unclamped_sps));
      metrics.emitted_peak_sps =
          std::max(metrics.emitted_peak_sps, std::abs(row.left_slewed_sps));
      emitted_sps.push_back(std::abs(row.left_slewed_sps));
      metrics.phase_peak_rad =
          std::max(metrics.phase_peak_rad,
                   std::max(std::abs(row.stepper_electrical_phase_error_left_rad),
                            std::abs(row.stepper_electrical_phase_error_right_rad)));
      metrics.torque_peak_nm_per_motor =
          std::max(metrics.torque_peak_nm_per_motor,
                   std::max(std::abs(row.stepper_torque_left_nm),
                            std::abs(row.stepper_torque_right_nm)));
      metrics.field_wheel_velocity_difference_peak_mps =
          std::max(metrics.field_wheel_velocity_difference_peak_mps,
                   std::abs(row.stepper_commanded_field_velocity_mps -
                            row.stepper_actual_wheel_velocity_mps));
      metrics.current_error_peak_a =
          std::max(metrics.current_error_peak_a,
                   std::max(std::abs(row.stepper_current_ref_a_left - row.stepper_current_a_left) +
                                std::abs(row.stepper_current_ref_b_left - row.stepper_current_b_left),
                            std::abs(row.stepper_current_ref_a_right -
                                     row.stepper_current_a_right) +
                                std::abs(row.stepper_current_ref_b_right -
                                         row.stepper_current_b_right)));
      if (row.command_saturated > 0.5) metrics.command_saturated_s += 1.0 / 400.0;
      if (row.stepper_voltage_saturated_left > 0.5 ||
          row.stepper_voltage_saturated_right > 0.5) {
        metrics.voltage_saturated_s += 1.0 / 400.0;
      }
      if (index > 0) {
        const auto& previous = metrics.result.rows[index - 1U];
        const double dt_s = row.sim_time_s - previous.sim_time_s;
        if (dt_s > 0.0) {
          completed_sps.push_back(std::abs(row.left_actual_steps -
                                           previous.left_actual_steps) /
                                  dt_s);
        }
      }
      const double torque = row.stepper_torque_left_nm;
      if (std::abs(torque) > 1.0e-8) {
        if (have_torque_sign && torque * previous_torque < 0.0 &&
            metrics.first_torque_reversal_ms < 0.0) {
          metrics.first_torque_reversal_ms = row.sim_time_s * 1000.0;
        }
        previous_torque = torque;
        have_torque_sign = true;
      }
    }
    metrics.completed_p95_sps = percentile(completed_sps, 0.95);
    metrics.completed_p99_sps = percentile(completed_sps, 0.99);
    metrics.requested_p95_sps = percentile(requested_sps, 0.95);
    metrics.requested_p99_sps = percentile(requested_sps, 0.99);
    metrics.emitted_p95_sps = percentile(emitted_sps, 0.95);
    metrics.emitted_p99_sps = percentile(emitted_sps, 0.99);
    return metrics;
  };

  const std::array<std::pair<double, double>, 14> gain_region = {{
      {160000.0, 8000.0}, {180000.0, 8000.0}, {200000.0, 10000.0},
      {160000.0, 9000.0}, {180000.0, 9000.0}, {200000.0, 9000.0},
      {220000.0, 10000.0}, {240000.0, 10000.0}, {260000.0, 12000.0},
      {220000.0, 12000.0}, {240000.0, 12000.0}, {260000.0, 14000.0},
      {280000.0, 14000.0}, {300000.0, 16000.0},
  }};
  for (const auto& [pitch_gain, pitch_rate_gain] : gain_region) {
    const auto positive = run(pitch_gain, pitch_rate_gain, 1.0, 2.0);
    const auto negative = run(pitch_gain, pitch_rate_gain, -1.0, 2.0);
    EXPECT_FALSE(positive.result.fell);
    EXPECT_FALSE(negative.result.fell);
    std::cout << "stepper_gain_region kp=" << pitch_gain << " kr=" << pitch_rate_gain
              << " positive_peak_deg=" << positive.result.max_abs_pitch_deg
              << " negative_peak_deg=" << negative.result.max_abs_pitch_deg
              << " positive_tail_rms_deg=" << positive.result.tail_rms_pitch_deg
              << " saturation_s=" << positive.command_saturated_s << '\n';
  }

  const std::array<std::pair<double, double>, 3> historical_gains = {{
      {6000.0, 350.0}, {12000.0, 700.0}, {24000.0, 1400.0},
  }};
  for (const auto& [pitch_gain, pitch_rate_gain] : historical_gains) {
    const auto positive = run(pitch_gain, pitch_rate_gain, 1.0, 2.0);
    const auto negative = run(pitch_gain, pitch_rate_gain, -1.0, 2.0);
    EXPECT_TRUE(positive.result.fell);
    EXPECT_TRUE(negative.result.fell);
    std::cout << "stepper_historical_gain kp=" << pitch_gain << " kr=" << pitch_rate_gain
              << " positive_peak_deg=" << positive.result.max_abs_pitch_deg
              << " negative_peak_deg=" << negative.result.max_abs_pitch_deg
              << " requested_peak_sps=" << positive.requested_peak_sps
              << " emitted_peak_sps=" << positive.emitted_peak_sps << '\n';
  }

  constexpr double selected_pitch_gain = 280000.0;
  constexpr double selected_pitch_rate_gain = 12000.0;
  const auto selected_positive = run(selected_pitch_gain, selected_pitch_rate_gain, 1.0, 3.0);
  const auto selected_negative = run(selected_pitch_gain, selected_pitch_rate_gain, -1.0, 3.0);
  EXPECT_FALSE(selected_positive.result.fell);
  EXPECT_FALSE(selected_negative.result.fell);
  EXPECT_LE(selected_positive.requested_peak_sps, Config::nominal_balance_max_sps + 1.0e-9);
  EXPECT_LT(selected_positive.command_saturated_s, 0.05);
  EXPECT_LT(selected_positive.voltage_saturated_s, 0.01);
  EXPECT_NEAR(selected_positive.result.max_abs_pitch_deg,
              selected_negative.result.max_abs_pitch_deg, 1.0e-9);
  EXPECT_NEAR(selected_positive.result.final_pitch_deg,
              -selected_negative.result.final_pitch_deg, 1.0e-9);
  const auto quiet = run(selected_pitch_gain, selected_pitch_rate_gain, 0.0, 2.0);
  EXPECT_FALSE(quiet.result.fell);
  EXPECT_LT(quiet.result.max_abs_pitch_deg, 1.0e-9);
  std::cout << "stepper_selected_quiet max_abs_pitch_deg="
            << quiet.result.max_abs_pitch_deg << " requested_peak_sps="
            << quiet.requested_peak_sps << '\n';
  std::cout << "stepper_selected_gain kp=" << selected_pitch_gain
            << " kr=" << selected_pitch_rate_gain
            << " positive_peak_deg=" << selected_positive.result.max_abs_pitch_deg
            << " positive_final_deg=" << selected_positive.result.final_pitch_deg
            << " positive_tail_rms_deg=" << selected_positive.result.tail_rms_pitch_deg
            << " requested_peak_sps=" << selected_positive.requested_peak_sps
            << " requested_unclamped_peak_sps="
            << selected_positive.requested_unclamped_peak_sps
            << " requested_p95_sps=" << selected_positive.requested_p95_sps
            << " requested_p99_sps=" << selected_positive.requested_p99_sps
            << " emitted_peak_sps=" << selected_positive.emitted_peak_sps
            << " emitted_p95_sps=" << selected_positive.emitted_p95_sps
            << " emitted_p99_sps=" << selected_positive.emitted_p99_sps
            << " completed_p95_sps=" << selected_positive.completed_p95_sps
            << " completed_p99_sps=" << selected_positive.completed_p99_sps
            << " command_saturated_s=" << selected_positive.command_saturated_s
            << " voltage_saturated_s=" << selected_positive.voltage_saturated_s
            << " phase_peak_rad=" << selected_positive.phase_peak_rad
            << " torque_peak_nm_per_motor=" << selected_positive.torque_peak_nm_per_motor
            << " field_wheel_velocity_difference_peak_mps="
            << selected_positive.field_wheel_velocity_difference_peak_mps
            << " current_error_peak_a=" << selected_positive.current_error_peak_a
            << " first_torque_reversal_ms=" << selected_positive.first_torque_reversal_ms
            << '\n';

  for (const double pitch_deg : {4.0, 6.0}) {
    const double duration_s = pitch_deg <= 4.0 ? 3.0 : 10.0;
    const auto verified_limit = run(selected_pitch_gain, selected_pitch_rate_gain, pitch_deg,
                                    duration_s, Config::nominal_balance_max_sps);
    const auto historical_limit = run(selected_pitch_gain, selected_pitch_rate_gain, pitch_deg,
                                      duration_s, 8000.0);
    std::cout << "stepper_limit_comparison pitch_deg=" << pitch_deg
              << " verified_fell=" << verified_limit.result.fell
              << " historical_fell=" << historical_limit.result.fell
              << " verified_requested_peak_sps=" << verified_limit.requested_peak_sps
              << " historical_requested_peak_sps=" << historical_limit.requested_peak_sps
              << " verified_saturation_s=" << verified_limit.command_saturated_s
              << " historical_saturation_s=" << historical_limit.command_saturated_s << '\n';
  }

  for (const double pitch_deg : {1.0, 2.0, 4.0, 6.0, 8.0}) {
    const double duration_s = pitch_deg <= 4.0 ? 3.0 : 10.0;
    const auto frontier = run(selected_pitch_gain, selected_pitch_rate_gain, pitch_deg,
                              duration_s);
    const auto mirrored_frontier = run(selected_pitch_gain, selected_pitch_rate_gain,
                                       -pitch_deg, duration_s);
    EXPECT_EQ(frontier.result.fell, mirrored_frontier.result.fell);
    EXPECT_NEAR(frontier.result.max_abs_pitch_deg,
                mirrored_frontier.result.max_abs_pitch_deg, 1.0e-9);
    if (pitch_deg <= 4.0) {
      EXPECT_FALSE(frontier.result.fell);
      EXPECT_FALSE(mirrored_frontier.result.fell);
    } else {
      // The compact tune is deliberately judged on quiet recovery through
      // +/-4 degrees.  Larger releases are reported as the current frontier:
      // they eventually hit the 16000-SPS authority and fall, rather than
      // being mislabeled as stable from a short bounded transient.
      EXPECT_TRUE(frontier.result.fell);
      EXPECT_TRUE(mirrored_frontier.result.fell);
    }
    std::cout << "stepper_selected_frontier pitch_deg=" << pitch_deg
              << " fell=" << frontier.result.fell
              << " peak_pitch_deg=" << frontier.result.max_abs_pitch_deg
              << " final_pitch_deg=" << frontier.result.final_pitch_deg
              << " tail_rms_deg=" << frontier.result.tail_rms_pitch_deg
              << " requested_peak_sps=" << frontier.requested_peak_sps
              << " emitted_peak_sps=" << frontier.emitted_peak_sps
              << " command_saturated_s=" << frontier.command_saturated_s << '\n';
  }
}

}  // namespace
