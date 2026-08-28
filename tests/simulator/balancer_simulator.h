#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <optional>
#include <string_view>

#include "messages/balancer_msgs.h"
#include "services/main/config.h"
#include "simulator/stepper_phase_actuator.h"

enum class PhysicsProfile {
  // Existing wire values remain stable: simplified=0, realistic=1,
  // actuator_stress=2. New offline-only references follow them.
  Simplified = 0,
  Realistic = 1,
  // Realistic with the deliberately conservative aggregate actuator lag used
  // for robustness experiments. This is not the nominal motor time constant.
  ActuatorStress = 2,
  // Direct applied-force references bypass the phase-position model. They are
  // controller-architecture fixtures, not hardware-calibrated motor models.
  // Numeric value 3 is retained from the former IdealForce profile so old
  // simulator requests remain wire-compatible.
  DirectActuator = 3,
  // Source/config compatibility alias. New code and reports should use
  // DirectActuator; this is intentionally not a separate profile value.
  IdealForce = DirectActuator,
  // Wire values 4 and 5 are retained only so old simulator requests can be
  // rejected/migrated without renumbering the message field.  They no longer
  // select active actuator implementations.
  RetiredSimpleForce = 4,
  RetiredNoSlipActuator = 5,
  // First-stage magnetic-field/rotor phase model. It uses the authoritative
  // production 1/32 kinematics and ideal current tracking; electrical
  // dynamics are deferred.
  StepperPhase = 6,
  // First electrical/driver-stage StepperPhase model. It retains the same
  // mechanical topology but bounds current evolution with motor R/L,
  // back-EMF, and bus voltage.
  StepperPhaseElectrical = 7,
};

struct SimulatorPhysics {
  // Aggregate zero-speed tangential force from both motors at the wheel rim
  // for the legacy force-based profiles. StepperPhase profiles use their
  // actuator torque/current parameters instead.
  double max_force_n = 22.5;
  double no_load_speed_mps = 1.2;
  double traction_coefficient = 1.0;
  double motor_velocity_damping = 8.0;
  // Linear viscous damping on chassis translation: generalized force
  // F_cart = -cart_damping * x_dot, in N*s/m. It is not wheel, rotor, or
  // relative motor damping; the coupled mass matrix propagates this force
  // into pitch acceleration as appropriate.
  double cart_damping = 1.0;
  // Low-speed wheel-ground rolling/Coulomb resistance at the chassis contact,
  // applied opposite to chassis translation. This is distinct from viscous
  // cart_damping and is used by the electrical StepperPhase profile to model
  // the finite force needed to sustain ordinary hardware-like translation.
  // A zero value preserves the frictionless diagnostic model.
  double rolling_resistance_force_n = 0.0;
  // Static contact force available before the chassis breaks away.  This is
  // deliberately separate from rolling_resistance_force_n: the latter only
  // acts while moving, while this term permits a small sustained lean to be
  // held without an artificial low-speed velocity limit cycle.
  double static_breakaway_force_n = 0.0;
  double pitch_damping = 0.02;
  // Gravity is configurable for conservative/passivity fixtures.  Production
  // profiles leave this at standard terrestrial gravity.
  double gravity_mps2 = 9.81;
  double motor_tau_s = 0.008;
  double phase_error_limit_steps = 16.0;
  double tire_stiffness_n_per_m = 3000.0;
  double tire_damping_n_s_per_m = 12.0;
  double wheel_equivalent_mass_kg = 0.10;
  bool direct_force = false;
  double direct_force_per_sps = 0.0;
  // Optional realization terms used only by retained legacy force-based
  // diagnostic profiles. They are not interpreted by StepperPhase.
  double command_delay_s = 0.0;
  bool speed_dependent_force_limit = false;
  bool force_from_velocity_error = false;
  // The actuator owns STEP/magnetic-field/rotor phase state. The other fields
  // retain their historical meanings for the remaining profiles.
  bool stepper_phase = false;
  // Simulator-only localization fixture: advance the magnetic field from the
  // requested SPS continuously instead of consuming discrete STEP edges.
  // Mechanical torque, phase, and rigid-body equations are unchanged.
  bool stepper_phase_continuous_field = false;
  bool stepper_phase_electrical = false;
  // Total rigidly attached wheel/hub/rotor inertia per motor side.  This is
  // absolute wheel/rotor inertia: it contributes to xdot/r, not to the
  // rotor/stator-relative phase coordinate x/r - pitch.
  double stepper_rotating_inertia_kg_m2_per_motor = 0.0;
  // Motor-relative actuator-loss damping. The maintained electrical profile
  // uses the provisional midpoint derived from the optical ringdown; the
  // ideal-current diagnostic profile keeps this at zero.
  double stepper_motor_relative_damping_nm_s_per_rad = 0.0;
  // Electrical-profile diagnostics only. These are configurable so actuator
  // sensitivity tests do not need a second simulator implementation.
  double stepper_current_limit_a = 1.065;
  double stepper_bus_voltage_v = 11.1;
  // Maximum continuous physical integration interval.  Zero preserves the
  // historical single-call behavior for legacy profiles; StepperPhase
  // profiles set this explicitly so their approximately 93 Hz phase mode is
  // resolved independently of the 400 Hz controller clock.
  double max_physical_integration_step_s = 0.0;
};

struct SimulatorConfig {
  double com_angle_offset_rad = 0.001;
  double initial_pitch_deg = 2.0;
  double initial_pitch_rate_dps = 0.0;
  double initial_velocity_mps = 0.0;
  PhysicsProfile physics_profile = PhysicsProfile::Realistic;
  std::optional<SimulatorPhysics> physics_override;
  double total_mass_scale = 1.0;
  double pitch_inertia_scale = 1.0;
  // Offline robustness knob for the measured first mass moment H. The default
  // preserves the current nominal plant exactly.
  double first_mass_moment_scale = 1.0;
  double imu_height_m = 0.070;
};

class BalancerSimulator {
 public:
  using Config = SimulatorConfig;

  struct State {
    double pitch = 0.0;
    double pitch_rate = 0.0;
    double position = 0.0;
    double velocity = 0.0;
  };

  struct Diagnostics {
    double target_wheel_velocity = 0.0;
    double actual_wheel_velocity = 0.0;
    double velocity_error = 0.0;
    double f_cmd = 0.0;
    double f_app = 0.0;
    double desired_drive_force = 0.0;
    double limited_drive_force = 0.0;
    double applied_drive_force = 0.0;
    double desired_tire_force = 0.0;
    double external_force_n = 0.0;
    double external_com_bias_rad = 0.0;
    double x_ddot = 0.0;
    double theta_ddot = 0.0;
    double phase_error_steps = 0.0;
    double missed_steps = 0.0;
    double traction_limit_n = 0.0;
    double motor_force_limit_n = 0.0;
    bool command_saturated = false;
    bool phase_saturated = false;
    bool motor_force_saturated = false;
    bool traction_saturated = false;
    // StepperPhase-only diagnostics. These remain internal simulator fields;
    // the production telemetry wire schema is unchanged.
    double stepper_commanded_microsteps_left = 0.0;
    double stepper_commanded_microsteps_right = 0.0;
    double stepper_commanded_field_angle_left_rad = 0.0;
    double stepper_commanded_field_angle_right_rad = 0.0;
    double stepper_commanded_field_electrical_angle_left_rad = 0.0;
    double stepper_commanded_field_electrical_angle_right_rad = 0.0;
    double stepper_commanded_field_velocity_mps = 0.0;
    double stepper_actual_relative_angle_left_rad = 0.0;
    double stepper_actual_relative_angle_right_rad = 0.0;
    double stepper_actual_rotor_electrical_angle_left_rad = 0.0;
    double stepper_actual_rotor_electrical_angle_right_rad = 0.0;
    double stepper_electrical_phase_error_left_rad = 0.0;
    double stepper_electrical_phase_error_right_rad = 0.0;
    double stepper_torque_left_nm = 0.0;
    double stepper_torque_right_nm = 0.0;
    double stepper_summed_torque_nm = 0.0;
    double stepper_damping_torque_left_nm = 0.0;
    double stepper_damping_torque_right_nm = 0.0;
    double stepper_applied_torque_left_nm = 0.0;
    double stepper_applied_torque_right_nm = 0.0;
    double stepper_motor_relative_velocity_left_rad_s = 0.0;
    double stepper_motor_relative_velocity_right_rad_s = 0.0;
    double stepper_actual_wheel_velocity_mps = 0.0;
    double stepper_chassis_velocity_mps = 0.0;
    // StepperPhaseElectrical-only diagnostics.
    double stepper_current_ref_a_left = 0.0;
    double stepper_current_ref_b_left = 0.0;
    double stepper_current_a_left = 0.0;
    double stepper_current_b_left = 0.0;
    double stepper_phase_voltage_a_left = 0.0;
    double stepper_phase_voltage_b_left = 0.0;
    double stepper_back_emf_a_left = 0.0;
    double stepper_back_emf_b_left = 0.0;
    double stepper_electrical_power_left_w = 0.0;
    double stepper_mechanical_power_left_w = 0.0;
    double stepper_resistive_loss_left_w = 0.0;
    double stepper_magnetic_energy_left_j = 0.0;
    double stepper_current_ref_a_right = 0.0;
    double stepper_current_ref_b_right = 0.0;
    double stepper_current_a_right = 0.0;
    double stepper_current_b_right = 0.0;
    double stepper_phase_voltage_a_right = 0.0;
    double stepper_phase_voltage_b_right = 0.0;
    double stepper_back_emf_a_right = 0.0;
    double stepper_back_emf_b_right = 0.0;
    double stepper_electrical_power_right_w = 0.0;
    double stepper_mechanical_power_right_w = 0.0;
    double stepper_resistive_loss_right_w = 0.0;
    double stepper_magnetic_energy_right_j = 0.0;
    bool stepper_voltage_saturated_left = false;
    bool stepper_voltage_saturated_right = false;
  };

  struct LinearizedUprightModel {
    std::array<std::array<double, 4>, 4> A{};
    std::array<double, 4> horizontal_force_input{};
    std::array<double, 4> motor_force_input{};
  };

  struct StepperMassMatrix {
    double d11 = 0.0;
    double d12 = 0.0;
    double d22 = 0.0;
    double determinant = 0.0;
  };

  explicit BalancerSimulator(const Config& cfg = Config());

  void set_motor_targets(double left_sps, double right_sps);
  void set_emitted_steps(double left_steps, double right_steps);
  void set_emitted_motor_steps(double left_steps, double right_steps);
  void set_emitted_motor_step_indices(std::int64_t left_steps, std::int64_t right_steps);
  // Simulator-only analytical fixture. When set, the StepperPhase mechanical
  // equations use these two torques instead of the actuator's phase torque.
  // This lets tests compare direct generalized torque with phase-generated
  // torque without adding another production actuator path.
  void set_stepper_direct_torque_for_test(double left_torque_nm, double right_torque_nm);
  void clear_stepper_direct_torque_for_test();
  // Test-only phase-mode initialization.  The production simulator derives
  // this state from rolling mechanics; exposing it here keeps isolated
  // numerical ringdown tests on the same plant/actuator path.
  void set_stepper_relative_angles_for_test(double left_angle_rad,
                                             double right_angle_rad);
  void set_external_force_n(double force_n);
  void set_external_com_bias_rad(double com_bias_rad);
  void step(double dt_s);
  ipc::ImuRawPayload make_raw_imu_payload(uint64_t sim_time_us) const;

  const Config& config() const {
    return cfg_;
  }
  const SimulatorPhysics& physics() const {
    return physics_;
  }
  const State& state() const {
    return state_;
  }
  const Diagnostics& diagnostics() const {
    return diagnostics_;
  }
  const stepper_phase::Output& stepper_phase_output() const {
    return stepper_phase_actuator_.output();
  }
  const stepper_phase::ElectricalOutput& stepper_phase_electrical_output() const {
    return stepper_phase_electrical_actuator_.output();
  }
  double get_pitch() const {
    return state_.pitch;
  }
  double get_position() const {
    return state_.position;
  }

  static SimulatorPhysics physics_for_profile(PhysicsProfile profile);
  static std::string_view profile_name(PhysicsProfile profile);
  static StepperMassMatrix stepper_mass_matrix(
      const SimulatorPhysics& physics, double pitch_rad, double total_mass_scale = 1.0,
      double first_mass_moment_scale = 1.0, double pitch_inertia_scale = 1.0);
  static LinearizedUprightModel linearized_upright_model(const SimulatorPhysics& physics);
  static std::array<double, 4> overdamped_candidate_poles(const SimulatorPhysics& physics);

 private:
  void step_once(double dt_s);
  static StepperMassMatrix stepper_mass_matrix_with_cosine(
      const SimulatorPhysics& physics, double cos_pitch, double total_mass_scale,
      double first_mass_moment_scale, double pitch_inertia_scale);

  Config cfg_;
  SimulatorPhysics physics_{};
  State state_{};
  double left_target_sps_{0.0};
  double right_target_sps_{0.0};
  double actual_wheel_velocity_{0.0};
  double applied_drive_force_{0.0};
  double actuator_time_s_{0.0};
  double delayed_command_sps_{0.0};
  struct DelayedCommand {
    double time_s = 0.0;
    double command_sps = 0.0;
  };
  std::deque<DelayedCommand> command_history_;
  double wheel_position_m_{0.0};
  double wheel_velocity_mps_{0.0};
  double emitted_steps_avg_{0.0};
  double emitted_left_steps_{0.0};
  double emitted_right_steps_{0.0};
  double continuous_field_left_steps_{0.0};
  double continuous_field_right_steps_{0.0};
  bool have_external_emitted_step_indices_{false};
  double missed_distance_m_{0.0};
  bool have_external_emitted_steps_{false};
  std::optional<std::array<double, 2>> stepper_direct_torque_for_test_;
  double external_force_n_{0.0};
  double external_com_bias_rad_{0.0};
  stepper_phase::Actuator stepper_phase_actuator_;
  stepper_phase::ElectricalActuator stepper_phase_electrical_actuator_;
  Diagnostics diagnostics_{};

 public:
  struct HardwareNominal {
    static constexpr double gravity = 9.81;
    // Use the production geometry as the single source of truth.  The
    // measured wheel radius is 41.2 mm (82.4 mm diameter).
    static constexpr double wheel_radius = ::Config::wheel_diam_m / 2.0;
    static constexpr double motor_count = 2.0;
    // Manufacturer holding torque is specified at rated current in each
    // winding.  StepperPhase uses the Euclidean two-winding current-vector
    // amplitude, so the vector torque/back-EMF constants are normalized by
    // sqrt(2) rather than treating 0.45 / 1.5 as a vector constant.
    static constexpr double motor_stall_torque_nm = 0.45;
    static constexpr double motor_rated_phase_current_a = 1.5;
    static constexpr double stepper_sqrt_two = 1.4142135623730950488;
    static constexpr double stepper_torque_constant_nm_per_a =
        motor_stall_torque_nm / (stepper_sqrt_two * motor_rated_phase_current_a);
    static constexpr double stepper_back_emf_constant_v_per_rad_s =
        stepper_torque_constant_nm_per_a;
    static constexpr double combined_stall_force_n =
        motor_count * motor_stall_torque_nm / wheel_radius;
    static constexpr double total_mass_kg = 1.032;
    // Measured complete-robot COM height is 60 mm above the axle: H = T * z_c.
    static constexpr double first_mass_moment_kg_m = 0.06192;
    // Best current passive-pendulum estimate; controller retuning is expected around this value.
    static constexpr double pitch_inertia_about_axle_kg_m2 = 0.0045;
    // Production/controller kinematics are the authoritative 1/32 path.
    static constexpr double full_steps_per_rev =
        static_cast<double>(::Config::motor_full_steps_per_rev);
    static constexpr double microsteps_per_full_step =
        static_cast<double>(::Config::microsteps_per_full_step);
    static constexpr double microsteps_per_rev =
        full_steps_per_rev * microsteps_per_full_step;
    static constexpr double steps_per_rev = ::Config::steps_per_rev;
    static constexpr double meters_per_step = ::Config::meters_per_step;
    // StepperPhase uses exactly the production command kinematics.  Its
    // equivalent wheel radius is derived from the same production
    // meters-per-step value so field motion and applied torque use one scale.
    // The older wheel_radius remains unchanged for the other simulator
    // profiles and their historical plant semantics.
    static constexpr double stepper_phase_microsteps_per_full_step =
        steps_per_rev / full_steps_per_rev;
    static constexpr double stepper_phase_steps_per_rev = steps_per_rev;
    static constexpr double stepper_phase_meters_per_step = ::Config::meters_per_step;
    static constexpr double stepper_phase_wheel_radius =
        stepper_phase_meters_per_step * stepper_phase_steps_per_rev /
        (2.0 * 3.14159265358979323846);
    static constexpr double stepper_phase_radians_per_step =
        2.0 * 3.14159265358979323846 / stepper_phase_steps_per_rev;
    static constexpr double stepper_phase_electrical_cycles_per_rev = full_steps_per_rev / 4.0;
    // The diagnostic current LUT is 1/32-equivalent. Production STEP commands
    // advance it by one state at the verified microstep setting.
    static constexpr double stepper_phase_reference_lut_states_per_cycle =
        4.0 * microsteps_per_full_step;
    static constexpr double stepper_phase_lut_state_stride =
        microsteps_per_full_step / stepper_phase_microsteps_per_full_step;
    static constexpr double stepper_phase_electrical_radians_per_step =
        stepper_phase_radians_per_step * stepper_phase_electrical_cycles_per_rev;
    // Wheel/hub/O-ring plus motor rotor inertia per side. The maintained
    // characterization estimate is 2.7e-5 + 5.4e-6 kg m^2.
    static constexpr double stepper_rotating_inertia_kg_m2_per_motor = 3.24e-5;
    static constexpr double stepper_current_limit_a = 1.065;
    static constexpr double stepper_peak_torque_nm_per_motor =
        stepper_torque_constant_nm_per_a * stepper_current_limit_a;
    static constexpr double stepper_combined_force_n =
        motor_count * stepper_peak_torque_nm_per_motor / stepper_phase_wheel_radius;
    // Midpoint of the provisional optical ringdown damping range, derived
    // from the one-wheel fixture's phase stiffness and rotating inertia.
    static constexpr double stepper_motor_relative_damping_nm_s_per_rad = 0.0027;
  };
};
