# StepperPhaseElectrical testing profile

`StepperPhaseElectrical` is a simulator/testing profile for evaluating the
controller against a detailed stepper actuator realization. It is not part of
the model-neutral physical plant standard. The shared rigid-body equations and
controller-facing algebra are in [`control_plant.md`](../arch/control_plant.md); this
page owns the actuator realization, fixed comparison constants, and profile
test procedure.

## What this profile represents

The profile models the actual 1/32 stepper command path rather than treating
SPS as a force input:

```text
SPS target
  -> timestamped STEP/DIR events
  -> magnetic field position
  -> voltage-limited phase currents
  -> phase/rotor torque
  -> coupled wheel/body/tire plant
  -> completed-step velocity feedback
```

The field and rotor remain separate. Current dynamics, back-EMF, phase error,
relative rotor speed, torque, missed-step behavior, and the nonlinear mass
matrix are observable in simulator telemetry. The model is averaged at the
driver level: it does not include PWM chopping, battery sag, structural
resonance, or tire compliance.

> [Open the interactive stepper electrical ringdown](../arch/interactive_stepper_electrical.html)

The profile adds absolute wheel/rotor inertia to the common translational
coordinate and keeps motor-relative phase as a separate actuator coordinate.
With `u_wheel = r * psi` and the configured initial pitch as the zero-step
reference:

```text
q_m     = u_wheel - r * (theta - theta_0)
qdot_m  = udot_wheel - r * thetadot
```

The rotor electrical angle is `50 * q_m`. One 1/32 STEP advances the magnetic
field by `2*pi/128` electrical radians. For phase currents `(i_a, i_b)` and
rotor electrical angle `phi`, the actuator uses:

```text
tau = Kt * (-i_a * sin(phi) + i_b * cos(phi))
e   = Ke * omega_relative * (-sin(phi), cos(phi))
L di/dt = v - R i - e
```

The averaged bridge selects a voltage in `[-V_bus, +V_bus]` to move actual
current toward the indexed field reference. This is a testing realization of
the motor command path; SPS is not converted directly into force.

## Fixed profile comparison surface

| Parameter | Maintained value |
| --- | ---: |
| wheel radius | `0.0412 m` |
| motor full steps/rev | `200` |
| microsteps/full step | `32` |
| commanded STEP/rev | `6400` |
| wheel travel/STEP | `40.448 µm` |
| electrical cycles/mechanical rev | `50` |
| total mass | `1.032 kg` |
| first mass moment | `0.06192 kg m` |
| pitch inertia about axle | `0.0045 kg m²` |
| wheel/rotor inertia per side | `3.24e-5 kg m²` |
| phase resistance / inductance | `2.3 Ω / 4.4 mH` |
| bus voltage / current limit | `11.1 V / 1.065 A` |
| `K_t = K_e` | `0.212132` in the vector-current convention |
| relative motor damping | `0.0027 N m s/rad` per side, provisional |
| balance ceiling | `16000 SPS` |

These values come from `Config`, `HardwareNominal`, and
`stepper_phase::ElectricalParameters`; they must not be duplicated as new
tuning constants in scenario definitions. See the standard page for the
common coordinate equations and the reason this profile keeps wheel/rotor
inertia in the absolute wheel coordinate.

## Profile invariants and correlation checks

The electrical actuator tests cover:

- `6400` STEP events per wheel revolution and correct 1/32 field motion;
- restoring torque while STEP is stopped and the rotor is displaced;
- signed sinusoidal phase torque;
- voltage-limited current rise and R/L convergence;
- back-EMF sign and electrical power consistency;
- no persistent force caused solely by constant field speed;
- physical-step convergence when the integration interval is halved.

The fixed-stator/free-wheel fixture predicts approximately `93.97 Hz`
analytically and `93.74 Hz` numerically. Retained hardware evidence shows a
`93–95 Hz` free-wheel mode and provisional damping around `0.04–0.08`; the
electrical fixture gives `0.0706`. The coupled two-motor body mode is
approximately `62.62 Hz` and is a different fixture. These are actuator/model
correlation checks, not controller tuning targets and not grounds for changing
the controller notch without new evidence.

The retained optical input is in
[`data/hardware_sessions/motor_tracking`](../../data/hardware_sessions/motor_tracking/).
It supports frequency, phase-sign, and current/torque timing checks. It does
not support fitting controller gains or adding a per-microstep correction.

## Current controller profile

The normal electrical profile is [`pid.conf`](../../pid.conf):

```text
inner gains: 203550 / 1932 / 0 SPS units
user speed: 0.12 m/s
velocity P: 8 1/s
velocity feedback pole: 3 Hz
planner acceleration/deceleration/jerk: 0.25 / 0.25 / 1
outer pitch authority: 10 deg
fixed COM trim: 0 deg
adaptive COM trim: disabled
velocity I: disabled
turn / balance ceilings: 1600 / 16000 SPS
```

The DirectActuator comparison uses its explicit fixture at
`tests/data/direct_actuator_pid.conf`. Do not substitute that profile's gains
for the electrical plant. Do not change physics or add a `physics_override`
when evaluating controller candidates.

## Simulator clocks and scenarios

- IMU/controller state is sampled at the nominal 400 Hz cadence.
- The outer observer and velocity-reference planner run at 100 Hz.
- STEP events are timestamped within 2.5 ms motor frames and are processed at
  their event time.
- The completed-step observer uses corrected axle motion and a compiled
  approximately 10 Hz measurement filter; `velocity_feedback_cutoff_hz` is a
  separate control-path pole.

The maintained Python suite compares this profile against DirectActuator over
quiet balance, signed attitude recovery, disturbance/noise, drive/stop/
reversal, authority, COM, long-horizon, and startup cases. Composite scenarios
retain their concrete signed and uncertainty sub-runs in `build/sim` artifacts.

The current electrical result is `23 passed, 4 skipped, 8 strict xfailed` in
the focused Python matrix. The checked-in [`pid.conf`](../../pid.conf) is the
golden simulator configuration for this surface. The remaining xfails are
specific evidence boundaries: noisy/sustained-authority recovery, direct
constant-lean electrical authority, plant uncertainty, and the high-angle
startup boundary. Adaptive COM acquisition/maintenance and the generic
reduced-traction override are explicit skips rather than controller failures.

The golden nominal motion result is approximately `0.61 m` of signed travel
for `0.56 m` of active reference travel in the full-forward case, with roughly
90% hold-velocity tracking and mirrored forward/reverse behavior. This does
not imply that the direct constant-pitch diagnostic is a valid stationary-lean
test: a constant lean continuously accelerates the electrical plant and can
reach its phase/safety boundary, while the velocity-reference loop actively
arrests that motion.

## Required checks

Run the electrical actuator tests and the full host gate before changing the
comparison surface:

```bash
cmake --build build --target balancer_tests
./build/balancer_tests --gtest_filter='StepperPhase*'
pytest --build
```

For a linearized parameter audit:

```bash
./build/balancer_plant_audit --all
```

The profile is a simulator/controller-development fixture. It is closer to the
hardware actuator than the ideal reference profile, but its current constants
and any selected controller still require cautious hardware validation.
