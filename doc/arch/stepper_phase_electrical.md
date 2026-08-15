# StepperPhaseElectrical simulator scenario

This document describes the maintained physical actuator scenario at a level
that can be reproduced without reading the simulator implementation. The
first-principles audit and controller-development result are recorded in the
[StepperPhase audit report](stepper_phase_audit.md) and the current
[controller gain-scale audit](stepper_gain_scale_audit.md).

## Purpose

The scenario answers two separate questions:

1. Does the actuator produce torque from magnetic-field/rotor phase rather than
   from an artificial force proportional to SPS?
2. How do finite winding-current dynamics affect a known-attitude balance
   disturbance?

The corrected model is suitable as a first-order inner-loop development plant:
its fixed-stator/free-wheel mode agrees with the retained hardware frequency
and damping range, and its ideal-current and electrical paths pass the same
mechanical and energy checks. It remains an averaged driver model, not a final
hardware-gain authority.

## Authoritative configuration

Use the shared production geometry and the maintained StepperPhaseElectrical
parameters:

| Quantity | Value |
| --- | ---: |
| motor full steps/revolution | 200 |
| microsteps/full step | 32 |
| commanded STEP/revolution | 6400 |
| wheel radius | 0.0412 m |
| wheel circumference | `2*pi*0.0412` m |
| wheel travel/STEP | approximately 40.448 µm |
| total mass `M` | 1.032 kg |
| first mass moment `H` | 0.06192 kg·m |
| pitch inertia about axle `J` | 0.0045 kg·m² |
| attached rotating inertia/side | 3.24e-5 kg·m² |
| driver current limit | 1.065 A nominal |
| phase resistance | 2.3 Ω |
| phase inductance | 4.4 mH |
| torque constant, vector-current convention | 0.212132 N·m/A, derived |
| back-EMF constant, same convention | 0.212132 V/(rad/s), derived |
| peak modeled torque at 1.065 A, per motor | 0.225921 N·m, derived |
| peak modeled two-motor rim force | 10.967 N, derived |
| nominal bus voltage | 11.1 V |
| motor-relative damping | 0.0027 N·m·s/rad per side, provisional |
| balance command/scheduler ceiling | 16000 SPS, verified 1/32 migration |

The focused inner-loop comparison historically used `pitch_accel_gain = 0`,
the existing rate notch and estimator, and neutral velocity/drive/turn/COM
paths. The shared `pid.conf` now contains the rounded current electrical-model
profile: `203550 / 1932 / 14.7`, with the selected outer-loop values recorded
in the tuning report. Full-precision values remain in
`tests/data/stepper_phase_electrical_tuned_pid.conf`; the explicit historical
DirectActuator reference is `tests/data/direct_actuator_pid.conf`.

The shared profile is simulator-validated controller evidence, not a substitute
for hardware safety validation.

## Simulated time domains

The scenario has three independent simulated clocks:

1. The controller and estimator sample at the hardware-like 400 Hz rate.
2. The pulse layer schedules individual timestamped STEP/DIR events. A STEP
   event advances the commanded magnetic microstep index by one; SPS is never
   converted directly into force.
3. Between events, the continuous plant evolves. Chassis translation, body
   pitch/rate, attached wheel/rotor mechanics, and winding currents are
   integrated until the next event or controller sample.

The physical integrator may take internal substeps, but every STEP timestamp
is processed exactly. The standard controller telemetry remains sampled at the
400 Hz controller clock; a diagnostic trace may additionally record event-time
field, current, torque, and mechanical states.

## Actuator and plant model

For each motor, the signal path is:

```text
controller SPS request
  -> timestamped STEP/DIR events
  -> commanded 1/32 microstep field position
  -> normalized two-phase current references
  -> voltage-limited R/L current evolution with back-EMF
  -> electromagnetic torque from actual phase currents and rotor angle
  -> rigid no-slip wheel/body equations
  -> chassis and body state
```

The motor shaft is rigidly attached to the wheel. Let `x` be chassis
translation in the robot-forward direction, `theta` chassis pitch, and `r`
the wheel radius. The absolute wheel/rotor angle and speed are `x/r` and
`xdot/r`; the rotor/stator-relative angle and speed are
`q = x/r - theta` and `qdot = xdot/r - thetadot`. The commanded field angle
is kept separate from the actual relative rotor angle. The electrical rotor
angle is `50*q`, and one 1/32 STEP advances the field by `2*pi/128` electrical
radians.

For current-vector amplitude `I = hypot(i_a, i_b)`, the electrical path uses

```text
i_ref = I [cos(field_electrical_angle), sin(field_electrical_angle)]
tau = Kt (-i_a sin(rotor_electrical_angle)
          + i_b cos(rotor_electrical_angle))
e   = Ke omega_relative [-sin(rotor_electrical_angle),
                         cos(rotor_electrical_angle)]
```

`Ke = Kt` is required by the electrical/mechanical power identity in this
representation. The constant-voltage winding update is `L di/dt = v - R i -
e`, with `v` bounded by the 11.1 V bus and the requested current bounded by
the reachable interval for the physical substep.

The averaged driver model chooses an admissible bridge voltage within
`+/-V_bus` to move each actual phase current toward its indexed reference. The
winding update follows the constant-voltage R/L solution over each physical
substep. Back-EMF is derived from motor-relative shaft speed. Torque is
calculated from actual currents, so current tracking can lag or saturate
naturally without adding PWM switching details.

The constrained mechanical kinetic energy is

```text
T = 1/2 (M + 2 I_w/r^2) xdot^2
    + H cos(theta) xdot thetadot
    + 1/2 J thetadot^2
```

The wheel/rotor inertia therefore enters the absolute translation coordinate,
not the relative magnetic phase coordinate. A motor torque `tau` contributes
`+tau/r` to the translation equation and `-tau` to the body-pitch equation;
the damping torque is included in that same pair exactly once.

No tire compliance, slip, battery sag, DRV8825 chopping, structural resonance,
or fitted balance-specific force term is part of this scenario.

## Reproducible actuator checks

Run these checks before interpreting a balance trace:

1. **Geometry:** 6400 STEP events advance one wheel revolution and one event
   advances 0.05625° of mechanical motor command.
2. **Holding:** advance the field, stop STEP events, mechanically displace the
   rotor/body, and verify that restoring torque remains present at zero SPS.
3. **Phase curve:** hold the field fixed and sweep phase error. Torque should
   be bounded and sinusoidal, with the expected sign for both directions.
4. **Electrical transient:** apply one STEP from an indexed equilibrium and
   record current references, actual currents, bridge voltage, phase error,
   and torque until the transition settles.
5. **Tracking:** command a modest constant STEP rate under light load. The
   field and mechanical velocities should converge without a persistent force
   proportional to SPS.
6. **Resolution:** repeat the electrical/physical trace with a halved maximum
   physical integration interval and confirm that current, torque, and phase
   results are materially unchanged.

## Fixed-field ringdown

Disable the controller, estimator, velocity loop, COM logic, and STEP updates.
Hold the commanded field at a valid microstep index. Apply a small prescribed
relative displacement corresponding to approximately 1°, 2°, and 5° of
electrical phase error, then release.

Record:

- phase error and its frequency;
- commanded field and actual rotor angle;
- phase currents and current references;
- electromagnetic torque;
- wheel/body motion;
- energy or amplitude decay.

Run the same fixture with the ideal-current diagnostic actuator. The two
profiles should share the same mechanical phase topology; differences in
current tracking, torque timing, and dissipation identify what the electrical
stage contributes.

The current audit gives the one-stator/one-wheel fixture:

| Quantity | Result |
| --- | ---: |
| one-motor small-signal magnetic stiffness | 11.2960 N·m/rad |
| analytical natural frequency | 93.9746 Hz |
| ideal-current numerical frequency | 93.7417 Hz |
| electrical numerical frequency | 93.7417 Hz |
| modeled damping ratio | 0.0706 |
| retained hardware frequency | 93–95 Hz |
| retained hardware damping ratio | 0.04–0.08, provisional |

The full coupled two-motor body mode has `22.5921 N·m/rad` stiffness and a
different `62.6211 Hz` natural frequency; it is not the fixed-stator fixture
used for hardware correlation.

The frequency discrepancy between the old `Kt = 0.30` convention and the
hardware fixture was approximately 111.6 Hz versus 93–95 Hz. Replacing it by
`0.45/(sqrt(2)*1.5) = 0.212132` changes the prediction by `1/sqrt(2)` and
accounts for the observed fixture mode. The remaining numerical shift is the
small damped-frequency shift and time-discretization error, not a fitted
force term.

## Known-attitude recovery

Initialize the actual and fused pitch to the same value, with zero pitch rate
and zero wheel velocity. Preload the field/rotor state consistently. Use a
fixed equilibrium pitch target and keep velocity, drive, turn, and COM paths
inactive.

Run symmetric releases at `+1°` and `-1°` first, then optionally `+/-2°`,
`+/-4°`, and `+/-6°` only after the smallest case is numerically stable.

Capture at least:

- pitch and pitch rate;
- requested SPS and emitted STEP timestamps;
- commanded field angle/velocity;
- actual motor-relative angle/velocity;
- electrical phase error;
- current reference and actual phase currents;
- phase voltage and voltage saturation;
- motor torque and mechanical acceleration;
- pitch outcome and time to settle/fall.

The primary diagnostic is the causal relationship between field motion,
mechanical rotor motion, phase error, torque sign, and body recovery. A fall is
reported with that mechanism; it is not corrected by adding an arbitrary
SPS-to-force mapping.

With the corrected plant, the historical `6000/350`, `12000/700`, and
`24000/1400` pairs still fall from ±1°. A focused electrical-plant tune found
a symmetric local ±1° region near `K_pitch = 160000–300000` and
`K_rate = 8000–16000` SPS units. The selected compact-frontier point is
`280000/12000`; it recovers quietly through ±4° with the 16000-SPS authority,
while ±6° and ±8° eventually fall after rail saturation. Those values are
historical audit points. The current checked-in `pid.conf` is the later rounded
`203550 / 1932 / 14.7` electrical-model profile; the exact candidate and the
explicit DirectActuator reference are documented above.

## Hardware-correlation reference

The retained optical fixture data contains extracted optical timestamps and
the commanded small-step event schedule. The supported reference is a fixed
body/stator with one free wheel and one active motor. Use the data README for
the retained columns and the calculation of the mechanical ringdown frequency.

The optical data is a bounded correlation reference. It does not provide an
independent absolute torque measurement, and the original source video and
transient extraction script are not part of the repository.

## Interpretation boundary

DirectActuator remains an optimistic controller/reference fixture. The
ideal-current StepperPhase actuator remains useful for isolating phase
mechanics. StepperPhaseElectrical is now the primary first-order plant for
inner-loop controller development, with the explicit limits that its driver is
averaged, structural and tire effects are absent, and the retained hardware
fixture does not identify absolute torque or high-speed current-regulator
behavior. The former `IdealForce`/`ideal_force` profile name is retained only
as a compatibility alias; maintained source and reports use `DirectActuator`.
