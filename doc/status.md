# Current status

Snapshot: **2026-08-17**.

The current software comparison surface is the `StepperPhaseElectrical` plant
with the v12 velocity-reference outer loop. The old `DirectActuator` profile is
still useful as a frozen controller-capability reference, but it is not the
physical model used to judge the robot.

## Verified in the repository

- The host gate is `pytest --build`.
- The latest host gate completed with CTest passing, `130` Python tests passed,
  `7` intentional skips, and `11` expected strict xfails.
- The C++ suite completed with `259/259` tests passed.
- The focused behavioral matrix reports `23 passed, 4 skipped, 8 strict
  xfailed` for the electrical profile and `31 passed, 3 skipped, 1 strict
  xfailed` for DirectActuator.
- The electrical model uses the production 1/32 geometry: `6400 STEP/rev`,
  `40.448 µm/STEP`, and a fixed `16000 SPS` balance ceiling.
- The electrical actuator model includes timestamped STEP events, magnetic
  field/rotor phase, voltage-limited winding currents, back-EMF, torque, and
  completed-step feedback. It does not convert SPS directly into force.
- The v12 controller uses a reversal-aware velocity reference, shared
  acceleration authority, velocity P feedback, optional bounded leaky I, and
  fixed COM trim with adaptive trim disabled in the checked-in profile.
- The current shared [`pid.conf`](../pid.conf) is `203550 / 1932 / 0` inner
  gains, `K_v=8`, a `3 Hz` velocity-feedback pole, `10°` outer authority, and
  zero I. It is the golden StepperPhaseElectrical simulator profile; it is not
  a hardware deployment approval.

## Hardware evidence and limits

The retained hardware evidence supports two important conclusions:

- the free-wheel actuator mode is approximately `93–95 Hz`, matching the
  electrical simulator fixture near `93.74 Hz`;
- zeroing `pitch_accel_gain` materially reduced the observed balance vibration,
  while reducing pitch/rate gains alone had much less effect.

The retained wood-floor neutral capture contains a bounded approximately
`2.3065 Hz` closed-loop rocking mode. It is evidence about that historical
run, not a current simulator parameter or a reason to move the electrical
actuator notch to the 93–95 Hz mode. Hardware captures are not plant-fit data
unless their manifest includes the source, PID snapshot, repository revision,
command semantics, and selection procedure.

The robot has not been authorized for unattended validation of the new outer
loop. Simulator success does not prove that a configuration is safe on the
physical robot, especially near command or balance authority limits.

## Remaining engineering question

The broad simulator search found a stable P-only region and did not justify
enabling integral action. The golden profile produces useful signed motion at
`cart_damping = 1`; its nominal full-forward run travels about `0.61 m` for
about `0.56 m` of reference travel, with roughly 90% hold-velocity tracking.
The remaining direct constant-lean and delayed-estimator cases are explicit
simulator boundaries. Static friction and COM behavior remain separate future
model/hardware investigations.

## Confidence boundaries

### Strong

- reflected interfaces, generated bindings, protocol documentation, and host
  build wiring;
- controller algebra, signs, reset behavior, observer validity, and telemetry
  plumbing;
- electrical geometry/current/power invariants and timestamped STEP scheduling;
- repeatable simulator behavior under the named scenarios.

### Moderate

- Raspberry Pi deployment and neutral restrained bring-up;
- the electrical model's first-order correspondence to the stepper actuator;
- the current outer-loop design under real static friction and COM bias.

### Requires hardware validation

- unrestrained balancing with the current v12 profile;
- high-gain outer-loop candidates;
- translating-push recovery and actual motor headroom;
- whether the simulator's remaining outer-loop failures are caused by friction,
  authority allocation, estimator bias, or controller architecture.

For deployment, use [Running on Raspberry Pi](Running_on_Pi.md). For common
equations and the controller law, use [Control and plant model](arch/control_plant.md);
for electrical actuator constants and profile evidence, use the
[StepperPhaseElectrical testing profile](testing/stepper_phase_electrical.md).
For retained evidence, use the [hardware data index](../data/README.md).
