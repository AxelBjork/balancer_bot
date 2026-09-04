# StepperPhaseElectrical testing profile

`StepperPhaseElectrical` is a simulator/testing profile for evaluating the
controller against a detailed stepper actuator realization. It is not part of
the model-neutral physical plant standard. The shared rigid-body equations and
controller-facing algebra are in [`control_plant.md`](../arch/control_plant.md); this
page defines the actuator's normative signal flow, physics, and profile test
procedure. Numerical parameter values remain in the C++ sources.

## What this profile represents

The profile models the configured stepper command path rather than treating
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

## Actuator coordinates and equations

The profile adds absolute wheel/rotor inertia to the common translational
coordinate and keeps motor-relative phase as a separate actuator coordinate.
Let `u_wheel = r * psi`, and let `theta_0` be the configured initial pitch
reference. The relative mechanical coordinate and velocity are

$$
\begin{aligned}
q_m &= \psi-(\theta-\theta_0)
    = \frac{u_{\mathrm{wheel}}}{r}-(\theta-\theta_0),\\
\dot{q}_m &= \dot{\psi}-\dot{\theta}
    = \frac{\dot{u}_{\mathrm{wheel}}}{r}-\dot{\theta}.
\end{aligned}
$$

The electrical rotor angle and relative angular velocity are

$$
\phi=N_e q_m,\qquad \omega_m=\dot{q}_m,
$$

where `N_e` is the maintained electrical-angle ratio for one mechanical
radian. If `N_mu` is the configured number of commanded microsteps per
mechanical revolution, one STEP advances the field by

$$
\Delta\phi_{\mathrm{step}}=\frac{2\pi N_e}{N_\mu}.
$$

For field angle `phi_f`, the vector-current reference is

$$
\mathbf{i}_{\mathrm{ref}}=I_{\max}
\begin{bmatrix}\cos\phi_f\\\sin\phi_f\end{bmatrix}.
$$

For phase currents `i_a`, `i_b` and rotor electrical angle `phi`, the
actuator equations are

$$
\begin{aligned}
\tau &= K_t\left(-i_a\sin\phi+i_b\cos\phi\right),\\
\mathbf{e} &= K_e\omega_m
\begin{bmatrix}-\sin\phi\\\cos\phi\end{bmatrix},\\
L\dot{\mathbf{i}} &= \mathbf{v}-R\mathbf{i}-\mathbf{e}.
\end{aligned}
$$

The averaged bridge constrains each phase voltage to the bus limit and chooses
that voltage to move actual current toward the indexed field reference. SPS
therefore determines field-step timing; it is not converted directly into
force or torque.

The actuator returns torque to the coupled wheel/body plant. It does not
integrate an independent rotor-inertia state: the plant supplies the
constrained mechanical coordinate, while the actuator maintains the separate
relative phase coordinate.

## Coupled mechanics, support, and traction

The electrical profile retains chassis translation, body pitch, and independent
left/right wheel coordinates. Contact force is solved from those coupled
coordinates; motor torque is not converted directly to ground force.

For the symmetric, sticking, disturbance-free case, define

$$
A=T+\frac{I_{w,L}+I_{w,R}}{r^2},\qquad B=H\cos\theta,
$$

where each $I_w$ is the absolute wheel/hub/rotor inertia. Eliminating the wheel
coordinates from the full constrained solve gives the useful audit form

$$
\begin{aligned}
A\ddot{x}+B\ddot{\theta} &= \frac{\tau}{r},\\
B\ddot{x}+J\ddot{\theta} &= Hg\sin\theta-\tau.
\end{aligned}
$$

The implementation keeps the two wheel coordinates and two tire forces rather
than using only this reduction. For side $i$, its no-slip acceleration
constraint and wheel equation are

$$
\ddot{x}-r(\ddot{\alpha}_i+\ddot{\theta})=0,
$$

$$
\tau_i=rC_i+I_{w,i}(\ddot{\alpha}_i+\ddot{\theta}),
$$

so $C_i$ is a Lagrange multiplier produced by the coupled solve. The second
term is why $C_i\ne\tau_i/r$ during wheel acceleration. Chassis damping,
rolling resistance, pitch damping, brace torque, and external disturbances are
included in the implementation but omitted from the compact equations above.

Using the horizontal-ground support load defined by the simulator fixture, the
symmetric traction envelope is

$$
|C_L|\le\mu\frac{Tg}{2},\qquad
|C_R|\le\mu\frac{Tg}{2}.
$$

Requested traction utilization is therefore

$$
u_{traction}=
\max\left(
\frac{|C_L|}{\mu Tg/2},
\frac{|C_R|}{\mu Tg/2}
\right).
$$

If either multiplier exceeds its envelope, that contact force is clipped and
the independent wheel state is allowed to slip. Simulator analysis should use
the requested/actual contact-force and traction-utilization diagnostics; it
must not reconstruct utilization from motor torque divided by wheel radius.

The fixture's support-load convention and brace limitations are documented in
the [testing strategy](strategy.md#large-angle-brace-recovery).

### Large-angle numerical check

At the nominal settled `67.815°` pitch, the maintained constants give
$A\approx1.07018\ \mathrm{kg}$, $B\approx0.02338\ \mathrm{kg\,m}$, and a
gravity moment of approximately `0.56247 Nm`. Setting
$\ddot{\theta}=0$ in the coupled equations gives

$$
\tau_{threshold}=\frac{Ar}{B+Ar}Hg\sin\theta
\approx0.36756\ \mathrm{Nm},
$$

with $\ddot{x}\approx8.336\ \mathrm{m/s^2}$, total contact force
$C_L+C_R\approx8.603\ \mathrm{N}$, and required $\mu\approx0.8498$.
At the full nominal $\mu=1$ boundary, the corresponding values are
$\tau\approx0.43344\ \mathrm{Nm}$ and
$\ddot{\theta}\approx-25.272\ \mathrm{rad/s^2}$ (inward).

These values are derived verification anchors, not separately owned plant
parameters. They are enforced by
`SimulatorRunnerTest.ElectricalTractionUsesCoupledContactForceAtLargePitch` in
[`tests/simulator_runner_test.cpp`](../../tests/simulator_runner_test.cpp).

## Source of truth

The numerical parameter set is deliberately not maintained as a second table
here. It is defined by the production and simulator sources; the worked check
above is only a derived regression anchor:

- `Config` owns production wheel and command kinematics.
- `HardwareNominal` owns common plant quantities and derived StepperPhase
  nominal quantities.
- `stepper_phase::ElectricalParameters` owns the electrical model interface.
- `stepper_phase_actuator.cpp` owns the current update, torque, back-EMF, and
  power implementation.

Tests and tools must read these definitions rather than introducing copied
values in Markdown, scenario files, or alternate tuning tables. See
[`control_plant.md`](../arch/control_plant.md) for the model-neutral plant
equations and coordinate conventions.

## Profile invariants and correlation checks

The electrical actuator tests cover:

- the configured STEP count per wheel revolution and corresponding field motion;
- restoring torque while STEP is stopped and the rotor is displaced;
- signed sinusoidal phase torque;
- voltage-limited current rise and R/L convergence;
- back-EMF sign and electrical power consistency;
- no persistent force caused solely by constant field speed;
- physical-step convergence when the integration interval is halved.

The fixed-stator/free-wheel fixture is used for analytical and numerical
ringdown correlation. Retained hardware captures can provide evidence about
frequency, phase sign, and current/torque timing, but they are not a license
to fit controller gains or silently change the actuator model. The coupled
two-motor body mode is a separate fixture from the one-motor ringdown.

The retained optical input is in
[`data/hardware_sessions/motor_tracking`](../../data/hardware_sessions/motor_tracking/).
It supports frequency, phase-sign, and current/torque timing checks. It does
not support fitting controller gains or adding a per-microstep correction.

## Controller and profile boundary

Controller gains, motion limits, COM policy, and the normal electrical profile
are owned by [`pid.conf`](../../pid.conf) and the explicit test fixtures. They
are not reproduced here. The DirectActuator comparison uses its dedicated
fixture at `tests/data/direct_actuator_pid.conf`; do not substitute that
profile's controller settings for the electrical plant.

This document describes how the actuator realizes a command. It does not
authorize changing physics or adding a `physics_override` while evaluating
controller candidates.

### High-angle field command

The configured pitch and pitch-rate gains remain unchanged throughout brace
recovery. While an explicit held forward/recovery request is active, the
high-authority StepperPhase path separately feeds forward the known
chassis-rotation contribution to field velocity. For large disturbances this
term reaches `-0.88 * steps_per_rad * pitch_rate`; it blends from zero to full
between `5–10 deg` absolute pitch or `15–30 deg/s` absolute pitch rate,
whichever is larger. Below both bands, ordinary small-angle feedback is
unchanged. It is enabled only for configurations with at least a `32,000 SPS`
field cap; lower-authority reference configurations do not enter this
high-field regime. This is a coordinate conversion for a field expressed in
the rotating chassis frame, not pitch-gain scheduling or a recovery
trajectory. With no held forward request, the feedforward is zero, including
during ordinary disturbances.

### MotorRunner bounded actuator envelope

The production field trajectory has a `200,000 SPS/s` sustainable slew. A
per-wheel, actuator-only credit of `250 SPS` permits one initial `300,000
SPS/s` update at the `400 Hz` motor cadence when a same-direction growth
request targets at least `32,000 SPS`. Credit refills at `3,000 SPS/s` only
while the request is already reachable within the sustainable update. Stop
and reset restore it; a direction transition discards it. Thus scheduler
delay, an opposite-direction request, or a persistent unreachable request
cannot accumulate burst authority.

Same-direction field acceleration tapers above `20,000 SPS`, reaching `50,000
SPS/s` at the `48,000 SPS` ceiling as bus-voltage margin falls. Reducing field
magnitude above `20,000 SPS` may use up to `350,000 SPS/s`; this is bounded
high-field braking, not growth or reversal authority. Direction reversals
still brake to zero under the ordinary `200,000 SPS/s` envelope before the
DIR transition. All decisions use requested and emitted STEP history only;
`MotorRunner` has no IMU or controller-state input.

## Simulation and test boundary

The simulator owns the scheduler, timestamped STEP stream, coupled rigid-body
plant, completed-step observer, and profile-specific diagnostics. The actuator
must preserve event timing and keep controller time, mechanical time, and
electrical phase distinct.

The maintained test suite compares this profile against DirectActuator over
balance, signed recovery, disturbance, drive/stop/reversal, authority, COM,
long-horizon, and startup scenarios. Scenario outputs and retained evidence
belong in the generated artifacts and source-controlled data manifests, not in
this normative profile description.

The production high-angle end-to-end and focused in-process regressions are
defined in the [testing strategy](strategy.md#large-angle-brace-recovery).

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
