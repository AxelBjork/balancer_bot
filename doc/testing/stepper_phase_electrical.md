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

## Source of truth

The numerical instantiation is deliberately not repeated here. It is defined
by the production and simulator sources:

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
