# Control and plant model

This is the authoritative description of the robot's physical coordinates,
aggregate rigid-body model, and controller-facing equations. It is deliberately
model-neutral: it does not choose a simulator actuator realization, a PID
file, or a tuning result. Those belong to the simulator profile and behavioral
test documents.

The implementation mapping below describes the production controller and the
common plant interface. Exact message fields come from the generated
[IPC protocol](../ipc/protocol.md); exact behavior is enforced by the source
and tests named here.

> [Open the interactive plant view](interactive_balancer_plant.html)

## Aggregate simulator parameters

The nonlinear simulator uses aggregate rigid-body parameters rather than an
arbitrary split between cart and body mass: total translating mass `T`, first
mass moment `H` about the axle, and pitch inertia `J` about the axle. Its
common rigid-body mass matrix is

$$
D(\theta)=
\begin{bmatrix}
T & H\cos\theta \\
H\cos\theta & J
\end{bmatrix}.
$$

The maintained nominal values used by the simulator's common plant are:

| Quantity | Value |
| --- | ---: |
| gravity `g` | `9.81 m/s²` |
| total translating mass `T` | `1.032 kg` |
| first mass moment `H` | `0.06192 kg m` |
| pitch inertia `J` | `0.0045 kg m²` |
| wheel radius `r` | `0.0412 m` |
| nominal chassis damping | `1 N s/m` |

The authoritative code constants are in
[`HardwareNominal`](../../tests/simulator/balancer_simulator.h). The `J`
value comes from the retained passive-pendulum measurement in
[`20260722_pitch_inertia`](../../data/hardware_sessions/20260722_pitch_inertia/).
That experiment was supported at the axle with the wheels held against motor
detent/friction, so it is a provisional pitch-inertia measurement rather than
a friction-free wheel-spin calibration.

Actuator-specific additions to this common model—such as absolute wheel/rotor
inertia, electrical current states, phase coordinates, force realization, or
command delay—are owned by the corresponding simulator profile. They must not
be silently folded into the aggregate equations above.

The full nonlinear plant includes gravity, the mass matrix, chassis damping,
external horizontal force and COM-bias disturbances, wheel/tire coupling where
the selected profile provides it, and the selected actuator's applied torque
or force. It is richer than the compact audit model below; the compact model
is for signs, poles, and local parameter checks.

## Variables and compact-model assumptions

- $x$, $\dot{x}$, $\ddot{x}$: forward wheel-axle position, velocity, and
  acceleration
- $\theta$, $\dot{\theta}$, $\ddot{\theta}$: body pitch angle from upright,
  pitch rate, and pitch angular acceleration
- $n$, $\dot{n}$: common wheel speed in RPM and its rate in RPM/s
- $r$: wheel radius
- $g$: gravitational acceleration
- $T$: total translating mass
- $H$: first mass moment about the axle
- $J$: pitch inertia about the axle
- $m$, $l$, $I$: equivalent body mass, axle-to-COM distance, and body inertia
  used only by the compact small-angle form
- $k$, $a$, $b$: compact-model constants defined below
- $s$: Laplace variable; $X(s)$, $\Theta(s)$, and $N(s)$: transforms of
  $x(t)$, $\theta(t)$, and $n(t)$
- $N_{steps}$: configured motor steps per revolution in the completed-step
  observer correction
- $q_{steps,L/R}$: completed left/right motor-step counts
- $v_{user}$, $v_{ref}$, $a_{ref}$: user velocity, planned velocity, and
  transition-derived planned acceleration
- $v_{feedback}$: slow velocity-feedback estimate derived from corrected
  completed steps; $K_v$: velocity feedback gain
- $a_{raw}$, $a_{cmd}$: shared acceleration request before and after motion
  authority limiting
- $\theta_{ref}$: requested pitch angle; $\theta_{COM}$: fixed physical COM
  trim, with optional adaptive learning
- $\theta_f$, $\dot{\theta}_f$, $\ddot{\theta}_f$: filtered pitch, rate, and
  acceleration signals used by the attitude controller
- $K_{pitch}$, $K_{rate}$, $K_{accel}$: independent state-feedback gains;
  $u_{sps}$: common wheel command in steps/s

The compact audit model assumes:

- no gearbox;
- no slipping;
- both wheels have identical speed;
- small angle around upright;
- zero initial conditions for the Laplace transfer functions.

## Compact constants and small-angle equations

$$
k=\frac{2\pi r}{60},\qquad
a=\frac{mgl}{I+ml^2},\qquad
b=\frac{ml}{I+ml^2}\frac{2\pi r}{60}.
$$

The local audit equations are

$$
\ddot{x}=k\dot{n},\qquad
\ddot{\theta}=a\theta-b\dot{n}.
$$

Equivalently,

$$
\ddot{x}=\frac{2\pi r}{60}\dot{n},\qquad
\ddot{\theta}=\frac{mgl}{I+ml^2}\theta
-\frac{ml}{I+ml^2}\frac{2\pi r}{60}\dot{n}.
$$

The associated transfer functions are

$$
\boxed{\frac{X(s)}{N(s)}=\frac{k}{s}},\qquad
\boxed{\frac{\Theta(s)}{N(s)}=-\frac{bs}{s^2-a}}.
$$

The important local derivatives are

$$
\frac{\partial\ddot{\theta}}{\partial\theta}=a,\qquad
\frac{\partial\ddot{\theta}}{\partial\dot{n}}=-b,
$$

$$
\frac{\partial\ddot{x}}{\partial\dot{n}}=k,qquad
\frac{\partial\ddot{x}}{\partial n}=0.
$$

The open-loop pitch poles are

$$
s=\pm\sqrt{a}=\pm\sqrt{\frac{mgl}{I+ml^2}}.
$$

One pole is positive, so upright balance is open-loop unstable. The transfer
function also has a zero at $s=0$.

## Controller-facing model

The physical actuator is reached through the existing attitude loop. The
outer motion controller requests a pitch target; it does not bypass the inner
balance authority.

### Inner attitude loop

The production inner loop keeps the state-feedback terms independent:

$$
e_\theta=\theta-\theta_{target},\qquad
u_{sps}=K_{pitch}e_\theta
       +K_{rate}\dot{\theta}_f
       +K_{accel}\ddot{\theta}_f.
$$

The configured units are `SPS/rad`, `SPS/(rad/s)`, and
`SPS/(rad/s²)`. Positive wheel acceleration produces negative initial body
pitch acceleration; the motor-boundary signs above therefore implement the
required negative feedback. The acceleration term is optional and remains an
independent weight; its value is a profile/configuration decision, not a plant
constant.

### MotorRunner zero-speed and reversal behavior

`MotorRunner` keeps each wheel's logical direction latched separately from
its nonnegative pulse-rate magnitude. A signed target in the opposite
direction first slews the magnitude to exactly zero; the slew state never
crosses through zero in one update. The stepper remains enabled during this
stationary hold.

After reaching zero, a direction change requires two fresh opposite control
observations and approximately one accumulated requested physical step. The
qualification credit uses nominal control periods rather than elapsed wall
time, so a delayed callback cannot turn scheduler latency into reversal
authority. A qualified change accounts pulses already emitted from both
queued wheels, rebuilds the paired wave queue, changes only the required DIR
pins, and then accelerates from zero.

The fractional pulse accumulator is scheduler state, not motor electrical
phase. When queued frames are canceled, it is restored to the prefix that
actually executed so future fractional-rate pulses are neither duplicated nor
discarded.

The simulator primes its initial DIR from the first nonzero command while no
step has yet been emitted. This avoids making a zero-time, no-motion fixture
depend on an arbitrary GPIO startup level; all subsequent direction changes
use the same reversal qualification and queue-accounting rules as hardware.

### Completed-step velocity observer

At the outer-loop cadence, common completed motor steps are corrected for
pitch-induced axle motion:

$$
\Delta u_{steps}=
\frac{\Delta q_{steps,L}+\Delta q_{steps,R}}{2}
 +\frac{N_{steps}}{2\pi}\mathrm{wrap}(\Delta\theta).
$$

The corrected axle velocity passes through the compiled measurement filter
and then through a separate, configurable velocity-feedback pole. The
measurement filter describes signal conditioning; the slower pole is an
outer-loop bandwidth choice and must not be described as the physical
observer's bandwidth.

After startup, counter discontinuity, invalid timing, or another observer
reset, the observer is cleared and marked invalid. Feedback remains zero until
the first valid completed-step interval seeds the estimate. A diagnostic
completed-step average maintained by the motor service is not automatically
controller feedback.

### Velocity-reference outer loop

The normalized forward command is a persistent user velocity request:

```text
v_user = normalized_forward_command * drive_max_velocity_mps
```

The pure planner maintains `v_ref` and `a_ref`. It brakes through zero before
changing direction, selects acceleration versus deceleration by speed
magnitude, and derives both values from a kinematically consistent trajectory:

```text
active_target = v_user
if v_ref and v_user have opposite signs:
    active_target = 0

slowing_down = abs(active_target) < abs(v_ref)
limit = slowing_down ? max_deceleration : max_acceleration
v_ref_new = jerk_limited_step(v_ref, a_ref, active_target, limit, jerk, dt)
a_ref = acceleration_at(v_ref_new)
```

The planner never overshoots its active target. When jerk is configured, it
starts the terminal deceleration early enough that `v_ref` and `a_ref=0`
arrive together; it does not publish a nonzero acceleration while holding a
constant target. For the velocity-feedback error, the planner reference is
passed through the same measurement and control poles as the measured
completed-step velocity:

```text
v_feedback_reference = velocity_control_filter(measurement_filter(v_ref))
e_v = v_feedback_reference - v_feedback
```

The raw planner `v_ref` and transition acceleration remain available for
feedforward and diagnostics.

Velocity feedback and motion authority share one acceleration budget:

$$
e_v=v_{ref}-v_{feedback},\qquad a_{fb}=K_v e_v
$$

$$
a_{raw}=a_{ref}+a_{fb},
$$

$$
a_{cmd}=\mathrm{clamp}\left(
a_{raw},\;-g\tan\theta_{outer,max},\;+g\tan\theta_{outer,max}\right),
$$

$$
\theta_{drive}=\mathrm{atan2}(a_{cmd},g),\qquad
\theta_{target}=\theta_{drive}+\theta_{COM}.
$$

The optional velocity integral, when enabled by a profile, is bounded and
anti-windup protected. It is not part of the first-order plant model. Adaptive
COM learning is likewise a controller feature, not an actuator or plant
parameter; fixed trim is composed after motion pitch.

The outer authority limit applies only to motion demand. It must not lower the
independent balance recovery clamp. Observer validity gates feedback, while a
joystick watchdog only changes the user request to zero so the planner can
perform normal braking.

## Timing, safety, and control authority

The production loop intentionally uses several clocks:

- IMU sampling: approximately `833 Hz`;
- inner attitude controller: `400 Hz`;
- outer planner, observer, and velocity feedback: `100 Hz`;
- compiled completed-step measurement filter: approximately `10 Hz`.

The exact actuator frame and pulse scheduling are profile-specific. The
controller's balance ceiling, turn allocation, pitch safety limits, stale/future
IMU checks, actuator-fault handling, fallover boundary, and re-arm conditions
remain independent of the motion-pitch authority limit.

The final target still passes through the existing safety path, including the
controller setpoint limit, fallover boundary, and re-arm checks. A controller
candidate must preserve inner recovery authority while reserving actuator
headroom for translation and turning.

## Validation boundary

This page defines the common plant and control equations. The following pages
own the questions that are intentionally outside this standard:

- the maintained electrical actuator realization and its fixed constants:
  [StepperPhaseElectrical test profile](../testing/stepper_phase_electrical.md);
- cross-profile scenario definitions, strict xfails, and pass/fail evidence:
  [simulator behavioral matrix](../testing/simulator_behavioral_matrix.md);
- retained hardware captures and their claim limits:
  [hardware data guide](../../data/README.md).

The linearized audit is run with:

```bash
./build/balancer_plant_audit --all
```

Controller equations and reset paths are covered by
[`tests/control_loop_test.cpp`](../../tests/control_loop_test.cpp). Do not use
this page's compact equations as permission to change simulator physics or to
select hardware gains.
