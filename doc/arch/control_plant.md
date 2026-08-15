# Control and Plant Model

## Aggregate Simulator Parameters

The nonlinear simulator uses aggregate rigid-body parameters rather than an arbitrary split between
cart and body mass: total translating mass `T`, first mass moment `H` about the axle, and pitch
inertia `J` about the axle. Its mass matrix is

$$
\begin{bmatrix}
T & H\cos\theta \\
H\cos\theta & J
\end{bmatrix}.
$$

The authoritative nominal mass, geometry, and inertia values are defined by
[`HardwareNominal`](../../tests/simulator/balancer_simulator.h). Pitch inertia should be updated
from the physical-pendulum procedure documented in the Pi runtime guide; these values are
intentionally not duplicated here.

The controller-design reference simulator uses `J = 0.0045 kg m^2`, `cart_damping = 1 N s/m`,
and direct applied-force authority. Its purpose is to evaluate the controller against the
identified rigid-body dynamics without assigning unmeasured phase-position parameters physical
meaning. Its direct N/SPS authority is an explicit reference scale, independent of wheel STEP
geometry. `StepperPhaseElectrical` is the maintained physical-actuator profile: it uses timestamped
1/32 STEP events (6400 STEP/rev), indexed field position, and voltage-limited averaged winding
currents. The
older SPS-to-force and phase/tire experiments are historical diagnostics only; their aggregate
lag, phase envelope, tire, and force-limit parameters are not calibrated well enough to select
production gains. In particular, an aggregate 20 ms response must not be interpreted as the
measured approximately 1.95 ms electrical motor time constant.

The variables and equations through the transfer-function section form a compact, linearized audit
model. The implementation mapping below describes the richer nonlinear simulator and controller;
assumptions in the compact model should not be read as claims that the simulator has no slip,
actuator lag, or left/right asymmetry.

## Variables

- $x$, $\dot{x}$, $\ddot{x}$: forward wheel-axle position, velocity, and acceleration
- $\theta$, $\dot{\theta}$, $\ddot{\theta}$: body pitch angle from upright, pitch rate, and
  pitch angular acceleration
- $n$, $\dot{n}$: common wheel speed in RPM and its rate in RPM/s
- $r$: wheel radius
- $g$: gravitational acceleration
- $T$: total translating mass
- $H$: first mass moment about the axle
- $J$: pitch inertia about the axle
- $m$: body mass; $l$: axle-to-body-COM distance; $I$: body pitch inertia about its COM. These
  are the equivalent lumped parameters used only in the compact small-angle model.
- $k$, $a$, $b$: compact-model constants defined below
- $s$: Laplace variable; $X(s)$, $\Theta(s)$, and $N(s)$: Laplace transforms of $x(t)$,
  $\theta(t)$, and $n(t)$
- $a_{\mathrm{nominal}}$: jerk-limited requested forward acceleration;
  $v_{\mathrm{axle}}$: corrected axle velocity derived from completed motor steps
- $k_v$: corrected-velocity damping gain
- $\theta_{\mathrm{ref}}$: requested pitch angle; $\theta_{\mathrm{COM}}$: bounded
  center-of-mass trim
- $\theta_f$, $\dot{\theta}_f$, $\ddot{\theta}_f$: filtered pitch, rate, and acceleration
  signals used by the explicit attitude controller
- $K_{\mathrm{pitch}}$, $K_{\mathrm{rate}}$, $K_{\mathrm{accel}}$: independent state-feedback
  gains; $u_{\mathrm{sps}}$: common wheel command in steps/s
- $\psi$: absolute wheel angle; $u_{wheel} = r\psi$: absolute circumferential wheel motion;
  $q_m$: rotor motion relative to the chassis; $q_{steps,L/R}$: completed motor-step counts used
  by the observer; $\theta_0$: configured initial pitch
- $F_m$, $F_t$: motor and tire/contact forces

Compact audit-model assumptions:

- no gearbox
- no slipping
- both wheels have identical RPM
- small-angle linearization around upright
- zero initial conditions for Laplace transfer functions

---

## Compact Constants

> $$
> k = \frac{2\pi r}{60}
> $$

> $$
> a = \frac{mgl}{I+ml^2}
> $$

> $$
> b = \frac{ml}{I+ml^2}\frac{2\pi r}{60}
> $$

---

## Small-Angle Equations of Motion

> $$
> \ddot x = k\dot n
> $$

> $$
> \ddot\theta = a\theta - b\dot n
> $$

Expanded form:

> $$
> \ddot x = \frac{2\pi r}{60}\dot n
> $$

> $$
> \ddot\theta = \frac{mgl}{I+ml^2}\theta - \frac{ml}{I+ml^2}\frac{2\pi r}{60}\dot n
> $$

---

## Transfer Functions

> $$
> \boxed{\frac{X(s)}{N(s)} = \frac{k}{s}}
> $$

> $$
> \boxed{\frac{\Theta(s)}{N(s)} = -\frac{bs}{s^2-a}}
> $$

Equivalent expanded forms:

> $$
> \boxed{\frac{X(s)}{N(s)} = \frac{\frac{2\pi r}{60}}{s}}
> $$

> $$
> \boxed{
> \frac{\Theta(s)}{N(s)} =
> -\frac{
> \left(\frac{ml}{I+ml^2}\frac{2\pi r}{60}\right)s
> }{
> s^2-\frac{mgl}{I+ml^2}
> }
> }
> $$

---

## Stability Derivatives

### Pitch Dynamics

> $$
> \boxed{\frac{\partial \ddot\theta}{\partial \theta} = a = \frac{mgl}{I+ml^2}}
> $$

> $$
> \boxed{\frac{\partial \ddot\theta}{\partial \dot\theta} = 0}
> $$

> $$
> \boxed{\frac{\partial \ddot\theta}{\partial n} = 0}
> $$

> $$
> \boxed{\frac{\partial \ddot\theta}{\partial \dot n} = -b = -\frac{ml}{I+ml^2}\frac{2\pi r}{60}}
> $$

### Translational Dynamics

> $$
> \boxed{\frac{\partial \ddot x}{\partial \dot n} = k = \frac{2\pi r}{60}}
> $$

> $$
> \boxed{\frac{\partial \ddot x}{\partial n} = 0}
> $$

> $$
> \boxed{\frac{\partial \ddot x}{\partial \theta} = 0}
> $$

> $$
> \boxed{\frac{\partial \ddot x}{\partial \dot\theta} = 0}
> $$

---

## Open-Loop Pitch Poles

From

> $$
> \frac{\Theta(s)}{N(s)} = -\frac{bs}{s^2-a}
> $$

The poles are

> $$
> \boxed{s = \pm\sqrt{a}}
> $$

that is,

> $$
> \boxed{s = \pm\sqrt{\frac{mgl}{I+ml^2}}}
> $$

Since one pole is positive, the upright system is open-loop unstable.

The transfer function also has a zero at

> $$
> \boxed{s = 0}
> $$

---

## Implementation Mapping

The equations above are the compact small-angle audit model. The current code adds damping, actuator lag, and estimator filtering on top of this idealized plant.

### Rate hierarchy

The project has several intentional rates rather than one universal loop rate:

- `PhysicsTick` and the inner rate controller run at the nominal 400 Hz control cadence.
- The completed-step axle-velocity observer, velocity filter update, jerk limiter, and COM-trim
  integration run in the 100 Hz outer-loop interval.
- The completed-step observer retains its 10 Hz measurement filter; the production controller
  adds a separate configurable velocity-control pole, currently 3 Hz, so observer bandwidth and
  translational-control bandwidth are not conflated.
- The motor runner keeps a separate 50 ms completed-step average for actuator diagnostics; the
  controller does not use that diagnostic average as its feedback observer.

This distinction prevents the 100 Hz observer, 10 Hz filter, 50 ms diagnostic window, and 400 Hz
control tick from being mistaken for competing controller clocks.

Current controller structure in code:

At 100 Hz, common completed motor steps are first corrected to axle motion. The observer is

> $$
> \Delta u_{\mathrm{steps}} = \frac{\Delta q_{steps,L} + \Delta q_{steps,R}}{2}
> + \frac{N}{2\pi}\mathrm{wrap}(\Delta\theta).
> $$

The velocity is calculated from the elapsed observer interval and passed through the configured
10 Hz single-pole filter. Filtering, jerk limiting, and COM-trim integration all use that measured
outer-loop interval, including under control-tick jitter. `MotorRunner` also maintains a separate
50 ms completed-step average for actuator diagnostics; that field is not controller feedback.

The jerk-limited nominal acceleration and corrected-velocity damping then form the pitch reference;
a bounded integral term still learns only stationary center-of-mass trim:

$$
\theta_{\mathrm{ref}} = \mathrm{clamp}\left(
\mathrm{atan2}(a_{\mathrm{nominal}} - k_v v_{\mathrm{axle}}, g) + \theta_{\mathrm{COM}}
\right)
$$

The drive-generated portion is bounded by the configured acceleration equilibrium,
`atan2(drive_max_acceleration_mps2, g)`: this is the nonlinear form of the small-angle
condition `theta_ddot = 0`. The bounded COM trim remains available at zero drive command.

Explicit attitude state feedback:

The production controller keeps pitch stiffness, pitch-rate damping, and optional acceleration
feedback independent:

> $$
> e_\theta = \theta - \theta_{\mathrm{ref}},\\
> u_{\mathrm{sps}} = K_{\mathrm{pitch}}e_\theta
> + K_{\mathrm{rate}}\dot\theta_f
> + K_{\mathrm{accel}}\ddot\theta_f.
> $$

The plus signs are the robot-forward motor-boundary form: positive wheel acceleration produces
negative initial body pitch acceleration. They are equivalent to negative feedback after including
that mechanical polarity. `pitch_gain`, `pitch_rate_gain`, and `pitch_accel_gain` are expressed in
SPS/rad, SPS/(rad/s), and SPS/(rad/s²). Acceleration feedback is supported but zero in the checked-in
default. The separate `velocity_control_cutoff_hz` pole keeps translational feedback below the fast
attitude path.

Motor scaling and safety:

- the outer loop requests motion only through the pitch reference; it adds no direct wheel-speed
  feed-forward
- `balance_max_sps` is the hard common-mode clamp, then turn allocation consumes only remaining
  balance authority; `drive_max_sps` and `turn_max_sps` remain explicit safety/allocation limits
- the motor runner limits command slope to 200,000 SPS/s and applies pulses through two synchronous
  2.5 ms frames (one active and one queued); telemetry separates the target, continuous post-slew
  command, and quantized active-frame rate
- positive wheel acceleration produces negative initial pitch acceleration, while sustained
  positive vehicle acceleration requires positive equilibrium pitch; motor output may therefore
  initially reduce or reverse while acquiring a forward lean

Notes:

- `theta_dot_f` is the 30 Hz two-pole filtered pitch gyro rate, not the raw gyro debug signal;
  its controller derivative input is filtered separately at 10 Hz
- the simulator now applies disturbances as exogenous plant inputs:
  external horizontal force and optional COM bias
- the simulator plant is intentionally richer than the cheat-sheet model: scheduled pulses advance
  commanded motor phase; completed pulses supply controller feedback; a separate rotor/wheel state,
  torque-speed limit, actuator lag, missed-step estimate, and traction-limited tire coupling produce
  force on the nonlinear cart-pole
- the simulator's continuous field-speed term consumes the post-slew command; exact pulse-frame
  quantization enters separately through emitted motor position and is not applied twice

### Simulator wheel and motor coordinates

The nonlinear simulator keeps the wheel coordinate `u_wheel = r psi` as absolute circumferential wheel
motion. Tire deformation and tire speed therefore remain $u_{wheel} - x$ and
$\dot u_{wheel} - \dot x$.
Motor steps,
however, measure rotor motion relative to the chassis. With the configured initial pitch as the
zero-step reference, the motor coordinates are

> $$
> q_m = u_{wheel}-r(\theta-\theta_0), \qquad \dot q_m = \dot u_{wheel}-r\dot\theta.
> $$

The phase-error controller and torque-speed limit use `q_m` and `q_m_dot`. A motor force `F_m` applies
`+F_m` to the wheel and the equal-and-opposite torque `-r F_m` to the chassis. The tire/contact force
and motor force are consequently distinct inputs to the linearized audit model; adding their input
vectors is only the quasi-static, no-wheel-acceleration approximation where `F_t = F_m`.

## Audit Command

To inspect the current linearized upright model used by the simulator profiles:

```bash
./build/balancer_plant_audit --all
```

That prints, for each simulator profile:

- the plant parameters actually used
- the linearized `A` matrix and separate horizontal-force and motor-force input vectors
- the controllability rank
- the current candidate overdamped pole set
