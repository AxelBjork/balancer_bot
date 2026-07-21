# Two-Wheel Balancer — Compact Laplace Cheat Sheet

## Aggregate Simulator Parameters

The nonlinear simulator uses aggregate rigid-body parameters rather than an arbitrary split between
cart and body mass: total translating mass `T`, first mass moment `H` about the axle, and pitch
inertia `J` about the axle. Its mass matrix is

> $$
> \begin{bmatrix}T&H\cos\theta\\H\cos\theta&J\end{bmatrix}.
> $$

The authoritative nominal mass, geometry, and inertia values are defined by
[`HardwareNominal`](../../tests/simulator/balancer_simulator.h). Pitch inertia should be updated
from the physical-pendulum procedure documented in the Pi runtime guide; these values are
intentionally not duplicated here.

## Variables

- `x`: forward position of the wheel axle
- `x_dot`: forward velocity
- `x_ddot`: forward acceleration
- `theta`: body pitch angle from upright
- `theta_dot`: pitch rate
- `theta_ddot`: pitch angular acceleration
- `n`: wheel speed in RPM
- `n_dot`: wheel RPM rate in RPM/s
- `r`: wheel radius
- `m`: body mass
- `l`: distance from wheel axle to body center of mass
- `I`: body pitch inertia about the body center of mass
- `g`: gravitational acceleration
- `X(s)`: Laplace transform of `x(t)`
- `Theta(s)`: Laplace transform of `theta(t)`
- `N(s)`: Laplace transform of `n(t)`

Assumptions:

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

Current controller structure in code:

Outer velocity PI at 50 Hz:

> $$
> e_v = v_{\mathrm{ref}} - v_{\mathrm{completed\ pulses}}
> $$

> $$
> \theta_{\mathrm{ref}} = \operatorname{clamp}(K_{pv}e_v + K_{iv}\int e_v dt)
> $$

Inner attitude shaping:

> $$
> \dot\theta_{\mathrm{ref}} = k_{\mathrm{pitch}}(\theta_{\mathrm{ref}} - \theta) - k_{\mathrm{pitch\_rate}}\dot\theta_f
> $$

PX4 rate loop:

- it tracks `theta_dot_ref` and produces the normalized motor command

Motor scaling and target-speed feed-forward:

- the normalized PX4 output is converted to wheel command with fixed

> $$
> u_{\mathrm{sps}} = v_{\mathrm{ref,sps}} - u_{\mathrm{norm}}\,k_{\mathrm{output}}
> $$

- the result is clamped by `balance_max_sps`, then turn allocation consumes only the remaining
  balance authority

Notes:

- `theta_dot_f` is the filtered pitch-rate signal from the complementary filter, not the raw gyro debug signal
- the simulator now applies disturbances as exogenous plant inputs:
  external horizontal force and optional COM bias
- the simulator plant is intentionally richer than the cheat-sheet model: scheduled pulses advance
  commanded motor phase; completed pulses supply controller feedback; a separate rotor/wheel state,
  torque-speed limit, actuator lag, missed-step estimate, and traction-limited tire coupling produce
  force on the nonlinear cart-pole

### Simulator wheel and motor coordinates

The nonlinear simulator keeps the wheel coordinate `u = r psi` as absolute circumferential wheel
motion. Tire deformation and tire speed therefore remain `u - x` and `u_dot - x_dot`. Motor steps,
however, measure rotor motion relative to the chassis. With the configured initial pitch as the
zero-step reference, the motor coordinates are

> $$
> q = u-r(\theta-\theta_0), \qquad \dot q = \dot u-r\dot\theta.
> $$

The phase-error controller and torque-speed limit use `q` and `q_dot`. A motor force `F_m` applies
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
