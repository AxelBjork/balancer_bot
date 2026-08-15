# Control and Simulator Notes

This is the maintainer-facing notebook for the balancing stack. It captures the behavior we learned while migrating the project to the message-bus architecture and rebuilding the simulator/test harness around deterministic ticks.

## Control Structure

`RateControllerCore` is the real control law. `ControlService` is mostly an adapter around it.

The control pipeline is:

1. joystick forward input becomes a bounded, jerk-limited nominal acceleration
2. a 100 Hz observer corrects common completed motor steps for chassis pitch, filters velocity at
   10 Hz, then applies velocity damping and a smooth speed-envelope taper to form the pitch setpoint
3. the pitch error plus filtered pitch-rate damping become a pitch-rate setpoint
4. the 400 Hz PX4 `RateControl` produces the normalized balance correction
5. the balance correction becomes the common wheel command in steps per second
6. balance-priority turn allocation produces the left/right stepper commands

Important details:

- control is tick-driven by `PhysicsTick`
- the controller uses fused pitch plus filtered pitch-rate for control; raw gyro stays diagnostic-only
- axle velocity uses common completed-step position corrected as
  `Δu = (ΔqL + ΔqR)/2 + steps_per_rad * wrap(Δpitch)`; the motor-rate estimate remains
  a separate 50 ms diagnostic-only signal and differential turning cannot enter the observer
- the observer, velocity filter, jerk limiter, and COM-trim integrator use the elapsed 100 Hz
  interval so control-tick jitter does not change their continuous-time behavior
- missing, stale, or future IMU data, fallover, and actuator faults reset all controller state,
  command zero, and publish the corresponding controller-fault bitmask
- electrical direction inversion exists only inside the stepper boundary
- the active outer-loop parameters are `drive_max_acceleration_mps2`,
  `velocity_damping_per_s`, `velocity_I`, `angle_P`, and `angle_D`; jerk limiting is a fixed
  internal safety value, and maximum drive lean is derived from the configured acceleration

## Why Tick-Driven Control Matters

The older timing model mixed wall-clock threads and sensor-driven updates. The current model centralizes control execution on `PhysicsTick`.

That enabled:

- deterministic direct simulation
- explicit tick injection over UDP in `sil_app`
- easier control-loop testing
- faster-than-real-time scenario runs without wall-clock sleeps

IMU samples are still essential, but they are data inputs rather than timing authorities.

## Simulator Structure

The deterministic simulator consists of:

- `BalancerSimulator`
  the plant model and IMU synthesis
- `run_simulator_scenario`
  the runner that delegates to the same `SimulatorEngine` as the UDP service
- `balancer_simulator`
  the CLI/front-end for manual runs and artifact generation

The plant model exposes two named profiles and aggregate hardware-nominal parameters:

- `simplified`
  tuned to be a fast sanity/stability baseline
- `realistic`
  closer to the current hardware-oriented assumptions and used for the main representative ladder

The plant equations use total translating mass `T`, first mass moment about the axle `H`, and pitch
inertia about the axle `J`. Their authoritative nominal values and supported scenario scales live
in the simulator code. Use the physical-pendulum procedure in the Pi guide when updating `J`.

The actuator model advances requested motor position from scheduled pulses, tracks a separate
rotor/wheel state, limits motor force with a torque-speed envelope, estimates missed steps from
excess phase error, and transmits force to the cart-pole through a traction-limited tire coupling.
Its continuous motor field-speed term uses the post-slew command, while exact pulse-frame
quantization enters through scheduled emitted position. This keeps the two actuator effects
separate instead of applying frame quantization twice.
The controller sees only completed-pulse feedback. Its corrected-axle outer reference is

> $$
> \theta_{sp} = \operatorname{atan2}(a_{nominal} - k_v v_{axle},g) + \theta_{COM}.
> $$

Because the inverted-pendulum plant has an inverse response, a forward command may initially
reduce or reverse motor output to acquire positive lean before settling into forward travel.
The requested acceleration drives lean, while corrected axle velocity supplies immediate braking.
Removing `theta_COM` from both measured and requested pitch prevents physical trim from being
interpreted as drive lean.

The simulator applies disturbances as exogenous plant inputs rather than controller references:

- external horizontal force
- optional COM bias

That means a "push" scenario is a plant disturbance, not a synthetic joystick command.

## Scenario Ladder

The canonical acceptance catalog is defined alongside the simulator runner. It covers neutral
balance, disturbance recovery, bidirectional drive/stop behavior, and representative plant and
sensor margins. Direct callers, UDP runs, tuning, tests, and fuzz harnesses all use
`SimulatorEngine`; safety acceptance is evaluated by the shared C++ implementation.

## Artifact Outputs

Each preserved full-rate simulator run writes:

- `timeline.csv`
- `metadata.json`
- `summary.json`
- `overview_plot.svg`
- `actuator_plot.svg`

The most useful summary fields are:

- `fell`
- `max_abs_pitch_deg`
- `tail_rms_pitch_deg`
- `tail_mean_abs_pitch_deg`
- `tail_rail_fraction`
- `max_abs_position_m`
- `tail_mean_abs_velocity_mps`

The multiplots are designed for quick review:

- pitch is centered around zero
- wheel and plant velocities share a readable velocity panel
- axes and units are explicit
- long traces are downsampled for SVG readability while the CSV stays full-fidelity

`SimStartRunPayload.telemetry_stride` controls wire/artifact density: `0` emits only the terminal
summary, `1` emits all 400 Hz ticks, and `N` emits every Nth tick. Peak/tail metrics, continuous
saturation, controller/actuator faults, and the timeline hash are calculated on every engine tick
and are therefore independent of this stride.

The release evidence command creates a new run-specific directory and updates the stable summary:

```bash
python3 tools/run_transfer_validation.py --include-build-gates
```

## Learned Behavior

### The simulator and UDP service use the same engine

The UDP service is a reflected transport wrapper around the deterministic engine. It no longer
contains a second plant or wall-clock joystick path.

### Simulator and hardware PID configurations remain independent

Simulator and hardware PID files use the same validated schema but remain separate so hardware
validation can adjust its configuration without changing the simulation baseline.

### Real motor feedback matters on hardware

The hardware runtime now republishes requested targets, continuous post-slew wheel commands,
pulse-frame-applied wheel rates, independent slew-limit flags, and actual step counts. The
production motor slew is 200,000 SPS/s. Pulse generation retains an active plus synchronously
queued frame at 2.5 ms per frame, so requested, post-slew, and quantized applied stages remain
distinguishable in telemetry. The separate 50 ms average-speed estimate is diagnostic only; the
controller uses its own 10 ms completed-step observer.

### Drift is now treated as the main realism target

The rewritten simulator/controller path no longer treats "did not fall" as the main representative signal. The main diagnostic is slow drift or slow runaway under small bias and push cases, because that is closer to the observed hardware failure mode than short pulse rejection alone.

## What to Watch During Future Tuning

- avoid claiming simulator success from SIL smoke tests alone
- score nominal plus one-at-a-time margins and reject any candidate with a hard safety failure
- preserve artifact generation whenever changing controller or plant behavior

For the current confidence level and remaining caveats, read [Current Status](../status.md).
