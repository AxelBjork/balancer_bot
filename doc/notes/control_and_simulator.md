# Control and Simulator Notes

This is the maintainer-facing notebook for the balancing stack. It captures the behavior we learned while migrating the project to the message-bus architecture and rebuilding the simulator/test harness around deterministic ticks. For canonical runtime boundaries and validation terminology, see [Runtime architecture](../arch/runtime.md), [Control and plant model](../arch/control_plant.md), and [Testing strategy](../testing/strategy.md).

## Control Structure

`RateControllerCore` is the real control law. `ControlService` is mostly an adapter around it.

The control pipeline is:

1. joystick forward input becomes a bounded, jerk-limited nominal acceleration
2. a 100 Hz observer corrects common completed motor steps for chassis pitch, filters velocity at
   10 Hz, then applies a separate 3 Hz velocity-control filter, velocity damping, and a smooth
   speed-envelope taper to form the translational pitch request
3. the velocity pitch request is bounded by `velocity_pitch_limit_deg`; the bounded request,
   joystick lean target, and COM trim are summed into one pitch target
4. explicit pitch, pitch-rate, and filtered pitch-acceleration feedback produces the balance command
5. the balance command becomes the common wheel command in steps per second
6. balance-priority turn allocation produces the left/right stepper commands

Important details:

- control is tick-driven by `PhysicsTick`
- the controller uses fused pitch plus filtered pitch-rate for control; raw gyro stays diagnostic-only
- axle velocity uses common completed-step position corrected as
  `Δu = (Δq_steps,L + Δq_steps,R)/2 + steps_per_rad * wrap(Δpitch)`; the motor-rate estimate remains
  a separate 50 ms diagnostic-only signal and differential turning cannot enter the observer
- the observer, velocity filter, jerk limiter, and COM-trim integrator use the elapsed 100 Hz
  interval so control-tick jitter does not change their continuous-time behavior
- missing, stale, or future IMU data, fallover, and actuator faults reset all controller state,
  command zero, and publish the corresponding controller-fault bitmask
- electrical direction inversion exists only inside the stepper boundary
- the active controller parameters are `pitch_gain`, `pitch_rate_gain`, `pitch_accel_gain`,
  `velocity_damping_per_s`, `velocity_pitch_limit_deg`, `velocity_control_cutoff_hz`, and
  `velocity_I`; the three attitude
  gains are independently expressed in SPS/rad, SPS/(rad/s), and SPS/(rad/s²), while `velocity_I`
  is the bounded stationary COM-trim integrator; jerk limiting is a fixed internal safety value,
  and maximum drive lean is derived from the configured acceleration

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

The plant model exposes a nominal profile, an explicit actuator-stress profile, and direct-force
reference profiles used for controller architecture validation:

- `simplified`
  tuned to be a fast sanity/stability baseline
- `realistic`
  the nominal hardware-oriented phase/tire plant retained for diagnostic stress reporting
- `ideal_force`
  the primary controller-design reference: direct applied wheel force with the current physical
  body model, bypassing the unidentified phase-position actuator layer
- `simple_force`
  a secondary direct-force reference with a declared aggregate force lag for historical comparison
- `actuator_stress`
  the same low-damping physical baseline with a 20 ms aggregate actuator response for robustness
  diagnostics; it is not the measured electrical motor time constant and is not a second PID target

The nominal physical baseline is `J = 0.0045 kg m^2` and `cart_damping = 1 N s/m`. The inertia is
the current passive-pendulum estimate in `HardwareNominal`; cart damping remains the original
baseline because no hardware identification justifies replacing it. The nominal actuator response
is provisionally `motor_tau_s = 0.002 s`, anchored to the approximately 1.95 ms electrical time
constant. Pulse-frame scheduling, slew limiting, phase error, and tire coupling are modeled
separately. The stress profile uses `0.020 s` only as an end-to-end aggregate robustness case;
its high phase/motor saturation duty cycle is diagnostic evidence of model uncertainty, not a
physical motor constant. The phase/tire profile is not the tuning authority for the shared
controller default.

The pure `actuator_stress` profile with this nominal damping is intentionally diagnostic rather
than a hard acceptance target: the 20 ms aggregate lag can still drive the current common
controller beyond the plausible actuator envelope. The maintained stress scenarios add modest
mass, inertia, traction, and damping variation to exercise robustness without redefining the
nominal plant. If the pure profile remains unstable, that is recorded as an actuator-model
contradiction to resolve, not a reason to create a second PID default.

The plant equations use total translating mass `T`, first mass moment about the axle `H`, and pitch
inertia about the axle `J`. Their authoritative nominal values and supported scenario scales live
in the simulator code. Use the physical-pendulum procedure in the Pi guide when updating `J`.

The actuator model advances requested motor position from scheduled pulses, tracks a separate
rotor/wheel state, limits motor force with a torque-speed envelope, estimates missed steps from
excess phase error, and transmits force to the cart-pole through a traction-limited tire coupling.
Its continuous motor field-speed term uses the post-slew command, while exact pulse-frame
quantization enters through scheduled emitted position. This keeps the two actuator effects
separate instead of applying frame quantization twice.
Feedback semantics are mode-specific: hardware uses actual completed steps, the direct simulator
uses quantized simulated completed steps, and SIL uses a commanded-speed proxy because it has no
plant or motor runner. The corrected-axle outer-loop equations are:

> $$
> v_c = \operatorname{LPF}_{3\,Hz}\left(\operatorname{LPF}_{10\,Hz}\left(\frac{\Delta q + N\Delta\theta}{\Delta t}\right)\right)
> $$
>
> $$
> a_v = -D_v\,v_c\,s_{step},\qquad
> \theta_v = \operatorname{clamp}\left(\operatorname{atan2}(a_v,g),\ \pm\theta_{v,max}\right)
> $$
>
> $$
> \theta_{target} = \operatorname{clamp}\left(
> \operatorname{atan2}(a_{nominal},g) + \theta_v + \theta_{COM},\ \pm\theta_{max}\right).
> $$

Here `D_v` is `velocity_damping_per_s`, `s_step` is `Config::meters_per_step`, and
`theta_v,max` is the dedicated `velocity_pitch_limit_deg` authority boundary. A zero dedicated
limit is retained only as an explicit diagnostic escape hatch; the checked-in default is 4°.
The final `theta_max` remains the independently derived total pitch safety limit (about 7.26° at
the current 1.25 m/s² drive-acceleration limit).

COM trim is updated in degrees as

> $$\Delta\theta_{COM} = -I_v\,v_{axle}\,\Delta t.$$

Learning requires neutral command and zero nominal acceleration, and is blocked by velocity
authority limiting, total pitch-target or balance saturation, prior-cycle controller saturation,
or the motion state. An untrusted trim can acquire while the body is gently settling, but a
substantial velocity/rate/pitch excursion pauses acquisition as well. Once a quiet dwell has
established `trim_trusted`, motion freezes the estimate and a later quiet dwell re-enables slow
maintenance learning. Quiet rate is a 2 Hz low-frequency RMS metric for this state machine only;
it does not alter fused pitch, rate feedback, or the locked 29 Hz attitude notch.

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

### One checked-in baseline, separate tuning outputs

The checked-in `pid.conf` is the single preliminary state-feedback controller profile loaded by
both the hardware runtime and the simulator. Simulator-oriented tuning can select another input or
write candidate profiles under `build/sim`, which lets an experiment preserve its provenance
without silently changing the shared default. A simulator candidate is never an authorization to
apply that profile to the physical robot; hardware validation remains a separate step.

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
- select gains on the direct-force/reference profiles, then report the phase/tire profile as a
  diagnostic stress result rather than silently tuning around its unidentified parameters
- preserve artifact generation whenever changing controller or plant behavior

For the current confidence level and remaining caveats, read [Current Status](../status.md).
