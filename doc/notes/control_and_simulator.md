# Control and Simulator Notes

This is the maintainer-facing notebook for the balancing stack. It captures the behavior we learned while migrating the project to the message-bus architecture and rebuilding the simulator/test harness around deterministic ticks.

## Control Structure

`RateControllerCore` is the real control law. `ControlService` is mostly an adapter around it.

The control pipeline is:

1. joystick forward input becomes a requested longitudinal acceleration
2. a jerk-limited acceleration controller, with velocity damping, turns that request into a pitch setpoint
3. slow trims bias the pitch setpoint when persistent error or drift is present
4. the pitch error plus filtered pitch-rate damping become a pitch-rate setpoint
5. PX4 `RateControl` produces the normalized pitch-axis effort
6. the effort is scaled into wheel speed commands in steps per second

Important details:

- control is tick-driven by `PhysicsTick`
- the controller uses fused pitch plus filtered pitch-rate for control; raw gyro stays diagnostic-only
- trim bias is an internal `RateControllerCore` behavior, not a PID file knob
- when hardware feedback exists, velocity and position feedback come from `MotorFeedback`
- in SIL without a motor backend, the controller falls back to the last commanded wheel speeds
- the outer-loop parameters are `max_longitudinal_accel_mps2`, `max_jerk_mps3`,
  `velocity_damping_per_s`, `pitch_P`, and `pitch_D`

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
  the runner that couples the plant to `RateControllerCore`
- `balancer_simulator`
  the CLI/front-end for manual runs and artifact generation

The plant model exposes two named profiles:

- `simplified`
  tuned to be a fast sanity/stability baseline
- `realistic`
  closer to the current hardware-oriented assumptions and used for the main representative ladder

`I_com` is kept fixed at the hardware-oriented value in the simulator. Stability work has focused on controller structure, profile tuning, and explicit scenario coverage rather than hiding problems by changing that inertia constant.

The simulator now applies disturbances as exogenous plant inputs rather than controller references:

- external horizontal force
- optional COM bias

That means a "push" scenario is a plant disturbance, not a synthetic joystick command.

## Scenario Ladder

The current Python scenario ladder includes:

- simplified sanity:
  - `neutral_hold`
- realistic representative cases:
  - neutral hold
  - slow push recover
  - disturbance train
- realistic drift diagnostics:
  - small pitch bias
  - COM offset
  - slow push runaway
  - long-horizon hold bias
- realistic frontier diagnostic:
  - large-angle recovery that remains `xfail`

The direct simulator, not `sil_app`, is the primary stability gate.

## Artifact Outputs

Each preserved simulator run writes:

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

## Learned Behavior

### `sil_app` is a smoke path, not the stability gate

`sil_app` proves the service bus, UDP bridge, generated bindings, and controller service path. It does not replace the direct simulator for stability work.

### The simulator and hardware intentionally use different PID profiles

- `pid.conf` is the hardware profile
- `pid_sim.conf` is the simulator/SIL profile

The simulator profile is allowed to diverge while the model and control structure continue to evolve.

### Real motor feedback matters on hardware

The hardware runtime now republishes applied wheel rates and actual step counts from `MotorRunner` as `MotorFeedback`. The controller should not assume the last command was achieved, especially because motor ramping and step application are not instantaneous.

### Drift is now treated as the main realism target

The rewritten simulator/controller path no longer treats "did not fall" as the main representative signal. The main diagnostic is slow drift or slow runaway under small bias and push cases, because that is closer to the observed hardware failure mode than short pulse rejection alone.

## What to Watch During Future Tuning

- avoid claiming simulator success from SIL smoke tests alone
- keep `pid.conf` and `pid_sim.conf` intentionally separate until a convergence pass is justified
- prefer representative scenario promotion over adding many mirrored near-duplicate cases
- preserve artifact generation whenever changing controller or plant behavior

For the current confidence level and remaining caveats, read [Current Status](../status.md).
