# Control and Simulator Notes

This is the maintainer-facing notebook for the balancing stack. It captures the behavior we learned while migrating the project to the message-bus architecture and rebuilding the simulator/test harness around deterministic ticks.

## Control Structure

`RateControllerCore` is the real control law. `ControlService` is mostly an adapter around it.

The control pipeline is:

1. joystick forward input becomes a bounded target wheel velocity
2. a 50 Hz velocity PI turns completed-pulse speed error into a bounded pitch setpoint
3. the pitch error plus filtered pitch-rate damping become a pitch-rate setpoint
4. the 400 Hz PX4 `RateControl` produces the normalized balance correction
5. the target wheel speed is fed forward and the balance correction is added in steps per second
6. balance-priority turn allocation produces the left/right stepper commands

Important details:

- control is tick-driven by `PhysicsTick`
- the controller uses fused pitch plus filtered pitch-rate for control; raw gyro stays diagnostic-only
- velocity feedback always comes from completed pulses in `MotorFeedback`; plant truth is telemetry only
- missing, stale, or future IMU data, fallover, and actuator faults reset all controller state,
  command zero, and publish the corresponding controller-fault bitmask
- electrical direction inversion exists only inside the stepper boundary
- the active outer-loop gains are `velocity_P`, `velocity_I`, `angle_P`, and `angle_D`

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

The plant model exposes two named profiles and one hardware-nominal geometry/inertia definition:

- `simplified`
  tuned to be a fast sanity/stability baseline
- `realistic`
  closer to the current hardware-oriented assumptions and used for the main representative ladder

`I_com` is kept fixed at the hardware-oriented value in the simulator. Stability work has focused on controller structure, profile tuning, and explicit scenario coverage rather than hiding problems by changing that inertia constant.

The actuator model advances requested motor position from scheduled pulses, tracks a separate
rotor/wheel state, limits motor force with a torque-speed envelope, estimates missed steps from
excess phase error, and transmits force to the cart-pole through a traction-limited tire coupling.
The controller sees only completed-pulse feedback.

The simulator applies disturbances as exogenous plant inputs rather than controller references:

- external horizontal force
- optional COM bias

That means a "push" scenario is a plant disturbance, not a synthetic joystick command.

## Scenario Ladder

The canonical acceptance catalog covers nominal neutral hold, a 3 N/100 ms push, the same push with
IMU noise and lag, an 800 SPS drive/stop command, and all 16 one-at-a-time plant margins. There are
no remaining simulator `xfail` cases. Direct callers, UDP runs, the tuner, tests, and fuzz harnesses
all use `SimulatorEngine`; transfer tests, tuning, and UDP validation also share the C++ hard-safety
evaluator.

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

### The simulator and hardware keep separate PID files with the same current winner

`pid.conf` and `pid_sim.conf` both use strict `config_version = 2` and currently contain the same
conservative simulation winner. They remain separate so later restrained hardware validation can
adjust the hardware profile without changing the simulation baseline.

### Real motor feedback matters on hardware

The hardware runtime now republishes applied wheel rates and actual step counts from `MotorRunner` as `MotorFeedback`. The controller should not assume the last command was achieved, especially because motor ramping and step application are not instantaneous.

### Drift is now treated as the main realism target

The rewritten simulator/controller path no longer treats "did not fall" as the main representative signal. The main diagnostic is slow drift or slow runaway under small bias and push cases, because that is closer to the observed hardware failure mode than short pulse rejection alone.

## What to Watch During Future Tuning

- avoid claiming simulator success from SIL smoke tests alone
- keep `pid.conf` and `pid_sim.conf` separate even while their current values match
- score nominal plus one-at-a-time margins and reject any candidate with a hard safety failure
- preserve artifact generation whenever changing controller or plant behavior

For the current confidence level and remaining caveats, read [Current Status](../status.md).
