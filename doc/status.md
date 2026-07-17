# Current Status

This page is the short, candid status snapshot for the project. The rest of the handbook stays mostly happy-path; this page captures the important caveats.

## What Is in Good Shape

- the verified host gate passes 76 C++ and 15 Python tests with no skips or xfails
- the full-rate transfer report passes all twenty scenarios; host fuzz smoke passes all fourteen
  seeds; the Pi cross-build succeeds
- the runtime is tick-driven and the service split is wired through the message bus
- generated bindings and generated IPC docs are part of the normal host build flow
- hardware control uses real `MotorFeedback` from `MotorRunner`, not just last-command estimates
- simulator runs produce structured artifacts and plots under `build/sim`
- controller safety states and plant parameters are observable in telemetry, and terminal run
  metrics remain exact under telemetry downsampling

## Important Caveats

- the physical robot has not been fully revalidated on hardware after the message-bus refactor
- simulation validation does not make the current gains hardware-proven
- `pid_sim.conf` and `pid.conf` currently match but remain separate for later hardware adjustment
- motor authority, tire coupling, damping, and missed-step limits remain uncertain until measured
- the transfer report identifies a dirty working tree until these changes are committed; the
  run-specific manifest and PID digest provide the remaining configuration provenance

## Practical Confidence Model

### Strong Confidence

- build wiring
- reflection-generated docs and bindings
- message-bus service integration
- simulator artifact generation
- nominal and one-at-a-time-margin simulator acceptance ladder
- exact direct-versus-UDP all-tick equivalence and stride-invariant terminal summaries
- warning-clean host compilation/execution of all registered fuzz seeds

### Moderate Confidence

- Pi deployment and runtime bring-up, assuming the expected libraries and IMU setup are present
- hardware runtime behavior with neutral/no-controller startup

### Needs Real Hardware Validation

- final balancing behavior on the physical robot with the current refactored stack
- how closely the simulator tuning now matches the real robot in difficult cases

## Recommended Next Real-Robot Session

1. Cross-build `balancer_pi`.
2. Copy both `balancer_pi` and `pid.conf` to the Pi.
3. Verify `pigpiod`, IMU binding, and IIO permissions.
4. Bring the robot up with the wheels off the ground or physically restrained first.
5. Use UDP telemetry and the existing controller print path to confirm the runtime is alive before floor testing.
