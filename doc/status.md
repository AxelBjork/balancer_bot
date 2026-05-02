# Current Status

This page is the short, candid status snapshot for the project. The rest of the handbook stays mostly happy-path; this page captures the important caveats.

## What Is in Good Shape

- the host build, C++ tests, Python tests, and Pi cross-build are expected to be green in the current tree
- the runtime is tick-driven and the service split is wired through the message bus
- generated bindings and generated IPC docs are part of the normal host build flow
- hardware control uses real `MotorFeedback` from `MotorRunner`, not just last-command estimates
- simulator runs produce structured artifacts and plots under `build/sim`

## Important Caveats

- the physical robot has not been fully revalidated on hardware after the message-bus refactor
- `sil_app` is useful for service-level integration, but it is not the main stability benchmark
- `pid_sim.conf` is intentionally not the same as `pid.conf`
- the realistic simulator profile still has a harder frontier case tracked as `xfail`

## Practical Confidence Model

### Strong Confidence

- build wiring
- reflection-generated docs and bindings
- message-bus service integration
- simulator artifact generation
- representative simulator stability ladder

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
