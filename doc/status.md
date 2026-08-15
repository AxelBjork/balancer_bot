# Current Status

This page is the short, candid status snapshot for the project. The rest of the handbook stays mostly happy-path; this page captures the important caveats. For validation details, see [Testing Strategy](testing/strategy.md); for physical evidence, see the [data archive](../data/README.md).

_Snapshot date: 2026-08-07. Run-specific claims still require the captured manifest and PID
digest; this page is not a release record._

## What Is in Good Shape

- the host gate is defined by `pytest --build`; do not rely on a hard-coded test count because the
  registered suites change
- transfer-matrix validation uses the seven-scenario acceptance set, while fuzz smoke runs the currently
  registered seed corpus; run the commands before reporting a new pass
- the runtime is tick-driven and the service split is wired through the message bus
- generated bindings and generated IPC docs are part of the normal host build flow; cross-builds
  consume the host-generated interface outputs rather than regenerating them
- UDP port `9000` is the production external runtime boundary, with the telemetry dashboard as the
  primary peer and the Python SIL client as an integration driver for `sil_app`
- hardware control uses real `MotorFeedback` from `MotorRunner`, not just last-command estimates
- simulator runs produce structured artifacts and plots under `build/sim`
- controller safety states and simulator plant parameters are observable in telemetry where the
  relevant fields are populated, and terminal run metrics remain exact under telemetry downsampling

## Important Caveats

- the physical robot has not been fully revalidated on hardware after the message-bus refactor
- simulation validation does not make the current gains hardware-proven
- `pid.conf` is shared, but exact physical characteristics are only modeled after hardware adjustment
- motor authority, tire coupling, damping, and missed-step limits remain uncertain until measured
- transfer reports and telemetry captures must retain their run-specific manifest and PID digest for
  configuration provenance
- retained hardware sessions are historical evidence by default; a checksum-bound extract is not a
  current verification result unless its repository, PID, protocol, and hardware context are
  explicitly re-established

## Practical Confidence Model

### Strong Confidence

- build wiring
- reflection-generated docs and bindings
- message-bus service integration
- simulator artifact generation
- the four nominal and three actuator-stress cases in the transfer matrix
- focused direct-versus-UDP all-tick equivalence for transfer catalog index `1`, plus stride-invariant
  terminal summaries
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
