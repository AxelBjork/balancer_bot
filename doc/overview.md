# Project Overview

`balancer_bot` has four main executable entrypoints and two primary runtime configurations: hardware and simulation.

## Main Binaries

### `balancer_pi`

The Raspberry Pi production executable. It:

- loads `pid.conf`
- creates the real `MotorRunner`
- enables the hardware IMU reader
- starts `TimeService` to publish `PhysicsTick`
- accepts neutral joystick input when no Xbox controller is attached
- can also expose telemetry over UDP through `UdpBridge`

This binary is built with `BUILD_TESTS=OFF`.

### `sil_app`

The UDP-driven software-in-the-loop runtime. It:

- loads `pid_sim.conf`
- disables the hardware IMU reader
- does not create a real motor backend
- receives `PhysicsTick`, `ImuRawData`, and `JoystickCommand` over UDP
- sends `MotorTargets` and `SystemTelemetry` back to Python

This binary is built with `BUILD_TESTS=ON`.

### `balancer_simulator`

The deterministic direct simulator. It runs the control core and plant model in one process, writes artifacts under `build/sim`, and is the main software stability gate for the balancing behavior.

### `balancer_tests`

The C++ unit and integration test binary. It covers the motor runner, control core, service interactions, time service, simulator invariants, and filter sanity checks.

## Runtime Modes

### Hardware Runtime

`balancer_pi` is the real robot path:

- `TimeService` owns the 400 Hz control tick
- `ImuService` publishes IMU samples from the ISM330 reader
- `ControlService` steps `RateControllerCore` on each tick
- `MotorService` forwards commands to `MotorRunner` and republishes real motor feedback

### SIL Runtime

`sil_app` is the service-level integration path:

- Python injects ticks and IMU samples over UDP
- the controller still runs through the same service/message-bus path
- no real IMU reader or motor backend is attached
- motor feedback falls back to the last commanded wheel speeds when no hardware feedback exists

### Direct Simulator

`balancer_simulator` can drive the shared deterministic engine directly or expose it through its
UDP wrapper. The engine includes the production IMU filter, control/motor services, pulse
scheduler, and physical plant model. The complete twenty-scenario gate runs in-process; focused
Python tests cover the real UDP boundary and artifact generation.

## PID Config Files

- `pid.conf` is the default hardware profile used by `balancer_pi`.
- `pid_sim.conf` is the default simulator and SIL profile used by `sil_app` and `balancer_simulator`.
- `BALANCER_PID_CONF` overrides the default path in simulator-oriented code paths and tests.

The two files are intentionally separate. Simulator tuning is allowed to diverge from hardware tuning as long as the difference is documented and understood.

## Build Modes

### `BUILD_TESTS=ON`

Builds:

- `balancer_tests`
- `balancer_simulator`
- `sil_app`
- generated bindings and IPC docs on host builds

### `BUILD_TESTS=OFF`

Builds:

- `balancer_pi`
- `imu_demo`

Use [Running on Pi](Running_on_Pi.md) for the deployment path and [Testing Strategy](testing/strategy.md) for host-side verification.
