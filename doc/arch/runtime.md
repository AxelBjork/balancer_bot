# Runtime Architecture

This page describes the runtime as it exists today, not the earlier migration plan.

## Core Model

The runtime is built around a small synchronous `MessageBus` plus a handful of explicit services:

- `TimeService`
- `ImuService`
- `ControlService`
- `MotorService`
- `UdpBridge`

The bus itself does not do discovery, scheduling, or queueing. It serializes dispatch with a
recursive mutex, then forwards a typed payload pointer into a manually written dispatcher owned by
the application entrypoint. Nested synchronous publications remain atomic with respect to other
producer threads.

## Service Responsibilities

### `TimeService`

Publishes `PhysicsTick`. In production it owns the wall-clock worker thread that emits the 400 Hz
control heartbeat using absolute steady-clock timestamps and measured elapsed `dt`. In tests and
the deterministic simulator it advances a zero-based timeline explicitly.

### `ImuService`

Publishes `ImuRawData` from the hardware reader, consumes that raw sample, and republishes fused `ImuData` internally. In SIL mode the reader is disabled and raw samples are injected externally over UDP as `ImuRawData`.

### `ControlService`

Caches the latest `PhysicsTick`, `ImuData`, `JoystickCommand`, and `MotorFeedback` inputs, then steps `RateControllerCore` on each tick. It publishes:

- `MotorTargets`
- `SystemTelemetry`

This is the layer that binds the pure control core to the message bus.

### `MotorService`

Consumes `MotorTargets`, forwards them to `MotorRunner` when hardware exists, and republishes `MotorFeedback` using the applied motor state and accumulated actual steps.

### `UdpBridge`

Bridges the internal bus to UDP:

- UDP ingress: `PhysicsTick`, `JoystickCommand`, `ImuRawData`
- UDP egress: `MotorTargets`, `SystemTelemetry`

`MotorFeedback` is intentionally internal-only and is not part of the Python UDP contract.

## Production Path

`balancer_pi` creates the real runtime in `src/services/main/control_app.h`:

1. `main.cpp` loads `pid.conf`.
2. `ControlApp` constructs the `MotorRunner`, services, and `MessageBus`.
3. `TimeService` publishes ticks at the configured control rate.
4. `ImuService` publishes live IMU samples.
5. `ControlService` steps `RateControllerCore` on each `PhysicsTick`.
6. `MotorService` applies wheel targets and publishes `MotorFeedback`.
7. `UdpBridge` can mirror telemetry and motor targets to a Python client.

The important hardware detail is that the controller now closes the loop on real motor feedback from `MotorRunner`, not just the last commanded target.

## SIL Paths

`sil_app` remains a narrower message/service smoke runtime:

- no `TimeService`
- no hardware IMU reader
- no real `MotorRunner`

Instead, Python drives the system by sending:

- `PhysicsTick`
- `ImuRawData`
- `JoystickCommand`

The controller/plant acceptance path is `balancer_simulator`. Its direct runner, UDP wrapper,
tuner, tests, and fuzz targets all instantiate the same deterministic `SimulatorEngine`, including
the production IMU filter, control service, motor service, and pulse scheduler. The UDP wrapper
transports scenario configuration and telemetry; it does not implement a second simulation.

On every controller tick, missing/stale/future IMU input, fallover, or an actuator fault produces a
zero motor target, resets controller state, and remains observable through telemetry fault flags.

## Tick Ownership

The controller is tick-driven now. `PhysicsTick` is the timing authority for control execution in both production and SIL. IMU samples are data inputs, not schedulers.

That change matters because it:

- makes SIL deterministic when time is injected explicitly
- keeps controller step timing independent from IMU delivery cadence
- allows the direct simulator to run faster than real time without wall-clock sleeps

## Related Docs

- [MessageBus and UDP Bridge](message_hub.md)
- [IPC Protocol Reference](../ipc/protocol.md)
- [Project Overview](../overview.md)
