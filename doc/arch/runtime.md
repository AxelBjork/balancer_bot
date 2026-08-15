# Runtime architecture

Read this page after the [project overview](../overview.md). The overview explains why there are
several runtime modes; this page explains how the shared services fit together.

## Shared shape

The runtime has one control clock, one synchronous internal bus, and a small set of services with
explicit responsibilities. The [project overview](../overview.md) contains the conceptual flow
diagram; this page focuses on ownership and the differences between runtime compositions.

The control loop is tick-driven. IMU samples, joystick commands, motor feedback, and network
packets are inputs to the tick; they do not independently schedule controller execution.

## Service ownership

| Service | Owns or publishes | Consumes | Important boundary |
| --- | --- | --- | --- |
| `TimeService` | The `PhysicsTick` heartbeat and elapsed `dt` | Nothing | Runs from wall clock in hardware; callers advance it explicitly in tests/simulation |
| `ImuService` | Raw-to-filtered attitude path; publishes `ImuData` | `ImuRawData` | Hardware reader can be disabled for SIL |
| `ControlService` | Balance control, `MotorTargets`, `SystemTelemetry`, and PID status | Tick, attitude, joystick, motor feedback, and PID overrides | Binds the pure controller and live PID adapter to the message bus |
| `MotorService` | Hardware-facing target forwarding and `MotorFeedback` | Tick and `MotorTargets` | Accepts a null runner for SIL and in-process tests |
| `InputService` | Xbox polling, external-command arbitration, and resolved `JoystickCommand` | `ExternalJoystickCommand` | Xbox has strict priority; external commands use a short watchdog and neutral fallback |
| `UdpBridge` | External ingress/egress for selected messages | UDP datagrams and subscribed bus messages | Port `9000`; one active peer; validates message IDs and sizes |

The exact publish/subscribe lists are generated into the [IPC protocol reference](../ipc/protocol.md).
The conceptual roles of the key messages are summarized in the [project overview](../overview.md#messages-and-boundaries).

## Hardware runtime

`balancer_pi` assembles the real runtime in `src/services/main/control_app.h`:

1. It loads `pid.conf` and initializes the stepper hardware.
2. It creates a real `MotorRunner` when `controller_enabled` is true.
3. `TimeService` publishes the 400 Hz control heartbeat.
4. `ImuService` reads the ISM330DHCX through the Linux IIO path and publishes the filtered estimate.
5. `InputService` polls the optional Xbox controller and resolves it against external dashboard
   commands; no attached controller is a safe startup state.
6. `ControlService` validates and applies complete session PID snapshots received over UDP, then
   publishes the asynchronous status response.
7. `ControlService` steps the controller on each `PhysicsTick`.
8. `MotorService` applies targets and reports completed-step feedback.
9. `UdpBridge` exposes the authorized port-9000 runtime messages to the active peer, normally the
   telemetry dashboard; external joystick ingress is delivered to `InputService` by the normal bus.

The dashboard is primarily an observation and logging peer. It receives `SystemTelemetry` and
writes CSV logs; deployment, starting, and aborting the Pi process remain SSH operations. The
bridge is not a network-based motor-arming interlock: local controller safety logic still applies
if the dashboard disconnects.

For passive IMU or physical-pendulum work, `controller_enabled = 0` keeps the estimator and
telemetry path alive while both steppers remain de-energized and no `MotorRunner` is created. The
operational procedure belongs in [Running on Raspberry Pi](../Running_on_Pi.md).

## SIL runtime

`sil_app` contains the service-level path without hardware scheduling or a plant:

- `ImuService` runs with its hardware reader disabled.
- `MotorService` has no real `MotorRunner`.
- `InputService` resolves `ExternalJoystickCommand` with no attached Xbox controller.
- `ControlService` and `UdpBridge` are the same service-level components used by the production path.
- Python registers as the active peer and injects `PhysicsTick`, `ImuRawData`, and
  `ExternalJoystickCommand`.
- The application does not start `TimeService`; the external client supplies the ticks.

This makes SIL useful for proving that the production UDP boundary, message authorization, service
dispatch, controller stepping, telemetry, and serialization remain coherent. It does not prove
closed-loop stability because no physical plant is present.

The single-peer rule matters here: stop the dashboard before starting a SIL client against the same
bridge. The most recent sender owns the return path for outbound messages.

## Direct simulator

`balancer_simulator` uses the deterministic `SimulatorEngine`. The engine combines:

- the production IMU sampling and estimator path;
- the production control and motor services;
- the pulse scheduler and quantized completed-step feedback; and
- a nonlinear plant model with configurable physical, actuator, sensor, and disturbance parameters.

The simulator can run through its scenario-control wrapper on UDP port `9001`, or run a direct
in-process summary. The wrapper is a transport around the deterministic engine; it is not another
production `UdpBridge` and it is not the SIL application.

The transfer catalog contains ten scenarios. Each run records a complete deterministic timeline and
is judged using explicit checks such as finite values, no fall, peak pitch, tail RMS, continuous
saturation, actuator faults, and controller faults. The [testing strategy](../testing/strategy.md)
explains which runs are acceptance evidence and where their artifacts live.

## The message bus

The internal `MessageBus` is intentionally small:

- dispatch is synchronous in the publishing thread;
- it does not discover services, schedule work, or create a background queue;
- nested publications complete in the same dispatch chain; and
- each service declares the message IDs it may publish and consume.

This makes service ownership and ordering explicit. The bus is a control-path boundary, not a
general-purpose broker. For the implementation-level synchronization and authorization rules, see
[MessageBus and UDP Bridge](message_hub.md).

## The UDP boundary

The production `UdpBridge` listens on port `9000`. Any received datagram identifies its sender as
the current peer; a zero-ID datagram is the explicit registration form used by the Python client.
The most recent sender replaces the previous peer, even before the bridge checks whether that
datagram is valid or authorized. Invalid packets are then discarded and never published to the bus.

Ingress is accepted only when the message ID belongs to the bridge's allowed publish list and the
payload size matches the reflected message definition. Egress is sent only for subscribed message
IDs and only to the active peer. The external datagram is a little-endian message ID followed by
the reflected payload bytes; exact sizes and fields belong in the [generated protocol](../ipc/protocol.md).

For the practical endpoints, the production bridge on port `9000` accepts `PhysicsTick`,
`ExternalJoystickCommand`, `ImuRawData`, and `PidConfigOverride`, and returns `MotorTargets`,
`SystemTelemetry`, and `PidConfigStatus`. The separate simulator wrapper on port `9001` accepts
`SimStartRun`, `SimStopRun`, and optional `JoystickCommand`, and returns `SimulatorTelemetry`,
`SimStartAck`, and `SimRunDone`.
Internal-only messages such as `ImuData` and `MotorFeedback` do not cross either practical boundary.

The generated C++ bridge contract also lists simulator-related message IDs so the reflected schema
can describe the complete family of runtime messages. That generated inventory should not be read
as evidence that the production Pi process is running a simulator service on port `9000`.

## Failure behavior

The controller stops sending nonzero motor targets when it has no valid IMU, receives stale or
future IMU data, detects fallover, or sees an actuator fault. It resets dynamic command state and
keeps the fault visible in telemetry. After fallover, the current local re-arm rule requires pitch
within 10 degrees and pitch rate within 30 degrees per second, unless a nonzero forward command
requests recovery. Network connectivity is not part of that decision; see
[`RateControllerCore`](../../src/services/control/rate_controller_core.cpp).

## Related pages

- [Project overview](../overview.md) — modes, message roles, and the reader-level mental model.
- [MessageBus and UDP Bridge](message_hub.md) — lower-level dispatch and transport behavior.
- [Generated IPC protocol](../ipc/protocol.md) — exact message topology and wire layout.
- [Control and plant model](control_plant.md) — controller equations and plant mapping.
- [IMU attitude design](imu_attitude_design.md) — sensor axes, filtering, and estimator limits.
