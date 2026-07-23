# IPC Protocol Reference

[Home](../../README.md)

> **Auto-generated** by `generate_balancer_docs` using C++26 static reflection (P2996 + P3394).
> Do not edit by hand. Re-run `cmake --build build --target balancer_docs` to refresh.

## Overview

This document is generated from the balancer runtime message registry and the reflected payload types in `src/messages/`.
It describes the reflected runtime message bus used by the balancer services, including the UDP-facing
messages consumed by the SIL harness and the internal-only messages exchanged between services.

- Documented balancer message count: `11`
- Protocol hash: `e9f92793dfef58a7`
- UDP ingress/egress gateway: `UdpBridge`

## System Architecture

The architecture is divided into three logical areas:

1. **Pytest Harness**: Python fixtures send fixed-size UDP datagrams and decode telemetry using the generated bindings.
2. **Network Layer**: `UdpBridge` translates between UDP traffic and the internal message bus.
3. **Balancer Services**: services publish and consume reflected payload structs on the bus.

![IPC Flow Diagram](ipc_flow.svg)

---

## Component Services

### `ImuService`

> Consumes raw accelerometer/gyroscope samples and publishes `ImuData` samples that represent the controller's current view of body pitch, specific force, angular rate, and sample time.
>
> When hardware reading is enabled, the service owns an `Ism330IioReader` that discovers the split accel/gyro IIO devices, converts raw sensor counts into SI units, timestamps each sample, and publishes `ImuRawData` onto the internal message bus. The raw accelerometer and gyroscope vectors are then fused by a complementary filter before `ImuData` reaches control.
>
> In SIL mode the hardware reader can be disabled entirely, but Python can still inject `ImuRawData` through `UdpBridge` to exercise the same filter path. `ImuData` remains an internal controller-facing contract rather than a UDP payload.

- Publishes: `ImuRawData`, `ImuData`
- Subscribes: `ImuRawData`

### `TimeService`

> Publishes the global `PhysicsTick` heartbeat that drives deterministic controller execution and advances the shared simulation clock.
>
> The service supports two operating modes. In runtime mode it owns a worker thread that sleeps against `std::chrono::steady_clock` and emits ticks at the configured default cadence. In SIL or test mode it can instead be advanced explicitly by callers, which lets the rest of the system run from a fully deterministic external timeline instead of wall clock time. The default timestep is currently `1 / 400 s`, so the nominal scheduler frequency is about `400 Hz`.
>
> Each tick increments the monotonically increasing elapsed timestamp and publishes
>
> $$ t_{sim,us} \leftarrow t_{sim,us} + \Delta t \cdot 10^6 $$
>
> with the exact `dt_s` used for that step embedded in the payload. `ControlService` consumes these ticks as the authoritative integration step, so keeping this service as the sole owner of tick publication prevents divergent notions of time across hardware, SIL replay, and unit t

- Publishes: `PhysicsTick`
- Subscribes: _None_

### `ControlService`

> Owns the balancing control pipeline that converts `PhysicsTick`, `ImuData`, `JoystickCommand`, and `MotorFeedback` inputs into actuator commands and streaming controller telemetry.
>
> This service is intentionally thin: it caches the latest bus inputs, translates them into the `RateControllerCore` API, and republishes the core's outputs as reflected IPC payloads. The control law is an acceleration-to-pitch outer loop wrapped around the PX4 pitch-rate controller. A joystick forward command requests longitudinal acceleration; the request is jerk-limited, velocity damping is subtracted, and the result is converted to a pitch reference before lean trim is added. The PX4 `RateControl` block then tracks a damped pitch-rate setpoint:
>
> $$ a_{sp} = \operatorname{jerkLimit}(a_{joy} - k_v v) $$
>
> $$ \theta_{sp} = \arcsin(\operatorname{clamp}(a_{sp}/g)) + \theta_{trim} $$
>
> $$ \omega_{sp} = k_{pitch}(\theta_{sp} - \theta) - k_{pitch\_rate}\dot{\theta} $$
>
> The resulting normalized pitch-axis effort is scaled into motor commands in steps per second, clamped to the configured ceiling, and split into left/right wheel targets by adding a tu

- Publishes: `MotorTargets`, `SystemTelemetry`
- Subscribes: `PhysicsTick`, `ImuData`, `JoystickCommand`, `MotorFeedback`

### `MotorService`

> Implements the actuator boundary between reflected IPC commands and the low-level motor runner.
>
> The service subscribes to `PhysicsTick` and `MotorTargets` and deliberately contains almost no control state of its own. Its job is to remember the latest physics timestamp, accept wheel-speed targets expressed in steps per second, and forward them to the configured `MotorRunner` if one is attached:
>
> $$ u_L, u_R \; [\mathrm{steps/s}] \rightarrow \texttt{MotorRunner::setTargets}(u_L, u_R) $$
>
> Keeping this service narrow is intentional. Closed-loop balance, trim estimation, and telemetry all remain in `ControlService` and `RateControllerCore`, while hardware-specific pulse generation, slew limiting, and direction control remain below this layer in the motor runner. The service also listens for `PhysicsTick` so it can keep the runner aligned with the current physics time before forwarding motor targets. When hardware is present the service also republishes the runner's applied rate, steps-derived average speed estim

- Publishes: `MotorFeedback`
- Subscribes: `PhysicsTick`, `MotorTargets`

### `InputService`

> Reads input from a hardware Xbox controller and publishes normalized `JoystickCommand` messages to the bus.
>
> This service isolates the platform-dependent gamepad reading (SDL2) from the main application logic. It runs a dedicated worker thread at a fixed cadence, polling the controller state and emitting standard forward/turn commands. This allows the controller to be replaced or simulated by external sources like Python tests or UDP injection without modifying the balancing application.

- Publishes: `JoystickCommand`
- Subscribes: _None_

### `UdpBridge`

> Stateful transport bridge that connects the internal `MessageBus` to external UDP-based SIL clients.
>
> On ingress, the bridge binds a UDP socket on port `9000`, receives datagrams from the latest test harness peer, extracts the leading `uint16_t` message identifier, and republishes the remaining payload bytes through `publish_if_authorized`. That keeps external injection limited to the message types the bridge explicitly advertises in `Publishes`, and the downstream bus path retains ownership of payload-size checks before handlers see any data.
>
> On egress, the bridge remembers the most recent sender address and uses it as the return path for outbound telemetry and motor command traffic. Each authorized outbound message is encoded as the reflected message ID followed immediately by the trivially-copyable payload bytes:
>
> $$ \text{datagram} = \texttt{uint16\_t MsgId} \; || \; \texttt{Payload bytes} $$
>
> This makes the UDP contract symmetric with the Python bindings generated from the same message definitions. Ope

- Publishes: `PhysicsTick`, `JoystickCommand`, `ImuRawData`, `SimStartRun`, `SimStopRun`
- Subscribes: `MotorTargets`, `SystemTelemetry`, `SimStartAck`, `SimRunDone`

---

## Message Payloads

Each section corresponds to one reflected balancer message. Some are exposed over UDP, while others are
internal-only service messages. Wire sizes come directly from `sizeof(Payload)`.

### `MsgId::PhysicsTick`

- Numeric ID: `1`
- Payload type: `PhysicsTickPayload`
- Python type: `PhysicsTickPayload`
- Wire size: `16` bytes
- Published by: `TimeService`, `UdpBridge`
- Consumed by: `ControlService`, `MotorService`

| Field | C++ Type | Python Type | Bytes | Offset | Description |
|---|---|---|---:|---:|---|
| `dt_s` | `double` | `float` | 8 | 0 |  |
| `sim_time_us` | `uint64_t` | `int` | 8 | 8 |  |

### `MsgId::ImuData`

- Numeric ID: `3000`
- Payload type: `ImuSamplePayload`
- Python type: `ImuSamplePayload`
- Wire size: `80` bytes
- Published by: `ImuService`
- Consumed by: `ControlService`

| Field | C++ Type | Python Type | Bytes | Offset | Description |
|---|---|---|---:|---:|---|
| `pitch_rad` | `double` | `float` | 8 | 0 |  |
| `pitch_rate_rad_s` | `double` | `float` | 8 | 8 |  |
| `pitch_accel_rad_s2` | `double` | `float` | 8 | 16 |  |
| `acc` | `std::array<double, 3>` | `list[float]` | 24 | 24 |  |
| `gyr` | `std::array<double, 3>` | `list[float]` | 24 | 48 |  |
| `timestamp_us` | `uint64_t` | `int` | 8 | 72 |  |

### `MsgId::JoystickCommand`

- Numeric ID: `3001`
- Payload type: `JoystickCommandPayload`
- Python type: `JoystickCommandPayload`
- Wire size: `16` bytes
- Published by: `InputService`, `UdpBridge`
- Consumed by: `ControlService`

| Field | C++ Type | Python Type | Bytes | Offset | Description |
|---|---|---|---:|---:|---|
| `forward` | `double` | `float` | 8 | 0 |  |
| `turn` | `double` | `float` | 8 | 8 |  |

### `MsgId::MotorTargets`

- Numeric ID: `3002`
- Payload type: `MotorTargetsPayload`
- Python type: `MotorTargetsPayload`
- Wire size: `16` bytes
- Published by: `ControlService`
- Consumed by: `MotorService`, `UdpBridge`

| Field | C++ Type | Python Type | Bytes | Offset | Description |
|---|---|---|---:|---:|---|
| `left_sps` | `double` | `float` | 8 | 0 |  |
| `right_sps` | `double` | `float` | 8 | 8 |  |

### `MsgId::SystemTelemetry`

- Numeric ID: `3003`
- Payload type: `SystemTelemetryPayload`
- Python type: `SystemTelemetryPayload`
- Wire size: `312` bytes
- Published by: `ControlService`
- Consumed by: `UdpBridge`

| Field | C++ Type | Python Type | Bytes | Offset | Description |
|---|---|---|---:|---:|---|
| `run_id` | `uint32_t` | `int` | 4 | 0 |  |
| `t_sec` | `double` | `float` | 8 | 8 |  |
| `age_ms` | `double` | `float` | 8 | 16 |  |
| `pitch_deg` | `double` | `float` | 8 | 24 |  |
| `pitch_rate_dps` | `double` | `float` | 8 | 32 |  |
| `raw_acc_pitch_deg` | `double` | `float` | 8 | 40 |  |
| `fused_pitch_deg` | `double` | `float` | 8 | 48 |  |
| `gyro_pitch_rate_dps` | `double` | `float` | 8 | 56 |  |
| `filtered_pitch_rate_dps` | `double` | `float` | 8 | 64 |  |
| `u_sps` | `double` | `float` | 8 | 72 |  |
| `turn_sps` | `double` | `float` | 8 | 80 |  |
| `vel_error` | `double` | `float` | 8 | 88 |  |
| `measured_vel_sps` | `double` | `float` | 8 | 96 |  |
| `vel_p_term` | `double` | `float` | 8 | 104 |  |
| `pitch_ref_from_vel_deg` | `double` | `float` | 8 | 112 |  |
| `pitch_error_deg` | `double` | `float` | 8 | 120 |  |
| `pitch_sp_deg` | `double` | `float` | 8 | 128 |  |
| `pitch_trim_deg` | `double` | `float` | 8 | 136 |  |
| `trim_active` | `double` | `float` | 8 | 144 |  |
| `left_applied_sps` | `double` | `float` | 8 | 152 |  |
| `right_applied_sps` | `double` | `float` | 8 | 160 |  |
| `motor_update_dt_ms` | `double` | `float` | 8 | 168 |  |
| `motor_feedback_age_ms` | `double` | `float` | 8 | 176 |  |
| `left_actual_steps` | `int64_t` | `int` | 8 | 184 |  |
| `right_actual_steps` | `int64_t` | `int` | 8 | 192 |  |
| `plant_pitch_deg` | `double` | `float` | 8 | 200 |  |
| `plant_pitch_rate_dps` | `double` | `float` | 8 | 208 |  |
| `plant_position_m` | `double` | `float` | 8 | 216 |  |
| `plant_velocity_mps` | `double` | `float` | 8 | 224 |  |
| `target_wheel_velocity` | `double` | `float` | 8 | 232 |  |
| `actual_wheel_velocity` | `double` | `float` | 8 | 240 |  |
| `plant_velocity_error` | `double` | `float` | 8 | 248 |  |
| `f_cmd` | `double` | `float` | 8 | 256 |  |
| `f_app` | `double` | `float` | 8 | 264 |  |
| `external_force_n` | `double` | `float` | 8 | 272 |  |
| `external_com_bias_rad` | `double` | `float` | 8 | 280 |  |
| `x_ddot` | `double` | `float` | 8 | 288 |  |
| `theta_ddot` | `double` | `float` | 8 | 296 |  |
| `force_saturated` | `double` | `float` | 8 | 304 |  |

### `MsgId::MotorFeedback`

- Numeric ID: `3004`
- Payload type: `MotorFeedbackPayload`
- Python type: `MotorFeedbackPayload`
- Wire size: `56` bytes
- Published by: `MotorService`
- Consumed by: `ControlService`

| Field | C++ Type | Python Type | Bytes | Offset | Description |
|---|---|---|---:|---:|---|
| `left_applied_sps` | `double` | `float` | 8 | 0 |  |
| `right_applied_sps` | `double` | `float` | 8 | 8 |  |
| `measured_avg_sps` | `double` | `float` | 8 | 16 |  |
| `update_dt_ms` | `double` | `float` | 8 | 24 |  |
| `feedback_age_ms` | `double` | `float` | 8 | 32 |  |
| `left_actual_steps` | `int64_t` | `int` | 8 | 40 |  |
| `right_actual_steps` | `int64_t` | `int` | 8 | 48 |  |

### `MsgId::SimStartRun`

- Numeric ID: `3005`
- Payload type: `SimStartRunPayload`
- Python type: `SimStartRunPayload`
- Wire size: `824` bytes
- Published by: `UdpBridge`
- Consumed by: _None_

| Field | C++ Type | Python Type | Bytes | Offset | Description |
|---|---|---|---:|---:|---|
| `run_id` | `uint32_t` | `int` | 4 | 0 |  |
| `physics_profile` | `uint8_t` | `int` | 1 | 4 |  |
| `reserved0` | `uint8_t` | `int` | 1 | 5 |  |
| `reserved1` | `uint16_t` | `int` | 2 | 6 |  |
| `duration_s` | `double` | `float` | 8 | 8 |  |
| `initial_pitch_deg` | `double` | `float` | 8 | 16 |  |
| `com_angle_offset_rad` | `double` | `float` | 8 | 24 |  |
| `wheel_slip_factor` | `double` | `float` | 8 | 32 |  |
| `velocity_feedback_scale` | `double` | `float` | 8 | 40 |  |
| `velocity_feedback_tau_s` | `double` | `float` | 8 | 48 |  |
| `imu_pitch_lag_s` | `double` | `float` | 8 | 56 |  |
| `imu_noise_seed` | `uint32_t` | `int` | 4 | 64 |  |
| `accel_noise_std_mps2` | `double` | `float` | 8 | 72 |  |
| `gyro_noise_std_rad_s` | `double` | `float` | 8 | 80 |  |
| `accel_bias_mps2` | `std::array<double, 3>` | `list[float]` | 24 | 88 |  |
| `gyro_bias_rad_s` | `std::array<double, 3>` | `list[float]` | 24 | 112 |  |
| `disturbances` | `std::array<SimDisturbancePayload, 10>` | `list[SimDisturbancePayload]` | 560 | 136 |  |
| `pid_config_path` | `std::array<char, 128>` | `bytes` | 128 | 696 |  |

### `MsgId::SimStartAck`

- Numeric ID: `3006`
- Payload type: `SimStartAckPayload`
- Python type: `SimStartAckPayload`
- Wire size: `8` bytes
- Published by: _None_
- Consumed by: `UdpBridge`

| Field | C++ Type | Python Type | Bytes | Offset | Description |
|---|---|---|---:|---:|---|
| `run_id` | `uint32_t` | `int` | 4 | 0 |  |
| `accepted` | `uint8_t` | `int` | 1 | 4 |  |
| `status_code` | `uint8_t` | `int` | 1 | 5 |  |
| `reserved` | `uint16_t` | `int` | 2 | 6 |  |

### `MsgId::SimStopRun`

- Numeric ID: `3007`
- Payload type: `SimStopRunPayload`
- Python type: `SimStopRunPayload`
- Wire size: `4` bytes
- Published by: `UdpBridge`
- Consumed by: _None_

| Field | C++ Type | Python Type | Bytes | Offset | Description |
|---|---|---|---:|---:|---|
| `run_id` | `uint32_t` | `int` | 4 | 0 |  |

### `MsgId::SimRunDone`

- Numeric ID: `3008`
- Payload type: `SimRunDonePayload`
- Python type: `SimRunDonePayload`
- Wire size: `80` bytes
- Published by: _None_
- Consumed by: `UdpBridge`

| Field | C++ Type | Python Type | Bytes | Offset | Description |
|---|---|---|---:|---:|---|
| `run_id` | `uint32_t` | `int` | 4 | 0 |  |
| `reason_code` | `uint8_t` | `int` | 1 | 4 |  |
| `reserved0` | `uint8_t` | `int` | 1 | 5 |  |
| `reserved1` | `uint16_t` | `int` | 2 | 6 |  |
| `sample_count` | `uint32_t` | `int` | 4 | 8 |  |
| `elapsed_s` | `double` | `float` | 8 | 16 |  |
| `final_pitch_deg` | `double` | `float` | 8 | 24 |  |
| `max_abs_pitch_deg` | `double` | `float` | 8 | 32 |  |
| `tail_rms_pitch_deg` | `double` | `float` | 8 | 40 |  |
| `tail_rail_fraction` | `double` | `float` | 8 | 48 |  |
| `tail_mean_abs_pitch_deg` | `double` | `float` | 8 | 56 |  |
| `max_abs_position_m` | `double` | `float` | 8 | 64 |  |
| `tail_mean_abs_velocity_mps` | `double` | `float` | 8 | 72 |  |

### `MsgId::ImuRawData`

- Numeric ID: `3009`
- Payload type: `ImuRawPayload`
- Python type: `ImuRawPayload`
- Wire size: `56` bytes
- Published by: `ImuService`, `UdpBridge`
- Consumed by: `ImuService`

| Field | C++ Type | Python Type | Bytes | Offset | Description |
|---|---|---|---:|---:|---|
| `acc` | `std::array<double, 3>` | `list[float]` | 24 | 0 |  |
| `gyr` | `std::array<double, 3>` | `list[float]` | 24 | 24 |  |
| `timestamp_us` | `uint64_t` | `int` | 8 | 48 |  |

---

## Regenerating This File

```bash
cmake -S . -B build
cmake --build build --target balancer_docs
```

_Generated with GCC trunk `-std=c++26 -freflection`._
