# IPC Protocol Reference

[Home](../../README.md)

> **Auto-generated** by `generate_balancer_docs` using C++26 static reflection (P2996 + P3394).
> Do not edit by hand. Re-run `cmake --build build --target balancer_docs` to refresh.

## Overview

This document is generated from the balancer runtime message registry and the reflected payload types in `src/messages/`.
It describes the reflected runtime message bus used by the balancer services, including the UDP-facing
messages consumed by the SIL harness and the internal-only messages exchanged between services.

- Documented balancer message count: `12`
- Protocol hash: `75636990e95c6daa`
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

> Consumes raw accelerometer/gyroscope samples and publishes `ImuData` samples that represent the controller's current view of body pitch, angular motion, and sample time.
>
> The hardware reader converts synchronized sensor samples into SI-valued robot axes and publishes `ImuRawData`. This service applies 15 Hz accelerometer and 30 Hz gyro two-pole low-pass filters plus a 10 Hz filtered gyro derivative. Full-circle gravity pitch corrects short-term gyro prediction at 0.5 Hz, with each innovation limited symmetrically to 2.5 degrees so translation and motor vibration cannot abruptly steer attitude. The optional fixed notch and 70 mm lever-arm correction are disabled by default. It never learns gyro bias, mounting, gravity recovery modes, or COM correction, and marks invalid input invalid.
>
> SIL can disable the hardware reader and inject `ImuRawData` through `UdpBridge` while using the same estimator. `ImuData` remains an internal controller-facing contract.

- Publishes: `ImuRawData`, `ImuData`
- Subscribes: `ImuRawData`

### `TimeService`

> Publishes the global `PhysicsTick` heartbeat that drives deterministic controller execution and advances the shared simulation clock.
>
> The service supports two operating modes. In runtime mode it owns a worker thread that sleeps against `std::chrono::steady_clock` and emits ticks at the configured default cadence. In SIL or test mode it can instead be advanced explicitly by callers, which lets the rest of the system run from a fully deterministic external timeline instead of wall clock time. The default timestep is currently `1 / 400 s`, so the nominal scheduler frequency is about `400 Hz`.
>
> Deterministic ticks increment a zero-based timestamp. Runtime ticks publish an absolute `std::chrono::steady_clock` timestamp and the elapsed wall-clock `dt_s`. `ControlService` consumes these ticks as the authoritative integration step, so keeping this service as the sole owner of tick publication prevents divergent notions of time across hardware, SIL replay, and unit tests.

- Publishes: `PhysicsTick`
- Subscribes: _None_

### `ControlService`

> Owns the balancing control pipeline that converts `PhysicsTick`, `ImuData`, and `JoystickCommand`, and `MotorFeedback` inputs into wheel-speed targets and streaming controller telemetry.
>
> At 100 Hz, completed common-mode steps are corrected for chassis pitch to observe axle velocity and filtered at 10 Hz. A jerk-limited acceleration request plus corrected-velocity damping form the pitch reference, while a bounded integral term learns only stationary center-of-mass trim:
>
> $$ \theta_{sp} = \operatorname{atan2}(a_{nominal} - k_v v_{axle},g) + \theta_{COM} $$
>
> $$ \omega_{sp} = k_{pitch}(\theta_{sp} - \theta) - k_{pitch\_rate}\dot{\theta} $$
>
> The pitch-rate controller supplies the wheel command before turn allocation. Motor output can initially reduce or reverse to acquire lean. Faults clear dynamic state but preserve bounded COM trim. Telemetry reports the pitch-reference terms, target/post-slew/applied commands, feedback, saturation, and faults. In `actuator_saturation_flags`, bit 0 is left slew limiting and bi

- Publishes: `MotorTargets`, `SystemTelemetry`
- Subscribes: `PhysicsTick`, `ImuData`, `JoystickCommand`, `MotorFeedback`

### `MotorService`

> Implements the actuator boundary between reflected IPC commands and the low-level motor runner.
>
> The service subscribes to `PhysicsTick` and `MotorTargets` and deliberately contains almost no control state of its own. Its job is to remember the latest physics timestamp, accept wheel-speed targets expressed in steps per second, and forward them to the configured `MotorRunner` if one is attached:
>
> $$ u_L, u_R \; [\mathrm{steps/s}] \rightarrow \texttt{MotorRunner::setTargets}(u_L, u_R) $$
>
> Keeping this service narrow is intentional. Closed-loop balance, velocity estimation, and telemetry all remain in `ControlService` and `RateControllerCore`, while hardware-specific pulse generation, slew limiting, and direction control remain below this layer in the motor runner. The service also listens for `PhysicsTick` so it can keep the runner aligned with the current physics time before forwarding motor targets. When hardware is present the service also republishes the runner's continuous post-slew command, pulse-frame-a

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
- Subscribes: `MotorTargets`, `SystemTelemetry`, `SimulatorTelemetry`, `SimStartAck`, `SimRunDone`

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
| `timestamp_us` | `uint64_t` | `int` | 8 | 8 |  |

### `MsgId::ImuData`

- Numeric ID: `3000`
- Payload type: `ImuSamplePayload`
- Python type: `ImuSamplePayload`
- Wire size: `88` bytes
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
| `estimate_valid` | `bool` | `bool` | 1 | 80 |  |

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
- Wire size: `144` bytes
- Published by: `ControlService`
- Consumed by: `UdpBridge`

| Field | C++ Type | Python Type | Bytes | Offset | Description |
|---|---|---|---:|---:|---|
| `run_id` | `uint32_t` | `int` | 4 | 0 |  |
| `controller_fault_flags` | `uint32_t` | `int` | 4 | 4 |  |
| `controller_saturation_flags` | `uint32_t` | `int` | 4 | 8 |  |
| `imu_timestamp_us` | `uint64_t` | `int` | 8 | 16 |  |
| `t_sec` | `float` | `float` | 4 | 24 |  |
| `age_ms` | `float` | `float` | 4 | 28 |  |
| `pitch_deg` | `float` | `float` | 4 | 32 |  |
| `pitch_rate_dps` | `float` | `float` | 4 | 36 |  |
| `raw_acc_pitch_deg` | `float` | `float` | 4 | 40 |  |
| `fused_pitch_deg` | `float` | `float` | 4 | 44 |  |
| `gyro_pitch_rate_dps` | `float` | `float` | 4 | 48 |  |
| `filtered_pitch_rate_dps` | `float` | `float` | 4 | 52 |  |
| `u_sps` | `float` | `float` | 4 | 56 |  |
| `turn_sps` | `float` | `float` | 4 | 60 |  |
| `nominal_acceleration_mps2` | `float` | `float` | 4 | 64 |  |
| `raw_completed_velocity_sps` | `float` | `float` | 4 | 68 |  |
| `corrected_axle_velocity_sps` | `float` | `float` | 4 | 72 |  |
| `velocity_damping_acceleration_mps2` | `float` | `float` | 4 | 76 |  |
| `com_trim_deg` | `float` | `float` | 4 | 80 |  |
| `pitch_error_deg` | `float` | `float` | 4 | 84 |  |
| `pitch_sp_deg` | `float` | `float` | 4 | 88 |  |
| `rate_setpoint_dps` | `float` | `float` | 4 | 92 |  |
| `rate_error_dps` | `float` | `float` | 4 | 96 |  |
| `left_target_sps` | `float` | `float` | 4 | 100 |  |
| `right_target_sps` | `float` | `float` | 4 | 104 |  |
| `left_slewed_sps` | `float` | `float` | 4 | 108 |  |
| `right_slewed_sps` | `float` | `float` | 4 | 112 |  |
| `motor_update_dt_ms` | `float` | `float` | 4 | 116 |  |
| `motor_feedback_age_ms` | `float` | `float` | 4 | 120 |  |
| `left_actual_steps` | `int32_t` | `int` | 4 | 124 |  |
| `right_actual_steps` | `int32_t` | `int` | 4 | 128 |  |
| `actuator_saturation_flags` | `uint32_t` | `int` | 4 | 132 |  |
| `command_saturated` | `bool` | `bool` | 1 | 136 |  |
| `actuator_fault` | `bool` | `bool` | 1 | 137 |  |

### `MsgId::MotorFeedback`

- Numeric ID: `3004`
- Payload type: `MotorFeedbackPayload`
- Python type: `MotorFeedbackPayload`
- Wire size: `64` bytes
- Published by: `MotorService`
- Consumed by: `ControlService`

| Field | C++ Type | Python Type | Bytes | Offset | Description |
|---|---|---|---:|---:|---|
| `left_slewed_sps` | `double` | `float` | 8 | 0 |  |
| `right_slewed_sps` | `double` | `float` | 8 | 8 |  |
| `measured_avg_sps` | `double` | `float` | 8 | 16 |  |
| `update_dt_ms` | `double` | `float` | 8 | 24 |  |
| `feedback_age_ms` | `double` | `float` | 8 | 32 |  |
| `left_actual_steps` | `int64_t` | `int` | 8 | 40 |  |
| `right_actual_steps` | `int64_t` | `int` | 8 | 48 |  |
| `actuator_saturation_flags` | `uint32_t` | `int` | 4 | 56 |  |
| `actuator_fault` | `uint8_t` | `int` | 1 | 60 |  |

### `MsgId::SimStartRun`

- Numeric ID: `3005`
- Payload type: `SimStartRunPayload`
- Python type: `SimStartRunPayload`
- Wire size: `1120` bytes
- Published by: `UdpBridge`
- Consumed by: _None_

| Field | C++ Type | Python Type | Bytes | Offset | Description |
|---|---|---|---:|---:|---|
| `run_id` | `uint32_t` | `int` | 4 | 0 |  |
| `physics_profile` | `uint8_t` | `int` | 1 | 4 |  |
| `has_physics_override` | `uint8_t` | `int` | 1 | 5 |  |
| `telemetry_stride` | `uint16_t` | `int` | 2 | 6 |  |
| `transfer_scenario_index` | `uint16_t` | `int` | 2 | 8 |  |
| `reserved1` | `uint16_t` | `int` | 2 | 10 |  |
| `duration_s` | `double` | `float` | 8 | 16 |  |
| `initial_pitch_deg` | `double` | `float` | 8 | 24 |  |
| `com_angle_offset_rad` | `double` | `float` | 8 | 32 |  |
| `total_mass_scale` | `double` | `float` | 8 | 40 |  |
| `pitch_inertia_scale` | `double` | `float` | 8 | 48 |  |
| `motor_max_force_n` | `double` | `float` | 8 | 56 |  |
| `motor_no_load_speed_mps` | `double` | `float` | 8 | 64 |  |
| `motor_velocity_damping` | `double` | `float` | 8 | 72 |  |
| `motor_tau_s` | `double` | `float` | 8 | 80 |  |
| `traction_coefficient` | `double` | `float` | 8 | 88 |  |
| `pitch_damping` | `double` | `float` | 8 | 96 |  |
| `cart_damping` | `double` | `float` | 8 | 104 |  |
| `phase_error_limit_steps` | `double` | `float` | 8 | 112 |  |
| `tire_stiffness_n_per_m` | `double` | `float` | 8 | 120 |  |
| `tire_damping_n_s_per_m` | `double` | `float` | 8 | 128 |  |
| `wheel_equivalent_mass_kg` | `double` | `float` | 8 | 136 |  |
| `imu_pitch_lag_s` | `double` | `float` | 8 | 144 |  |
| `imu_noise_seed` | `uint32_t` | `int` | 4 | 152 |  |
| `accel_noise_std_mps2` | `double` | `float` | 8 | 160 |  |
| `gyro_noise_std_rad_s` | `double` | `float` | 8 | 168 |  |
| `imu_timestamp_jitter_us` | `double` | `float` | 8 | 176 |  |
| `imu_sample_loss_rate` | `double` | `float` | 8 | 184 |  |
| `accel_bias_mps2` | `std::array<double, 3>` | `list[float]` | 24 | 192 |  |
| `gyro_bias_rad_s` | `std::array<double, 3>` | `list[float]` | 24 | 216 |  |
| `disturbances` | `std::array<SimDisturbancePayload, 10>` | `list[SimDisturbancePayload]` | 560 | 240 |  |
| `joy_segments` | `std::array<SimJoySegmentPayload, 4>` | `list[SimJoySegmentPayload]` | 192 | 800 |  |
| `pid_config_path` | `std::array<char, 128>` | `bytes` | 128 | 992 |  |

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
- Wire size: `104` bytes
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
| `max_continuous_saturation_s` | `double` | `float` | 8 | 80 |  |
| `actuator_fault_count` | `uint32_t` | `int` | 4 | 88 |  |
| `controller_fault_flags` | `uint32_t` | `int` | 4 | 92 |  |
| `timeline_hash` | `uint64_t` | `int` | 8 | 96 |  |

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

### `MsgId::SimulatorTelemetry`

- Numeric ID: `3010`
- Payload type: `SimulatorTelemetryPayload`
- Python type: `SimulatorTelemetryPayload`
- Wire size: `272` bytes
- Published by: _None_
- Consumed by: `UdpBridge`

| Field | C++ Type | Python Type | Bytes | Offset | Description |
|---|---|---|---:|---:|---|
| `system` | `SystemTelemetryPayload` | `SystemTelemetryPayload` | 144 | 0 |  |
| `seed` | `uint32_t` | `int` | 4 | 144 |  |
| `plant_pitch_deg` | `float` | `float` | 4 | 148 |  |
| `plant_pitch_rate_dps` | `float` | `float` | 4 | 152 |  |
| `plant_position_m` | `float` | `float` | 4 | 156 |  |
| `plant_velocity_mps` | `float` | `float` | 4 | 160 |  |
| `target_wheel_velocity` | `float` | `float` | 4 | 164 |  |
| `actual_wheel_velocity` | `float` | `float` | 4 | 168 |  |
| `plant_velocity_error` | `float` | `float` | 4 | 172 |  |
| `f_cmd` | `float` | `float` | 4 | 176 |  |
| `f_app` | `float` | `float` | 4 | 180 |  |
| `external_force_n` | `float` | `float` | 4 | 184 |  |
| `external_com_bias_rad` | `float` | `float` | 4 | 188 |  |
| `x_ddot` | `float` | `float` | 4 | 192 |  |
| `theta_ddot` | `float` | `float` | 4 | 196 |  |
| `phase_error_steps` | `float` | `float` | 4 | 200 |  |
| `missed_steps` | `float` | `float` | 4 | 204 |  |
| `traction_limit_n` | `float` | `float` | 4 | 208 |  |
| `motor_force_limit_n` | `float` | `float` | 4 | 212 |  |
| `total_mass_scale` | `float` | `float` | 4 | 216 |  |
| `pitch_inertia_scale` | `float` | `float` | 4 | 220 |  |
| `motor_max_force_n` | `float` | `float` | 4 | 224 |  |
| `motor_no_load_speed_mps` | `float` | `float` | 4 | 228 |  |
| `motor_velocity_damping` | `float` | `float` | 4 | 232 |  |
| `motor_tau_s` | `float` | `float` | 4 | 236 |  |
| `traction_coefficient` | `float` | `float` | 4 | 240 |  |
| `pitch_damping` | `float` | `float` | 4 | 244 |  |
| `cart_damping` | `float` | `float` | 4 | 248 |  |
| `phase_error_limit_steps` | `float` | `float` | 4 | 252 |  |
| `tire_stiffness_n_per_m` | `float` | `float` | 4 | 256 |  |
| `tire_damping_n_s_per_m` | `float` | `float` | 4 | 260 |  |
| `wheel_equivalent_mass_kg` | `float` | `float` | 4 | 264 |  |
| `force_saturated` | `bool` | `bool` | 1 | 268 |  |

---

## Regenerating This File

```bash
cmake -S . -B build
cmake --build build --target balancer_docs
```

_Generated with GCC trunk `-std=c++26 -freflection`._
