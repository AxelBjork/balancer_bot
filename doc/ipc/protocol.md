# IPC Protocol Reference

[Home](../../README.md)

> **Auto-generated** by `generate_balancer_docs` using C++26 static reflection (P2996 + P3394).
> Do not edit by hand. Re-run `cmake --build build --target balancer_docs` to refresh.

## Overview

This document is generated from the balancer runtime message registry and the reflected payload types in `src/messages/`.
It describes the reflected runtime message bus used by the balancer services, including the production
UDP runtime API and the internal-only messages exchanged between services.

- Documented balancer message count: `15`
- Protocol hash: `85ce2b2c9452f623`
- UDP ingress/egress gateway: `UdpBridge`

## System Architecture

The architecture is divided into three logical areas:

1. **Client**: The telemetry server is the primary peer; SIL uses the same generated bindings and bridge contract.
2. **Network Layer**: `UdpBridge` translates between UDP traffic and the internal message bus on the production port-9000 boundary.
3. **Balancer Services**: services publish and consume reflected payload structs on the bus.

![IPC Flow Diagram](ipc_flow.svg)

---

## Component Services

### `ImuService`

> Consumes raw accelerometer/gyroscope samples and publishes `ImuData` samples that represent the controller's current view of body pitch, angular motion, and sample time.
>
> The hardware reader converts synchronized sensor samples into SI-valued robot axes and publishes `ImuRawData`. The reader configures the ISM330 gyroscope at 833 Hz with chip-side LPF1 enabled at 140 Hz and verifies the register readback. This service then applies locked 26.9 Hz / 14 Hz and 33.4 Hz / 32 Hz gyro notches. The accelerometer retains its 15 Hz two-pole low-pass and the gyro derivative retains its 10 Hz filter. Full-circle gravity pitch corrects short-term gyro prediction at 0.5 Hz, with each innovation limited symmetrically to 2.5 degrees so translation and motor vibration cannot abruptly steer attitude. The fixed notches are compiled into the production IMU path rather than exposed as runtime parameters; 70 mm lever-arm correction remains disabled. It never learns gyro bias, mounting, gravity recovery modes, or COM correction, and marks invalid input invalid.
>
> SIL can disable the hardware reader and inject `ImuRawData` through `UdpBridge` while using the same estimator implementation. `ImuData` remains an internal controller-facing contract.

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

> Owns the balancing control pipeline that converts `PhysicsTick`, `ImuData`, `JoystickCommand`, and `MotorFeedback` inputs into wheel-speed targets and streaming controller telemetry.
>
> At 100 Hz, completed common-mode steps are corrected for chassis pitch to observe axle velocity and filtered at 10 Hz. A separate configurable velocity-control filter can add a slower velocity-feedback pole without changing that observer. A reversal-aware velocity reference planner produces v_ref and a_ref; velocity feedback then contributes to one shared acceleration authority before conversion to a motion pitch target:
>
> $$ a_{raw} = a_{ref} + K_v(v_{ref} - v_{feedback}), \quad a_{cmd} = \mathrm{clamp}(a_{raw}, \pm g\tan(\theta_{limit})) $$
>
> $$ \theta_{sp} = \mathrm{atan2}(a_{cmd},g) + \theta_{COM} $$
>
> The optional adaptive COM learner is disabled in the v1 configuration; fixed COM trim is the only default trim input.
>
> The attitude controller uses independently configured state feedback at the robot-forward motor boundary:
>
> $$ u = K_{pitch}(\theta-\theta_{sp}) + K_{rate}\dot{\theta} + K_{accel}\ddot{\theta} $$
>
> The signs above account for the electrical/motor-boundary polarity; they are equivalent to the public negative-feedback form in the controller's internal rate convention. The explicit terms are independently tunable and reported in telemetry. The attitude controller supplies a balance correction around nominal drive before final common limiting and turn allocation. Balance can therefore cancel or reverse drive. Motor output can initially reduce or reverse to acquire lean. Faults clear dynamic state but preserve bounded COM trim. Telemetry reports the pitch-reference terms, target/post-slew/applied commands, feedback, saturation, and faults. In `actuator_saturation_flags`, bit 0 is left slew limiting and bit 1 is right slew limiting.

- Publishes: `MotorTargets`, `SystemTelemetry`, `PidConfigStatus`
- Subscribes: `PhysicsTick`, `ImuData`, `JoystickCommand`, `MotorFeedback`, `PidConfigOverride`

### `MotorService`

> Implements the actuator boundary between reflected IPC commands and the low-level motor runner.
>
> The service subscribes to `PhysicsTick` and `MotorTargets` and deliberately contains almost no control state of its own. Its job is to remember the latest physics timestamp, accept wheel-speed targets expressed in steps per second, and forward them to the configured `MotorRunner` if one is attached:
>
> $$ u_L, u_R \; [\mathrm{steps/s}] \rightarrow \texttt{MotorRunner::setTargets}(u_L, u_R) $$
>
> Keeping this service narrow is intentional. Closed-loop balance, velocity estimation, and telemetry all remain in `ControlService` and `RateControllerCore`, while hardware-specific pulse generation, slew limiting, and direction control remain below this layer in the motor runner. The service also listens for `PhysicsTick` so it can keep the runner aligned with the current physics time before forwarding motor targets. When hardware is present the service also republishes the runner's continuous post-slew command, pulse-frame-applied rate, slew-limit flags, steps-derived diagnostic speed estimate, and integrated step state as `MotorFeedback`; actuator saturation bit 0 is left slew limiting and bit 1 is right slew limiting. This feedback lets `ControlService` observe the real actuator stages instead of assuming the last commanded target was achieved. In SIL or unit-test configurations the runner pointer may be null, allowing the bus and controller stack to execute without requiring a physical motor backend.

- Publishes: `MotorFeedback`
- Subscribes: `PhysicsTick`, `MotorTargets`

### `InputService`

> Arbitrates hardware and external joystick input, then publishes resolved normalized `JoystickCommand` messages to the bus.
>
> This service isolates the platform-dependent gamepad reading (SDL2) from the main application logic. An available Xbox controller has priority; otherwise a validated external command is held until its short watchdog expires or an explicit neutral command is received.

- Publishes: `JoystickCommand`
- Subscribes: `ExternalJoystickCommand`

### `UdpBridge`

> Stateful transport bridge that connects the internal `MessageBus` to external UDP runtime clients. In the production Pi runtime it listens on port `9000`; the telemetry server is the primary peer, while SIL and other authorized clients use the same boundary.
>
> A client registers by sending a UDP datagram. The bridge remembers the most recent sender as the single active peer, so outbound messages always have one explicit return path. On ingress, the bridge extracts the leading `uint16_t` message identifier and republishes the remaining payload only when the ID is authorized by `Publishes` and the payload size is valid.
>
> On egress, each authorized subscribed message is encoded as the reflected message ID followed immediately by the trivially-copyable payload bytes and placed on a bounded transport queue. High-rate telemetry uses latest-value coalescing and a dedicated nonblocking TX worker, so a slow UDP peer cannot extend the control dispatch path:
>
> $$ \text{datagram} = \texttt{uint16\_t MsgId} \; || \; \texttt{Payload bytes} $$
>
> This makes the UDP contract symmetric with the generated Python bindings and keeps the external runtime API aligned with the reflected C++ message definitions. The dashboard receives and logs `SystemTelemetry`; deployment and process control remain separate SSH operations. The simulator scenario service uses a separate UDP endpoint on port `9001` and is not this bridge.

- Publishes: `PhysicsTick`, `ExternalJoystickCommand`, `ImuRawData`, `SimStartRun`, `SimStopRun`, `PidConfigOverride`
- Subscribes: `MotorTargets`, `SystemTelemetry`, `SimulatorTelemetry`, `SimStartAck`, `SimRunDone`, `PidConfigStatus`

---

## Message Payloads

Each section corresponds to one reflected balancer message. Some are exposed over UDP, while others are
internal-only service messages. Wire sizes come directly from `sizeof(Payload)`. Nested struct types are
listed as `Sub-struct` sections under the first message that references them; types that are also
documented messages reuse their message section.

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
- Published by: `InputService`
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
- Wire size: `392` bytes
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
| `left_target_sps` | `float` | `float` | 4 | 92 |  |
| `right_target_sps` | `float` | `float` | 4 | 96 |  |
| `left_slewed_sps` | `float` | `float` | 4 | 100 |  |
| `right_slewed_sps` | `float` | `float` | 4 | 104 |  |
| `motor_update_dt_ms` | `float` | `float` | 4 | 108 |  |
| `motor_feedback_age_ms` | `float` | `float` | 4 | 112 |  |
| `left_actual_steps` | `int32_t` | `int` | 4 | 116 |  |
| `right_actual_steps` | `int32_t` | `int` | 4 | 120 |  |
| `actuator_saturation_flags` | `uint32_t` | `int` | 4 | 124 |  |
| `command_saturated` | `bool` | `bool` | 1 | 128 |  |
| `actuator_fault` | `bool` | `bool` | 1 | 129 |  |
| `trim_learning_enabled` | `uint8_t` | `int` | 1 | 130 |  |
| `trim_learning_block_reason` | `uint8_t` | `int` | 1 | 131 |  |
| `trim_learning_reserved` | `uint16_t` | `int` | 2 | 132 |  |
| `pitch_feedback_sps` | `float` | `float` | 4 | 136 |  |
| `pitch_rate_feedback_sps` | `float` | `float` | 4 | 140 |  |
| `pitch_accel_feedback_sps` | `float` | `float` | 4 | 144 |  |
| `velocity_pitch_target_deg` | `float` | `float` | 4 | 148 |  |
| `balance_unclamped_sps` | `float` | `float` | 4 | 152 |  |
| `active_pitch_gain_sps_per_rad` | `float` | `float` | 4 | 156 |  |
| `active_pitch_rate_gain_sps_per_rad_s` | `float` | `float` | 4 | 160 |  |
| `active_pitch_accel_gain_sps_per_rad_s2` | `float` | `float` | 4 | 164 |  |
| `active_velocity_pitch_gain_rad_per_sps` | `float` | `float` | 4 | 168 |  |
| `active_velocity_control_cutoff_hz` | `float` | `float` | 4 | 172 |  |
| `active_velocity_observer_cutoff_hz` | `float` | `float` | 4 | 176 |  |
| `active_com_trim_gain_deg_per_sps_s` | `float` | `float` | 4 | 180 |  |
| `active_com_trim_limit_deg` | `float` | `float` | 4 | 184 |  |
| `active_accel_lpf_hz` | `float` | `float` | 4 | 188 |  |
| `active_gyro_derivative_lpf_hz` | `float` | `float` | 4 | 192 |  |
| `active_config_generation` | `uint64_t` | `int` | 8 | 200 |  |
| `velocity_pitch_request_unclamped_deg` | `float` | `float` | 4 | 208 |  |
| `velocity_pitch_request_limited_deg` | `float` | `float` | 4 | 212 |  |
| `pitch_target_unclamped_deg` | `float` | `float` | 4 | 216 |  |
| `active_velocity_pitch_limit_deg` | `float` | `float` | 4 | 220 |  |
| `trim_quiet_rate_rms_dps` | `float` | `float` | 4 | 224 |  |
| `velocity_authority_limited` | `bool` | `bool` | 1 | 228 |  |
| `trim_trusted` | `bool` | `bool` | 1 | 229 |  |
| `trim_learning_allowed` | `bool` | `bool` | 1 | 230 |  |
| `pitch_target_limit_reason` | `uint8_t` | `int` | 1 | 231 |  |
| `velocity_control_sps` | `float` | `float` | 4 | 232 |  |
| `completed_step_acceleration_sps2` | `float` | `float` | 4 | 236 |  |
| `packet_seq` | `uint64_t` | `int` | 8 | 240 |  |
| `loop_seq` | `uint64_t` | `int` | 8 | 248 |  |
| `sender_monotonic_ns` | `uint64_t` | `int` | 8 | 256 |  |
| `user_velocity_mps` | `float` | `float` | 4 | 264 |  |
| `reference_velocity_mps` | `float` | `float` | 4 | 268 |  |
| `reference_acceleration_mps2` | `float` | `float` | 4 | 272 |  |
| `velocity_feedback_estimate_mps` | `float` | `float` | 4 | 276 |  |
| `velocity_error_mps` | `float` | `float` | 4 | 280 |  |
| `velocity_feedback_acceleration_mps2` | `float` | `float` | 4 | 284 |  |
| `acceleration_raw_mps2` | `float` | `float` | 4 | 288 |  |
| `acceleration_cmd_mps2` | `float` | `float` | 4 | 292 |  |
| `drive_pitch_target_deg` | `float` | `float` | 4 | 296 |  |
| `fixed_com_trim_deg` | `float` | `float` | 4 | 300 |  |
| `velocity_feedback_valid` | `bool` | `bool` | 1 | 304 |  |
| `velocity_feedback_active` | `bool` | `bool` | 1 | 305 |  |
| `outer_acceleration_limited` | `bool` | `bool` | 1 | 306 |  |
| `outer_pitch_target_limited` | `bool` | `bool` | 1 | 307 |  |
| `active_drive_max_velocity_mps` | `float` | `float` | 4 | 308 |  |
| `active_drive_max_acceleration_mps2` | `float` | `float` | 4 | 312 |  |
| `active_drive_max_deceleration_mps2` | `float` | `float` | 4 | 316 |  |
| `active_velocity_gain_per_s` | `float` | `float` | 4 | 320 |  |
| `active_velocity_feedback_cutoff_hz` | `float` | `float` | 4 | 324 |  |
| `active_outer_pitch_limit_deg` | `float` | `float` | 4 | 328 |  |
| `active_fixed_com_trim_deg` | `float` | `float` | 4 | 332 |  |
| `adaptive_com_trim_enabled` | `bool` | `bool` | 1 | 336 |  |
| `legacy_outer_fields_valid` | `bool` | `bool` | 1 | 337 |  |
| `reference_jerk_mps3` | `float` | `float` | 4 | 340 |  |
| `velocity_p_acceleration_mps2` | `float` | `float` | 4 | 344 |  |
| `velocity_i_acceleration_mps2` | `float` | `float` | 4 | 348 |  |
| `velocity_integral_state_mps_s` | `float` | `float` | 4 | 352 |  |
| `final_pitch_target_deg` | `float` | `float` | 4 | 356 |  |
| `active_planner_max_acceleration_mps2` | `float` | `float` | 4 | 360 |  |
| `active_planner_max_deceleration_mps2` | `float` | `float` | 4 | 364 |  |
| `active_planner_max_jerk_mps3` | `float` | `float` | 4 | 368 |  |
| `active_velocity_i_gain_per_s2` | `float` | `float` | 4 | 372 |  |
| `active_velocity_i_leak_time_s` | `float` | `float` | 4 | 376 |  |
| `active_velocity_i_acceleration_limit_mps2` | `float` | `float` | 4 | 380 |  |
| `planner_acceleration_limited` | `bool` | `bool` | 1 | 384 |  |
| `planner_jerk_limited` | `bool` | `bool` | 1 | 385 |  |
| `velocity_integral_limited` | `bool` | `bool` | 1 | 386 |  |
| `velocity_anti_windup_active` | `bool` | `bool` | 1 | 387 |  |

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
- Wire size: `1288` bytes
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
| `initial_velocity_mps` | `double` | `float` | 8 | 1120 |  |
| `velocity_estimator_bias_mps` | `double` | `float` | 8 | 1128 |  |
| `velocity_estimator_bias_drift_mps_per_s` | `double` | `float` | 8 | 1136 |  |
| `velocity_estimator_scale` | `double` | `float` | 8 | 1144 |  |
| `velocity_estimator_latency_s` | `double` | `float` | 8 | 1152 |  |
| `initial_pitch_rate_dps` | `double` | `float` | 8 | 1160 |  |
| `brace_enabled` | `uint8_t` | `int` | 1 | 1168 |  |
| `brace_reserved0` | `uint8_t` | `int` | 1 | 1169 |  |
| `brace_reserved1` | `uint16_t` | `int` | 2 | 1170 |  |
| `brace_pitch_deg` | `double` | `float` | 8 | 1176 |  |
| `brace_stiffness_nm_per_rad` | `double` | `float` | 8 | 1184 |  |
| `brace_damping_nm_s_per_rad` | `double` | `float` | 8 | 1192 |  |
| `brace_rest_events` | `std::array<SimBraceRestEventPayload, 4>` | `list[SimBraceRestEventPayload]` | 64 | 1200 |  |
| `first_mass_moment_scale` | `double` | `float` | 8 | 1264 |  |
| `stepper_current_limit_a` | `double` | `float` | 8 | 1272 |  |
| `stepper_bus_voltage_v` | `double` | `float` | 8 | 1280 |  |

#### Sub-struct: `SimDisturbancePayload`

> One scheduled simulator plant disturbance segment. Step disturbances apply a constant external horizontal force and COM bias for the active window. Ramp disturbances interpolate from the start values to the end values across the window. Hold-bias disturbances apply a constant external force and COM bias from start_s until duration_s expires, or to the end of the run when duration_s is non-positive.

- C++ type: `SimDisturbancePayload`
- Python type: `SimDisturbancePayload`
- Wire size: `56` bytes

| Field | C++ Type | Python Type | Bytes | Offset | Description |
|---|---|---|---:|---:|---|
| `kind` | `uint8_t` | `int` | 1 | 0 |  |
| `reserved0` | `uint8_t` | `int` | 1 | 1 |  |
| `reserved1` | `uint16_t` | `int` | 2 | 2 |  |
| `start_s` | `double` | `float` | 8 | 8 |  |
| `duration_s` | `double` | `float` | 8 | 16 |  |
| `force_n` | `double` | `float` | 8 | 24 |  |
| `com_bias_rad` | `double` | `float` | 8 | 32 |  |
| `force_n_end` | `double` | `float` | 8 | 40 |  |
| `com_bias_rad_end` | `double` | `float` | 8 | 48 |  |

#### Sub-struct: `SimJoySegmentPayload`

> One deterministic joystick segment scheduled on the simulator timeline.

- C++ type: `SimJoySegmentPayload`
- Python type: `SimJoySegmentPayload`
- Wire size: `48` bytes

| Field | C++ Type | Python Type | Bytes | Offset | Description |
|---|---|---|---:|---:|---|
| `start_s` | `double` | `float` | 8 | 0 |  |
| `duration_s` | `double` | `float` | 8 | 8 |  |
| `forward` | `double` | `float` | 8 | 16 |  |
| `turn` | `double` | `float` | 8 | 24 |  |
| `forward_end` | `double` | `float` | 8 | 32 |  |
| `turn_end` | `double` | `float` | 8 | 40 |  |

#### Sub-struct: `SimBraceRestEventPayload`

> One simulator-only event that places the plant onto a configured physical brace.

- C++ type: `SimBraceRestEventPayload`
- Python type: `SimBraceRestEventPayload`
- Wire size: `16` bytes

| Field | C++ Type | Python Type | Bytes | Offset | Description |
|---|---|---|---:|---:|---|
| `start_s` | `double` | `float` | 8 | 0 |  |
| `pitch_deg` | `double` | `float` | 8 | 8 |  |

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
- Wire size: `96` bytes
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
- Wire size: `592` bytes
- Published by: _None_
- Consumed by: `UdpBridge`

| Field | C++ Type | Python Type | Bytes | Offset | Description |
|---|---|---|---:|---:|---|
| `system` | `SystemTelemetryPayload` | `SystemTelemetryPayload` | 392 | 0 |  |
| `seed` | `uint32_t` | `int` | 4 | 392 |  |
| `plant_pitch_deg` | `float` | `float` | 4 | 396 |  |
| `plant_pitch_rate_dps` | `float` | `float` | 4 | 400 |  |
| `plant_position_m` | `float` | `float` | 4 | 404 |  |
| `plant_velocity_mps` | `float` | `float` | 4 | 408 |  |
| `target_wheel_velocity` | `float` | `float` | 4 | 412 |  |
| `actual_wheel_velocity` | `float` | `float` | 4 | 416 |  |
| `plant_velocity_error` | `float` | `float` | 4 | 420 |  |
| `f_cmd` | `float` | `float` | 4 | 424 |  |
| `f_app` | `float` | `float` | 4 | 428 |  |
| `external_force_n` | `float` | `float` | 4 | 432 |  |
| `external_com_bias_rad` | `float` | `float` | 4 | 436 |  |
| `x_ddot` | `float` | `float` | 4 | 440 |  |
| `theta_ddot` | `float` | `float` | 4 | 444 |  |
| `phase_error_steps` | `float` | `float` | 4 | 448 |  |
| `missed_steps` | `float` | `float` | 4 | 452 |  |
| `traction_limit_n` | `float` | `float` | 4 | 456 |  |
| `motor_force_limit_n` | `float` | `float` | 4 | 460 |  |
| `total_mass_scale` | `float` | `float` | 4 | 464 |  |
| `pitch_inertia_scale` | `float` | `float` | 4 | 468 |  |
| `motor_max_force_n` | `float` | `float` | 4 | 472 |  |
| `motor_no_load_speed_mps` | `float` | `float` | 4 | 476 |  |
| `motor_velocity_damping` | `float` | `float` | 4 | 480 |  |
| `motor_tau_s` | `float` | `float` | 4 | 484 |  |
| `traction_coefficient` | `float` | `float` | 4 | 488 |  |
| `pitch_damping` | `float` | `float` | 4 | 492 |  |
| `cart_damping` | `float` | `float` | 4 | 496 |  |
| `phase_error_limit_steps` | `float` | `float` | 4 | 500 |  |
| `tire_stiffness_n_per_m` | `float` | `float` | 4 | 504 |  |
| `tire_damping_n_s_per_m` | `float` | `float` | 4 | 508 |  |
| `wheel_equivalent_mass_kg` | `float` | `float` | 4 | 512 |  |
| `force_saturated` | `bool` | `bool` | 1 | 516 |  |
| `emitted_step_velocity_sps` | `float` | `float` | 4 | 520 |  |
| `synthetic_estimator_velocity_sps` | `float` | `float` | 4 | 524 |  |
| `controller_feedback_velocity_sps` | `float` | `float` | 4 | 528 |  |
| `brace_enabled` | `bool` | `bool` | 1 | 532 |  |
| `brace_contact_active` | `bool` | `bool` | 1 | 533 |  |
| `brace_reserved` | `uint16_t` | `int` | 2 | 534 |  |
| `brace_pitch_deg` | `float` | `float` | 4 | 536 |  |
| `brace_penetration_deg` | `float` | `float` | 4 | 540 |  |
| `brace_torque_nm` | `float` | `float` | 4 | 544 |  |
| `recovery_command_active` | `bool` | `bool` | 1 | 548 |  |
| `fallover_inhibited` | `bool` | `bool` | 1 | 549 |  |
| `recovery_reserved` | `uint16_t` | `int` | 2 | 550 |  |
| `first_mass_moment_scale` | `float` | `float` | 4 | 552 |  |
| `stepper_current_limit_a` | `float` | `float` | 4 | 556 |  |
| `stepper_bus_voltage_v` | `float` | `float` | 4 | 560 |  |
| `stepper_traction_utilization` | `float` | `float` | 4 | 564 |  |
| `stepper_accumulated_slip_distance_m` | `float` | `float` | 4 | 568 |  |
| `stepper_summed_torque_nm` | `float` | `float` | 4 | 572 |  |
| `stepper_accumulated_cycle_slips_left` | `uint32_t` | `int` | 4 | 576 |  |
| `stepper_accumulated_cycle_slips_right` | `uint32_t` | `int` | 4 | 580 |  |
| `stepper_voltage_saturated_left` | `bool` | `bool` | 1 | 584 |  |
| `stepper_voltage_saturated_right` | `bool` | `bool` | 1 | 585 |  |
| `stepper_recovery_reserved` | `uint16_t` | `int` | 2 | 586 |  |

### `MsgId::ExternalJoystickCommand`

- Numeric ID: `3011`
- Payload type: `JoystickCommandPayload`
- Python type: `JoystickCommandPayload`
- Wire size: `16` bytes
- Published by: `UdpBridge`
- Consumed by: `InputService`

| Field | C++ Type | Python Type | Bytes | Offset | Description |
|---|---|---|---:|---:|---|
| `forward` | `double` | `float` | 8 | 0 |  |
| `turn` | `double` | `float` | 8 | 8 |  |

### `MsgId::PidConfigOverride`

- Numeric ID: `3012`
- Payload type: `PidConfigOverridePayload`
- Python type: `PidConfigOverridePayload`
- Wire size: `160` bytes
- Published by: `UdpBridge`
- Consumed by: `ControlService`

| Field | C++ Type | Python Type | Bytes | Offset | Description |
|---|---|---|---:|---:|---|
| `request_id` | `uint32_t` | `int` | 4 | 0 |  |
| `reserved` | `uint32_t` | `int` | 4 | 4 |  |
| `values` | `ConfigPidValuesPayload` | `ConfigPidValuesPayload` | 152 | 8 |  |

#### Sub-struct: `ConfigPidValuesPayload`

> The shared numeric controller configuration block carried by override and status messages.

- C++ type: `ConfigPidValuesPayload`
- Python type: `ConfigPidValuesPayload`
- Wire size: `152` bytes

| Field | C++ Type | Python Type | Bytes | Offset | Description |
|---|---|---|---:|---:|---|
| `drive_max_velocity_mps` | `double` | `float` | 8 | 0 |  |
| `velocity_gain_per_s` | `double` | `float` | 8 | 8 |  |
| `velocity_feedback_cutoff_hz` | `double` | `float` | 8 | 16 |  |
| `outer_pitch_limit_deg` | `double` | `float` | 8 | 24 |  |
| `fixed_com_trim_deg` | `double` | `float` | 8 | 32 |  |
| `adaptive_com_trim_enabled` | `double` | `float` | 8 | 40 |  |
| `adaptive_com_trim_gain_deg_per_mps_s` | `double` | `float` | 8 | 48 |  |
| `adaptive_com_trim_limit_deg` | `double` | `float` | 8 | 56 |  |
| `turn_max_sps` | `double` | `float` | 8 | 64 |  |
| `balance_max_sps` | `double` | `float` | 8 | 72 |  |
| `pitch_gain` | `double` | `float` | 8 | 80 |  |
| `pitch_rate_gain` | `double` | `float` | 8 | 88 |  |
| `pitch_accel_gain` | `double` | `float` | 8 | 96 |  |
| `planner_max_acceleration_mps2` | `double` | `float` | 8 | 104 |  |
| `planner_max_deceleration_mps2` | `double` | `float` | 8 | 112 |  |
| `planner_max_jerk_mps3` | `double` | `float` | 8 | 120 |  |
| `velocity_i_gain_per_s2` | `double` | `float` | 8 | 128 |  |
| `velocity_i_leak_time_s` | `double` | `float` | 8 | 136 |  |
| `velocity_i_acceleration_limit_mps2` | `double` | `float` | 8 | 144 |  |

### `MsgId::PidConfigStatus`

- Numeric ID: `3013`
- Payload type: `PidConfigStatusPayload`
- Python type: `PidConfigStatusPayload`
- Wire size: `160` bytes
- Published by: `ControlService`
- Consumed by: `UdpBridge`

| Field | C++ Type | Python Type | Bytes | Offset | Description |
|---|---|---|---:|---:|---|
| `request_id` | `uint32_t` | `int` | 4 | 0 |  |
| `accepted` | `uint8_t` | `int` | 1 | 4 |  |
| `result_code` | `uint8_t` | `int` | 1 | 5 |  |
| `reserved` | `uint16_t` | `int` | 2 | 6 |  |
| `values` | `ConfigPidValuesPayload` | `ConfigPidValuesPayload` | 152 | 8 |  |

---

## Regenerating This File

```bash
cmake -S . -B build
cmake --build build --target balancer_docs
```

_Generated with standard C++26 reflection using GCC 16.1.0 `-std=c++26 -freflection`._
