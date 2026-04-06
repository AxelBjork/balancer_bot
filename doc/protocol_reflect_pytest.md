# IPC Protocol Reference

[Home](../../README.md)

> **Auto-generated** by `generate_docs` using C++26 static reflection (P2996 + P3394).
> Do not edit by hand — re-run `cmake --build build --target generate_docs` to refresh.

## System Architecture

The complete system architecture, wire format, and message flow are detailed below.

![IPC Flow Diagram](ipc_flow.svg)

### How to Read the Diagram

The architecture is divided into three logical vertical layers:

1.  **Test Harness (Left)**: The `pytest` environment. Test cases use the `UdpClient` to orchestrate scenarios, sending commands and receiving telemetry via the auto-generated Python bindings.
2.  **Network Layer (Middle)**: The UDP transport bridging the two processes. It shows the explicit socket mapping (Port 9000 for Inbound SIL traffic, Port 9001 for Outbound) handled by the host OS.
3.  **Simulator (Right)**: The C++ `sil_app`. The `UdpBridge` acts as a gateway, translating UDP packets into internal `MessageBus` events which are then routed to decoupled services.

**Legend**:
- **Solid White Lines**: External UDP socket traffic between the harness and the bridge.
- **Dotted Slate Lines**: Internal C++ `MessageBus` Publish/Subscribe routing.
- **Colored Nodes**: Services and components grouped by their logical domain.

**Wire format:** Every datagram starts with a `uint16_t` message ID (host-byte
order) followed immediately by the fixed-size payload struct (packed, no padding).
If `sizeof(received payload) != sizeof(Payload)` the message is silently discarded.

**Threads (C++ side):**

| Thread | Purpose | Location |
|---|---|---|
| `main` | Entry point; waits on shutdown signal (futex wait) | `main.cpp` |
| `heartbeat` | Publishes standard "Heartbeat" logs every 500ms | `main.cpp` |
| `bus-listener` | Listens to AF_UNIX socket for message routing | `MessageBus` |
| `bridge-rx` | Receives and injects UDP packets into the local bus | `UdpBridge` |
| `sim-clock` | **100Hz Heartbeat**: Drives physics ticks for all services | `StateService` |
| `log-worker` | Async processing: prints logs and publishes to bus | `LogService` |

---
---

## Component Services

The application is composed of the following services:

### `MotorService`

> Reactive service that executes motor command sequences.

This service is responsible for stepping through a sequence of motor commands in response to `PhysicsTick` events, and reporting its status via `MotorStatus` messages.

### `KinematicsService`

> Simulates vehicle motion by integrating motor RPM over time to track position and linear velocity.

The physics model applies a linear conversion from RPM to meters-per-second, and integrates this velocity over the `PhysicsTick` delta-time to continuously evaluate the vehicle's position:

$$ v = \text{RPM} \times 0.01 \text{ (m/s)} $$

$$ x = \int v \, dt $$

### `PowerService`

> Models a simple battery pack dynamically responding to motor load.

The simulation estimates motor current draw from speed using a non-linear power-law curve, then applies Ohm's law over the internal resistance to calculate the per-tick voltage drop. The state of charge (SOC) is linearly interpolated between the maximum and minimum voltage limits.

$$ I = I_{idle} + k\,|\mathrm{RPM}|^{p} \quad (\mathrm{A}) $$

$$ V \mathrel{-}= I\,R_{int}\,\Delta t $$

$$ SOC = \frac{V - V_{min}}{V_{max} - V_{min}} \times 100 $$

### `StateService`

> Maintains the central lifecycle state machine and the master simulation clock.

This component tracks the system state and generates the 100Hz `PhysicsTick` heartbeat that drives all other simulation services.

### `ThermalService`

> Simulates basic motor and battery temperature dynamics.

On each physics tick, the service updates motor and battery temperatures using a simple first-order heat balance: a speed-proportional heat generation term minus a linear cooling term to ambient. Ambient temperature is updated from EnvironmentData. A ThermalRequest publishes the latest temperatures.

Motor model:
$$ \dot{T}_m = q_m - c_m \,(T_m - T_a) $$
$$ q_m = 0.005\,|\mathrm{RPM}|,\quad c_m = 0.05 $$

Battery model:
$$ \dot{T}_b = q_b - c_b \,(T_b - T_a) $$
$$ q_b = 0.002\,|\mathrm{RPM}|,\quad c_b = 0.02 $$

Discrete update per tick (for each body):
$$ T \leftarrow T + \bigl(\dot{T}\bigr)\,\Delta t $$

Where $T_a$ is ambient temperature from EnvironmentData, and $\Delta t$ is the physics timestep in seconds.

### `EnvironmentService`

> Centralized service for managing environmental simulation data.

It maintains a spatial cache of environmental regions (temperature, incline, friction) and provides efficient, lifetime-tracked access to this data via in-process bus messages. By publishing InternalEnvData messages containing std::shared_ptr, it allows consumers like AutonomousService to access stable map regions without data copying or direct component coupling.

### `AutonomousService`

> High-level autonomous driving service executing a waypoint (node) route.

The service accepts an AutoDriveCommand containing a list of ManeuverNodes (1D x targets). While a route is active, it periodically requests Kinematics and Power data on each physics tick, decides when the current node has been reached, and publishes MotorSequence commands to drive toward the node.

### `SensorService`

> Sensor service that procedurally generates environment data based on requested locations.

Simulates reading from physical terrain maps with a 10us delay.

### `RevisionService`

> Provides runtime verification of the compiled IPC protocol hash.

### `LogService`

> Asynchronous logging sink.

Collects LogPayloads into a queue and processes them on a background thread to avoid blocking simulation services.

### `UdpBridge`

> Stateful bridge that relays IPC messages between the internal MessageBus and external UDP clients.

It remembers the IP address and port of the last connected test harness and bidirectionally routes all subscribed C++ events out through the UDP socket while safely injecting incoming UDP datagrams onto the internal MessageBus.

---

## Message Payloads

Each section corresponds to one `MsgId` enumerator. The **direction badge** shows which side initiates the message.

<details>
<summary><font size="+1"><b>MsgId::Log (LogPayload)</b></font></summary>

> Unidirectional log/trace message. Emitted by any component at any time; Python receives these passively from the bus.

**Direction:** `Outbound`<br>
**Publishes:** `LogService`<br>
**Wire size:** 288 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>text</td>
      <td>char[255]</td>
      <td>bytes</td>
      <td>255</td>
      <td>0</td>
    </tr>
    <tr>
      <td>severity</td>
      <td>Severity</td>
      <td>Severity</td>
      <td>1</td>
      <td>255</td>
    </tr>
    <tr>
      <td>component</td>
      <td>char[32]</td>
      <td>bytes</td>
      <td>32</td>
      <td>256</td>
    </tr>
  </tbody>
</table>

</details>

<details>
<summary><font size="+1"><b>MsgId::PhysicsTick (PhysicsTickPayload)</b></font></summary>

> Internal IPC: Broadcast at 100Hz during sequence execution to drive kinematics and power integration.

**Direction:** `Internal`<br>
**Publishes:** `StateService`<br>
**Subscribes:** `MotorService`, `KinematicsService`, `PowerService`, `ThermalService`, `AutonomousService`<br>
**Wire size:** 12 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>cmd_id</td>
      <td>uint32_t</td>
      <td>int</td>
      <td>4</td>
      <td>0</td>
    </tr>
    <tr>
      <td>speed_rpm</td>
      <td>int16_t</td>
      <td>int</td>
      <td>2</td>
      <td>4</td>
    </tr>
    <tr>
      <td>dt_us</td>
      <td>uint32_t</td>
      <td>int</td>
      <td>4</td>
      <td>8</td>
    </tr>
  </tbody>
</table>

</details>

<details>
<summary><font size="+1"><b>MsgId::StateRequest (StateRequestPayload)</b></font></summary>

> One-byte sentinel. Send to request a StateData snapshot. The payload value is ignored.

**Direction:** `Inbound`<br>
**Subscribes:** `StateService`<br>
**Wire size:** 1 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>reserved</td>
      <td>uint8_t</td>
      <td>int</td>
      <td>1</td>
      <td>0</td>
    </tr>
  </tbody>
</table>

</details>

<details>
<summary><font size="+1"><b>MsgId::StateData (StatePayload)</b></font></summary>

> State machine snapshot. Carries the current coarse lifecycle SystemState.

**Direction:** `Bidirectional`<br>
**Publishes:** `StateService`<br>
**Subscribes:** `SensorService`<br>
**Wire size:** 1 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>state</td>
      <td>SystemState</td>
      <td>SystemState</td>
      <td>1</td>
      <td>0</td>
    </tr>
  </tbody>
</table>

</details>

<details>
<summary><font size="+1"><b>MsgId::MotorSequence (MotorSequencePayloadTemplate<5>)</b></font></summary>

> Deliver a sequence of up to 10 timed motor sub-commands to the simulator. The simulator executes steps[0..num_steps-1] in real time; a new command preempts any currently running sequence.

**Direction:** `Bidirectional`<br>
**Publishes:** `AutonomousService`<br>
**Subscribes:** `MotorService`<br>
**Wire size:** 48 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>cmd_id</td>
      <td>uint32_t</td>
      <td>int</td>
      <td>4</td>
      <td>0</td>
    </tr>
    <tr>
      <td>num_steps</td>
      <td>uint8_t</td>
      <td>int</td>
      <td>1</td>
      <td>4</td>
    </tr>
    <tr>
      <td>steps</td>
      <td>std::array<MotorSubCmd, 5></td>
      <td>Any</td>
      <td>40</td>
      <td>8</td>
    </tr>
  </tbody>
</table>

#### Sub-struct: `std::array<MotorSubCmd, 5>`

**Wire size:** 40 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>_M_elems</td>
      <td>MotorSubCmd[5]</td>
      <td>bytes</td>
      <td>40</td>
      <td>0</td>
    </tr>
  </tbody>
</table>

#### Sub-struct: `MotorSubCmd`

> One timed motor command step, embedded in MotorSequencePayload.

**Wire size:** 8 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>speed_rpm</td>
      <td>int16_t</td>
      <td>int</td>
      <td>2</td>
      <td>0</td>
    </tr>
    <tr>
      <td>duration_us</td>
      <td>uint32_t</td>
      <td>int</td>
      <td>4</td>
      <td>4</td>
    </tr>
  </tbody>
</table>

</details>

<details>
<summary><font size="+1"><b>MsgId::MotorStatus (MotorStatusPayload)</b></font></summary>

> Internal IPC: Periodic RPM and activity update from MotorService.

**Direction:** `Internal`<br>
**Publishes:** `MotorService`<br>
**Subscribes:** `KinematicsService`, `PowerService`, `StateService`<br>
**Wire size:** 8 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>cmd_id</td>
      <td>uint32_t</td>
      <td>int</td>
      <td>4</td>
      <td>0</td>
    </tr>
    <tr>
      <td>speed_rpm</td>
      <td>int16_t</td>
      <td>int</td>
      <td>2</td>
      <td>4</td>
    </tr>
    <tr>
      <td>is_active</td>
      <td>bool</td>
      <td>bool</td>
      <td>1</td>
      <td>6</td>
    </tr>
  </tbody>
</table>

</details>

<details>
<summary><font size="+1"><b>MsgId::KinematicsRequest (KinematicsRequestPayload)</b></font></summary>

> One-byte sentinel. Send to request a KinematicsData snapshot. The payload value is ignored.

**Direction:** `Bidirectional`<br>
**Publishes:** `AutonomousService`<br>
**Subscribes:** `KinematicsService`<br>
**Wire size:** 1 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>reserved</td>
      <td>uint8_t</td>
      <td>int</td>
      <td>1</td>
      <td>0</td>
    </tr>
  </tbody>
</table>

</details>

<details>
<summary><font size="+1"><b>MsgId::KinematicsData (KinematicsPayload)</b></font></summary>

> Kinematics snapshot sent in response to a KinematicsRequest. Reflects physics state integrated since the start of the current sequence.

**Direction:** `Bidirectional`<br>
**Publishes:** `KinematicsService`<br>
**Subscribes:** `AutonomousService`<br>
**Wire size:** 16 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>cmd_id</td>
      <td>uint32_t</td>
      <td>int</td>
      <td>4</td>
      <td>0</td>
    </tr>
    <tr>
      <td>elapsed_us</td>
      <td>uint32_t</td>
      <td>int</td>
      <td>4</td>
      <td>4</td>
    </tr>
    <tr>
      <td>position_m</td>
      <td>float</td>
      <td>float</td>
      <td>4</td>
      <td>8</td>
    </tr>
    <tr>
      <td>speed_mps</td>
      <td>float</td>
      <td>float</td>
      <td>4</td>
      <td>12</td>
    </tr>
  </tbody>
</table>

</details>

<details>
<summary><font size="+1"><b>MsgId::PowerRequest (PowerRequestPayload)</b></font></summary>

> One-byte sentinel. Send to request a PowerData snapshot. The payload value is ignored.

**Direction:** `Bidirectional`<br>
**Publishes:** `AutonomousService`<br>
**Subscribes:** `PowerService`<br>
**Wire size:** 1 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>reserved</td>
      <td>uint8_t</td>
      <td>int</td>
      <td>1</td>
      <td>0</td>
    </tr>
  </tbody>
</table>

</details>

<details>
<summary><font size="+1"><b>MsgId::PowerData (PowerPayload)</b></font></summary>

> Power-model snapshot sent in response to a PowerRequest. Models a simple battery with internal resistance drain.

**Direction:** `Bidirectional`<br>
**Publishes:** `PowerService`<br>
**Subscribes:** `AutonomousService`<br>
**Wire size:** 16 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>cmd_id</td>
      <td>uint32_t</td>
      <td>int</td>
      <td>4</td>
      <td>0</td>
    </tr>
    <tr>
      <td>voltage_v</td>
      <td>float</td>
      <td>float</td>
      <td>4</td>
      <td>4</td>
    </tr>
    <tr>
      <td>current_a</td>
      <td>float</td>
      <td>float</td>
      <td>4</td>
      <td>8</td>
    </tr>
    <tr>
      <td>state_of_charge</td>
      <td>uint8_t</td>
      <td>int</td>
      <td>1</td>
      <td>12</td>
    </tr>
  </tbody>
</table>

</details>

<details>
<summary><font size="+1"><b>MsgId::ThermalRequest (ThermalRequestPayload)</b></font></summary>

> One-byte sentinel. Send to request a ThermalData snapshot. The payload value is ignored.

**Direction:** `Inbound`<br>
**Subscribes:** `ThermalService`<br>
**Wire size:** 1 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>reserved</td>
      <td>uint8_t</td>
      <td>int</td>
      <td>1</td>
      <td>0</td>
    </tr>
  </tbody>
</table>

</details>

<details>
<summary><font size="+1"><b>MsgId::ThermalData (ThermalPayload)</b></font></summary>

> Thermal snapshot sent in response to a ThermalRequest. Models temperature of motor and battery based on power metrics.

**Direction:** `Outbound`<br>
**Publishes:** `ThermalService`<br>
**Wire size:** 8 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>motor_temp_c</td>
      <td>float</td>
      <td>float</td>
      <td>4</td>
      <td>0</td>
    </tr>
    <tr>
      <td>battery_temp_c</td>
      <td>float</td>
      <td>float</td>
      <td>4</td>
      <td>4</td>
    </tr>
  </tbody>
</table>

</details>

<details>
<summary><font size="+1"><b>MsgId::EnvironmentAck (EnvironmentAckPayload)</b></font></summary>

> ACK sent by the application when it accepts new environment data.

**Direction:** `Outbound`<br>
**Publishes:** `EnvironmentService`<br>
**Wire size:** 4 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>region_id</td>
      <td>uint32_t</td>
      <td>int</td>
      <td>4</td>
      <td>0</td>
    </tr>
  </tbody>
</table>

</details>

<details>
<summary><font size="+1"><b>MsgId::EnvironmentRequest (EnvironmentRequestPayload)</b></font></summary>

> Request conditions for a specific location.

**Direction:** `Outbound`<br>
**Publishes:** `EnvironmentService`<br>
**Wire size:** 8 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>target_location</td>
      <td>Point2D</td>
      <td>Any</td>
      <td>8</td>
      <td>0</td>
    </tr>
  </tbody>
</table>

#### Sub-struct: `Point2D`

> A 2D coordinate.

**Wire size:** 8 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>x</td>
      <td>float</td>
      <td>float</td>
      <td>4</td>
      <td>0</td>
    </tr>
    <tr>
      <td>y</td>
      <td>float</td>
      <td>float</td>
      <td>4</td>
      <td>4</td>
    </tr>
  </tbody>
</table>

</details>

<details>
<summary><font size="+1"><b>MsgId::EnvironmentData (EnvironmentPayload)</b></font></summary>

> Environmental conditions delivered to the application from the outside world.

**Direction:** `Bidirectional`<br>
**Publishes:** `SensorService`<br>
**Subscribes:** `ThermalService`, `EnvironmentService`<br>
**Wire size:** 36 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>region_id</td>
      <td>uint32_t</td>
      <td>int</td>
      <td>4</td>
      <td>0</td>
    </tr>
    <tr>
      <td>bounds</td>
      <td>BoundingBox2D</td>
      <td>Any</td>
      <td>16</td>
      <td>4</td>
    </tr>
    <tr>
      <td>ambient_temp_c</td>
      <td>float</td>
      <td>float</td>
      <td>4</td>
      <td>20</td>
    </tr>
    <tr>
      <td>incline_percent</td>
      <td>float</td>
      <td>float</td>
      <td>4</td>
      <td>24</td>
    </tr>
    <tr>
      <td>surface_friction</td>
      <td>float</td>
      <td>float</td>
      <td>4</td>
      <td>28</td>
    </tr>
    <tr>
      <td>max_speed_rpm</td>
      <td>float</td>
      <td>float</td>
      <td>4</td>
      <td>32</td>
    </tr>
  </tbody>
</table>

#### Sub-struct: `BoundingBox2D`

> An axis-aligned 2D bounding box.

**Wire size:** 16 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>min_pt</td>
      <td>Point2D</td>
      <td>Any</td>
      <td>8</td>
      <td>0</td>
    </tr>
    <tr>
      <td>max_pt</td>
      <td>Point2D</td>
      <td>Any</td>
      <td>8</td>
      <td>8</td>
    </tr>
  </tbody>
</table>

#### Sub-struct: `Point2D`

> A 2D coordinate.

**Wire size:** 8 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>x</td>
      <td>float</td>
      <td>float</td>
      <td>4</td>
      <td>0</td>
    </tr>
    <tr>
      <td>y</td>
      <td>float</td>
      <td>float</td>
      <td>4</td>
      <td>4</td>
    </tr>
  </tbody>
</table>

</details>

<details>
<summary><font size="+1"><b>MsgId::AutoDriveCommand (AutoDriveCommandTemplate<8>)</b></font></summary>

> High level autonomous driving route and configuration.

**Direction:** `Inbound`<br>
**Subscribes:** `AutonomousService`<br>
**Wire size:** 180 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>route_name</td>
      <td>char[32]</td>
      <td>bytes</td>
      <td>32</td>
      <td>0</td>
    </tr>
    <tr>
      <td>mode</td>
      <td>DriveMode</td>
      <td>DriveMode</td>
      <td>1</td>
      <td>32</td>
    </tr>
    <tr>
      <td>p_gain</td>
      <td>float</td>
      <td>float</td>
      <td>4</td>
      <td>36</td>
    </tr>
    <tr>
      <td>use_environment_tuning</td>
      <td>bool</td>
      <td>bool</td>
      <td>1</td>
      <td>40</td>
    </tr>
    <tr>
      <td>route_transform</td>
      <td>std::array<Vector3, 3></td>
      <td>Any</td>
      <td>36</td>
      <td>44</td>
    </tr>
    <tr>
      <td>num_nodes</td>
      <td>uint8_t</td>
      <td>int</td>
      <td>1</td>
      <td>80</td>
    </tr>
    <tr>
      <td>route</td>
      <td>std::array<ManeuverNode, 8></td>
      <td>Any</td>
      <td>96</td>
      <td>84</td>
    </tr>
  </tbody>
</table>

#### Sub-struct: `std::array<Vector3, 3>`

**Wire size:** 36 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>_M_elems</td>
      <td>Vector3[3]</td>
      <td>bytes</td>
      <td>36</td>
      <td>0</td>
    </tr>
  </tbody>
</table>

#### Sub-struct: `Vector3`

> A 3D coordinate vector.

**Wire size:** 12 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>x</td>
      <td>float</td>
      <td>float</td>
      <td>4</td>
      <td>0</td>
    </tr>
    <tr>
      <td>y</td>
      <td>float</td>
      <td>float</td>
      <td>4</td>
      <td>4</td>
    </tr>
    <tr>
      <td>z</td>
      <td>float</td>
      <td>float</td>
      <td>4</td>
      <td>8</td>
    </tr>
  </tbody>
</table>

#### Sub-struct: `std::array<ManeuverNode, 8>`

**Wire size:** 96 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>_M_elems</td>
      <td>ManeuverNode[8]</td>
      <td>bytes</td>
      <td>96</td>
      <td>0</td>
    </tr>
  </tbody>
</table>

#### Sub-struct: `ManeuverNode`

> A single target maneuver point.

**Wire size:** 12 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>target_pos</td>
      <td>Point2D</td>
      <td>Any</td>
      <td>8</td>
      <td>0</td>
    </tr>
    <tr>
      <td>timeout_ms</td>
      <td>uint16_t</td>
      <td>int</td>
      <td>2</td>
      <td>8</td>
    </tr>
  </tbody>
</table>

#### Sub-struct: `Point2D`

> A 2D coordinate.

**Wire size:** 8 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>x</td>
      <td>float</td>
      <td>float</td>
      <td>4</td>
      <td>0</td>
    </tr>
    <tr>
      <td>y</td>
      <td>float</td>
      <td>float</td>
      <td>4</td>
      <td>4</td>
    </tr>
  </tbody>
</table>

</details>

<details>
<summary><font size="+1"><b>MsgId::AutoDriveStatus (AutoDriveStatusTemplate<8, 4>)</b></font></summary>

> Status and efficiency report from the AutonomousService.

**Direction:** `Outbound`<br>
**Publishes:** `AutonomousService`<br>
**Wire size:** 156 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>cmd_id</td>
      <td>uint32_t</td>
      <td>int</td>
      <td>4</td>
      <td>0</td>
    </tr>
    <tr>
      <td>current_node_idx</td>
      <td>uint8_t</td>
      <td>int</td>
      <td>1</td>
      <td>4</td>
    </tr>
    <tr>
      <td>route_complete</td>
      <td>bool</td>
      <td>bool</td>
      <td>1</td>
      <td>5</td>
    </tr>
    <tr>
      <td>num_stats</td>
      <td>uint8_t</td>
      <td>int</td>
      <td>1</td>
      <td>6</td>
    </tr>
    <tr>
      <td>node_stats</td>
      <td>std::array<ManeuverStats, 8></td>
      <td>Any</td>
      <td>128</td>
      <td>8</td>
    </tr>
    <tr>
      <td>num_environments_used</td>
      <td>uint8_t</td>
      <td>int</td>
      <td>1</td>
      <td>136</td>
    </tr>
    <tr>
      <td>environment_ids</td>
      <td>std::array<EnvId, 4></td>
      <td>Any</td>
      <td>16</td>
      <td>140</td>
    </tr>
  </tbody>
</table>

#### Sub-struct: `std::array<ManeuverStats, 8>`

**Wire size:** 128 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>_M_elems</td>
      <td>ManeuverStats[8]</td>
      <td>bytes</td>
      <td>128</td>
      <td>0</td>
    </tr>
  </tbody>
</table>

#### Sub-struct: `ManeuverStats`

> Efficiency metrics for a single traveled node.

**Wire size:** 16 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>initial_energy_mj</td>
      <td>float</td>
      <td>float</td>
      <td>4</td>
      <td>0</td>
    </tr>
    <tr>
      <td>final_energy_mj</td>
      <td>float</td>
      <td>float</td>
      <td>4</td>
      <td>4</td>
    </tr>
    <tr>
      <td>energy_per_meter_mj</td>
      <td>float</td>
      <td>float</td>
      <td>4</td>
      <td>8</td>
    </tr>
    <tr>
      <td>total_energy_used_mj</td>
      <td>float</td>
      <td>float</td>
      <td>4</td>
      <td>12</td>
    </tr>
  </tbody>
</table>

#### Sub-struct: `std::array<EnvId, 4>`

**Wire size:** 16 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>_M_elems</td>
      <td>EnvId[4]</td>
      <td>bytes</td>
      <td>16</td>
      <td>0</td>
    </tr>
  </tbody>
</table>

#### Sub-struct: `EnvId`

> Environment ID Wrapper.

**Wire size:** 4 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>id</td>
      <td>uint32_t</td>
      <td>int</td>
      <td>4</td>
      <td>0</td>
    </tr>
  </tbody>
</table>

</details>

<details>
<summary><font size="+1"><b>MsgId::InternalEnvRequest (InternalEnvRequestPayload)</b></font></summary>

**Direction:** `Internal`<br>
**Publishes:** `AutonomousService`<br>
**Subscribes:** `EnvironmentService`<br>
**Wire size:** 8 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>x</td>
      <td>float</td>
      <td>float</td>
      <td>4</td>
      <td>0</td>
    </tr>
    <tr>
      <td>y</td>
      <td>float</td>
      <td>float</td>
      <td>4</td>
      <td>4</td>
    </tr>
  </tbody>
</table>

</details>

<details>
<summary><font size="+1"><b>MsgId::InternalEnvData (InternalEnvDataPayload)</b></font></summary>

**Direction:** `Internal`<br>
**Publishes:** `EnvironmentService`<br>
**Subscribes:** `AutonomousService`<br>
**Wire size:** 16 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>ptr</td>
      <td>std::shared_ptr<const EnvironmentPayload></td>
      <td>Any</td>
      <td>16</td>
      <td>0</td>
    </tr>
  </tbody>
</table>

#### Sub-struct: `std::shared_ptr<const EnvironmentPayload>`

**Wire size:** 16 bytes

_No fields._

</details>

<details>
<summary><font size="+1"><b>MsgId::SensorRequest (SensorRequestPayload)</b></font></summary>

> Explicitly request the SensorService to read terrain from its deterministically generated world map.

**Direction:** `Inbound`<br>
**Subscribes:** `SensorService`<br>
**Wire size:** 8 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>target_location</td>
      <td>Point2D</td>
      <td>Any</td>
      <td>8</td>
      <td>0</td>
    </tr>
  </tbody>
</table>

#### Sub-struct: `Point2D`

> A 2D coordinate.

**Wire size:** 8 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>x</td>
      <td>float</td>
      <td>float</td>
      <td>4</td>
      <td>0</td>
    </tr>
    <tr>
      <td>y</td>
      <td>float</td>
      <td>float</td>
      <td>4</td>
      <td>4</td>
    </tr>
  </tbody>
</table>

</details>

<details>
<summary><font size="+1"><b>MsgId::SensorAck (SensorAckPayload)</b></font></summary>

> ACK sent by SensorService indicating whether a SensorRequest was accepted.

**Direction:** `Outbound`<br>
**Publishes:** `SensorService`<br>
**Wire size:** 8 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>request_id</td>
      <td>uint32_t</td>
      <td>int</td>
      <td>4</td>
      <td>0</td>
    </tr>
    <tr>
      <td>success</td>
      <td>bool</td>
      <td>bool</td>
      <td>1</td>
      <td>4</td>
    </tr>
    <tr>
      <td>reason</td>
      <td>uint8_t</td>
      <td>int</td>
      <td>1</td>
      <td>5</td>
    </tr>
  </tbody>
</table>

</details>

<details>
<summary><font size="+1"><b>MsgId::RevisionRequest (RevisionRequestPayload)</b></font></summary>

> Request the current system revision and protocol hash.

**Direction:** `Inbound`<br>
**Subscribes:** `RevisionService`<br>
**Wire size:** 1 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>reserved</td>
      <td>uint8_t</td>
      <td>int</td>
      <td>1</td>
      <td>0</td>
    </tr>
  </tbody>
</table>

</details>

<details>
<summary><font size="+1"><b>MsgId::RevisionResponse (RevisionResponsePayload)</b></font></summary>

> Response containing the system revision and protocol hash.

**Direction:** `Outbound`<br>
**Publishes:** `RevisionService`<br>
**Wire size:** 65 bytes

<table>
  <thead>
    <tr><th>Field</th><th>C++ Type</th><th>Py Type</th><th>Bytes</th><th>Offset</th></tr>
  </thead>
  <tbody>
    <tr>
      <td>protocol_hash</td>
      <td>char[65]</td>
      <td>bytes</td>
      <td>65</td>
      <td>0</td>
    </tr>
  </tbody>
</table>

</details>

---

## Regenerating This File

```bash
# From the repo root:
cmake -B build -G Ninja
cmake --build build --target generate_docs
# Output: doc/ipc/protocol.md
```

_Generated with GCC trunk `-std=c++26 -freflection` (P2996R13 + P3394R4)._
