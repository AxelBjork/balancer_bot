# SIL Guide

`sil_app` is the C++ service-level software-in-the-loop runtime. A separate Python SIL client
drives it through the same production UDP boundary used by the telemetry server, without involving
the full direct simulator CLI.

## What `sil_app` Includes

- `ControlService`
- `MotorService`
- `ImuService` with hardware reading disabled
- `InputService` for resolved joystick input
- `UdpBridge`

It does **not** start `TimeService`. Python is expected to inject `PhysicsTick` explicitly.

## UDP Contract

The Python SIL client must register as the active peer on UDP port `9000`. Only one peer receives
outbound traffic; a dashboard or another UDP observer must be stopped before the Python SIL client
starts.

Python can inject the messages authorized by the bridge, including:

- `PhysicsTick`
- `ImuRawData`
- `ExternalJoystickCommand`
- `PidConfigOverride`

Python can observe the bridge’s subscribed runtime messages, including:

- `MotorTargets`
- `SystemTelemetry`

The payload schema and message IDs come from the generated bindings in `tests/python/generated_balancer.py`.
The exact wire layout and complete bridge message inventory are maintained in the [generated IPC
protocol reference](../ipc/protocol.md).

This is a runtime/client integration test. It must not be treated as the main balancing stability
benchmark, and it must not be confused with `balancer_simulator`’s separate port-9001 scenario
endpoint.

## Manual Flow

### Build

```bash
cmake -S . -B build -DBUILD_TESTS=ON
cmake --build build
```

### Run `sil_app`

```bash
./build/sil_app
```

### Run the Python SIL Tests

```bash
pytest -q tests/python/test_udp_bridge.py
pytest -q tests/python/test_sil_loop.py
```

Or run the full combined workflow:

```bash
pytest --build
```

## Key Files

- `tests/python/conftest.py`
  process management, binary discovery, and Python test fixtures
- `conftest.py`
  repository-level `pytest --build` options and CMake/CTest workflow
- `tests/python/udp_client.py`
  UDP helper for the `[uint16_t MsgId][payload]` wire format
- `tests/python/test_udp_bridge.py`
  transport smoke coverage
- `tests/python/test_sil_loop.py`
  tick-driven controller smoke coverage

## Important Boundaries

- `sil_app` plus the Python SIL client form a smoke/integration path for the production UDP boundary, not the main stability benchmark
- the primary balance/stability gate is the in-process seven-scenario transfer matrix
- focused UDP coverage verifies exact all-tick equivalence for transfer catalog index `1` between
  the direct engine and the simulator-wrapper UDP endpoint on port `9001`, plus terminal summary
  independence from telemetry stride
- when no hardware motor feedback exists, `ControlService` falls back to commanded-speed proxy feedback

For the broader testing model, read [Testing Strategy](strategy.md). For payloads and wire layout, read [IPC Protocol](../ipc/protocol.md).
