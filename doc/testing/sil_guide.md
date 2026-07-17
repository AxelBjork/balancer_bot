# SIL Guide

`sil_app` is the service-level software-in-the-loop runtime. It is useful when you want to exercise the UDP-facing message bus path without involving the full direct simulator CLI.

## What `sil_app` Includes

- `ControlService`
- `MotorService`
- `ImuService` with hardware reading disabled
- `UdpBridge`

It does **not** start `TimeService`. Python is expected to inject `PhysicsTick` explicitly.

## UDP Contract

Python can inject:

- `PhysicsTick`
- `ImuRawData`
- `JoystickCommand`

Python can observe:

- `MotorTargets`
- `SystemTelemetry`

The payload schema and message IDs come from the generated bindings in `tests/python/generated_balancer.py`.

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
  process management and `pytest --build`
- `tests/python/udp_client.py`
  UDP helper for the `[uint16_t MsgId][payload]` wire format
- `tests/python/test_udp_bridge.py`
  transport smoke coverage
- `tests/python/test_sil_loop.py`
  tick-driven controller smoke coverage

## Important Boundaries

- `sil_app` is a smoke/integration path, not the main stability benchmark
- the primary balance/stability gate is the in-process twenty-scenario transfer matrix
- focused UDP coverage verifies exact all-tick equivalence with the direct engine and terminal
  summary independence from telemetry stride
- when no hardware motor feedback exists, `ControlService` falls back to commanded-speed proxy feedback

For the broader testing model, read [Testing Strategy](strategy.md). For payloads and wire layout, read [IPC Protocol](../ipc/protocol.md).
