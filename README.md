# Self-Balancing Robot (Pi-Based)

`balancer_bot` is a Raspberry Pi self-balancing robot project built around a tick-driven control loop, a small internal message bus, deterministic simulator tooling, and a Python SIL harness.

## Quick Start

### Host Build and Tests
```bash
./build_cmake
pytest -q
```

### Standalone Simulator
```bash
./build/balancer_simulator
```

### Raspberry Pi Cross-Build
```bash
./build_cmake OFF
```

## Documentation

- [Documentation Portal](doc/index.md)
- [Project Overview](doc/overview.md)
- [Runtime Architecture](doc/arch/runtime.md)
- [Testing Strategy](doc/testing/strategy.md)
- [Running on Pi](doc/Running_on_Pi.md)
- [Current Status](doc/status.md)
- [IPC Protocol Reference](doc/ipc/protocol.md)
- [Hardware Reference](hardware/README.md)

## Repo Layout

- `src/` contains the runtime, services, reflection generators, and platform helpers.
- `tests/` contains C++ tests, pytest SIL tests, simulator support, and captured reference data.
- `hardware/` contains the frame assets, BOM notes, and wiring reference.
- `doc/` contains the human-written handbook plus generated IPC documentation.
- `pid.conf` and `pid_sim.conf` are the default hardware and simulator PID profiles.
