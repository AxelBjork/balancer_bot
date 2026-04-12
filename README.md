# balancer_bot

`balancer_bot` is a Raspberry Pi self-balancing robot project with a strong software spine: a tick-driven controller, a message-bus runtime, a deterministic direct simulator, and a Python SIL harness built from reflected C++ message definitions.

It is both a real robot project and a control/runtime playground. The same codebase supports:

- a hardware runtime on Raspberry Pi
- a UDP-driven SIL runtime for Python tests
- a fast direct simulator for stability work
- generated IPC docs and Python bindings from the C++ message definitions

## Why This Project Is Interesting

- **Tick-driven control**
  The balancing loop runs from an explicit `PhysicsTick`, which makes the control path easier to reason about in both hardware and simulation.
- **Service-oriented runtime**
  `TimeService`, `ImuService`, `ControlService`, `MotorService`, and `UdpBridge` are wired through a small internal message bus instead of ad hoc callback chains.
- **Deterministic simulation**
  The direct simulator can run representative scenarios, emit artifacts under `build/sim`, and act as the main software stability gate.
- **Generated interfaces**
  IPC docs and Python bindings are generated from the reflected message definitions and service metadata, which keeps the UDP contract and documentation aligned with the code.

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

### Linearized Plant Audit

```bash
./build/balancer_plant_audit --all
```

### Timeline Analysis

```bash
python3 tools/analyze_timeline.py build/sim/realistic_neutral_hold_40s/timeline.csv --summary-json
```

### Raspberry Pi Cross-Build

```bash
./build_cmake OFF
```

## Main Binaries

- `balancer_pi`
  the Raspberry Pi hardware runtime
- `sil_app`
  the UDP-driven software-in-the-loop runtime
- `balancer_simulator`
  the deterministic plant + controller runner
- `balancer_plant_audit`
  prints the linearized upright plant, controllability rank, and candidate overdamped poles
- `tools/analyze_timeline.py`
  estimates lag and scale relationships from a `timeline.csv` artifact or captured telemetry log
- `balancer_tests`
  the C++ unit and integration test binary

## Simplified BOM

| Area | Main Parts |
| --- | --- |
| Compute | Raspberry Pi 4 |
| Sensing | ISM330DHCX IMU breakout + Qwiic SHIM/cable |
| Actuation | 2x NEMA-17 steppers + Waveshare Stepper Motor HAT |
| Power | 3S 18650 battery pack |
| Structure | B-robot EVO 2 frame + local modified print files |

The detailed parts list, frame assets, and wiring notes live in [hardware/README.md](hardware/README.md).

## Architecture At a Glance

![IPC Flow](doc/ipc/ipc_flow.svg)

The generated flow graph above is the best quick picture of the system:

- `TimeService` drives the control timeline with `PhysicsTick`
- `ImuService` publishes the robot state estimate
- `ControlService` converts tick + IMU + input into `MotorTargets` and telemetry
- `MotorService` closes the loop with real motor feedback on hardware
- `UdpBridge` exposes the SIL/test boundary without becoming the core runtime

## Project Layout

- `src/`
  runtime, services, control code, reflection generators, and platform helpers
- `tests/`
  C++ tests, pytest SIL tests, simulator helpers, and captured reference data
- `hardware/`
  physical build assets, frame files, and hardware reference notes
- `doc/`
  handbook pages plus generated IPC docs
- `pid.conf` and `pid_sim.conf`
  default hardware and simulator PID profiles

## Learn More

- [Documentation Portal](doc/index.md)
- [Project Overview](doc/overview.md)
- [Runtime Architecture](doc/arch/runtime.md)
- [Control / Plant Notes](doc/arch/control_plant.md)
- [Testing Strategy](doc/testing/strategy.md)
- [Running on Pi](doc/Running_on_Pi.md)
- [Current Status](doc/status.md)
- [IPC Protocol Reference](doc/ipc/protocol.md)
