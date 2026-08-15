# balancer_bot

`balancer_bot` is a Raspberry Pi two-wheel self-balancing robot and a control-system playground.
It contains the real hardware runtime, a software-in-the-loop (SIL) version of that runtime, a
deterministic simulator, and the tests and telemetry tools used to compare them.

If you are browsing the project for the first time, start with the
[documentation portal](doc/index.md). This page is the short version of the project story.

For a first ten-minute tour, read [the overview](doc/overview.md), check [current confidence](doc/status.md),
then choose [testing](doc/testing/strategy.md), [deployment](doc/Running_on_Pi.md), or [retained evidence](data/README.md)
depending on what you want to explore.

For a clean host setup, use the [host setup and first build guide](doc/host_setup.md) before running
the standard test gate.

## The one-minute picture

The robot balances by running a tick-driven control loop:

```text
clock ──> PhysicsTick ──> ControlService ──> MotorTargets ──> MotorService ──> wheels
                              ^                  ^                 |
                              |                  |                 └─ MotorFeedback
                 ImuService ── ImuData       joystick              |
                              |                                    v
                         SystemTelemetry ──> UdpBridge:9000 ──> dashboard / one client
```

The services communicate through a small synchronous internal message bus. The UDP bridge is the
external boundary: it lets one active client inject selected inputs and receive telemetry and
other selected outputs. The exact message inventory and payload layout are maintained in the
[generated IPC protocol](doc/ipc/protocol.md).

The same control ideas appear in three different environments:

| Mode | Executable | What is real | What it is for |
| --- | --- | --- | --- |
| Hardware | `balancer_pi` | IMU, stepper motors, and optional Xbox input | Running the robot on a Raspberry Pi |
| SIL | `sil_app` | The service/message-bus/UDP path; no hardware reader, plant, or motor backend | Checking the production UDP boundary with Python-injected inputs |
| Direct simulator | `balancer_simulator` | The production estimator/control path plus a deterministic motor and plant model | Stability, disturbance, transfer, and artifact-producing runs |

The hardware bridge uses UDP port `9000`. The simulator's optional scenario-control wrapper uses
port `9001`; it is a different endpoint and should not be confused with the production bridge.

## What to read next

- [Browse the project](doc/index.md) — choose a question or a path through the documentation.
- [Host setup and first build](doc/host_setup.md) — prerequisites, the standard host gate, and result reporting.
- [Project overview](doc/overview.md) — the modes, message roles, boundaries, and safety behavior.
- [Runtime architecture](doc/arch/runtime.md) — how the services are assembled and how a tick moves through them.
- [Testing strategy](doc/testing/strategy.md) — what the C++ tests, SIL tests, simulator, and fuzzing each prove.
- [Control and plant model](doc/arch/control_plant.md) — the equations and code mapping.
- [Control and simulator notes](doc/notes/control_and_simulator.md) — how repeatable simulator experiments are structured.
- [IMU attitude design](doc/arch/imu_attitude_design.md) — the estimator and coordinate conventions.
- [Current status](doc/status.md) — what has strong evidence and what still requires hardware validation.
- [Running on Raspberry Pi](doc/Running_on_Pi.md) — deployment and physical bring-up.
- [Hardware reference](hardware/README.md) — parts, frame, wiring, and physical references.
- [Telemetry analysis](doc/testing/telemetry_analysis_cli.md) and [retained hardware data](data/README.md) — how runtime evidence is handled.

## A few useful commands

The standard host build and test gate is:

```bash
pytest --build
```

This configures and builds the host targets, runs CTest, and runs the Python SIL and simulator tests.

After that build, the main local executable is the simulator:

```bash
./build/balancer_simulator
```

Build the separate plant-audit target when you want the linearized model checks:

```bash
cmake --build build --target balancer_plant_audit
./build/balancer_plant_audit --all
```

For a quick look at the simulator's transfer catalog or one direct summary, use:

```bash
./build/balancer_simulator --catalog-json
./build/balancer_simulator --direct-summary 0
```

For a recorded simulator run or a transfer-validation report, follow the commands in the
[testing strategy](doc/testing/strategy.md). For a physical robot, use the
[Pi guide](doc/Running_on_Pi.md) rather than copying operational steps from this overview.

## Project shape

- `src/services/` — runtime services, controller integration, IMU, input, motors, and timing
- `src/messages/` — the reflected C++ message payload definitions
- `src/ipc/` — the synchronous message bus and UDP bridge
- `src/reflection/` — generators for the Python bindings and IPC documentation
- `tests/simulator/` and `tests/simulator_main.cpp` — the deterministic engine and its service wrapper
- `tests/python/` — SIL, simulator, dashboard, and artifact tests
- `tools/` — simulator runners, telemetry analysis, dashboard, and validation utilities
- `doc/` — the human-written handbook; `doc/ipc/` is generated
- `hardware/` and `data/` — physical references and retained runtime evidence
- `pid.conf` — the checked-in default profile used by the hardware runtime and simulator

## Scope and confidence

The simulator is the main software stability environment, not a substitute for a hardware
balance test. The project deliberately records that distinction in the
[current status](doc/status.md): software behavior can be strongly covered while final gains,
mechanical parameters, and difficult recovery behavior still require restrained physical
validation.
