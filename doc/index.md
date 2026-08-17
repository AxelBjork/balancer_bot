# Browse the project

This is the human-written handbook for `balancer_bot`. It is organized around the questions a
reader usually has, rather than around the order in which the source directories happen to appear.

For the shortest introduction, read the [repository landing page](../README.md). This page is the
canonical map for choosing what to read next; it does not need to be read front-to-back. The
generated IPC pages are the source of truth for message topology and payload layout; the pages below
explain what the system is doing and why.

## The shortest route to a mental model

1. [Project overview](overview.md) — the three runtime modes, the control loop, message roles, and safety boundary.
2. [Runtime architecture](arch/runtime.md) — how the services share the tick and message bus.
3. [Current status](status.md) — what the simulator and hardware evidence do and do not establish.

That route should be enough to understand what the project does without reading the protocol tables,
control equations, or deployment instructions.

## Choose a question

| If you want to know… | Read next | Then, if useful |
| --- | --- | --- |
| How do I prepare a host checkout and run the first build? | [Host setup](host_setup.md) | [Testing strategy](testing/strategy.md) |
| What is this project and why are there several executables? | [Project overview](overview.md) | [Runtime architecture](arch/runtime.md) |
| How does a sensor reading become wheel motion? | [Runtime architecture](arch/runtime.md) | [Control and plant model](arch/control_plant.md) |
| What is the difference between SIL and the direct simulator? | [Project overview](overview.md) | [SIL guide](testing/sil_guide.md) or [StepperPhaseElectrical test profile](testing/stepper_phase_electrical.md) |
| What does the simulator actually test? | [Testing strategy](testing/strategy.md) | [StepperPhaseElectrical test profile](testing/stepper_phase_electrical.md) |
| What equations and plant assumptions does the controller use? | [Control and plant model](arch/control_plant.md) | [StepperPhaseElectrical test profile](testing/stepper_phase_electrical.md) |
| How does the estimator interpret the IMU? | [IMU attitude design](arch/imu_attitude_design.md) | [Generated IPC protocol](ipc/protocol.md) |
| What are the StepperPhase equations, electrical constants, and profile limits? | [StepperPhaseElectrical test profile](testing/stepper_phase_electrical.md) | [Control and plant model](arch/control_plant.md) |
| What is the current velocity-reference outer loop? | [Control and plant model](arch/control_plant.md) | [Simulator behavioral matrix](testing/simulator_behavioral_matrix.md) |
| Which controller behaviors pass on each simulator plant? | [Simulator behavioral matrix](testing/simulator_behavioral_matrix.md) | [StepperPhaseElectrical test profile](testing/stepper_phase_electrical.md) |
| What can cross the UDP boundary? | [Project overview](overview.md#messages-and-boundaries) | [Generated IPC protocol](ipc/protocol.md) or [MessageBus and UDP Bridge](arch/message_hub.md) |
| How do generated bindings and IPC docs stay synchronized? | [Reflection system](reflection/system.md) | [Reflection quick reference](reflection/cheat_sheet.md) |
| How do I run or deploy the physical robot? | [Running on Raspberry Pi](Running_on_Pi.md) | [Hardware reference](../hardware/README.md) |
| What do the telemetry files mean? | [Telemetry analysis](testing/telemetry_analysis_cli.md) | [Retained hardware data](../data/README.md) |
| What is known, uncertain, or still hardware-only? | [Current status](status.md) | [Retained hardware data](../data/README.md) |

## Choose a reader path

### Just browsing

Read [Project overview](overview.md), look at the conceptual flow there, and finish with
[Current status](status.md). This gives the system shape and its confidence limits in three pages.

### Understanding the control system

Read [Project overview](overview.md), [Runtime architecture](arch/runtime.md),
[Control and plant model](arch/control_plant.md), and [IMU attitude design](arch/imu_attitude_design.md).
Use the [generated protocol](ipc/protocol.md) only when you need exact fields or wire sizes.

### Understanding validation

Read [Testing strategy](testing/strategy.md), then [StepperPhaseElectrical test profile](testing/stepper_phase_electrical.md),
and finally [Current status](status.md). The key distinction is that simulator acceptance is strong
software evidence but is not final proof of physical balancing behavior.

### Working with the physical robot

Read [Running on Raspberry Pi](Running_on_Pi.md), then [Hardware reference](../hardware/README.md),
and keep [Current status](status.md) open during bring-up.

### Looking at runtime evidence

Read [Telemetry analysis](testing/telemetry_analysis_cli.md) before interpreting simulator or
hardware CSVs. The [data archive](../data/README.md) explains which retained sessions are evidence
and which conclusions they cannot support.

## Reference pages

- [Host setup and first build](host_setup.md) — host prerequisites, build entry points, and verification reporting.
- [MessageBus and UDP Bridge](arch/message_hub.md) — the internal dispatch and external transport boundary.
- [Reflection system](reflection/system.md) — how C++ definitions produce the Python bindings and IPC docs.
- [Reflection quick reference](reflection/cheat_sheet.md) — project-specific notes for the C++26 generator.
- [Generated IPC protocol](ipc/protocol.md) — exact messages, fields, sizes, and service topology.

## Validation and maintainer notes

- [StepperPhaseElectrical test profile](testing/stepper_phase_electrical.md) — the maintained
  actuator realization, fixed simulator constants, and reproducible correlation scenario.
- [Control and plant model](arch/control_plant.md) — the model-neutral mass/coordinate equations,
  velocity-reference outer-loop algebra, safety limits, and controller-facing plant mapping.
- [Simulator behavioral matrix](testing/simulator_behavioral_matrix.md) — the shared controller-level
  scenario slice evaluated against DirectActuator and StepperPhaseElectrical with explicit configs;
  the same Python suite retains the full DirectActuator scope and the explicit 50-degree boundary
  xfail.
- [Testing strategy](testing/strategy.md) — canonical test-layer, transfer-validation, and artifact
  terminology.

## Document boundaries

| Material | Purpose and authority |
| --- | --- |
| Active handbook pages | Human-written guidance for project behavior, architecture, testing, deployment, and evidence interpretation |
| `doc/ipc/` and `tests/python/generated_balancer.py` | Generated interface products; derive from reflected C++ definitions and do not edit by hand |
| `data/hardware_sessions/` | Retained historical evidence bounded by each manifest and session README |
| `doc/testing/stepper_phase_electrical.md` | High-level actuator realization and reproducible simulator test profile |
