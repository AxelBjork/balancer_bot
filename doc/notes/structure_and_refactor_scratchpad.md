# Structural Notes and Refactoring Scratchpad

> This is a non-normative maintainer note. It records larger structural questions and possible
> refactors; it is not a roadmap or a statement that any change is approved. Current behavior is
> described by the [project overview](../overview.md), [runtime architecture](../arch/runtime.md),
> [testing strategy](../testing/strategy.md), and generated [IPC protocol](../ipc/protocol.md).

**Status key:** `Current fact` describes the repository today; `Constraint` is a boundary a refactor
must preserve; `Candidate proposal` is an idea for later evaluation; `Deferred` means deliberately
out of scope until its prerequisites are known; `Decision needed` identifies an unresolved choice.

**Non-goals for this note:** nothing here authorizes changing controller law, PID schema or tuning,
IMU coordinate conventions, `MotorFeedback`, the telemetry wire schema, generated files, or hardware
deployment and validation. Those require a separate scoped request and proportionate tests.

## Executive view

The project is understandable once its central model is visible, but the repository currently
contains several overlapping ways to describe and run the same control system:

```text
physical robot      balancer_pi       real IMU + motors + TimeService       UDP :9000
service-level SIL   sil_app           injected clock/IMU, no plant          UDP :9000
direct simulation   balancer_simulator deterministic engine + plant          UDP :9001 or direct
```

That overlap is useful for verification, but it also creates the main structural cost: a reader or
maintainer must know whether a file describes the production service path, the simulator wrapper,
the direct engine, the wire contract, or the evidence generated from one of those paths.

The highest-value future work is therefore likely to be boundary clarification and composition
cleanup, not a broad controller rewrite. The hardware behavior, simulator behavior, telemetry,
generated interfaces, and tests are coupled; moving or renaming them casually would make the system
harder to compare and could hide safety regressions.

## Current structural pressure points

### 1. Three runtime modes plus an in-process test surface

**Current fact:** the project has three runtime modes plus an in-process test surface. They share
some interfaces while intentionally owning different clocks, sensors, motors, and plants.

The project has a hardware executable, a SIL executable, a direct simulator, and an in-process test
engine. They intentionally share some services and intentionally do not share others:

- hardware owns the real clock, IMU reader, input service, motor runner, and UDP bridge;
- SIL owns the service-level controller path and UDP bridge but receives its clock and sensor data;
- the direct simulator owns deterministic time and a plant model;
- tests call the engine and service pieces directly.

The [project overview](../overview.md) and [runtime architecture](../arch/runtime.md) explain these
distinctions, but the code still makes a reader infer them from several entry points. A future
runtime-topology document or source-level mode registry could make the composition explicit without
merging the modes.

**Candidate proposal:**

- define a small table or typed description of runtime capabilities: clock source, IMU source, motor
  backend, plant presence, transport, and artifact sink;
- have each executable's composition be visibly derived from that table or from named builders;
- keep the behavioral interfaces (`PhysicsTick`, `ImuData`, `MotorTargets`, `MotorFeedback`, and
  `SystemTelemetry`) unchanged.

Risk: a shared builder can accidentally flatten meaningful safety differences between hardware, SIL,
and simulation. The first step should be documentation or test coverage around composition, not a
large constructor rewrite.

### 2. Simulator implementation lives under `tests/`

**Current fact:** the simulator is central project functionality, but its implementation and service
wrapper live under `tests/`.

The simulator is central project functionality, but its implementation and service wrapper live in
`tests/simulator/` and `tests/simulator_main.cpp`. That is historically understandable—the
simulator began as a test harness—but it makes the source tree suggest that the simulator is merely a
test fixture.

**Candidate proposal:** possible long-term layouts include:

```text
src/simulator/       deterministic engine, plant, scenarios, artifact-facing results
tests/simulator/     simulator-specific tests and fixtures
tools/               CLI runners, validation, analysis, and report generation
```

or a smaller first step:

- keep the files where they are;
- add a clearly named simulator source map;
- separate “engine,” “scenario catalog,” “UDP wrapper,” and “test-only assertions” in the existing
  files and CMake targets.

Moving the simulator should be a deliberate refactor. It touches include paths, CMake target
ownership, generated artifacts, fuzz harnesses, and the Python test fixtures. It should not be done
just to make the directory tree look conventional.

### 3. Runtime composition is duplicated

**Current fact:** hardware and SIL entry points assemble overlapping service sets, while their
capability differences are intentional.

`src/services/main/control_app.h` and `tests/sil_app.cpp` each assemble overlapping service sets and
dispatchers. The differences are real, but duplicated wiring creates a risk that a new service or
message is added to one runtime and forgotten in another.

**Candidate proposal:**

- extract a small, non-owning dispatch/composition layer that declares the common service graph;
- pass explicit capability objects for the hardware IMU, motor runner, clock, and input source;
- keep hardware startup, GPIO/stepper energization, and safety checks at the hardware boundary;
- keep SIL's lack of `TimeService`, hardware reader, and motor backend obvious in its composition.

The desired result is shared wiring with visibly different capabilities—not one universal runtime
class with many boolean flags.

### 4. There are two UDP concepts with similar vocabulary

**Current fact:** the production `UdpBridge` and simulator endpoint are separate contracts, even
though both use UDP.

The production `UdpBridge` uses port `9000`, an active peer, reflected message authorization, and
service-bus dispatch. The simulator wrapper uses port `9001`, its own peer and run lifecycle, and
simulator telemetry/status messages. They are both UDP-facing, but they are not the same boundary.

**Candidate proposal:**

- name the abstractions distinctly in code and docs, such as `RuntimeUdpBridge` and
  `SimulatorUdpEndpoint`;
- define a small shared datagram/peer utility only for mechanics that are genuinely identical;
- keep the two message namespaces and lifecycle contracts separate;
- expose the port distinction in tests and artifact metadata.

Do not merge them merely because both use UDP. The separation is part of the current safety and
testability model.

### 5. Generated interfaces have several sources and outputs

**Current fact:** reflected C++ definitions and service annotations are the source for generated
bindings and IPC documentation. See the [reflection system](../reflection/system.md).

The reflected C++ message and service declarations generate Python bindings, Markdown protocol
documentation, DOT, SVG, and PNG artifacts. The pipeline is valuable, but a browser has to learn
which files are authoritative and which are products of the build.

**Candidate proposal:**

- make the source-of-truth chain explicit in one small diagram:

  ```text
  src/messages + service annotations
             ↓
  reflection registry and generators
             ↓
  Python bindings + IPC Markdown + graph assets
  ```

- make generated-file headers and regeneration commands consistent;
- add a drift check that regenerates into a temporary directory and compares outputs;
- keep generated protocol content out of hand-written architecture pages except for links and
  conceptual summaries.

The GCC reflection toolchain is currently a special host-only dependency. A future build cleanup
could isolate that dependency more clearly, but changing the generator or compiler is a separate
risk from reorganizing the documentation.

### 6. Scenario definitions and acceptance logic are distributed

**Current fact:** the [testing strategy](../testing/strategy.md) defines the validation vocabulary,
while the catalog, runner, acceptance checks, and artifact recorder remain distributed.

The simulator scenario catalog, engine, C++ acceptance checks, Python runners, fuzz registry, and
artifact recorder all participate in validation. This is powerful because direct, UDP, tuning, and
fuzz workflows can share the same engine, but it is difficult to see which part owns:

- scenario names and parameters;
- the acceptance decision;
- telemetry sampling versus full-rate metrics;
- report/artifact naming; and
- provenance such as PID digest and physics profile.

**Candidate proposal:**

- retain one canonical scenario catalog and one canonical acceptance function;
- expose a machine-readable catalog and a small human-readable scenario index;
- make every generated report point back to the catalog entry, PID input, physics profile, and
  engine version/commit;
- keep telemetry stride a presentation choice, never an acceptance choice.

This is a good candidate for incremental tests and metadata improvements before any directory move.

### 7. PID profiles need an explicit lifecycle

**Current fact:** the checked-in `pid.conf` is the shared baseline, while simulator tuning can emit
candidates under `build/sim`; [Current Status](../status.md) records the hardware-confidence limit.

The checked-in `pid.conf` is the default profile for hardware and simulation, while simulator tuning
can emit candidates under `build/sim`. The [control and simulator notes](control_and_simulator.md)
and [current status](../status.md) state this baseline/candidate distinction, but the structure
still leaves room for a future operator to confuse:

- the shared baseline;
- a simulator experiment input;
- a best-observed candidate; and
- a hardware-validated configuration.

**Candidate proposal:**

- define explicit profile roles and provenance fields without changing the current PID wire/schema
  contract;
- make candidate and qualified files carry a visible source profile and plant assumptions;
- require an explicit copy or deployment step before a candidate can be used on hardware;
- keep hardware validation status separate from simulator acceptance.

Do not split or retune PID files opportunistically while changing documentation or estimator code.

### 8. Telemetry is a cross-cutting interface

**Current fact:** `SystemTelemetry` crosses runtime, dashboard, simulator, analysis, and retained
evidence boundaries. The [telemetry analysis workflow](../testing/telemetry_analysis_cli.md) is the
shared-ingestion constraint.

`SystemTelemetry` serves the controller, UDP bridge, dashboard, simulator artifacts, Python analysis,
and hardware evidence. This makes it useful, but it also means seemingly small changes can affect
wire compatibility, generated bindings, CSV schemas, plots, and tests.

**Candidate proposal:**

- keep the reflected wire schema stable unless a task explicitly requires a change;
- document which fields are controller-facing, actuator diagnostics, plant-only, or hardware-only;
- keep one shared pandas-backed analysis implementation;
- distinguish controller time, receive wall time, and receive monotonic time in every derived
  artifact;
- treat absent or all-zero fields as unavailable rather than silently fabricating evidence.

The safest improvement here is better field ownership and artifact provenance, not adding telemetry
fields for every diagnostic question.

### 9. Safety and lifecycle ownership must stay visible

**Constraint:** any composition refactor must keep startup, shutdown, clock, motor authority, fault,
peer, and evidence ownership explicit in each mode.

This is the most important constraint on any shared runtime or composition refactor. The current
system has several kinds of ownership that should not be hidden behind a generic application
builder:

| Concern | Hardware runtime | SIL runtime | Direct simulator |
| --- | --- | --- | --- |
| Process startup/shutdown | `ControlApp` and the hardware executable | `sil_app` | `SimulatorService` or direct runner |
| Control clock | `TimeService` and steady-clock worker | External `PhysicsTick` injection | Deterministic engine timeline |
| IMU source | Linux IIO reader through `ImuService` | Disabled reader; external `ImuRawData` | Simulator sensor synthesis through the engine |
| Motor enable/disable | Stepper boundary and `controller_enabled` | Null motor backend | Simulated actuator and pulse scheduler |
| Balance/fault transitions | `RateControllerCore` plus `MotorRunner` fault state | Same controller logic, no physical actuator | Same controller logic plus simulated faults/limits |
| Network peer | Production `UdpBridge` on port `9000` | Production `UdpBridge` on port `9000` | Separate simulator endpoint on port `9001` |
| Thread lifetime | Runtime services and hardware readers | Service threads and bridge | Simulator wrapper or direct caller |
| Evidence output | Dashboard CSV and retained captures | UDP/test artifacts | Timeline, summary, plots, and validation reports |

The table is a design constraint, not a proposal to force all modes into one state machine. Before
extracting a shared builder, answer explicitly:

- Which object owns motor energization and guarantees de-energization during startup failure,
  shutdown, and actuator fault?
- Which object owns each thread, and what is the join/stop guarantee if a control cycle is active?
- Which clock is authoritative in each mode, and can a transport failure prevent the clock from
  advancing or merely prevent observation?
- Which component turns a stale/future IMU, fallover, or actuator failure into zero targets?
- Which endpoint owns peer registration and what happens to malformed or unauthorized packets?

The dependency direction should remain visible as well. The [MessageBus and UDP Bridge](../arch/message_hub.md)
and [Control and Plant Model](../arch/control_plant.md) pages describe the active boundaries.

```text
reflected C++ definitions + service annotations
                    ↓
        reflection registry and generators
                    ↓
      Python bindings + IPC docs + graph assets

hardware/platform boundaries → runtime services → synchronous bus → controller / estimator
                                          ↓
                               telemetry and artifact consumers
```

This is intentionally approximate. The rule it captures is that controller and estimator code
should not depend on the dashboard, artifact plotting, or deployment tools; the transport should not
become the owner of control safety; and generated outputs should not become a second source of
truth. Any refactor that changes those directions needs stronger review than a directory move.

## Larger refactors worth considering

These are separate proposals, not a single recommended rewrite.

### A. Make runtime composition first-class

Create named composition descriptions or builders for `HardwareRuntime`, `SilRuntime`, and
`SimulatorRuntime`. Their public differences would be visible in one place while the message/service
contracts remain shared.

Benefit: easier browsing, less duplicated wiring, lower risk of inconsistent service registration.

Cost: constructor ownership, thread startup, shutdown, and hardware safety become more abstract;
that abstraction must not hide when motors are energized.

### B. Promote the simulator into a first-class source area

Move engine/plant/scenario code from `tests/` into a dedicated source area, leaving tests and fuzz
harnesses under `tests/`.

Benefit: the directory tree reflects the simulator's importance and makes production-like simulator
code easier to discover.

Cost: broad CMake/include/test churn with little immediate behavior change. Do this only with a
complete build, artifact, and fuzz verification pass.

### C. Separate transport mechanics from runtime contracts

Introduce a small reusable UDP framing/peer utility while retaining separate production-bridge and
simulator-endpoint contracts.

Benefit: fewer duplicated socket details and clearer tests for framing and peer ownership.

Cost: peer replacement, malformed datagrams, authorization, and shutdown behavior are safety- and
test-sensitive. A shared utility must not accidentally make the two endpoints appear interchangeable.

### D. Formalize generated-interface ownership

Treat `src/messages/`, service annotations, the reflection registry, generated Python bindings, and
generated IPC docs as one explicitly documented interface pipeline.

Benefit: easier protocol maintenance and less uncertainty about which file to edit.

Cost: build-toolchain and CI work; generated artifacts must remain consistent across host and
cross-builds.

### E. Make evidence provenance a first-class artifact concept

Give simulator runs, transfer reports, dashboard captures, and retained hardware sessions a shared
metadata vocabulary for source, session, clocks, PID profile, physics profile, and transformations.

Benefit: easier comparison and fewer accidental claims that a hardware capture calibrated the plant.

Cost: touches Python artifacts, reports, dashboards, and retained-data conventions. It should be
implemented through the shared telemetry-analysis package, not a second parser.

## Conditional sequencing — requires separate approval

**Deferred:** this ordering is a dependency sketch, not an approved implementation roadmap. Each
step would need its own scope, tests, and review before work begins.

1. Keep the current runtime and wire contracts stable; improve source maps, mode tables, and
   generated-interface ownership documentation.
2. Add validation for generated-file drift, scenario-catalog provenance, and runtime composition
   coverage.
3. Extract or clarify shared composition only after tests express the hardware/SIL/simulator
   differences explicitly.
4. Improve artifact metadata and telemetry field ownership.
5. Consider directory moves or transport abstractions last, with a complete build and symmetric
   hardware/simulator tests.

## Questions to answer before a large refactor

- Which differences between hardware, SIL, and simulation are intentional safety boundaries?
- How should simulator-related messages remain visible in the reflected registry while the
  simulator's port-9001 transport stays distinct from the production `UdpBridge` on port `9000`?
- Which simulator code is reusable product behavior, and which is only test assertion/reporting?
- What metadata is required to reproduce or reject a simulator result?
- Which telemetry fields are stable API, and which are diagnostic implementation detail?
- Can every mode be started, stopped, and faulted without relying on hidden global state?
- What symmetric positive/negative tests must accompany any coordinate, sign, timing, or transport
  change?

## Reader-feedback summary

Informal fresh-reader reviews during the 2026-08-05 documentation pass consistently understood the
project as a self-balancing robot with three related runtime modes plus a C++ test binary. These
ratings are directional feedback from short read-only browsing sessions, not a usability study.
Before the documentation cleanup, readers had to ask for a single end-to-end diagram, the
SIL/direct-simulator distinction, the meaning of the active UDP peer, the message boundary, and the
hardware-confidence limits.

After the cleanup, independent readers rated the browseability about 8/10 and then 9/10. They found
the mode table, question-based portal, conceptual flow, port table, and local safety/confidence
boundary effective. The remaining feedback was mostly operational detail that belongs in specialist
pages: exact simulator client flows, protocol field layouts, estimator equations, and deployment
prerequisites.

The strongest recurring recommendation was to make boundaries explicit without collapsing them:
production UDP on `9000` versus simulator control on `9001`, SIL versus a plant-backed simulator,
internal messages versus wire-visible messages, and simulator evidence versus hardware proof.
