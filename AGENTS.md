# Repository Agent Guide

These instructions apply to the entire repository.

## Project Priorities

This repository controls real balancing hardware. Prefer small, explainable changes with focused
tests over broad controller rewrites. Keep hardware behavior, simulator behavior, telemetry, and
tests consistent, but do not expand a control task into dashboard, protocol, or PID-schema work
unless the request requires it.

The worktree may contain active hardware experiments and user edits. Inspect `git status` before
editing, preserve unrelated changes, and never discard or rewrite captured data without an explicit
request.

## Exploration and Handoff Protocol

For repository-review or documentation-discovery work, first inspect the repository state and state
the intended audience and question being answered. Choose evidence in whatever order best serves the
question: human Markdown, source code, tests, generated documentation, manifests, retained data, or
safe local checks. Do not prescribe Markdown or source code as the universal starting point.

Separate conclusions into:

- **Verified facts** — directly confirmed in the repository, with the relevant path or command.
- **Inferences** — interpretations supported by evidence but not directly stated by one source.
- **Unresolved questions** — expected information that was not found or could not be verified.
- **Recommendations** — proposed documentation, navigation, or structure improvements, clearly
  distinguished from current behavior.

Read-only reviewers must report a compact handoff containing:

1. the audience and question they adopted;
2. their first meaningful exploration path and evidence categories consulted;
3. each route switch and why it happened;
4. what they expected but could not find;
5. verified facts, inferences, unresolved questions, and recommendations; and
6. the route they would choose next time and why.

During review, do not edit tracked files, generated files, captured data, or unrelated content, and
do not execute hardware or deployment commands. Safe repository inspection, tests, builds, and
analysis commands are allowed when they do not alter tracked outputs. Keep generated outputs under
their existing ownership and report any unexpected changes rather than hiding them.

## Build and Verification

Use focused builds or tests while iterating. Before handing off a completed code change, run the
repository workflow:

```sh
pytest --build
```

Report failures and intentionally disabled tests accurately. Do not describe a focused test subset
as a complete pass.

For documentation-only changes, a build is normally unnecessary. Run an appropriate formatting or
`git diff --check` sanity check instead.

## Control and Hardware Conventions

- Motor steps per second are valid vehicle-speed feedback for the current stepper drivetrain.
- Public forward motion and telemetry use robot-forward as positive. Electrical motor inversion
  belongs at the hardware boundary.
- The configured IMU axis map presents gravity on mounted `-Z` when upright. The signed full-circle
  gravity pitch convention is `atan2(-ax, -az)`.
- Angle fusion and angle differences must be circular and wrap through `-pi`/`pi`; do not reintroduce
  folded `asin(sin(angle))` behavior.
- Do not add online orientation learning. Physical COM trim and IMU mounting orientation are
  different concepts.
- Preserve motor-SPS feedback, the existing inner balance loop, PID configuration schema, and
  telemetry wire schema unless the task explicitly places them in scope.
- Do not tune hardware and simulator PID files opportunistically while fixing estimator, message,
  dashboard, or tooling defects.
- Treat hardware logs as evidence, not calibrated plant-identification data, unless their manifest
  explicitly permits fitting.

When changing a sign or coordinate convention, update the estimator, controller-facing telemetry,
simulator sensor convention, fixtures, and tests together. Add symmetric positive/negative tests;
one-direction success is insufficient.

## Telemetry Analysis

Follow [doc/testing/telemetry_analysis_cli.md](doc/testing/telemetry_analysis_cli.md). The intended
implementation is one pandas-backed analysis package shared by:

- the telemetry-analysis CLI
- `tools/analyze_timeline.py`
- simulator/test artifact generation
- the existing summary and hand-written SVG code from
  `tests/python/support/run_artifacts.py`

Do not create another independent CSV parser, metric implementation, or plotting stack. Prefer the
shared CLI/library for ingestion, session detection, selections, common statistics, lag/scale,
spectra, JSON, and SVG output.

For a novel investigation that the shared tool cannot express, use a concise pandas exploration in
one terminal session or import the shared loader. Do not commit incident-specific parsing scripts.
Promote a calculation into the shared library only when it is reusable or needed by regression
tests. Never mutate the source CSV, and never silently treat absent or all-zero fields as real data.

Keep analysis clocks distinct: controller/simulator time, receive wall time, and receive monotonic
time answer different questions. Split or select sessions before calculating derivatives,
correlations, lags, or spectra. Never interpolate through a reset or material capture gap.

## Telemetry and Generated Interfaces

Avoid adding telemetry fields when the same information can be derived reliably from existing
signals. If a wire-visible message does change, update its reflected C++ definition and regenerate
or verify all derived bindings and protocol documentation through the repository workflow; do not
hand-maintain divergent copies.

Hardware-only captures often contain simulator fields that are consistently zero. Check field
availability before using them, and include exact source file, session, rows, and time window in any
reported result.

## Data and Artifacts

- Do not add large raw captures or new `hardware_sessions` extracts unless requested.
- Derived CSV, JSON, or SVG artifacts must state their source and transformation and should be
  written only to an explicit output path.
- Preserve existing checksum-bound fixtures and manifests.
- Build and transient analysis output belongs under `build/` unless a documented fixture is being
  deliberately added.

## Code Style and Scope

- Match surrounding C++ and Python style; avoid unrelated formatting churn.
- Use existing message types, helpers, signal catalogs, and artifact functions before adding new
  abstractions.
- Keep safety/fault behavior explicit and covered by tests.
- A diagnosis request is read-only: explain the evidence and cause, but do not implement a fix until
  a change is requested.
- A requested implementation should include proportionate tests and verification, but should not
  broaden into adjacent cleanup.
