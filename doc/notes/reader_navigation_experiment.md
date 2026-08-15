# Reader Navigation Experiment

This note records an adaptive, qualitative review of how an independent visitor or maintainer
discovers `balancer_bot`. It is separate from the
[structural refactoring scratchpad](structure_and_refactor_scratchpad.md): this note records
navigation and documentation evidence, while the scratchpad is reserved for larger codebase
structural issues.

## Objective and boundaries

The objective is to learn whether readers can start from the evidence that feels natural, reach an
authoritative explanation quickly, and distinguish runtime behavior from simulator behavior,
generated interfaces, retained evidence, and future structural proposals.

The review is not a formal usability study or statistical experiment. Existing staged and unstaged
Markdown edits are preserved. Reviewers do not edit files, generated outputs under `doc/ipc/`,
captured data, or hardware. Human-written Markdown may be rewritten, split, consolidated, or
restructured when the evidence supports a clearer discovery path.

## Adaptive protocol

The default target is six concurrent pairs, or twelve completed reviewers. The pairs are not assigned
a fixed order. After each pair, the findings are verified, the documentation is improved, and the
next focus is selected from the most important remaining confusion, stale claim, navigation gap, or
source/documentation mismatch. A lens may be repeated, skipped, or combined when the latest evidence
calls for it. If the evidence converges early, the actual stopping point and rationale will be
recorded.

Every reviewer receives this core request:

> Explore this project as an independent visitor or maintainer. Use whichever repository evidence
> seems most useful. Explain what the project does, how easy it is to navigate, what you trust, what
> remains unclear, and what should improve. Do not edit files. Record your initial path, evidence
> sources used, reasons for switching, unresolved questions, and preferred route.

Available review lenses include cold-start discovery, runtime composition, hardware and telemetry,
control and validation, tooling and generated interfaces, and skeptical final review. A lens is a
focus for questions, not an instruction to begin in a particular evidence category.

## Reviewer handoff format

Each reviewer is summarized with:

- audience and question;
- first meaningful path;
- evidence categories consulted;
- route switches and reasons;
- verified facts;
- inferences;
- unresolved or missing information;
- documentation or structural recommendations; and
- preferred next route.

## Pair log

Pair summaries, adaptive focus, source verification, documentation changes, and checks are recorded
below.

### Pair 1 — independent baseline discovery

Both reviewers independently adopted an independent-maintainer audience and began by checking
repository state and the top-level tree. Both then followed the current visitor route through
`README.md`, `doc/index.md`, `doc/overview.md`, and `doc/status.md` before switching to source,
tests, build metadata, generated interfaces, telemetry tooling, deployment guidance, and retained
data. Neither reviewer was assigned a starting evidence category; both ended with a combined
Markdown-plus-source route preference.

Verified findings included the three runtime modes, the distinct UDP ports `9000` and `9001`, SIL’s
external tick/IMU/command injection, host reflection generation, and checksum-bound hardware data
limits. The source checks were made against `CMakeLists.txt`, `conftest.py`, `tests/sil_app.cpp`,
`src/services/main/control_app.h`, `src/ipc/udp_bridge.h`, `cmake/reflection.cmake`, the generated
protocol, and the relevant data manifests. CTest and pytest collection counts were treated as
discovery facts, not pass results.

The main unresolved navigation gaps were a first-class host prerequisites/build page, a visible
policy for checking generated outputs without overwriting them, and a clearer ownership boundary
between `tools/telemetry_analysis/`, `tools/analyze_timeline.py`, and
`tests/python/support/run_artifacts.py`. A further concern was that the strongest navigation pages
are currently part of the dirty worktree rather than a clean committed baseline.

Changes after this pair:

- added [Host Setup and First Build](../host_setup.md) and linked it from the README, portal, and overview;
- added a source map for the hardware, SIL, direct-simulator, and in-process-test surfaces;
- documented generated-output synchronization and the source-tree write behavior;
- clarified telemetry-analysis ownership without creating another parser or metric stack.

The next pair will independently inspect the host/build/reflection path and test whether the new
setup and provenance guidance answers the reviewers’ missing-information questions without requiring
source-first exploration.

### Pair 2 — host, reflection, and telemetry follow-up

Both reviewers used the prior findings as an adaptive focus while retaining independent route choice.
They began with repository state and the portal/note, then checked the new host guide against
`CMakeLists.txt`, `conftest.py`, the devcontainer, reflection CMake, runtime entry points, telemetry
modules, dashboard playback, and data documentation. Both switched from documentation to source or
build metadata to verify claims about generated writes and analysis ownership.

Verified findings included that the runtime source map matches the actual entry points, `pytest
--build` builds reflection targets and runs CTest before Python tests, and reflection writes tracked
outputs directly into the source tree. The reviewers also confirmed that `xbox_controller.cpp` is
compiled into the normal test library and includes SDL2, that the CLI imports analysis through
`run_artifacts.py`, and that `measure_pitch_inertia.py` already uses the shared frame loader while
owning its domain-specific calculation.

The first pass had three gaps: it understated SDL2’s host-build requirement, called a mutating
reflection command a safe check, and described telemetry ownership only by directory. The current
CLI also does not expose the session/time-window selectors described by the general analysis rules.

Changes after this pair:

- corrected the host prerequisites and added the explicit plant-audit target build;
- renamed the reflection section as a controlled mutating synchronization check and documented the
  ignored PNG caveat;
- added a function-level telemetry ownership table and clarified current CLI selection limits;
- added a portal question for generated-interface synchronization.

The next pair will focus on telemetry evidence and hardware/data discovery: whether a reader can move
from a capture or manifest to a valid, bounded claim without mistaking retained extracts for raw
data, calibration, or a current verification result.

### Pair 3 — hardware evidence and provenance

Both reviewers chose an independent maintainer auditing a retained hardware result and began at the
data archive rather than being routed through the general portal. They checked session READMEs,
manifests, retained CSV/SVG hashes and headers, ignored dashboard captures and event logs, the
dashboard logger, shared analysis code, the Pi guide, status, and the current PID identity. Both
switched from the archive to implementation when a manifest claim needed verification.

Verified findings included that dashboard files are received telemetry packet logs, source captures
are often unavailable, retained artifact hashes are present for most later sessions, and the
2026-07-18 manifest describes 400 Hz source windows while its retained fixtures are 50 Hz. The
reviewers also confirmed that the historical 2026-07-19 PID identity differs from the current
working-tree profile and that the analysis rules correctly prohibit interpolation across gaps or
resets.

The archive’s strongest property is bounded claim language and artifact integrity. Its remaining
reproducibility gap is provenance: a checksum validates retained bytes, but not a discarded
extraction procedure, source-to-extract mapping, or current runtime identity. The generic timeline
CLI still lacks session and time-window selectors, which makes the documented analysis rules harder
to follow for arbitrary multi-session dashboard files.

Changes after this pair:

- added claim/reproducibility levels and packet-log/session caveats to the data archive;
- documented the 2026-07-18 source-rate versus fixture-rate limitation without changing its manifest;
- marked the retained 2026-07-19 PID identity as historical rather than current verification;
- strengthened the status-page caveat about historical hardware evidence.

The next pair will inspect control-validation terminology and the agreement between testing pages,
scenario catalogs, acceptance functions, simulator paths, and executable/test boundaries.

### Pair 4 — control validation and test terminology

Both reviewers started from repository state and executable/test inventory, then compared the portal,
overview, runtime, status, testing, SIL, simulator, CMake, root pytest workflow, scenario sources,
Python protocol tests, fuzz registry, and transfer-report script. Both switched from documentation
to implementation when the meaning of “equivalence,” “acceptance,” or “catalog” required verification.

They agreed that the SIL/direct-simulator distinction, port separation, safety caveat, and broad test
layer story are clear. They found several source-backed mismatches: the exact direct-versus-UDP Python
test exercises only transfer catalog index `1` (while its inline comment names a different case), the
transfer report’s `passed` value comes from direct acceptance while UDP completion is recorded
separately, the scenario APIs contain more than one catalog, and the build workflow is owned by the
repository-root `conftest.py`, not the Python fixture conftest.

The reviewers also identified missing human-readable maps for the ten transfer cases and for the
additional plant-audit, tuner, and fuzz targets. A contradictory “Hardware-in-the-loop” CMake comment
for the plant-free `sil_app` was recorded as a source-level issue but left unchanged because this
task is documentation-only.

Changes after this pair:

- narrowed direct-versus-UDP equivalence claims to the representative Python-tested case;
- documented the direct acceptance versus UDP transport boundary in transfer reports;
- added a ten-case transfer matrix and explicit executable/build-boundary table;
- corrected the `conftest.py` ownership references and expanded the overview source map.

The next pair will focus on generated interfaces, protocol boundaries, Python tooling, and the
maintainer workflow for moving from reflected C++ definitions to usable tests and reports.

### Pair 5 — generated interfaces and tooling boundaries

Both reviewers investigated the source-to-generated chain without running generators. They checked
repository state, CMake reflection wiring, message IDs and payload definitions, service metadata,
the UDP bridge and publisher, the reflection registry, generated Python/protocol outputs, dashboard
and analysis consumers, and the host workflow. One followed build/reflection authority first; the
other followed the portal into the same chain. Both switched into source and generated outputs to
verify ownership and endpoint boundaries.

Verified findings included that `MsgId` and payload definitions are authoritative C++ sources, the
registry selects the UDP-visible binding subset, generated IPC docs include broader internal/runtime
messages, port `9000` and port `9001` are separate transports, and reflection writes outputs into the
source tree. The reviewers also found that dashboard JavaScript requests `motion.measured_velocity_sps`
and `controller.velocity_error`, while the current `telemetry_view()` output does not provide those
keys, and that the generated `UdpBridge` description currently ends abruptly because the annotation
buffer is bounded.

These are tooling/source issues rather than reasons to edit generated files during this task. The
documentation now gives a source-of-truth chain, distinguishes generated products from authorities,
corrects the reflection toolchain wording, and warns readers not to treat a missing dashboard chart
or truncated generated paragraph as authoritative evidence.

The next pair is the final skeptical end-to-end pass. It will start without a prescribed route,
review the improved handbook as a whole, and focus only on durable remaining navigation or factual
clarity gaps after the five adaptive pairs.

### Pair 6 — skeptical end-to-end review

Both final reviewers independently started with repository state and the README/portal/overview/status
route, then traversed the handbook, build/reflection sources, generated interfaces, testing and data
pages, dashboard tooling, and the structural scratchpad. Their route switches were intentional:
handbook to source/build metadata for claim verification, overview to runtime entry points for mode
boundaries, generated protocol to C++ authority, testing to scenario/transfer code, and data manifests
to logger and current PID identity.

They found the visitor mental model strong and the active/generated/historical/structural separation
mostly clear. They independently confirmed that the remaining issues are not basic portal failures:
the general telemetry CLI still lacks bounded selection flags, provenance is inconsistent across old
manifests, reflection has no non-mutating drift check, dashboard browser paths do not all match its
server JSON, and generated explanatory prose can be truncated. The CMake SIL comment and generated
compiler footer remain source/generated wording issues outside this documentation-only scope.

Final changes after this pair:

- linked and classified this process note from the documentation portal;
- replaced the document-boundary bullets with a compact authority/purpose table;
- completed the reviewer count, route observations, stopping rationale, strongest improvements, and
  durable remaining questions below.

## Final qualitative summary

Twelve reviewers completed six adaptive pairs. All twelve ultimately used a combination of human
Markdown plus authoritative implementation, generated, test, tooling, or data evidence; no reviewer
remained in a single evidence category. Markdown and the portal were the common route for forming a
mental model, while source/build inspection was the common switch for disputed runtime, build, or
generated-interface claims. Data-first reviewers switched to manifests, hashes, logger behavior, and
analysis code when claim provenance mattered. Reflection-first reviewers switched to C++ definitions,
the registry, and CMake outputs to establish authority. The final pair returned to the portal and
found the improved route durable.

The strongest improvements were the question-based portal and three-page mental-model route, the
host setup/source map, explicit generated-output ownership and mutation warnings, the evidence-level
rules for retained hardware data, and the human-readable validation/build matrix. These changes let a
reader begin where it feels natural while making the switch to authoritative evidence explicit.

Remaining structural or tooling questions are deliberately not hidden in the active handbook: the
project still lacks a non-mutating generated-output drift check, the generic telemetry CLI lacks
session/window selectors, old capture manifests have uneven extraction and identity metadata, the
dashboard has known browser/server field-path mismatches, and generated annotation prose can truncate.
The source-level stale SIL comment, generated compiler footer, and broader telemetry/catalog
refactors belong in their respective code or structural follow-up work, not in this navigation log.

The default twelve-reviewer target was reached, so the review stopped without claiming statistical
significance. No hardware, deployment, build, test, or generator pass was claimed; final verification
was limited to documentation whitespace, relative-link, changed-path, and generated-directory checks.
