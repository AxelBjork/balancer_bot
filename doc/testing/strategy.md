# Testing Strategy

The project uses four complementary test layers:

- C++ unit and integration tests
- Python SIL tests against the production UDP boundary
- deterministic transfer-validation tests
- host/AFL fuzz-harness validation

## Validation vocabulary

These terms refer to different layers of the same validation workflow:

- **Behavioral matrix** — the shared controller-level acceptance slice in
  `tests/python/test_sim_scenarios.py`. Each scenario is parameterized over both `DirectActuator`
  and `StepperPhaseElectrical` with explicit PID files. This is the maintained source of truth for
  cross-model controller behavior; the DirectActuator scope remains in the same Python suite.
- **DirectActuator** — the current-force reference profile. Renaming `IdealForce` to
  `DirectActuator` did not remove its behavioral scenarios or its strict 50° boundary xfail.
- **Transfer matrix** — the legacy seven-case catalog used by the simulator CLI, UDP artifact
  workflow, tuner, and fuzz harnesses: four nominal cases plus three actuator-stress cases. It is
  retained for those interfaces and is not cross-model controller acceptance evidence.
- **Acceptance function** — the shared pass/fail decision applied to one completed scenario. It
  checks hard failures such as falls, faults, non-finite values, excessive pitch, tail RMS, and
  continuous saturation.
- **Behavioral-matrix validation** — running each shared controller behavior against both plants
  and preserving a result row for every execution. Strict model-specific xfails remain visible and
  are not counted as shared success. The generated Python artifact also records concrete sub-runs
  from scenarios that sweep signs, disturbances, or uncertainty.
- **Transfer-matrix validation** — running the legacy deterministic simulator matrix and
  preserving the per-run evidence and manifest. This is a software-model gate, not authorization
  to run candidate gains on hardware.
- **Direct-versus-UDP equivalence** — a separate focused protocol check. The current Python test
  compares the direct engine and simulator UDP wrapper tick-for-tick for transfer catalog index `1`;
  it is not an all-seven-case equivalence proof. The test’s inline scenario comment currently names a
  different case than the catalog order, so use the numeric index until that source comment is
  corrected.

When other pages say “transfer matrix” or “acceptance catalog,” they mean the legacy seven-case
workflow unless they explicitly describe the behavioral matrix. The [simulator behavioral matrix]
(simulator_behavioral_matrix.md) is the Python cross-model controller-behavior gate. The
[StepperPhaseElectrical test profile](stepper_phase_electrical.md) explains actuator/profile limits; the
[current status](../status.md) reports confidence rather than redefining either gate.

## Component Coverage

| Component | Unit tests | Simulator | Hardware |
| --- | --- | --- | --- |
| IMU | Axis mapping, circular fusion, wrap, reliability, and timing | Production sampling path with noise, bias, loss, and lag | Restrained orientation, axle rotations, and upright return |
| Motor feedback | Pulse scheduling, completed steps, reversal, saturation, and faults | Quantized completed-step feedback through actuator/tire dynamics | Restrained direction, step polarity, and missed-step checks |
| Balance control | Equation signs, cadence, limits, reset, and COM-trim state | Release and symmetric-push recovery across nominal and corner plants | Thirty-second neutral balance and restrained fallover |
| Joystick/velocity control | Ramp, reversal, command symmetry, stopping, and trim freeze | Cross-model drive/stop/reversal and COM-freeze scenarios; legacy transfer catalog retained separately | Equal positive/negative command response and stopping |
| Plant physics | Nonlinear/linear small-angle signs and parameter influence | Nonlinear cart-pole, actuator lag, traction, tire, and measured geometry | Parameters come from measured hardware; mismatches become simulator defects |
| Safety | Fallover, stale IMU, saturation, and actuator-fault paths | Fault-free finite state, pitch margin, and saturation-duration gates | Restrained fallover followed by cautious unrestrained validation |
| Runtime/telemetry | Message bus, timestamps, schema, and artifact parsing | Production UDP message path plus focused direct/simulator timeline equivalence | Telemetry-server capture used for transfer diagnosis |

## C++ Test Layer

`balancer_tests` covers the low-level behavior that is easiest to verify in-process:

- `motor_runner_test.cpp`
  time-based motor slew, pulse-frame scheduling, completed-step accumulation, reversal, and faults
- `control_loop_test.cpp`
  v12 planner/outer-loop equations, signs, cadence, saturation, reset paths, and completed-pulse feedback
- `simulator_runner_test.cpp`
  sign conventions, engine equivalence, all seven transfer scenarios, plant/IMU parameter A/B
  coverage, and physical invariants
- `time_service_test.cpp`
  exact tick publication and runtime monotonicity
- `latency_test.cpp`
  stepper timing/path sanity

Run it with:

```bash
./build/balancer_tests
```

## Executable and build boundaries

The main host workflow and the additional validation tools are different entry points:

| Target or layer | How it is built or run | What it proves |
| --- | --- | --- |
| `balancer_tests` | Built by `pytest --build`; run through CTest or directly | In-process controller, service, timing, simulator, and transfer-matrix checks |
| `balancer_simulator` and `balancer_simulator_tuner` | Built by `pytest --build`; tuner is a dependency of the simulator target | Deterministic scenario service, direct summaries, artifacts, and candidate evaluation |
| `sil_app` | Built by `pytest --build`; exercised by Python SIL tests | Production service/message-bus/UDP path without a plant or hardware backend |
| `balancer_plant_audit` | Build separately with `cmake --build build --target balancer_plant_audit` | Linearized plant and model consistency checks |
| AFL++ harnesses | `pytest --fuzz --build-only` or compiler-independent fuzz smoke | Fuzz target execution and corpus coverage, not transfer acceptance |
| `balancer_pi` | Raspberry Pi cross-build through `build_cmake OFF` | Physical deployment artifact; host validation does not prove hardware behavior |

The repository-root [`conftest.py`](../../conftest.py) owns the `pytest --build` options and build
workflow. [`tests/python/conftest.py`](../../tests/python/conftest.py) owns process fixtures and
binary discovery for the Python tests.

## Pytest Layer

The Python harness is responsible for the repo-level end-to-end workflow:

```bash
pytest --build
```

That command:

1. configures and builds the C++ targets
2. runs `ctest`
3. runs the Python SIL and simulator tests

For AFL++ harness validation, use:

```bash
pytest --fuzz --build-only
```

That keeps the normal `build/` flow unchanged, then configures a separate `build-afl/`
tree, builds the registered AFL++ harnesses, generates a deterministic seed corpus under
`build-afl/fuzz-corpus/`, and runs `afl-showmap` over those generated seeds.

The compiler-independent smoke gate is available even when AFL++ is not installed:

```bash
pytest --fuzz-smoke --build-only -q
```

It builds all three harnesses in `build-fuzz-smoke/`, regenerates the registered corpus, and runs
the registered seeds. The seed formats can select and mutate cases from the canonical transfer
catalog. AFL++ remains the instrumentation/coverage gate when its tools are available.

Key files:

- `conftest.py`
  owns the `pytest --build` options, CMake targets, and CTest workflow
- `tests/python/conftest.py`
  owns process management, binary discovery, and Python test fixtures
- `tests/fuzz/registry.py`
  registers the AFL++ targets, corpora, and `@@` command lines used by both pytest and
  `tools/run_afl.py`
- `tests/python/test_udp_bridge.py`
  verifies the production UDP bridge transport and peer behavior
- `tests/python/test_sil_loop.py`
  verifies the `sil_app` control path with injected tick/IMU traffic
- `tests/python/test_sim_scenarios.py`
  is the shared behavioral source of truth. It evaluates the maintained scenarios against both
  explicit model/PID pairs, retains the 50-degree estimator-limited boundary xfail, and keeps
  transport/actuator-specific tests separate
- `tests/python/conftest.py`
  applies centralized strict model-specific xfails and writes the machine-readable and
  human-readable behavioral matrix under `build/sim/`
- `tests/python/test_simulator_main_artifacts.py`
  checks protocol behavior, stride-invariant summaries, and exact direct-versus-real-UDP timeline
  hashes for a representative transfer scenario
- `tests/python/test_run_artifacts.py`
  checks artifact summarization and hardware-log parsing

### Large-angle brace recovery

The simulator's brace is a one-sided compliant pitch-torque fixture. It has no
vertical coordinate, contact geometry, or solved vertical reaction. On the
horizontal simulator floor the vertical balance is therefore

$$
N_L+N_R=Tg,
$$

both while the brace torque is active and after release; body pitch does not
introduce a $\cos\theta$ wheel-load factor. A future force-resolved brace must
instead solve its vertical reaction and use
$N_L+N_R=Tg-N_{brace}$. This fixture convention is common simulator logic,
while each actuator profile owns how it solves or applies tire contact force.

The production high-angle end-to-end regression is
`test_production_controller_recovers_from_67_degree_brace_over_ipc` in
`tests/python/test_sim_scenarios.py`. Pytest launches a clean simulator binary
and configures, commands, and observes the run only through the public UDP IPC
contract. It runs the real controller and `MotorRunner` path after estimator
settling, with `2.0 A`, `12.6 V`, a `48,000 SPS` controller cap, `mu=1`, and a
physical `0.200 kg` ballast contribution at `-3 mm` applied consistently to
total mass, first mass moment, and pitch inertia. The reflected start command
carries independent total-mass, first-moment, pitch-inertia, current, and
bus-voltage overrides. Reflected telemetry reports the applied values plus
traction, slip, cycle-slip, torque, and voltage-saturation diagnostics needed
for black-box acceptance.

The production enable path permits an explicit, direction-valid held forward
command to re-arm in the known fallen brace band (`60–70 deg`) when absolute
pitch rate is at most `30 deg/s`. There is no separate re-arm exception around
`40 deg`: an unbraced 40-degree cold start races estimator settle against
falling and is not the same maneuver. Once armed above the normal fallover
angle, authority continues only while the command remains direction-valid,
pitch stays within `70 deg`, and outward motion is no more than the `5 deg/s`
tolerance. These are continuously evaluated Boolean gates, not a recovery
phase, trajectory, or persistent recovery state. Inside the normal safe
region, ordinary enable semantics apply.

`SimulatorRunnerTest.ProductionControllerRecoversFromSixtySevenDegreeBrace`
is the focused in-process companion check. These fixtures are
controller/actuator acceptance evidence; they do not change nominal electrical
constants and do not imply that the tune is hardware-validated.

## Legacy Transfer Acceptance and Artifacts

The focused four-nominal-plus-three-actuator-stress legacy transfer matrix runs in-process in `balancer_tests`.
Direct tests, the simulator service, tuner, and fuzz harnesses obtain scenarios from
`transfer_scenario_set()`; direct
tests, the tuner, and UDP transfer runs use `evaluate_transfer_scenario()` for the hard acceptance
decision. This keeps the existing CLI/protocol workflow stable while the cross-model behavioral
matrix provides the controller-development comparison.

The human-readable matrix is:

| Profile | Release cases | Push case | Drive case | Duration |
| --- | --- | --- | --- | --- |
| Nominal | `nominal_release_pos`, `nominal_release_neg` | `nominal_push_symmetric` | `nominal_drive_bidirectional` | 20 s for release/push; 23 s for drive |
| Actuator stress | `actuator_stress_release` | `actuator_stress_push_symmetric` | `actuator_stress_drive_bidirectional` | 20 s for release/push; 23 s for drive |

The source of truth is [`transfer_scenario_set()`](../../tests/simulator/simulator_runner.cpp), and
the shared hard decision is [`evaluate_transfer_scenario()`](../../tests/simulator/simulator_runner.cpp).
The separate `simulator_scenario_set()` APIs contain required, capability, slow-push, and tuning
scenarios; they are not additional transfer-matrix cases.

The direct C++ tests evaluate all seven cases with the shared acceptance function. The
`run_transfer_validation.py` report runs all seven through the UDP wrapper, but its per-case `passed`
field comes from a separate direct-summary acceptance call; an unexpected UDP completion reason is
also recorded as a failure. That report therefore combines UDP transport evidence with direct-model
acceptance and should not be described as an all-tick direct/UDP equivalence result. The focused
Python equivalence test compares the complete result for transfer catalog index `1` only.

The simulator service’s scenario-control traffic uses its separate UDP port `9001`. Tests of the
production `UdpBridge` on port `9000` and tests of the simulator endpoint must remain distinguishable
in fixtures, logs, and reports.

To regenerate full-rate evidence and the stable report:

```bash
python3 tools/run_transfer_validation.py --include-build-gates
```

Each invocation writes a new directory under `build/sim/transfer/` and updates
`build/sim/transfer_summary.md`. Use `--telemetry-stride N` for investigative runs; the release
evidence uses stride `1`.

Run the staged velocity-reference tuner with, for example:

```bash
./build/balancer_simulator_tuner --stage feedback \
  --base tests/data/stepper_phase_electrical_pid.conf \
  --output build/sim/velocity_reference_tuning/feedback \
  --budget 140
```

The supported stages are `feedforward`, `observer`, `feedback`, `integral`, and
`joint`. They search only translational weights while keeping the electrical
physics, estimator, inner gains, adaptive COM trim, and fixed balance/turn
ceilings unchanged. Use feedforward to check planner motion, observer to
compare the slow velocity-feedback pole against plant truth, feedback for P,
integral for bounded leaky I, and joint only after the preceding regions are
understood. Every promoted candidate must still run the complete Python
behavioral matrix.

Candidates are ordered by hard faults/falls, bounded behavior, scenario
acceptance, and continuous recovery cost including pitch/rate RMS, saturation,
velocity residual, release distance, and rebound velocity. Each run writes
`candidate_summary.csv`, `scenario_metrics.csv`, `validation_results.csv`,
`search_bounds.txt`, `best_observed.pid.conf`, and a companion result note.
These files are simulator evidence and are never automatic authorization to
apply a configuration to hardware.

Artifacts are written under:

```text
build/sim/<run_id>/
```

Each preserved run contains:

- `timeline.csv`
- `metadata.json`
- `summary.json`
- `overview_plot.svg`
- `actuator_plot.svg`

These are generated by `tests/python/support/run_artifacts.py`.

You can also inspect a run or captured runtime telemetry with:

```bash
python3 tools/analyze_timeline.py build/sim/<run_id>/timeline.csv --summary-json
```

That reports:

- estimator pitch lag (`raw_acc_pitch_deg` -> `fused_pitch_deg`)
- estimator pitch-rate lag (`gyro_pitch_rate_dps` -> `filtered_pitch_rate_dps`)
- actuator/feedback lag (`u_sps` -> `raw_completed_velocity_sps` /
  `corrected_axle_velocity_sps`)

## Captured Reference Data

`tests/data/` contains captured hardware logs and parser fixtures. They are used for:

- parser robustness
- broad data sanity checks
- range/cadence inspection

They are not treated as golden replay traces for the simulator.

## What Each Layer Proves

- C++ tests prove the local math, contracts, service glue, and complete transfer matrix.
- `sil_app` tests prove the production UDP-facing message-bus runtime is alive and coherent.
- focused UDP tests prove wire behavior, artifact generation, telemetry downsampling, and exact
  timeline equivalence with the direct engine for the representative transfer case they exercise.
- AFL++ harness validation proves the fuzz targets still build, execute, and produce coverage on their seed corpora without changing the normal pytest gate.

For the deeper simulator and control scenario, read [StepperPhaseElectrical](stepper_phase_electrical.md). For the SIL-specific transport path, read [SIL Guide](sil_guide.md).
