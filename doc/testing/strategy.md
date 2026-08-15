# Testing Strategy

The project uses four complementary test layers:

- C++ unit and integration tests
- Python SIL tests over UDP
- deterministic unified-engine acceptance tests
- host/AFL fuzz-harness validation

## Component Coverage

| Component | Unit tests | Simulator | Hardware |
| --- | --- | --- | --- |
| IMU | Axis mapping, circular fusion, wrap, reliability, and timing | Production sampling path with noise, bias, loss, and lag | Restrained orientation, axle rotations, and upright return |
| Motor feedback | Pulse scheduling, completed steps, reversal, saturation, and faults | Quantized completed-step feedback through actuator/tire dynamics | Restrained direction, step polarity, and missed-step checks |
| Balance control | Equation signs, cadence, limits, reset, and COM-trim state | Release and symmetric-push recovery across nominal and corner plants | Thirty-second neutral balance and restrained fallover |
| Joystick/velocity control | Ramp, reversal, command symmetry, stopping, and trim freeze | Bidirectional 800 SPS command and stop scenarios | Equal positive/negative command response and stopping |
| Plant physics | Nonlinear/linear small-angle signs and parameter influence | Nonlinear cart-pole, actuator lag, traction, tire, and measured geometry | Parameters come from measured hardware; mismatches become simulator defects |
| Safety | Fallover, stale IMU, saturation, and actuator-fault paths | Fault-free finite state, pitch margin, and saturation-duration gates | Restrained fallover followed by cautious unrestrained validation |
| Runtime/telemetry | Message bus, timestamps, schema, and artifact parsing | Production message/control path plus direct/UDP timeline equivalence | Telemetry-server capture used for transfer diagnosis |

## C++ Test Layer

`balancer_tests` covers the low-level behavior that is easiest to verify in-process:

- `motor_runner_test.cpp`
  time-based motor slew, pulse-frame scheduling, completed-step accumulation, reversal, and faults
- `control_loop_test.cpp`
  controller v2 equations, signs, cadence, saturation, reset paths, and completed-pulse feedback
- `simulator_runner_test.cpp`
  sign conventions, engine equivalence, all ten transfer scenarios, plant/IMU parameter A/B
  coverage, and physical invariants
- `time_service_test.cpp`
  exact tick publication and runtime monotonicity
- `latency_test.cpp`
  stepper timing/path sanity

Run it with:

```bash
./build/balancer_tests
```

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
all fourteen seeds. The seed formats can select and mutate cases from the canonical transfer
catalog. AFL++ remains the instrumentation/coverage gate when its tools are available.

Key files:

- `tests/python/conftest.py`
  owns the `pytest --build` workflow and process fixtures
- `tests/fuzz/registry.py`
  registers the AFL++ targets, corpora, and `@@` command lines used by both pytest and
  `tools/run_afl.py`
- `tests/python/test_udp_bridge.py`
  verifies the UDP bridge transport path
- `tests/python/test_sil_loop.py`
  verifies the `sil_app` control path with injected tick/IMU traffic
- `tests/python/test_sim_scenarios.py`
  checks representative downsampled UDP runs and complete physics overrides
- `tests/python/test_simulator_main_artifacts.py`
  checks protocol behavior, stride-invariant summaries, and exact direct-versus-real-UDP timeline
  hashes
- `tests/python/test_run_artifacts.py`
  checks artifact summarization and hardware-log parsing

## Transfer Acceptance and Artifacts

The focused four-nominal-plus-six-conservative matrix runs in-process in `balancer_tests`. Direct tests,
the UDP simulator, tuner, and fuzz harnesses obtain scenarios from `transfer_scenario_set()`; direct
tests, the tuner, and UDP transfer runs use `evaluate_transfer_scenario()` for the hard acceptance
decision. This keeps the normal gate fast while retaining focused real-UDP protocol coverage.

To regenerate full-rate evidence and the stable report:

```bash
python3 tools/run_transfer_validation.py --include-build-gates
```

Each invocation writes a new directory under `build/sim/transfer/` and updates
`build/sim/transfer_summary.md`. Use `--telemetry-stride N` for investigative runs; the release
evidence uses stride `1`.

Run the direct closed-loop tuner with:

```bash
./build/balancer_simulator_tuner --base pid.conf \
  --output build/sim/tuning --top-k 12 --budget 350
```

The direct tuner evaluates every candidate against the complete transfer matrix from the start. It
first sweeps each gain independently from the configured PID, then runs deterministic local
two-axis refinements from the best observed candidates. This suits the measured low-inertia plant:
rate authority, pitch damping, and wheel damping interact, so a gain should not be discarded
because an artificial intermediate stage cannot settle on its own.

Candidates are ordered by transfer cases passed, hard faults/falls, then continuous recovery cost
(pitch, tail RMS, saturation, velocity residual, and command activity). The artifacts are
`candidate_summary.csv`, `scenario_metrics.csv`, and `validation_results.csv`. The tool always
writes `best_observed.pid.conf` and a companion text file recording its transfer result. It writes
`transfer_qualified.pid.conf` only if that candidate passes every transfer case. Neither file is an
automatic authorization to apply a configuration to hardware.

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
- `sil_app` tests prove the UDP-facing message-bus runtime is alive and coherent.
- focused UDP tests prove wire behavior, artifact generation, telemetry downsampling, and exact
  timeline equivalence with the direct engine.
- AFL++ harness validation proves the fuzz targets still build, execute, and produce coverage on their seed corpora without changing the normal pytest gate.

For the deeper simulator and control notes, read [Control and Simulator Notes](../notes/control_and_simulator.md). For the SIL-specific transport path, read [SIL Guide](sil_guide.md).
