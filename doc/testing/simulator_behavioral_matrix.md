# Simulator behavioral matrix

The maintained controller-level comparison is the Python suite in
[`tests/python/test_sim_scenarios.py`](../../tests/python/test_sim_scenarios.py).
Every shared behavior runs against two explicit configurations:

| Model | Physics profile | PID file | Inner gains | Balance ceiling |
| --- | --- | --- | ---: | ---: |
| `DirectActuator` | `direct_actuator` | `tests/data/direct_actuator_pid.conf` | `6000 / 350 / 0` | `16000 SPS` |
| `StepperPhaseElectrical` | `stepper_phase_electrical` | `pid.conf` | `203550 / 1932 / 0` | `16000 SPS` |

The physics profiles, geometry, estimator, scenario definitions, and assertions
are fixed comparison inputs. Tuning changes controller weights only. The
common plant equations and controller law are documented in
[`doc/arch/control_plant.md`](../arch/control_plant.md); the electrical
actuator realization is documented in the
[`StepperPhaseElectrical test profile`](stepper_phase_electrical.md).

## Scope

The suite contains 35 named scenario definitions, with composite definitions
retaining their concrete sub-runs. It covers:

- neutral balance, noise, disturbances, and long-horizon behavior;
- signed `±1°`, `±2°`, `±4°`, and harder high-angle recovery;
- drive, stop, reversal, initial velocity, and reduced-authority behavior;
- pitch authority and estimator/uncertainty cases;
- fixed-trim motion and disturbance behavior; adaptive COM acquisition,
  motion-time COM freeze, and maintenance behavior are explicitly deferred;
- startup and fallover/re-arm boundaries.

Acceptance rejects falls, controller or actuator faults, non-finite state,
extreme persistent pitch/rate, sustained rail operation, and clear late-window
growth. Bounded oscillation remains distinguishable from divergence. Composite
diagnostics record each signed, disturbance, gain, or uncertainty sub-run in
the build tree.

## Current result

The current host result is:

| Model/surface | Result |
| --- | ---: |
| `StepperPhaseElectrical` focused matrix | `23 passed, 4 skipped, 8 strict xfailed` |
| `DirectActuator` focused matrix | `31 passed, 3 skipped, 1 strict xfailed` |
| Complete `pytest --build` Python run | `130 passed, 7 skipped, 11 expected xfailed` |
| C++ controller/simulator suite | `259/259 passed` |

The StepperPhaseElectrical xfails are intentional and currently represent:

- noisy push and sustained initial-velocity authority boundaries;
- direct constant-lean pitch-authority and plant-uncertainty boundaries;
- the retained 50° estimator-limited startup boundary.

The focused matrix deliberately skips adaptive COM acquisition/maintenance and
the generic reduced-traction override for StepperPhaseElectrical. Two precise
standalone boundary tests are also expected to xfail: `P=8/s` with `2.5°`
motion authority, and `100 ms` controller-velocity latency.

The DirectActuator 50° startup case remains its single strict xfail. An
unexpected pass is a failure under strict xfail semantics and must be reviewed
before removing a marker.

## Artifacts and interpretation

Run the matrix with:

```bash
pytest -q tests/python/test_sim_scenarios.py
```

The test harness writes:

- `build/sim/simulator_behavioral_matrix.json` — machine-readable scenario,
  model, status, config, and principal metrics;
- `build/sim/simulator_behavioral_matrix.md` — generated human-readable table;
- per-sub-run `behavioral_diagnostics.json` files under the model artifact
  directories.

The artifacts are simulator evidence only. They do not authorize a hardware
configuration. Preserve the PID file, repository revision, scenario metadata,
and physics profile whenever quoting a result.

## What is deliberately separate

The behavioral matrix is not the entire simulator test surface. Dedicated C++
tests cover:

- StepperPhaseElectrical current dynamics, phase geometry, power consistency,
  fixed-field ringdown, and numerical convergence;
- STEP scheduling and timestamp/event behavior;
- direct-versus-UDP transport and artifact serialization;
- the legacy seven-case transfer catalog and actuator-stress profiles;
- estimator, input, safety, and message-schema invariants.

These are separate because they answer different questions; adding a
DirectActuator twin to every physics or transport test would obscure rather
than improve the comparison.
