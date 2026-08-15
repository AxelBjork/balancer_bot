# Simulator behavioral matrix

The maintained controller-level scenario suite is the Python UDP/SIL suite in
[`tests/python/test_sim_scenarios.py`](../../tests/python/test_sim_scenarios.py). Each shared
behavior is defined once and parameterized over two explicit simulator/model configurations:

| Model | Physics profile | PID file | Attitude gains | Balance limit |
| --- | --- | --- | ---: | ---: |
| `DirectActuator` | `direct_actuator` | `tests/data/direct_actuator_pid.conf` | `6000 / 350 / 0` | `16000 SPS` |
| `StepperPhaseElectrical` | `stepper_phase_electrical` | `pid.conf` | `203550 / 1932 / 14.7` | `16000 SPS` |

The PID files are intentionally separate. `pid.conf` is now the rounded
StepperPhaseElectrical-calibrated shared default. The old DirectActuator values remain in an
explicit reference fixture so controller changes can still be compared against that model without
silently changing the production/default profile.

## Shared scope

The Python suite currently maintains 35 model-independent scenario definitions and executes each
against both models. The scope includes:

- quiet balance, sensor stress, and bounded long-horizon behavior;
- symmetric known-attitude `±1°`, `±2°`, `±4°`, and `±6°` recovery;
- the retained 50° estimator-limited high-angle boundary proxy;
- moderate disturbance recovery;
- drive, stop, and reversal;
- COM acquisition, motion-time learning freeze, and maintenance reacquisition;
- pitch-authority diagnostics and uncertainty/authority cases.

Some definitions are intentionally composite: they contain several signed, disturbance, gain, or
uncertainty sub-runs. They remain one pytest item so the suite retains its scenario-level meaning;
the runner records each concrete sub-run and continues collecting later cases after an earlier
behavioral gate fails. The generated matrix therefore reports both the aggregate pytest
model/scenario execution count and the concrete sub-run evidence. This is diagnostic expansion,
not a flattening of the maintained scenario catalog.

Acceptance is behavioral rather than trajectory-identical. It rejects falls, controller or
actuator faults, non-finite state, extreme persistent pitch/rate, sustained rail behavior, and
clear late-window growth. `_assert_no_growing_oscillation()` uses early/middle/late envelope
evidence plus a shorter late-window check, so a bounded residual oscillation is distinguishable
from an unstable one. DirectActuator retains its historical scenario scope and reference gates;
a few shared residual bounds are deliberately wider for the current StepperPhaseElectrical
candidate only when the behavior remains bounded and semantically useful.

## Strict model-specific xfails

Known gaps are applied centrally by `tests/python/conftest.py` with strict xfail semantics. An
unexpected pass is therefore visible and requires removing the corresponding expectation. The
current StepperPhaseElectrical xfails are grouped as:

- noisy push and high-authority attitude diagnostics;
- repeated ±1° authority pulses that lack a StepperPhaseElectrical zero-recovery witness (the
  DirectActuator hardware-envelope check remains a reference-only gate);
- drive/stop/reversal and external or initial-velocity recovery;
- velocity-estimator, authority-region, and reduced-translation cases;
- COM acquisition/maintenance and combined startup;
- long-horizon outer-loop behavior.

The DirectActuator 50° and StepperPhaseElectrical 50° cold-start proxy remain strict xfails. The
older 67° duplicate was removed from this maintained suite because the 50° case is the agreed
equivalent estimator/righting-authority boundary proxy; the design issue is still represented and
is not declared solved.

## Deliberate exclusions

The cross-model parameterization is limited to controller-level behavioral scenarios. Dedicated
tests remain model-specific for:

- StepperPhase current dynamics, phase geometry, free-wheel frequency, electrical power/passivity,
  and actuator convergence;
- STEP scheduling and timestamp/event behavior;
- UDP framing, artifact serialization, and direct-versus-UDP equivalence;
- the legacy transfer catalog and fuzz corpus;
- the non-ideal diagnostic profiles (`realistic` and `actuator_stress`).

Those tests do not acquire a DirectActuator twin merely to increase counts. The C++ GTest suite
also does not run this long UDP scenario matrix; it remains the fast in-process unit/integration
layer.

## Result artifacts

Run the maintained behavioral suite with:

```bash
pytest -q tests/python/test_sim_scenarios.py
```

`tests/python/conftest.py` writes generated results under the build tree:

- `build/sim/simulator_behavioral_matrix.json` — machine-readable rows, statuses, configs, and
  principal metrics, aggregate/sub-run counts, and grouped xfail evidence;
- `build/sim/simulator_behavioral_matrix.md` — human-readable summary and matrix.

Each composite simulator artifact may also contain
`behavioral_diagnostics.json`. It records named gates, model-specific diagnostic observations,
early/middle/late oscillation metrics, actuator authority, saturation, and the sub-run's principal
state metrics. The matrix uses the concrete artifact matching that sub-run and attaches aggregate
diagnostic records once, avoiding duplicate partial snapshots.

The artifacts are simulator/test evidence only; they are not hardware validation or permission to
apply the StepperPhaseElectrical candidate to the robot.

The numeric profile value `3` and the spelling `ideal_force` remain accepted as compatibility
aliases, while maintained source, CLI output, reports, and tests use `DirectActuator`.
