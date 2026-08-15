# StepperPhaseElectrical controller-weight tuning report

Date: 2026-08-15

## Result

The final behavioral comparison surface was held constant. `DirectActuator` was not tuned and
remains 34/35. The original StepperPhaseElectrical configuration remains unchanged at 17/35.

An earlier proxy-search iteration incorrectly injected a `cart_damping = 40` physics override
from `simulator_runner`. That was removed. Proxy artifacts from that iteration are quarantined
below and are not used as clean-search evidence. The selected PID is retained only because it
was independently rerun against the fixed Python behavioral suite.

The selected simulator configuration is retained with full search precision in
`tests/data/stepper_phase_electrical_tuned_pid.conf`. Its rounded operational copy is now the
shared default `pid.conf`; the old DirectActuator reference is retained separately in
`tests/data/direct_actuator_pid.conf`:

```text
K_pitch = 203549.530474
K_rate  = 1932.44537688
K_accel = 14.6894521723
drive_max_acceleration_mps2 = 0.25
velocity_damping_per_s       = 5.5
velocity_pitch_limit_deg     = 2.5
velocity_I                   = 0.000980192549224
velocity_I_limit_deg         = 4.0
velocity_control_cutoff_hz  = 0.683120197506
balance_max_sps              = 16000
```

Normal strict-xfail execution of the current rounded profiles reports:

| Model | Strict pass | Strict xfail | Recovered strict-xpass | Behavioral coverage |
|---|---:|---:|---:|---:|
| DirectActuator | 34 | 1 | 0 | 34/35 |
| StepperPhaseElectrical | 28 | 7 | 0 | 28/35 |

The eleven recovered Stepper behaviors are now ordinary passes; only the seven remaining
Stepper gaps retain strict xfail markers. The complete repository gate currently reports
`135 passed, 8 xfailed`.

## Tools audited and changes

The relevant tools and interfaces were:

- `tests/simulator_tuner_main.cpp`: the old tuner searched only seven gains, used
  DirectActuator-scale priors, evaluated `transfer_scenario_set()`, and clamped pitch gain to
  200,000 and pitch-rate gain to 5,000. Its coarse pitch grid was only 4,000--20,000.
- `tests/simulator/simulator_runner.cpp` and `.h`: tuning scenarios were made profile-selectable;
  the Stepper searches use the existing `PhysicsProfile::StepperPhaseElectrical` physics profile.
  The runner does not inject or modify any physics parameters.
- `tests/python/test_sim_scenarios.py`: the Stepper PID path accepts the
  `STEPPER_PHASE_ELECTRICAL_PID_CONFIG` environment override. With no override, the original
  baseline file and its value checks are unchanged.
- `tests/python/conftest.py`: strict xfail definitions were not changed.

The tuner now searches all nine existing numeric weights that affect the relevant path:

1. Inner attitude: `pitch_gain`, `pitch_rate_gain`, `pitch_accel_gain`.
2. Outer velocity/drive: `drive_max_acceleration_mps2`, `velocity_damping_per_s`,
   `velocity_pitch_limit_deg`, `velocity_control_cutoff_hz`.
3. COM/trim: `velocity_I`, `velocity_I_limit_deg`.

`balance_max_sps`, `drive_max_sps`, and `turn_max_sps` remain fixed at 16000, 1200, and 1600;
they were not optimizer variables. Filters, estimator state, motor physics, safety behavior, and
controller structure were not changed.

## Search ranges and objective

The broad ranges actually used were:

| Weight | Coarse coverage | Random coverage / clamp |
|---|---|---|
| `pitch_gain` | 20k, 50k, 100k, 180k, 280k, 420k, 650k, 1M | log 20k--1M; clamp 0--1M |
| `pitch_rate_gain` | 1k, 3k, 6k, 12k, 24k, 40k, 70k | log 1k--70k; clamp 0--1M |
| `pitch_accel_gain` | 0, 10, 50, 200 | linear 0--500; clamp 0--1M |
| `drive_max_acceleration_mps2` | 0.25--4.0 coarse grid | log 0.25--4.0; clamp 0--10 |
| `velocity_damping_per_s` | 0.25--40 coarse grid | log 0.25--40; clamp 0--64 |
| `velocity_pitch_limit_deg` | 1--10 degree coarse grid | log 0.5--12; clamp 0--30 |
| `velocity_I` | zero plus 0.00005--0.016 | zero plus log 0.00002--0.02; clamp 0--0.02 |
| `velocity_I_limit_deg` | 0.5--20 degree grid | log 0.5--20; clamp 0--20 |
| `velocity_control_cutoff_hz` | 0.25--15 coarse grid | log 0.25--15; clamp 0.10--30 |

The old search space excluded the final useful pitch region and omitted the independent velocity
pitch limit and trim-limit weights. Candidate ordering is hard-failure first, then accepted-case
count, then a continuous cost emphasizing falls/faults, growing oscillation, pitch/rate RMS,
saturation, and outer tracking quality.

## Staged search results

The search budgets and best clean proxy results were:

| Stage | Budget | Best clean proxy result | Interpretation |
|---|---:|---|---|
| Inner wide | 600 | 5/5, score 4847.5, 37,889.7 / 1,563.0 / 2.70 | Found a quiet low-gain region, but it was not the best fixed-suite attitude/authority compromise. |
| Outer, profile-only | 600 | 4/5, score 36267.6, drive 0.309, damping 2.387, limit 8.531 | The clean profile-only proxy does not reproduce the fixed suite’s outer scenario override. |
| COM/trim, profile-only | 250 | 0/2, score 41736.6, `velocity_I` 6.66e-4, trim limit 8.00 | Useful as a scale probe, but not a reliable selector for the fixed outer suite. |
| Joint, profile-only | 700 | 11/12, score 39218.1, 214,229 / 2,749 / 20.60 | Best clean proxy candidate; it reverted to 17/35 on the fixed behavioral suite. |
| Fixed-suite candidate validation | neighborhood | 28/35 behavioral coverage at the selected point | Valid final-suite evidence for the selected PID, but its original discovery used the quarantined proxy. |

The earlier `outer_cart40`, `joint_cart40`, `outer_high_inner`, and related directories under
`build/sim/stepper_tuning/` were generated before the runner cleanup and must not be compared
with the clean proxy rows above. The selected PID was independently evaluated by the unchanged
behavioral suite and produced 28/35; that result is valid as a candidate behavior measurement,
but the current profile-only C++ proxy has not yet rediscovered it.

Intermediate tables and clean validation CSVs are retained under
`build/sim/stepper_tuning/clean_inner`, `clean_outer`, `clean_com`, and `clean_joint`. The older
`outer_cart40`, `com_wide3`, `joint_cart40`, `outer_high_inner`, and `final/neighborhood`
directories are retained as historical artifacts only and are not clean-search evidence.

## Baseline versus selected behavior

The fixed suite represents 202 concrete model/subrun rows. For StepperPhaseElectrical:

| Measurement | Baseline | Selected |
|---|---:|---:|
| Model-level strict pass | 17 | 17 |
| Unresolved strict xfail items | 18 | 7 |
| Recovered strict-xpass items | 0 | 11 |
| Recorded subruns passed | 18 | 69 |
| Recorded subruns failed | 62 | 11 |
| Diagnostic subruns | 1 | 1 |
| Not-recorded aggregate rows | 21 | 21 |

The fixed-suite matrix is therefore a large continuous improvement even where the retained
strict xfail marker still makes pytest exit nonzero. The selected configuration recovers COM
acquisition/interruption behavior, long-horizon behavior, estimator perturbation behavior,
startup behavior, the outer gain region, external push recovery, and the repeated pitch-pulse
behavior.

Across 81 recorded Stepper row summaries, selected versus baseline continuous values were:

| Metric | Baseline median / p95 | Selected median / p95 |
|---|---:|---:|
| Max absolute pitch | 5.04° / 11.81° | 2.38° / 7.18° |
| Peak pitch rate | 188.5 / 288.9°/s | 24.8 / 210.8°/s |
| Tail pitch RMS | 2.88° / 3.82° | 0.016° / 1.14° |
| Tail pitch-rate RMS | 73.97 / 108.4°/s | 1.87 / 12.22°/s |

Known frontier rows still dominate the selected maxima: the recorded maximum pitch is 43.5°,
maximum peak rate is 371.6°/s, and maximum tail pitch RMS is 10.15°. These are not hidden by the
aggregate improvements and correspond to the remaining authority/noise/diagnostic boundary.

## SPS authority and saturation

The 16000-SPS ceiling was fixed throughout. In the 81 recorded Stepper rows:

- Baseline: 80 rows reached at least 15000 SPS, 78 reached exactly 16000 SPS, and summed rail
  time was 369.77 s.
- Selected: 17 rows reached at least 15000 SPS, 2 reached exactly 16000 SPS, and summed rail time
  was 0.508 s.
- Selected maximum rail time in one row was 0.259 s; maximum continuous saturation was 0.193 s.
- For selected rows, the median per-row command 95th percentile was 86.6 SPS and its across-row
  p95 was 1673 SPS. The corresponding command 99th-percentile values were 166.8 SPS and 9076 SPS.

This is a materially less rail-dependent result. The remaining rail use is concentrated in the
unresolved authority, sensor-lag, noisy-push, and high-angle cases rather than nominal balance.

## Remaining failure groups

The selected configuration leaves seven Stepper strict-xfail items:

- Command authority: reduced-translation authority remains a traction-limit reporting failure
  in all four authority fractions; the sustained transient still misses the authority witness
  and post-transient-speed envelope.
- Drive/stop: full-forward-then-stop and both signed drive/reversal subruns remain outside the
  current motion envelope. The selected low-drive point is bounded but does not meet the minimum
  drive-direction witness in every window.
- Pitch-authority uncertainty: both sensor-lag signs stop after one pulse with controller fault
  flag 8; the aggregate observes 38 rather than 42 authority rows.
- Noisy slow push: the run reaches the existing controller-fault/authority boundary.
- Cold start: the 50 degree estimator-limited boundary remains an expected high-angle failure.

The former initial-velocity recovery item is recovered at the selected point. The remaining
authority and sensor-lag failures are not evidence that more inner gain alone will solve the
problem.

## Neighborhood, symmetry, and discontinuity checks

The neighborhood table below was measured around the pre-selection high-inner center
(`K_pitch = 226166.144971`, `K_rate = 1932.44537688`, damping `= 5.5`). The final tracked point is
the `K_pitch -10%` row, and it was rerun independently as the selected configuration.

Full-suite `--runxfail` neighborhood results were:

| Reference / perturbation about pre-selection center | Stepper behavioral coverage | Recorded fail rows |
|---|---:|---:|
| Center | 27/35 | 11 |
| `K_pitch -10%` | 28/35 | 11 |
| `K_pitch +10%` | 27/35 | 24 |
| `K_rate -10%` | 26/35 | 13 |
| `K_rate +10%` | 28/35 | 11 |
| Damping -10% | 26/35 | 13 |
| Damping +10% | 28/35 | 11 |
| All three favorable moves combined | 27/35 | 11 |

The final selection uses `K_pitch -10%` because it ties the best coverage with lower median/p95
command demand and lower summed saturation than the `K_rate +10%` point. The damping +10% point
has similar coverage but substantially worse frontier peak and rail metrics. The favorable moves
do not combine monotonically, so the region is broad but coupled rather than a single separable
optimum.

Signed positive/negative pairs were symmetric in the ordinary recovery and COM cases (many
paired metric differences were exactly zero). The largest recorded signed differences were small
transient/uncertainty effects, such as 0.11° peak pitch for the transient pair and 0.30° for the
estimator-bias pair. Both sensor-lag signs fail in the same way. No unexplained one-sided failure,
narrow pathological island, or tiny-gain discontinuity was observed.

## Assessment and next step

The fixed behavioral suite establishes that controller weights alone can increase coverage from
17/35 to 28/35 for the selected candidate, with substantially quieter attitude/rate behavior and
less rail use. However, after removing the invalid runner physics override, the clean C++ proxy
search did not rediscover that candidate: its best joint candidate returned to 17/35. The amount
of recoverable coverage is therefore real, but the current proxy objective/search path is not yet
validated as a reliable way to find it.

The remaining gap is not purely tuning. The unresolved cases cluster around fixed authority
reporting/diagnostics, sensor-lag completion, high-angle estimator limits, noisy authority, and a
drive/stop envelope that trades off against the successful outer recovery region. This supports a
mixed diagnosis: mostly untuned behavior was recovered, while the final seven scenarios need
targeted authority/architecture or simulator-diagnostic triage.

The next smallest experiment is to make candidate promotion use the unchanged fixed behavioral
suite (or an equivalent external harness) while leaving `simulator_runner` profile-only. Then
search a small neighborhood around the selected outer point over
`drive_max_acceleration_mps2`, `velocity_damping_per_s`, and
`velocity_pitch_limit_deg`, scored on full-forward/drive-reversal, transient authority, initial
velocity, and long-horizon cases. No runner physics override should be reintroduced.
