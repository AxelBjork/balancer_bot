# Stepper controller gain-scale and balance-chain audit

Status: historical verified simulator result, 2026-08-12. The shared default
was subsequently promoted to the rounded StepperPhaseElectrical profile from
the later controller-weight tuning report; this document retains the earlier
gain-scale audit evidence.

Audience: controller developers and maintainers deciding whether a simulator
gain is comparable with a historical hardware gain. The question is whether
the remaining gain discrepancy survives an end-to-end audit from IMU state to
motor torque and balance-body acceleration.

## Executive result

The 16000-SPS migration is complete and uses one shared limit. The timestamped
wave path carries 40 pulses per 2.5 ms frame at the new ceiling; its scheduler
does not depend on the 400 Hz controller clock. The focused scheduler test
passes with the first and last pulse at 31 and 2469 microseconds.

The controller chain has no hidden degree/radian, degree/second, or timestep
scale. Its physical input is radians and radians/second all the way through
the notch/filter path and into the state-feedback expression. The end-to-end
simulator test now locks this down.

The retained hardware archive does not contain a session identified as
`6000/350` with enough provenance to replay it. The clean 400 Hz retained
capture instead obeys an effective `17920/800` state-feedback law to about
`1e-5 SPS` RMS. Applying `6000/350` to that same recorded state has a
`141.8 SPS` RMS residual. The 50 Hz fixtures are explicitly marked
`controller_fitting_allowed=false`, and their raw source is unavailable.
Therefore the historical command-reproduction question is unresolved for a
known `6000/350` session; it is not evidence of a current unit-conversion bug.

The independently derived balance equations agree with the simulator. The
measured `J = 0.0045 kg m^2` provenance is a passive physical-pendulum
measurement with the wheels held against motor-rotor resistance, so it is a
body pitch inertia about the axle, not a wheel-spin coordinate. The explicit
wheel/rotor inertia is consequently retained once in absolute wheel rotation.

With the verified 1/32 geometry and 16000-SPS authority, a compact search found
a symmetric local ±1 degree region from approximately
`160000/8000` through `300000/16000`. The selected simulator-only point is
`280000/12000`: it gives quiet symmetric recovery through ±4 degrees in the
focused run. ±6 and ±8 degrees eventually hit the 16000-SPS rail and fall in
the 10-second frontier run. The previously informative `180000/8000` point
remains a lower-feedback ±1-degree candidate, but the selected point gives a
materially better ±4-degree frontier.

After the fourfold raw-SPS conversion, the selected point is equivalent to
`70000/3000` when expressed on a 1/8 raw-step scale. Relative to historical
`6000/350 @ 1/8`, that is `11.67x` pitch feedback and `8.57x` rate feedback.
The lower `180000/8000` point is `7.50x` and `5.71x`, respectively. This
remaining ratio is therefore a real closed-loop/topology requirement of the
corrected moving-field plant in this simulator, not an unexplained unit
factor. It should still be treated as a simulator-to-hardware hypothesis until
a known-gain hardware capture is retained.

At the time of this historical audit, the current `pid.conf` gains remained
`6000/350`; only the authorized balance command ceiling had been migrated to
16000. The later controller-weight tuning report promoted the rounded
StepperPhaseElectrical profile to `pid.conf`; the audit's selected
`280000/12000` point remains historical evidence and was not applied to
hardware by that report.

## 1. 16000-SPS migration and scheduler authority

`Config::max_step_rate_sps` in `src/services/main/config.h` is the central
`16000.0` constant. It feeds:

- `RateControllerCore`'s controller rail;
- PID numeric validation;
- `DualWave::kMaxScheduledHz` and target clamping;
- the checked-in `pid.conf` balance limit;
- the telemetry dashboard validation limit; and
- the simulator/tuner baseline.

The `DualWave` frame remains 2500 microseconds with a 2 microsecond minimum
pulse. At 16000 SPS, `kMaxN` is 40 pulses per channel per frame. The timestamp
test observes 40 merged same-direction events, strictly increasing timestamps,
and the expected 31–2469 microsecond span. The motor runner still queues and
consumes timestamped frames independently of controller samples; no runtime
source path retains an 8000-SPS or 12000-SPS scheduler rail. Existing lower
rates in tests are intentional coverage points.

The relevant permanent checks are:

```text
SimulatorSchedulerTest.VerifiedOneThirtySecondAuthorityFitsTimestampedWaveFrame
SimulatorClockIsolationTest.SameStepEventStreamIsIndependentOfPollingBoundaries
```

This verifies software timing and array capacity. It does not claim a
pigpio/driver electrical waveform has been measured at 16000 SPS on hardware.

## 2. Exact controller units and signs

The path is:

```text
ImuRawPayload.acc/gyr                 SI m/s^2 and rad/s
  -> ImuPitchEstimator                 angle rad, gyro rad/s, accel rad/s^2
  -> ControlService / ImuSample         no numeric conversion
  -> rate notch and low-pass filters    filtering only; units unchanged
  -> RateControllerCore                 state feedback in SPS units
```

The implemented expression is:

```text
u_unclamped = K_pitch * (pitch_rad - pitch_target_rad)
            + K_rate  * pitch_rate_rad_s
            + K_accel * pitch_accel_rad_s2
```

Thus the configured units are `SPS/rad`, `SPS/(rad/s)`, and
`SPS/(rad/s^2)`. There is no multiplication or division by the controller
period. The rate notch changes phase and amplitude as a filter, not units.
The telemetry field named `pitch_error_deg` is the opposite display
convention, `pitch_target - pitch`; consequently
`pitch_feedback_sps = -K_pitch * pitch_error_deg * pi/180`.

`RateControllerCoreStateFeedbackTest.GainsConsumeRadiansAndRadiansPerSecondWithoutDtScale`
checks both a 400 Hz and a 200 Hz call with the same physical state. The
end-to-end `SimulatorReferenceTest.EndToEndStateScalingPreservesRadiansAtControllerBoundary`
checks the sensor, estimator, telemetry, and controller boundary together.

## 3. Historical command reproduction

The most useful retained data is
`data/hardware_sessions/20260719_wood_floor_neutral/steady_state_10s_400hz.csv`.
Using its `fused_pitch_deg`, `pitch_sp_deg`, `filtered_pitch_rate_dps`, and
`u_sps` fields, the direct reconstruction is:

```text
u = K_pitch * (fused_pitch_deg - pitch_sp_deg) * pi/180
  + K_rate * filtered_pitch_rate_dps * pi/180
```

The least-squares result over all 4001 retained rows is:

| model | pitch gain | rate gain | bias | RMS residual |
| --- | ---: | ---: | ---: | ---: |
| fitted to retained capture | 17920.0003 | 800.000013 | ~0 | `1.0e-5 SPS` |
| current historical claim | 6000 | 350 | — | `141.8 SPS` |

Representative rows, with command in SPS, are:

| sample | pitch error | rate | measured | reconstructed | residual |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | -0.634829 deg | -0.268592 deg/s | -202.301383 | -202.301380 | -0.000004 |
| 87 | 0.281473 deg | 5.031441 deg/s | 158.286572 | 158.286571 | 0.000001 |
| 181 | -0.419494 deg | -6.946299 deg/s | -228.190827 | -228.190822 | -0.000006 |
| 267 | 0.229020 deg | 5.860698 deg/s | 153.459907 | 153.459908 | -0.000001 |

The archive README binds this run to an old PID digest, not the current
working-tree `pid.conf`; it does not identify that digest as `6000/350`.
The 50 Hz noisy/runaway fixtures lack the source capture and are explicitly
not controller-fitting data. A future hardware test must retain the full PID
snapshot and raw 400 Hz telemetry if it is to answer this question directly.

## 4. Verified 1/8 to 1/32 field-motion scaling

For `n` commanded steps per revolution,

```text
omega_field_mechanical = SPS * 2*pi/n
v_field_wheel          = SPS * 2*pi*r/n.
```

The central geometry gives `1600 STEP/rev` at 1/8 and `6400 STEP/rev` at
1/32. Therefore preserving field motion is exactly a fourfold raw-SPS change:

```text
6000/350 @ 1/8  == 24000/1400 @ 1/32
```

The permanent geometry test uses the production wheel radius and checks both
field angular speed and wheel-equivalent speed. At 1/32, one SPS is
`0.0009817477 rad/s` of mechanical field motion and `40.4480 um/s` of wheel
equivalent motion.

## 5. Independent balance derivation

Use `x` for forward chassis translation and `q` for body pitch. Under no slip,
the absolute wheel/rotor angle is `x/r`; the magnetic relative angle is
`x/r - q`. The latter is not the physical wheel/rotor kinetic coordinate.

With `M = 1.032 kg`, `H = 0.06192 kg m`, `J = 0.0045 kg m^2`, and
`I_w = 3.24e-5 kg m^2` per side, the constrained kinetic energy is:

```text
T = 1/2 (M + 2*I_w/r^2) xdot^2
  + H*cos(q) xdot*qdot
  + 1/2 J qdot^2.
```

Therefore:

```text
D = [ M + 2*I_w/r^2,  H*cos(q) ]
    [ H*cos(q),        J         ].
```

At the upright point the independently calculated matrix is:

```text
D = [1.0701751343, 0.0619200000]
    [0.0619200000, 0.0045000000] kg-based units
det(D) = 0.0009817017044.
```

For total motor torque `T_m = T_left + T_right`, virtual work gives:

```text
Q_x =  T_m/r + F_external
Q_q = -T_m + F_external * (H/M) * cos(q).
```

The zero-rate equations used by the independent invariant are:

```text
rhs_x = T_m/r + F_external
rhs_q = g*H*sin(q) - T_m + external_pitch_moment
[xddot, qddot]^T = D^-1 [rhs_x, rhs_q]^T.
```

The implementation and independent calculation agree for both signs. The
focused results are:

| test | positive result | negative result |
| --- | ---: | ---: |
| gravity-only at ±0.1 deg, `xddot` | -0.0668687 / +0.0668687 m/s² | mirrored |
| gravity-only at ±0.1 deg, `qddot` | +1.15571 / -1.15571 rad/s² | mirrored |
| torque ±0.020 Nm/motor, `xddot` | +6.97325 / -6.97325 m/s² | mirrored |
| torque ±0.020 Nm/motor, `qddot` | -104.841 / +104.841 rad/s² | mirrored |
| torque ±0.100 Nm/motor, `xddot` | +34.8662 / -34.8662 m/s² | mirrored |
| torque ±0.100 Nm/motor, `qddot` | -524.203 / +524.203 rad/s² | mirrored |

The direct-torque and Stepper actuator paths also agree in instantaneous
acceleration. No additional wheel force, body reaction, or two-motor factor is
applied in a second path.

## 6. Quasi-static lean authority

For a freely rolling plant, `qddot = 0` does not imply `xddot = 0`: a wheel
torque can accelerate the chassis while holding body pitch. Solving the two
equations gives:

```text
T_required = g*H*sin(q) /
             (1 + D12/(r*D11)).
```

The test applies half this total torque to each motor and verifies the pitch
angle remains fixed over the short numerical run:

| lean | total torque | total rim force | per-motor torque | available total torque |
| ---: | ---: | ---: | ---: | ---: |
| 0.5 deg | 0.00220471 Nm | 0.0535124 N | 0.00110236 Nm | 0.451841 Nm |
| 1 deg | 0.00440955 Nm | 0.107028 N | 0.00220477 Nm | 0.451841 Nm |
| 2 deg | 0.00882011 Nm | 0.214080 N | 0.00441005 Nm | 0.451841 Nm |
| 4 deg | 0.0176483 Nm | 0.428357 N | 0.00882415 Nm | 0.451841 Nm |

The 1.065 A vector-current limit gives `0.225921 Nm` peak modeled torque per
motor, or `0.451841 Nm` total. These low-speed equilibrium numbers are well
within the StepperPhaseElectrical capability and are not a static holding
torque problem.

## 7. `J` provenance and inertia accounting

`data/hardware_sessions/20260722_pitch_inertia/README.md` identifies the
experiment as a passive physical-pendulum measurement with the controller and
motor runner absent. The robot was supported through the wheel-axis line and
the wheels were held against motor-rotor resistance. The two measured periods
produce `J = 0.004551` and `0.004518 kg m^2`, rounded to `0.0045`.

This supports interpreting `J` as pitch inertia about the axle for the
body/chassis pendulum configuration. The wheels did not execute free wheel
spin in that experiment. The documentation does not identify every mass
component of the pendulum to metrology precision, so it cannot prove whether
the wheel/hub masses were included in the aggregate body inertia inventory.
That uncertainty is recorded rather than fitted away. In the current
coordinate model, explicit wheel/rotor inertia enters once through
`2*I_w/r^2` in `D11`; it is not added to `D12` or `D22` through the relative
phase coordinate. No correction to `J` is justified by the retained evidence.

## 8. Estimator/controller scaling audit

The simulator produces raw accelerometer specific force and gyro rate in SI
units. `ImuPitchEstimator` solves the signed full-circle gravity pitch with
`atan2(-acc_x, -acc_z)`, fuses the gyro rate in rad/s, and emits `ImuSample`
in radians/radians-per-second. The production notch and low-pass filters do
not change those units. `ControlService` forwards the sample without a
conversion. The simulator timeline converts to degrees only for human-facing
telemetry.

The end-to-end test checks that fused and controller pitch agree, filtered and
controller rate agree, and that telemetry feedback reconstructs the raw
radian-domain expression within the expected float serialization tolerance.
It also confirms a nonzero simulated gyro rate reaches the controller. No
duplicate conversion or hidden sample-period factor was found.

## 9. Focused tune and pulse authority

The search neutralized velocity damping/I, drive, turn, COM adaptation, and
pitch acceleration feedback. It retained the existing estimator and rate
notch. The tested local ±1 degree region was:

```text
K_pitch = 160000 .. 300000 SPS/rad
K_rate  =   8000 ..  16000 SPS/(rad/s)
```

Historical `6000/350`, `12000/700`, and `24000/1400` all fall from symmetric
±1 degree releases in the corrected electrical plant. The selected point for
the compact frontier is `280000/12000`.

### Selected ±1-degree telemetry

| quantity | positive release |
| --- | ---: |
| peak pitch | 1.00316 deg |
| final pitch at 3 s | -0.06345 deg |
| tail RMS | 0.40865 deg |
| requested peak / unclamped peak | 13314.7 / 13314.7 SPS |
| requested p95 / p99 | 11677.8 / 12346.3 SPS |
| emitted peak | 3926.8 SPS |
| emitted p95 / p99 | 3085.0 / 3585.0 SPS |
| completed-step p95 / p99 | 3200 / 3600 SPS |
| time at command saturation | 0 s |
| voltage saturation | 0 s |
| phase-error peak | 0.26457 rad |
| torque peak per motor | 0.05908 Nm |
| field/mechanical wheel-velocity mismatch peak | 0.1861 m/s |
| first torque reversal | 22.5 ms |

The positive and negative ±1 degree traces have equal peak and mirrored final
pitch to test precision.

The same selected configuration also has a two-second exact quiet-balance run:
the plant remains at `0 deg` with `0 SPS` requested and no fault or saturation.

### Compact recovery frontier

The ±1/±2/±4 rows use a 3-second run. The ±6/±8 rows use 10 seconds so a
bounded short transient is not mislabeled as recovery.

| release | peak pitch | tail RMS | requested peak | emitted peak | saturation | outcome |
| ---: | ---: | ---: | ---: | ---: | ---: | --- |
| ±1 deg | 1.003 deg | 0.409 deg | 13315 SPS | 3927 SPS | 0 s | quiet recovery |
| ±2 deg | 2.009 deg | 0.433 deg | 13317 SPS | 4062 SPS | 0 s | quiet recovery |
| ±4 deg | 4.020 deg | 0.390 deg | 16000 SPS | 5500 SPS | 0.0175 s | recovery |
| ±6 deg | 90 deg | 90 deg | 16000 SPS | 16000 SPS | 1.7275 s | fall by 10 s |
| ±8 deg | 90 deg | 90 deg | 16000 SPS | 16000 SPS | 1.7300 s | fall by 10 s |

The `8000` versus `16000` comparison with the same `280000/12000` gains is:

| release | verified 16000: fall / saturation | stale 8000: fall / saturation |
| ---: | --- | --- |
| ±4 deg, 3 s | no / 0.0175 s | no / 1.4075 s |
| ±6 deg, 10 s | yes / 1.7275 s | yes / 2.6250 s |

The new authority materially reduces clipping and makes the ±4 case less
rail-dominated, but it does not make the ±6 release robust. The ±6 frontier is
therefore not an artifact of leaving the old 8000-SPS cap in place.

## 10. Changes and remaining uncertainty

Changes made for this audit:

- introduced the central verified-1/32 `16000 SPS` rail and migrated
  `pid.conf`, validation, dashboard validation, simulator tuner, and wave
  scheduling to it;
- added controller unit/timestep, scheduler timestamp, geometry scaling,
  independent gravity/torque, quasi-static authority, and end-to-end estimator
  scaling tests;
- retained the historical `6000/350` DirectActuator reference in an explicit
  fixture; the later rounded StepperPhaseElectrical tune is now the shared
  `pid.conf` default;
- added the limit-comparison telemetry to the focused Stepper tune test; and
- updated the maintained actuator report and dashboard API documentation.

No controller-unit, estimator-scale, body mass-matrix, torque-factor, or
`J`-double-counting defect was found in this second audit. The earlier actuator
physics corrections remain the justified model changes.

Remaining uncertainties are driver current-loop dynamics, current-limit
calibration under load, bus sag, thermal resistance, tire/contact compliance,
structural modes, mechanical friction, and the absence of a retained
known-gain 400 Hz hardware capture. The electrical plant is suitable as the
primary controller-development plant for relative gain studies and causal
debugging. It is not yet sufficient by itself to authorize applying
`280000/12000` to hardware. The next hardware experiment should retain the
complete PID snapshot, verified step geometry, raw 400 Hz IMU/controller
telemetry, and independent fall protection.

## Verification commands

Focused checks:

```sh
cmake --build build -j2
./build/balancer_tests --gtest_filter='StepperPhaseElectricalTuningTest.*:SimulatorReferenceTest.EndToEndStateScalingPreservesRadiansAtControllerBoundary:StepperPhaseModelTest.*:SimulatorSchedulerTest.VerifiedOneThirtySecondAuthorityFitsTimestampedWaveFrame:RateControllerCoreStateFeedbackTest.GainsConsumeRadiansAndRadiansPerSecondWithoutDtScale'
```

Repository gate:

```sh
pytest --build
git diff --check
```
