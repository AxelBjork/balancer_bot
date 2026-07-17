# Simulator-to-Hardware Improvement Plan

This is the living design and integration record for making simulator results transfer more
reliably to the physical balancing robot. Update the checklists, decisions, test dispositions, and
artifact links in the same change that implements each phase.

## Goals

- Exercise the same timing, estimator, controller, and emitted-step feedback semantics in SIL and
  on the Raspberry Pi.
- Replace simulator stabilizing crutches with explicit actuator, traction, sensor, and timing
  behavior.
- Tune one controller against the hardware-nominal plant plus named one-at-a-time margins.
- Keep every completed phase buildable and testable with `pytest --build`.

## Non-goals

- Backward compatibility for PID files, reflected payload field names, generated Python bindings,
  or obsolete simulator entry points.
- Encoder, stall-detection, or ground-truth velocity feedback on the physical robot.
- New live Raspberry Pi UDP collection during this program.
- Claiming physical validation from simulation or cross-compilation alone.

## Authoritative Hardware-Nominal Plant

These existing simulator constants take precedence over estimates derived from the component bill
of materials and must not be retuned as part of this work.

| Parameter | Value |
| --- | ---: |
| Gravity | 9.81 m/s^2 |
| Wheel radius | 0.040 m |
| Robot mass | 1.032 kg |
| Wheel mass | 0.050 kg each |
| Cart mass | 0.100 kg |
| Body mass | 0.932 kg |
| COM height | 0.060 m |
| Body inertia about COM | 0.0034 kg m^2 |
| Motor resolution | 3,200 steps/revolution |

`wheel_mass` and `cart_mass` are effective plant-model values. They are not recomputed from the
motor or frame BOM.

## Data Flow

Before this program, the paths diverged:

```text
direct tests/fuzz -> RateControllerCore -> idealized plant
UDP scenarios     -> services/message bus -> different simulator wrapper
hardware          -> independent IMU, clock, input, and UDP threads -> wave motor runner
```

Implemented flow:

```text
deterministic event timeline
  -> raw IMU synthesis (833 Hz) -> production IMU filter
  -> velocity PI (50 Hz) + attitude/rate control (400 Hz)
  -> motor target -> 5 ms pulse scheduler -> completed-pulse feedback
  -> motor phase -> rotor/wheel state -> traction-limited tire force -> cart-pole
  -> next sensor sample

The UDP wrapper, direct runner, tuner, and fuzz targets all drive this same engine.
Hardware replaces only the deterministic sensor and pulse backends.
```

## Controller Design

All public motion quantities use robot-forward as positive. Per-wheel electrical inversion stays
inside the stepper boundary.

At 50 Hz:

```text
target_velocity_sps = deadzone_and_scale(joystick.forward, drive_max_sps)
velocity_error_sps  = target_velocity_sps - emitted_velocity_sps
velocity_P_rad      = deg_to_rad(velocity_P * velocity_error_sps)
velocity_I_rad     += deg_to_rad(velocity_I * velocity_error_sps * dt)
pitch_sp_rad        = clamp(velocity_P_rad + velocity_I_rad, +/-pitch_max)
```

At 400 Hz:

```text
rate_sp_rad_s = clamp(angle_P * (pitch_sp - pitch) - angle_D * pitch_rate,
                      +/-max_pitch_rate)
balance_correction_sps = -rate_pid(rate_sp, pitch_rate) * output_scale_sps
balance_sps   = clamp(target_velocity_sps + balance_correction_sps,
                      +/-balance_max_sps)
```

`velocity_P` and `velocity_I` are degrees per SPS and degrees per SPS-second respectively;
`angle_P` is inverse seconds and `angle_D` is dimensionless. Positive joystick, target velocity,
completed-pulse feedback, and plant travel are robot-forward. The target-velocity feed-forward is
required because the stepper boundary commands wheel speed rather than motor torque; the rate PID
is the balance correction around that kinematic target.

The velocity integrator freezes when pitch or actuator output saturates. Turn allocation consumes
only authority left after balancing. Fallover, stale/future IMU data, or an actuator fault clears
all integrators and outputs.

## Actuator and Plant Design

- The production scheduler transmits 5 ms one-shot pigpio waves with no more than one frame queued
  ahead. Per-wheel phase accumulators retain fractional pulses between frames.
- Applied steps and the 50 ms velocity estimate advance only when a frame completes.
- A reversal drains/stops pulses, forces STEP low, changes DIR, waits 50 us, resets directional
  phase, and resumes.
- Simulation integrates scheduled physical pulses into requested motor position while controller
  feedback advances only at frame completion. Motor phase and relative-velocity errors produce a
  force request. A separate rotor/wheel state transmits that force through a damped tire coupling
  capped by traction before it reaches the cart-pole.
- Phase error beyond the supported bound is retained as an explicit missed-step estimate rather
  than granting the plant unlimited force.

The motor authority envelope is
`F_motor_max(v) = F_stall * clamp(1 - abs(v) / v_no_load, 0, 1)`. The tire limit is
`F_traction = mu * m_robot * g`. Motor force follows its capped request through the configured
first-order lag. Tire stiffness/damping and wheel effective mass are explicit profile parameters,
not changes to the authoritative robot mass or geometry.

The nominal plant is evaluated with one-at-a-time margins: masses +/-10%, COM +/-15%, inertia
+/-20%, motor force -30%/+20%, motor lag 4-20 ms, traction 0.6-1.2, pitch damping 0-0.04 N m s/rad,
and cart drag 0.3-2.0 N s/m.

## IMU and Timing Design

- Hardware ticks use absolute `steady_clock` microseconds and measured elapsed `dt`.
- Deterministic runs use a zero-based timeline shared by ticks and simulated IMU samples.
- Accelerometer/gyro samples are paired by nearest timestamp with at most 2 ms skew.
- Controller input is rejected when older than 30 ms or more than 2 ms in the future.
- Simulated accelerometer specific force includes gravity, axle translation, tangential
  acceleration, and centripetal acceleration at the IMU location before bias, noise, quantization,
  latency, jitter, and loss are applied.

## Public Interface Changes

- `PhysicsTickPayload.sim_time_us` becomes `timestamp_us`.
- PID configuration requires `config_version = 2`; no legacy aliases are accepted.
- Outer-loop configuration becomes `velocity_P`, `velocity_I`, `velocity_I_limit_deg`, `angle_P`,
  `angle_D`, `drive_max_sps`, `turn_max_sps`, `pitch_max_deg`, `balance_max_sps`, and
  `output_scale_sps`, alongside the rate PID keys.
- `SimStartRunPayload.telemetry_stride` selects summary-only (`0`), full-rate (`1`), or every-Nth
  controller-tick telemetry. The request carries every retained `SimulatorPhysics` override and no
  longer contains `wheel_slip_factor`.
- Telemetry exposes target velocity, velocity P/I terms, pitch/rate setpoints and errors, explicit
  controller fault/saturation bitmasks, completed pulses, missed-step estimate, plant truth, sensor
  timing, seed, and every retained plant parameter.
- `SimRunDonePayload` reports exact all-tick peak/tail metrics, longest continuous command
  saturation, actuator-fault count, accumulated controller faults, and an all-tick timeline hash.
  Telemetry downsampling does not change these values.

## Implementation Checklist

| Phase | Work | Owner | Status | Acceptance gate | Tests/artifact evidence |
| --- | --- | --- | --- | --- | --- |
| 0 | Living plan and documentation index | Simulator transfer milestone | Complete | Plan is indexed before code changes | This document; baseline `pytest --build`: 60/60 C++, 26 Python passed, 1 pre-existing xfail |
| 1 | Clock domain, sample pairing, and bus serialization | Runtime | Complete | Runtime tests and `pytest --build` pass | 63/63 C++, 26 Python passed; 1 pre-existing xfail |
| 2 | Short-frame pulse scheduler and completed-pulse feedback | Motor runtime | Complete | 1-12,000 SPS scheduler tests and build pass | 69/69 C++, 24 Python passed; temporary Phase-3 xfails recorded below |
| 3 | Unified event engine, physical actuator, and IMU model | Simulation | Complete and verified | Direct/UDP equivalence and parameter A/B tests pass | One `SimulatorEngine` backs direct, UDP, tuning, and fuzz paths; exact all-tick hashes and retained-parameter A/B tests pass |
| 4 | Controller v2 and breaking PID schema | Controls | Complete and verified | Control/config/safety-observability tests pass | Strict v2 parser; equation/sign/cadence/reset/allocation tests; safe telemetry for missing/stale/future/fallover/actuator states; matching PID files |
| 5 | Telemetry, tuning, artifacts, fuzz, and cross-build | Integration | Complete | All final gates pass | `pytest --build`: 76/76 C++ and 15/15 Python; 20/20 full-rate transfer scenarios; 14/14 fuzz seeds; Pi cross-build; [transfer summary](../../build/sim/transfer_summary.md) |

Dependencies are strictly ordered. A phase may introduce scaffolding needed by the next phase, but
must not claim later behavior as complete.

### Recovery Rebaseline

The two WIP commits were rebaselined before final verification: phases 1-2 were recorded complete,
phases 3-4 were treated as implemented but under verification, and phase 5 remained pending. The
complete statuses above were applied only after the layered host gate, staged tuning record,
full-rate transfer artifacts, fuzz smoke, and Pi cross-build all regenerated successfully.

## Scenario Acceptance Gates

Nominal runs require no fallover, stale-data event, or actuator fault; peak pitch below 15 degrees;
final two-second pitch RMS below 1 degree; continuous saturation below 250 ms; neutral-hold travel
below 0.35 m; recovery below 2 degrees within two seconds of a 3 N, 100 ms push; and an 800 SPS
command that reaches within 30% of target plant speed then stops below 0.05 m/s within two seconds.

Margin runs may loosen performance thresholds by 25%, but retain the 15-degree safety limit and
zero-fallover requirement.

## Test Disposition

| Test area | Final disposition | Replacement evidence |
| --- | --- | --- |
| Runtime safety, units, signs, and timing | Retained and extended | Clock-domain, IMU skew/age, nested bus, controller, and scheduler tests |
| Tests integrating requested Hz as real steps | Rewritten | Completed-frame and in-frame physical-pulse accounting tests |
| Direct simulator-only behavior | Rewritten | Unified-engine deterministic and UDP scenario coverage |
| Exact values caused by artificial damping/force | Removed/replaced | Force/traction/phase invariants and acceptance scenarios |
| Existing large-angle frontier xfail | Removed | Replaced by the 15-degree safety gate and deterministic fallover reset tests |
| Positive/negative COM drift thresholds | Removed after Phase 2 xfail | Replaced by COM sign symmetry and +/-15% OAT acceptance tests |

Any temporary `xfail` must name its implementing phase and removal condition here. Xfailed tests do
not satisfy a phase gate, and none introduced by this program may remain after Phase 5.

## Decision Log

| Decision | Rationale |
| --- | --- |
| Break legacy interfaces freely | Correct new behavior is more important than compatibility |
| Preserve current physical constants | They are the best available measurements for this robot |
| Retain pigpio waves | Keeps arbitrary-pin support and the existing hardware boundary |
| Validate nominal plus one-at-a-time margins | Balances iteration speed with transfer robustness |
| Defer new Pi UDP capture | Robust SIL iteration is the current priority |
| Treat emitted pulses as estimates, not real motion | There are no wheel encoders |
| Separate motor phase from tire/axle displacement | Tire slip must not be reported as a missed motor step |
| Feed target speed forward at the actuator boundary | A velocity actuator otherwise requires permanent attitude error to sustain travel |

## Known Uncertainties and Hardware Follow-up

- Running torque versus wheel speed and battery voltage.
- Tire traction on the intended surface.
- Actual missed-step behavior under balance transients.
- IMU mounting position, mechanical vibration, and runtime scheduling jitter.
- Physical confirmation of positive motion and pitch signs.

A later restrained hardware session should validate those items before gains are described as
hardware-proven.

## Artifact Record

- Baseline build: `pytest --build` passed 60/60 C++ tests and 26 Python tests; the existing
  large-angle simulator frontier remained the single expected xfail.
- Baseline simulator artifacts: `build/sim/` (generated, not committed).
- Final comparison and tuning summary: [build/sim/transfer_summary.md](../../build/sim/transfer_summary.md).
- Run-specific full-rate evidence is under `build/sim/transfer/<timestamp>_<commit>/`; the stable
  summary links to every scenario's current `summary.json`.
- Tuning ran in the required order: inner rate/attitude basin, outer velocity regulation, then
  local joint refinement in `balancer_simulator_tuner`. The resulting candidate was admitted only
  after the shared nominal-plus-16-margin evaluator passed; each stage records its grid, ranked
  results, selected candidate, and candidate PID file. No aggregate score can override a fall,
  actuator fault, invalid controller timing, or 15-degree safety violation.

## Final Verification

- `pytest --build`: 76 C++ tests and 15 Python tests passed; no skips or xfails. The full transfer
  matrix now takes about 0.5 seconds in-process; final CTest and Python phases took 1.65 and 3.99
  seconds respectively, down from about 70.6 seconds for Python. The complete warm incremental gate
  took 16.3 seconds in this environment.
- Nominal and one-at-a-time margins: 20/20 passed with no fallovers or actuator faults.
- Fuzz smoke: all three harnesses built with the host compiler and all 14 generated corpus seeds
  executed successfully. AFL++ instrumentation was unavailable in this environment, so the smoke
  gate validates harness/corpus execution rather than a timed fuzz campaign.
- Raspberry Pi: `./build_cmake OFF` produced `build-pi/balancer_pi` and `build-pi/imu_demo`.
- No robot deployment, motor operation, or new Raspberry Pi data capture was performed.
