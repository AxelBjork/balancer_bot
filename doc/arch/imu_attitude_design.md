# IMU Conditioning and Pitch Attitude Design

Status: implemented in software; hardware rollout requires validation

## Purpose

This design defines a deliberately small IMU path for chassis pitch. It separates
sensor conditioning from attitude estimation and keeps all physical center-of-mass
(COM) compensation in the controller.

The design aims to:

- provide low-noise pitch, pitch-rate, and pitch-acceleration signals at the control rate;
- use the fixed sensor mounting as a configuration fact;
- avoid learned mounting angles, gyro bias learning, slow gravity recovery, and hidden
  COM trim;
- keep the IMU independent of motor feedback;
- run the same production signal path in hardware and simulation; and
- make latency, invalid-data handling, and estimator limitations explicit.

The software implementation follows this design. The stated cutoff, lever-arm, and
notch choices remain subject to the hardware validation below.

The focused estimator, service, sensor-model, and realistic closed-loop simulator
scenarios pass. Hardware balance rollout still requires the restrained validation
described below.

## Coordinate Convention

The IIO reader is the sensor-to-robot boundary. It is responsible only for:

- reading and synchronizing accelerometer and gyroscope samples;
- scaling raw values to SI units;
- applying the fixed axis permutation and inversion for the installed sensor; and
- publishing robot-axis vectors with a monotonic timestamp.

With the configured mounting:

- gravity is on robot `-Z` while upright;
- robot pitch rate is the mapped gyroscope `Y` component; and
- signed full-circle gravity pitch is `atan2(-ax, -az)`.

There is no residual software mounting offset. If the physical mounting or axis map
changes, the fixed mapping must be changed and verified rather than learned online.

## Signal Path

At the current 833 Hz IMU output data rate, the proposed path is:

```text
ISM330DHCX IIO accelerometer ─┐
                             ├─ scale, synchronize, fixed axis map
ISM330DHCX IIO gyroscope ────┘

mapped acceleration ── 2-pole LPF ── optional lever-arm correction ── atan2 ── gravity pitch
                                                                            │
mapped pitch rate ── optional notch ── 2-pole LPF ── integrate one dt ──────┼─ bounded correction ── pitch

                                      ├──────────────────────────────── pitch rate
                                      └─ derivative ── LPF ─────────── pitch acceleration
```

The production filter configuration is:

| Signal | Filter | Initial cutoff |
| --- | --- | ---: |
| Accelerometer | PX4 two-pole low-pass | 15 Hz |
| Pitch gyro rate | PX4 two-pole low-pass | 30 Hz |
| Pitch gyro derivative | Derivative followed by low-pass | 10 Hz |
| Gravity attitude correction | Circular complementary correction | 0.5 Hz, ±2.5° innovation |
| Motor-vibration notch | Disabled until measured | N/A |

The fixed notch is bypassed, and the 70 mm lever-arm correction is compiled off by
default. The accelerometer cutoff rejects the broad 75–177 Hz stepper-vibration band seen in
hardware captures. Gyro prediction carries dynamic pitch, while the bounded gravity path
limits abrupt apparent-angle changes caused by translation. These are signal-conditioning
settings, not PID tuning parameters.
Changes must be supported by measured noise and phase-delay results.

The repository already vendors suitable PX4 primitives:

- [`LowPassFilter2p.hpp`](../../src/px4_stub/src/lib/mathlib/math/filter/LowPassFilter2p.hpp)
- [`NotchFilter.hpp`](../../src/px4_stub/src/lib/mathlib/math/filter/NotchFilter.hpp)
- [`FilteredDerivative.hpp`](../../src/px4_stub/src/lib/mathlib/math/filter/FilteredDerivative.hpp)

The wrappers used by the IMU service should expose sample rate and cutoff explicitly,
reset deterministically, and contain no attitude-specific recovery modes.

## Pitch Estimate

Conditioned acceleration supplies a full-circle gravity reference:

```text
gravity_pitch = atan2(-ax_filtered, -az_filtered)
```

Each normal sample predicts pitch from the filtered gyro and actual timestamp delta.
The wrapped gravity innovation is clamped symmetrically to ±2.5 degrees and applied
with a fixed 0.5 Hz correction:

```text
predicted_pitch = wrap(pitch + pitch_rate_filtered * dt)
innovation = wrap(gravity_pitch - predicted_pitch)
bounded_innovation = clamp(innovation, -2.5°, +2.5°)
pitch = wrap(predicted_pitch + (1 - exp(-2π * 0.5 Hz * dt)) * bounded_innovation)
```

There is no dwell, ramp, adaptive recovery mode, learned bias, mounting correction,
or motor input. The bound limits how quickly translational acceleration can steer
attitude, while the fixed correction prevents unbounded gyro drift.

The IMU is approximately 70 mm from the axle. If testing shows that rotational
lever-arm acceleration is material, it may be removed using IMU-only measurements:

```text
A = ax_filtered - 0.070 * pitch_acceleration_filtered
B = az_filtered + 0.070 * pitch_rate_filtered²

pitch = atan2(-A, -B)
```

This correction must be independently switchable for validation. It must not depend
on motor position, motor speed, completed steps, controller state, or an estimated
pitch. If it does not improve restrained hardware measurements across both pitch
directions, it should be removed.

The accelerometer magnitude is a diagnostic and validity input, not a source for a
slow recovery state. The production path accepts pitch-plane specific force from
`0.1g` through `3.5g`; non-finite or out-of-range input invalidates and resets the
sample. Ordinary dynamic acceleration inside that range remains observable in
telemetry rather than being reinterpreted as a missed rotation.

## Filter State Is Not Calibration State

The low-pass and complementary filters retain bounded signal-processing state. This
is different from learning a calibration offset:

- the filters have unity DC gain;
- their state is bounded by the recent input;
- gyro angle state is corrected continuously rather than allowed to drift freely;
- they do not estimate gyro bias or mounting orientation;
- they do not alter COM trim; and
- old startup input decays according to the configured cutoff rather than permanently
  changing the estimate.

On the first valid sample, the accelerometer and gyro filters are reset to that sample,
while the fused pitch state starts at zero. Duplicate or backward timestamps and
non-finite input invalidate and reset the path. A gap longer than four nominal sample
periods reseeds from the current sample with zero derivative and a zero pitch state.
Invalid input is never fed through the filter as a numeric value.

## Bounded Gyro/Gravity Fusion

PX4 commonly conditions gyro and accelerometer data before an attitude estimator. Its
rate-control path uses filtered angular velocity and a separately filtered derivative.
That part is useful here.

A PX4-style attitude estimator, gyro-bias learner, or EKF remains outside this design.
The estimator carries gyro angle between samples and applies only the fixed bounded
gravity correction above. A constant gyro bias therefore creates a bounded
steady-state error rather than monotonic drift. No estimated bias or mounting state
is retained.

## Fundamental Observability Limit

An accelerometer measures specific force, not gravity alone. A pure IMU cannot
algebraically distinguish chassis tilt from sustained horizontal vehicle acceleration.
For example, forward acceleration changes `ax` and therefore changes gravity-derived
pitch even if the chassis angle is fixed.

Low-pass filtering and bounded innovation can reduce the dynamic effect of vibration
and translation. They cannot solve the DC ambiguity.

Consequently:

- translational acceleration may produce an apparent pitch error;
- tests must not expect exact chassis pitch during arbitrary translation from this
  estimator;
- the simulator must preserve translational specific force rather than hiding it; and
- persistent low-frequency translation can still require an independent reference.

Possible future choices, each requiring a separate design decision, are:

1. accept the bounded apparent tilt during translation; or
2. add an independent vehicle-motion or attitude reference.

Motor feedback must not be routed into the IMU service to conceal this ambiguity.

## Motor Vibration and Notch Filtering

A notch filter should not be enabled merely because motor vibration is suspected.
Stepper excitation can move with step rate and can contain several harmonics, so a
fixed notch may help at one speed and do little at another.

Before adding a notch:

1. capture controller-disabled IMU data with the robot restrained;
2. measure spectra with motors de-energized and at several fixed positive and negative
   step rates;
3. identify a narrow, repeatable peak that remains inside a useful notch range; and
4. measure the resulting attenuation and phase effect on the pitch-control band.

The first implementation should support one disabled-by-default fixed notch. Dynamic
notch tracking, FFT-based adaptation, and motor-command coupling are out of scope.
Mechanical vibration reduction remains preferable when practical.

## Service and Controller Boundary

`Ism330IioReader` publishes synchronized, scaled, robot-axis sensor samples and their
timestamps. It does not compute pitch.

`ImuService` owns sensor filtering, bounded complementary pitch, and IMU validity. It
does not subscribe to motor feedback.

The controller consumes:

- bounded complementary pitch;
- filtered pitch gyro rate;
- filtered pitch gyro derivative; and
- sample validity and timestamp.

COM trim remains exclusively in the controller. This design does not change the PID
schema, motor-SPS feedback convention, controller structure, or public telemetry wire
schema.

Non-finite or invalid current IMU data clears the controller's usable IMU sample
immediately so the existing no-IMU safe-zero behavior applies on the next control
tick.

## Simulator Contract

The simulator must exercise the production `ImuService` path rather than implement a
second estimator. Its virtual sensor should publish samples at the configured IMU
rate and use the same axes, units, timestamps, filter configuration, and reset rules
as hardware.

`imu_pitch_lag_s` represents only an explicitly requested additional transport delay.
Nominal simulator profiles add no delay on top of the production filters.

The simulated accelerometer must include:

- gravity in the mapped robot frame;
- vehicle translational specific force;
- the configured IMU lever-arm acceleration;
- representative noise and quantization; and
- optional measured vibration injected before production filtering.

The simulated gyroscope must include angular rate, representative noise, and any
explicit test bias. A constant bias may affect reported rate and create a bounded
pitch offset, but it must not create monotonic pitch drift.

Simulator assertions should distinguish true chassis pitch from apparent
accelerometer pitch. A translation test should verify the expected ambiguity rather
than require an impossible exact attitude reconstruction.

## Validation

### Unit and integration tests

Tests should cover:

- static upright and symmetric positive/negative pitch;
- full-circle output and crossing `-pi`/`pi`;
- deterministic initialization and reset with no persistent startup offset;
- filter DC gain, step response, attenuation, and finite output;
- gyro-rate and derivative filtering in both directions;
- optional lever-arm correction in both directions;
- non-finite input and timestamp discontinuities;
- constant gyro bias with bounded, non-growing pitch error;
- motor-vibration injection before the filters; and
- translational acceleration changing apparent pitch without introducing learned
  state or history dependence.

Frequency-response checks should include representative points below, near, and above
each cutoff. End-to-end simulator tests should use the same production filter code.

### Hardware rollout

Hardware validation proceeds without simultaneous PID retuning:

1. With actuation disabled, record upright and restrained positive/negative pitch
   against an independent physical angle reference.
2. Repeat slow restrained rotations and compare pitch, gyro rate, and derivative for
   symmetry, lag, and noise.
3. With the robot restrained, record motors de-energized and at multiple fixed step
   rates to determine whether a notch is justified.
4. Compare lever-arm correction enabled and disabled using the same maneuvers.
5. Run restrained balance and return tests while monitoring apparent pitch, gyro
   signals, COM trim, faults, and motor commands.
6. Only after those checks, run a short neutral balance before longer operation.

Static mean error must be at most 0.5 degrees at upright and restrained positive and
negative 10 degrees. Slow bidirectional rotations must remain within 2 degrees dynamic
RMS with measured 5 Hz pitch delay below 18 ms. Dynamic interpretation must account for
the accelerometer's translation ambiguity.

## Non-Goals

This design does not include:

- online mounting-angle or gyro-bias learning;
- gyro-bias learning or adaptive gravity-recovery modes;
- motor feedback in the IMU service;
- estimator-owned COM trim;
- a PX4 attitude estimator, EKF, or dynamic notch;
- removal of real translational acceleration from simulation; or
- opportunistic controller or PID retuning.

## Open Validation Decisions

The following choices remain hardware-validation decisions:

- whether the disabled 70 mm lever-arm correction provides a measurable improvement;
- whether motor vibration contains a stable narrow peak suitable for a fixed notch;
  and
- whether bounded gyro/gravity pitch is adequate during the required vehicle translation.

The last item is an architecture boundary rather than a cutoff-tuning problem.
