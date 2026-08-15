# StepperPhaseElectrical first-principles audit

Status: corrected simulator baseline; controller gain-scale follow-up completed
2026-08-12. See the [gain-scale and balance-chain follow-up](stepper_gain_scale_audit.md)
for the current 16000-SPS migration, historical telemetry audit, and tune.

Audience: future controller developers and maintainers who need to decide
whether the electrical StepperPhase plant is a useful development authority.
Question answered: are the field, rotor, electrical, mechanical, and controller
interfaces internally consistent, and do the constants reproduce the retained
free-wheel hardware mode?

The implementation is in
`tests/simulator/stepper_phase_actuator.cpp` and
`tests/simulator/balancer_simulator.cpp`. Permanent checks are in
`tests/stepper_phase_actuator_test.cpp`.

## Executive result

The audit found two material model defects and one stale test assumption.

1. `0.45/1.5 = 0.30 N m/A` treated a two-phase holding-torque datum as the
   norm of the simulator's two-winding current vector. The corrected vector
   constant is `0.45/(sqrt(2)*1.5) = 0.212132 N m/A`.
2. The constrained mass matrix put attached wheel/rotor inertia into the
   rotor/stator-relative coordinate. The physical rotor speed is absolute
   wheel speed `xdot/r`; the relative coordinate is magnetic phase only.
3. A torque-authority test still used 1/16-era electrical step angles. It now
   derives one-, two-, and four-step angles from the verified 1/32 geometry.

After correction, the one-stator/one-wheel fixture predicts 93.9746 Hz,
while ideal-current and electrical numerical ringdowns both give about
93.74 Hz. The retained hardware mode is 93–95 Hz, with provisional damping
ratio 0.04–0.08; the model gives 0.0706. The ideal and electrical paths pass
the same mechanical signs, symmetry, timestep, and energy checks.

The corrected electrical plant is therefore defensible as the primary
first-order plant for inner-loop controller development. It is not a final
hardware-gain authority: the driver is averaged, and the retained fixture does
not identify absolute torque, structural modes, tire behavior, bus sag, or the
actual high-speed current regulator.

## Evidence categories

Verified facts are direct results of the source and tests:

- `200` full steps/revolution, verified `1/32`, `6400 STEP/rev`, wheel radius
  `0.0412 m`, `R=2.3 ohm`, `L=4.4 mH`, bus `11.1 V`, current setting `1.065 A`.
- The current C++ focused run selected 57 tests and passed all 57; the full
  repository gate reports 92 Python tests passed and one expected failure.
- The fixed-field analytical and numerical values are listed above and below.
- The permanent tune test reports the historical gains, a local stable region,
  the selected preliminary point, pulse rates, saturation, and the recovery
  frontier.

Inferences are conclusions supported by more than one check:

- The old 111.6 Hz free-wheel prediction was primarily a current-vector
  normalization error, not an electrical R/L or body-coupling effect.
- Early controlled torque reversal is a valid underdamped phase response plus
  moving-field interaction. The fixed-field energy test does not show an
  actuator energy source; the old controller simply did not close the coupled
  plant robustly.

Unresolved questions remain about the driver current interpretation under
load, actual winding-current waveforms, structural/tire compliance, and
hardware recovery with the preliminary controller.

## 1. Coordinates and STEP-to-field path

The coordinate conventions are:

| Symbol/state | Reference frame and meaning |
| --- | --- |
| `x` | chassis translation, positive robot-forward, measured in metres |
| `theta` | chassis pitch about the axle |
| `x/r` | absolute wheel and rotor mechanical angle under no slip |
| `q = x/r - theta` | rotor angle relative to the stator/chassis |
| `qdot` | relative rotor/stator angular speed |
| `N=50` | electrical cycles per mechanical revolution |
| `N*q` | electrical rotor angle |
| `field` | commanded electrical field angle from cumulative STEP position |
| `delta = field - N*q` | wrapped electrical phase error |

One STEP advances mechanical field angle by `2*pi/6400`, or electrical field
angle by `2*pi/128 = 2.8125 degrees`. The actuator keeps cumulative field
position separate from the actual relative rotor angle. SPS only schedules
STEP events; it does not inject force.

The production release fixture initializes the arbitrary translation origin so
that an initially aligned field has `x/r - theta = 0`. This is a coordinate
choice, not a physical translation impulse. Prescribed ringdown fixtures set
the same relation explicitly.

## 2. Actuator equations

The ideal-current path evaluates each motor as

```text
i_ref = I [cos(field), sin(field)]
tau = Kt I sin(delta)
```

where `I` is the two-phase current-vector norm. Positive phase error produces
positive translation torque and the equal/opposite negative body reaction.

The electrical path evaluates the same geometry from actual winding currents:

```text
tau = Kt (-ia sin(N*q) + ib cos(N*q))
ea  = -Ke qdot sin(N*q)
eb  =  Ke qdot cos(N*q)
L dia/dt = va - R ia - ea
L dib/dt = vb - R ib - eb
```

The bridge voltage is bounded by `+/-11.1 V`. For each physical substep the
code uses the exact constant-voltage R/L solution and chooses a voltage that
reaches the reference if the reference is reachable; otherwise it saturates
at the bus. Electrical and mechanical power use

```text
P_e = va*ia + vb*ib
P_m = tau*qdot
P_R = R*(ia^2 + ib^2)
E_L = 1/2*L*(ia^2 + ib^2)
```

The equality `Ke=Kt` is required here because the back-EMF projection is the
same current-vector projection as torque. With the signs above,

```text
P_e = P_R + dE_L/dt + P_m
```

for a fixed field/current reference. A positive prescribed rotor speed gives
back-EMF that opposes the applied current drive in the winding equations.

Motor-relative damping is `tau_d = -c*qdot`. It is an internal rotor/stator
torque pair and is included in the generalized force once.

## 3. Mechanical derivation and corrected mass matrix

The physical kinetic energy for two identical attached wheel/rotor assemblies
is

```text
T = 1/2 (M + 2*Iw/r^2) xdot^2
    + H*cos(theta)*xdot*thetadot
    + 1/2*J*thetadot^2.
```

Here `Iw=3.24e-5 kg m^2` per side already includes wheel, hub, O-ring, and
rotor. The mass matrix is therefore

```text
D = [ M + 2*Iw/r^2,  H*cos(theta) ]
    [ H*cos(theta),   J            ].
```

The old implementation used `d12 = H*cos(theta) - 2*Iw/r` and
`d22 = J + 2*Iw`. Those terms are what result from treating `q` as a physical
rotor kinetic coordinate after already imposing no slip. That double-counts
relative stator motion and is not the kinetic energy of the attached rotor.

For one motor torque `tau`, virtual work is

```text
delta W = tau*(delta x/r - delta theta)
```

so the two-motor mechanical equations use

```text
Qx     = (tau_left + tau_right)/r + external_force
Qtheta = -(tau_left + tau_right) + external_pitch_moment.
```

The remaining terms are the Lagrange gravity, Coriolis, cart damping, and
pitch damping terms. The simulator inverts `D` once and integrates velocity,
position, pitch rate, and pitch with a trapezoidal state update. The actuator's
hidden relative angle is advanced from the same relative velocity, so it does
not drift away from the no-slip coordinate.

The following equivalences are permanent tests:

- the public mass matrix equals the explicit kinetic-energy quadratic form;
- removing rotating inertia changes only `d11`;
- the determinant is positive;
- actuator torque and independently applied generalized torque produce the
  same instantaneous accelerations;
- positive and negative torque have symmetric translation/pitch signs.

## 4. Torque normalization

The maintained motor specification is the 1.8-degree, 200-step motor with
approximately `0.45 N m` holding torque at `1.5 A` per winding. The vendor
specification is retained at the [motor reference page](https://www.omc-stepperonline.com/nema-17-bipolar-45ncm-63-74oz-in-1-5a-42x42x39mm-4-wires-1m-pin-connector-17hs15-1504s-x1).
The [DRV8825 datasheet](https://www.ti.com/lit/ds/symlink/drv8825.pdf)
documents the two-phase/full-step current convention and 1/32 microstep
capability.

The simulator current is not one winding current. It is the norm of a
two-winding vector. Interpreting the rated two-phase holding datum in that
representation gives

```text
Kt_vector = 0.45 / (sqrt(2) * 1.5)
           = 0.2121320344 N m/A.
```

The same value is used for `Ke` in mechanical-relative SI units. With the
nominal simulator current vector `1.065 A`, the modeled peak torque is
`0.2259206 N m` per motor and the two-motor rim-force equivalent is
`10.967 N`. These are derived model values, not a claim that the motor's
manufacturer holding torque has been re-measured at the configured driver
setting.

The normalization is independently supported by the fixture. The old
`Kt=0.30` value predicted about `111.6 Hz`; the corrected value predicts
`93.9746 Hz`, matching the retained `93–95 Hz` mode. Because the fixture
stator is fixed and its inertia is measured, changing the balancing mass
matrix cannot explain that particular frequency ratio.

## 5. Energy and passivity checks

The ideal-current fixed-field potential used by the diagnostic is

```text
U = -(Kt*I/N) [cos(delta_left) + cos(delta_right)].
```

Its derivative gives the same rotor torque as the actuator equation. The
gravity-disabled coupled-plant test starts with a symmetric 5-degree
electrical phase displacement and checks:

- bounded ideal-current motion with cart, pitch, and motor damping disabled;
- total mechanical plus magnetic energy error below `2e-5 J` over 4000
  25-microsecond steps;
- positive/negative state symmetry;
- lower final energy with positive motor-relative damping;
- no material energy-growth sequence in the damped case.

The electrical test integrates the winding power balance in both rotor-motion
directions and checks electrical work against mechanical work, copper loss,
and inductive-energy change. Static prescribed phase tests check amplitude,
sign, and symmetry, while voltage-limit tests check current rise and bus
voltage sensitivity. In the prescribed-motion check, reversing rotor speed
reverses the back-EMF sign and mechanical work; the generating/back-driven
case has negative electrical work while copper loss remains positive.

These tests rule out the former coordinate-coupling and torque-sign failure
mechanisms within the modeled plant. They do not prove that the real driver
has the same current-loop dynamics.

## 6. Free-wheel calibration

For the fixed-stator fixture, one motor has

```text
k_phase = Kt*I*N = 11.2960 N m/rad
omega_n = sqrt(k_phase/Iw) = 590.46 rad/s
f_n     = 93.9746 Hz.
```

With `c=0.0027 N m s/rad`, the fixed-stator small-signal damping ratio is
approximately `0.0706`. The maintained numerical fixture gives:

| Path/quantity | Result |
| --- | ---: |
| analytical fixed-stator frequency | 93.9746 Hz |
| ideal-current numerical ringdown | 93.7417 Hz |
| electrical numerical ringdown | 93.7417 Hz |
| positive/negative one-step frequency | identical within test tolerance |
| two-step frequency | identical within test tolerance |
| numerical damping ratio | 0.0706 |
| measured hardware mode | 93–95 Hz |
| measured provisional damping ratio | 0.04–0.08 |

The body-coupled two-motor mode is a different mode, not a contradiction: its
combined magnetic stiffness is `22.5921 N m/rad`, its projected effective
inertia is `1.459334e-4 kg m^2`, and its natural frequency is `62.6211 Hz`.
The numerical coupled ringdown is about 62.2 Hz. A 10-microsecond physical
step and a 5-microsecond step agree within the permanent convergence test.

The electrical fixture has negligible current error at the low-speed
ringdown, as expected: its R/L and bus limits matter in current-rise and
high-speed tests, not materially in this small free-wheel calibration.

## 7. Known-attitude release and early reversal

The corrected electrical plant was run from symmetric ±1-degree releases.
The historical controller points still fall:

| `K_pitch / K_rate` | +1-degree peak | -1-degree peak | result |
| --- | ---: | ---: | --- |
| `6000 / 350` | 90 degrees | 90 degrees | fall |
| `12000 / 700` | 90 degrees | 90 degrees | fall |
| `24000 / 1400` | 90 degrees | 90 degrees | fall |

At the two lower historical electrical cases, the recorded voltage-saturation
fraction is zero: `6000/350` reaches a phase peak of 0.391 rad and
`8000/350` 0.333 rad before the body envelope diverges. Their peak modeled
torques are 0.0861 and 0.0738 N m per motor. This supports the conclusion that
the original failure is not primarily bus-voltage/current-rise limitation.

The old 6000/350 causal trace had a first clear torque reversal around
12–15 ms, followed by repeated phase reversals and an expanding body envelope.
With the selected corrected-plant candidate, the first sampled torque
reversal is 22.5 ms; its ±1-degree peak is 1.003 degrees and the three-second
tail RMS is 0.405 degrees. The phase peak is 0.297 rad, peak torque is
0.0661 N m per motor, and the current error and voltage-saturation time are
zero in that release.

The fixed-field ringdown shows that phase reversal is physically valid: an
underdamped rotor crosses the commanded field and reverses restoring torque
at approximately the 94 Hz mode. In the controlled release, the field is
moving and the controller rate term changes the field speed. The early
reversal is therefore not itself evidence of a sign bug. The old gain pair
fails because the moving field, estimator/controller delay, and body mode
leave a growing coupled error before enough stabilizing wheel motion is
created; the passivity tests show no corresponding ideal-plant energy source.

## 8. Follow-up controller gain-scale audit

The original tune section above is superseded by the focused
[gain-scale and balance-chain follow-up](stepper_gain_scale_audit.md). The
follow-up verifies that controller inputs remain radians/radians per second,
independently checks the full balance mass matrix and generalized motor force,
and migrates the verified 1/32 balance rail to `16000 SPS`.

The current local electrical-plant gain region is approximately:

```text
K_pitch = 160000 .. 300000 SPS/rad
K_rate  =   8000 ..  16000 SPS/(rad/s)
```

The selected simulator-only frontier point is `280000/12000`. It recovers
quietly through ±4 degrees in the compact run; ±6 and ±8 degrees eventually
fall after reaching the 16000-SPS rail. The earlier `180000/8000` point
remains a lower-feedback symmetric ±1-degree candidate and was not silently
promoted to hardware.

## 9. Pulse-rate authority

`Config::max_step_rate_sps` is now the single `16000 SPS` scheduler/controller
rail. `DualWave` carries 40 pulses per channel per 2.5 ms frame at that limit.
The selected `280000/12000` ±1-degree release uses approximately:

- requested peak/p95/p99: `13314.7/11677.8/12346.3 SPS`;
- emitted peak/p95/p99: `3926.8/3085.0/3585.0 SPS`;
- completed-step p95/p99: `3200/3600 SPS`;
- command and voltage saturation: `0 s` in the three-second ±1-degree run.

The same gains at the stale 8000-SPS cap require 1.4075 s of command
saturation in the three-second ±4-degree case; at 16000 this falls to 0.0175
s. Both ±6-degree ten-second cases still fall, so the new authority reduces
clipping without falsely claiming a larger robust frontier.

## 10. Exact changes

- Shared `Kt/Ke` definitions now use the vector-current normalization and the
  verified nominal current setting.
- Ideal-current and electrical actuators use the same magnetic torque scale.
- Stepper mechanics use the absolute wheel/rotor inertia matrix and the
  virtual-work torque pair.
- The initial Stepper release coordinate is initialized consistently with
  `q=x/r-theta`.
- Stepper state integration uses a trapezoidal position/pitch update so the
  actuator relative angle and generalized coordinates use the same interval.
- Gravity is configurable for gravity-disabled passivity fixtures; production
  profiles retain 9.81 m/s².
- Added mass-matrix, normalization, coordinate, passivity, power-balance,
  static torque, back-EMF, voltage-limit, ringdown, and focused-tune tests.
- Added controller-unit, estimator-boundary, 1/8-to-1/32 field-scaling,
  gravity-only, known-torque, quasi-static lean, scheduler-limit, and
  8000-versus-16000 authority checks.
- Updated the maintained scenario documentation and kept the wire telemetry
  schema unchanged.

## 11. Remaining uncertainties and recommendation

Remaining simulator uncertainties are the actual DRV8825 current-loop shape,
current-limit calibration under both phases, winding temperature/resistance,
bus droop, structural and tire compliance, wheel contact/slip, unmodeled
friction, and estimator/hardware timing under the selected gain scale. The
hardware ringdown supports stiffness and damping correlation but does not
identify absolute torque.

Recommendation: use `StepperPhaseElectrical` as the primary plant for
analytical/controller-development work and for selecting relative gain
regions. Treat `280000/12000` as a simulator-only preliminary candidate, with
`180000/8000` retained as a lower-feedback comparison point. The archived
hardware data does not contain a known-gain 6000/350 400 Hz command replay.
Before any hardware use, retain a complete PID snapshot, reproduce the
free-wheel frequency/current trace, validate safe pulse and current limits,
and perform a restrained signed hardware release test with independent fall
protection.

## Verification commands

The focused audit command was:

```sh
ctest --test-dir build --output-on-failure \
  -R 'StepperPhase|SimulatorReference|SimulatorModelIdentification'
```

The normal repository gate remains:

```sh
pytest --build
```

The handoff also requires `git diff --check` and an inspection of generated or
unexpected worktree changes.
