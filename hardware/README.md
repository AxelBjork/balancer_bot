# Hardware reference

This page is the physical-build reference. Software behavior and
controller-facing equations belong in
[`doc/arch/control_plant.md`](../doc/arch/control_plant.md); retained runtime
captures belong in [`data/README.md`](../data/README.md). The detailed
electrical simulator realization is the separate
[`StepperPhaseElectrical testing profile`](../doc/testing/stepper_phase_electrical.md).

## Build

- Raspberry Pi 4 Model B, 4 GB (`SC0194`)
- ISM330DHCX 6-DoF IMU breakout (Adafruit 4502)
- Two NEMA-17 stepper motors, 45 N·cm, 1.5 A, 12 V, 42×39 mm
- Waveshare Stepper Motor HAT with dual DRV8825 drivers, up to 1/32
  microstepping (`15669`)
- 3S 18650 Li-ion battery pack, approximately 70×55×20 mm
- SparkFun Qwiic SHIM (`DEV-15794`) and 100 mm JST-SH cable (`PRT-14427`)
- PA12 SLS frame based on [B-robot EVO 2](https://www.thingiverse.com/thing:2306541)

Physical frame assets are under [`hardware/frame/`](frame/). The original
frame source is the [B-robot EVO 2 Thingiverse page](https://www.thingiverse.com/thing:2306541).

## Wiring

- Connect the IMU to the Pi through the Qwiic SHIM and cable.
- Connect each motor to one DRV8825 channel on the HAT.
- Power the HAT from the battery pack.
- Do not assume the HAT can safely back-power the Pi; verify regulator current
  capacity and wiring before using that arrangement.

## Physical values used by the simulator

These are the current shared nominal values, not a complete hardware
calibration:

| Quantity | Value |
| --- | ---: |
| wheel radius | `41.2 mm` |
| commanded microsteps/rev | `6400` at 1/32 |
| wheel travel/STEP | `40.448 µm` |
| nominal bus voltage in model | `11.1 V` |
| configured driver current limit in model | `1.065 A` |
| balance command ceiling | `16000 SPS` |

The `16000 SPS` value is a controller/scheduler ceiling, not a promise that a
motor can sustain that speed under every load. Translation must retain headroom
for balance recovery and turn allocation.

The current shared controller profile is the StepperPhaseElectrical profile in
[`pid.conf`](../pid.conf). It uses `203550 / 1932 / 0` inner gains, a
conservative velocity P gain, `10°` motion authority, fixed zero COM trim, and
adaptive COM trim disabled. It has not been cleared for unattended hardware
operation.

## Bring-up and evidence

Use [`doc/Running_on_Pi.md`](../doc/Running_on_Pi.md) for deployment and
restrained bring-up. Start with the wheels off the ground or physically
restrained, confirm IMU validity and motor polarity, and inspect telemetry
before floor tests.

Hardware logs are historical evidence unless their manifest binds the source
capture, PID/config snapshot, repository revision, command semantics, and
selection window. Do not fit motor or controller parameters directly from a
session README. The electrical model and its limits are documented in the
[StepperPhaseElectrical testing profile](../doc/testing/stepper_phase_electrical.md).
