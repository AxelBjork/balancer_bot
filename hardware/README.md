# Hardware reference

This page is the physical-build reference. Software behavior and
controller-facing equations belong in
[`doc/arch/control_plant.md`](../doc/arch/control_plant.md); retained runtime
captures belong in [`data/README.md`](../data/README.md).

## Bill of Materials

| Qty | Item | Function | Brand/Manufacturer | Part Number / Model |
| --- | ---- | -------- | ------------------ | ------------------- |
| 1 | Raspberry Pi 4 Model B (4 GB) | Main controller | Raspberry Pi | SC0194 |
| 1 | ISM330DHCX 6-DoF IMU breakout (STEMMA QT/Qwiic) | IMU | Adafruit | 4502 |
| 2 | NEMA-17 stepper motor, 45 N·cm, 1.5 A, 12 V, 42×39 mm | Actuation | STEPPERONLINE | 17HS15-1504S-X1 |
| 1 | Stepper Motor HAT (dual DRV8825, up to 1/32 microstepping) | Motor driver | Waveshare | 15669 |
| 1 | 3S 18650 Li-ion battery pack, ~70×55×20 mm | Power | — | Generic |
| 1 | Qwiic SHIM for Raspberry Pi | I²C connector | SparkFun | DEV-15794 |
| 1 | Qwiic cable, JST-SH 4-pin, 100 mm | I²C cable | SparkFun | PRT-14427 |
| — | Frame (PA12 SLS print) | Mechanical structure | jjrobots | see source below |

Physical frame assets are under [`hardware/frame/`](frame/). The original
frame source is the [B-robot EVO 2 Thingiverse page](https://www.thingiverse.com/thing:2306541).

## Wiring

- Connect the IMU to the Pi through the Qwiic SHIM and cable.
- Connect each motor to one DRV8825 channel on the HAT.
- Power the HAT from the battery pack.
- Do not assume the HAT can safely back-power the Pi; verify regulator current
  capacity and wiring before using that arrangement.

## Measured and configured hardware values

These are build measurements, verified hardware configuration, or values
derived directly from them. The list focuses on the physical build and
measured hardware behavior.

| Quantity | Value | Basis |
| --- | ---: | --- |
| wheel radius | `41.2 mm` | Measured wheel geometry; [production configuration](../src/services/main/config.h) |
| motor full steps/rev | `200` | Motor/driver configuration |
| microsteps/full step | `32` | Driver configuration |
| commanded STEP/rev | `6400` | `200 × 32` |
| wheel travel/STEP | `40.448 µm` | Derived from the measured radius and configured STEP/rev |
| free-wheel actuator ringdown | `93–95 Hz` | Optical hardware correlation; [retained measurement](../data/hardware_sessions/motor_tracking/README.md) |
| IMU/chassis vibration modes | `26.9 Hz` and `33.4 Hz` | Repeatable hardware rate bands; [filter evidence](../doc/arch/imu_attitude_design.md#motor-vibration-and-notch-filtering) |

The `93–95 Hz` ringdown is the one-wheel optical actuator measurement; the
`26.9 Hz` and `33.4 Hz` entries are IMU-observed chassis/rate vibration bands.
The retained neutral-floor capture also reports a `2.3065 Hz` closed-loop
rocking mode, but that is specific to that run rather than a hardware constant.

## Bring-up and evidence

Use [`doc/Running_on_Pi.md`](../doc/Running_on_Pi.md) for deployment and
restrained bring-up. Start with the wheels off the ground or physically
restrained, confirm IMU validity and motor polarity, and inspect telemetry
before floor tests.

Hardware logs are historical evidence unless their manifest binds the source
capture, PID/config snapshot, repository revision, command semantics, and
selection window. Do not fit motor or controller parameters directly from a
session README.
