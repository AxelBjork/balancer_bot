# Hardware Reference

This directory contains the physical build reference material for the robot. It is separate from the runtime and software handbook.

## What Is Here

- `hardware/frame/B-robot_EVO_2/`
  original or upstream frame/STL assets
- `hardware/frame/custom/`
  local modified print files for this build

The [frame source directory](frame/B-robot_EVO_2/) and [local custom frame directory](frame/custom/)
contain the physical assets referenced below.

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

## Frame Source

- **B-robot EVO 2 (jjrobots)** — [Thingiverse frame source](https://www.thingiverse.com/thing:2306541)

## Wiring Summary

- **IMU → Pi**
  Use the Qwiic SHIM on the Pi and a Qwiic cable to the ISM330DHCX breakout.
- **Motors → HAT**
  Connect each NEMA-17 to one DRV8825 channel and set microstepping as needed.
- **Power**
  Feed the Stepper Motor HAT from the battery pack. Back-powering the Pi from the HAT is electrically
  possible, but it is not a validated project recommendation; verify the regulator current budget
  and wiring before using it.

## Notes

- This directory is reference material for the physical build.
- Software setup, deployment, and runtime bring-up are documented in [Running on Pi](../doc/Running_on_Pi.md).
- Runtime confidence and caveats are summarized in [Current Status](../doc/status.md); retained
  hardware evidence is listed in the [data archive](../data/README.md).
