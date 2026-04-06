# Running on Raspberry Pi

This guide covers the practical path for cross-building, deploying, and bringing up `balancer_pi`.

## Cross-Build

The standard cross-build command is:

```bash
./build_cmake OFF
```

That configures CMake with `cmake/toolchain-rpi4.cmake` and builds:

- `build-pi/balancer_pi`
- `build-pi/imu_demo`

## What to Copy to the Pi

`balancer_pi` loads `pid.conf` from its working directory, so copy both files:

```bash
scp build-pi/balancer_pi pid.conf pi@rpi4:~/
```

If you want to keep the PID config elsewhere on the Pi, use the `BALANCER_PID_CONF` environment variable for simulator-oriented flows. The normal hardware path expects `pid.conf` to be present next to the launched process or in the current working directory.

## Runtime Prerequisites

### Required Packages and Services

```bash
sudo apt-get install libpigpiod-if2-1 libsdl2-2.0-0
sudo systemctl enable --now pigpiod
```

### IMU Binding

The runtime expects the ISM330DHCX to be visible through the Linux IIO path. A manual bind looks like:

```bash
echo ism330dhcx 0x6a | sudo tee /sys/bus/i2c/devices/i2c-1/new_device
```

### Linux Setup Assets

The repo keeps the Linux-side helper assets here:

```text
src/platform/linux/udev/99-iio-perms.rules
src/platform/linux/setup_permissions.sh
```

The udev rule is the stable source-controlled reference. The helper script is useful if you want to install a more automated local setup on the Pi.

## First Bring-Up

Once the binary and `pid.conf` are on the Pi:

```bash
ssh pi@rpi4
chmod +x ~/balancer_pi
sudo systemctl enable --now pigpiod
echo ism330dhcx 0x6a | sudo tee /sys/bus/i2c/devices/i2c-1/new_device
sudo ./balancer_pi
```

Notes:

- the app will start even if no Xbox controller is attached
- it still loads `pid.conf` from the current working directory
- `UdpBridge` is also started, so you can observe telemetry if port `9000` is reachable

## Recommended First Hardware Session

1. Start with the wheels off the ground or the robot physically restrained.
2. Confirm `pigpiod` is up.
3. Confirm the IMU node is visible and the app does not fail during startup.
4. Confirm `pid.conf` is the version you intend to run.
5. Only move on to floor testing after verifying the runtime is alive and stable enough for a controlled attempt.

## Troubleshooting

### `balancer_pi` Fails to Start

- verify `pid.conf` exists in the working directory
- verify `pigpiod` is running
- verify the IMU has been bound and the IIO devices exist

### Missing Shared Libraries

Check:

```bash
ldd ~/balancer_pi
```

Install missing packages:

```bash
sudo apt-get install libpigpiod-if2-1 libsdl2-2.0-0
```

### Cross-Compiler Not Found in the Dev Environment

Check:

```bash
which aarch64-linux-gnu-gcc
```

The project’s helper script uses `cmake/toolchain-rpi4.cmake`.
