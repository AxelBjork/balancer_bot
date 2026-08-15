# Running on Raspberry Pi

This guide covers the practical path for cross-building, deploying, and bringing up `balancer_pi`.

## Cross-Build

The standard cross-build command is:

```bash
./build_cmake OFF
```

That configures CMake with `cmake/toolchain-rpi4.cmake` and builds:

- `build-pi/balancer_pi`

## Pi Base Setup

### Firmware Config

Edit `/boot/firmware/config.txt` and make sure the board has I2C enabled. The prior working setup used:

```ini
dtparam=i2c_arm=on
dtparam=i2c_arm_baudrate=400000
gpio=4=op,dl
```

Notes:

- `dtparam=i2c_arm=on` is required for the IMU on bus `1`
- `dtparam=i2c_arm_baudrate=400000` is the expected fast I2C setting
- `gpio=4=op,dl` is hardware-specific; keep it if your existing wiring/setup depends on GPIO 4 being driven low at boot

Reboot after changing `config.txt`.

## What to Copy to the Pi

`balancer_pi` loads `pid.conf` from its working directory, so copy both files:

```bash
scp build-pi/balancer_pi pid.conf pi@rpi4:~/
```

If you want to keep the PID config elsewhere on the Pi, use the `BALANCER_PID_CONF` environment variable for simulator-oriented flows. The normal hardware path expects `pid.conf` to be present next to the launched process or in the current working directory.

## Passive Pitch-Inertia Measurement

Disable the controller so IMU telemetry remains active while the motors are de-energized. Support
the robot on a low-friction pivot through the wheel axis and record a few small free oscillations.
Use the IMU pitch data to estimate the period `P`, then calculate the pitch inertia about the axle:

> $$
> J = g H \left(\frac{P}{2\pi}\right)^2
> $$

Here, `H` is the robot's first mass moment about the wheel axis. The
[measurement helper](../tools/measure_pitch_inertia.py) can calculate the period and inertia from a
telemetry capture. Select one continuous free-swing session and a clean interval, for example:

```bash
python3 tools/measure_pitch_inertia.py data/server/telemetry_YYYYMMDD-HHMMSS_00.csv \
  --session 0 --start-s 51.2 --end-s 54.6
```

It reports a robust median period and its spread. If you have an unassisted amplitude-decay
measurement, pass its natural-log decrement per period with `--log-decrement`; do not infer
damping from a manually assisted capture. Update the authoritative value in
[`HardwareNominal`](../tests/simulator/balancer_simulator.h), restore the normal configuration, and
verify the build before balancing.

## Runtime Prerequisites

### Required Packages and Services

```bash
sudo apt-get install libpigpiod-if2-1 libsdl2-2.0-0
sudo systemctl enable --now pigpiod
```

If you need to build the fallback IMU driver directly on the Pi, also install:

```bash
sudo apt-get install raspberrypi-kernel-headers build-essential wget
```

### IMU Binding

The runtime expects the ISM330DHCX to be visible through the Linux IIO path. A manual bind looks like:

```bash
echo ism330dhcx 0x6a | sudo tee /sys/bus/i2c/devices/i2c-1/new_device
```

Verify the I2C device is present and that the kernel actually creates IIO nodes after binding:

```bash
i2cdetect -y 1
ls /sys/bus/iio/devices/
echo 0x6a | sudo tee /sys/bus/i2c/devices/i2c-1/delete_device 2>/dev/null
echo ism330dhcx 0x6a | sudo tee /sys/bus/i2c/devices/i2c-1/new_device
ls /sys/bus/iio/devices/
```

If `i2cdetect` shows `0x6a` but `/sys/bus/iio/devices/` stays empty after `new_device`, the IMU driver is not probing correctly and `balancer_pi` will fail before runtime startup.

### IMU Driver Fallback for Raspberry Pi Kernels Without ST IIO Support

Some Raspberry Pi kernels ship without the ST IMU IIO drivers enabled. When that happens:

- `i2cdetect -y 1` still shows `0x6a`
- `echo ism330dhcx 0x6a > .../new_device` succeeds
- `/sys/bus/iio/devices/` stays empty

In that case, build the `st_lsm6dsx` IIO driver out of tree on the Pi.

The branch should match the Raspberry Pi kernel family, not the full `uname -r` string. For a kernel like:

```text
6.12.75+rpt-rpi-v8
```

use:

Run:

```bash
sudo apt-get install -y raspberrypi-kernel-headers build-essential wget
mkdir -p ~/lsm6dsx-oot && cd ~/lsm6dsx-oot
BRANCH=rpi-6.12.y
BASE=https://raw.githubusercontent.com/raspberrypi/linux/$BRANCH/drivers/iio/imu/st_lsm6dsx
for f in st_lsm6dsx_core.c st_lsm6dsx_buffer.c st_lsm6dsx_shub.c st_lsm6dsx_i2c.c st_lsm6dsx.h; do
  wget -q "$BASE/$f"
done
cat > Makefile <<'EOF'
obj-m += st_lsm6dsx.o
obj-m += st_lsm6dsx_i2c.o
st_lsm6dsx-objs := st_lsm6dsx_core.o st_lsm6dsx_buffer.o st_lsm6dsx_shub.o
EOF
make -C /lib/modules/$(uname -r)/build M=$PWD modules
sudo mkdir -p /lib/modules/$(uname -r)/updates
sudo cp st_lsm6dsx*.ko /lib/modules/$(uname -r)/updates/
sudo depmod -a
sudo modprobe industrialio
sudo modprobe industrialio_triggered_buffer
sudo modprobe st_lsm6dsx_i2c
echo 0x6a | sudo tee /sys/bus/i2c/devices/i2c-1/delete_device 2>/dev/null
echo ism330dhcx 0x6a | sudo tee /sys/bus/i2c/devices/i2c-1/new_device
ls /sys/bus/iio/devices/
```

If that creates IIO devices, the current userspace reader should be able to run.

### Optional Auto-Load and Auto-Bind

If you want the IMU stack reloaded on boot:

```bash
printf "industrialio\nindustrialio_triggered_buffer\nst_lsm6dsx_i2c\niio-trig-hrtimer\n" | \
  sudo tee /etc/modules-load.d/ism330dhcx.conf
```

Optional rebinding helper:

```bash
sudo tee /usr/local/sbin/add-ism330dhcx.sh >/dev/null <<'EOF'
#!/bin/sh
NODE="/sys/bus/i2c/devices/i2c-1/new_device"
for i in $(seq 1 20); do
  [ -e "$NODE" ] && { echo "ism330dhcx 0x6a" > "$NODE"; exit 0; }
  sleep 0.25
done
exit 1
EOF
sudo chmod +x /usr/local/sbin/add-ism330dhcx.sh
```

```bash
sudo tee /etc/udev/rules.d/60-ism330dhcx.rules >/dev/null <<'EOF'
SUBSYSTEM=="i2c", KERNEL=="i2c-1", ACTION=="add", RUN+="/usr/local/sbin/add-ism330dhcx.sh"
EOF
sudo udevadm control --reload
```

### Linux Setup Assets

The repo keeps the Linux-side helper assets here:

```text
src/platform/linux/udev/99-iio-perms.rules
src/platform/linux/setup_permissions.sh
```

The udev rule is the stable source-controlled reference. The helper script is useful if you want to install a more automated local setup on the Pi.

### Optional IIO Permissions for Non-Root Runs

If you want to run the app without `sudo`, set up the IIO group and trigger permissions:

```bash
sudo groupadd -f iio
sudo usermod -aG iio "$USER"
newgrp iio
```

The current udev rule reference is:

```text
src/platform/linux/udev/99-iio-perms.rules
```

The helper script is:

```text
src/platform/linux/setup_permissions.sh
```

The runtime creates the `imu833` hrtimer trigger directory as needed, but non-root access still depends on the right configfs and IIO permissions.

At startup the reader now writes and reads back the 833 Hz accelerometer, gyroscope, and trigger
rates, along with the expected ±2 g and ±250 dps IIO scales. It stops instead of running with an
unverified conversion.

### Optional Bluetooth / Xbox Controller Setup

The app now starts safely with no controller attached, but if you want local Xbox input on the Pi:

```bash
bluetoothctl
power on
menu scan
pattern Xbox
back
scan on
pair <controller-mac>
connect <controller-mac>
```

## First Bring-Up

The examples below use one SSH alias consistently. If your SSH config calls the Pi something else,
change only `PI_HOST`; the dashboard, `ssh`, and `scp` commands will all use it:

```bash
PI_HOST=rpi4
PI_TARGET="pi@${PI_HOST}"
```

Once the binary and `pid.conf` are on the Pi:

```bash
ssh "$PI_TARGET"
chmod +x ~/balancer_pi
sudo systemctl enable --now pigpiod
echo ism330dhcx 0x6a | sudo tee /sys/bus/i2c/devices/i2c-1/new_device
sudo ./balancer_pi
```

If you want a one-shot deploy and run command from the host:

```bash
scp build-pi/balancer_pi pid.conf "${PI_TARGET}:~/" && \
ssh -t "$PI_TARGET" "echo ism330dhcx 0x6a | sudo tee /sys/bus/i2c/devices/i2c-1/new_device || true && \
chmod +x ~/balancer_pi && \
sudo ~/balancer_pi"
```

### Live Dashboard

Run this on the development laptop after the Pi runtime has started:

```bash
python3 tools/telemetry_dashboard/server.py --pi "$PI_HOST"
```

Then open `http://127.0.0.1:8080`. The dashboard expands the SSH alias through
`~/.ssh/config`, so `rpi4` can map to a changing LAN address without updating another command.
It receives the existing UDP stream and claims its one external peer slot; stop the dashboard
before running a SIL client or another UDP observer.

If the dashboard is running on the Windows host where `scp` already works, enable its optional
file-picker deployment panel with:

```powershell
py tools/telemetry_dashboard/server.py --pi rpi4
```

Press **Deploy current build** to copy `build-pi/balancer_pi` and `pid.conf` to the Pi. The
dashboard compares `pid.conf`'s schema version with a version marker embedded in the binary. Gain
and limit edits can be deployed without rebuilding; schema changes require a matching rebuild.
**Start bot**
launches the uploaded binary asynchronously, waits briefly for it to remain alive, and reports an
immediate startup failure with the relevant tail of `~/balancer_pi.log` on the page. **Abort bot**
sends `SIGTERM` to every process whose exact name is `balancer_pi`. This lets the robot disable
the motor driver during its normal teardown. These buttons use `sudo -n`; configure
non-interactive sudo for the Pi user first or the page reports the sudo error.

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
