# Self-Balancing Robot (Pi-Based)

A compact, personal project that repurposes the **B-robot EVO 2** frame for a Raspberry Pi–based self-balancing robot. This README keeps the hardware generic (no vendor links or prices) but includes part numbers so you can source equivalents easily.

# build
```
cd /home/axel/Public/Project/vscode/balancer_bot/PX4-Autopilot
make px4_sitl_default -j$(nproc) EXTERNAL_MODULES_LOCATION=../external
```

## Run SITL
```
make px4_sitl none

pxh> balance_controller start
pxh> balance_controller status

xbox_remote start
xbox_remote status
```

## Bill of Materials

| Qty | Item | Function | Brand/Manufacturer | Part Number / Model |
| --- | ---- | -------- | ------------------ | ------------------- |
| 1 | Raspberry Pi 4 Model B (4 GB) | Main controller | Raspberry Pi | SC0194 |
| 1 | ISM330DHCX 6-DoF IMU breakout (STEMMA QT/Qwiic) | IMU | Adafruit | 4502 |
| 2 | NEMA-17 stepper motor, 45 N·cm, 1.5 A, 12 V, 42×39 mm | Actuation | STEPPERONLINE | 17HS15-1504S-X1 |
| 1 | Stepper Motor HAT (dual DRV8825, up to 1/32 microstepping) | Motor driver | Waveshare | 15669 (Stepper Motor HAT) |
| 1 | 3S 18650 Li-ion battery pack, ~70×55×20 mm | Power | — | Generic (match size/BMS) |
| 1 | Qwiic SHIM for Raspberry Pi | I²C connector | SparkFun | DEV-15794 |
| 1 | Qwiic cable, JST-SH 4-pin, 100 mm | I²C cable | SparkFun | PRT-14427 |
| — | Frame (PA12 SLS print) | Mechanical structure | jjrobots | see link below |


## Frame Design Source

- **B-robot EVO 2 (jjrobots)** — Thingiverse: https://www.thingiverse.com/thing:2306541

## Wiring (at a glance)

- **IMU → Pi**: Use the Qwiic SHIM on the Pi and the 100 mm Qwiic cable to the ISM330DHCX breakout (I²C: SDA/SCL, 3V3, GND).
- **Motors → HAT**: Connect each NEMA-17 to one DRV8825 channel. Set microstepping with DIP switches as needed (e.g., 1/16 or 1/32).
- **Power**: Feed the Stepper Motor HAT with your 3S (11.1–12.6 V) pack. If you choose to back-power the Pi from the HAT’s 5 V regulator, ensure adequate current; otherwise power the Pi via USB-C 5 V/3 A.


xbox_remote start
xbox_remote status

## Bill of Materials

| Qty | Item | Function | Brand/Manufacturer | Part Number / Model |
| --- | ---- | -------- | ------------------ | ------------------- |
| 1 | Raspberry Pi 4 Model B (4 GB) | Main controller | Raspberry Pi | SC0194 |
| 1 | ISM330DHCX 6-DoF IMU breakout (STEMMA QT/Qwiic) | IMU | Adafruit | 4502 |
| 2 | NEMA-17 stepper motor, 45 N·cm, 1.5 A, 12 V, 42×39 mm | Actuation | STEPPERONLINE | 17HS15-1504S-X1 |
| 1 | Stepper Motor HAT (dual DRV8825, up to 1/32 microstepping) | Motor driver | Waveshare | 15669 (Stepper Motor HAT) |
| 1 | 3S 18650 Li-ion battery pack, ~70×55×20 mm | Power | — | Generic (match size/BMS) |
| 1 | Qwiic SHIM for Raspberry Pi | I²C connector | SparkFun | DEV-15794 |
| 1 | Qwiic cable, JST-SH 4-pin, 100 mm | I²C cable | SparkFun | PRT-14427 |
| — | Frame (PA12 SLS print) | Mechanical structure | jjrobots | see link below |


## Frame Design Source

- **B-robot EVO 2 (jjrobots)** — Thingiverse: https://www.thingiverse.com/thing:2306541

## Wiring (at a glance)

- **IMU → Pi**: Use the Qwiic SHIM on the Pi and the 100 mm Qwiic cable to the ISM330DHCX breakout (I²C: SDA/SCL, 3V3, GND).
- **Motors → HAT**: Connect each NEMA-17 to one DRV8825 channel. Set microstepping with DIP switches as needed (e.g., 1/16 or 1/32).
- **Power**: Feed the Stepper Motor HAT with your 3S (11.1–12.6 V) pack. If you choose to back-power the Pi from the HAT’s 5 V regulator, ensure adequate current; otherwise power the Pi via USB-C 5 V/3 A.



# Pi Setup
`sudo nano /boot/firmware/config.txt`
Add
```
gpio=4=op,dl
dtparam=i2c_arm=on
dtparam=i2c_arm_baudrate=400000
```


**Enable pigiod daemon** `sudo systemctl enable --now pigpiod`

## Motor Test
**Stubs**
g++ motor_controller.cpp -o motor_controller -lSDL2 -lrt -pthread -I./stubs -DPIGPIOD_STUB_IMPL
**Pi Build**
g++ motor_controller.cpp -o motor_controller -lSDL2 -lpigpiod_if2 -lrt -pthread
g++ extras/motor_off.cpp -o motor_off -lpigpiod_if2 -lrt -pthread

./motor_controller


## Bluetooth connect
bluetoothctl
power on
scan on
pair 28:EA:0B:E1:04:E6
connect 28:EA:0B:E1:04:E6

## XBOX Control

sudo apt install libsdl2-dev

g++ extras/xbox_demo.cpp -o xbox_demo -lSDL2
./xbox_demo

## IMU Demo

`mkdir -p ~/lsm6dsx-oot && cd ~/lsm6dsx-oot`

Download only the driver sources from the matching RPi kernel branch
```
BRANCH=rpi-6.12.y
BASE=https://raw.githubusercontent.com/raspberrypi/linux/$BRANCH/drivers/iio/imu/st_lsm6dsx
for f in st_lsm6dsx_core.c st_lsm6dsx_buffer.c st_lsm6dsx_shub.c st_lsm6dsx_i2c.c st_lsm6dsx.h; do
  wget -q "$BASE/$f"
done
```
**Create makefile**
```
obj-m += st_lsm6dsx.o
obj-m += st_lsm6dsx_i2c.o

# st_lsm6dsx.ko is built from these three objects:
st_lsm6dsx-objs := st_lsm6dsx_core.o st_lsm6dsx_buffer.o st_lsm6dsx_shub.o
```

make -C /lib/modules/$(uname -r)/build M=$PWD modules

sudo mkdir -p /lib/modules/$(uname -r)/kernel/drivers/iio/imu/st_lsm6dsx
sudo cp st_lsm6dsx*.ko /lib/modules/$(uname -r)/kernel/drivers/iio/imu/st_lsm6dsx/
sudo depmod -a

// load and bind
sudo modprobe industrialio industrialio_triggered_buffer
sudo modprobe st_lsm6dsx_i2c

echo 0x6a | sudo tee /sys/bus/i2c/devices/i2c-1/delete_device 2>/dev/null
echo ism330dhcx 0x6a | sudo tee /sys/bus/i2c/devices/i2c-1/new_device

g++ extras/imu_demo.cpp -o imu_demo

### Configure auto setup
printf "industrialio\nindustrialio_triggered_buffer\nst_lsm6dsx_i2c\n" | \
  sudo tee /etc/modules-load.d/ism330dhcx.conf


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

sudo tee /etc/udev/rules.d/60-ism330dhcx.rules >/dev/null <<'EOF'
SUBSYSTEM=="i2c", KERNEL=="i2c-1", ACTION=="add", RUN+="/usr/local/sbin/add-ism330dhcx.sh"
EOF
sudo udevadm control --reload

# Unittest

./build_cmake