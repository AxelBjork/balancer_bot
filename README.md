# balancer_bot
balancer_bot

# build
```
cd /home/axel/Public/Project/vscode/balancer_bot/PX4-Autopilot
make px4_sitl_default -j$(nproc) EXTERNAL_MODULES_LOCATION=../external
``

# Run SITL
```
make px4_sitl none

pxh> balance_controller start
pxh> balance_controller status

```

## Motor Test

g++ motor_off.cpp -o motor_hat_test -lpigpio -lrt
g++ motor_hat_test.cpp -o motor_hat_test -lSDL2 -lpigpiod_if2 -lrt -pthread
./motor_hat_test
sudo ./motor_off


## Bluetooth connect
bluetoothctl
power on
scan on
pair 28:EA:0B:E1:04:E6
connect 28:EA:0B:E1:04:E6

## XBOX Control

sudo apt install libsdl2-dev

g++ main.cpp -o xbox_demo -lSDL2
./xbox_demo