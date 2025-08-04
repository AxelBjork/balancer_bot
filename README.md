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