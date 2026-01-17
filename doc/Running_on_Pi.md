# Cross-Compilation Setup for Raspberry Pi 4

## Prerequisites

### Docker (Recommended)
No local toolchain installation needed! The build uses a Docker container with the ARM64 cross-compiler.

**Verify Docker is installed:**
```bash
docker --version
```

## Building for Raspberry Pi
The devcontainer now includes the cross-compilation toolchain.

### Build Command
```bash
./build_cmake OFF
```
This will:
1. Configure CMake with `toolchain-rpi4.cmake`.
2. Cross-compile `balancer_pi` for ARM64 using the installed toolchain.
3. Output artifacts to the `build-pi/` directory.

## Deploying and Running

You can deploy and run the application on the Raspberry Pi with a single command (run from the host machine):

```bash
scp build-pi/balancer_pi pi@rpi4:~/ && ssh -t pi@rpi4 "echo ism330dhcx 0x6a | sudo tee /sys/bus/i2c/devices/i2c-1/new_device; chmod +x ~/balancer_pi && sudo ~/balancer_pi"
```

This command will:
1. Copy the executable to the Pi.
2. Ensure the `pigpiod` daemon is running.
3. specific execute permissions.
4. Run the application with `sudo`.

## Troubleshooting

### Missing Libraries on Pi
If you get library errors, check with:
```bash
ldd balancer_pi
```

Install missing libraries:
```bash
sudo apt-get install libpigpiod-if2-1 libsdl2-2.0-0
```

### Cross-Compiler Not Found
Make sure the toolchain is installed and in your PATH:
```bash
which aarch64-linux-gnu-gcc
```

### Wrong Architecture
If you see "cannot execute binary file: Exec format error" on the Pi:
- Your Pi might be 32-bit. Check with `uname -m` on the Pi.
- If `armv7l`, you need the 32-bit toolchain (see Prerequisites above).
