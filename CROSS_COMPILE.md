# Cross-Compilation Setup for Raspberry Pi 4

## Prerequisites

### Docker (Recommended)
No local toolchain installation needed! The build uses a Docker container with the ARM64 cross-compiler.

**Verify Docker is installed:**
```bash
docker --version
```

### Alternative: Local Toolchain
If you prefer not to use Docker, install the ARM64 cross-compiler:
```bash
sudo apt-get update
sudo apt-get install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu cmake
```

## Building for Raspberry Pi

### Using Docker (Recommended)
```bash
./build_pi
```

This will:
1. Build a Docker image with the ARM64 toolchain (first time only)
2. Cross-compile the `balancer_pi` executable inside the container
3. Output to `build-pi/balancer_pi` on your host

**First build** takes ~2 minutes (downloads Ubuntu + installs toolchain).  
**Subsequent builds** are fast (~10 seconds).

### Using Local Toolchain
If you installed the toolchain locally:
```bash
cmake -S . -B build-pi \
    -DCMAKE_TOOLCHAIN_FILE=toolchain-rpi4.cmake \
    -DBUILD_TESTS=OFF \
    -DCMAKE_BUILD_TYPE=Release
cmake --build build-pi -j$(nproc)
```

## Deploying to Raspberry Pi

### Copy Executable
```bash
scp build-pi/balancer_pi pi@<your-pi-ip>:~/
```

### On the Pi

#### Install Runtime Dependencies
```bash
sudo apt-get update
sudo apt-get install libpigpiod-if2-1 libsdl2-2.0-0
```

#### Run
```bash
# Start pigpio daemon if not running
sudo pigpiod

# Run balancer
./balancer_pi
```

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
