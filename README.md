# balancer_bot

`balancer_bot` is a Raspberry Pi self-balancing robot project with a strong software spine: a tick-driven controller, a message-bus runtime, a deterministic direct simulator, and a Python SIL harness built from reflected C++ message definitions.

It is both a real robot project and a control/runtime playground. The same codebase supports:

- a hardware runtime on Raspberry Pi
- a UDP-driven SIL runtime for Python tests
- a fast direct simulator for stability work
- generated IPC docs and Python bindings from the C++ message definitions

## Why This Project Is Interesting

- **Tick-driven control**
  The balancing loop runs from an explicit `PhysicsTick`, which makes the control path easier to reason about in both hardware and simulation.
- **Service-oriented runtime**
  `TimeService`, `ImuService`, `ControlService`, `MotorService`, and `UdpBridge` are wired through a small internal message bus instead of ad hoc callback chains.
- **Deterministic simulation**
  The direct simulator can run representative scenarios, emit artifacts under `build/sim`, and act as the main software stability gate.
- **Generated interfaces**
  IPC docs and Python bindings are generated from the reflected message definitions and service metadata, which keeps the UDP contract and documentation aligned with the code.

## Quick Start

### Host Build and Tests

```bash
./build_cmake
pytest -q
```

### AFL++ Harness Validation

```bash
pytest --fuzz --build-only
python3 tools/run_afl.py --list
python3 tools/run_afl.py udp_sequence --output-dir build-afl/afl-udp -V 60
python3 tools/run_afl.py simulator_scenario --output-dir build-afl/afl-sim -V 300
sed -n '1,40p' build-afl/afl-sim/default/fuzzer_stats
find build-afl/afl-sim/default/queue -maxdepth 1 -type f | head
```

The seed corpus is generated on demand under `build-afl/fuzz-corpus/`; it is not checked into git.

### Standalone Simulator

```bash
./build/balancer_simulator
```

### Linearized Plant Audit

```bash
./build/balancer_plant_audit --all
```

### Timeline Analysis

```bash
python3 tools/analyze_timeline.py build/sim/realistic_neutral_hold_40s/timeline.csv --summary-json
```

### Live Raspberry Pi Telemetry Dashboard

Start the local dashboard on a laptop on the same network. It can start before the Pi is
available; set or test the Pi target from the page and it will retry telemetry registration.

```bash
python3 tools/telemetry_dashboard/server.py --pi rpi4
```

To start without selecting a Pi, omit `--pi`. Existing dashboard logs and simulator `timeline.csv`
files can be chosen with the page's **Choose CSV** button; uploads are temporary and discarded when
another source is selected or the server exits. `--csv` remains available as a startup shortcut:

```bash
python3 tools/telemetry_dashboard/server.py --csv build/sim/example/timeline.csv
```

Live telemetry uses only the Python standard library. CSV playback uses the shared pandas-backed
telemetry loader, so install `requirements-dev.txt` when using `--csv` or uploading a CSV.

Open `http://127.0.0.1:8080`. The dashboard is read-only, uses the existing UDP telemetry
stream, and intentionally claims the bridge's one active UDP peer; do not run it alongside a
SIL client or another UDP observer. Add `--listen-lan` only when the laptop's network is trusted.
`rpi4` may be the same SSH `Host` alias used by `ssh` and `scp`; the dashboard expands it through
your `~/.ssh/config` before sending UDP. Drag any chart to pan all plots together, wheel to zoom,
and use **Follow latest** to resume the live window.

On Windows CMD, use the Python launcher. If port `8080` is reserved, choose another local port:

```cmd
py tools\telemetry_dashboard\server.py --pi rpi4 --port 8081
```

When running the dashboard on the host that already has working `scp` and non-interactive `sudo`
access on the Pi, the page can deploy `build-pi/balancer_pi` plus `pid.conf`, start the bot, or
abort the process it launched. Build the binary first with `./build_cmake OFF`.

Every valid telemetry packet is logged automatically with a fixed CSV schema under `data/server/`.
The dashboard keeps the ten newest files and rotates an active file at 128 MiB.

### Raspberry Pi Cross-Build

```bash
./build_cmake OFF
```

## Simplified BOM

| Area | Main Parts |
| --- | --- |
| Compute | Raspberry Pi 4 |
| Sensing | ISM330DHCX IMU breakout + Qwiic SHIM/cable |
| Actuation | 2x NEMA-17 steppers + Waveshare Stepper Motor HAT |
| Power | 3S 18650 battery pack |
| Structure | B-robot EVO 2 frame + local modified print files |

The detailed parts list, frame assets, and wiring notes live in [hardware/README.md](hardware/README.md).

## Architecture At a Glance

![IPC Flow](doc/ipc/ipc_flow.svg)

The generated flow graph above is the best quick picture of the system:

- `TimeService` drives the control timeline with `PhysicsTick`
- `ImuService` publishes the robot state estimate
- `ControlService` converts tick + IMU + input into `MotorTargets` and telemetry
- `MotorService` closes the loop with real motor feedback on hardware
- `UdpBridge` exposes the SIL/test boundary without becoming the core runtime

## Project Layout

- `src/`
  runtime, services, control code, reflection generators, and platform helpers
- `tests/`
  C++ tests, pytest SIL tests, simulator helpers, and captured reference data
- `hardware/`
  physical build assets, frame files, and hardware reference notes
- `doc/`
  handbook pages plus generated IPC docs
- `pid.conf`
  default hardware and simulator PID profiles

## Learn More

- [Documentation Portal](doc/index.md)
- [Project Overview](doc/overview.md)
- [Runtime Architecture](doc/arch/runtime.md)
- [Control / Plant Notes](doc/arch/control_plant.md)
- [Testing Strategy](doc/testing/strategy.md)
- [Running on Pi](doc/Running_on_Pi.md)
- [Current Status](doc/status.md)
- [IPC Protocol Reference](doc/ipc/protocol.md)
