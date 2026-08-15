# Host Setup and First Build

This page is the host-side starting point for building and checking `balancer_bot`. It does not
replace the [Raspberry Pi guide](Running_on_Pi.md), which covers cross-building, deployment, and
physical bring-up.

## Host prerequisites

The reproducible development environment is described by the
[`.devcontainer/Dockerfile`](../.devcontainer/Dockerfile). A manual host setup needs the following:

- CMake 3.18 or newer and a C++23 compiler for the normal project targets.
- GCC 16.1.0 with `-freflection` support for the host-only reflection targets. The project uses
  `/usr/local/gcc-16.1.0/bin/g++` automatically when that compiler is present.
- Graphviz `dot` when building the generated IPC documentation and flow graph.
- SDL2 development headers and libraries. The normal host library compiles the Xbox controller
  implementation even when tests use stubs for pigpio; SDL2 is not only a production-runtime
  dependency.
- Python 3 with the packages in [`requirements-dev.txt`](../requirements-dev.txt): pandas,
  matplotlib, and pytest.
- Network access during a first configure if the CMake `FetchContent` cache does not already contain
  GoogleTest `v1.14.0`.

The normal host test build uses a pigpio stub, but physical deployment and the Pi toolchain have
additional ARM, pigpio, and target-library requirements described in the [Pi guide](Running_on_Pi.md).
The devcontainer is the concrete provisioning path for the unusual GCC reflection compiler; a
manual host needs an equivalent GCC 16.1.0 toolchain or must deliberately build without reflection
artifacts.

## First host check

Install the Python dependencies, then run the repository’s standard gate:

```bash
python3 -m pip install -r requirements-dev.txt
pytest --build
```

The `--build` option configures the host tree with tests and reflection artifacts enabled, builds
the C++ tests, simulator, SIL executable, and reflection targets, runs CTest, and then allows the
Python suite to run. It is the full host workflow, not merely Python test collection.

If the build is already available and only the Python layer is needed, use `pytest` without
`--build`. A missing executable is a setup failure; the Python fixtures explain which build command
to run.

## Useful host targets

After a successful build:

```bash
./build/balancer_simulator
./build/balancer_simulator --catalog-json
cmake --build build --target balancer_plant_audit
./build/balancer_plant_audit --all
```

Use the [testing strategy](testing/strategy.md) for simulator transfer validation, artifacts, SIL
tests, and fuzz checks. Use the [reflection system](reflection/system.md) when you need to inspect
or deliberately refresh generated bindings and IPC documentation.

## Reporting a verification result

Build and test claims are tied to the invocation that produced them. Before reporting a result,
record the commit and PID profile digest:

```bash
git rev-parse HEAD
sha256sum pid.conf
```

Do not turn CTest or pytest collection counts into pass claims. If a host build regenerates tracked
interfaces, inspect those changes and preserve the source/generated ownership rules in the
[reflection documentation](reflection/system.md).
