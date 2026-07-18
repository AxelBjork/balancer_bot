# Documentation Portal

This is the main handbook for `balancer_bot`. Use it as the human-written guide to the project, and use the generated IPC docs as the source of truth for payload layout and message topology.

## Start Here

1. Read [Project Overview](overview.md) for the binaries, runtime modes, and config files.
2. Read [Runtime Architecture](arch/runtime.md) for how the services and message bus fit together.
3. Read [Testing Strategy](testing/strategy.md) for the C++ tests, SIL flow, and simulator scenarios.
4. Read [Running on Pi](Running_on_Pi.md) for cross-build, deployment, and bring-up.
5. Read [Current Status](status.md) for confidence level and known caveats.

## Key References

- [Generated IPC Protocol](ipc/protocol.md)
- [IPC Flow Diagram](ipc/ipc_flow.svg)
- [MessageBus and UDP Bridge](arch/message_hub.md)
- [Reflection System](reflection/system.md)
- [Reflection Quick Reference](reflection/cheat_sheet.md)
- [SIL Guide](testing/sil_guide.md)
- [Control and Simulator Notes](notes/control_and_simulator.md)
- [Hardware Reference](../hardware/README.md)

## Generated vs Human-Written Docs

- `doc/ipc/` is generated from the reflected message registry and service annotations.
- Everything else in `doc/` is hand-written project guidance.
- `doc/archive/` contains historical or reference-only material that is not part of the active handbook.
