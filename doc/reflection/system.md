# Reflection System

[Docs Portal](../index.md) | [IPC Protocol](../ipc/protocol.md) | [Reflection Quick Reference](cheat_sheet.md)

The reflection pipeline is what keeps the C++ message definitions, generated Python bindings, and generated IPC docs aligned.

## Purpose

The reflection system exists to preserve one source of truth:

- `MsgId`
- `MessageTraits`
- reflected payload structs in `src/messages/`
- reflected service metadata in the service headers

From that information, the build generates:

- `tests/python/generated_balancer.py`
- `doc/ipc/protocol.md`
- `doc/ipc/ipc_flow.dot`
- `doc/ipc/ipc_flow.svg`
- `doc/ipc/ipc_flow.png`

## Where the Generators Live

The project-specific generators are under:

- `src/reflection/generate_balancer_bindings.cpp`
- `src/reflection/generate_balancer_docs.cpp`
- `src/reflection/balancer_message_registry.h`
- `src/reflection/reflection_common.h`

The active reflection docs should describe those files, not an older prototype layout.

## Build Behavior

Reflection is host-only.

- the main project still builds under normal C++20
- the reflection targets are compiled separately with GCC trunk and `-freflection`
- cross-builds skip reflection artifacts

Current build targets:

- `balancer_bindings`
- `balancer_docs`
- `balancer_reflection`

On host builds, `balancer_reflection` is part of the default build.

## What Gets Generated

### Python Bindings

`tests/python/generated_balancer.py` is generated from the UDP-facing contract. It is derived from the messages that `UdpBridge` publishes or subscribes to.

That means:

- UDP-visible messages appear in the Python bindings
- internal-only bus messages such as `MotorFeedback` do not

### IPC Docs

`doc/ipc/protocol.md` documents the reflected balancer message set and service graph. The docs generator includes the runtime messages needed to describe the real internal topology, including internal-only service messages.

## Service Annotations

Service descriptions and payload descriptions come from `DOC_DESC(...)` annotations. The docs generator reads those annotations to populate the generated protocol reference.

This is why the service headers contain:

- `using Publishes = ipc::MsgList<...>`
- `using Subscribes = ipc::MsgList<...>`
- `DOC_DESC(...)`

## LSP and Compiler Reality

Reflection-specific syntax is only compiled in the generator targets. The rest of the codebase remains editor-friendly by guarding annotation support behind `REFLECT_DOCS`.

That split is deliberate:

- normal development stays on the standard project toolchain
- only the generators require GCC trunk reflection support

## Practical Workflow

Rebuild the generated artifacts with:

```bash
cmake -S . -B build
cmake --build build --target balancer_bindings
cmake --build build --target balancer_docs
```

Use the generated protocol docs for payload sizes, field tables, and graph topology. Use the human-written handbook for behavior, architecture, and operational guidance.
