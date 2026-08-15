# Reflection System

[Docs Portal](../index.md) | [IPC Protocol](../ipc/protocol.md) | [Reflection Quick Reference](cheat_sheet.md)

The reflection pipeline is what keeps the C++ message definitions, generated Python bindings, and generated IPC docs aligned.

## Purpose

The reflection system exists to preserve one source of truth:

- `MsgId`
- `MessageTraits`
- reflected payload structs in `src/messages/`
- reflected service metadata in the service headers

The authority and consumer chain is:

| Authority or stage | Role | Main consumers/outputs |
| --- | --- | --- |
| `src/messages/msg_base.h` | `MsgId` values and reflection description helper | Message registry and generators |
| `src/messages/core_msgs.h`, `src/messages/balancer_msgs.h` | Payload structs and `MessageTraits` payload bindings | Runtime bus, UDP serialization, registry |
| Service headers and `src/ipc/udp_bridge.h` | `Publishes`, `Subscribes`, endpoint authorization, and descriptions | Runtime dispatch and docs generator |
| `src/reflection/balancer_message_registry.h` | Selects reflected messages and derives enum names/sizes | Python and protocol generators |
| `cmake/reflection.cmake` | Compiles generators and names their outputs | `balancer_bindings`, `balancer_docs`, `balancer_reflection` |
| Generated outputs | Python wire bindings, IPC Markdown, and graph assets | SIL/dashboard/tools, exact protocol lookup, topology review |

The registry and generators are part of the derivation pipeline; they are not replacements for the
payload definitions or service/transport declarations. The generated Python binding follows the
UDP-visible subset, while the generated IPC document may include internal-only messages and the
simulator message family needed to describe the broader runtime graph.

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

- the main project builds under normal C++23
- the reflection targets are compiled separately as standard C++26 with GCC 16.1.0 and `-freflection`
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

The generated dataclass `pack_wire()` and `unpack_wire()` methods are the supported external
serialization API. They encode reflected field offsets and padding with little-endian Python
`struct` formats; callers must not treat dataclass construction or ad-hoc byte packing as a
separate protocol. The UDP bridge forwards the corresponding reflected C++ object bytes and
therefore requires little-endian storage, IEEE-754 `float`/`double`, and one-byte `bool` on the
supported target ABI. Those assumptions are checked in the bridge at compile time.

### IPC Docs

`doc/ipc/protocol.md` documents the reflected balancer message set and service graph. The docs generator includes the runtime messages needed to describe the real internal topology, including internal-only service messages.

The supported wire-shape matrix and generator limitations are summarized in the [reflection quick reference](cheat_sheet.md#supported-reflected-wire-shapes).

Generated descriptions are bounded by the current `doc::Desc` annotation storage. If a generated
paragraph ends abruptly, treat that as a generated-artifact/source limitation to investigate; do
not repair `doc/ipc/protocol.md` by hand. The current production UDP endpoint remains the
`UdpBridge` on port `9000`; simulator control on port `9001` is a separate hand-written transport
that reuses reflected payload types.

## Service Annotations

Service descriptions and payload descriptions come from `DOC_DESC(...)` annotations. The docs generator reads those annotations to populate the generated protocol reference.

This is why the service headers contain:

- `using Publishes = ipc::MsgList<...>`
- `using Subscribes = ipc::MsgList<...>`
- `DOC_DESC(...)`

## LSP and Compiler Reality

Standard C++26 reflection syntax is only compiled in the generator targets. The rest of the codebase remains editor-friendly by guarding annotation support behind `REFLECT_DOCS`.

That split is deliberate:

- normal development stays on the standard project toolchain
- only the generators require the GCC reflection toolchain

## Practical Workflow

Rebuild the generated artifacts with:

```bash
cmake -S . -B build
cmake --build build --target balancer_bindings
cmake --build build --target balancer_docs
```

Use the generated protocol docs for payload sizes, field tables, and graph topology. Use the human-written handbook for behavior, architecture, and operational guidance.

## Controlled Synchronization Check

The generators write into the source tree: `tests/python/generated_balancer.py` and the outputs
under `doc/ipc/`. A normal host `pytest --build` includes `balancer_reflection`, while a
cross-compiling build skips these host-only targets. Do not edit generated outputs by hand.

This is a mutating operation, not a read-only drift check. Do not run it in a dirty worktree that
contains valuable generated edits. Use a disposable clean checkout or another explicitly preserved
workspace when deliberately checking synchronization, then inspect the result:

```bash
cmake --build build --target balancer_reflection
git diff -- tests/python/generated_balancer.py doc/ipc/
```

The diff is evidence of generated output changes, not a place to hand-maintain corrections. Preserve
any pre-existing user changes, and treat an unexpected generated diff as a source/build-state issue
to investigate. Review the human-written handbook for behavior and the generated protocol only for
the exact reflected interface. The repository currently has no out-of-tree comparison target, so a
reviewer who must remain read-only should inspect the generator and its declared outputs instead of
running it. The generated `doc/ipc/ipc_flow.png` is ignored by Git, so a complete output audit also
needs `git status --ignored --short -- doc/ipc/` rather than `git diff` alone.
