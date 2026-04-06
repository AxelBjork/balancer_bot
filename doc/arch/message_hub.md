# MessageBus and UDP Bridge

This project uses a deliberately small message-bus abstraction. It is not a general broker or background queue. It is a typed synchronous dispatch mechanism that lets the runtime entrypoint wire the services together explicitly.

## Bus Model

`MessageBus` stores:

- a context pointer
- one dispatcher function

Publishing a message is just:

1. `TypedPublisher<Component>::publish<Id>(payload)`
2. `MessageBus::dispatch(Id, &payload)`
3. the application dispatcher routes the payload to the relevant services

That means delivery is synchronous and happens in the publishing thread. The bus itself does not create threads or store queued messages.

## Authorization Model

Each component declares:

- `Publishes = MsgList<...>`
- `Subscribes = MsgList<...>`

`TypedPublisher` enforces `Publishes` at compile time for normal publishing, and `publish_if_authorized()` uses the same publish list to gate raw UDP ingress.

This is what lets `UdpBridge` accept only the message IDs it is supposed to inject:

- `PhysicsTick`
- `JoystickCommand`
- `ImuData`

## UDP Bridge Contract

`UdpBridge` is the network edge for the SIL harness.

### UDP Ingress

The bridge receives datagrams on port `9000`, reads the leading `uint16_t` message ID, and republishes the payload only if the ID is present in `UdpBridge::Publishes`.

### UDP Egress

The bridge forwards these outbound bus messages:

- `ImuData`
- `MotorTargets`
- `SystemTelemetry`

`MotorFeedback` remains internal-only and is not exposed to Python.

## Data Copy Story

### Internal C++ Path

Internal publishing passes a pointer to an already-typed payload. No extra copy is introduced by `MessageBus` itself.

### C++ to UDP

`UdpBridge` uses `sendmsg` with two `iovec` entries:

- the `uint16_t` message ID
- the trivially-copyable payload bytes

There is no extra user-space concatenation buffer on the send path.

### UDP to C++

On ingress, the bridge receives raw bytes, checks the message ID against its allowed publish list, copies the payload into a stack-allocated typed struct with `std::memcpy`, and republishes it on the internal bus.

## Why This Fits the Project

The runtime has a small fixed set of services, explicit dispatcher logic, and a narrow UDP boundary. That makes the lightweight bus a good fit:

- no discovery layer to debug
- no background queue ownership ambiguity
- easy to reason about in tests
- easy to reflect for generated docs and Python bindings

For the full runtime view, read [Runtime Architecture](runtime.md). For the generated topology and payload reference, read [IPC Protocol Reference](../ipc/protocol.md). For the send-path deep dive, read [Send Path Assembly Analysis](send_path_asm_analysis.md).
