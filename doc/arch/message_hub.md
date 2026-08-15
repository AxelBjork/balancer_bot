# MessageBus and UDP Bridge

This project uses a deliberately small message-bus abstraction. It is not a general broker or
background queue. The runtime must use typed synchronous dispatch so service ownership and control
ordering remain explicit.

## Bus Model

The bus is a synchronous dispatch boundary. Publishing must deliver in the publishing thread; the
bus must not create worker threads or queue messages. Nested publications run immediately in the
same dispatch chain, and separate producer threads wait until that chain completes.

The application supplies a context and dispatcher when it constructs the bus:

```cpp
using DispatchFn = void (*)(void* context, MsgId id, const void* payload);
ipc::MessageBus bus{&application, &dispatch};
```

The dispatcher owns the runtime wiring. It routes each message to the services that subscribe to
that ID:

```cpp
inline void app_dispatcher(void* ctx, MsgId id, const void* payload) {
  auto* s = static_cast<AppServices*>(ctx);
  ipc::dispatch_to_services(id, payload, s->is, s->ms, s->cs, s->udp, s->ts);
}
```

Components declare their allowed message directions in the type itself:

```cpp
using Publishes = ipc::MsgList<MsgId::MotorTargets, MsgId::SystemTelemetry>;
using Subscribes = ipc::MsgList<MsgId::PhysicsTick, MsgId::ImuData>;
```

`TypedPublisher` turns the declaration into a compile-time publishing rule:

```cpp
ipc::TypedPublisher<sil::ControlService> publisher{bus};
publisher.publish<MsgId::SystemTelemetry>(telemetry);
```

The equivalent raw entry point is reserved for the UDP bridge. It applies the same `Publishes`
list and payload-size check before converting bytes into a typed message:

```cpp
ipc::TypedPublisher<ipc::UdpBridge> udp_publisher{bus};
udp_publisher.publish_if_authorized(msg_id, payload_bytes, payload_size);
```

The bus is the callback-synchronization boundary: service handlers invoked through it may assume
serialized entry and should not add another lock solely for bus delivery. Components whose public
APIs are also called outside bus dispatch must synchronize those APIs independently.

## Authorization Model

Each component declares:

- `Publishes = MsgList<...>`
- `Subscribes = MsgList<...>`

`TypedPublisher` enforces `Publishes` at compile time for normal publishing, and
`publish_if_authorized()` uses the same publish list to gate raw UDP ingress.

This is what lets `UdpBridge` accept only the message IDs it is supposed to inject. The complete
reflected message inventory is maintained in the generated protocol reference rather than
duplicated in this guide. The production `UdpBridge` is the port-9000 boundary; the simulator's
port-9001 scenario endpoint is a separate transport and is described in the
[project overview](../overview.md#messages-and-boundaries).

## UDP Bridge Contract

`UdpBridge` is the external runtime boundary for the production Pi application. The telemetry
server is its primary peer, while SIL and other authorized clients use the same contract.

### UDP Ingress

The bridge must receive datagrams on port `9000`. Each datagram consists of a reflected message ID
followed by its payload. The bridge must publish an inbound payload only when its ID is authorized
by `UdpBridge::Publishes` and its payload size matches the reflected message definition.

The first responsibility of a client is peer registration. The bridge maintains one active peer;
the newest sender becomes the return path for outbound traffic. A telemetry server or SIL client
must therefore stop before another client takes ownership of the peer.

### UDP Egress

The bridge must forward every message authorized by `UdpBridge::Subscribes` to the active peer.
On the production path this includes controller outputs and system telemetry. Simulator telemetry
and terminal status are emitted by the separate port-9001 scenario endpoint, not by a simulator
service hidden inside the production bridge.
`MotorFeedback` remains internal-only unless the reflected bridge contract changes.

## Ownership and Wire Rules

Internal handlers must treat a bus payload as valid only for the duration of synchronous dispatch.
The bridge must validate the reflected message ID and payload size before publishing network data.
Outbound datagrams must use the reflected ID-plus-payload format described by the generated protocol
reference. Performance properties such as allocation or copy counts are implementation details, not
runtime guarantees.

## Why This Fits the Project

The runtime has a small fixed set of services, explicit dispatcher logic, and a single external UDP
boundary for the production runtime; the simulator has a separate endpoint on port `9001`. That
makes the lightweight bus a good fit:

- no discovery layer to debug
- no background queue ownership ambiguity
- easy to reason about in tests
- easy to reflect for generated docs and Python bindings

For the normative runtime view, read [Runtime Architecture](runtime.md). For the exact topology and
payload reference, read [IPC Protocol Reference](../ipc/protocol.md). The old send-path assembly
analysis is historical and is not an active runtime contract.
