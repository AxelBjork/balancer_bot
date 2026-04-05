# SIL Testing Guide

This repo’s Python harness acts as the network peer for `sil_app`. Tests talk to the C++ process over UDP using the generated bindings in [tests/python/generated_balancer.py](/workspaces/balancer_bot/tests/python/generated_balancer.py) and the local helper in [tests/python/udp_client.py](/workspaces/balancer_bot/tests/python/udp_client.py).

## Overview

- `tests/python/conftest.py` owns the `pytest --build` workflow and manages the `sil_app` process lifetime.
- `tests/python/udp_client.py` is the low-level UDP helper for the `[uint16_t msgId][payload]` wire format.
- `tests/python/test_udp_bridge.py` is the transport smoke test.
- `tests/python/test_sil_loop.py` is the control-loop smoke test.

## Test Loop

1. `pytest --build` configures and builds the C++ targets, then runs `ctest`.
2. The session fixture launches `build/sil_app`.
3. Each Python test registers with `UdpBridge`, injects messages, and asserts on outbound traffic.

## Related Docs

- [IPC Protocol](../ipc/protocol.md)
- [Reflection System](../reflection/system.md)
