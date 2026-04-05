# Reflection System Design

This document describes the reflection pipeline now hosted directly in `balancer_bot`.

## Overview

The project uses GCC trunk with `-freflection` for two isolated generators:

- `generate_balancer_bindings.cpp`
  Produces [tests/python/generated_balancer.py](/workspaces/balancer_bot/tests/python/generated_balancer.py)
- `generate_balancer_docs.cpp`
  Produces [doc/ipc/protocol.md](/workspaces/balancer_bot/doc/ipc/protocol.md), [doc/ipc/ipc_flow.dot](/workspaces/balancer_bot/doc/ipc/ipc_flow.dot), [doc/ipc/ipc_flow.svg](/workspaces/balancer_bot/doc/ipc/ipc_flow.svg), and [doc/ipc/ipc_flow.png](/workspaces/balancer_bot/doc/ipc/ipc_flow.png)

The source of truth is the C++ message schema in [messages/msg_base.h](/workspaces/balancer_bot/messages/msg_base.h), [messages/core_msgs.h](/workspaces/balancer_bot/messages/core_msgs.h), [messages/simulation_msgs.h](/workspaces/balancer_bot/messages/simulation_msgs.h), [messages/autonomous_msgs.h](/workspaces/balancer_bot/messages/autonomous_msgs.h), and [messages/balancer_msgs.h](/workspaces/balancer_bot/messages/balancer_msgs.h).

## Notes

- Reflection is isolated in [cmake/reflection.cmake](/workspaces/balancer_bot/cmake/reflection.cmake) so the main build stays on standard C++20.
- `DOC_DESC(...)` is guarded by `REFLECT_DOCS`, which keeps regular tooling like `clangd` from choking on the annotation syntax.
- The docs generator currently reflects the balancer-facing SIL/IPC surface, not the full historical `reflect_pytest` protocol.
