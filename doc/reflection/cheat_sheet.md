# C++26 Reflection Cheat Sheet

## Core ideas

- `^^T`
  Reflect a type or entity into `std::meta::info`
- `[:info:]`
  Splice reflected information back into code
- `std::meta::nonstatic_data_members_of(^^T, ctx)`
  Enumerate struct fields
- `std::meta::enumerators_of(^^E)`
  Enumerate enum values
- `std::meta::identifier_of(info)`
  Get the short identifier name
- `std::meta::display_string_of(info)`
  Get a compiler-formatted display name
- `std::meta::annotations_of(info)`
  Read attached annotations such as `DOC_DESC`

## In this repo

- [generator/generate_balancer_bindings.cpp](/workspaces/balancer_bot/generator/generate_balancer_bindings.cpp)
  Generates Python bindings
- [generator/generate_balancer_docs.cpp](/workspaces/balancer_bot/generator/generate_balancer_docs.cpp)
  Generates Markdown and Graphviz docs
- [messages/msg_base.h](/workspaces/balancer_bot/messages/msg_base.h)
  Defines `DOC_DESC` and the top-level `MsgId`

## Regeneration

```bash
cmake -S . -B build
cmake --build build --target balancer_bindings
cmake --build build --target balancer_docs
```
