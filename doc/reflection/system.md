# Reflection System Design

[Home](../../README.md) | [Architecture](../arch/design.md) | [Cheat Sheet](./cheat_sheet.md) | [IPC Protocol](../ipc/protocol.md) | [Testing](../testing/sil_guide.md)

This document details the C++26 static reflection (P2996) implementation used within `reflect_pytest` to automatically generate Python bindings and Markdown documentation from the core C++ message definitions. For a high-level overview of the feature set, see the [Reflection Cheat Sheet](./cheat_sheet.md).

### 🤖 Agent Note
> [!IMPORTANT]
> To effectively reason about the message bus, AI agents MUST review the visual architecture in **[ipc_flow.png](../ipc/ipc_flow.png)**. This diagram provides the definitive mapping of service relationships and wire-protocol routing that source code alone cannot easily convey.

## Overview

The reflection system is built to ensure a **Single Source of Truth**: the C++ message headers (`messages.h` and `traits.h`). From these definitions, the build system uses a GCC snapshot compiler capable of C++26 reflection (`-freflection`) to compile highly introspective generators. 

Instead of writing boilerplate serialization code or manual documentation, the reflection pipeline analyzes the source code at compile time, extracting type information, struct fields, enum values, and custom annotations to emit target strings (Python code and Markdown text).

## Core Utilities (`generator/common.h`)

The foundation of the reflection system resides in `generator/common.h`. It leverages the `std::meta` namespace to interact with the Abstract Syntax Tree (AST).

### 1. Annotations (`get_desc`)

To embed human-readable descriptions directly into the structs, the project uses a custom C++ attribute macro `DOC_DESC(...)`.

```cpp
template <std::meta::info R>
consteval doc::Desc get_desc() {
  for (std::meta::info a : std::meta::annotations_of(R)) {
    if (std::meta::type_of(a) == ^^doc::Desc) {
      return std::meta::extract<doc::Desc>(a);
    }
  }
  return doc::Desc("");
}
```
This function safely extracts `doc::Desc` instances attached to mirrored types, skipping unrelated compiler attributes.

### 2. Enum and Struct Reflection

Iterating over fields and enumerators requires converting `std::vector<std::meta::info>` (returned by reflection) into `std::array` at compile time, since `std::vector` allocations cannot easily escape a `consteval` context into template instantiations.

- **`get_enum_size<E>()`** / **`EnumArrHolder`**: Computes the size and builds a fixed array of `std::meta::info` for each enum element.
- **`get_fields_size<T>()`** / **`StructArrHolder`**: Extracts the member variables (`nonstatic_data_members_of`) to allow automated serialization iterations.

### 3. Safe Type Naming

C++ templates do not possess standard "identifiers" in the same way regular structs do. Thus `std::meta::identifier_of` acts dangerously on instantiations like `std::array<MotorSubCmd, 10>`.

- **`get_cxx_type_name<T>()`**: Safely falls back to `std::meta::display_string_of()` to parse out cleanly formatted C++ types, stripping internal namespaces (`ipc::`, `sil::`).
- **`get_python_type_name<T>()`**: Aggressively sanitizes template characters (`<`, `>`, `,`) into underscores (`_`) to generate valid Python class names (e.g., `MotorSequencePayloadTemplate_10`).

## Code Generators

### Python Bindings (`generator/python_code_generator.h`)

The Python Code Generator iterates through the C++ structs to automatically write data classes equipped with `pack_wire` and `unpack_wire` methods.

1. **Format String Mapping**: Standard primitives are mapped to Python `struct` module format characters (e.g., `uint32_t` -> `I`, `int16_t` -> `h`).
2. **Recursive Array Handling**: Standard arrays (`std::array`) and primitive bounds (`char[N]`) are automatically detected and unrolled. Bytes are packed natively, whereas complex arrays of nested structs fall back to sequential duck-typing evaluation during runtime packing.
3. **Enum Generation**: C++ Enums are emitted as Python `IntEnum` classes, mapping `[:e:]` (spliced value) to the literal names.

### Documentation Generation (`generator/doc_generator.h`)

The Documentation Generator executes the same reflection techniques but outputs GitHub Flavored Markdown ([protocol.md](../ipc/protocol.md)).

1. **Payload Tables**: Introspects offsets and byte sizes using `sizeof(T)` to draw deterministic binary memory layouts for the IPC wire protocol.
2. **Component Architecture via Metaprogramming**: Using variadic template inspection across `AllComponents` (a tuple of all services), the generator extracts `typename Component::Publishes` and `typename Component::Subscribes`. 
3. **Dynamic Mermaid Graphs**: Component publish/subscribe relationships are topologically sorted and emitted as a dynamic flowchart predicting internal system routing and UDP bridge mapping.

## Limitations & Edge Cases

- Currently relies strictly on GCC Trunk/Snapshot capabilities, as Clang and MSVC do not yet fully implement P2996.
- Fixed-size strings (`char[N]`) and standard arrays are correctly supported, but dynamic structures (like `std::vector` or `std::string`) are not supported across the binary IPC wire, triggering static assertions.
