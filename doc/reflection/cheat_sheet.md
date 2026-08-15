# Reflection Quick Reference

[Docs Portal](../index.md) | [Reflection System](system.md) | [IPC Protocol](../ipc/protocol.md)

This is the short project-oriented reflection reference for `balancer_bot`. It describes the
current generator layout, not the earlier `reflect_pytest` prototype.

## Contents

- [Project workflow](#what-reflection-is-used-for) — generated outputs, source files, targets, and annotations.
- [Supported wire shapes](#supported-reflected-wire-shapes) — generated type support and compile-time limits.
- [Practical notes](#practical-notes) — compiler, cross-build, and editor boundaries.
- [C++26 language reference](#language-reference) — a longer background tutorial; not required for normal project browsing.

## What Reflection Is Used For

The project uses standard C++26 static reflection, compiled with GCC 16.1.0 because it is the
current available compiler implementation, to generate:

- Python UDP bindings: `tests/python/generated_balancer.py`
- generated IPC docs: `doc/ipc/protocol.md`
- the IPC flow diagram: `doc/ipc/ipc_flow.{dot,svg,png}`

## Where to Look

- registry and helpers:
  - `src/reflection/balancer_message_registry.h`
  - `src/reflection/reflection_common.h`
- generators:
  - `src/reflection/generate_balancer_bindings.cpp`
  - `src/reflection/generate_balancer_docs.cpp`
- build wiring:
  - `cmake/reflection.cmake`

## Build Targets

```bash
cmake --build build --target balancer_bindings
cmake --build build --target balancer_docs
```

On host builds, `balancer_reflection` depends on both.

## What to Annotate

Use `DOC_DESC(...)` on:

- payload structs
- services
- other reflected types that should appear with human-readable descriptions in generated docs

Use `MessageTraits<MsgId::...>` to bind each message ID to its payload type. Message names are
derived from the reflected `MsgId` enumerators.

Use `ipc::MsgList<...>` in services to declare:

- `Publishes`
- `Subscribes`

## Important Project Rule

Bindings and docs are not generated from the same message subset:

- Python bindings follow the UDP contract
- generated IPC docs describe the runtime message graph, including internal-only messages when relevant

That split is intentional. For example, `MotorFeedback` belongs in the protocol docs but not in the Python UDP binding surface.

## Supported Reflected Wire Shapes

The generators intentionally support a small, explicit wire-schema surface. The check marks below
describe current generated support; unsupported shapes fail during generator compilation.

| Reflected shape | Python bindings | IPC docs | Constraint |
| --- | :---: | :---: | --- |
| `bool`, integral, `float`, or `double` scalar | ✅ | ✅ | Integral values must be 1, 2, 4, or 8 bytes. |
| Enum scalar | ✅ | ✅ | The enum storage must be 1, 2, 4, or 8 bytes. |
| `T[N]` or `std::array<T, N>` | ✅ | ✅ | `N` must be non-zero; `T` is a supported scalar, enum, or reflected struct. |
| Reflected nested struct | ✅ | ✅ | Must be non-empty, standard-layout, and trivially copyable. |
| One-dimensional array of reflected structs | ✅ | ✅ | Nested arrays are not supported. |
| Nested arrays | ❌ | ❌ | Rejected during generator compilation. |
| Pointers, strings, optionals, variants, and other wrappers | ❌ | ❌ | Rejected during generator compilation. |

## Practical Notes

- reflection uses standard C++26 features with GCC 16.1.0 at `/usr/local/gcc-16.1.0/bin/g++`
- cross-builds skip reflection targets
- normal runtime code still builds as C++23
- `REFLECT_DOCS` gates the annotation path so editors like `clangd` stay usable

## Language Reference

The detailed C++26 papers are archived under:

- `doc/archive/reflection/P2996.html`
- `doc/archive/reflection/P3096.html`
- `doc/archive/reflection/P3293.html`
- `doc/archive/reflection/P3394.html`
- `doc/archive/reflection/P3491.html`
- `doc/archive/reflection/P3560.html`

Use those when you need language-level detail. Use the rest of this handbook when you need project-level detail.

---

## Introduction to C++26 Reflection

C++26 reflection brings native, compiler-supported metaprogramming without macros or external code generators. It allows you to inspect types, structs, and enums at compile time.

### 1. Basic Introspection Core

C++26 reflection relies on the core opaque type `std::meta::info`, representing a handle to a program element at compile time.

**Reflecting on an entity:** Use the prefix `^^` operator.
```cpp
#include <meta>

constexpr std::meta::info class_info = ^^MyClass;
constexpr std::meta::info int_info = ^^int;
```

**Splicing (un-reflecting):** Use the `[: ... :]` syntax to convert a `std::meta::info` back into a grammatical element (like a type or an expression).
```cpp
using MyType = [: class_info :]; // Same as using MyType = MyClass;

constexpr std::meta::info val_info = ^^my_const_var;
int x = [: val_info :];
```

### 2. Common Metafunctions (`<meta>`)

Metafunctions in `std::meta` are `consteval` functions that process `std::meta::info` handles.

```cpp
namespace std::meta {
    // Queries
    consteval string_view name_of(info x);
    consteval string_view identifier_of(info x);
    consteval string_view display_string_of(info x);
    consteval bool has_identifier(info x);
    consteval source_location source_location_of(info x);
    consteval bool is_type(info x);
    consteval bool is_class(info x);
    consteval bool is_function(info x);
    consteval bool is_enum(info x);

    // Retrieving members
    consteval vector<info> members_of(info x);
    consteval vector<info> nonstatic_data_members_of(info x);
    consteval vector<info> enumerators_of(info x);
    consteval vector<info> base_classes_of(info x);
}
```

### 3. Compile-Time Iteration

C++26 includes `template for` loops (P1306), but the current GCC 16.1.0 implementation used by this
project does not expose that form in the generator toolchain. The generators therefore use
`index_sequence` expansion instead.

Instead, you must iterate using standard immediate functions and lambda unrolling across `std::index_sequence`.

**Iterating over an enum safely:**
```cpp
template <typename E>
void print_enum(E c) {
    constexpr auto enumerators = std::meta::enumerators_of(^^E);
    constexpr size_t N = enumerators.size();
    
    [&]<std::size_t... Is>(std::index_sequence<Is...>) {
        (..., [&] {
             // Extract the specific compile-time array element
             constexpr auto e = enumerators[Is];
             if (c == [:e:]) {
                 std::cout << std::meta::identifier_of(e) << "\n";
             }
        }());
    }(std::make_index_sequence<N>{});
}
```
*Note: Because `std::vector` returned from `<meta>` cannot escape a consteval context seamlessly in all template instantiations, it's highly recommended to convert them into `std::array` wrappers first.*

### 4. Code Generation & Types

You can use the splice operator to automatically generate boilerplate like struct padding, recursive deserialization, or bindings manually.

```cpp
template <typename T>
void generate_struct_fields() {
    constexpr auto members = std::meta::nonstatic_data_members_of(^^T);
    // Iterate manually via an index sequence
    [&]<std::size_t... Is>(std::index_sequence<Is...>) {
        (..., [&] {
             constexpr auto field = members[Is];
             constexpr auto type = std::meta::type_of(field);
             using FieldType = typename [:type:]; // Rehydrate to a real type
             
             std::cout << "Field: " << std::meta::identifier_of(field) 
                       << " Size: " << sizeof(FieldType) << "\n";
        }());
    }(std::make_index_sequence<members.size()>{});
}
```

### 5. Annotations (P3394R4)

C++26 introduces a mechanism for user-defined attributes that can be queried at compile-time via reflection. Unlike standard attributes (like `[[nodiscard]]`), these can carry data.

**Syntax:** `[[= expression ]]`
- The expression must be a constant expression.
- Multiple annotations can be attached to a single entity.

**Usage in the reflection generator:**
We use a custom struct `doc::Desc` to attach human-readable descriptions to messages and fields. These are abstracted behind the `DOC_DESC` macro.

```cpp
#ifdef REFLECT_DOCS
  #define DOC_DESC(str) [[= doc::Desc(str) ]]
#else
  #define DOC_DESC(str)
#endif

struct [[DOC_DESC("This is a reflected message")]] MyMessage {
    [[DOC_DESC("Primary key")]] uint32_t id;
};
```

> [!NOTE]
> **LSP Compatibility**: The `#ifdef` guard is specifically used to prevent LSPs (like `clangd`) from reporting errors on the `[[= ... ]]` syntax. The annotation path is compiled only by the C++26 reflection generator target, allowing the rest of the codebase to remain C++23-compatible while enabling rich metadata for that target.

**Querying Annotations:**
Use `std::meta::annotations_of(info)` to retrieve a vector of handles to the annotations.

```cpp
template <std::meta::info R>
consteval std::string_view get_description() {
    for (auto attr : std::meta::annotations_of(R)) {
        if (std::meta::type_of(attr) == ^^doc::Desc) {
            return std::meta::extract<doc::Desc>(attr).text;
        }
    }
    return "";
}
```

---

### Summary of New Syntax
- `^^Entity` : Creates a `std::meta::info` handle representing `Entity`.
- `[: info_val :]` : Reifies (splices) a `std::meta::info` back into a C++ type or value construct.
- `std::meta::...` : Const-evaluated standard library metaprogramming tools (e.g. `identifier_of`).

---

## Architectural Usage Overview

How does the project apply C++26 reflection?

For deep-dives into the C++26 introspection generation pipeline, see **[Reflection System Design](./system.md)**.
Here is a high-level summary:

1. **Custom Annotations**: C++ structs are tagged with a `DOC_DESC("text")` macro. Reflection (`std::meta::annotations_of`) plucks these strings out to build documentation tables and Mermaid logic naturally from the compiler AST.
2. **Safe Identifier Fallbacks**: Advanced templated types can lack unique identifier lexemes. The
   project uses `std::meta::display_string_of` and the helpers in `src/reflection/reflection_common.h`
   where a stable display name is required.
3. **Array Wrappers**: The reflection helpers provide the project-specific wrappers needed to
   describe reflected arrays within the compiler’s constant-evaluation limits.
4. **Python + Markdown Generators**: Utilizing the index-sequence unrolling shown above across the `MsgId` enum, the framework recursively deduces structure sizes (`sizeof`), data variables, and C-to-Python primitives natively mapping types.
