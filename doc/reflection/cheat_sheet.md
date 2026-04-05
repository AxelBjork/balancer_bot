# C++26 Reflection Cheat Sheet & Summary

[Home](../../README.md) | [Architecture](../arch/design.md) | [System Design](./system.md) | [IPC Protocol](../ipc/protocol.md) | [Testing](../testing/sil_guide.md)

This document serves as a high-level summary and quick reference for the new static reflection features introduced in C++26 (P2996), specifically tailored to modern GCC snapshot environments. It also includes an architectural overview of how `reflect_pytest` utilizes these features.

## Approved Papers Reference

The following foundational papers are included in C++26 related to reflection. Full HTML documents are located in this directory (`/doc`).

| Paper ID | Title | Summary |
|----------|-------|---------|
| **[P2996R13](P2996.html)** | Reflection for C++26 | The core reflection proposal. Introduces `std::meta::info`, the `^^` operator, splicers (`[: :]`), and the fundamental `<meta>` header. |
| **[P3096R12](P3096.html)** | Function Parameter Reflection | Allows introspecting function parameters via reflection APIs. |
| **[P3293R3](P3293.html)**  | Splicing a base class subobject | Allows generating base class derivations programmatically. |
| **[P3394R4](P3394.html)**  | Annotations for reflection | Introduces a mechanism for user-defined attributes (annotations) that can be queried at compile-time. |
| **[P3491R3](P3491.html)**  | `define_static_` | Capability for defining static variables from reflection. |
| **[P3560R2](P3560.html)**  | Error handling in reflection | Clarifies error handling behaviors when using reflection metafunctions. |

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

### 3. Compile-Time Iteration (The GCC Trunk Way)

C++26 currently proposes `template for` loops (P1306), but **GCC Trunk does not yet implement them**. Attempting to write `template for (constexpr auto m : members)` will crash your build. 

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

**Usage in reflect_pytest:**
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
> **LSP Compatibility**: The `#ifdef` guard is specifically used to prevent LSPs (like `clangd`) from reporting errors on the `[[= ... ]]` syntax, which is currently only supported by GCC trunk with `-freflection`. This allows the rest of the codebase to remain clean and compatible with standard compilers while enabling rich metadata for the reflection generators.

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

How does `reflect_pytest` practically apply C++26 reflection?

For deep-dives into the C++26 introspection generation pipeline, see **[Reflection System Design](./system.md)**.
Here is a high-level summary:

1. **Custom Annotations**: C++ structs are tagged with a `DOC_DESC("text")` macro. Reflection (`std::meta::annotations_of`) plucks these strings out to build documentation tables and Mermaid logic naturally from the compiler AST.
2. **Safe Identifier Fallbacks**: Advanced templated types (`std::array<MotorSubCmd, 10>`) throw compiler exceptions when parsed using `std::meta::identifier_of` because they lack unique string lexemes. `reflect_pytest` handles this natively utilizing string stripping via `std::meta::display_string_of`.
3. **Array Wrappers**: `generator/common.h` ships with `EnumArrHolder` and `StructArrHolder` to bypass consteval allocation limits securely.
4. **Python + Markdown Generators**: Utilizing the index-sequence unrolling shown above across the `MsgId` enum, the framework recursively deduces structure sizes (`sizeof`), data variables, and C-to-Python primitives natively mapping types.
