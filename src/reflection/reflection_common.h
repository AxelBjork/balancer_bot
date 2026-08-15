#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <meta>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>

#include "messages/balancer_msgs.h"
#include "messages/core_msgs.h"
#include "messages/msg_base.h"

namespace balancer_reflection {

using namespace std::string_view_literals;

template <std::meta::info R>
consteval doc::Desc get_desc() {
  for (std::meta::info a : std::meta::annotations_of(R)) {
    if (std::meta::type_of(a) == ^^doc::Desc) {
      return std::meta::extract<doc::Desc>(a);
    }
    if (std::meta::type_of(a) == ^^const doc::Desc) {
      return std::meta::extract<const doc::Desc>(a);
    }
    if (std::meta::type_of(a) == ^^doc::Desc&) {
      return std::meta::extract<doc::Desc&>(a);
    }
    if (std::meta::type_of(a) == ^^const doc::Desc&) {
      return std::meta::extract<const doc::Desc&>(a);
    }
    if (std::meta::type_of(a) == ^^doc::Desc&&) {
      return std::meta::extract<doc::Desc&&>(a);
    }
    if (std::meta::type_of(a) == ^^const doc::Desc&&) {
      return std::meta::extract<const doc::Desc&&>(a);
    }
  }
  return doc::Desc("");
}

template <typename E>
consteval std::size_t get_enum_size() {
  return std::meta::enumerators_of(^^E).size();
}

template <typename E, std::size_t N>
consteval std::array<std::meta::info, N> get_enum_array() {
  auto vec = std::meta::enumerators_of(^^E);
  std::array<std::meta::info, N> arr{};
  for (std::size_t i = 0; i < N; ++i) {
    arr[i] = vec[i];
  }
  return arr;
}

template <typename E, std::size_t N>
struct EnumArrHolder {
  static constexpr auto arr = get_enum_array<E, N>();
};

template <typename E, E Value>
consteval std::string_view get_enum_name() {
  for (auto enumerator : std::meta::enumerators_of(^^E)) {
    if (std::meta::extract<E>(enumerator) == Value) {
      return std::meta::identifier_of(enumerator);
    }
  }
  throw "enum value has no reflected enumerator";
}

template <typename T>
consteval std::size_t get_fields_size() {
  auto ctx = std::meta::access_context::current();
  return std::meta::nonstatic_data_members_of(^^T, ctx).size();
}

template <typename T, std::size_t N>
consteval std::array<std::meta::info, N> get_fields_array() {
  auto ctx = std::meta::access_context::current();
  auto vec = std::meta::nonstatic_data_members_of(^^T, ctx);
  std::array<std::meta::info, N> arr{};
  for (std::size_t i = 0; i < N; ++i) {
    arr[i] = vec[i];
  }
  return arr;
}

template <typename T, std::size_t N>
struct StructArrHolder {
  static constexpr auto arr = get_fields_array<T, N>();
};

template <typename>
struct is_std_array : std::false_type {};

template <typename T, std::size_t N>
struct is_std_array<std::array<T, N>> : std::true_type {};

template <typename>
struct array_traits;

template <typename T, std::size_t N>
struct array_traits<std::array<T, N>> {
  using element_type = T;
  static constexpr std::size_t size = N;
};

template <typename T, std::size_t N>
struct array_traits<T[N]> {
  using element_type = T;
  static constexpr std::size_t size = N;
};

template <typename T>
inline constexpr bool is_array_like_v =
    is_std_array<std::remove_cv_t<T>>::value || std::is_array_v<T>;

template <typename T>
inline constexpr bool is_byte_like_v = [] {
  using U = std::remove_cv_t<T>;
  return std::is_same_v<U, char> || std::is_same_v<U, int8_t> || std::is_same_v<U, uint8_t>;
}();

template <typename T>
inline constexpr bool is_supported_wire_scalar_v = [] {
  using U = std::remove_cv_t<T>;
  if constexpr (std::is_integral_v<U> || std::is_enum_v<U>) {
    return sizeof(U) == 1 || sizeof(U) == 2 || sizeof(U) == 4 || sizeof(U) == 8;
  } else {
    return std::is_same_v<U, float> || std::is_same_v<U, double>;
  }
}();

template <typename>
inline constexpr bool dependent_false_v = false;

template <typename E>
consteval bool reflected_enum_values_are_unique() {
  auto enumerators = std::meta::enumerators_of(^^E);
  for (std::size_t i = 0; i < enumerators.size(); ++i) {
    for (std::size_t j = i + 1; j < enumerators.size(); ++j) {
      if (std::meta::extract<E>(enumerators[i]) == std::meta::extract<E>(enumerators[j])) {
        return false;
      }
    }
  }
  return true;
}

template <typename T>
consteval void validate_wire_type();

template <typename T>
consteval void validate_reflected_struct() {
  using U = std::remove_cv_t<T>;
  static_assert(!std::is_union_v<U>, "wire unions are not supported by the reflection generators");
  static_assert(std::is_standard_layout_v<U>,
                "reflected wire structs must have standard layout");
  static_assert(std::is_trivially_copyable_v<U>,
                "reflected wire structs must be trivially copyable");

  constexpr std::size_t N = get_fields_size<U>();
  static_assert(N > 0, "reflected wire structs must contain at least one field");

  [&]<std::size_t... Is>(std::index_sequence<Is...>) {
    (..., [&] {
      constexpr auto field = StructArrHolder<U, N>::arr[Is];
      constexpr auto type = std::meta::type_of(field);
      using FieldT = typename[:type:];
      validate_wire_type<FieldT>();
    }());
  }(std::make_index_sequence<N>{});
}

template <typename T>
consteval void validate_wire_type() {
  using U = std::remove_cv_t<T>;
  if constexpr (is_array_like_v<U>) {
    using ElemT = typename array_traits<U>::element_type;
    static_assert(array_traits<U>::size > 0, "zero-length wire arrays are not supported");
    static_assert(!is_array_like_v<ElemT>, "nested wire arrays are not supported");
    validate_wire_type<ElemT>();
  } else if constexpr (std::is_class_v<U>) {
    validate_reflected_struct<U>();
  } else {
    static_assert(is_supported_wire_scalar_v<U>,
                  "unsupported reflected wire type; use a supported scalar, enum, or struct");
  }
}

inline std::string strip_namespaces(std::string s) {
  auto erase_all = [&](std::string_view needle) {
    while (true) {
      std::size_t pos = s.find(needle);
      if (pos == std::string::npos) {
        return;
      }
      s.erase(pos, needle.size());
    }
  };

  erase_all("ipc::");
  erase_all("sil::");
  erase_all("std::__cxx11::");
  return s;
}

template <typename T>
std::string get_cxx_type_name() {
  if constexpr (std::is_same_v<T, bool>) {
    return "bool";
  } else if constexpr (std::is_same_v<T, char>) {
    return "char";
  } else if constexpr (std::is_same_v<T, uint8_t>) {
    return "uint8_t";
  } else if constexpr (std::is_same_v<T, int8_t>) {
    return "int8_t";
  } else if constexpr (std::is_same_v<T, uint16_t>) {
    return "uint16_t";
  } else if constexpr (std::is_same_v<T, int16_t>) {
    return "int16_t";
  } else if constexpr (std::is_same_v<T, uint32_t>) {
    return "uint32_t";
  } else if constexpr (std::is_same_v<T, int32_t>) {
    return "int32_t";
  } else if constexpr (std::is_same_v<T, uint64_t>) {
    return "uint64_t";
  } else if constexpr (std::is_same_v<T, int64_t>) {
    return "int64_t";
  } else if constexpr (std::is_same_v<T, float>) {
    return "float";
  } else if constexpr (std::is_same_v<T, double>) {
    return "double";
  } else if constexpr (is_std_array<T>::value) {
    using ElemT = typename T::value_type;
    return "std::array<" + get_cxx_type_name<ElemT>() + ", " + std::to_string(std::tuple_size_v<T>) +
           ">";
  } else if constexpr (std::is_array_v<T>) {
    using ElemT = std::remove_extent_t<T>;
    return get_cxx_type_name<ElemT>() + "[" + std::to_string(std::extent_v<T>) + "]";
  } else if constexpr (std::meta::has_identifier(^^T)) {
    return std::string(std::meta::identifier_of(^^T));
  } else {
    return strip_namespaces(std::string(std::meta::display_string_of(^^T)));
  }
}

template <typename T>
std::string get_python_type_name() {
  std::string s = get_cxx_type_name<T>();
  for (char& c : s) {
    if (c == '<' || c == '>' || c == ',' || c == ' ' || c == ':' || c == '[' || c == ']') {
      c = '_';
    }
  }
  while (!s.empty() && s.back() == '_') {
    s.pop_back();
  }
  return s;
}

template <typename T>
std::string get_python_scalar_type_name() {
  using U = std::remove_cv_t<T>;
  if constexpr (std::is_same_v<U, bool>) {
    return "bool";
  } else if constexpr (std::is_integral_v<U> && is_supported_wire_scalar_v<U>) {
    return "int";
  } else if constexpr (std::is_same_v<U, float> || std::is_same_v<U, double>) {
    return "float";
  } else if constexpr (std::is_enum_v<U> && is_supported_wire_scalar_v<U>) {
    return get_python_type_name<U>();
  } else if constexpr (std::is_class_v<U>) {
    return get_python_type_name<U>();
  } else {
    static_assert(dependent_false_v<U>,
                  "unsupported reflected scalar; use a supported integral, float, double, or enum");
    return {};
  }
}

template <typename T>
std::string get_python_annotation() {
  using U = std::remove_cv_t<T>;
  validate_wire_type<U>();
  if constexpr (is_array_like_v<U>) {
    using ElemT = typename array_traits<U>::element_type;
    if constexpr (is_byte_like_v<ElemT>) {
      return "bytes";
    } else {
      return "list[" + get_python_scalar_type_name<ElemT>() + "]";
    }
  } else {
    return get_python_scalar_type_name<T>();
  }
}

template <std::meta::info Type>
consteval std::string_view get_struct_format_char() {
  using T = std::remove_cv_t<typename[:Type:]>;
  static_assert(is_supported_wire_scalar_v<T>,
                "unsupported reflected scalar has no Python struct format");
  if constexpr (std::is_same_v<T, bool>) {
    return "?"sv;
  } else if constexpr (std::is_integral_v<T>) {
    if constexpr (sizeof(T) == 1) return std::is_signed_v<T> ? "b"sv : "B"sv;
    if constexpr (sizeof(T) == 2) return std::is_signed_v<T> ? "h"sv : "H"sv;
    if constexpr (sizeof(T) == 4) return std::is_signed_v<T> ? "i"sv : "I"sv;
    if constexpr (sizeof(T) == 8) return std::is_signed_v<T> ? "q"sv : "Q"sv;
  } else if constexpr (std::is_same_v<T, float> || std::is_same_v<T, double>) {
    if constexpr (sizeof(T) == 4) return "f"sv;
    if constexpr (sizeof(T) == 8) return "d"sv;
  } else if constexpr (std::is_enum_v<T>) {
    if constexpr (sizeof(T) == 1) return "B"sv;
    if constexpr (sizeof(T) == 2) return "H"sv;
    if constexpr (sizeof(T) == 4) return "I"sv;
    if constexpr (sizeof(T) == 8) return "Q"sv;
  }
  throw "unsupported reflected scalar has no Python struct format";
}

template <typename T>
void hash_combine(std::size_t& seed, const T& value) {
  std::hash<T> hasher;
  seed ^= hasher(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

}  // namespace balancer_reflection
