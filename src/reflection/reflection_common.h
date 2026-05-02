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
inline constexpr bool is_array_like_v = is_std_array<T>::value || std::is_array_v<T>;

template <typename T>
inline constexpr bool is_byte_like_v =
    std::is_same_v<T, char> || std::is_same_v<T, int8_t> || std::is_same_v<T, uint8_t>;

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
  if constexpr (std::is_same_v<T, bool>) {
    return "bool";
  } else if constexpr (std::is_integral_v<T>) {
    return "int";
  } else if constexpr (std::is_floating_point_v<T>) {
    return "float";
  } else if constexpr (std::is_enum_v<T>) {
    return get_python_type_name<T>();
  } else if constexpr (std::is_class_v<T>) {
    return get_python_type_name<T>();
  } else {
    return "Any";
  }
}

template <typename T>
std::string get_python_annotation() {
  if constexpr (is_array_like_v<T>) {
    using ElemT = typename array_traits<T>::element_type;
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
  using T = typename[:Type:];
  if constexpr (std::is_same_v<T, bool>) {
    return "?"sv;
  } else if constexpr (std::is_integral_v<T>) {
    if constexpr (sizeof(T) == 1) return std::is_signed_v<T> ? "b"sv : "B"sv;
    if constexpr (sizeof(T) == 2) return std::is_signed_v<T> ? "h"sv : "H"sv;
    if constexpr (sizeof(T) == 4) return std::is_signed_v<T> ? "i"sv : "I"sv;
    if constexpr (sizeof(T) == 8) return std::is_signed_v<T> ? "q"sv : "Q"sv;
  } else if constexpr (std::is_floating_point_v<T>) {
    if constexpr (sizeof(T) == 4) return "f"sv;
    if constexpr (sizeof(T) == 8) return "d"sv;
  } else if constexpr (std::is_enum_v<T>) {
    if constexpr (sizeof(T) == 1) return "B"sv;
    if constexpr (sizeof(T) == 2) return "H"sv;
    if constexpr (sizeof(T) == 4) return "I"sv;
    if constexpr (sizeof(T) == 8) return "Q"sv;
  }
  return "?"sv;
}

template <typename T>
void hash_combine(std::size_t& seed, const T& value) {
  std::hash<T> hasher;
  seed ^= hasher(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

}  // namespace balancer_reflection
