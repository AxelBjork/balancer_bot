// generate_balancer_bindings.cpp — C++26 reflection generator for balancer_bot payloads.
//
// This is compiled ONLY by gcc-trunk with -freflection. It emits a Python
// module with @dataclass types matching the wire layout of our IPC payloads.

#include <cstdint>
#include <iostream>
#include <meta>
#include <set>
#include <string>
#include <string_view>
#include <type_traits>
#include <array>

// Pull in msg_base.h for MsgId and MessageTraits
#include "msg_base.h"
#include "balancer_msgs.h"

using namespace std::string_view_literals;

// ── Minimal reflection helpers (subset of reflect_pytest/generator/common.h) ─

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
struct FieldHolder {
  static constexpr auto arr = get_fields_array<T, N>();
};

// ── struct.pack format character lookup ──────────────────────────────────────

template <std::meta::info Type>
consteval std::string_view fmt_char() {
  using T = typename[:Type:];
  if constexpr (std::is_same_v<T, bool>)   return "?"sv;
  else if constexpr (std::is_integral_v<T>) {
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
  }
  return "?"sv;
}

template <typename> struct is_std_array : std::false_type {};
template <typename T, std::size_t N>
struct is_std_array<std::array<T, N>> : std::true_type {};

// ── Python type-hint string ─────────────────────────────────────────────────

template <std::meta::info Type>
consteval std::string_view py_hint() {
  using T = typename[:Type:];
  if constexpr (std::is_same_v<T, bool>)         return "bool"sv;
  else if constexpr (std::is_integral_v<T>)      return "int"sv;
  else if constexpr (std::is_floating_point_v<T>) return "float"sv;
  return "Any"sv;
}

// ── Emit one @dataclass + pack/unpack ───────────────────────────────────────

template <typename T>
void emit_struct(const char* py_name) {
  constexpr std::size_t N = get_fields_size<T>();

  std::cout << "@dataclass\nclass " << py_name << ":\n";
  std::cout << "    WIRE_SIZE = " << sizeof(T) << "\n";

  // Field declarations
  std::string pack_fmt  = "<";
  std::string pack_args;
  std::string unpack_body = "        offset = 0\n";
  std::string field_names;

  if constexpr (N > 0) {
    [&]<std::size_t... Is>(std::index_sequence<Is...>) {
      (..., [&] {
        constexpr auto field = FieldHolder<T, N>::arr[Is];
        constexpr auto type  = std::meta::type_of(field);
        using FT = typename[:type:];

        std::string name{std::meta::identifier_of(field)};

        if constexpr (is_std_array<FT>::value) {
          using ElemT = typename FT::value_type;
          constexpr std::size_t Len = std::tuple_size_v<FT>;
          // e.g. acc: list[float]
          std::cout << "    " << name << ": list[float]\n";

          // pack: extend format "Nd" or "Nf"
          auto fc = fmt_char<^^ElemT>();
          pack_fmt += std::to_string(Len) + std::string(fc);

          for (std::size_t k = 0; k < Len; ++k) {
            if (!pack_args.empty()) pack_args += ", ";
            pack_args += "self." + name + "[" + std::to_string(k) + "]";
          }

          // unpack
          unpack_body += "        " + name + " = list(struct.unpack_from(\"<" +
                         std::to_string(Len) + std::string(fc) + "\", data, offset))\n";
          unpack_body += "        offset += " + std::to_string(sizeof(FT)) + "\n";

        } else {
          // Scalar
          auto hint = py_hint<type>();
          std::cout << "    " << name << ": " << hint << "\n";

          auto fc = fmt_char<type>();
          pack_fmt += std::string(fc);
          if (!pack_args.empty()) pack_args += ", ";
          pack_args += "self." + name;

          unpack_body += "        " + name + ", = struct.unpack_from(\"<" +
                         std::string(fc) + "\", data, offset)\n";
          unpack_body += "        offset += " + std::to_string(sizeof(FT)) + "\n";
        }

        if (!field_names.empty()) field_names += ", ";
        field_names += name + "=" + name;
      }());
    }(std::make_index_sequence<N>{});
  }

  // pack method
  std::cout << "\n    def pack(self) -> bytes:\n";
  std::cout << "        return struct.pack(\"" << pack_fmt << "\", " << pack_args << ")\n";

  // unpack classmethod
  std::cout << "\n    @classmethod\n";
  std::cout << "    def unpack(cls, data: bytes) -> '" << py_name << "':\n";
  std::cout << unpack_body;
  std::cout << "        return cls(" << field_names << ")\n\n";
}

// ── main ────────────────────────────────────────────────────────────────────

int main() {
  std::cout << "\"\"\"Auto-generated balancer_bot IPC bindings (C++26 reflection).\"\"\"\n\n";
  std::cout << "import struct\n";
  std::cout << "from dataclasses import dataclass\n";
  std::cout << "from enum import IntEnum\n\n";

  // Emit a mini MsgId enum for just our messages
  std::cout << "class BalancerMsgId(IntEnum):\n";
  std::cout << "    PhysicsTick = " << static_cast<uint16_t>(MsgId::PhysicsTick) << "\n";
  std::cout << "    ImuData = " << static_cast<uint16_t>(ipc::ImuData) << "\n";
  std::cout << "    JoystickCommand = " << static_cast<uint16_t>(ipc::JoystickCommand) << "\n";
  std::cout << "    MotorTargets = " << static_cast<uint16_t>(ipc::MotorTargets) << "\n";
  std::cout << "    SystemTelemetry = " << static_cast<uint16_t>(ipc::SystemTelemetry) << "\n\n";

  // Generate dataclasses via reflection
  emit_struct<PhysicsTickPayload>("PhysicsTickPayload");
  emit_struct<ipc::ImuSamplePayload>("ImuSamplePayload");
  emit_struct<ipc::JoystickCommandPayload>("JoystickCommandPayload");
  emit_struct<ipc::MotorTargetsPayload>("MotorTargetsPayload");
  emit_struct<ipc::SystemTelemetryPayload>("SystemTelemetryPayload");

  // Lookup table
  std::cout << "MESSAGE_BY_ID = {\n";
  std::cout << "    BalancerMsgId.PhysicsTick: PhysicsTickPayload,\n";
  std::cout << "    BalancerMsgId.ImuData: ImuSamplePayload,\n";
  std::cout << "    BalancerMsgId.JoystickCommand: JoystickCommandPayload,\n";
  std::cout << "    BalancerMsgId.MotorTargets: MotorTargetsPayload,\n";
  std::cout << "    BalancerMsgId.SystemTelemetry: SystemTelemetryPayload,\n";
  std::cout << "}\n";
}
