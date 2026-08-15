#pragma once

#include <cstdio>
#include <string>

#include "../ipc/publisher.h"
#include "../ipc/udp_bridge.h"
#include "reflection_common.h"

namespace balancer_reflection {

template <typename Fn>
constexpr void for_each_message_enum(Fn&& fn) {
  static_assert(reflected_enum_values_are_unique<MsgId>(),
                "MsgId enumerators must have unique wire values");

  template for (constexpr auto e : reflected_enumerators<MsgId>()) {
    constexpr auto id = static_cast<MsgId>([:e:]);
    static_assert(requires { typename MessageTraits<id>::Payload; },
                  "every MsgId enumerator must have a MessageTraits Payload binding");
    fn.template operator()<id>();
  }
}

template <::MsgId Target, ::MsgId... Ids>
consteval bool contains_msg(ipc::MsgList<Ids...>) {
  return ((Target == Ids) || ...);
}

template <typename Component, ::MsgId Id>
consteval bool component_subscribes() {
  if constexpr (requires { typename Component::Subscribes; }) {
    return contains_msg<Id>(typename Component::Subscribes{});
  } else {
    return false;
  }
}

template <typename Component, ::MsgId Id>
consteval bool component_publishes() {
  return contains_msg<Id>(typename Component::Publishes{});
}

template <::MsgId Id>
consteval bool udp_contract_message() {
  return component_publishes<ipc::UdpBridge, Id>() || component_subscribes<ipc::UdpBridge, Id>();
}

template <typename Fn>
constexpr void for_each_udp_message(Fn&& fn) {
  for_each_message_enum([&]<::MsgId Id>() {
    if constexpr (udp_contract_message<Id>()) {
      fn.template operator()<Id>();
    }
  });
}

inline std::string compute_protocol_hash() {
  std::size_t structural_hash = 0;
  hash_combine(structural_hash, std::string_view("MsgId"));

  for_each_udp_message([&]<MsgId Id>() {
    hash_combine(structural_hash, get_enum_name<MsgId, Id>());
    hash_combine(structural_hash, std::to_string(static_cast<uint16_t>(Id)));
    hash_combine(structural_hash, std::to_string(sizeof(typename MessageTraits<Id>::Payload)));
  });

  char hex_string[17];
  std::snprintf(hex_string, sizeof(hex_string), "%016lx", structural_hash);
  return hex_string;
}

}  // namespace balancer_reflection
