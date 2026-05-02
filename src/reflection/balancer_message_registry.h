#pragma once

#include <cstdio>
#include <string>

#include "../ipc/publisher.h"
#include "../ipc/udp_bridge.h"
#include "reflection_common.h"

namespace balancer_reflection {

template <::MsgId... Ids>
constexpr std::size_t message_count(ipc::MsgList<Ids...>) {
  return sizeof...(Ids);
}

template <MsgId... Ids, typename Fn>
constexpr void for_each_message(ipc::MsgList<Ids...>, Fn&& fn) {
  (fn.template operator()<Ids>(), ...);
}

template <typename Fn>
constexpr void for_each_message_enum(Fn&& fn) {
  constexpr std::size_t N = get_enum_size<MsgId>();
  [&]<std::size_t... Is>(std::index_sequence<Is...>) {
    (..., [&] {
      constexpr auto e = EnumArrHolder<MsgId, N>::arr[Is];
      constexpr auto id = static_cast<MsgId>([:e:]);
      fn.template operator()<id>();
    }());
  }(std::make_index_sequence<N>{});
}

template <::MsgId Target, ::MsgId... Ids>
consteval bool contains_msg(ipc::MsgList<Ids...>) {
  return ((Target == Ids) || ...);
}

template <typename Component, ::MsgId Id>
consteval bool component_subscribes() {
  return contains_msg<Id>(typename Component::Subscribes{});
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
    hash_combine(structural_hash, std::string_view(MessageTraits<Id>::name));
    hash_combine(structural_hash, std::to_string(static_cast<uint16_t>(Id)));
    hash_combine(structural_hash, std::to_string(sizeof(typename MessageTraits<Id>::Payload)));
  });

  char hex_string[17];
  std::snprintf(hex_string, sizeof(hex_string), "%016lx", structural_hash);
  return hex_string;
}

}  // namespace balancer_reflection
