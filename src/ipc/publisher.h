#pragma once

#include <cstring>
#include <mutex>
#include <type_traits>

#include "messages/msg_base.h"

namespace ipc {

class MessageBus {
 public:
  using DispatchFn = void (*)(void* ctx, MsgId id, const void* payload);

  explicit MessageBus(void* ctx, DispatchFn dispatcher) : ctx_(ctx), dispatcher_(dispatcher) {
  }
  ~MessageBus() = default;

  MessageBus(const MessageBus&) = delete;
  MessageBus& operator=(const MessageBus&) = delete;

  template <MsgId Id>
  void publish(const typename MessageTraits<Id>::Payload& payload) {
    dispatch(Id, &payload);
  }

  template <typename T>
  friend class TypedPublisher;

 private:
  void* const ctx_;
  const DispatchFn dispatcher_;
  std::recursive_mutex dispatch_mu_;

  void dispatch(MsgId id, const void* payload);
};

template <MsgId... Ids>
struct MsgList {};

namespace detail {

template <MsgId Target, MsgId... Ids>
struct MsgContains;

template <MsgId Target>
struct MsgContains<Target> : std::false_type {};

template <MsgId Target, MsgId First, MsgId... Rest>
struct MsgContains<Target, First, Rest...>
    : std::conditional_t<Target == First, std::true_type, MsgContains<Target, Rest...>> {};

template <MsgId Target, typename List>
struct IsInList;

template <MsgId Target, MsgId... Ids>
struct IsInList<Target, MsgList<Ids...>> : MsgContains<Target, Ids...> {};

template <MsgId Target, typename List>
constexpr bool is_in_list_v = IsInList<Target, List>::value;

template <MsgId... Ids>
bool try_publish_raw(MessageBus& bus, MsgId id, const void* data, size_t size, MsgList<Ids...>) {
  return ((id == Ids && size == sizeof(typename MessageTraits<Ids>::Payload) &&
           [&] {
             static_assert(std::is_trivially_copyable_v<typename MessageTraits<Ids>::Payload>);
             typename MessageTraits<Ids>::Payload p;
             std::memcpy(&p, data, sizeof(p));
             bus.publish<Ids>(p);
             return true;
           }()) ||
          ...);
}

template <typename Service, MsgId... SubscribedIds>
bool try_dispatch_raw(Service& service, MsgId id, const void* payload, MsgList<SubscribedIds...>) {
  return ((id == SubscribedIds &&
           (service.template on_message<SubscribedIds>(unpack_payload<SubscribedIds>(payload)),
            true)) ||
          ...);
}

}  // namespace detail

template <typename Service>
bool dispatch_to_service(Service& service, MsgId id, const void* payload) {
  if constexpr (requires { typename Service::Subscribes; }) {
    return detail::try_dispatch_raw(service, id, payload, typename Service::Subscribes{});
  } else {
    return false;
  }
}

template <typename... Services>
void dispatch_to_services(MsgId id, const void* payload, Services&... services) {
  (dispatch_to_service(services, id, payload), ...);
}

template <typename Component>
class TypedPublisher {
 public:
  explicit TypedPublisher(MessageBus& bus) : bus_(bus) {
  }

  template <MsgId Id>
  void publish(const typename MessageTraits<Id>::Payload& payload) {
    static_assert(detail::is_in_list_v<Id, typename Component::Publishes>,
                  "Message ID not found in Component::Publishes list!");
    bus_.publish<Id>(payload);
  }

  bool publish_if_authorized(MsgId id, const void* data, size_t size) {
    return detail::try_publish_raw(bus_, id, data, size, typename Component::Publishes{});
  }

  MessageBus& bus() {
    return bus_;
  }

 private:
  MessageBus& bus_;
};

}  // namespace ipc
