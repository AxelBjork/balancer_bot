#pragma once

#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/uio.h>

#include <atomic>
#include <thread>

#include "autonomous_msgs.h"
#include "publisher.h"
#include "core_msgs.h"
#include "simulation_msgs.h"
#include "balancer_msgs.h"

namespace ipc {

struct PeerAddress {
  uint32_t ip;
  uint16_t port;
  uint16_t _pad;

  bool operator==(const PeerAddress& other) const { return ip == other.ip && port == other.port; }
  bool operator!=(const PeerAddress& other) const { return !(*this == other); }
  explicit operator bool() const { return ip != 0 || port != 0; }
};

class DOC_DESC("Stateful bridge that relays IPC messages between the internal MessageBus and external UDP clients.\n\nIt remembers the IP address and port of the last connected test harness and bidirectionally routes all subscribed C++ events out through the UDP socket while safely injecting incoming UDP datagrams onto the internal MessageBus.") UdpBridge {
 public:
  using Subscribes = MsgList<ipc::ImuData, ipc::MotorTargets, ipc::SystemTelemetry>;
  using Publishes = MsgList<MsgId::PhysicsTick, ipc::JoystickCommand, ipc::ImuData>;

  static constexpr uint16_t kDefaultPort = 9000;

  bool is_connected() const;

  explicit UdpBridge(MessageBus& bus);
  ~UdpBridge();

  void start();
  UdpBridge(const UdpBridge&) = delete;
  UdpBridge& operator=(const UdpBridge&) = delete;

  template <MsgId Id>
  void on_message(const typename MessageTraits<Id>::Payload& p) {
    forward_to_udp<Id>(p);
  }

 private:
  TypedPublisher<UdpBridge> bus_;
  int udp_fd_;
  int wake_[2];
  std::thread rx_thread_;
  std::atomic<PeerAddress> active_peer_{PeerAddress{0, 0, 0}};

  void rx_loop();

  template <MsgId Id, typename Payload>
  int forward_to_udp(const Payload& payload) {
    static_assert(std::is_trivially_copyable_v<Payload>);
    PeerAddress peer_val = active_peer_.load(std::memory_order_acquire);
    if (!peer_val) return -1;

    sockaddr_in peer{};
    peer.sin_family = AF_INET;
    peer.sin_port = peer_val.port;
    peer.sin_addr.s_addr = peer_val.ip;

    uint16_t id_raw = static_cast<uint16_t>(Id);
    struct iovec iov[2];
    iov[0].iov_base = &id_raw;
    iov[0].iov_len = sizeof(id_raw);
    iov[1].iov_base = const_cast<Payload*>(&payload);
    iov[1].iov_len = sizeof(Payload);

    struct msghdr msg = {};
    msg.msg_name = &peer;
    msg.msg_namelen = sizeof(peer);
    msg.msg_iov = iov;
    msg.msg_iovlen = 2;

    return ::sendmsg(udp_fd_, &msg, 0);
  }
};

}  // namespace ipc
