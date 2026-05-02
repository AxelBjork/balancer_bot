#pragma once

#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/uio.h>

#include <atomic>
#include <thread>

#include "messages/core_msgs.h"
#include "messages/balancer_msgs.h"
#include "publisher.h"

namespace ipc {

inline constexpr char kUdpBridgeDoc[] =
    "Stateful transport bridge that connects the internal `MessageBus` to external UDP-based SIL "
    "clients.\n\n"
    "On ingress, the bridge binds a UDP socket on port `9000`, receives datagrams from the latest "
    "test harness peer, extracts the leading `uint16_t` message identifier, and republishes the "
    "remaining payload bytes through `publish_if_authorized`. That keeps external injection limited "
    "to the message types the bridge explicitly advertises in `Publishes`, and the downstream bus "
    "path retains ownership of payload-size checks before handlers see any data.\n\n"
    "On egress, the bridge remembers the most recent sender address and uses it as the return path "
    "for outbound telemetry and motor command traffic. Each authorized outbound message is encoded "
    "as the reflected message ID followed immediately by the trivially-copyable payload bytes:\n\n"
    "$$ \\text{datagram} = \\texttt{uint16\\_t MsgId} \\; || \\; \\texttt{Payload bytes} $$\n\n"
    "This makes the UDP contract symmetric with the Python bindings generated from the same message "
    "definitions. Operationally, `UdpBridge` is what turns the balancer into a SIL endpoint: it "
    "lets pytest inject `PhysicsTick`, `JoystickCommand`, `ImuRawData`, and simulator-control "
    "messages, while streaming `MotorTargets`, `SystemTelemetry`, and simulator-status "
    "messages back out for observation and closed-loop test orchestration.";

struct PeerAddress {
  uint32_t ip;
  uint16_t port;
  uint16_t _pad;

  bool operator==(const PeerAddress& other) const { return ip == other.ip && port == other.port; }
  bool operator!=(const PeerAddress& other) const { return !(*this == other); }
  explicit operator bool() const { return ip != 0 || port != 0; }
};

class DOC_DESC(kUdpBridgeDoc) UdpBridge {
 public:
  static constexpr const char* kDocDescription = kUdpBridgeDoc;

  using Subscribes = MsgList<MsgId::MotorTargets, MsgId::SystemTelemetry, MsgId::SimStartAck, MsgId::SimRunDone>;
  using Publishes = MsgList<MsgId::PhysicsTick, MsgId::JoystickCommand, MsgId::ImuRawData,
                            MsgId::SimStartRun, MsgId::SimStopRun>;

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
    iovec iov[2];
    iov[0].iov_base = &id_raw;
    iov[0].iov_len = sizeof(id_raw);
    iov[1].iov_base = const_cast<Payload*>(&payload);
    iov[1].iov_len = sizeof(Payload);

    msghdr msg = {};
    msg.msg_name = &peer;
    msg.msg_namelen = sizeof(peer);
    msg.msg_iov = iov;
    msg.msg_iovlen = 2;

    return static_cast<int>(::sendmsg(udp_fd_, &msg, 0));
  }
};

}  // namespace ipc
