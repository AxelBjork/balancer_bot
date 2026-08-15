#pragma once

#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/uio.h>

#include <atomic>
#include <bit>
#include <limits>
#include <thread>

#include "messages/core_msgs.h"
#include "messages/balancer_msgs.h"
#include "publisher.h"

namespace ipc {

inline constexpr char kUdpBridgeDoc[] =
    "Stateful transport bridge that connects the internal `MessageBus` to external UDP runtime "
    "clients. In the production Pi runtime it listens on port `9000`; the telemetry server is the "
    "primary peer, while SIL and other authorized clients use the same boundary.\n\n"
    "A client registers by sending a UDP datagram. The bridge remembers the most recent sender as "
    "the single active peer, so outbound messages always have one explicit return path. On ingress, "
    "the bridge extracts the leading `uint16_t` message identifier and republishes the remaining "
    "payload only when the ID is authorized by `Publishes` and the payload size is valid.\n\n"
    "On egress, each authorized subscribed message is encoded as the reflected message ID followed "
    "immediately by the trivially-copyable payload bytes:\n\n"
    "$$ \\text{datagram} = \\texttt{uint16\\_t MsgId} \\; || \\; \\texttt{Payload bytes} $$\n\n"
    "This makes the UDP contract symmetric with the generated Python bindings and keeps the external "
    "runtime API aligned with the reflected C++ message definitions. The dashboard receives and logs "
    "`SystemTelemetry`; deployment and process control remain separate SSH operations. The simulator "
    "scenario service uses a separate UDP endpoint on port `9001` and is not this bridge.";

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

  using Subscribes = MsgList<MsgId::MotorTargets, MsgId::SystemTelemetry, MsgId::SimulatorTelemetry,
                             MsgId::SimStartAck, MsgId::SimRunDone, MsgId::PidConfigStatus>;
  using Publishes = MsgList<MsgId::PhysicsTick, MsgId::ExternalJoystickCommand,
                            MsgId::ImuRawData, MsgId::SimStartRun, MsgId::SimStopRun,
                            MsgId::PidConfigOverride,
                            MsgId::PitchAuthorityDiagnosticCommand>;

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

static_assert(std::endian::native == std::endian::little,
              "UDP reflected payloads require little-endian C++ storage");
static_assert(sizeof(bool) == 1, "UDP reflected payloads require one-byte bool");
static_assert(std::numeric_limits<float>::is_iec559 && std::numeric_limits<double>::is_iec559,
              "UDP reflected payloads require IEEE-754 floating point");

}  // namespace ipc
