#pragma once

#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/uio.h>

#include <array>
#include <atomic>
#include <bit>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <type_traits>

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
    "immediately by the trivially-copyable payload bytes and placed on a bounded transport queue. "
    "High-rate telemetry uses latest-value coalescing and a dedicated nonblocking TX worker, so a "
    "slow UDP peer cannot extend the control dispatch path:\n\n"
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
                            MsgId::PidConfigOverride>;

  static constexpr uint16_t kDefaultPort = 9000;
  static constexpr std::size_t kMaxDgram = 4096;

  struct TxStats {
    uint64_t enqueued = 0;
    uint64_t coalesced = 0;
    uint64_t dropped = 0;
    uint64_t no_peer = 0;
    uint64_t sends = 0;
    uint64_t eagain = 0;
    uint64_t send_errors = 0;
    uint64_t max_enqueue_ns = 0;
    uint64_t max_send_ns = 0;
    uint64_t queue_depth = 0;
  };

  using SendFunction = ssize_t (*)(int, const msghdr*, int);

  bool is_connected() const;

  explicit UdpBridge(MessageBus& bus, uint16_t port = kDefaultPort,
                     SendFunction send_function = nullptr);
  ~UdpBridge();

  void start();
  uint16_t local_port() const;
  TxStats tx_stats() const;
  UdpBridge(const UdpBridge&) = delete;
  UdpBridge& operator=(const UdpBridge&) = delete;

  template <MsgId Id>
  void on_message(const typename MessageTraits<Id>::Payload& p) {
    forward_to_udp<Id>(p);
  }

 private:
  static constexpr std::size_t kLatestSlotCount = 3;
  static constexpr std::size_t kLowPriorityQueueCapacity = 64;

  struct TxPacket {
    std::array<uint8_t, kMaxDgram> bytes{};
    std::size_t size = 0;
    MsgId id = MsgId::SystemTelemetry;
    bool latest_only = false;
  };

  TypedPublisher<UdpBridge> bus_;
  int udp_fd_;
  int wake_[2];
  std::thread rx_thread_;
  std::thread tx_thread_;
  std::atomic<PeerAddress> active_peer_{PeerAddress{0, 0, 0}};
  SendFunction send_function_ = nullptr;

  std::unique_ptr<TxPacket[]> low_priority_queue_;
  std::size_t low_priority_head_ = 0;
  std::size_t low_priority_size_ = 0;
  std::array<TxPacket, kLatestSlotCount> latest_packets_{};
  std::array<bool, kLatestSlotCount> latest_pending_{};
  mutable std::mutex tx_mutex_;
  std::condition_variable tx_condition_;
  bool tx_stopping_ = false;

  std::atomic<uint64_t> tx_enqueued_{0};
  std::atomic<uint64_t> tx_coalesced_{0};
  std::atomic<uint64_t> tx_dropped_{0};
  std::atomic<uint64_t> tx_no_peer_{0};
  std::atomic<uint64_t> tx_sends_{0};
  std::atomic<uint64_t> tx_eagain_{0};
  std::atomic<uint64_t> tx_send_errors_{0};
  std::atomic<uint64_t> tx_max_enqueue_ns_{0};
  std::atomic<uint64_t> tx_max_send_ns_{0};
  std::atomic<uint64_t> tx_queue_depth_{0};

  void rx_loop();
  void tx_loop();
  bool enqueue_packet(const TxPacket& packet);
  bool dequeue_packet(TxPacket& packet);
  bool requeue_low_priority(const TxPacket& packet);
  uint64_t queue_depth_locked() const;

  static constexpr bool is_latest_only(MsgId id) {
    return id == MsgId::MotorTargets || id == MsgId::SystemTelemetry ||
           id == MsgId::SimulatorTelemetry;
  }

  static constexpr std::size_t latest_index(MsgId id) {
    return id == MsgId::MotorTargets ? 0 : id == MsgId::SystemTelemetry ? 1 : 2;
  }

  template <MsgId Id, typename Payload>
  void forward_to_udp(const Payload& payload) {
    static_assert(std::is_trivially_copyable_v<Payload>);
    static_assert(sizeof(Payload) + sizeof(uint16_t) <= kMaxDgram);
    const auto start = std::chrono::steady_clock::now();
    TxPacket packet{};
    uint16_t id_raw = static_cast<uint16_t>(Id);
    std::memcpy(packet.bytes.data(), &id_raw, sizeof(id_raw));
    std::memcpy(packet.bytes.data() + sizeof(id_raw), &payload, sizeof(Payload));
    packet.size = sizeof(id_raw) + sizeof(Payload);
    packet.id = Id;
    packet.latest_only = is_latest_only(Id);
    enqueue_packet(packet);
    const uint64_t duration_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - start)
            .count());
    uint64_t previous = tx_max_enqueue_ns_.load(std::memory_order_relaxed);
    while (previous < duration_ns &&
           !tx_max_enqueue_ns_.compare_exchange_weak(previous, duration_ns,
                                                      std::memory_order_relaxed,
                                                      std::memory_order_relaxed)) {
    }
  }
};

static_assert(std::endian::native == std::endian::little,
              "UDP reflected payloads require little-endian C++ storage");
static_assert(sizeof(bool) == 1, "UDP reflected payloads require one-byte bool");
static_assert(std::numeric_limits<float>::is_iec559 && std::numeric_limits<double>::is_iec559,
              "UDP reflected payloads require IEEE-754 floating point");

}  // namespace ipc
