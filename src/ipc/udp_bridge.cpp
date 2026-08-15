#include "udp_bridge.h"

#include <arpa/inet.h>
#include <cerrno>
#include <sys/select.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <stdexcept>

namespace ipc {

bool UdpBridge::is_connected() const {
  PeerAddress peer_val = active_peer_.load(std::memory_order_acquire);
  return static_cast<bool>(peer_val);
}

static_assert(std::is_trivially_copyable_v<PeerAddress>);
static_assert(std::atomic<PeerAddress>::is_always_lock_free);

UdpBridge::UdpBridge(MessageBus& bus, uint16_t port, SendFunction send_function)
    : bus_(bus), send_function_(send_function),
      low_priority_queue_(std::make_unique<TxPacket[]>(kLowPriorityQueueCapacity)) {
  udp_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (udp_fd_ < 0) throw std::runtime_error("UdpBridge: socket() failed");

  int opt = 1;
  ::setsockopt(udp_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(port);

  if (::bind(udp_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    ::close(udp_fd_);
    throw std::runtime_error("UdpBridge: bind() failed");
  }

  if (::pipe(wake_) < 0) {
    ::close(udp_fd_);
    throw std::runtime_error("UdpBridge: pipe() failed");
  }
}

void UdpBridge::start() {
  tx_thread_ = std::thread(&UdpBridge::tx_loop, this);
  rx_thread_ = std::thread(&UdpBridge::rx_loop, this);
}

uint16_t UdpBridge::local_port() const {
  sockaddr_in address{};
  socklen_t length = sizeof(address);
  if (::getsockname(udp_fd_, reinterpret_cast<sockaddr*>(&address), &length) < 0) {
    return 0;
  }
  return ntohs(address.sin_port);
}

UdpBridge::~UdpBridge() {
  {
    std::lock_guard<std::mutex> lock(tx_mutex_);
    tx_stopping_ = true;
  }
  tx_condition_.notify_all();
  if (tx_thread_.joinable()) tx_thread_.join();

  char b = 0;
  const ssize_t wake_write_result = ::write(wake_[1], &b, 1);
  (void)wake_write_result;
  if (rx_thread_.joinable()) rx_thread_.join();
  ::close(udp_fd_);
  ::close(wake_[0]);
  ::close(wake_[1]);
}

UdpBridge::TxStats UdpBridge::tx_stats() const {
  std::lock_guard<std::mutex> lock(tx_mutex_);
  return TxStats{
      .enqueued = tx_enqueued_.load(std::memory_order_relaxed),
      .coalesced = tx_coalesced_.load(std::memory_order_relaxed),
      .dropped = tx_dropped_.load(std::memory_order_relaxed),
      .no_peer = tx_no_peer_.load(std::memory_order_relaxed),
      .sends = tx_sends_.load(std::memory_order_relaxed),
      .eagain = tx_eagain_.load(std::memory_order_relaxed),
      .send_errors = tx_send_errors_.load(std::memory_order_relaxed),
      .max_enqueue_ns = tx_max_enqueue_ns_.load(std::memory_order_relaxed),
      .max_send_ns = tx_max_send_ns_.load(std::memory_order_relaxed),
      .queue_depth = tx_queue_depth_.load(std::memory_order_relaxed),
  };
}

bool UdpBridge::enqueue_packet(const TxPacket& packet) {
  std::lock_guard<std::mutex> lock(tx_mutex_);
  if (packet.latest_only) {
    const std::size_t index = latest_index(packet.id);
    if (latest_pending_[index]) {
      tx_coalesced_.fetch_add(1, std::memory_order_relaxed);
    } else {
      tx_enqueued_.fetch_add(1, std::memory_order_relaxed);
    }
    latest_packets_[index] = packet;
    latest_pending_[index] = true;
  } else if (low_priority_size_ >= kLowPriorityQueueCapacity) {
    tx_dropped_.fetch_add(1, std::memory_order_relaxed);
    return false;
  } else {
    const std::size_t index = (low_priority_head_ + low_priority_size_) % kLowPriorityQueueCapacity;
    low_priority_queue_[index] = packet;
    ++low_priority_size_;
    tx_enqueued_.fetch_add(1, std::memory_order_relaxed);
  }
  tx_queue_depth_.store(queue_depth_locked(), std::memory_order_relaxed);
  tx_condition_.notify_one();
  return true;
}

bool UdpBridge::dequeue_packet(TxPacket& packet) {
  std::unique_lock<std::mutex> lock(tx_mutex_);
  tx_condition_.wait(lock, [this]() {
    const bool pending = low_priority_size_ != 0 ||
                         std::any_of(latest_pending_.begin(), latest_pending_.end(),
                                     [](bool value) { return value; });
    return tx_stopping_ || (static_cast<bool>(active_peer_.load(std::memory_order_acquire)) &&
                            pending);
  });
  if (tx_stopping_) return false;

  if (low_priority_size_ != 0) {
    packet = low_priority_queue_[low_priority_head_];
    low_priority_head_ = (low_priority_head_ + 1) % kLowPriorityQueueCapacity;
    --low_priority_size_;
    tx_queue_depth_.store(queue_depth_locked(), std::memory_order_relaxed);
    return true;
  }
  for (std::size_t index = 0; index < latest_pending_.size(); ++index) {
    if (latest_pending_[index]) {
      packet = latest_packets_[index];
      latest_pending_[index] = false;
      tx_queue_depth_.store(queue_depth_locked(), std::memory_order_relaxed);
      return true;
    }
  }
  return false;
}

bool UdpBridge::requeue_low_priority(const TxPacket& packet) {
  std::lock_guard<std::mutex> lock(tx_mutex_);
  if (low_priority_size_ >= kLowPriorityQueueCapacity) {
    return false;
  }
  low_priority_head_ = (low_priority_head_ + kLowPriorityQueueCapacity - 1) %
                       kLowPriorityQueueCapacity;
  low_priority_queue_[low_priority_head_] = packet;
  ++low_priority_size_;
  tx_queue_depth_.store(queue_depth_locked(), std::memory_order_relaxed);
  tx_condition_.notify_one();
  return true;
}

uint64_t UdpBridge::queue_depth_locked() const {
  return low_priority_size_ +
         static_cast<std::size_t>(std::count(latest_pending_.begin(), latest_pending_.end(), true));
}

void UdpBridge::tx_loop() {
  while (true) {
    TxPacket packet{};
    if (!dequeue_packet(packet)) return;

    const PeerAddress peer_val = active_peer_.load(std::memory_order_acquire);
    if (!peer_val) {
      tx_no_peer_.fetch_add(1, std::memory_order_relaxed);
      if (!packet.latest_only && !requeue_low_priority(packet)) {
        tx_dropped_.fetch_add(1, std::memory_order_relaxed);
      }
      continue;
    }

    sockaddr_in peer{};
    peer.sin_family = AF_INET;
    peer.sin_port = peer_val.port;
    peer.sin_addr.s_addr = peer_val.ip;
    iovec iov{};
    iov.iov_base = packet.bytes.data();
    iov.iov_len = packet.size;
    msghdr msg{};
    msg.msg_name = &peer;
    msg.msg_namelen = sizeof(peer);
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;

    const auto start = std::chrono::steady_clock::now();
    const ssize_t result = send_function_ != nullptr
                               ? send_function_(udp_fd_, &msg, MSG_DONTWAIT)
                               : ::sendmsg(udp_fd_, &msg, MSG_DONTWAIT);
    const int send_error = result < 0 ? errno : 0;
    const uint64_t duration_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - start)
            .count());
    uint64_t previous = tx_max_send_ns_.load(std::memory_order_relaxed);
    while (previous < duration_ns &&
           !tx_max_send_ns_.compare_exchange_weak(previous, duration_ns,
                                                   std::memory_order_relaxed,
                                                   std::memory_order_relaxed)) {
    }

    if (result == static_cast<ssize_t>(packet.size)) {
      tx_sends_.fetch_add(1, std::memory_order_relaxed);
      continue;
    }

    tx_send_errors_.fetch_add(1, std::memory_order_relaxed);
    if (send_error == EAGAIN) {
      tx_eagain_.fetch_add(1, std::memory_order_relaxed);
      if (!packet.latest_only && !requeue_low_priority(packet)) {
        tx_dropped_.fetch_add(1, std::memory_order_relaxed);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } else {
      tx_dropped_.fetch_add(1, std::memory_order_relaxed);
    }
  }
}

void UdpBridge::rx_loop() {
  uint8_t buf[kMaxDgram];
  PeerAddress current_peer{0, 0, 0};

  while (true) {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(udp_fd_, &fds);
    FD_SET(wake_[0], &fds);
    int maxfd = std::max(udp_fd_, wake_[0]) + 1;

    if (::select(maxfd, &fds, nullptr, nullptr, nullptr) < 0) {
      if (errno == EINTR) continue;
      break;
    }
    if (FD_ISSET(wake_[0], &fds)) break;

    if (FD_ISSET(udp_fd_, &fds)) {
      sockaddr_in sender{};
      socklen_t slen = sizeof(sender);
      ssize_t n = ::recvfrom(udp_fd_, buf, sizeof(buf), 0, reinterpret_cast<sockaddr*>(&sender), &slen);
      if (n < 0) continue;

      PeerAddress sender_val{sender.sin_addr.s_addr, sender.sin_port, 0};
      if (current_peer != sender_val) {
        current_peer = sender_val;
        active_peer_.store(current_peer, std::memory_order_release);
        tx_condition_.notify_all();
      }

      if (n < static_cast<ssize_t>(sizeof(uint16_t))) continue;

      uint16_t id_raw{};
      std::memcpy(&id_raw, buf, sizeof(id_raw));
      bus_.publish_if_authorized(static_cast<MsgId>(id_raw), buf + sizeof(uint16_t),
                                 static_cast<size_t>(n) - sizeof(uint16_t));
    }
  }
}

}  // namespace ipc
