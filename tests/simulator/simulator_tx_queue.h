#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <netinet/in.h>
#include <thread>
#include <utility>
#include <array>

#include "messages/balancer_msgs.h"

namespace simulator {

// SimulatorTelemetryPayload is the largest datagram emitted by the standalone
// simulator.  Keeping the storage fixed to that payload size avoids a second
// heap allocation for every queued packet while leaving room for the message
// id carried by the UDP framing.
constexpr std::size_t kSimulatorTxPacketBytes =
    sizeof(uint16_t) + sizeof(ipc::SimulatorTelemetryPayload);

enum class SimulatorTxKind : uint8_t {
  Telemetry,
  Control,
};

struct SimulatorTxPacket {
  sockaddr_in peer{};
  uint16_t size = 0;
  SimulatorTxKind kind = SimulatorTxKind::Control;
  std::array<uint8_t, kSimulatorTxPacketBytes> bytes{};
};

static_assert(sizeof(ipc::SimRunDonePayload) + sizeof(uint16_t) <=
              kSimulatorTxPacketBytes);
static_assert(sizeof(ipc::SimStartAckPayload) + sizeof(uint16_t) <=
              kSimulatorTxPacketBytes);

constexpr std::size_t kSimulatorTxQueueBytes = 8U * 1024U * 1024U;
constexpr std::size_t kSimulatorTxQueuePackets =
    kSimulatorTxQueueBytes / sizeof(SimulatorTxPacket);

// A small reserve lets a terminal control packet enter the queue even when
// telemetry has filled the ordinary admission limit.  The queue remains one
// FIFO, so this reserve does not change packet ordering.
template <std::size_t CapacityPackets = kSimulatorTxQueuePackets,
          std::size_t ControlReservePackets = 8>
class SimulatorTxQueue {
 public:
  using SendFunction = std::function<void(const SimulatorTxPacket&)>;

  static_assert(CapacityPackets > ControlReservePackets);

  SimulatorTxQueue() = default;
  SimulatorTxQueue(const SimulatorTxQueue&) = delete;
  SimulatorTxQueue& operator=(const SimulatorTxQueue&) = delete;

  ~SimulatorTxQueue() {
    stop();
  }

  void start(SendFunction send_function) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (worker_.joinable()) {
      return;
    }
    send_function_ = std::move(send_function);
    stopping_ = false;
    worker_ = std::thread(&SimulatorTxQueue::worker_main, this);
  }

  bool enqueue(SimulatorTxPacket packet, SimulatorTxKind kind) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (stopping_ || !worker_.joinable()) {
      return false;
    }

    if (kind == SimulatorTxKind::Telemetry) {
      // Telemetry never waits behind a full queue: the caller can turn this
      // result into a run-level transport error instead of silently dropping a
      // packet.  The reserved slots are available to terminal/control frames.
      if (queue_.size() >= CapacityPackets - ControlReservePackets) {
        return false;
      }
    } else {
      space_available_.wait(lock, [this] {
        return stopping_ || queue_.size() < CapacityPackets;
      });
      if (stopping_) {
        return false;
      }
    }

    packet.kind = kind;
    queue_.push_back(std::move(packet));
    items_available_.notify_one();
    return true;
  }

  void stop() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!worker_.joinable()) {
        return;
      }
      stopping_ = true;
    }
    items_available_.notify_all();
    space_available_.notify_all();
    worker_.join();
  }

  std::size_t size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

 private:
  void worker_main() {
    for (;;) {
      SimulatorTxPacket packet;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        items_available_.wait(lock, [this] { return stopping_ || !queue_.empty(); });
        if (queue_.empty()) {
          return;
        }
        packet = std::move(queue_.front());
        queue_.pop_front();
        space_available_.notify_all();
      }
      send_function_(packet);
    }
  }

  mutable std::mutex mutex_;
  std::condition_variable items_available_;
  std::condition_variable space_available_;
  std::deque<SimulatorTxPacket> queue_;
  SendFunction send_function_;
  std::thread worker_;
  bool stopping_ = false;
};

}  // namespace simulator
