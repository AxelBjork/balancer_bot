#include "gtest/gtest.h"

#include <condition_variable>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <vector>

#include "simulator/simulator_tx_queue.h"

namespace {

simulator::SimulatorTxPacket packet(uint8_t id) {
  simulator::SimulatorTxPacket out{};
  out.size = 1;
  out.bytes[0] = id;
  return out;
}

}  // namespace

TEST(SimulatorTxQueueTest, PreservesFifoOrderAndRejectsEnqueueAfterShutdown) {
  simulator::SimulatorTxQueue<8, 1> queue;
  std::vector<uint8_t> delivered;
  queue.start([&](const simulator::SimulatorTxPacket& value) {
    delivered.push_back(value.bytes[0]);
  });

  for (uint8_t id = 0; id < 5; ++id) {
    ASSERT_TRUE(queue.enqueue(packet(id), simulator::SimulatorTxKind::Control));
  }
  queue.stop();

  ASSERT_EQ(delivered, (std::vector<uint8_t>{0, 1, 2, 3, 4}));
  EXPECT_FALSE(queue.enqueue(packet(5), simulator::SimulatorTxKind::Control));
}

TEST(SimulatorTxQueueTest, BoundsTelemetryAndLeavesReserveForControlPackets) {
  simulator::SimulatorTxQueue<3, 1> queue;
  std::mutex mutex;
  std::condition_variable condition;
  bool first_packet_started = false;
  bool release_first_packet = false;
  std::vector<uint8_t> delivered;
  queue.start([&](const simulator::SimulatorTxPacket& value) {
    if (value.bytes[0] == 0) {
      std::unique_lock<std::mutex> lock(mutex);
      first_packet_started = true;
      condition.notify_all();
      condition.wait(lock, [&] { return release_first_packet; });
    }
    std::lock_guard<std::mutex> lock(mutex);
    delivered.push_back(value.bytes[0]);
    condition.notify_all();
  });

  ASSERT_TRUE(queue.enqueue(packet(0), simulator::SimulatorTxKind::Control));
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(1),
                                   [&] { return first_packet_started; }));
  }

  ASSERT_TRUE(queue.enqueue(packet(1), simulator::SimulatorTxKind::Telemetry));
  ASSERT_TRUE(queue.enqueue(packet(2), simulator::SimulatorTxKind::Telemetry));
  EXPECT_FALSE(queue.enqueue(packet(3), simulator::SimulatorTxKind::Telemetry));

  // The control packet is admitted into the reserved slot and remains behind
  // the already queued telemetry in the same FIFO.
  ASSERT_TRUE(queue.enqueue(packet(4), simulator::SimulatorTxKind::Control));
  {
    std::lock_guard<std::mutex> lock(mutex);
    release_first_packet = true;
  }
  condition.notify_all();
  queue.stop();

  ASSERT_EQ(delivered, (std::vector<uint8_t>{0, 1, 2, 4}));
}
