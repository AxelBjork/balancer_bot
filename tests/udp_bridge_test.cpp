#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <thread>

#include "ipc/udp_bridge.h"

namespace {

void discard_dispatch(void*, MsgId, const void*) {
}

std::atomic<bool> fake_sender_entered{false};
std::atomic<bool> fake_sender_released{false};

ssize_t blocked_sender(int, const msghdr* message, int) {
  fake_sender_entered.store(true, std::memory_order_release);
  while (!fake_sender_released.load(std::memory_order_acquire)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return static_cast<ssize_t>(message->msg_iov[0].iov_len);
}

struct DispatchContext {
  ipc::UdpBridge* bridge = nullptr;
};

void dispatch_to_bridge(void* raw, MsgId id, const void* payload) {
  auto& context = *static_cast<DispatchContext*>(raw);
  if (id == MsgId::SystemTelemetry) {
    context.bridge->on_message<MsgId::SystemTelemetry>(
        unpack_payload<MsgId::SystemTelemetry>(payload));
  }
}

TEST(UdpBridgeTest, HighRateMessagesCoalesceWithoutGrowingQueue) {
  ipc::MessageBus bus(nullptr, &discard_dispatch);
  ipc::UdpBridge bridge(bus, 0);
  ipc::SystemTelemetryPayload payload{};
  payload.run_id = 7;

  for (uint64_t sequence = 1; sequence <= 100; ++sequence) {
    payload.packet_seq = sequence;
    payload.loop_seq = sequence;
    bridge.on_message<MsgId::SystemTelemetry>(payload);
  }

  const auto stats = bridge.tx_stats();
  EXPECT_EQ(stats.enqueued, 1U);
  EXPECT_EQ(stats.coalesced, 99U);
  EXPECT_EQ(stats.dropped, 0U);
  EXPECT_EQ(stats.queue_depth, 1U);
}

TEST(UdpBridgeTest, LowRateQueueIsBoundedWithoutBlockingPublisher) {
  ipc::MessageBus bus(nullptr, &discard_dispatch);
  ipc::UdpBridge bridge(bus, 0);
  ipc::PidConfigStatusPayload payload{};

  for (uint32_t request_id = 1; request_id <= 100; ++request_id) {
    payload.request_id = request_id;
    bridge.on_message<MsgId::PidConfigStatus>(payload);
  }

  const auto stats = bridge.tx_stats();
  EXPECT_EQ(stats.enqueued, 64U);
  EXPECT_EQ(stats.dropped, 36U);
  EXPECT_EQ(stats.queue_depth, 64U);
}

TEST(UdpBridgeTest, MessageBusPublisherDoesNotWaitForBlockedTxWorker) {
  fake_sender_entered.store(false, std::memory_order_release);
  fake_sender_released.store(false, std::memory_order_release);
  DispatchContext context;
  ipc::MessageBus bus(&context, &dispatch_to_bridge);
  ipc::UdpBridge bridge(bus, 0, &blocked_sender);
  context.bridge = &bridge;
  bridge.start();

  const int peer = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (peer < 0) {
    fake_sender_released.store(true, std::memory_order_release);
    ADD_FAILURE() << "could not create UDP peer socket";
    return;
  }
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(bridge.local_port());
  if (::inet_pton(AF_INET, "127.0.0.1", &address.sin_addr) != 1) {
    fake_sender_released.store(true, std::memory_order_release);
    ::close(peer);
    ADD_FAILURE() << "could not construct UDP peer address";
    return;
  }
  const char registration[] = "r";
  if (::sendto(peer, registration, sizeof(registration), 0,
               reinterpret_cast<sockaddr*>(&address), sizeof(address)) !=
      static_cast<ssize_t>(sizeof(registration))) {
    fake_sender_released.store(true, std::memory_order_release);
    ::close(peer);
    ADD_FAILURE() << "could not register UDP peer";
    return;
  }

  ipc::SystemTelemetryPayload payload{};
  payload.run_id = 1;
  payload.packet_seq = 1;
  payload.loop_seq = 1;
  bus.publish<MsgId::SystemTelemetry>(payload);
  const auto wait_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (!fake_sender_entered.load(std::memory_order_acquire) &&
         std::chrono::steady_clock::now() < wait_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (!fake_sender_entered.load(std::memory_order_acquire)) {
    fake_sender_released.store(true, std::memory_order_release);
    ::close(peer);
    FAIL() << "fake TX sender was not reached";
    return;
  }

  payload.packet_seq = 2;
  const auto publish_started = std::chrono::steady_clock::now();
  bus.publish<MsgId::SystemTelemetry>(payload);
  const auto publish_duration = std::chrono::steady_clock::now() - publish_started;
  EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(publish_duration).count(), 20);

  fake_sender_released.store(true, std::memory_order_release);
  ::close(peer);
}

}  // namespace
