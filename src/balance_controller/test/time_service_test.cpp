#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>

#include "publisher.h"
#include "services/time_service.h"

namespace {

struct TickSink {
  std::mutex mu;
  std::vector<PhysicsTickPayload> ticks;
};

void on_dispatch(void* ctx, MsgId id, const void* payload) {
  if (id != MsgId::PhysicsTick) {
    return;
  }

  auto* sink = static_cast<TickSink*>(ctx);
  std::lock_guard<std::mutex> lock(sink->mu);
  sink->ticks.push_back(unpack_payload<MsgId::PhysicsTick>(payload));
}

TEST(TimeServiceTest, AdvancePublishesExactTicks) {
  TickSink sink;
  ipc::MessageBus bus(&sink, on_dispatch);
  sil::TimeService time(bus);

  time.advance(0.01);
  time.advance(0.02);

  ASSERT_EQ(sink.ticks.size(), 2U);
  EXPECT_DOUBLE_EQ(sink.ticks[0].dt_s, 0.01);
  EXPECT_EQ(sink.ticks[0].sim_time_us, 10000U);
  EXPECT_DOUBLE_EQ(sink.ticks[1].dt_s, 0.02);
  EXPECT_EQ(sink.ticks[1].sim_time_us, 30000U);
}

TEST(TimeServiceTest, RuntimeThreadPublishesMonotonicTicks) {
  TickSink sink;
  ipc::MessageBus bus(&sink, on_dispatch);
  sil::TimeService time(bus, 1.0 / 400.0);

  time.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(15));
  time.stop();

  ASSERT_GE(sink.ticks.size(), 3U);
  for (size_t i = 1; i < sink.ticks.size(); ++i) {
    EXPECT_GT(sink.ticks[i].sim_time_us, sink.ticks[i - 1].sim_time_us);
    EXPECT_GT(sink.ticks[i].dt_s, 0.0);
  }
}

}  // namespace
