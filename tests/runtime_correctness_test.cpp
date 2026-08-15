#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include "messages/balancer_msgs.h"
#include "publisher.h"
#include "services/imu/imu_sample_synchronizer.h"

namespace {

TEST(ImuSampleSynchronizerTest, SelectsNearestSamplesAndRejectsExcessiveSkew) {
  ImuSampleSynchronizer sync;
  EXPECT_FALSE(sync.push_accel({{1.0, 0.0, 0.0}, 1'000'000}));
  EXPECT_FALSE(sync.push_accel({{2.0, 0.0, 0.0}, 2'000'000}));

  const auto nearest = sync.push_gyro({{0.0, 3.0, 0.0}, 1'900'000});
  ASSERT_TRUE(nearest);
  EXPECT_DOUBLE_EQ(nearest->accel.value[0], 2.0);
  EXPECT_EQ(nearest->skew_ns(), 100'000);

  EXPECT_FALSE(sync.push_accel({{4.0, 0.0, 0.0}, 10'000'000}));
  EXPECT_FALSE(sync.push_gyro({{0.0, 5.0, 0.0}, 13'000'001}));
  const auto recovered = sync.push_accel({{6.0, 0.0, 0.0}, 13'000'000});
  ASSERT_TRUE(recovered);
  EXPECT_EQ(recovered->skew_ns(), 1);
  EXPECT_DOUBLE_EQ(recovered->accel.value[0], 6.0);
}

TEST(ImuSampleSynchronizerTest, DefaultSkewCannotSpanAn833HzSamplePeriod) {
  ImuSampleSynchronizer sync;
  EXPECT_FALSE(sync.push_accel({{1.0, 0.0, 0.0}, 1'000'000}));
  EXPECT_FALSE(sync.push_gyro({{0.0, 2.0, 0.0}, 1'800'000}));

  const auto recovered = sync.push_accel({{3.0, 0.0, 0.0}, 1'800'001});
  ASSERT_TRUE(recovered);
  EXPECT_EQ(recovered->skew_ns(), 1);
  EXPECT_DOUBLE_EQ(recovered->accel.value[0], 3.0);
}

struct DispatchProbe {
  ipc::MessageBus* bus{nullptr};
  std::atomic<int> active_top_level{0};
  std::atomic<bool> overlapped{false};
  std::atomic<int> ticks{0};
  std::atomic<int> nested_targets{0};
};

thread_local int dispatch_depth = 0;

void serialized_dispatch(void* ctx, MsgId id, const void* payload) {
  auto* probe = static_cast<DispatchProbe*>(ctx);
  const bool top_level = dispatch_depth++ == 0;
  if (top_level && probe->active_top_level.fetch_add(1, std::memory_order_acq_rel) != 0) {
    probe->overlapped.store(true, std::memory_order_release);
  }

  if (id == MsgId::PhysicsTick) {
    ++probe->ticks;
    ipc::MotorTargetsPayload target{};
    target.left_sps = static_cast<double>(unpack_payload<MsgId::PhysicsTick>(payload).timestamp_us);
    probe->bus->publish<MsgId::MotorTargets>(target);
    std::this_thread::yield();
  } else if (id == MsgId::MotorTargets) {
    ++probe->nested_targets;
  }

  if (top_level) {
    probe->active_top_level.fetch_sub(1, std::memory_order_acq_rel);
  }
  --dispatch_depth;
}

TEST(MessageBusTest, SerializesConcurrentProducersWhileAllowingNestedDispatch) {
  DispatchProbe probe;
  ipc::MessageBus bus(&probe, serialized_dispatch);
  probe.bus = &bus;

  constexpr int kThreads = 4;
  constexpr int kTicksPerThread = 250;
  std::vector<std::thread> workers;
  for (int worker = 0; worker < kThreads; ++worker) {
    workers.emplace_back([worker, &bus]() {
      for (int tick = 0; tick < kTicksPerThread; ++tick) {
        PhysicsTickPayload payload{};
        payload.dt_s = 0.0025;
        payload.timestamp_us = static_cast<uint64_t>(worker * kTicksPerThread + tick + 1);
        bus.publish<MsgId::PhysicsTick>(payload);
      }
    });
  }
  for (auto& worker : workers) {
    worker.join();
  }

  EXPECT_FALSE(probe.overlapped.load());
  EXPECT_EQ(probe.ticks.load(), kThreads * kTicksPerThread);
  EXPECT_EQ(probe.nested_targets.load(), kThreads * kTicksPerThread);
}

}  // namespace
