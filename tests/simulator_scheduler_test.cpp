#include "simulator/simulator_scheduler.h"

#include <gtest/gtest.h>

#include <vector>

namespace {

class NoopWaveBackend final : public WaveFrameBackend {
 public:
  int queueFrame(unsigned, unsigned, bool) override {
    return next_id_++;
  }
  void deleteFrame(int) override {
  }
  void stop() override {
  }

 private:
  int next_id_{1};
};

TEST(SimulatorSchedulerTest, KeepsControllerClockSeparateFromQueuedEvents) {
  SimulatorTimeScheduler scheduler(2500);
  scheduler.schedule(SimulatorEvent{100, SimulatorEventKind::Step, 1, 1});
  scheduler.schedule(SimulatorEvent{200, SimulatorEventKind::Step, 1, 1});

  EXPECT_EQ(scheduler.next_controller_time_us(), 2500U);
  EXPECT_EQ(scheduler.next_event_time_us(2500), 100U);
  scheduler.advance_to(100);
  ASSERT_EQ(scheduler.pop_events_at(100).size(), 1U);
  EXPECT_EQ(scheduler.next_controller_time_us(), 2500U);
  scheduler.advance_to(2500);
  scheduler.controller_sample_processed(2500);
  EXPECT_EQ(scheduler.next_controller_time_us(), 5000U);
}

TEST(SimulatorSchedulerTest, MotorRunnerExposesEveryHighRateStepAtWaveTimestamp) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  NoopWaveBackend backend;
  MotorRunner runner(left, right, 400.0, 1e9, &backend);

  runner.setTargets(10000.0, 10000.0, 0);
  const auto events = runner.getScheduledStepEvents(0, 2500);
  ASSERT_EQ(events.size(), 25U);
  for (size_t index = 0; index < events.size(); ++index) {
    EXPECT_EQ(events[index].timestamp_us, 50U + 100U * index);
    EXPECT_EQ(events[index].left_step_delta, 1);
    EXPECT_EQ(events[index].right_step_delta, 1);
  }

  const auto two_frames = runner.getScheduledStepEvents(0, 5000);
  ASSERT_EQ(two_frames.size(), 50U);
  EXPECT_EQ(two_frames.front().timestamp_us, 50U);
  EXPECT_EQ(two_frames.back().timestamp_us, 4950U);
}

TEST(SimulatorSchedulerTest, VerifiedOneThirtySecondAuthorityFitsTimestampedWaveFrame) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  NoopWaveBackend backend;
  MotorRunner runner(left, right, 400.0, 1e9, &backend);

  EXPECT_EQ(DualWave::kMaxScheduledHz,
            static_cast<unsigned>(Config::max_step_rate_sps));
  runner.setTargets(Config::max_step_rate_sps, Config::max_step_rate_sps, 0);
  const auto events = runner.getScheduledStepEvents(0, DualWave::kFrameUs);
  const auto expected_events = static_cast<std::size_t>(
      Config::max_step_rate_sps * static_cast<double>(DualWave::kFrameUs) / 1e6);
  ASSERT_EQ(events.size(), expected_events);
  uint64_t previous_timestamp = 0;
  for (const auto& event : events) {
    EXPECT_GT(event.timestamp_us, previous_timestamp);
    EXPECT_LE(event.timestamp_us, DualWave::kFrameUs);
    EXPECT_EQ(event.left_step_delta, 1);
    EXPECT_EQ(event.right_step_delta, 1);
    previous_timestamp = event.timestamp_us;
  }
  EXPECT_EQ(events.front().timestamp_us, 8U);
  EXPECT_EQ(events.back().timestamp_us, 2492U);
}

TEST(SimulatorSchedulerTest, EventCountsRemainAccurateAcrossSupportedRates) {
  constexpr uint64_t kFrameUs = 2500;
  constexpr unsigned kFrames = 40;
  for (const double rate_sps : {100.0, 500.0, 1000.0, 3200.0, 8000.0, 10000.0,
                               32000.0, 64000.0}) {
    Stepper left(1, Stepper::Pins{5, 6, 13});
    Stepper right(1, Stepper::Pins{7, 8, 14});
    NoopWaveBackend backend;
    MotorRunner runner(left, right, 400.0, 1e9, &backend);

    std::size_t event_count = 0;
    uint64_t previous_timestamp = 0;
    for (unsigned frame = 0; frame < kFrames; ++frame) {
      const uint64_t start_us = static_cast<uint64_t>(frame) * kFrameUs;
      runner.setTargets(rate_sps, rate_sps, start_us);
      const auto events = runner.getScheduledStepEvents(start_us, start_us + kFrameUs);
      for (const auto& event : events) {
        EXPECT_GT(event.timestamp_us, start_us);
        EXPECT_LE(event.timestamp_us, start_us + kFrameUs);
        EXPECT_EQ(event.left_step_delta, 1);
        EXPECT_EQ(event.right_step_delta, 1);
        if (event_count > 0) {
          EXPECT_GT(event.timestamp_us, previous_timestamp);
        }
        previous_timestamp = event.timestamp_us;
        ++event_count;
      }
    }
    const double expected = rate_sps * static_cast<double>(kFrames * kFrameUs) / 1e6;
    EXPECT_NEAR(static_cast<double>(event_count), expected, 1.0) << "rate=" << rate_sps;
  }
}

TEST(SimulatorSchedulerTest, WaveEventsPreserveDirectionAcrossQueuedFrames) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  NoopWaveBackend backend;
  MotorRunner runner(left, right, 400.0, 1e9, &backend);

  runner.setTargets(1000.0, 1000.0, 0);
  const auto forward = runner.getScheduledStepEvents(0, 2500);
  ASSERT_EQ(forward.size(), 2U);
  EXPECT_EQ(forward[0].left_step_delta, 1);
  EXPECT_EQ(forward[1].left_step_delta, 1);

  runner.setTargets(-1000.0, -1000.0, 2500);
  const auto reverse = runner.getScheduledStepEvents(2500, 5000);
  ASSERT_EQ(reverse.size(), 2U);
  EXPECT_EQ(reverse[0].left_step_delta, -1);
  EXPECT_EQ(reverse[0].right_step_delta, -1);
  EXPECT_EQ(reverse[1].left_step_delta, -1);
}

}  // namespace
