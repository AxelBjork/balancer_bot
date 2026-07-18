#include "services/motor/motor_runner.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>
#include <thread>
#include <vector>

#include "services/main/config.h"
#include "services/motor/motor_service.h"

extern "C" void pigpio_stub_reset();
int pigpio_stub_get_gpio_level(int pin);

namespace {

class RecordingWaveBackend final : public WaveFrameBackend {
 public:
  struct Frame {
    unsigned left_pulses;
    unsigned right_pulses;
    bool synchronous;
  };

  int queueFrame(unsigned left_pulses, unsigned right_pulses, bool synchronous) override {
    if (fail_next) {
      fail_next = false;
      return -1;
    }
    frames.push_back({left_pulses, right_pulses, synchronous});
    return next_id++;
  }

  void deleteFrame(int) override {
    ++deleted_frames;
  }

  void stop() override {
    ++stop_calls;
  }

  std::vector<Frame> frames;
  bool fail_next{false};
  int deleted_frames{0};
  int stop_calls{0};

 private:
  int next_id{1};
};

}  // namespace

class MotorRunnerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    pigpio_stub_reset();
  }
  // Helper to pump the loop
  void RunFor(MotorRunner& runner, double spsL, double spsR, std::chrono::milliseconds duration) {
    auto start = std::chrono::steady_clock::now();
    auto end = start + duration;
    while (std::chrono::steady_clock::now() < end) {
      const auto now_us = static_cast<uint64_t>(
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::steady_clock::now().time_since_epoch())
              .count());
      runner.setTargets(spsL, spsR, now_us);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));  // Pump at ~500Hz
    }
  }

  void RunDeterministic(MotorRunner& runner, double spsL, double spsR, uint64_t& now_us,
                        int ticks, uint64_t dt_us = 2500) {
    for (int i = 0; i < ticks; ++i) {
      now_us += dt_us;
      runner.setTargets(spsL, spsR, now_us);
    }
  }
};

TEST_F(MotorRunnerTest, StepTrackingForwardConstantRate) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  MotorRunner runner(left, right, 1000.0);

  // Run at 100 sps for 100ms
  RunFor(runner, 100.0, 100.0, std::chrono::milliseconds(100));

  // Expected: ~10 steps (100 sps * 0.1s)
  // Tolerance increased due to S-D jitter and sleep timing
  int64_t leftSteps = runner.getLeftSteps();
  int64_t rightSteps = runner.getRightSteps();

  EXPECT_NEAR(static_cast<double>(leftSteps), 10.0, 3.0) << "Left steps should be ~10";
  EXPECT_NEAR(static_cast<double>(rightSteps), 10.0, 3.0) << "Right steps should be ~10";
}

TEST_F(MotorRunnerTest, StepTrackingReverseDirection) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  MotorRunner runner(left, right, 1000.0);

  RunFor(runner, -100.0, -100.0, std::chrono::milliseconds(100));

  int64_t leftSteps = runner.getLeftSteps();
  int64_t rightSteps = runner.getRightSteps();

  EXPECT_NEAR(static_cast<double>(leftSteps), -10.0, 3.0) << "Left steps should be ~-10";
  EXPECT_NEAR(static_cast<double>(rightSteps), -10.0, 3.0) << "Right steps should be ~-10";
}

TEST_F(MotorRunnerTest, InvertedMotorWritesPhysicalDirectionForBothCommandPolarities) {
  constexpr int kLeftDirPin = 13;
  constexpr int kRightDirPin = 14;
  Stepper left(1, Stepper::Pins{5, 6, kLeftDirPin});
  Stepper right(1, Stepper::Pins{7, 8, kRightDirPin}, true);
  RecordingWaveBackend backend;
  MotorRunner runner(left, right, 400.0, 1e9, &backend);

  // Logical forward uses opposite physical DIR levels for the mirrored motors.
  EXPECT_TRUE(left.dirForward());
  EXPECT_TRUE(right.dirForward());
  EXPECT_EQ(pigpio_stub_get_gpio_level(kLeftDirPin), 1);
  EXPECT_EQ(pigpio_stub_get_gpio_level(kRightDirPin), 0);

  // A first command in reverse must update both physical pins, including the
  // inverted right motor whose startup level previously remained stale.
  runner.setTargets(-500.0, -500.0, 1000);
  EXPECT_FALSE(left.dirForward());
  EXPECT_FALSE(right.dirForward());
  EXPECT_EQ(pigpio_stub_get_gpio_level(kLeftDirPin), 0);
  EXPECT_EQ(pigpio_stub_get_gpio_level(kRightDirPin), 1);

  runner.setTargets(500.0, 500.0, 2000);
  EXPECT_TRUE(left.dirForward());
  EXPECT_TRUE(right.dirForward());
  EXPECT_EQ(pigpio_stub_get_gpio_level(kLeftDirPin), 1);
  EXPECT_EQ(pigpio_stub_get_gpio_level(kRightDirPin), 0);
}

TEST_F(MotorRunnerTest, HardwareConfigMapsRobotForwardToCalibratedDirLevels) {
  constexpr int kLeftDirPin = 13;
  constexpr int kRightDirPin = 24;
  Stepper left(1, Stepper::Pins{12, 19, kLeftDirPin}, Config::invert_left);
  Stepper right(1, Stepper::Pins{4, 18, kRightDirPin}, Config::invert_right);
  RecordingWaveBackend backend;
  MotorRunner runner(left, right, 400.0, 1e9, &backend);

  runner.setTargets(500.0, 500.0, 1000);
  EXPECT_EQ(pigpio_stub_get_gpio_level(kLeftDirPin), 0);
  EXPECT_EQ(pigpio_stub_get_gpio_level(kRightDirPin), 1);

  runner.setTargets(-500.0, -500.0, 2000);
  EXPECT_EQ(pigpio_stub_get_gpio_level(kLeftDirPin), 1);
  EXPECT_EQ(pigpio_stub_get_gpio_level(kRightDirPin), 0);
}

TEST_F(MotorRunnerTest, StepTrackingDifferentialSteering) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  MotorRunner runner(left, right, 1000.0);

  RunFor(runner, 100.0, -100.0, std::chrono::milliseconds(100));

  int64_t leftSteps = runner.getLeftSteps();
  int64_t rightSteps = runner.getRightSteps();

  EXPECT_NEAR(static_cast<double>(leftSteps), 10.0, 3.0) << "Left steps should be ~10";
  EXPECT_NEAR(static_cast<double>(rightSteps), -10.0, 3.0) << "Right steps should be ~-10";
}

TEST_F(MotorRunnerTest, StepTrackingAccumulation) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  MotorRunner runner(left, right, 1000.0);

  // The completed-pulse count includes the configured slew and queued-frame latency.
  RunFor(runner, 1000.0, 1000.0, std::chrono::milliseconds(50));

  // It must report completed frames, not the ideal 50 requested steps.
  int64_t leftSteps = runner.getLeftSteps();
  int64_t rightSteps = runner.getRightSteps();

  EXPECT_GT(leftSteps, 20);
  EXPECT_LT(leftSteps, 50);
  EXPECT_EQ(rightSteps, leftSteps);
}

TEST_F(MotorRunnerTest, StepTrackingZeroRate) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  MotorRunner runner(left, right, 1000.0);

  RunFor(runner, 0.0, 0.0, std::chrono::milliseconds(100));

  int64_t leftSteps = runner.getLeftSteps();
  int64_t rightSteps = runner.getRightSteps();

  EXPECT_NEAR(static_cast<double>(leftSteps), 0.0, 1.0) << "Left steps should be 0";
  EXPECT_NEAR(static_cast<double>(rightSteps), 0.0, 1.0) << "Right steps should be 0";
}

TEST_F(MotorRunnerTest, VelocityEstimation) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  MotorRunner runner(left, right, 1000.0);

  // Init state (first call returns 0 and sets baseline)
  runner.getActualSpeedSps();

  // Move forward 1000 sps for 500ms
  // Note: RunFor pumps at ~500Hz (2ms sleep), so plenty of updates.
  RunFor(runner, 400.0, 400.0, std::chrono::milliseconds(50));

  double v = runner.getActualSpeedSps();
  // Allow loose tolerance due to simulation timing jitter
  EXPECT_NEAR(v, 400.0, 150.0) << "Velocity should be approx 1000 sps";

  // Differential (spin)
  RunFor(runner, 400.0, -400.0, std::chrono::milliseconds(80));
  v = runner.getActualSpeedSps();
  EXPECT_NEAR(v, 0.0, 25.0) << "Average velocity should be 0 after the pulse window flushes";
}

TEST_F(MotorRunnerTest, VelocityEstimationResolvesBelowOnePulsePerFrame) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  MotorRunner runner(left, right, 400.0, 50000.0);

  runner.getActualSpeedSps();
  RunFor(runner, 100.0, 100.0, std::chrono::milliseconds(100));

  const double v = runner.getActualSpeedSps();
  EXPECT_NEAR(v, 100.0, 35.0) << "Estimated velocity should average below one wave-frame pulse";
}

TEST_F(MotorRunnerTest, VelocityEstimationUsesActualStepDeltaOverWindow) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  MotorRunner runner(left, right, 400.0, 500000.0);

  uint64_t now_us = 1000;
  RunDeterministic(runner, 400.0, 400.0, now_us, 60);

  const auto feedback = runner.getFeedbackSample();
  EXPECT_NEAR(feedback.update_dt_ms, 2.5, 1e-9);
  EXPECT_NEAR(feedback.measured_avg_sps, 400.0, 20.0);
  EXPECT_NEAR(static_cast<double>(feedback.left_actual_steps), 59.0, 1.0);
  EXPECT_NEAR(static_cast<double>(feedback.right_actual_steps), 59.0, 1.0);
}

TEST_F(MotorRunnerTest, VelocityEstimationAveragesSubFrameSigmaDeltaSteps) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  MotorRunner runner(left, right, 400.0, 500000.0);

  uint64_t now_us = 1000;
  RunDeterministic(runner, 125.0, 125.0, now_us, 100);

  const auto feedback = runner.getFeedbackSample();
  EXPECT_NEAR(feedback.measured_avg_sps, 125.0, 25.0);
  EXPECT_NEAR(static_cast<double>(feedback.left_actual_steps), 31.0, 2.0);
  EXPECT_NEAR(static_cast<double>(feedback.right_actual_steps), 31.0, 2.0);
}

TEST_F(MotorRunnerTest, AppliedFeedbackReportsQuantizedFrameAndAccurateWindowAverage) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  MotorRunner runner(left, right, 400.0, 50000.0);

  RunFor(runner, 125.0, 125.0, std::chrono::milliseconds(60));

  const auto feedback = runner.getFeedbackSample();
  EXPECT_GT(feedback.left_applied_sps, 0.0);
  EXPECT_GT(feedback.right_applied_sps, 0.0);
  EXPECT_LE(feedback.left_applied_sps, 200.0);
  EXPECT_LE(feedback.right_applied_sps, 200.0);
  EXPECT_NEAR(feedback.measured_avg_sps, 125.0, 35.0);
}

TEST_F(MotorRunnerTest, CountsOnlyCompletedFrames) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  RecordingWaveBackend backend;
  MotorRunner runner(left, right, 400.0, 1e9, &backend);

  runner.setTargets(1000.0, 1000.0, 1000);
  EXPECT_EQ(runner.getActualLeftSteps(), 0);
  ASSERT_EQ(backend.frames.size(), 2U);
  EXPECT_EQ(backend.frames[0].left_pulses, 5U);
  EXPECT_FALSE(backend.frames[0].synchronous);
  EXPECT_TRUE(backend.frames[1].synchronous);

  runner.setTargets(1000.0, 1000.0, 5999);
  EXPECT_EQ(runner.getActualLeftSteps(), 0);
  runner.setTargets(1000.0, 1000.0, 6000);
  EXPECT_EQ(runner.getActualLeftSteps(), 5);
  EXPECT_EQ(runner.getActualRightSteps(), 5);
}

TEST_F(MotorRunnerTest, PhysicalPulsePositionAdvancesInsideFrameButFeedbackWaitsForCompletion) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  RecordingWaveBackend backend;
  MotorRunner runner(left, right, 400.0, 1e9, &backend);

  runner.setTargets(1000.0, 1000.0, 1000);
  EXPECT_EQ(runner.getFeedbackSample().left_actual_steps, 0);
  EXPECT_DOUBLE_EQ(runner.getScheduledStepPosition(1001).left_steps, 0.0);
  EXPECT_DOUBLE_EQ(runner.getScheduledStepPosition(2600).left_steps, 2.0);
  EXPECT_EQ(runner.getFeedbackSample().left_actual_steps, 0);
  EXPECT_DOUBLE_EQ(runner.getScheduledStepPosition(5600).left_steps, 5.0);

  runner.setTargets(1000.0, 1000.0, 6000);
  EXPECT_EQ(runner.getFeedbackSample().left_actual_steps, 5);
}

TEST_F(MotorRunnerTest, FractionalPhaseProducesCorrectLowRatePulseCount) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  RecordingWaveBackend backend;
  MotorRunner runner(left, right, 400.0, 1e9, &backend);

  uint64_t now_us = 1000;
  RunDeterministic(runner, 125.0, 125.0, now_us, 405);

  const auto feedback = runner.getFeedbackSample();
  EXPECT_NEAR(static_cast<double>(feedback.left_actual_steps), 125.0, 2.0);
  EXPECT_NEAR(feedback.measured_avg_sps, 125.0, 25.0);
  ASSERT_GT(backend.frames.size(), 100U);
  for (const auto& frame : backend.frames) {
    EXPECT_LE(frame.left_pulses, 1U);
    EXPECT_LE(frame.right_pulses, 1U);
  }
}

TEST_F(MotorRunnerTest, SupportsOneAndTwelveThousandSps) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  RecordingWaveBackend backend;
  MotorRunner runner(left, right, 400.0, 1e9, &backend);

  uint64_t now_us = 1000;
  RunDeterministic(runner, 1.0, 1.0, now_us, 805);
  EXPECT_NEAR(static_cast<double>(runner.getActualLeftSteps()), 2.0, 1.0);

  runner.stop();
  backend.frames.clear();
  RunDeterministic(runner, 12000.0, 12000.0, now_us, 8);
  ASSERT_FALSE(backend.frames.empty());
  EXPECT_EQ(backend.frames.front().left_pulses, 60U);
  EXPECT_EQ(backend.frames.front().right_pulses, 60U);
}

TEST_F(MotorRunnerTest, WaveFailureLatchesActuatorFaultAndZeroFeedback) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  RecordingWaveBackend backend;
  backend.fail_next = true;
  MotorRunner runner(left, right, 400.0, 1e9, &backend);

  runner.setTargets(500.0, 500.0, 1000);
  const auto feedback = runner.getFeedbackSample();
  EXPECT_TRUE(feedback.actuator_fault);
  EXPECT_DOUBLE_EQ(feedback.left_applied_sps, 0.0);
  EXPECT_DOUBLE_EQ(feedback.right_applied_sps, 0.0);
  EXPECT_GT(backend.stop_calls, 0);
}

TEST_F(MotorRunnerTest, ReversalStopsQueuedFramesBeforeChangingDirection) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  RecordingWaveBackend backend;
  MotorRunner runner(left, right, 400.0, 1e9, &backend);

  runner.setTargets(500.0, 500.0, 1000);
  ASSERT_EQ(backend.frames.size(), 2U);
  runner.setTargets(-500.0, -500.0, 2000);

  EXPECT_GT(backend.stop_calls, 0);
  EXPECT_FALSE(left.dirForward());
  EXPECT_FALSE(right.dirForward());
  EXPECT_EQ(runner.getActualLeftSteps(), 0);

  runner.setTargets(-500.0, -500.0, 7000);
  EXPECT_LT(runner.getActualLeftSteps(), 0);
  EXPECT_LT(runner.getActualRightSteps(), 0);
}

TEST_F(MotorRunnerTest, ReversalAccountsForPulsesAlreadyEmittedInStoppedFrame) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  RecordingWaveBackend backend;
  MotorRunner runner(left, right, 400.0, 1e9, &backend);

  runner.setTargets(12000.0, 12000.0, 1000);
  EXPECT_DOUBLE_EQ(runner.getScheduledStepPosition(3500).left_steps, 30.0);
  runner.setTargets(-12000.0, -12000.0, 3500);

  EXPECT_EQ(runner.getActualLeftSteps(), 30);
  EXPECT_EQ(runner.getActualRightSteps(), 30);
  EXPECT_DOUBLE_EQ(runner.getScheduledStepPosition(3500).left_steps, 30.0);
}

TEST_F(MotorRunnerTest, MotorServicePublishesActuatorFault) {
  struct FeedbackSink {
    std::vector<ipc::MotorFeedbackPayload> samples;
    static void dispatch(void* ctx, MsgId id, const void* payload) {
      if (id == MsgId::MotorFeedback) {
        static_cast<FeedbackSink*>(ctx)->samples.push_back(
            unpack_payload<MsgId::MotorFeedback>(payload));
      }
    }
  } sink;

  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  RecordingWaveBackend backend;
  backend.fail_next = true;
  MotorRunner runner(left, right, 400.0, 1e9, &backend);
  ipc::MessageBus bus(&sink, FeedbackSink::dispatch);
  sil::MotorService service(bus, &runner);

  service.on_message<MsgId::PhysicsTick>(PhysicsTickPayload{0.0025, 1000});
  service.on_message<MsgId::MotorTargets>(ipc::MotorTargetsPayload{500.0, 500.0});

  ASSERT_EQ(sink.samples.size(), 1U);
  EXPECT_EQ(sink.samples.back().actuator_fault, 1U);
  EXPECT_DOUBLE_EQ(sink.samples.back().left_applied_sps, 0.0);
}

TEST_F(MotorRunnerTest, FeedbackReportsUpdateTiming) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  MotorRunner runner(left, right, 400.0, 50000.0);

  RunFor(runner, 125.0, 125.0, std::chrono::milliseconds(60));

  const auto feedback = runner.getFeedbackSample();
  EXPECT_GT(feedback.update_dt_ms, 0.0);
  EXPECT_GE(feedback.feedback_age_ms, 0.0);
}

TEST_F(MotorRunnerTest, FeedbackSnapshotReflectsAppliedStateNotRawTarget) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  MotorRunner runner(left, right, 400.0, 50000.0);

  RunFor(runner, 4000.0, 4000.0, std::chrono::milliseconds(50));

  const auto feedback = runner.getFeedbackSample();
  EXPECT_LT(feedback.left_applied_sps, 4000.0);
  EXPECT_LT(feedback.right_applied_sps, 4000.0);
  EXPECT_GT(feedback.left_applied_sps, 0.0);
  EXPECT_GT(feedback.right_applied_sps, 0.0);
  EXPECT_GT(feedback.left_actual_steps, 0);
  EXPECT_GT(feedback.right_actual_steps, 0);
}

TEST_F(MotorRunnerTest, FeedbackSignMatchesAppliedDirectionWithRightMotorInversion) {
  {
    Stepper left(1, Stepper::Pins{5, 6, 13});
    Stepper right(1, Stepper::Pins{7, 8, 14}, true);
    MotorRunner runner(left, right, 400.0, 50000.0);

    RunFor(runner, 800.0, 800.0, std::chrono::milliseconds(80));
    const auto feedback = runner.getFeedbackSample();
    EXPECT_GT(feedback.left_applied_sps, 0.0);
    EXPECT_GT(feedback.right_applied_sps, 0.0);
    EXPECT_GT(feedback.left_actual_steps, 0);
    EXPECT_GT(feedback.right_actual_steps, 0);
  }

  {
    Stepper left(1, Stepper::Pins{15, 16, 17});
    Stepper right(1, Stepper::Pins{18, 19, 20}, true);
    MotorRunner runner(left, right, 400.0, 50000.0);

    RunFor(runner, -800.0, -800.0, std::chrono::milliseconds(80));
    const auto feedback = runner.getFeedbackSample();
    EXPECT_LT(feedback.left_applied_sps, 0.0);
    EXPECT_LT(feedback.right_applied_sps, 0.0);
    EXPECT_LT(feedback.left_actual_steps, 0);
    EXPECT_LT(feedback.right_actual_steps, 0);
  }
}
