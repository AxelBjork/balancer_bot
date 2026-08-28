#include "services/motor/motor_runner.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
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

double rmsDifference(const std::vector<double>& lhs, const std::vector<double>& rhs,
                     size_t first_sample) {
  double squared_sum = 0.0;
  for (size_t index = first_sample; index < lhs.size(); ++index) {
    const double error = lhs[index] - rhs[index];
    squared_sum += error * error;
  }
  return std::sqrt(squared_sum / static_cast<double>(lhs.size() - first_sample));
}

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

  // A first opposite request brakes/qualifies at zero; it must not change DIR
  // until the second fresh opposite sample.
  runner.setTargets(-500.0, -500.0, 1000);
  EXPECT_TRUE(left.dirForward());
  EXPECT_TRUE(right.dirForward());
  EXPECT_EQ(pigpio_stub_get_gpio_level(kLeftDirPin), 1);
  EXPECT_EQ(pigpio_stub_get_gpio_level(kRightDirPin), 0);

  runner.setTargets(-500.0, -500.0, 2000);
  EXPECT_FALSE(left.dirForward());
  EXPECT_FALSE(right.dirForward());
  EXPECT_EQ(pigpio_stub_get_gpio_level(kLeftDirPin), 0);
  EXPECT_EQ(pigpio_stub_get_gpio_level(kRightDirPin), 1);

  runner.setTargets(500.0, 500.0, 3000);
  runner.setTargets(500.0, 500.0, 4000);
  runner.setTargets(500.0, 500.0, 5000);
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
  runner.setTargets(500.0, 500.0, 2000);
  EXPECT_EQ(pigpio_stub_get_gpio_level(kLeftDirPin), 0);
  EXPECT_EQ(pigpio_stub_get_gpio_level(kRightDirPin), 1);

  runner.setTargets(-500.0, -500.0, 2000);
  runner.setTargets(-500.0, -500.0, 3000);
  runner.setTargets(-500.0, -500.0, 4000);
  runner.setTargets(-500.0, -500.0, 5000);
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

TEST_F(MotorRunnerTest, StepDerivedFeedbackReportsAccurateWindowAverage) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  MotorRunner runner(left, right, 400.0, 50000.0);

  RunFor(runner, 125.0, 125.0, std::chrono::milliseconds(60));

  const auto feedback = runner.getFeedbackSample();
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
  EXPECT_EQ(backend.frames[0].left_pulses, 2U);
  EXPECT_FALSE(backend.frames[0].synchronous);
  EXPECT_TRUE(backend.frames[1].synchronous);

  runner.setTargets(1000.0, 1000.0, 3499);
  EXPECT_EQ(runner.getActualLeftSteps(), 0);
  runner.setTargets(1000.0, 1000.0, 3500);
  EXPECT_EQ(runner.getActualLeftSteps(), 2);
  EXPECT_EQ(runner.getActualRightSteps(), 2);
}

TEST_F(MotorRunnerTest, PhysicalPulsePositionAdvancesInsideFrameButFeedbackWaitsForCompletion) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  RecordingWaveBackend backend;
  MotorRunner runner(left, right, 400.0, 1e9, &backend);

  runner.setTargets(1000.0, 1000.0, 1000);
  EXPECT_EQ(runner.getFeedbackSample().left_actual_steps, 0);
  EXPECT_DOUBLE_EQ(runner.getScheduledStepPosition(1001).left_steps, 0.0);
  EXPECT_DOUBLE_EQ(runner.getScheduledStepPosition(2250).left_steps, 1.0);
  EXPECT_EQ(runner.getFeedbackSample().left_actual_steps, 0);
  EXPECT_DOUBLE_EQ(runner.getScheduledStepPosition(3400).left_steps, 2.0);

  runner.setTargets(1000.0, 1000.0, 3500);
  EXPECT_EQ(runner.getFeedbackSample().left_actual_steps, 2);
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
  EXPECT_EQ(backend.frames.front().left_pulses, 30U);
  EXPECT_EQ(backend.frames.front().right_pulses, 30U);
}

TEST_F(MotorRunnerTest, ProductionSlewAllowsExactlyFiveHundredSpsPerFourHundredHertzTick) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  RecordingWaveBackend backend;
  MotorRunner runner(left, right, Config::control_hz, Config::motor_slew_sps_per_s, &backend);

  runner.setTargets(2000.0, -2000.0, 2500);
  auto feedback = runner.getFeedbackSample();
  EXPECT_DOUBLE_EQ(feedback.left_slewed_sps, 500.0);
  EXPECT_DOUBLE_EQ(feedback.right_slewed_sps, 0.0);
  EXPECT_EQ(feedback.actuator_saturation_flags,
            ActuatorSaturationLeftSlew | ActuatorSaturationRightSlew);

  runner.setTargets(2000.0, -2000.0, 5000);
  feedback = runner.getFeedbackSample();
  EXPECT_DOUBLE_EQ(feedback.left_slewed_sps, 1000.0);
  EXPECT_DOUBLE_EQ(feedback.right_slewed_sps, -500.0);

  runner.setTargets(1000.0, -1000.0, 7500);
  feedback = runner.getFeedbackSample();
  EXPECT_DOUBLE_EQ(feedback.left_slewed_sps, 1000.0);
  EXPECT_DOUBLE_EQ(feedback.right_slewed_sps, -1000.0);
  EXPECT_EQ(feedback.actuator_saturation_flags, ActuatorSaturationNone);
}

TEST_F(MotorRunnerTest, NewTargetIsQueuedForTheNextTwoPointFiveMillisecondFrame) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  RecordingWaveBackend backend;
  MotorRunner runner(left, right, 400.0, 1e9, &backend);

  runner.setTargets(1000.0, 1000.0, 1000);
  ASSERT_EQ(backend.frames.size(), 2U);
  runner.setTargets(2000.0, 2000.0, 3500);

  ASSERT_EQ(backend.frames.size(), 3U);
  EXPECT_EQ(backend.frames.back().left_pulses, 5U);
  EXPECT_EQ(backend.frames.back().right_pulses, 5U);
  EXPECT_TRUE(backend.frames.back().synchronous);
  EXPECT_FALSE(runner.getFeedbackSample().actuator_fault);
}

TEST_F(MotorRunnerTest, NineHertzThreeThousandSpsTracksWithoutSteadySlewLimiting) {
  constexpr double kFrequencyHz = 9.0;
  constexpr double kOffsetSps = 5000.0;
  constexpr double kAmplitudeSps = 3000.0;
  constexpr double kDtS = 1.0 / 400.0;
  constexpr int kTicks = 1600;
  constexpr size_t kFirstMeasuredSample = 400;

  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  RecordingWaveBackend backend;
  MotorRunner runner(left, right, Config::control_hz, Config::motor_slew_sps_per_s, &backend);
  std::vector<double> requested;
  std::vector<double> slewed;
  requested.reserve(kTicks);
  slewed.reserve(kTicks);
  size_t steady_slew_ticks = 0;

  for (int tick = 0; tick < kTicks; ++tick) {
    const double time_s = static_cast<double>(tick + 1) * kDtS;
    const double target = kOffsetSps + kAmplitudeSps *
        std::sin(2.0 * M_PI * kFrequencyHz * time_s);
    runner.setTargets(target, target, static_cast<uint64_t>((tick + 1) * 2500));
    const auto feedback = runner.getFeedbackSample();
    requested.push_back(target);
    slewed.push_back(feedback.left_slewed_sps);
    if (static_cast<size_t>(tick) >= kFirstMeasuredSample &&
        feedback.actuator_saturation_flags != ActuatorSaturationNone) {
      ++steady_slew_ticks;
    }
  }

  EXPECT_EQ(steady_slew_ticks, 0U);
  EXPECT_NEAR(rmsDifference(requested, slewed, kFirstMeasuredSample), 0.0, 1e-9);
  EXPECT_FALSE(runner.getFeedbackSample().actuator_fault);
  EXPECT_GE(backend.frames.size(), static_cast<size_t>(kTicks));
}

TEST_F(MotorRunnerTest, DelayedUpdateCannotCrossZeroInOneSlewStep) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  RecordingWaveBackend backend;
  MotorRunner runner(left, right, 400.0, Config::motor_slew_sps_per_s, &backend);

  runner.setTargets(5000.0, 5000.0, 2500);
  runner.setTargets(-5000.0, -5000.0, 22500);

  const auto feedback = runner.getFeedbackSample();
  EXPECT_DOUBLE_EQ(feedback.left_slewed_sps, 0.0);
  EXPECT_DOUBLE_EQ(feedback.right_slewed_sps, 0.0);
  EXPECT_TRUE(left.dirForward());
  EXPECT_TRUE(right.dirForward());
}

TEST_F(MotorRunnerTest, ZeroHoldKeepsStepperEnergizedWithoutDirectionChatter) {
  constexpr int kLeftEnablePin = 5;
  constexpr int kRightEnablePin = 7;
  Stepper left(1, Stepper::Pins{kLeftEnablePin, 6, 13});
  Stepper right(1, Stepper::Pins{kRightEnablePin, 8, 14});
  RecordingWaveBackend backend;
  MotorRunner runner(left, right, 400.0, 1e9, &backend);

  runner.setTargets(500.0, 500.0, 1000);
  runner.setTargets(-100.0, -100.0, 2000);
  runner.setTargets(-100.0, -100.0, 3000);
  runner.setTargets(100.0, 100.0, 4000);
  runner.setTargets(-100.0, -100.0, 5000);
  runner.setTargets(100.0, 100.0, 6000);

  EXPECT_EQ(backend.stop_calls, 0);
  EXPECT_TRUE(left.dirForward());
  EXPECT_TRUE(right.dirForward());
  EXPECT_EQ(pigpio_stub_get_gpio_level(kLeftEnablePin), 1);
  EXPECT_EQ(pigpio_stub_get_gpio_level(kRightEnablePin), 1);
}

TEST_F(MotorRunnerTest, PersistentLowRateOppositeRequestEventuallyReverses) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  RecordingWaveBackend backend;
  MotorRunner runner(left, right, 400.0, 1e9, &backend);

  runner.setTargets(-100.0, -100.0, 1000);
  runner.setTargets(-100.0, -100.0, 2000);
  runner.setTargets(-100.0, -100.0, 3000);
  EXPECT_TRUE(left.dirForward());
  runner.setTargets(-100.0, -100.0, 4000);

  EXPECT_FALSE(left.dirForward());
  EXPECT_FALSE(right.dirForward());
  EXPECT_EQ(backend.stop_calls, 1);
  EXPECT_DOUBLE_EQ(runner.getFeedbackSample().left_slewed_sps, -100.0);
}

TEST_F(MotorRunnerTest, CancellingPendingReversalPreservesDirection) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  RecordingWaveBackend backend;
  MotorRunner runner(left, right, 400.0, 1e9, &backend);

  runner.setTargets(500.0, 500.0, 1000);
  runner.setTargets(-2000.0, -2000.0, 2000);
  runner.setTargets(-2000.0, -2000.0, 3000);
  runner.setTargets(500.0, 500.0, 4000);

  EXPECT_EQ(backend.stop_calls, 0);
  EXPECT_TRUE(left.dirForward());
  EXPECT_TRUE(right.dirForward());
  EXPECT_DOUBLE_EQ(runner.getFeedbackSample().left_slewed_sps, 500.0);
}

TEST_F(MotorRunnerTest, LeftAndRightDirectionsChangeIndependently) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  RecordingWaveBackend backend;
  MotorRunner runner(left, right, 400.0, 1e9, &backend);

  runner.setTargets(1000.0, 1000.0, 1000);
  runner.setTargets(-1000.0, 1000.0, 2000);
  runner.setTargets(-1000.0, 1000.0, 3000);
  runner.setTargets(-1000.0, 1000.0, 4000);

  EXPECT_FALSE(left.dirForward());
  EXPECT_TRUE(right.dirForward());
  const auto events = runner.getScheduledStepEvents(4000, 6500);
  ASSERT_EQ(events.size(), 2U);
  EXPECT_EQ(events[0].left_step_delta, -1);
  EXPECT_EQ(events[0].right_step_delta, 1);
  EXPECT_EQ(events[1].left_step_delta, -1);
  EXPECT_EQ(events[1].right_step_delta, 1);
}

TEST_F(MotorRunnerTest, QueuedFrameCancellationRestoresExecutedPulsePhase) {
  const auto run_case = [](double initial_sps, bool expect_first_frame_pulse) {
    Stepper left(1, Stepper::Pins{5, 6, 13});
    Stepper right(1, Stepper::Pins{7, 8, 14});
    RecordingWaveBackend backend;
    MotorRunner runner(left, right, 400.0, 1e9, &backend);

    runner.setTargets(initial_sps, initial_sps, 0);
    runner.setTargets(-200.0, -200.0, 1000);
    runner.setTargets(-200.0, -200.0, 2000);
    runner.setTargets(-200.0, -200.0, 3000);

    const auto events = runner.getScheduledStepEvents(3000, 5500);
    if (expect_first_frame_pulse) {
      ASSERT_EQ(events.size(), 1U);
      EXPECT_EQ(events.front().timestamp_us, 4250U);
      EXPECT_EQ(events.front().left_step_delta, -1);
      EXPECT_EQ(events.front().right_step_delta, -1);
    } else {
      EXPECT_TRUE(events.empty());
    }
  };

  // The executed prefix leaves 0.30 fractional pulses; retaining the phase
  // from all queued future frames would incorrectly emit a pulse here.
  run_case(100.0, false);
  // The executed prefix leaves 0.90 fractional pulses; resetting phase would
  // incorrectly suppress the first pulse after the reversal.
  run_case(300.0, true);
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
  EXPECT_GT(backend.stop_calls, 0);
}

TEST_F(MotorRunnerTest, ReversalBrakesToZeroBeforeChangingDirection) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  RecordingWaveBackend backend;
  MotorRunner runner(left, right, 400.0, 1e9, &backend);

  runner.setTargets(500.0, 500.0, 1000);
  ASSERT_EQ(backend.frames.size(), 2U);
  runner.setTargets(-500.0, -500.0, 2000);

  EXPECT_EQ(backend.stop_calls, 0);
  EXPECT_TRUE(left.dirForward());
  EXPECT_TRUE(right.dirForward());
  EXPECT_DOUBLE_EQ(runner.getFeedbackSample().left_slewed_sps, 0.0);
  EXPECT_EQ(runner.getActualLeftSteps(), 0);

  runner.setTargets(-500.0, -500.0, 3000);
  EXPECT_EQ(backend.stop_calls, 0);
  EXPECT_TRUE(left.dirForward());

  runner.setTargets(-500.0, -500.0, 4000);
  EXPECT_EQ(backend.stop_calls, 1);
  EXPECT_FALSE(left.dirForward());
  EXPECT_FALSE(right.dirForward());

  runner.setTargets(-500.0, -500.0, 10000);
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

  runner.setTargets(-12000.0, -12000.0, 6000);
  EXPECT_TRUE(left.dirForward());
  EXPECT_EQ(runner.getActualLeftSteps(), 60);

  runner.setTargets(-12000.0, -12000.0, 8500);
  EXPECT_FALSE(left.dirForward());
  EXPECT_FALSE(right.dirForward());
  EXPECT_EQ(runner.getActualLeftSteps(), 60);
  EXPECT_DOUBLE_EQ(runner.getScheduledStepPosition(8500).left_steps, 60.0);

  runner.setTargets(-12000.0, -12000.0, 11000);
  EXPECT_EQ(runner.getActualLeftSteps(), 30);
  EXPECT_EQ(runner.getActualRightSteps(), 30);
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
}

TEST_F(MotorRunnerTest, MotorServicePublishesPostSlewCommandsAndIndependentFlags) {
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
  MotorRunner runner(left, right, Config::control_hz, Config::motor_slew_sps_per_s, &backend);
  ipc::MessageBus bus(&sink, FeedbackSink::dispatch);
  sil::MotorService service(bus, &runner);

  service.on_message<MsgId::PhysicsTick>(PhysicsTickPayload{0.0025, 2500});
  service.on_message<MsgId::MotorTargets>(ipc::MotorTargetsPayload{2000.0, 100.0});

  ASSERT_EQ(sink.samples.size(), 1U);
  EXPECT_DOUBLE_EQ(sink.samples.back().left_slewed_sps, 500.0);
  EXPECT_DOUBLE_EQ(sink.samples.back().right_slewed_sps, 100.0);
  EXPECT_EQ(sink.samples.back().actuator_saturation_flags,
            ActuatorSaturationLeftSlew);
  EXPECT_EQ(sink.samples.back().actuator_fault, 0U);
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

TEST_F(MotorRunnerTest, FeedbackSnapshotIncludesActualSteps) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  MotorRunner runner(left, right, 400.0, 50000.0);

  RunFor(runner, 4000.0, 4000.0, std::chrono::milliseconds(50));

  const auto feedback = runner.getFeedbackSample();
  EXPECT_GT(feedback.left_actual_steps, 0);
  EXPECT_GT(feedback.right_actual_steps, 0);
}

TEST_F(MotorRunnerTest, FeedbackStepsMatchDirectionWithRightMotorInversion) {
  {
    Stepper left(1, Stepper::Pins{5, 6, 13});
    Stepper right(1, Stepper::Pins{7, 8, 14}, true);
    MotorRunner runner(left, right, 400.0, 50000.0);

    RunFor(runner, 800.0, 800.0, std::chrono::milliseconds(80));
    const auto feedback = runner.getFeedbackSample();
    EXPECT_GT(feedback.left_actual_steps, 0);
    EXPECT_GT(feedback.right_actual_steps, 0);
  }

  {
    Stepper left(1, Stepper::Pins{15, 16, 17});
    Stepper right(1, Stepper::Pins{18, 19, 20}, true);
    MotorRunner runner(left, right, 400.0, 50000.0);

    RunFor(runner, -800.0, -800.0, std::chrono::milliseconds(80));
    const auto feedback = runner.getFeedbackSample();
    EXPECT_LT(feedback.left_actual_steps, 0);
    EXPECT_LT(feedback.right_actual_steps, 0);
  }
}
