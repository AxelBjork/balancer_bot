#include "motor_runner.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>
#include <thread>

extern "C" void pigpio_stub_reset();

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
      runner.setTargets(spsL, spsR);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));  // Pump at ~500Hz
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

  // Run at 1000 sps for 25ms total
  RunFor(runner, 1000.0, 1000.0, std::chrono::milliseconds(25));

  // Expected: ~25 steps total (1000 sps * 0.025s)
  int64_t leftSteps = runner.getLeftSteps();
  int64_t rightSteps = runner.getRightSteps();

  EXPECT_NEAR(static_cast<double>(leftSteps), 25.0, 4.0) << "Left steps should accumulate to ~25";
  EXPECT_NEAR(static_cast<double>(rightSteps), 25.0, 4.0) << "Right steps should accumulate to ~25";
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
  RunFor(runner, 400.0, -400.0, std::chrono::milliseconds(50));
  v = runner.getActualSpeedSps();
  EXPECT_NEAR(v, 0.0, 50.0) << "Average velocity should be 0 for pure spin";
}

TEST_F(MotorRunnerTest, VelocityEstimationResolvesBelowOnePulsePerFrame) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  MotorRunner runner(left, right, 400.0, 50000.0);

  runner.getActualSpeedSps();
  RunFor(runner, 100.0, 100.0, std::chrono::milliseconds(100));

  const double v = runner.getActualSpeedSps();
  EXPECT_NEAR(v, 100.0, 35.0) << "Estimated velocity should average below the 200 sps quantization step";
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
