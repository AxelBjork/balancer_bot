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

  EXPECT_NEAR(leftSteps, 10, 3) << "Left steps should be ~10";
  EXPECT_NEAR(rightSteps, 10, 3) << "Right steps should be ~10";
}

TEST_F(MotorRunnerTest, StepTrackingReverseDirection) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  MotorRunner runner(left, right, 1000.0);

  RunFor(runner, -100.0, -100.0, std::chrono::milliseconds(100));

  int64_t leftSteps = runner.getLeftSteps();
  int64_t rightSteps = runner.getRightSteps();

  EXPECT_NEAR(leftSteps, -10, 3) << "Left steps should be ~-10";
  EXPECT_NEAR(rightSteps, -10, 3) << "Right steps should be ~-10";
}

TEST_F(MotorRunnerTest, StepTrackingDifferentialSteering) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  MotorRunner runner(left, right, 1000.0);

  RunFor(runner, 100.0, -100.0, std::chrono::milliseconds(100));

  int64_t leftSteps = runner.getLeftSteps();
  int64_t rightSteps = runner.getRightSteps();

  EXPECT_NEAR(leftSteps, 10, 3) << "Left steps should be ~10";
  EXPECT_NEAR(rightSteps, -10, 3) << "Right steps should be ~-10";
}

TEST_F(MotorRunnerTest, StepTrackingAccumulation) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  MotorRunner runner(left, right, 1000.0);

  // Run at 100 sps for 250ms total
  RunFor(runner, 100.0, 100.0, std::chrono::milliseconds(250));

  // Expected: ~25 steps total (100 sps * 0.25s)
  int64_t leftSteps = runner.getLeftSteps();
  int64_t rightSteps = runner.getRightSteps();

  EXPECT_NEAR(leftSteps, 25, 4) << "Left steps should accumulate to ~25";
  EXPECT_NEAR(rightSteps, 25, 4) << "Right steps should accumulate to ~25";
}

TEST_F(MotorRunnerTest, StepTrackingZeroRate) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  MotorRunner runner(left, right, 1000.0);

  RunFor(runner, 0.0, 0.0, std::chrono::milliseconds(100));

  int64_t leftSteps = runner.getLeftSteps();
  int64_t rightSteps = runner.getRightSteps();

  EXPECT_NEAR(leftSteps, 0, 1) << "Left steps should be 0";
  EXPECT_NEAR(rightSteps, 0, 1) << "Right steps should be 0";
}

TEST_F(MotorRunnerTest, VelocityEstimation) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  MotorRunner runner(left, right, 1000.0);

  // Init state (first call returns 0 and sets baseline)
  runner.getActualSpeedSps();

  // Move forward 1000 sps for 500ms
  // Note: RunFor pumps at ~500Hz (2ms sleep), so plenty of updates.
  RunFor(runner, 1000.0, 1000.0, std::chrono::milliseconds(500));

  float v = runner.getActualSpeedSps();
  // Allow loose tolerance due to simulation timing jitter
  EXPECT_NEAR(v, 1000.0f, 150.0f) << "Velocity should be approx 1000 sps";

  // Differential (spin)
  RunFor(runner, 1000.0, -1000.0, std::chrono::milliseconds(500));
  v = runner.getActualSpeedSps();
  EXPECT_NEAR(v, 0.0f, 50.0f) << "Average velocity should be 0 for pure spin";
}
