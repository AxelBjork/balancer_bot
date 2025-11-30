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
};

TEST_F(MotorRunnerTest, StepTrackingForwardConstantRate) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  MotorRunner runner(left, right, 1000.0);

  // Command 100 sps forward for both motors
  runner.setTargets(100.0, 100.0);

  // Wait for 100ms
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Update again to trigger step integration
  runner.setTargets(100.0, 100.0);

  // Expected: ~10 steps (100 sps * 0.1s)
  int64_t leftSteps = runner.getLeftSteps();
  int64_t rightSteps = runner.getRightSteps();

  EXPECT_NEAR(leftSteps, 10, 2) << "Left steps should be ~10";
  EXPECT_NEAR(rightSteps, 10, 2) << "Right steps should be ~10";
}

TEST_F(MotorRunnerTest, StepTrackingReverseDirection) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  MotorRunner runner(left, right, 1000.0);

  // Command -100 sps (reverse) for both motors
  runner.setTargets(-100.0, -100.0);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  runner.setTargets(-100.0, -100.0);

  // Expected: ~-10 steps
  int64_t leftSteps = runner.getLeftSteps();
  int64_t rightSteps = runner.getRightSteps();

  EXPECT_NEAR(leftSteps, -10, 2) << "Left steps should be ~-10";
  EXPECT_NEAR(rightSteps, -10, 2) << "Right steps should be ~-10";
}

TEST_F(MotorRunnerTest, StepTrackingDifferentialSteering) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  MotorRunner runner(left, right, 1000.0);

  // Command differential: left forward, right reverse
  runner.setTargets(100.0, -100.0);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  runner.setTargets(100.0, -100.0);

  int64_t leftSteps = runner.getLeftSteps();
  int64_t rightSteps = runner.getRightSteps();

  EXPECT_NEAR(leftSteps, 10, 2) << "Left steps should be ~10";
  EXPECT_NEAR(rightSteps, -10, 2) << "Right steps should be ~-10";
}

TEST_F(MotorRunnerTest, StepTrackingAccumulation) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  MotorRunner runner(left, right, 1000.0);

  // Run at 100 sps for multiple intervals
  runner.setTargets(100.0, 100.0);

  for (int i = 0; i < 5; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    runner.setTargets(100.0, 100.0);
  }

  // Expected: ~25 steps total (100 sps * 0.25s)
  int64_t leftSteps = runner.getLeftSteps();
  int64_t rightSteps = runner.getRightSteps();

  EXPECT_NEAR(leftSteps, 25, 5) << "Left steps should accumulate to ~25";
  EXPECT_NEAR(rightSteps, 25, 5) << "Right steps should accumulate to ~25";
}

TEST_F(MotorRunnerTest, StepTrackingZeroRate) {
  Stepper left(1, Stepper::Pins{5, 6, 13});
  Stepper right(1, Stepper::Pins{7, 8, 14});
  MotorRunner runner(left, right, 1000.0);

  // Set to zero
  runner.setTargets(0.0, 0.0);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  runner.setTargets(0.0, 0.0);

  int64_t leftSteps = runner.getLeftSteps();
  int64_t rightSteps = runner.getRightSteps();

  EXPECT_EQ(leftSteps, 0) << "Left steps should be 0";
  EXPECT_EQ(rightSteps, 0) << "Right steps should be 0";
}
