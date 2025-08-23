#include <gtest/gtest.h>
#include <gmock/gmock.h>

// Include your real declarations for Stepper and MotorRunner:
#include "stepper.h"

// ==== Test-only globals the production code expects ====
std::atomic<bool> g_stop{false};

// Provide a test-time Config with a modest control rate (20 ms slice).
// If your production already defines Config::control_hz, remove this.
struct Config {
  static constexpr int control_hz = 50; // 50 Hz -> slice_sec_ = 0.02 s
};

// ==== Stub pigpio daemon API so tests don't hit hardware ====
extern "C" {
int set_mode(int /*pi*/, unsigned /*gpio*/, unsigned /*mode*/) { return 0; }
int gpio_write(int /*pi*/, unsigned /*gpio*/, unsigned /*level*/) { return 0; }
void time_sleep(double /*seconds*/) { /* no-op to keep tests fast */ }
} // extern "C"

// ==== Mock Stepper ====
class MockStepper : public Stepper {
public:
  using Stepper::Stepper;
  MOCK_METHOD(void, stepN, (unsigned steps, unsigned periodUs, bool dirForward), (const, override));
};

// ---- Helpers ----
static Stepper::Pins makePins() {
  return Stepper::Pins{ .ena = 5u, .step = 6u, .dir = 13u };
}

// ==== Tests ====

using ::testing::_;
using ::testing::InSequence;
using ::testing::Invoke;
using ::testing::StrictMock;

TEST(MotorRunnerTest, PositiveTargetCallsStepNWithExpectedArgs) {
  g_stop.store(false);
  StrictMock<MockStepper> m(0 /*pi*/, makePins());

  // control_hz = 50 -> slice = 0.02 s
  // target = +500 sps => want = 500 * 0.02 = 10 steps per slice
  // period_us = round(1e6 / 500) = 2000 us, dirForward = true
  std::promise<void> called;
  EXPECT_CALL(m, stepN(10u, 2000u, true))
    .WillOnce(Invoke([&](unsigned, unsigned, bool) {
      called.set_value();
    }));

  MotorRunner runner(m);
  runner.setTarget(500.0);

  ASSERT_EQ(called.get_future().wait_for(std::chrono::milliseconds(250)), std::future_status::ready);
  runner.stop();
}

TEST(MotorRunnerTest, NegativeTargetSetsReverseAndExpectedArgs) {
  g_stop.store(false);
  StrictMock<MockStepper> m(0 /*pi*/, makePins());

  // target = -1000 sps => sps=1000, want=1000*0.02=20, period=1000 us, dir=false
  std::promise<void> called;
  EXPECT_CALL(m, stepN(20u, 1000u, false))
    .WillOnce(Invoke([&](unsigned, unsigned, bool) {
      called.set_value();
    }));

  MotorRunner runner(m);
  runner.setTarget(-1000.0);

  ASSERT_EQ(called.get_future().wait_for(std::chrono::milliseconds(250)), std::future_status::ready);
  runner.stop();
}

TEST(MotorRunnerTest, ResidualAccumulationYieldsOneStepPerSliceFor55Sps) {
  g_stop.store(false);
  StrictMock<MockStepper> m(0 /*pi*/, makePins());

  // target = +55 sps => want = 55 * 0.02 = 1.1
  // First slice: n=1, residual=0.1
  // Second slice: want=1.1+0.1=1.2 => n=1 again
  // period_us = round(1e6 / 55) = 18182
  std::promise<void> done;

  {
    InSequence seq;
    EXPECT_CALL(m, stepN(1u, 18182u, true))
      .WillOnce(Invoke([&](unsigned, unsigned, bool) {
        // Let the loop continue to next slice
      }));
    EXPECT_CALL(m, stepN(1u, 18182u, true))
      .WillOnce(Invoke([&](unsigned, unsigned, bool) {
        done.set_value(); // after the 2nd call we consider the test satisfied
      }));
  }

  MotorRunner runner(m);
  runner.setTarget(55.0);

  ASSERT_EQ(done.get_future().wait_for(std::chrono::milliseconds(400)), std::future_status::ready);
  runner.stop();
}

TEST(MotorRunnerTest, ZeroOrTinyTargetSleepsAndDoesNotCallStepN) {
  g_stop.store(false);
  StrictMock<MockStepper> m(0 /*pi*/, makePins());

  // For |sps| < 1e-3 the loop should just sleep; expect no calls within a short window.
  EXPECT_CALL(m, stepN(_, _, _)).Times(0);

  MotorRunner runner(m);
  runner.setTarget(5e-4); // 0.0005 sps -> below threshold

  // Give it a few slices to run
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  runner.stop();
}
