#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "stepper.h"
#include "motor_runner.h"
#include <future>


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

using ::testing::_;
using ::testing::AtLeast;
using ::testing::InSequence;
using ::testing::Invoke;
using ::testing::NiceMock;

TEST(MotorRunnerTest, PositiveTargetCallsStepNWithExpectedArgs) {
  g_stop.store(false);
  NiceMock<MockStepper> m(0 /*pi*/, makePins());

  // control_hz = 100 -> slice = 0.01 s
  // target = +500 sps => want = 500 * 0.01 = 5 steps per slice
  // period_us = round(1e6 / 500) = 2000 us, dirForward = true
  std::promise<void> called;
  EXPECT_CALL(m, stepN(5u, 2000u, true))
      .Times(AtLeast(1))
      .WillOnce(Invoke([&](unsigned, unsigned, bool) { called.set_value(); }))
      .WillRepeatedly(Invoke([](unsigned, unsigned, bool) {})); // ignore extras

  MotorRunner runner(m);
  runner.setTarget(500.0);

  ASSERT_EQ(called.get_future().wait_for(std::chrono::milliseconds(250)),
            std::future_status::ready);
  runner.stop();
}

TEST(MotorRunnerTest, NegativeTargetSetsReverseAndExpectedArgs) {
  g_stop.store(false);
  NiceMock<MockStepper> m(0 /*pi*/, makePins());

  // target = -1000 sps => sps=1000, want = 1000 * 0.01 = 10
  // period = 1000 us, dir=false
  std::promise<void> called;
  EXPECT_CALL(m, stepN(10u, 1000u, false))
      .Times(AtLeast(1))
      .WillOnce(Invoke([&](unsigned, unsigned, bool) { called.set_value(); }))
      .WillRepeatedly(Invoke([](unsigned, unsigned, bool) {}));

  MotorRunner runner(m);
  runner.setTarget(-1000.0);

  ASSERT_EQ(called.get_future().wait_for(std::chrono::milliseconds(250)),
            std::future_status::ready);
  runner.stop();
}

TEST(MotorRunnerTest, ResidualAccumulationYieldsOneStepPerSliceFor55Sps) {
  g_stop.store(false);
  NiceMock<MockStepper> m(0 /*pi*/, makePins());

  // control_hz = 100 -> slice = 0.01 s
  // target = 55 sps => want per slice = 0.55
  // Calls pattern over slices: 0,1,0,1,... (period_us = round(1e6/55)=18182)
  std::promise<void> done;

  {
    InSequence seq;
    EXPECT_CALL(m, stepN(1u, 18182u, true))
        .WillOnce(Invoke([&](unsigned, unsigned, bool) {
          // let loop continue
        }));
    EXPECT_CALL(m, stepN(1u, 18182u, true))
        .WillOnce(Invoke([&](unsigned, unsigned, bool) { done.set_value(); }));
  }

  MotorRunner runner(m);
  runner.setTarget(55.0);

  ASSERT_EQ(done.get_future().wait_for(std::chrono::milliseconds(400)),
            std::future_status::ready);
  runner.stop();
}

TEST(MotorRunnerTest, ZeroOrTinyTargetSleepsAndDoesNotCallStepN) {
  g_stop.store(false);
  NiceMock<MockStepper> m(0 /*pi*/, makePins());

  EXPECT_CALL(m, stepN(_, _, _)).Times(0);

  MotorRunner runner(m);
  runner.setTarget(5e-4); // below threshold

  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  runner.stop();
}

TEST(MotorRunnerTest, RetargetMidRunAdjustsStepsAndPeriod) {
  g_stop.store(false);
  NiceMock<MockStepper> m(0, makePins());

  // Start at 200 sps -> 2 steps/slice (period 5000 us),
  // then jump to 750 sps -> steps alternate around 7.5 (7 or 8), period 1333 us.
  std::promise<void> got_new_speed;

  // Let the low-speed phase run for a few slices.
  EXPECT_CALL(m, stepN(2u, 5000u, true))
      .Times(AtLeast(1))
      .WillRepeatedly(Invoke([](unsigned, unsigned, bool) {}));

  // After retarget, accept either 7 or 8 steps (residual-dependent), period fixed at 1333 us.
  EXPECT_CALL(m, stepN(::testing::AnyOf(7u, 8u), 1333u, true))
      .Times(AtLeast(1))
      .WillOnce(Invoke([&](unsigned, unsigned, bool) { got_new_speed.set_value(); }))
      .WillRepeatedly(Invoke([](unsigned, unsigned, bool) {}));

  MotorRunner runner(m);
  runner.setTarget(200.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(30)); // give the first target time to take effect
  runner.setTarget(750.0);

  ASSERT_EQ(got_new_speed.get_future().wait_for(std::chrono::milliseconds(400)),
            std::future_status::ready);
  runner.stop();
}

TEST(MotorRunnerTest, DirectionFlipFromForwardToReverse) {
  g_stop.store(false);
  NiceMock<MockStepper> m(0, makePins());

  // +300 sps -> 3 steps/slice (period 3333 us), then -300 sps -> same steps, dir=false.
  std::promise<void> saw_reverse;

  EXPECT_CALL(m, stepN(3u, 3333u, true))
      .Times(AtLeast(1))
      .WillRepeatedly(Invoke([](unsigned, unsigned, bool) {}));

  EXPECT_CALL(m, stepN(3u, 3333u, false))
      .Times(AtLeast(1))
      .WillOnce(Invoke([&](unsigned, unsigned, bool) { saw_reverse.set_value(); }))
      .WillRepeatedly(Invoke([](unsigned, unsigned, bool) {}));

  MotorRunner runner(m);
  runner.setTarget(300.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  runner.setTarget(-300.0);

  ASSERT_EQ(saw_reverse.get_future().wait_for(std::chrono::milliseconds(400)),
            std::future_status::ready);
  runner.stop();
}
