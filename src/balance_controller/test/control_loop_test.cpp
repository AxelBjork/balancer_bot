#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <vector>

#include "services/control/rate_controller_core.h"

namespace {

class FakeMotorRunner {
 public:
  void setTargets(float left_sps, float right_sps) {
    last_left_ = left_sps;
    last_right_ = right_sps;
    ++calls_;
  }

  float getActualSpeedSps() const {
    return actual_speed_sps_;
  }

  void setActualSpeedSps(float v) {
    actual_speed_sps_ = v;
  }

  float getAveragePositionSteps() const {
    return position_steps_;
  }

  void setAveragePositionSteps(float v) {
    position_steps_ = v;
  }

  float lastLeft() const { return last_left_; }
  float lastRight() const { return last_right_; }
  int callCount() const { return calls_; }

 private:
  float last_left_{0.0f};
  float last_right_{0.0f};
  float actual_speed_sps_{0.0f};
  float position_steps_{0.0f};
  int calls_{0};
};

class RateControllerHarness {
 public:
  RateControllerHarness() {
    core_.setMotorOutputs([this](float left, float right) { runner_.setTargets(left, right); });
    core_.setVelocityFeedback([this]() { return runner_.getActualSpeedSps(); });
    core_.setPositionFeedback([this]() { return runner_.getAveragePositionSteps(); });
    core_.setTelemetrySink([this](const Telemetry& t) { telemetry_.push_back(t); });
  }

  void setJoystick(float forward, float turn) {
    core_.setJoystick(JoyCmd{forward, turn});
  }

  void setImu(double angle_rad, double gyro_rad_s, uint64_t sim_time_us) {
    ImuSample s{};
    s.angle_rad = angle_rad;
    s.gyro_rad_s = gyro_rad_s;
    s.t = std::chrono::steady_clock::time_point(std::chrono::microseconds(sim_time_us));
    core_.pushImu(s);
  }

  void tick(double dt_s, uint64_t sim_time_us) {
    const auto now = std::chrono::steady_clock::time_point(std::chrono::microseconds(sim_time_us));
    core_.step(dt_s, now);
  }

  void run_steps(int count, double dt_s, double angle_rad = 0.0, double gyro_rad_s = 0.0) {
    for (int i = 0; i < count; ++i) {
      const uint64_t sim_time_us = static_cast<uint64_t>((i + 1) * dt_s * 1e6);
      setImu(angle_rad, gyro_rad_s, sim_time_us);
      tick(dt_s, sim_time_us);
    }
  }

  const std::vector<Telemetry>& telemetry() const { return telemetry_; }
  const FakeMotorRunner& runner() const { return runner_; }
  FakeMotorRunner& runner() { return runner_; }

 private:
  FakeMotorRunner runner_;
  RateControllerCore core_;
  std::vector<Telemetry> telemetry_;
};

TEST(RateControllerCoreTest, ZeroInputsStayNearZero) {
  RateControllerHarness h;
  h.setJoystick(0.0f, 0.0f);
  h.run_steps(120, 1.0 / 400.0);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_NEAR(h.runner().lastLeft(), 0.0f, 1e-3);
  EXPECT_NEAR(h.runner().lastRight(), 0.0f, 1e-3);
}

TEST(RateControllerCoreTest, SteeringSplitsWheelCommands) {
  RateControllerHarness h;
  h.setJoystick(0.0f, 0.6f);
  h.run_steps(120, 1.0 / 400.0);

  EXPECT_LT(h.runner().lastRight(), h.runner().lastLeft());
  EXPECT_NE(h.runner().lastLeft(), h.runner().lastRight());
}

TEST(RateControllerCoreTest, VelocityFeedbackAffectsTelemetry) {
  RateControllerHarness h;
  h.setJoystick(0.0f, 0.0f);
  h.run_steps(40, 1.0 / 400.0);

  h.runner().setActualSpeedSps(4000.0f);
  h.run_steps(80, 1.0 / 400.0);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_NEAR(h.telemetry().back().vel_error, -4000.0, 50.0);
  EXPECT_NE(h.telemetry().back().vel_p_term, 0.0);
}

TEST(RateControllerCoreTest, PositionHoldBiasesVelocityTargetBackTowardAnchor) {
  const double old_pos_p = ConfigPid::pos_P;
  struct RestorePosP {
    double& slot;
    double old_value;
    ~RestorePosP() { slot = old_value; }
  } restore{ConfigPid::pos_P, old_pos_p};
  ConfigPid::pos_P = 2.0;

  RateControllerHarness h;
  h.setJoystick(0.0f, 0.0f);
  h.run_steps(10, 1.0 / 400.0);
  h.runner().setAveragePositionSteps(0.1f);
  h.run_steps(80, 1.0 / 400.0);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_GT(h.telemetry().back().pitch_sp_deg, 0.0);
  EXPECT_GT(h.runner().lastLeft(), 0.0f);
  EXPECT_GT(h.runner().lastRight(), 0.0f);
}

}  // namespace
