#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <vector>

#include "config.h"
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
    core_.setPositionFeedback([this]() { return runner_.getAveragePositionSteps() * Config::meters_per_step; });
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

TEST(RateControllerCoreTest, VelocityFeedbackIsIgnoredWithoutCommand) {
  RateControllerHarness h;
  h.setJoystick(0.0f, 0.0f);
  h.run_steps(40, 1.0 / 400.0);

  h.runner().setActualSpeedSps(4000.0f);
  h.run_steps(80, 1.0 / 400.0);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_NEAR(h.telemetry().back().vel_error, 0.0, 1e-3);
  EXPECT_NEAR(h.telemetry().back().vel_p_term, 0.0, 1e-3);
  EXPECT_NEAR(h.telemetry().back().pitch_sp_deg, 0.0, 1e-3);
}

TEST(RateControllerCoreTest, VelocityFeedbackAffectsTelemetryWhenCommanded) {
  RateControllerHarness h;
  h.setJoystick(0.2f, 0.0f);
  h.run_steps(40, 1.0 / 400.0);

  h.runner().setActualSpeedSps(4000.0f);
  h.run_steps(80, 1.0 / 400.0);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_LT(h.telemetry().back().vel_error, -3000.0);
  EXPECT_NE(h.telemetry().back().vel_p_term, 0.0);
}

TEST(RateControllerCoreTest, PositionHoldAddsVelocityTargetBackTowardAnchor) {
  const double old_pos_p = ConfigPid::pos_P;
  struct RestorePosP {
    double& slot;
    double old_value;
    ~RestorePosP() { slot = old_value; }
  } restore{ConfigPid::pos_P, old_pos_p};
  ConfigPid::pos_P = 1000.0;

  RateControllerHarness h;
  h.setJoystick(0.0f, 0.0f);
  h.run_steps(10, 1.0 / 400.0);
  h.runner().setAveragePositionSteps(1000.0f);
  h.run_steps(80, 1.0 / 400.0);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_LT(h.telemetry().back().vel_error, 0.0);
  EXPECT_GT(h.telemetry().back().vel_p_term, 0.0);
  EXPECT_NE(h.runner().lastLeft(), 0.0f);
  EXPECT_NE(h.runner().lastRight(), 0.0f);
}

TEST(RateControllerCoreTest, AngleTrimAccumulatesAgainstPersistentPitchError) {
  const double old_angle_i = ConfigPid::angle_I;
  struct RestoreAngleI {
    double& slot;
    double old_value;
    ~RestoreAngleI() { slot = old_value; }
  } restore{ConfigPid::angle_I, old_angle_i};
  ConfigPid::angle_I = 0.1;

  RateControllerHarness h;
  h.setJoystick(0.0f, 0.0f);
  h.run_steps(200, 1.0 / 400.0, 0.02, 0.0);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_LT(h.telemetry().back().pitch_sp_deg, 0.0);
}

TEST(RateControllerCoreTest, LeanTrimAccumulatesCorrectiveBiasForPositiveVelocityDrift) {
  const double old_angle_i = ConfigPid::angle_I;
  const double old_lean_trim_i = ConfigPid::lean_trim_I;
  const double old_lean_trim_max_deg = ConfigPid::lean_trim_max_deg;
  const double old_lean_trim_decay_s = ConfigPid::lean_trim_decay_s;
  struct RestoreLeanTrimConfig {
    ~RestoreLeanTrimConfig() {
      ConfigPid::angle_I = old_angle_i;
      ConfigPid::lean_trim_I = old_lean_trim_i;
      ConfigPid::lean_trim_max_deg = old_lean_trim_max_deg;
      ConfigPid::lean_trim_decay_s = old_lean_trim_decay_s;
    }
    double old_angle_i;
    double old_lean_trim_i;
    double old_lean_trim_max_deg;
    double old_lean_trim_decay_s;
  } restore{old_angle_i, old_lean_trim_i, old_lean_trim_max_deg, old_lean_trim_decay_s};
  ConfigPid::angle_I = 0.0;
  ConfigPid::lean_trim_I = 0.12;
  ConfigPid::lean_trim_max_deg = 4.0;
  ConfigPid::lean_trim_decay_s = 3.0;

  RateControllerHarness h;
  h.setJoystick(0.0f, 0.0f);
  h.runner().setActualSpeedSps(800.0f);
  h.run_steps(400, 1.0 / 400.0, 0.0, 0.0);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_LT(h.telemetry().back().pitch_trim_deg, 0.0);
  EXPECT_GT(h.telemetry().back().trim_active, 0.5);
}

TEST(RateControllerCoreTest, LeanTrimAccumulatesOppositeBiasForNegativeVelocityDrift) {
  const double old_angle_i = ConfigPid::angle_I;
  const double old_lean_trim_i = ConfigPid::lean_trim_I;
  const double old_lean_trim_max_deg = ConfigPid::lean_trim_max_deg;
  const double old_lean_trim_decay_s = ConfigPid::lean_trim_decay_s;
  struct RestoreLeanTrimConfig {
    ~RestoreLeanTrimConfig() {
      ConfigPid::angle_I = old_angle_i;
      ConfigPid::lean_trim_I = old_lean_trim_i;
      ConfigPid::lean_trim_max_deg = old_lean_trim_max_deg;
      ConfigPid::lean_trim_decay_s = old_lean_trim_decay_s;
    }
    double old_angle_i;
    double old_lean_trim_i;
    double old_lean_trim_max_deg;
    double old_lean_trim_decay_s;
  } restore{old_angle_i, old_lean_trim_i, old_lean_trim_max_deg, old_lean_trim_decay_s};
  ConfigPid::angle_I = 0.0;
  ConfigPid::lean_trim_I = 0.12;
  ConfigPid::lean_trim_max_deg = 4.0;
  ConfigPid::lean_trim_decay_s = 3.0;

  RateControllerHarness h;
  h.setJoystick(0.0f, 0.0f);
  h.runner().setActualSpeedSps(-800.0f);
  h.run_steps(400, 1.0 / 400.0, 0.0, 0.0);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_GT(h.telemetry().back().pitch_trim_deg, 0.0);
  EXPECT_GT(h.telemetry().back().trim_active, 0.5);
}

TEST(RateControllerCoreTest, LeanTrimDecaysWhenOperatorCommandIsPresent) {
  const double old_angle_i = ConfigPid::angle_I;
  const double old_lean_trim_i = ConfigPid::lean_trim_I;
  const double old_lean_trim_max_deg = ConfigPid::lean_trim_max_deg;
  const double old_lean_trim_decay_s = ConfigPid::lean_trim_decay_s;
  struct RestoreLeanTrimConfig {
    ~RestoreLeanTrimConfig() {
      ConfigPid::angle_I = old_angle_i;
      ConfigPid::lean_trim_I = old_lean_trim_i;
      ConfigPid::lean_trim_max_deg = old_lean_trim_max_deg;
      ConfigPid::lean_trim_decay_s = old_lean_trim_decay_s;
    }
    double old_angle_i;
    double old_lean_trim_i;
    double old_lean_trim_max_deg;
    double old_lean_trim_decay_s;
  } restore{old_angle_i, old_lean_trim_i, old_lean_trim_max_deg, old_lean_trim_decay_s};
  ConfigPid::angle_I = 0.0;
  ConfigPid::lean_trim_I = 0.12;
  ConfigPid::lean_trim_max_deg = 4.0;
  ConfigPid::lean_trim_decay_s = 0.5;

  RateControllerHarness h;
  h.setJoystick(0.0f, 0.0f);
  h.runner().setActualSpeedSps(800.0f);
  h.run_steps(400, 1.0 / 400.0, 0.0, 0.0);
  const double accumulated_trim_deg = std::abs(h.telemetry().back().pitch_trim_deg);

  h.setJoystick(0.2f, 0.0f);
  h.run_steps(400, 1.0 / 400.0, 0.0, 0.0);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_LT(std::abs(h.telemetry().back().pitch_trim_deg), accumulated_trim_deg);
  EXPECT_LT(h.telemetry().back().trim_active, 0.5);
}

TEST(RateControllerCoreTest, LeanTrimHardResetsOnLargeTilt) {
  const double old_angle_i = ConfigPid::angle_I;
  const double old_lean_trim_i = ConfigPid::lean_trim_I;
  const double old_lean_trim_max_deg = ConfigPid::lean_trim_max_deg;
  const double old_lean_trim_decay_s = ConfigPid::lean_trim_decay_s;
  struct RestoreLeanTrimConfig {
    ~RestoreLeanTrimConfig() {
      ConfigPid::angle_I = old_angle_i;
      ConfigPid::lean_trim_I = old_lean_trim_i;
      ConfigPid::lean_trim_max_deg = old_lean_trim_max_deg;
      ConfigPid::lean_trim_decay_s = old_lean_trim_decay_s;
    }
    double old_angle_i;
    double old_lean_trim_i;
    double old_lean_trim_max_deg;
    double old_lean_trim_decay_s;
  } restore{old_angle_i, old_lean_trim_i, old_lean_trim_max_deg, old_lean_trim_decay_s};
  ConfigPid::angle_I = 0.0;
  ConfigPid::lean_trim_I = 0.12;
  ConfigPid::lean_trim_max_deg = 4.0;
  ConfigPid::lean_trim_decay_s = 3.0;

  RateControllerHarness h;
  h.setJoystick(0.0f, 0.0f);
  h.runner().setActualSpeedSps(800.0f);
  h.run_steps(400, 1.0 / 400.0, 0.0, 0.0);
  ASSERT_GT(std::abs(h.telemetry().back().pitch_trim_deg), 1e-3);

  h.run_steps(8, 1.0 / 400.0, 25.0 * M_PI / 180.0, 0.0);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_NEAR(h.telemetry().back().pitch_trim_deg, 0.0, 1e-6);
  EXPECT_LT(h.telemetry().back().trim_active, 0.5);
}

}  // namespace
