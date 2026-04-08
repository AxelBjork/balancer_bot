#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <vector>

#include "config.h"
#include "motor_runner.h"
#include "services/control_service.h"
#include "services/motor_service.h"
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

class ControlServiceHarness {
 public:
  ControlServiceHarness() : bus_(this, &ControlServiceHarness::dispatch), control_(bus_) {}

  void sendJoystick(float forward, float turn) {
    control_.on_message<MsgId::JoystickCommand>(ipc::JoystickCommandPayload{forward, turn});
  }

  void sendMotorFeedback(float left_applied_sps,
                         float right_applied_sps,
                         int64_t left_actual_steps,
                         int64_t right_actual_steps) {
    ipc::MotorFeedbackPayload payload{};
    payload.left_applied_sps = left_applied_sps;
    payload.right_applied_sps = right_applied_sps;
    payload.left_actual_steps = left_actual_steps;
    payload.right_actual_steps = right_actual_steps;
    control_.on_message<MsgId::MotorFeedback>(payload);
  }

  void step_with_imu(double dt_s, uint64_t sim_time_us, double angle_rad = 0.0, double gyro_rad_s = 0.0) {
    ipc::ImuSamplePayload imu{};
    imu.pitch_rad = angle_rad;
    imu.gyr = {0.0, gyro_rad_s, 0.0};
    imu.timestamp_us = sim_time_us;
    control_.on_message<MsgId::ImuData>(imu);

    PhysicsTickPayload tick{};
    tick.dt_s = dt_s;
    tick.sim_time_us = sim_time_us;
    control_.on_message<MsgId::PhysicsTick>(tick);
  }

  const std::vector<ipc::MotorTargetsPayload>& motor_targets() const { return motor_targets_; }
  const std::vector<ipc::SystemTelemetryPayload>& telemetry() const { return telemetry_; }

 private:
  static void dispatch(void* ctx, MsgId id, const void* payload) {
    auto* self = static_cast<ControlServiceHarness*>(ctx);
    if (id == MsgId::MotorTargets) {
      self->motor_targets_.push_back(*static_cast<const ipc::MotorTargetsPayload*>(payload));
    } else if (id == MsgId::SystemTelemetry) {
      self->telemetry_.push_back(*static_cast<const ipc::SystemTelemetryPayload*>(payload));
    }
  }

  ipc::MessageBus bus_;
  sil::ControlService control_;
  std::vector<ipc::MotorTargetsPayload> motor_targets_;
  std::vector<ipc::SystemTelemetryPayload> telemetry_;
};

class ServiceBusHarness {
 public:
  ServiceBusHarness()
      : left_(1, Stepper::Pins{5, 6, 13}),
        right_(1, Stepper::Pins{7, 8, 14}),
        runner_(left_, right_, 400.0, 100.0),
        bus_(this, &ServiceBusHarness::dispatch),
        control_(bus_),
        motor_(bus_, &runner_) {}

  void sendJoystick(float forward, float turn) {
    ipc::JoystickCommandPayload payload{};
    payload.forward = forward;
    payload.turn = turn;
    bus_.publish<MsgId::JoystickCommand>(payload);
  }

  void sendStep(double dt_s, uint64_t sim_time_us, double angle_rad = 0.0, double gyro_rad_s = 0.0) {
    ipc::ImuSamplePayload imu{};
    imu.pitch_rad = angle_rad;
    imu.gyr = {0.0, gyro_rad_s, 0.0};
    imu.timestamp_us = sim_time_us;
    bus_.publish<MsgId::ImuData>(imu);

    PhysicsTickPayload tick{};
    tick.dt_s = dt_s;
    tick.sim_time_us = sim_time_us;
    bus_.publish<MsgId::PhysicsTick>(tick);
  }

  const std::vector<ipc::MotorTargetsPayload>& motor_targets() const { return motor_targets_; }
  const std::vector<ipc::MotorFeedbackPayload>& feedback() const { return feedback_; }
  const std::vector<ipc::SystemTelemetryPayload>& telemetry() const { return telemetry_; }
  MotorRunner& runner() { return runner_; }

 private:
  static void dispatch(void* ctx, MsgId id, const void* payload) {
    auto* self = static_cast<ServiceBusHarness*>(ctx);
    if (id == MsgId::ImuData) {
      self->control_.on_message<MsgId::ImuData>(*static_cast<const ipc::ImuSamplePayload*>(payload));
    } else if (id == MsgId::PhysicsTick) {
      self->control_.on_message<MsgId::PhysicsTick>(*static_cast<const PhysicsTickPayload*>(payload));
    } else if (id == MsgId::JoystickCommand) {
      self->control_.on_message<MsgId::JoystickCommand>(
          *static_cast<const ipc::JoystickCommandPayload*>(payload));
    } else if (id == MsgId::MotorTargets) {
      const auto& p = *static_cast<const ipc::MotorTargetsPayload*>(payload);
      self->motor_targets_.push_back(p);
      self->motor_.on_message<MsgId::MotorTargets>(p);
    } else if (id == MsgId::MotorFeedback) {
      const auto& p = *static_cast<const ipc::MotorFeedbackPayload*>(payload);
      self->feedback_.push_back(p);
      self->control_.on_message<MsgId::MotorFeedback>(p);
    } else if (id == MsgId::SystemTelemetry) {
      self->telemetry_.push_back(*static_cast<const ipc::SystemTelemetryPayload*>(payload));
    }
  }

  Stepper left_;
  Stepper right_;
  MotorRunner runner_;
  ipc::MessageBus bus_;
  sil::ControlService control_;
  sil::MotorService motor_;
  std::vector<ipc::MotorTargetsPayload> motor_targets_;
  std::vector<ipc::MotorFeedbackPayload> feedback_;
  std::vector<ipc::SystemTelemetryPayload> telemetry_;
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

TEST(RateControllerCoreTest, SmallResidualVelocityIsIgnoredWithoutCommand) {
  RateControllerHarness h;
  h.setJoystick(0.0f, 0.0f);
  h.run_steps(40, 1.0 / 400.0);

  h.runner().setActualSpeedSps(50.0f);
  h.run_steps(80, 1.0 / 400.0);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_NEAR(h.telemetry().back().vel_error, 0.0, 1e-3);
  EXPECT_NEAR(h.telemetry().back().vel_p_term, 0.0, 1e-3);
  EXPECT_NEAR(h.telemetry().back().pitch_sp_deg, 0.0, 1e-3);
}

TEST(RateControllerCoreTest, LargeResidualVelocityIsBrakedWithoutCommand) {
  RateControllerHarness h;
  h.setJoystick(0.0f, 0.0f);
  h.run_steps(40, 1.0 / 400.0);

  h.runner().setActualSpeedSps(4000.0f);
  h.run_steps(80, 1.0 / 400.0);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_LT(h.telemetry().back().vel_error, 0.0);
  EXPECT_GT(h.telemetry().back().vel_p_term, 0.0);
  EXPECT_GT(std::abs(h.telemetry().back().pitch_sp_deg), 1e-3);
}

TEST(RateControllerCoreTest, VelocityFeedbackAffectsTelemetryWhenCommanded) {
  RateControllerHarness h;
  h.setJoystick(0.2f, 0.0f);
  h.run_steps(40, 1.0 / 400.0);

  constexpr float kMeasuredVelocitySps = 1234.0f;
  h.runner().setActualSpeedSps(kMeasuredVelocitySps);
  h.run_steps(80, 1.0 / 400.0);

  ASSERT_FALSE(h.telemetry().empty());
  const float target_velocity_sps = 0.2f * static_cast<float>(kMaxSps);
  const float used_velocity_sps = target_velocity_sps - static_cast<float>(h.telemetry().back().vel_error);
  EXPECT_NEAR(used_velocity_sps, kMeasuredVelocitySps, 20.0f);
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

TEST(ControlServiceTest, UsesFallbackVelocityProxyWhenNoMotorFeedbackExists) {
  ControlServiceHarness h;
  h.sendJoystick(0.2f, 0.0f);

  for (int i = 0; i < 200; ++i) {
    const uint64_t sim_time_us = static_cast<uint64_t>((i + 1) * 2500);
    h.step_with_imu(1.0 / 400.0, sim_time_us);
  }

  ASSERT_FALSE(h.telemetry().empty());
  const float target_velocity_sps = 0.2f * static_cast<float>(kMaxSps);
  const float used_velocity_sps = target_velocity_sps - h.telemetry().back().vel_error;
  EXPECT_GT(std::abs(used_velocity_sps), 1.0f);
}

TEST(ControlServiceTest, UsesMotorFeedbackForVelocityTelemetry) {
  ControlServiceHarness h;
  h.sendJoystick(0.2f, 0.0f);

  for (int i = 0; i < 200; ++i) {
    const uint64_t sim_time_us = static_cast<uint64_t>((i + 1) * 2500);
    h.sendMotorFeedback(123.0f, 123.0f, 0, 0);
    h.step_with_imu(1.0 / 400.0, sim_time_us);
  }

  ASSERT_FALSE(h.telemetry().empty());
  const float target_velocity_sps = 0.2f * static_cast<float>(kMaxSps);
  const float used_velocity_sps = target_velocity_sps - h.telemetry().back().vel_error;
  EXPECT_NEAR(used_velocity_sps, 123.0f, 5.0f);
}

TEST(ControlServiceTest, UsesMotorFeedbackPositionForPositionHold) {
  const double old_pos_p = ConfigPid::pos_P;
  struct RestorePosP {
    ~RestorePosP() { ConfigPid::pos_P = old_value; }
    double old_value;
  } restore{old_pos_p};
  ConfigPid::pos_P = 1000.0;

  ControlServiceHarness h;
  h.sendJoystick(0.0f, 0.0f);

  for (int i = 0; i < 20; ++i) {
    const uint64_t sim_time_us = static_cast<uint64_t>((i + 1) * 2500);
    h.sendMotorFeedback(0.0f, 0.0f, 0, 0);
    h.step_with_imu(1.0 / 400.0, sim_time_us);
  }

  const int64_t displacement_steps = 1000;
  for (int i = 20; i < 200; ++i) {
    const uint64_t sim_time_us = static_cast<uint64_t>((i + 1) * 2500);
    h.sendMotorFeedback(0.0f, 0.0f, displacement_steps, displacement_steps);
    h.step_with_imu(1.0 / 400.0, sim_time_us);
  }

  ASSERT_FALSE(h.telemetry().empty());
  const float expected_velocity_sps =
      -static_cast<float>(displacement_steps) * static_cast<float>(Config::meters_per_step) * 1000.0f;
  EXPECT_NEAR(h.telemetry().back().vel_error, expected_velocity_sps, 5.0f);
}

TEST(ServiceBusIntegrationTest, TickPublishesMotorTargetsAndImmediateMotorFeedback) {
  ServiceBusHarness h;
  h.sendJoystick(0.2f, 0.0f);

  h.sendStep(1.0 / 400.0, 2500);

  ASSERT_FALSE(h.motor_targets().empty());
  ASSERT_FALSE(h.feedback().empty());

  const auto expected = h.runner().getFeedbackSample();
  EXPECT_NEAR(h.feedback().back().left_applied_sps, expected.left_applied_sps, 1e-3f);
  EXPECT_NEAR(h.feedback().back().right_applied_sps, expected.right_applied_sps, 1e-3f);
  EXPECT_EQ(h.feedback().back().left_actual_steps, expected.left_actual_steps);
  EXPECT_EQ(h.feedback().back().right_actual_steps, expected.right_actual_steps);
}

}  // namespace
