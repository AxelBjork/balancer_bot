#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <vector>

#include "config.h"
#include "motor_runner.h"
#include "services/control_service.h"
#include "services/imu_service.h"
#include "services/motor_service.h"
#include "services/control/rate_controller_core.h"

namespace {

std::array<double, 3> accel_for_pitch(double angle_rad) {
  return {
      -9.81 * std::sin(angle_rad),
      0.0,
      9.81 * std::cos(angle_rad),
  };
}

class FakeMotorRunner {
 public:
  void setTargets(double left_sps, double right_sps) {
    last_left_ = left_sps;
    last_right_ = right_sps;
    ++calls_;
  }

  double getActualSpeedSps() const {
    return actual_speed_sps_;
  }

  void setActualSpeedSps(double v) {
    actual_speed_sps_ = v;
  }

  double getAveragePositionSteps() const {
    return position_steps_;
  }

  void setAveragePositionSteps(double v) {
    position_steps_ = v;
  }

  double lastLeft() const { return last_left_; }
  double lastRight() const { return last_right_; }
  int callCount() const { return calls_; }

 private:
  double last_left_{0.0};
  double last_right_{0.0};
  double actual_speed_sps_{0.0};
  double position_steps_{0.0};
  int calls_{0};
};

class RateControllerHarness {
 public:
  RateControllerHarness() {
    core_.setMotorOutputs([this](double left, double right) { runner_.setTargets(left, right); });
    core_.setVelocityFeedback([this]() { return runner_.getActualSpeedSps(); });
    core_.setPositionFeedback([this]() { return runner_.getAveragePositionSteps() * Config::meters_per_step; });
    core_.setTelemetrySink([this](const Telemetry& t) { telemetry_.push_back(t); });
  }

  void setJoystick(double forward, double turn) {
    core_.setJoystick(JoyCmd{static_cast<float>(forward), static_cast<float>(turn)});
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

  void sendJoystick(double forward, double turn) {
    control_.on_message<MsgId::JoystickCommand>(ipc::JoystickCommandPayload{static_cast<float>(forward), static_cast<float>(turn)});
  }

  void sendMotorFeedback(double left_applied_sps,
                         double right_applied_sps,
                         double measured_avg_sps,
                         int64_t left_actual_steps,
                         int64_t right_actual_steps) {
    ipc::MotorFeedbackPayload payload{};
    payload.left_applied_sps = left_applied_sps;
    payload.right_applied_sps = right_applied_sps;
    payload.measured_avg_sps = measured_avg_sps;
    payload.left_actual_steps = left_actual_steps;
    payload.right_actual_steps = right_actual_steps;
    control_.on_message<MsgId::MotorFeedback>(payload);
  }

  void step_with_imu(double dt_s,
                     uint64_t sim_time_us,
                     double angle_rad = 0.0,
                     double filtered_pitch_rate_rad_s = 0.0,
                     double raw_pitch_rate_rad_s = 0.0) {
    ipc::ImuSamplePayload imu{};
    imu.pitch_rad = angle_rad;
    imu.filtered_pitch_rate_rad_s = filtered_pitch_rate_rad_s;
    imu.acc = accel_for_pitch(angle_rad);
    if (raw_pitch_rate_rad_s == 0.0) {
      raw_pitch_rate_rad_s = filtered_pitch_rate_rad_s;
    }
    imu.gyr = {0.0, raw_pitch_rate_rad_s, 0.0};
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
      self->motor_targets_.push_back(unpack_payload<MsgId::MotorTargets>(payload));
    } else if (id == MsgId::SystemTelemetry) {
      self->telemetry_.push_back(unpack_payload<MsgId::SystemTelemetry>(payload));
    }
  }

  ipc::MessageBus bus_;
  sil::ControlService control_;
  std::vector<ipc::MotorTargetsPayload> motor_targets_;
  std::vector<ipc::SystemTelemetryPayload> telemetry_;
};

class ImuServiceHarness {
 public:
  ImuServiceHarness() : bus_(this, &ImuServiceHarness::dispatch), imu_(bus_, false) {}

  void publish_raw(const ipc::ImuRawPayload& payload) {
    bus_.publish<MsgId::ImuRawData>(payload);
  }

  const std::vector<ipc::ImuSamplePayload>& fused_samples() const { return fused_samples_; }

 private:
  static void dispatch(void* ctx, MsgId id, const void* payload) {
    auto* self = static_cast<ImuServiceHarness*>(ctx);
    if (id == MsgId::ImuData) {
      self->fused_samples_.push_back(unpack_payload<MsgId::ImuData>(payload));
    }
    ipc::dispatch_to_service(self->imu_, id, payload);
  }

  ipc::MessageBus bus_;
  sil::ImuService imu_;
  std::vector<ipc::ImuSamplePayload> fused_samples_;
};

class ServiceBusHarness {
 public:
  ServiceBusHarness()
      : left_(1, Stepper::Pins{5, 6, 13}),
        right_(1, Stepper::Pins{7, 8, 14}),
        runner_(left_, right_, 400.0, 100.0),
        bus_(this, &ServiceBusHarness::dispatch),
        control_(bus_),
        imu_(bus_, false),
        motor_(bus_, &runner_) {}

  void sendJoystick(double forward, double turn) {
    ipc::JoystickCommandPayload payload{};
    payload.forward = static_cast<float>(forward);
    payload.turn = static_cast<float>(turn);
    bus_.publish<MsgId::JoystickCommand>(payload);
  }

  void sendStep(double dt_s, uint64_t sim_time_us, double angle_rad = 0.0, double gyro_rad_s = 0.0) {
    ipc::ImuSamplePayload imu{};
    imu.pitch_rad = angle_rad;
    imu.filtered_pitch_rate_rad_s = gyro_rad_s;
    imu.acc = accel_for_pitch(angle_rad);
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
    
    // Explicitly handle vector collection for tests
    if (id == MsgId::MotorTargets) {
      self->motor_targets_.push_back(unpack_payload<MsgId::MotorTargets>(payload));
    } else if (id == MsgId::MotorFeedback) {
      self->feedback_.push_back(unpack_payload<MsgId::MotorFeedback>(payload));
    } else if (id == MsgId::SystemTelemetry) {
      self->telemetry_.push_back(unpack_payload<MsgId::SystemTelemetry>(payload));
    }

    ipc::dispatch_to_services(id, payload, self->imu_, self->control_, self->motor_);
  }

  Stepper left_;
  Stepper right_;
  MotorRunner runner_;
  ipc::MessageBus bus_;
  sil::ControlService control_;
  sil::ImuService imu_;
  sil::MotorService motor_;
  std::vector<ipc::MotorTargetsPayload> motor_targets_;
  std::vector<ipc::MotorFeedbackPayload> feedback_;
  std::vector<ipc::SystemTelemetryPayload> telemetry_;
};

struct ConfigPidSnapshot {
  double rate_P = ConfigPid::rate_P;
  double rate_I = ConfigPid::rate_I;
  double rate_D = ConfigPid::rate_D;
  double rate_I_lim = ConfigPid::rate_I_lim;
  double rate_FF = ConfigPid::rate_FF;
  double angle_to_rate_k = ConfigPid::angle_to_rate_k;
  double vel_P = ConfigPid::vel_P;
  double vel_I = ConfigPid::vel_I;
  double vel_D = ConfigPid::vel_D;
  double vel_I_lim = ConfigPid::vel_I_lim;
  double pos_P = ConfigPid::pos_P;
  double outer_k_pos = ConfigPid::outer_k_pos;
  double outer_k_vel = ConfigPid::outer_k_vel;
  double outer_k_pitch = ConfigPid::outer_k_pitch;
  double outer_k_pitch_rate = ConfigPid::outer_k_pitch_rate;
  double angle_I = ConfigPid::angle_I;
  double lean_trim_I = ConfigPid::lean_trim_I;
  double lean_trim_max_deg = ConfigPid::lean_trim_max_deg;
  double lean_trim_decay_s = ConfigPid::lean_trim_decay_s;

  void restore() const {
    ConfigPid::rate_P = rate_P;
    ConfigPid::rate_I = rate_I;
    ConfigPid::rate_D = rate_D;
    ConfigPid::rate_I_lim = rate_I_lim;
    ConfigPid::rate_FF = rate_FF;
    ConfigPid::angle_to_rate_k = angle_to_rate_k;
    ConfigPid::vel_P = vel_P;
    ConfigPid::vel_I = vel_I;
    ConfigPid::vel_D = vel_D;
    ConfigPid::vel_I_lim = vel_I_lim;
    ConfigPid::pos_P = pos_P;
    ConfigPid::outer_k_pos = outer_k_pos;
    ConfigPid::outer_k_vel = outer_k_vel;
    ConfigPid::outer_k_pitch = outer_k_pitch;
    ConfigPid::outer_k_pitch_rate = outer_k_pitch_rate;
    ConfigPid::angle_I = angle_I;
    ConfigPid::lean_trim_I = lean_trim_I;
    ConfigPid::lean_trim_max_deg = lean_trim_max_deg;
    ConfigPid::lean_trim_decay_s = lean_trim_decay_s;
  }
};

struct ScopedConfigPidRestore {
  ConfigPidSnapshot snapshot{};
  ~ScopedConfigPidRestore() { snapshot.restore(); }
};

std::filesystem::path write_temp_pid_conf(const std::string& body) {
  const auto path =
      std::filesystem::temp_directory_path() /
      ("balancer_pid_test_" + std::to_string(::getpid()) + "_" +
       std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".conf");
  std::ofstream out(path);
  out << body;
  out.close();
  return path;
}

TEST(RateControllerCoreTest, ZeroInputsStayNearZero) {
  RateControllerHarness h;
  h.setJoystick(0.0, 0.0);
  h.run_steps(120, 1.0 / 400.0);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_NEAR(h.runner().lastLeft(), 0.0, 1e-3);
  EXPECT_NEAR(h.runner().lastRight(), 0.0, 1e-3);
}

TEST(RateControllerCoreTest, HardwareScalingConstantsMatchValidatedHardwareSetup) {
  EXPECT_DOUBLE_EQ(kMaxSps, 16000.0);
  EXPECT_DOUBLE_EQ(kPitchOutToSps, 3200.0);
}

TEST(ConfigPidTest, LegacyOuterLoopKeysMapIntoRewrittenOuterLawWhenNeeded) {
  ScopedConfigPidRestore restore;
  ConfigPid::outer_k_pos = 111.0;
  ConfigPid::outer_k_vel = 222.0;
  ConfigPid::outer_k_pitch = 333.0;
  ConfigPid::outer_k_pitch_rate = 444.0;

  const auto path = write_temp_pid_conf(
      "angle_to_rate_k = 7\n"
      "vel_P = -0.0015\n"
      "vel_I = -0.2\n"
      "vel_D = 0.3\n"
      "vel_I_lim = 0.4\n"
      "pos_P = 2.5\n");

  ConfigPid::load(path.string());
  std::filesystem::remove(path);

  EXPECT_DOUBLE_EQ(ConfigPid::outer_k_pitch, 7.0);
  EXPECT_DOUBLE_EQ(ConfigPid::outer_k_vel, 0.0015);
  EXPECT_DOUBLE_EQ(ConfigPid::outer_k_pos, 0.00375);
  EXPECT_DOUBLE_EQ(ConfigPid::outer_k_pitch_rate, 0.0);
}

TEST(ConfigPidTest, ExplicitOuterLoopKeysOverrideLegacyCompatibilityMapping) {
  ScopedConfigPidRestore restore;

  const auto path = write_temp_pid_conf(
      "angle_to_rate_k = 7\n"
      "vel_P = -0.0015\n"
      "pos_P = 2.5\n"
      "outer_k_pos = -0.02\n"
      "outer_k_vel = -0.00011\n"
      "outer_k_pitch = 15\n"
      "outer_k_pitch_rate = 0.35\n");

  ConfigPid::load(path.string());
  std::filesystem::remove(path);

  EXPECT_DOUBLE_EQ(ConfigPid::outer_k_pos, -0.02);
  EXPECT_DOUBLE_EQ(ConfigPid::outer_k_vel, -0.00011);
  EXPECT_DOUBLE_EQ(ConfigPid::outer_k_pitch, 15.0);
  EXPECT_DOUBLE_EQ(ConfigPid::outer_k_pitch_rate, 0.35);
}

TEST(RateControllerCoreTest, SteeringSplitsWheelCommands) {
  RateControllerHarness h;
  h.setJoystick(0.0, 0.6);
  h.run_steps(120, 1.0 / 400.0);

  EXPECT_LT(h.runner().lastRight(), h.runner().lastLeft());
  EXPECT_NE(h.runner().lastLeft(), h.runner().lastRight());
}

TEST(RateControllerCoreTest, SmallResidualVelocityProducesOnlySmallCorrectivePitchRef) {
  RateControllerHarness h;
  h.setJoystick(0.0, 0.0);
  h.run_steps(40, 1.0 / 400.0);

  h.runner().setActualSpeedSps(50.0);
  h.run_steps(80, 1.0 / 400.0);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_LT(h.telemetry().back().vel_error, 0.0);
  EXPECT_LT(h.telemetry().back().vel_p_term, 0.0);
  EXPECT_LT(h.telemetry().back().pitch_ref_from_vel_deg, 0.0);
  EXPECT_LT(std::abs(h.telemetry().back().pitch_sp_deg), 1.0);
}

TEST(RateControllerCoreTest, LargeResidualVelocityIsBrakedWithoutCommand) {
  RateControllerHarness h;
  h.setJoystick(0.0, 0.0);
  h.run_steps(40, 1.0 / 400.0);

  h.runner().setActualSpeedSps(4000.0);
  h.run_steps(80, 1.0 / 400.0);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_LT(h.telemetry().back().vel_error, 0.0);
  EXPECT_LT(h.telemetry().back().vel_p_term, 0.0);
  EXPECT_LT(h.telemetry().back().pitch_ref_from_vel_deg, 0.0);
  EXPECT_GT(std::abs(h.telemetry().back().pitch_sp_deg), 1e-3);
  EXPECT_LE(std::abs(h.telemetry().back().pitch_sp_deg), Config::max_tilt_rad * 180.0 / M_PI + 0.1);
}

TEST(RateControllerCoreTest, NegativeResidualVelocityProducesPositiveCorrectivePitchRef) {
  RateControllerHarness h;
  h.setJoystick(0.0, 0.0);
  h.run_steps(40, 1.0 / 400.0);

  h.runner().setActualSpeedSps(-400.0);
  h.run_steps(80, 1.0 / 400.0);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_GT(h.telemetry().back().vel_error, 0.0);
  EXPECT_GT(h.telemetry().back().vel_p_term, 0.0);
  EXPECT_GT(h.telemetry().back().pitch_ref_from_vel_deg, 0.0);
}

TEST(RateControllerCoreTest, VelocityFeedbackAffectsTelemetryWhenCommanded) {
  RateControllerHarness h;
  h.setJoystick(0.2, 0.0);
  h.run_steps(40, 1.0 / 400.0);

  constexpr double kMeasuredVelocitySps = 1234.0;
  h.runner().setActualSpeedSps(kMeasuredVelocitySps);
  h.run_steps(80, 1.0 / 400.0);

  ASSERT_FALSE(h.telemetry().empty());
  const double target_velocity_sps = 0.2 * kMaxSps;
  const double used_velocity_sps = target_velocity_sps - h.telemetry().back().vel_error;
  EXPECT_NEAR(used_velocity_sps, kMeasuredVelocitySps, 20.0);
  EXPECT_NE(h.telemetry().back().vel_p_term, 0.0);
}

TEST(RateControllerCoreTest, PositionHoldAddsDirectPitchTargetBackTowardAnchor) {
  const double old_outer_k_pos = ConfigPid::outer_k_pos;
  struct RestoreOuterKPos {
    double& slot;
    double old_value;
    ~RestoreOuterKPos() { slot = old_value; }
  } restore{ConfigPid::outer_k_pos, old_outer_k_pos};
  ConfigPid::outer_k_pos = 0.4;

  RateControllerHarness h;
  h.setJoystick(0.0, 0.0);
  h.run_steps(10, 1.0 / 400.0);
  h.runner().setAveragePositionSteps(1000.0);
  h.run_steps(80, 1.0 / 400.0);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_NEAR(h.telemetry().back().vel_error, 0.0, 1e-3);
  EXPECT_NEAR(h.telemetry().back().vel_p_term, 0.0, 1e-6);
  EXPECT_LT(h.telemetry().back().position_target_vel_sps, 0.0);
  EXPECT_LT(h.telemetry().back().pitch_ref_from_pos_deg, 0.0);
  EXPECT_NE(h.runner().lastLeft(), 0.0);
  EXPECT_NE(h.runner().lastRight(), 0.0);
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
  h.setJoystick(0.0, 0.0);
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
  h.setJoystick(0.0, 0.0);
  h.runner().setActualSpeedSps(800.0);
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
  h.setJoystick(0.0, 0.0);
  h.runner().setActualSpeedSps(-800.0);
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
  h.setJoystick(0.0, 0.0);
  h.runner().setActualSpeedSps(800.0);
  h.run_steps(400, 1.0 / 400.0, 0.0, 0.0);
  const double accumulated_trim_deg = std::abs(h.telemetry().back().pitch_trim_deg);

  h.setJoystick(0.2, 0.0);
  h.run_steps(400, 1.0 / 400.0, 0.0, 0.0);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_LT(std::abs(h.telemetry().back().pitch_trim_deg), accumulated_trim_deg);
  EXPECT_LT(h.telemetry().back().trim_active, 0.5);
}

TEST(RateControllerCoreTest, LeanTrimHoldsWhenVelocityIsInsideDeadband) {
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
  h.setJoystick(0.0, 0.0);
  h.runner().setActualSpeedSps(800.0);
  h.run_steps(400, 1.0 / 400.0, 0.0, 0.0);
  const double accumulated_trim_deg = std::abs(h.telemetry().back().pitch_trim_deg);
  ASSERT_GT(accumulated_trim_deg, 1e-3);

  h.runner().setActualSpeedSps(0.0);
  h.run_steps(2400, 1.0 / 400.0, 0.0, 0.0);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_GT(std::abs(h.telemetry().back().pitch_trim_deg), 0.75 * accumulated_trim_deg);
  EXPECT_LT(h.telemetry().back().trim_active, 0.5);
}

TEST(RateControllerCoreTest, LeanTrimFreezesOnModerateTilt) {
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
  h.setJoystick(0.0, 0.0);
  h.runner().setActualSpeedSps(800.0);
  h.run_steps(400, 1.0 / 400.0, 0.0, 0.0);
  const double accumulated_trim_deg = h.telemetry().back().pitch_trim_deg;
  ASSERT_GT(std::abs(accumulated_trim_deg), 1e-3);

  h.run_steps(800, 1.0 / 400.0, 12.0 * M_PI / 180.0, 0.0);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_NEAR(h.telemetry().back().pitch_trim_deg, accumulated_trim_deg, 1e-3);
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
  h.setJoystick(0.0, 0.0);
  h.runner().setActualSpeedSps(800.0);
  h.run_steps(400, 1.0 / 400.0, 0.0, 0.0);
  ASSERT_GT(std::abs(h.telemetry().back().pitch_trim_deg), 1e-3);

  h.run_steps(8, 1.0 / 400.0, 25.0 * M_PI / 180.0, 0.0);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_NEAR(h.telemetry().back().pitch_trim_deg, 0.0, 1e-6);
  EXPECT_LT(h.telemetry().back().trim_active, 0.5);
}

TEST(ControlServiceTest, UsesFallbackVelocityProxyWhenNoMotorFeedbackExists) {
  ControlServiceHarness h;
  h.sendJoystick(0.2, 0.0);

  for (int i = 0; i < 200; ++i) {
    const uint64_t sim_time_us = static_cast<uint64_t>((i + 1) * 2500);
    h.step_with_imu(1.0 / 400.0, sim_time_us);
  }

  ASSERT_FALSE(h.telemetry().empty());
  const double target_velocity_sps = 0.2 * kMaxSps;
  const double used_velocity_sps = target_velocity_sps - h.telemetry().back().vel_error;
  EXPECT_GT(std::abs(used_velocity_sps), 1.0);
}

TEST(ImuServiceTest, ConvertsRawImuToFusedImuDataAndPreservesRawVectors) {
  ImuServiceHarness h;
  const double pitch_rad = 4.0 * M_PI / 180.0;
  ipc::ImuRawPayload raw{};
  raw.acc = accel_for_pitch(pitch_rad);
  raw.gyr = {0.1, 0.2, 0.3};
  raw.timestamp_us = 123456;

  h.publish_raw(raw);

  ASSERT_EQ(h.fused_samples().size(), 1u);
  const auto& fused = h.fused_samples().back();
  EXPECT_NEAR(fused.pitch_rad, pitch_rad, 1e-6);
  EXPECT_NEAR(fused.filtered_pitch_rate_rad_s, raw.gyr[1], 1e-6);
  EXPECT_EQ(fused.acc, raw.acc);
  EXPECT_EQ(fused.gyr, raw.gyr);
  EXPECT_EQ(fused.timestamp_us, raw.timestamp_us);
}

TEST(ControlServiceTest, UsesMotorFeedbackForVelocityTelemetry) {
  ControlServiceHarness h;
  h.sendJoystick(0.2, 0.0);

  for (int i = 0; i < 200; ++i) {
    const uint64_t sim_time_us = static_cast<uint64_t>((i + 1) * 2500);
    h.sendMotorFeedback(200.0, 46.0, 123.0, 0, 0);
    h.step_with_imu(1.0 / 400.0, sim_time_us);
  }

  ASSERT_FALSE(h.telemetry().empty());
  const double target_velocity_sps = 0.2 * kMaxSps;
  const double used_velocity_sps = target_velocity_sps - h.telemetry().back().vel_error;
  EXPECT_NEAR(used_velocity_sps, 123.0, 5.0);
  EXPECT_GT(h.telemetry().back().pitch_ref_from_vel_deg, 0.0);
  EXPECT_NEAR(h.telemetry().back().left_applied_sps, 200.0, 1e-3);
  EXPECT_NEAR(h.telemetry().back().right_applied_sps, 46.0, 1e-3);
  EXPECT_EQ(h.telemetry().back().left_actual_steps, 0);
  EXPECT_EQ(h.telemetry().back().right_actual_steps, 0);
}

TEST(ControlServiceTest, UsesFilteredPitchRateForControlAndKeepsRawGyroForDiagnostics) {
  ControlServiceHarness h;
  h.sendJoystick(0.0, 0.0);
  h.step_with_imu(1.0 / 400.0, 2500, 0.0, 0.25, 1.0);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_NEAR(h.telemetry().back().pitch_rate_dps, 0.25 * 180.0 / M_PI, 1e-3);
  EXPECT_NEAR(h.telemetry().back().filtered_pitch_rate_dps, 0.25 * 180.0 / M_PI, 1e-3);
  EXPECT_NEAR(h.telemetry().back().gyro_pitch_rate_dps, 1.0 * 180.0 / M_PI, 1e-3);
}

TEST(ControlServiceTest, UsesMotorFeedbackPositionForPositionHold) {
  const double old_outer_k_pos = ConfigPid::outer_k_pos;
  struct RestoreOuterKPos {
    ~RestoreOuterKPos() { ConfigPid::outer_k_pos = old_value; }
    double old_value;
  } restore{old_outer_k_pos};
  ConfigPid::outer_k_pos = 0.4;

  ControlServiceHarness h;
  h.sendJoystick(0.0, 0.0);

  for (int i = 0; i < 20; ++i) {
    const uint64_t sim_time_us = static_cast<uint64_t>((i + 1) * 2500);
    h.sendMotorFeedback(0.0, 0.0, 0.0, 0, 0);
    h.step_with_imu(1.0 / 400.0, sim_time_us);
  }

  const int64_t displacement_steps = 1000;
  for (int i = 20; i < 200; ++i) {
    const uint64_t sim_time_us = static_cast<uint64_t>((i + 1) * 2500);
    h.sendMotorFeedback(0.0, 0.0, 0.0, displacement_steps, displacement_steps);
    h.step_with_imu(1.0 / 400.0, sim_time_us);
  }

  ASSERT_FALSE(h.telemetry().empty());
  const double expected_velocity_sps = std::clamp(
      (ConfigPid::outer_k_pos / ConfigPid::outer_k_vel) *
          (-static_cast<double>(displacement_steps) * Config::meters_per_step),
      -800.0,
      800.0);
  EXPECT_NEAR(h.telemetry().back().vel_error, 0.0, 1e-3);
  EXPECT_NEAR(h.telemetry().back().position_target_vel_sps, expected_velocity_sps, 5.0);
  EXPECT_LT(h.telemetry().back().pitch_ref_from_pos_deg, 0.0);
}

TEST(ControlServiceTest, TelemetryCarriesImuDiagnostics) {
  ControlServiceHarness h;
  h.sendJoystick(0.0, 0.0);

  const double angle_rad = 5.0 * M_PI / 180.0;
  const double gyro_rad_s = 0.2;
  h.step_with_imu(1.0 / 400.0, 2500, angle_rad, gyro_rad_s);

  ASSERT_FALSE(h.telemetry().empty());
  const auto& t = h.telemetry().back();
  EXPECT_NEAR(t.raw_acc_pitch_deg, 5.0, 0.1);
  EXPECT_NEAR(t.fused_pitch_deg, 5.0, 0.1);
  EXPECT_NEAR(t.gyro_pitch_rate_dps, gyro_rad_s * 180.0 / M_PI, 0.1);
}

TEST(ControlServiceTest, TelemetryCarriesOuterLoopBreakdownAndMotorFeedback) {
  ControlServiceHarness h;
  h.sendJoystick(0.2, 0.0);

  for (int i = 0; i < 80; ++i) {
    const uint64_t sim_time_us = static_cast<uint64_t>((i + 1) * 2500);
    h.sendMotorFeedback(140.0, 100.0, 120.0, 123, 87);
    h.step_with_imu(1.0 / 400.0, sim_time_us, 2.0 * M_PI / 180.0, 0.1);
  }

  ASSERT_FALSE(h.telemetry().empty());
  const auto& t = h.telemetry().back();
  EXPECT_GT(t.pitch_ref_from_vel_deg, 0.0);
  EXPECT_LT(t.pitch_error_deg, t.pitch_ref_from_vel_deg);
  EXPECT_NEAR(t.left_applied_sps, 140.0, 1e-3);
  EXPECT_NEAR(t.right_applied_sps, 100.0, 1e-3);
  EXPECT_EQ(t.left_actual_steps, 123);
  EXPECT_EQ(t.right_actual_steps, 87);
}

TEST(ServiceBusIntegrationTest, TickPublishesMotorTargetsAndImmediateMotorFeedback) {
  ServiceBusHarness h;
  h.sendJoystick(0.2, 0.0);

  h.sendStep(1.0 / 400.0, 2500);

  ASSERT_FALSE(h.motor_targets().empty());
  ASSERT_FALSE(h.feedback().empty());

  const auto expected = h.runner().getFeedbackSample();
  EXPECT_NEAR(h.feedback().back().left_applied_sps, expected.left_applied_sps, 1e-3);
  EXPECT_NEAR(h.feedback().back().right_applied_sps, expected.right_applied_sps, 1e-3);
  EXPECT_EQ(h.feedback().back().left_actual_steps, expected.left_actual_steps);
  EXPECT_EQ(h.feedback().back().right_actual_steps, expected.right_actual_steps);
}

}  // namespace
