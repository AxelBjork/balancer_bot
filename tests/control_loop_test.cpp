#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <utility>
#include <vector>

#include "services/control/control_service.h"
#include "services/control/rate_controller_core.h"
#include "services/imu/imu_service.h"
#include "services/main/config.h"
#include "services/motor/motor_runner.h"
#include "services/motor/motor_service.h"

namespace {

std::array<double, 3> accel_for_pitch(double angle_rad) {
  return {
      -9.81 * std::sin(angle_rad),
      0.0,
      -9.81 * std::cos(angle_rad),
  };
}

class FakeMotorRunner {
 public:
  void setTargets(double left_sps, double right_sps, uint64_t /*timestamp_us*/) {
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

  double lastLeft() const {
    return last_left_;
  }
  double lastRight() const {
    return last_right_;
  }
  int callCount() const {
    return calls_;
  }

 private:
  double last_left_{0.0};
  double last_right_{0.0};
  double actual_speed_sps_{0.0};
  int calls_{0};
};

class RateControllerHarness {
 public:
  RateControllerHarness() {
    core_.setMotorOutputs(
        [this](double left, double right) { runner_.setTargets(left, right, current_time_us_); });
    core_.setTelemetrySink([this](const Telemetry& t) { telemetry_.push_back(t); });
  }

  void setJoystick(double forward, double turn) {
    core_.setJoystick(JoyCmd{static_cast<float>(forward), static_cast<float>(turn)});
  }

  void setImu(double angle_rad, double gyro_rad_s, uint64_t sim_time_us,
              double pitch_accel_rad_s2 = 0.0) {
    ImuSample s{};
    s.angle_rad = angle_rad;
    s.gyro_rad_s = gyro_rad_s;
    s.pitch_accel_rad_s2 = pitch_accel_rad_s2;
    s.t = std::chrono::steady_clock::time_point(std::chrono::microseconds(sim_time_us));
    core_.pushImu(s);
  }

  void tick(double dt_s, uint64_t sim_time_us) {
    const auto now = std::chrono::steady_clock::time_point(std::chrono::microseconds(sim_time_us));
    current_time_us_ = sim_time_us;
    core_.setMotorFeedback(runner_.getActualSpeedSps(), actuator_fault_);
    core_.step(dt_s, now);
  }

  void setActuatorFault(bool fault) {
    actuator_fault_ = fault;
  }

  void run_steps(int count, double dt_s, double angle_rad = 0.0, double gyro_rad_s = 0.0) {
    for (int i = 0; i < count; ++i) {
      const uint64_t sim_time_us =
          current_time_us_ + static_cast<uint64_t>(std::llround(dt_s * 1e6));
      setImu(angle_rad, gyro_rad_s, sim_time_us);
      tick(dt_s, sim_time_us);
    }
  }

  const std::vector<Telemetry>& telemetry() const {
    return telemetry_;
  }
  const FakeMotorRunner& runner() const {
    return runner_;
  }
  FakeMotorRunner& runner() {
    return runner_;
  }

 private:
  FakeMotorRunner runner_;
  RateControllerCore core_;
  std::vector<Telemetry> telemetry_;
  uint64_t current_time_us_{0};
  bool actuator_fault_{false};
};

class ControlServiceHarness {
 public:
  ControlServiceHarness() : bus_(this, &ControlServiceHarness::dispatch), control_(bus_) {
  }

  void sendJoystick(double forward, double turn) {
    control_.on_message<MsgId::JoystickCommand>(
        ipc::JoystickCommandPayload{static_cast<float>(forward), static_cast<float>(turn)});
  }

  void sendMotorFeedback(double left_applied_sps, double right_applied_sps, double measured_avg_sps,
                         int64_t left_actual_steps, int64_t right_actual_steps,
                         double update_dt_ms = 1000.0 / 400.0) {
    ipc::MotorFeedbackPayload payload{};
    payload.left_applied_sps = left_applied_sps;
    payload.right_applied_sps = right_applied_sps;
    payload.measured_avg_sps = measured_avg_sps;
    payload.update_dt_ms = update_dt_ms;
    payload.left_actual_steps = left_actual_steps;
    payload.right_actual_steps = right_actual_steps;
    control_.on_message<MsgId::MotorFeedback>(payload);
  }

  void step_with_imu(double dt_s, uint64_t sim_time_us, double angle_rad = 0.0,
                     double pitch_rate_rad_s = 0.0, double raw_pitch_rate_rad_s = 0.0,
                     double pitch_accel_rad_s2 = 0.0) {
    ipc::ImuSamplePayload imu{};
    imu.pitch_rad = angle_rad;
    imu.pitch_rate_rad_s = pitch_rate_rad_s;
    imu.pitch_accel_rad_s2 = pitch_accel_rad_s2;
    imu.acc = accel_for_pitch(angle_rad);
    if (raw_pitch_rate_rad_s == 0.0) {
      raw_pitch_rate_rad_s = pitch_rate_rad_s;
    }
    imu.gyr = {0.0, raw_pitch_rate_rad_s, 0.0};
    imu.timestamp_us = sim_time_us;
    control_.on_message<MsgId::ImuData>(imu);

    PhysicsTickPayload tick{};
    tick.dt_s = dt_s;
    tick.timestamp_us = sim_time_us;
    control_.on_message<MsgId::PhysicsTick>(tick);
  }

  const std::vector<ipc::MotorTargetsPayload>& motor_targets() const {
    return motor_targets_;
  }
  const std::vector<ipc::SystemTelemetryPayload>& telemetry() const {
    return telemetry_;
  }

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
  ImuServiceHarness() : bus_(this, &ImuServiceHarness::dispatch), imu_(bus_, false) {
  }

  void publish_raw(const ipc::ImuRawPayload& payload) {
    bus_.publish<MsgId::ImuRawData>(payload);
  }

  const std::vector<ipc::ImuSamplePayload>& fused_samples() const {
    return fused_samples_;
  }

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
        motor_(bus_, &runner_) {
  }

  void sendJoystick(double forward, double turn) {
    ipc::JoystickCommandPayload payload{};
    payload.forward = static_cast<float>(forward);
    payload.turn = static_cast<float>(turn);
    bus_.publish<MsgId::JoystickCommand>(payload);
  }

  void sendStep(double dt_s, uint64_t sim_time_us, double angle_rad = 0.0,
                double gyro_rad_s = 0.0) {
    ipc::ImuSamplePayload imu{};
    imu.pitch_rad = angle_rad;
    imu.pitch_rate_rad_s = gyro_rad_s;
    imu.pitch_accel_rad_s2 = 0.0;
    imu.acc = accel_for_pitch(angle_rad);
    imu.gyr = {0.0, gyro_rad_s, 0.0};
    imu.timestamp_us = sim_time_us;
    bus_.publish<MsgId::ImuData>(imu);

    PhysicsTickPayload tick{};
    tick.dt_s = dt_s;
    tick.timestamp_us = sim_time_us;
    bus_.publish<MsgId::PhysicsTick>(tick);
  }

  const std::vector<ipc::MotorTargetsPayload>& motor_targets() const {
    return motor_targets_;
  }
  const std::vector<ipc::MotorFeedbackPayload>& feedback() const {
    return feedback_;
  }
  const std::vector<ipc::SystemTelemetryPayload>& telemetry() const {
    return telemetry_;
  }
  MotorRunner& runner() {
    return runner_;
  }

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

    ipc::dispatch_to_services(id, payload, self->imu_, self->motor_, self->control_);
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
  double velocity_P = ConfigPid::velocity_P;
  double velocity_I = ConfigPid::velocity_I;
  double velocity_I_limit_deg = ConfigPid::velocity_I_limit_deg;
  double angle_P = ConfigPid::angle_P;
  double angle_D = ConfigPid::angle_D;
  double drive_max_sps = ConfigPid::drive_max_sps;
  double turn_max_sps = ConfigPid::turn_max_sps;
  double pitch_max_deg = ConfigPid::pitch_max_deg;
  double balance_max_sps = ConfigPid::balance_max_sps;
  double output_scale_sps = ConfigPid::output_scale_sps;
  bool controller_enabled = ConfigPid::controller_enabled;

  void restore() const {
    ConfigPid::rate_P = rate_P;
    ConfigPid::rate_I = rate_I;
    ConfigPid::rate_D = rate_D;
    ConfigPid::rate_I_lim = rate_I_lim;
    ConfigPid::rate_FF = rate_FF;
    ConfigPid::velocity_P = velocity_P;
    ConfigPid::velocity_I = velocity_I;
    ConfigPid::velocity_I_limit_deg = velocity_I_limit_deg;
    ConfigPid::angle_P = angle_P;
    ConfigPid::angle_D = angle_D;
    ConfigPid::drive_max_sps = drive_max_sps;
    ConfigPid::turn_max_sps = turn_max_sps;
    ConfigPid::pitch_max_deg = pitch_max_deg;
    ConfigPid::balance_max_sps = balance_max_sps;
    ConfigPid::output_scale_sps = output_scale_sps;
    ConfigPid::controller_enabled = controller_enabled;
  }
};

struct ScopedConfigPidRestore {
  ConfigPidSnapshot snapshot{};
  ~ScopedConfigPidRestore() {
    snapshot.restore();
  }
};

void set_zeroed_gain_audit_config() {
  ConfigPid::rate_P = 0.0;
  ConfigPid::rate_I = 0.0;
  ConfigPid::rate_D = 0.0;
  ConfigPid::rate_I_lim = 1.0;
  ConfigPid::rate_FF = 0.0;
  ConfigPid::velocity_P = 0.0;
  ConfigPid::velocity_I = 0.0;
  ConfigPid::velocity_I_limit_deg = 4.0;
  ConfigPid::angle_P = 0.0;
  ConfigPid::angle_D = 0.0;
  ConfigPid::drive_max_sps = 1000.0;
  ConfigPid::turn_max_sps = 1000.0;
  ConfigPid::pitch_max_deg = 10.0;
  ConfigPid::balance_max_sps = 12000.0;
  ConfigPid::output_scale_sps = 3200.0;
}

double run_fresh_core_once(double angle_rad, double gyro_rad_s, double velocity_sps = 0.0) {
  RateControllerHarness h;
  h.runner().setActualSpeedSps(velocity_sps);
  h.setJoystick(0.0, 0.0);
  h.setImu(angle_rad, gyro_rad_s, 2500);
  h.tick(1.0 / 400.0, 2500);
  return h.runner().lastLeft();
}

std::filesystem::path write_temp_pid_config(std::string contents) {
  const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto path = std::filesystem::temp_directory_path() /
                    ("balancer_pid_v3_" + std::to_string(suffix) + ".conf");
  std::ofstream output(path);
  output << contents;
  output.close();
  return path;
}

const char* valid_pid_v3_config() {
  return R"(config_version = 3
rate_P = 0.25
rate_I = 0
rate_D = 0
rate_I_lim = 0.15
rate_FF = 0
velocity_P = 0.002
velocity_I = 0.001
velocity_I_limit_deg = 4
angle_P = 12
angle_D = 0.25
drive_max_sps = 1200
turn_max_sps = 1600
pitch_max_deg = 10
balance_max_sps = 12000
output_scale_sps = 3200
)";
}

TEST(ConfigPidV3Test, LoadsCompleteStrictSchema) {
  ScopedConfigPidRestore restore;
  const auto path = write_temp_pid_config(valid_pid_v3_config());
  EXPECT_NO_THROW(ConfigPid::load(path.string()));
  EXPECT_DOUBLE_EQ(ConfigPid::velocity_P, 0.002);
  EXPECT_DOUBLE_EQ(ConfigPid::balance_max_sps, 12000.0);
  EXPECT_TRUE(ConfigPid::controller_enabled);
  std::filesystem::remove(path);
}

TEST(ConfigPidV3Test, OptionalControllerEnabledDisablesActuationMode) {
  ScopedConfigPidRestore restore;
  const auto path = write_temp_pid_config(std::string(valid_pid_v3_config()) + "controller_enabled = 0\n");
  ASSERT_NO_THROW(ConfigPid::load(path.string()));
  EXPECT_FALSE(ConfigPid::controller_enabled);
  std::filesystem::remove(path);
}

TEST(ConfigPidV3Test, RejectsUnknownDuplicateMissingNonFiniteAndInvalidValues) {
  const std::vector<std::string> invalid = {
      std::string(valid_pid_v3_config()) + "legacy_gain = 1\n",
      std::string(valid_pid_v3_config()) + "rate_P = 1\n",
      "config_version = 3\n",
      std::string(valid_pid_v3_config())
          .replace(std::string(valid_pid_v3_config()).find("rate_P = 0.25"), 13, "rate_P = nan"),
      std::string(valid_pid_v3_config())
          .replace(std::string(valid_pid_v3_config()).find("pitch_max_deg = 10"), 18,
                   "pitch_max_deg = 90"),
      std::string(valid_pid_v3_config()).replace(0, 18, "config_version = 2"),
  };
  for (const auto& contents : invalid) {
    const auto path = write_temp_pid_config(contents);
    EXPECT_THROW(ConfigPid::load(path.string()), std::runtime_error) << contents;
    std::filesystem::remove(path);
  }
}

TEST(ConfigPidV3Test, RejectsOldVersionBeforeNewSchemaKey) {
  const auto path = write_temp_pid_config(
      std::string(valid_pid_v3_config()).replace(0, 18, "config_version = 2") +
      "future_only_key = 1\n");
  try {
    ConfigPid::load(path.string());
    FAIL() << "Expected config version mismatch";
  } catch (const std::runtime_error& error) {
    EXPECT_STREQ(error.what(), "PID configuration version mismatch: expected 3, got 2");
  }
  std::filesystem::remove(path);
}

TEST(RateControllerCoreTest, ZeroInputsStayNearZero) {
  RateControllerHarness h;
  h.setJoystick(0.0, 0.0);
  h.run_steps(120, 1.0 / 400.0);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_NEAR(h.runner().lastLeft(), 0.0, 1e-3);
  EXPECT_NEAR(h.runner().lastRight(), 0.0, 1e-3);
}

TEST(RateControllerCoreTest, MissingImuPublishesSafeTelemetryAndZeroCommand) {
  RateControllerHarness h;
  h.setJoystick(1.0, 0.5);
  h.tick(1.0 / 400.0, 2500);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_NEAR(h.runner().lastLeft(), 0.0, 1e-9);
  EXPECT_NEAR(h.runner().lastRight(), 0.0, 1e-9);
  EXPECT_EQ(h.telemetry().back().controller_fault_flags, ControllerFaultNoImu);
  EXPECT_EQ(h.telemetry().back().controller_saturation_flags, ControllerSaturationNone);
}

TEST(RateControllerCoreGainAuditTest, RatePIsConnectedAndScalesOutput) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();
  ConfigPid::angle_P = 10.0;

  ConfigPid::rate_P = 0.0;
  const double no_p = run_fresh_core_once(1.0 * M_PI / 180.0, 0.0);

  ConfigPid::rate_P = 0.2;
  const double low_p = run_fresh_core_once(1.0 * M_PI / 180.0, 0.0);

  ConfigPid::rate_P = 0.4;
  const double high_p = run_fresh_core_once(1.0 * M_PI / 180.0, 0.0);

  EXPECT_NEAR(no_p, 0.0, 1e-6);
  EXPECT_GT(low_p, 0.0);
  EXPECT_NEAR(high_p, 2.0 * low_p, std::abs(low_p) * 0.05);
}

TEST(RateControllerCoreGainAuditTest, RateIIsConnectedAndAccumulatesAcrossSteps) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();
  ConfigPid::rate_I = 0.5;
  ConfigPid::rate_I_lim = 0.2;
  ConfigPid::angle_P = 10.0;

  RateControllerHarness h;
  h.setJoystick(0.0, 0.0);
  h.run_steps(1, 1.0 / 400.0, 1.0 * M_PI / 180.0, 0.0);
  const double first_output = h.runner().lastLeft();
  h.run_steps(80, 1.0 / 400.0, 1.0 * M_PI / 180.0, 0.0);

  EXPECT_NEAR(first_output, 0.0, 1e-6);
  EXPECT_GT(h.runner().lastLeft(), 0.0);
}

TEST(RateControllerCoreGainAuditTest, RateDConsumesImuPitchAcceleration) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();
  ConfigPid::rate_D = 0.01;

  RateControllerHarness h;
  h.setJoystick(0.0, 0.0);
  h.setImu(0.0, 0.0, 2500, 25.0);
  h.tick(1.0 / 400.0, 2500);

  EXPECT_GT(h.runner().lastLeft(), 0.0);
  EXPECT_NEAR(h.runner().lastLeft(), h.runner().lastRight(), 1e-6);
}

TEST(RateControllerCoreTest, PitchRateSetpointIsLimitedBeforeRateController) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();
  ConfigPid::rate_P = 0.25;
  ConfigPid::angle_P = 100.0;

  RateControllerHarness h;
  h.setJoystick(0.0, 0.0);
  h.setImu(-10.0 * M_PI / 180.0, 0.0, 2500);
  h.tick(1.0 / 400.0, 2500);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_NEAR(h.runner().lastLeft(), -0.25 * 4.0 * ConfigPid::output_scale_sps, 1e-3);
  EXPECT_NEAR(h.runner().lastRight(), h.runner().lastLeft(), 1e-6);
}

TEST(RateControllerCoreTest, RateControllerOutputSubtractsSymmetricallyAtMotorBoundary) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();
  ConfigPid::rate_P = 0.25;
  ConfigPid::angle_P = 100.0;

  const double positive_rate_output_motor =
      run_fresh_core_once(-10.0 * M_PI / 180.0, 0.0);
  const double negative_rate_output_motor =
      run_fresh_core_once(10.0 * M_PI / 180.0, 0.0);

  EXPECT_LT(positive_rate_output_motor, 0.0);
  EXPECT_GT(negative_rate_output_motor, 0.0);
  EXPECT_NEAR(positive_rate_output_motor, -negative_rate_output_motor, 1e-6);
}

TEST(RateControllerCoreTest, BalancePriorityTrimsTurnAtRail) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();
  ConfigPid::rate_P = 1.0;
  ConfigPid::angle_P = 100.0;
  ConfigPid::balance_max_sps = 8000.0;

  RateControllerHarness h;
  h.setJoystick(0.0, 0.6);
  h.setImu(10.0 * M_PI / 180.0, 0.0, 2500);
  h.tick(1.0 / 400.0, 2500);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_LE(std::abs(h.runner().lastLeft()), ConfigPid::balance_max_sps);
  EXPECT_LE(std::abs(h.runner().lastRight()), ConfigPid::balance_max_sps);
  EXPECT_NEAR(h.telemetry().back().u_sps, ConfigPid::balance_max_sps, 1e-6);
  EXPECT_NEAR(h.telemetry().back().turn_sps, 0.0, 1e-6);
  EXPECT_NEAR(h.runner().lastLeft(), ConfigPid::balance_max_sps, 1e-6);
  EXPECT_NEAR(h.runner().lastRight(), ConfigPid::balance_max_sps, 1e-6);
}

TEST(RateControllerCoreTest, VelocityOuterLoopRunsAtFiftyHertzAndMapsForwardCommand) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();
  ConfigPid::velocity_P = 0.002;
  ConfigPid::velocity_I = 0.001;

  RateControllerHarness h;
  h.setJoystick(1.0, 0.0);
  h.run_steps(7, 1.0 / 400.0);
  EXPECT_NEAR(h.telemetry().back().target_vel_sps, 0.0, 1e-9);
  h.run_steps(1, 1.0 / 400.0);
  EXPECT_NEAR(h.telemetry().back().target_vel_sps, 48.0, 1e-9);
  EXPECT_GT(h.telemetry().back().vel_p_term_deg, 0.0);
  EXPECT_NEAR(h.telemetry().back().vel_i_term_deg, 0.0, 1e-9);
  EXPECT_GT(h.telemetry().back().pitch_sp_deg, h.telemetry().back().vel_p_term_deg);
  EXPECT_NEAR(h.runner().lastLeft(), 0.0, 1e-6);
  EXPECT_NEAR(h.runner().lastRight(), 0.0, 1e-6);

  h.run_steps(192, 1.0 / 400.0);
  EXPECT_NEAR(h.telemetry().back().target_vel_sps, ConfigPid::drive_max_sps, 1e-9);
  EXPECT_NEAR(h.runner().lastLeft(), 0.0, 1e-6);
  EXPECT_NEAR(h.runner().lastRight(), 0.0, 1e-6);
}

TEST(RateControllerCoreTest, ForwardReferenceBrakesThroughZeroBeforeReversing) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();

  RateControllerHarness h;
  h.setJoystick(1.0, 0.0);
  h.run_steps(200, 1.0 / 400.0);
  ASSERT_NEAR(h.telemetry().back().target_vel_sps, ConfigPid::drive_max_sps, 1e-9);

  h.setJoystick(-1.0, 0.0);
  h.run_steps(112, 1.0 / 400.0);
  EXPECT_NEAR(h.telemetry().back().target_vel_sps, 0.0, 1e-9);
  h.run_steps(8, 1.0 / 400.0);
  EXPECT_NEAR(h.telemetry().back().target_vel_sps, -48.0, 1e-9);
}

TEST(RateControllerCoreTest, PositiveAndNegativeVelocityErrorsProduceSymmetricPitchReferences) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();
  ConfigPid::velocity_P = 0.004;

  const auto run_direction = [](double direction) {
    RateControllerHarness h;
    h.setJoystick(direction, 0.0);
    h.runner().setActualSpeedSps(0.0);
    h.run_steps(200, 1.0 / 400.0);
    return h.telemetry().back();
  };

  const auto positive_telemetry = run_direction(1.0);
  const auto negative_telemetry = run_direction(-1.0);
  EXPECT_GT(positive_telemetry.vel_error, 0.0);
  EXPECT_LT(negative_telemetry.vel_error, 0.0);
  EXPECT_GT(positive_telemetry.vel_p_term_deg, 0.0);
  EXPECT_LT(negative_telemetry.vel_p_term_deg, 0.0);
  EXPECT_GT(positive_telemetry.pitch_sp_deg, 0.0);
  EXPECT_LT(negative_telemetry.pitch_sp_deg, 0.0);
  EXPECT_NEAR(positive_telemetry.pitch_sp_deg, -negative_telemetry.pitch_sp_deg, 1e-6);
}

TEST(RateControllerCoreTest, TargetAccelerationPitchFeedForwardMatchesPlantPolarity) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();

  const auto run_direction = [](double direction) {
    RateControllerHarness h;
    h.setJoystick(direction, 0.0);
    h.runner().setActualSpeedSps(0.0);
    h.run_steps(8, 1.0 / 400.0);
    return h.telemetry().back();
  };

  const auto positive = run_direction(1.0);
  const auto negative = run_direction(-1.0);
  const double expected_pitch_deg =
      std::atan2(2400.0 * Config::meters_per_step, Config::g0) * 180.0 / M_PI;
  EXPECT_GT(positive.target_vel_sps, 0.0);
  EXPECT_LT(negative.target_vel_sps, 0.0);
  EXPECT_NEAR(positive.vel_p_term_deg, 0.0, 1e-9);
  EXPECT_NEAR(negative.vel_p_term_deg, 0.0, 1e-9);
  EXPECT_NEAR(positive.pitch_sp_deg, expected_pitch_deg, 1e-6);
  EXPECT_NEAR(negative.pitch_sp_deg, -expected_pitch_deg, 1e-6);
}

TEST(RateControllerCoreTest, StationaryComTrimOpposesNeutralDriftAndRemainsBounded) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();
  ConfigPid::velocity_I = 0.01;
  ConfigPid::velocity_I_limit_deg = 2.0;

  RateControllerHarness h;
  h.setJoystick(0.0, 0.0);
  h.runner().setActualSpeedSps(100.0);
  h.run_steps(800, 1.0 / 400.0);
  ASSERT_LT(h.telemetry().back().vel_i_term_deg, 0.0);
  EXPECT_GE(h.telemetry().back().vel_i_term_deg, -ConfigPid::velocity_I_limit_deg);

  const double learned_trim_deg = h.telemetry().back().vel_i_term_deg;
  h.runner().setActualSpeedSps(0.0);
  h.run_steps(8, 1.0 / 400.0);
  EXPECT_NEAR(h.telemetry().back().vel_i_term_deg, learned_trim_deg, 1e-9);
}

TEST(RateControllerCoreTest, StationaryComTrimCorrectsSymmetricNeutralDriftAtAnySpeed) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();
  ConfigPid::velocity_I = 0.01;
  ConfigPid::velocity_I_limit_deg = 4.0;

  const auto learn_trim = [](double measured_velocity_sps) {
    RateControllerHarness h;
    h.setJoystick(0.0, 0.0);
    h.runner().setActualSpeedSps(measured_velocity_sps);
    h.run_steps(800, 1.0 / 400.0, 6.0 * M_PI / 180.0,
                20.0 * M_PI / 180.0);
    return h.telemetry().back().vel_i_term_deg;
  };

  const double positive_drift_trim = learn_trim(1200.0);
  const double negative_drift_trim = learn_trim(-1200.0);
  EXPECT_LT(positive_drift_trim, 0.0);
  EXPECT_GT(negative_drift_trim, 0.0);
  EXPECT_NEAR(positive_drift_trim, -negative_drift_trim, 1e-6);
  EXPECT_GE(positive_drift_trim, -ConfigPid::velocity_I_limit_deg);
  EXPECT_LE(negative_drift_trim, ConfigPid::velocity_I_limit_deg);
}

TEST(RateControllerCoreTest, StationaryComTrimFreezesDuringCommand) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();
  ConfigPid::velocity_I = 0.01;
  ConfigPid::velocity_I_limit_deg = 4.0;

  RateControllerHarness h;
  h.setJoystick(0.0, 0.0);
  h.runner().setActualSpeedSps(100.0);
  h.run_steps(800, 1.0 / 400.0);
  const double learned_trim_deg = h.telemetry().back().vel_i_term_deg;
  ASSERT_LT(learned_trim_deg, -0.1);

  h.setJoystick(1.0, 0.0);
  h.runner().setActualSpeedSps(0.0);
  h.run_steps(400, 1.0 / 400.0);
  EXPECT_NEAR(h.telemetry().back().vel_i_term_deg, learned_trim_deg, 1e-9);
}

TEST(RateControllerCoreTest, StationaryComTrimFreezesWhileBalanceSaturated) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();
  ConfigPid::rate_P = 1.0;
  ConfigPid::angle_P = 100.0;
  ConfigPid::velocity_I = 0.01;
  ConfigPid::balance_max_sps = 100.0;

  RateControllerHarness h;
  h.setJoystick(0.0, 0.0);
  h.runner().setActualSpeedSps(100.0);
  h.run_steps(800, 1.0 / 400.0, 1.0 * M_PI / 180.0, 0.0);

  EXPECT_NE(h.telemetry().back().controller_saturation_flags & ControllerSaturationBalance, 0u);
  EXPECT_NEAR(h.telemetry().back().vel_i_term_deg, 0.0, 1e-9);
}

TEST(RateControllerCoreTest, CommandPitchClampsWithoutWindingComTrim) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();
  ConfigPid::velocity_P = 1.0;
  ConfigPid::velocity_I = 1.0;
  ConfigPid::velocity_I_limit_deg = 2.0;
  ConfigPid::pitch_max_deg = 3.0;

  RateControllerHarness h;
  h.setJoystick(1.0, 0.0);
  h.run_steps(400, 1.0 / 400.0);
  EXPECT_NEAR(h.telemetry().back().pitch_sp_deg, ConfigPid::pitch_max_deg, 1e-6);
  EXPECT_NEAR(h.telemetry().back().vel_i_term_deg, 0.0, 1e-6);
}

TEST(RateControllerCoreTest, StaleImuCommandsZeroAndPreservesComTrim) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();
  ConfigPid::rate_P = 1.0;
  ConfigPid::angle_P = 100.0;
  ConfigPid::velocity_I = 0.01;

  RateControllerHarness h;
  h.setJoystick(0.0, 0.0);
  h.runner().setActualSpeedSps(100.0);
  h.run_steps(800, 1.0 / 400.0);
  const double learned_trim_deg = h.telemetry().back().vel_i_term_deg;
  ASSERT_LT(learned_trim_deg, 0.0);
  h.tick(1.0 / 400.0, 3'000'000);

  EXPECT_NEAR(h.runner().lastLeft(), 0.0, 1e-6);
  EXPECT_NEAR(h.runner().lastRight(), 0.0, 1e-6);
  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_NE(h.telemetry().back().controller_fault_flags & ControllerFaultStaleImu, 0u);
  h.setJoystick(0.0, 0.0);
  h.setImu(0.0, 0.0, 3'002'500);
  h.tick(1.0 / 400.0, 3'002'500);
  EXPECT_NEAR(h.telemetry().back().vel_i_term_deg, learned_trim_deg, 1e-6);
}

TEST(RateControllerCoreTest, FalloverResetPreservesComTrimUntilRearm) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();
  ConfigPid::velocity_I = 0.01;

  RateControllerHarness h;
  h.setJoystick(0.0, 0.0);
  h.runner().setActualSpeedSps(100.0);
  h.run_steps(800, 1.0 / 400.0);
  const double learned_trim_deg = h.telemetry().back().vel_i_term_deg;
  ASSERT_LT(learned_trim_deg, 0.0);

  h.setImu(26.0 * M_PI / 180.0, 0.0, 2'002'500);
  h.tick(1.0 / 400.0, 2'002'500);
  EXPECT_NE(h.telemetry().back().controller_fault_flags & ControllerFaultFallover, 0u);
  EXPECT_NEAR(h.telemetry().back().vel_i_term_deg, learned_trim_deg, 1e-6);

  h.runner().setActualSpeedSps(0.0);
  h.setImu(0.0, 0.0, 2'005'000);
  h.tick(1.0 / 400.0, 2'005'000);
  EXPECT_EQ(h.telemetry().back().controller_fault_flags & ControllerFaultFallover, 0u);
  EXPECT_NEAR(h.telemetry().back().vel_i_term_deg, learned_trim_deg, 1e-6);
}

TEST(RateControllerCoreTest, FutureImuBeyondClockToleranceCommandsZero) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();
  ConfigPid::rate_P = 1.0;
  ConfigPid::angle_P = 100.0;

  RateControllerHarness h;
  h.setImu(1.0 * M_PI / 180.0, 0.0, 2500);
  h.tick(1.0 / 400.0, 2500);
  ASSERT_GT(std::abs(h.runner().lastLeft()), 1.0);

  h.setImu(0.0, 0.0, 10'000);
  h.tick(1.0 / 400.0, 5000);

  EXPECT_NEAR(h.runner().lastLeft(), 0.0, 1e-6);
  EXPECT_NEAR(h.runner().lastRight(), 0.0, 1e-6);
  EXPECT_NE(h.telemetry().back().controller_fault_flags & ControllerFaultFutureImu, 0u);
}

TEST(RateControllerCoreTest, FalloverAndActuatorFaultCommandZero) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();
  ConfigPid::rate_P = 1.0;
  ConfigPid::angle_P = 100.0;
  ConfigPid::pitch_max_deg = 10.0;

  RateControllerHarness h;
  h.setImu(1.0 * M_PI / 180.0, 0.0, 2500);
  h.tick(1.0 / 400.0, 2500);
  ASSERT_GT(std::abs(h.runner().lastLeft()), 1.0);
  h.setImu(24.0 * M_PI / 180.0, 0.0, 5000);
  h.tick(1.0 / 400.0, 5000);
  EXPECT_EQ(h.telemetry().back().controller_fault_flags & ControllerFaultFallover, 0u);

  h.setImu(26.0 * M_PI / 180.0, 0.0, 7500);
  h.tick(1.0 / 400.0, 7500);
  EXPECT_NEAR(h.runner().lastLeft(), 0.0, 1e-6);
  EXPECT_NE(h.telemetry().back().controller_fault_flags & ControllerFaultFallover, 0u);
  EXPECT_EQ(h.telemetry().back().controller_saturation_flags, ControllerSaturationNone);

  h.setImu(-24.0 * M_PI / 180.0, 0.0, 10'000);
  h.tick(1.0 / 400.0, 10'000);
  EXPECT_NEAR(h.runner().lastLeft(), 0.0, 1e-6);
  EXPECT_NE(h.telemetry().back().controller_fault_flags & ControllerFaultFallover, 0u);

  h.setImu(1.0 * M_PI / 180.0, 40.0 * M_PI / 180.0, 12'500);
  h.tick(1.0 / 400.0, 12'500);
  EXPECT_NEAR(h.runner().lastLeft(), 0.0, 1e-6);
  EXPECT_NE(h.telemetry().back().controller_fault_flags & ControllerFaultFallover, 0u);

  h.setImu(1.0 * M_PI / 180.0, 0.0, 15'000);
  h.tick(1.0 / 400.0, 15'000);
  EXPECT_EQ(h.telemetry().back().controller_fault_flags & ControllerFaultFallover, 0u);

  h.setActuatorFault(true);
  h.tick(1.0 / 400.0, 17'500);
  EXPECT_NEAR(h.runner().lastLeft(), 0.0, 1e-6);
  EXPECT_NE(h.telemetry().back().controller_fault_flags & ControllerFaultActuator, 0u);
}

TEST(RateControllerCoreTest, FalloverRecoveryRequiresLiveForwardCommand) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();
  ConfigPid::rate_P = 0.20;
  ConfigPid::angle_P = 32.0;

  for (const double pitch_deg : {26.0, -26.0, 67.0, -67.0}) {
    RateControllerHarness h;
    h.setJoystick(0.0, 0.0);
    h.setImu(pitch_deg * M_PI / 180.0, 0.0, 2500);
    h.tick(1.0 / 400.0, 2500);
    EXPECT_NEAR(h.runner().lastLeft(), 0.0, 1e-6) << pitch_deg;
    EXPECT_NE(h.telemetry().back().controller_fault_flags & ControllerFaultFallover, 0u)
        << pitch_deg;
  }

  for (const auto command : {JoyCmd{0.0, 1.0}, JoyCmd{Config::deadzone * 0.5, 0.0}}) {
    RateControllerHarness h;
    h.setJoystick(command.forward, command.turn);
    h.setImu(67.0 * M_PI / 180.0, 0.0, 2500);
    h.tick(1.0 / 400.0, 2500);
    EXPECT_NEAR(h.runner().lastLeft(), 0.0, 1e-6);
    EXPECT_NE(h.telemetry().back().controller_fault_flags & ControllerFaultFallover, 0u);
  }
}

TEST(RateControllerCoreTest, ForwardCommandEnablesSymmetricFalloverRecoveryAuthority) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();
  ConfigPid::rate_P = 0.20;
  ConfigPid::angle_P = 32.0;

  const auto recovery_output = [](double direction) {
    RateControllerHarness h;
    h.setJoystick(direction * 0.10, 0.0);
    h.setImu(direction * 67.0 * M_PI / 180.0, 0.0, 2500);
    h.tick(1.0 / 400.0, 2500);
    EXPECT_EQ(h.telemetry().back().controller_fault_flags & ControllerFaultFallover, 0u);
    return h.runner().lastLeft();
  };

  const double positive_output = recovery_output(1.0);
  const double negative_output = recovery_output(-1.0);
  EXPECT_GT(positive_output, 0.0);
  EXPECT_LT(negative_output, 0.0);
  EXPECT_NEAR(positive_output, -negative_output, 1e-3);
}

TEST(RateControllerCoreTest, FalloverRecoveryRequiresLowRateAndStopsOnRelease) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();
  ConfigPid::rate_P = 0.20;
  ConfigPid::angle_P = 32.0;

  RateControllerHarness moving;
  moving.setJoystick(0.10, 0.0);
  moving.setImu(67.0 * M_PI / 180.0, 31.0 * M_PI / 180.0, 2500);
  moving.tick(1.0 / 400.0, 2500);
  EXPECT_NEAR(moving.runner().lastLeft(), 0.0, 1e-6);
  EXPECT_NE(moving.telemetry().back().controller_fault_flags & ControllerFaultFallover, 0u);

  RateControllerHarness released;
  released.setJoystick(0.10, 0.0);
  released.setImu(67.0 * M_PI / 180.0, 0.0, 2500);
  released.tick(1.0 / 400.0, 2500);
  ASSERT_GT(std::abs(released.runner().lastLeft()), 1.0);

  released.setJoystick(0.0, 0.0);
  released.setImu(67.0 * M_PI / 180.0, 0.0, 5000);
  released.tick(1.0 / 400.0, 5000);
  EXPECT_NEAR(released.runner().lastLeft(), 0.0, 1e-6);
  EXPECT_NEAR(released.runner().lastRight(), 0.0, 1e-6);
  EXPECT_NE(released.telemetry().back().controller_fault_flags & ControllerFaultFallover, 0u);
}

TEST(RateControllerCoreTest, ImuAndActuatorFaultsOverrideFalloverRecoveryCommand) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();
  ConfigPid::rate_P = 0.20;
  ConfigPid::angle_P = 32.0;

  RateControllerHarness stale;
  stale.setJoystick(0.10, 0.0);
  stale.setImu(67.0 * M_PI / 180.0, 0.0, 2500);
  stale.tick(1.0 / 400.0, 40'000);
  EXPECT_NEAR(stale.runner().lastLeft(), 0.0, 1e-6);
  EXPECT_NE(stale.telemetry().back().controller_fault_flags & ControllerFaultStaleImu, 0u);

  RateControllerHarness actuator;
  actuator.setJoystick(0.10, 0.0);
  actuator.setActuatorFault(true);
  actuator.setImu(-67.0 * M_PI / 180.0, 0.0, 2500);
  actuator.tick(1.0 / 400.0, 2500);
  EXPECT_NEAR(actuator.runner().lastLeft(), 0.0, 1e-6);
  EXPECT_NE(actuator.telemetry().back().controller_fault_flags & ControllerFaultActuator, 0u);
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
  EXPECT_LT(h.telemetry().back().vel_p_term_deg, 0.0);
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
  EXPECT_LT(h.telemetry().back().vel_p_term_deg, 0.0);
  EXPECT_GT(std::abs(h.telemetry().back().pitch_sp_deg), 1e-3);
  EXPECT_LE(std::abs(h.telemetry().back().pitch_sp_deg), ConfigPid::pitch_max_deg + 0.1);
}

TEST(RateControllerCoreTest, NegativeResidualVelocityProducesPositiveCatchPitchRef) {
  RateControllerHarness h;
  h.setJoystick(0.0, 0.0);
  h.run_steps(40, 1.0 / 400.0);

  h.runner().setActualSpeedSps(-400.0);
  h.run_steps(80, 1.0 / 400.0);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_GT(h.telemetry().back().vel_error, 0.0);
  EXPECT_GT(h.telemetry().back().vel_p_term_deg, 0.0);
}

TEST(RateControllerCoreTest, VelocityFeedbackAffectsTelemetryWhenCommanded) {
  RateControllerHarness h;
  h.setJoystick(0.2, 0.0);
  h.run_steps(40, 1.0 / 400.0);

  constexpr double kMeasuredVelocitySps = 1234.0;
  h.runner().setActualSpeedSps(kMeasuredVelocitySps);
  h.run_steps(80, 1.0 / 400.0);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_NEAR(h.telemetry().back().target_vel_sps,
              ConfigPid::drive_max_sps * (0.2 - Config::deadzone) / (1.0 - Config::deadzone), 1e-3);
  EXPECT_NEAR(h.telemetry().back().vel_error,
              h.telemetry().back().target_vel_sps - kMeasuredVelocitySps, 1e-3);
  EXPECT_NE(h.telemetry().back().vel_p_term_deg, 0.0);
  EXPECT_NE(h.telemetry().back().pitch_sp_deg, 0.0);
}

TEST(ControlServiceTest, UsesZeroCompletedPulseVelocityWhenNoFeedbackExists) {
  ControlServiceHarness h;
  h.sendJoystick(0.2, 0.0);

  for (int i = 0; i < 200; ++i) {
    const uint64_t sim_time_us = static_cast<uint64_t>((i + 1) * 2500);
    h.step_with_imu(1.0 / 400.0, sim_time_us);
  }

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_GT(h.telemetry().back().target_velocity_sps, 0.0);
  EXPECT_NEAR(h.telemetry().back().measured_vel_sps, 0.0, 1e-6);
  EXPECT_GT(h.telemetry().back().vel_error, 0.0);
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
  EXPECT_NEAR(fused.pitch_rate_rad_s, raw.gyr[1], 1e-6);
  EXPECT_NEAR(fused.pitch_accel_rad_s2, 0.0, 1e-6);
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
  EXPECT_NEAR(h.telemetry().back().measured_vel_sps, 123.0, 5.0);
  EXPECT_NEAR(h.telemetry().back().vel_error,
              static_cast<double>(h.telemetry().back().target_velocity_sps) - 123.0,
              5.0);
  EXPECT_GT(h.telemetry().back().velocity_p_term_deg, 0.0);
  EXPECT_NEAR(h.telemetry().back().left_applied_sps, 200.0, 1e-3);
  EXPECT_NEAR(h.telemetry().back().right_applied_sps, 46.0, 1e-3);
  EXPECT_EQ(h.telemetry().back().left_actual_steps, 0);
  EXPECT_EQ(h.telemetry().back().right_actual_steps, 0);
}

TEST(ControlServiceTest, VelocityObserverUsesFirstOrderLowPass) {
  ControlServiceHarness h;
  h.sendJoystick(0.0, 0.0);
  h.sendMotorFeedback(0.0, 0.0, 0.0, 0, 0);

  h.step_with_imu(1.0 / 400.0, 2500);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_NEAR(h.telemetry().back().vel_error, 0.0, 1e-6);

  h.sendMotorFeedback(0.0, 0.0, 1000.0, 0, 0);
  h.step_with_imu(1.0 / 400.0, 5000);

  const double alpha = std::exp(-2.0 * M_PI * Config::fc_velocity_hz * (1.0 / 400.0));
  const double expected_velocity = (1.0 - alpha) * 1000.0;
  EXPECT_NEAR(h.telemetry().back().measured_vel_sps, expected_velocity, 1e-3);
  EXPECT_NEAR(h.telemetry().back().vel_error, 0.0, 1e-6);

  for (int i = 0; i < 6; ++i) {
    h.step_with_imu(1.0 / 400.0, static_cast<uint64_t>((i + 3) * 2500));
  }
  EXPECT_NEAR(h.telemetry().back().vel_error, -expected_velocity, 1e-3);
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
  EXPECT_GT(t.target_velocity_sps, 0.0);
  EXPECT_GT(t.velocity_p_term_deg, 0.0);
  EXPECT_NEAR(t.velocity_i_term_deg, 0.0, 1e-9);
  EXPECT_NE(t.rate_setpoint_dps, 0.0);
  EXPECT_NE(t.pitch_error_deg, 0.0);
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
  EXPECT_GE(h.feedback().back().update_dt_ms, 0.0);
  EXPECT_GE(h.feedback().back().feedback_age_ms, 0.0);
  EXPECT_EQ(h.feedback().back().left_actual_steps, expected.left_actual_steps);
  EXPECT_EQ(h.feedback().back().right_actual_steps, expected.right_actual_steps);
}

}  // namespace
