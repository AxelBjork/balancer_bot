#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "services/control/control_service.h"
#include "services/control/rate_controller_core.h"
#include "services/imu/imu_service.h"
#include "services/input/input_service.h"
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

  void advance(double dt_s) {
    actual_steps_ += actual_speed_sps_ * dt_s;
  }

  int64_t actualSteps() const {
    return static_cast<int64_t>(std::llround(actual_steps_));
  }

  void setActualSteps(double steps) {
    actual_steps_ = steps;
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
  double actual_steps_{0.0};
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
    runner_.advance(dt_s);
    core_.setMotorFeedback(runner_.actualSteps(), runner_.actualSteps(), actuator_fault_);
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
  ControlServiceHarness()
      : bus_(this, &ControlServiceHarness::dispatch), control_(bus_) {
  }

  void sendJoystick(double forward, double turn) {
    control_.on_message<MsgId::JoystickCommand>(
        ipc::JoystickCommandPayload{static_cast<float>(forward), static_cast<float>(turn)});
  }

  void sendPidOverride(uint32_t request_id, const ConfigPidValues& values) {
    ipc::PidConfigOverridePayload payload{};
    payload.request_id = request_id;
    payload.values = {values.rate_P,
                      values.rate_I,
                      values.rate_D,
                      values.rate_I_lim,
                      values.rate_FF,
                      values.drive_max_acceleration_mps2,
                      values.velocity_damping_per_s,
                      values.velocity_I,
                      values.velocity_I_limit_deg,
                      values.angle_P,
                      values.angle_D,
                      values.pitch_rate_max_sps,
                      values.drive_max_sps,
                      values.turn_max_sps,
                      values.balance_max_sps};
    control_.on_message<MsgId::PidConfigOverride>(payload);
  }

  void sendMotorFeedback(double left_command_sps, double right_command_sps, double measured_avg_sps,
                         int64_t left_actual_steps, int64_t right_actual_steps,
                         double update_dt_ms = 1000.0 / 400.0,
                         double left_slewed_sps = std::numeric_limits<double>::quiet_NaN(),
                         double right_slewed_sps = std::numeric_limits<double>::quiet_NaN(),
                         uint32_t actuator_saturation_flags = 0) {
    ipc::MotorFeedbackPayload payload{};
    payload.left_slewed_sps =
        std::isfinite(left_slewed_sps) ? left_slewed_sps : left_command_sps;
    payload.right_slewed_sps =
        std::isfinite(right_slewed_sps) ? right_slewed_sps : right_command_sps;
    payload.measured_avg_sps = measured_avg_sps;
    payload.update_dt_ms = update_dt_ms;
    payload.left_actual_steps = left_actual_steps;
    payload.right_actual_steps = right_actual_steps;
    payload.actuator_saturation_flags = actuator_saturation_flags;
    control_.on_message<MsgId::MotorFeedback>(payload);
  }

  void step_with_imu(double dt_s, uint64_t sim_time_us, double angle_rad = 0.0,
                     double pitch_rate_rad_s = 0.0, double raw_pitch_rate_rad_s = 0.0,
                     double pitch_accel_rad_s2 = 0.0, bool estimate_valid = true) {
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
    imu.estimate_valid = estimate_valid;
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
  const std::vector<ipc::PidConfigStatusPayload>& pid_status() const {
    return pid_status_;
  }

 private:
  static void dispatch(void* ctx, MsgId id, const void* payload) {
    auto* self = static_cast<ControlServiceHarness*>(ctx);
    if (id == MsgId::MotorTargets) {
      self->motor_targets_.push_back(unpack_payload<MsgId::MotorTargets>(payload));
    } else if (id == MsgId::SystemTelemetry) {
      self->telemetry_.push_back(unpack_payload<MsgId::SystemTelemetry>(payload));
    } else if (id == MsgId::PidConfigStatus) {
      self->pid_status_.push_back(unpack_payload<MsgId::PidConfigStatus>(payload));
    }
  }

  ipc::MessageBus bus_;
  sil::ControlService control_;
  std::vector<ipc::MotorTargetsPayload> motor_targets_;
  std::vector<ipc::SystemTelemetryPayload> telemetry_;
  std::vector<ipc::PidConfigStatusPayload> pid_status_;
};

class ImuServiceHarness {
 public:
  ImuServiceHarness() : bus_(this, &ImuServiceHarness::dispatch), imu_(bus_, false) {
  }

  void publish_raw(const ipc::ImuRawPayload& payload) {
    bus_.publish<MsgId::ImuRawData>(payload);
  }

  void publish_feedback(const ipc::MotorFeedbackPayload& payload) {
    bus_.publish<MsgId::MotorFeedback>(payload);
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
    imu.estimate_valid = true;
    bus_.publish<MsgId::ImuData>(imu);

    PhysicsTickPayload tick{};
    tick.dt_s = dt_s;
    tick.timestamp_us = sim_time_us;
    bus_.publish<MsgId::PhysicsTick>(tick);
  }

  void sendRawStep(double dt_s, uint64_t sim_time_us, const std::array<double, 3>& acc,
                   const std::array<double, 3>& gyr = {0.0, 0.0, 0.0}) {
    ipc::ImuRawPayload imu{};
    imu.acc = acc;
    imu.gyr = gyr;
    imu.timestamp_us = sim_time_us;
    bus_.publish<MsgId::ImuRawData>(imu);

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
  double drive_max_acceleration_mps2 = ConfigPid::drive_max_acceleration_mps2;
  double velocity_damping_per_s = ConfigPid::velocity_damping_per_s;
  double velocity_I = ConfigPid::velocity_I;
  double velocity_I_limit_deg = ConfigPid::velocity_I_limit_deg;
  double angle_P = ConfigPid::angle_P;
  double angle_D = ConfigPid::angle_D;
  double pitch_rate_max_sps = ConfigPid::pitch_rate_max_sps;
  double drive_max_sps = ConfigPid::drive_max_sps;
  double turn_max_sps = ConfigPid::turn_max_sps;
  double balance_max_sps = ConfigPid::balance_max_sps;
  bool controller_enabled = ConfigPid::controller_enabled;

  void restore() const {
    ConfigPid::rate_P = rate_P;
    ConfigPid::rate_I = rate_I;
    ConfigPid::rate_D = rate_D;
    ConfigPid::rate_I_lim = rate_I_lim;
    ConfigPid::rate_FF = rate_FF;
    ConfigPid::drive_max_acceleration_mps2 = drive_max_acceleration_mps2;
    ConfigPid::velocity_damping_per_s = velocity_damping_per_s;
    ConfigPid::velocity_I = velocity_I;
    ConfigPid::velocity_I_limit_deg = velocity_I_limit_deg;
    ConfigPid::angle_P = angle_P;
    ConfigPid::angle_D = angle_D;
    ConfigPid::pitch_rate_max_sps = pitch_rate_max_sps;
    ConfigPid::drive_max_sps = drive_max_sps;
    ConfigPid::turn_max_sps = turn_max_sps;
    ConfigPid::balance_max_sps = balance_max_sps;
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
  ConfigPid::drive_max_acceleration_mps2 = 0.0;
  ConfigPid::velocity_damping_per_s = 0.0;
  ConfigPid::velocity_I = 0.0;
  ConfigPid::velocity_I_limit_deg = 4.0;
  ConfigPid::angle_P = 0.0;
  ConfigPid::angle_D = 0.0;
  ConfigPid::pitch_rate_max_sps = 2000.0;
  ConfigPid::drive_max_sps = 1000.0;
  ConfigPid::turn_max_sps = 1000.0;
  ConfigPid::balance_max_sps = 12000.0;
}

struct ControllerSineFit {
  double amplitude{0.0};
  double phase_rad{0.0};
};

ControllerSineFit fit_controller_sine(const std::vector<double>& samples, double dt_s,
                                      double frequency_hz) {
  double sine_sum = 0.0;
  double cosine_sum = 0.0;
  for (size_t index = 0; index < samples.size(); ++index) {
    const double phase =
        2.0 * M_PI * frequency_hz * dt_s * static_cast<double>(index + 1);
    sine_sum += samples[index] * std::sin(phase);
    cosine_sum += samples[index] * std::cos(phase);
  }
  const double scale = 2.0 / static_cast<double>(samples.size());
  return {
      std::hypot(scale * sine_sum, scale * cosine_sum),
      std::atan2(scale * cosine_sum, scale * sine_sum),
  };
}

double controller_phase_lag_deg(const ControllerSineFit& source,
                                const ControllerSineFit& response) {
  double lag = std::remainder(source.phase_rad - response.phase_rad, 2.0 * M_PI);
  if (lag < 0.0) {
    lag += 2.0 * M_PI;
  }
  return lag * 180.0 / M_PI;
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
                    ("balancer_pid_v6_" + std::to_string(suffix) + ".conf");
  std::ofstream output(path);
  output << contents;
  output.close();
  return path;
}

const char* valid_pid_v6_config() {
  return R"(config_version = 6
rate_P = 0.25
rate_I = 0
rate_D = 0
rate_I_lim = 0.15
rate_FF = 0
drive_max_acceleration_mps2 = 1.5
velocity_damping_per_s = 2
velocity_I = 0.001
velocity_I_limit_deg = 4
angle_P = 12
angle_D = 0.25
pitch_rate_max_sps = 2000
drive_max_sps = 1200
turn_max_sps = 1600
balance_max_sps = 12000
)";
}

TEST(ConfigPidV6Test, LoadsCompleteStrictSchema) {
  ScopedConfigPidRestore restore;
  const auto path = write_temp_pid_config(valid_pid_v6_config());
  EXPECT_NO_THROW(ConfigPid::load(path.string()));
  EXPECT_DOUBLE_EQ(ConfigPid::drive_max_acceleration_mps2, 1.5);
  EXPECT_DOUBLE_EQ(ConfigPid::pitch_rate_max_sps, 2000.0);
  EXPECT_DOUBLE_EQ(ConfigPid::balance_max_sps, 12000.0);
  EXPECT_TRUE(ConfigPid::controller_enabled);
  std::filesystem::remove(path);
}

TEST(ConfigPidV6Test, OptionalControllerEnabledDisablesActuationMode) {
  ScopedConfigPidRestore restore;
  const auto path = write_temp_pid_config(std::string(valid_pid_v6_config()) + "controller_enabled = 0\n");
  ASSERT_NO_THROW(ConfigPid::load(path.string()));
  EXPECT_FALSE(ConfigPid::controller_enabled);
  std::filesystem::remove(path);
}

TEST(ConfigPidV6Test, RejectsUnknownDuplicateMissingNonFiniteAndInvalidValues) {
  const std::vector<std::string> invalid = {
      std::string(valid_pid_v6_config()) + "legacy_gain = 1\n",
      std::string(valid_pid_v6_config()) + "rate_P = 1\n",
      std::string(valid_pid_v6_config()) + "output_scale_sps = 3200\n",
      "config_version = 6\n",
      std::string(valid_pid_v6_config())
          .replace(std::string(valid_pid_v6_config()).find("rate_P = 0.25"), 13, "rate_P = nan"),
      std::string(valid_pid_v6_config())
          .replace(std::string(valid_pid_v6_config()).find("pitch_rate_max_sps = 2000"), 25,
                   "pitch_rate_max_sps = 0"),
      std::string(valid_pid_v6_config())
          .replace(std::string(valid_pid_v6_config()).find("pitch_rate_max_sps = 2000"), 25,
                   "pitch_rate_max_sps = -1"),
      std::string(valid_pid_v6_config()) + "pitch_max_deg = 10\n",
      std::string(valid_pid_v6_config()).replace(0, 18, "config_version = 5"),
  };
  for (const auto& contents : invalid) {
    const auto path = write_temp_pid_config(contents);
    EXPECT_THROW(ConfigPid::load(path.string()), std::runtime_error) << contents;
    std::filesystem::remove(path);
  }
}

TEST(ConfigPidV6Test, RejectsOldVersionBeforeNewSchemaKey) {
  const auto path = write_temp_pid_config(
      std::string(valid_pid_v6_config()).replace(0, 18, "config_version = 2") +
      "future_only_key = 1\n");
  try {
    ConfigPid::load(path.string());
    FAIL() << "Expected config version mismatch";
  } catch (const std::runtime_error& error) {
    EXPECT_STREQ(error.what(), "PID configuration version mismatch: expected 6, got 2");
  }
  std::filesystem::remove(path);
}

TEST(ConfigPidV6Test, SaveEmitsNewRateLimitAndNoOutputScale) {
  ScopedConfigPidRestore restore;
  ConfigPid::pitch_rate_max_sps = 1750.0;
  const auto path = write_temp_pid_config("");
  ASSERT_NO_THROW(ConfigPid::save(path.string()));

  std::ifstream input(path);
  const std::string contents((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  EXPECT_NE(contents.find("config_version       = 6"), std::string::npos);
  EXPECT_NE(contents.find("pitch_rate_max_sps   = 1750"), std::string::npos);
  EXPECT_EQ(contents.find("output_scale_sps"), std::string::npos);
  std::filesystem::remove(path);
}

TEST(ConfigPidV6Test, RuntimeNumericSnapshotValidatesAndAppliesAtomically) {
  ScopedConfigPidRestore restore;
  const ConfigPidValues original = ConfigPid::numeric_values();
  ConfigPidValues updated = original;
  updated.rate_P = 0.42;
  updated.drive_max_acceleration_mps2 = 2.25;
  updated.balance_max_sps = 9000.0;

  EXPECT_EQ(ConfigPid::validate_numeric(updated), ConfigPidValidationCode::Accepted);
  ASSERT_NO_THROW(ConfigPid::apply_numeric(updated));
  EXPECT_DOUBLE_EQ(ConfigPid::rate_P, 0.42);
  EXPECT_DOUBLE_EQ(ConfigPid::drive_max_acceleration_mps2, 2.25);
  EXPECT_DOUBLE_EQ(ConfigPid::balance_max_sps, 9000.0);

  const ConfigPidValues before_invalid = ConfigPid::numeric_values();
  ConfigPidValues invalid = updated;
  invalid.rate_I = -0.1;
  EXPECT_EQ(ConfigPid::validate_numeric(invalid), ConfigPidValidationCode::Negative);
  EXPECT_THROW(ConfigPid::apply_numeric(invalid), std::invalid_argument);
  const ConfigPidValues after_invalid = ConfigPid::numeric_values();
  EXPECT_DOUBLE_EQ(after_invalid.rate_P, before_invalid.rate_P);
  EXPECT_DOUBLE_EQ(after_invalid.rate_I, before_invalid.rate_I);
  EXPECT_DOUBLE_EQ(after_invalid.balance_max_sps, before_invalid.balance_max_sps);
}

TEST(ControlServicePidConfigTest, PublishesStatusAndRejectsAtomically) {
  ScopedConfigPidRestore restore;
  ControlServiceHarness harness;
  ConfigPidValues updated = ConfigPid::numeric_values();
  updated.rate_P = 0.37;
  harness.sendPidOverride(17, updated);

  ASSERT_EQ(harness.pid_status().size(), 1u);
  EXPECT_EQ(harness.pid_status().back().request_id, 17u);
  EXPECT_EQ(harness.pid_status().back().accepted, 1u);
  EXPECT_DOUBLE_EQ(ConfigPid::rate_P, 0.37);

  ConfigPidValues invalid = updated;
  invalid.turn_max_sps = 12001.0;
  harness.sendPidOverride(18, invalid);
  ASSERT_EQ(harness.pid_status().size(), 2u);
  EXPECT_EQ(harness.pid_status().back().request_id, 18u);
  EXPECT_EQ(harness.pid_status().back().accepted, 0u);
  EXPECT_EQ(harness.pid_status().back().result_code,
            static_cast<uint8_t>(ConfigPidValidationCode::OutOfRange));
  EXPECT_DOUBLE_EQ(ConfigPid::rate_P, 0.37);
}

TEST(RateControllerCoreTest, RuntimePidGenerationRefreshesAndClampsTrim) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();
  ConfigPid::velocity_I = 0.01;
  ConfigPid::velocity_I_limit_deg = 4.0;

  RateControllerHarness harness;
  harness.setJoystick(0.0, 0.0);
  harness.runner().setActualSpeedSps(100.0);
  harness.run_steps(800, 1.0 / 400.0);
  ASSERT_GT(std::abs(harness.telemetry().back().vel_i_term_deg), 0.25);

  ConfigPidValues updated = ConfigPid::numeric_values();
  updated.velocity_I_limit_deg = 0.25;
  ASSERT_NO_THROW(ConfigPid::apply_numeric(updated));
  harness.run_steps(1, 1.0 / 400.0);
  EXPECT_LE(std::abs(harness.telemetry().back().vel_i_term_deg), 0.25 + 1e-9);
}

TEST(InputServiceTest, ExternalCommandFallsBackButXboxPriorityClearsIt) {
  sil::JoystickInputArbiter arbiter;
  const auto start = std::chrono::steady_clock::now();
  const ipc::JoystickCommandPayload external{0.4, -0.2};

  arbiter.accept_external(external, false, start);
  EXPECT_DOUBLE_EQ(arbiter.resolve(false, {0.0, 0.0}, start + std::chrono::milliseconds(1)).forward,
                   0.4);

  arbiter.accept_external(external, true, start + std::chrono::milliseconds(2));
  const JoyCmd xbox{0.1, 0.3};
  const JoyCmd selected = arbiter.resolve(true, xbox, start + std::chrono::milliseconds(3));
  EXPECT_DOUBLE_EQ(selected.forward, xbox.forward);
  EXPECT_DOUBLE_EQ(selected.turn, xbox.turn);

  const JoyCmd after_xbox = arbiter.resolve(false, {0.0, 0.0}, start + std::chrono::milliseconds(4));
  EXPECT_DOUBLE_EQ(after_xbox.forward, 0.0);
  EXPECT_DOUBLE_EQ(after_xbox.turn, 0.0);

  arbiter.accept_external(external, false, start + std::chrono::milliseconds(5));
  EXPECT_DOUBLE_EQ(arbiter.resolve(false, {0.0, 0.0}, start + std::chrono::milliseconds(100)).turn,
                   -0.2);
  const JoyCmd expired = arbiter.resolve(false, {0.0, 0.0}, start + std::chrono::milliseconds(256));
  EXPECT_DOUBLE_EQ(expired.forward, 0.0);
  EXPECT_DOUBLE_EQ(expired.turn, 0.0);

  arbiter.accept_external({std::numeric_limits<double>::quiet_NaN(), 0.0}, false,
                          start + std::chrono::milliseconds(300));
  EXPECT_DOUBLE_EQ(arbiter.resolve(false, {0.0, 0.0}, start + std::chrono::milliseconds(301)).forward,
                   0.0);
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
  ConfigPid::pitch_rate_max_sps = 1500.0;

  RateControllerHarness h;
  h.setJoystick(0.0, 0.0);
  h.setImu(-10.0 * M_PI / 180.0, 0.0, 2500);
  h.tick(1.0 / 400.0, 2500);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_NEAR(h.telemetry().back().rate_sp_dps,
              ConfigPid::pitch_rate_max_sps * 360.0 / Config::steps_per_rev, 1e-6);
  EXPECT_NE(h.telemetry().back().controller_saturation_flags & ControllerSaturationRate, 0u);
  EXPECT_NEAR(h.runner().lastLeft(), -0.25 * ConfigPid::pitch_rate_max_sps * 2.0 * M_PI,
              1e-3);
  EXPECT_NEAR(h.runner().lastRight(), h.runner().lastLeft(), 1e-6);
}

TEST(RateControllerCoreTest, PitchRateSetpointLimitIsSymmetricAtMotorBoundary) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();
  ConfigPid::rate_P = 0.25;
  ConfigPid::angle_P = 100.0;
  ConfigPid::pitch_rate_max_sps = 1500.0;

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

TEST(RateControllerCoreTest, AccelerationOuterLoopRunsAtOneHundredHertzAndJerkLimitsForwardCommand) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();
  ConfigPid::drive_max_acceleration_mps2 = 1.5;
  ConfigPid::velocity_I = 0.001;

  RateControllerHarness h;
  h.setJoystick(1.0, 0.0);
  h.run_steps(3, 1.0 / 400.0);
  EXPECT_NEAR(h.telemetry().back().nominal_acceleration_mps2, 0.0, 1e-9);
  h.run_steps(1, 1.0 / 400.0);
  EXPECT_NEAR(h.telemetry().back().nominal_acceleration_mps2, 0.06, 1e-9);
  EXPECT_NEAR(h.telemetry().back().velocity_damping_acceleration_mps2, 0.0, 1e-9);
  EXPECT_NEAR(h.telemetry().back().com_trim_deg, 0.0, 1e-9);
  EXPECT_GT(h.telemetry().back().pitch_sp_deg, 0.0);
  EXPECT_NEAR(h.runner().lastLeft(), 0.0, 1e-6);
  EXPECT_NEAR(h.runner().lastRight(), 0.0, 1e-6);

  h.run_steps(96, 1.0 / 400.0);
  EXPECT_NEAR(h.telemetry().back().nominal_acceleration_mps2,
              ConfigPid::drive_max_acceleration_mps2, 1e-9);
  EXPECT_NEAR(h.runner().lastLeft(), 0.0, 1e-6);
  EXPECT_NEAR(h.runner().lastRight(), 0.0, 1e-6);
}

TEST(RateControllerCoreTest, OuterLoopUsesAccumulatedElapsedTimeUnderTickJitter) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();
  ConfigPid::drive_max_acceleration_mps2 = 1.5;

  RateControllerHarness h;
  h.setJoystick(1.0, 0.0);
  uint64_t time_us = 0;
  for (const double dt_s : {0.003, 0.002, 0.004}) {
    time_us += static_cast<uint64_t>(std::llround(dt_s * 1e6));
    h.setImu(0.0, 0.0, time_us);
    h.tick(dt_s, time_us);
  }
  EXPECT_NEAR(h.telemetry().back().nominal_acceleration_mps2, 0.0, 1e-12);

  constexpr double kFinalDtS = 0.003;
  time_us += static_cast<uint64_t>(std::llround(kFinalDtS * 1e6));
  h.setImu(0.0, 0.0, time_us);
  h.tick(kFinalDtS, time_us);
  EXPECT_NEAR(h.telemetry().back().nominal_acceleration_mps2, 6.0 * 0.012, 1e-12);
}

TEST(RateControllerCoreTest, NineHertzCompletedStepObserverPhaseRemainsBelowSixtyFiveDegrees) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();
  ConfigPid::velocity_damping_per_s = 8.0;
  constexpr double kControlDtS = 1.0 / 400.0;
  constexpr double kOuterDtS = 1.0 / 100.0;
  constexpr double kFrequencyHz = 9.0;
  constexpr double kAmplitudeSps = 3000.0;
  constexpr int kTicks = 2000;
  constexpr int kWarmupTicks = 400;

  RateControllerHarness h;
  std::vector<double> requested_velocity;
  std::vector<double> observed_velocity;
  std::vector<double> damping;
  for (int tick = 0; tick < kTicks; ++tick) {
    const uint64_t time_us = static_cast<uint64_t>((tick + 1) * 2500);
    const double time_s = static_cast<double>(time_us) / 1e6;
    const double velocity = kAmplitudeSps * std::sin(2.0 * M_PI * kFrequencyHz * time_s);
    h.runner().setActualSpeedSps(velocity);
    h.setImu(0.0, 0.0, time_us);
    h.tick(kControlDtS, time_us);
    if (tick >= kWarmupTicks && (tick + 1) % 4 == 0) {
      requested_velocity.push_back(velocity);
      observed_velocity.push_back(h.telemetry().back().corrected_axle_velocity_sps);
      damping.push_back(h.telemetry().back().velocity_damping_acceleration_mps2);
    }
  }

  ASSERT_GT(requested_velocity.size(), 300U);
  const ControllerSineFit source_fit =
      fit_controller_sine(requested_velocity, kOuterDtS, kFrequencyHz);
  const ControllerSineFit response_fit =
      fit_controller_sine(observed_velocity, kOuterDtS, kFrequencyHz);
  EXPECT_GT(response_fit.amplitude, 100.0);
  EXPECT_LT(controller_phase_lag_deg(source_fit, response_fit), 65.0);
  for (size_t index = 0; index < damping.size(); ++index) {
    EXPECT_TRUE(std::isfinite(observed_velocity[index]));
    EXPECT_TRUE(std::isfinite(damping[index]));
    EXPECT_NEAR(damping[index],
                -ConfigPid::velocity_damping_per_s * Config::meters_per_step *
                    observed_velocity[index],
                1e-12);
  }
  const auto [minimum, maximum] = std::minmax_element(observed_velocity.begin(),
                                                      observed_velocity.end());
  EXPECT_GT(*maximum, 0.0);
  EXPECT_LT(*minimum, 0.0);
  EXPECT_NEAR(*maximum, -*minimum, 0.05 * response_fit.amplitude);
}

TEST(RateControllerCoreTest, AccelerationRequestBrakesThroughZeroBeforeReversing) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();
  ConfigPid::drive_max_acceleration_mps2 = 1.5;

  RateControllerHarness h;
  h.setJoystick(1.0, 0.0);
  h.run_steps(200, 1.0 / 400.0);
  ASSERT_NEAR(h.telemetry().back().nominal_acceleration_mps2,
              ConfigPid::drive_max_acceleration_mps2, 1e-9);

  h.setJoystick(-1.0, 0.0);
  h.run_steps(96, 1.0 / 400.0);
  EXPECT_NEAR(h.telemetry().back().nominal_acceleration_mps2, 0.06, 1e-9);
  h.run_steps(8, 1.0 / 400.0);
  EXPECT_NEAR(h.telemetry().back().nominal_acceleration_mps2, -0.06, 1e-9);
}

TEST(RateControllerCoreTest, PositiveAndNegativeAccelerationRequestsProduceSymmetricPitchReferences) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();
  ConfigPid::drive_max_acceleration_mps2 = 1.5;

  const auto run_direction = [](double direction) {
    RateControllerHarness h;
    h.setJoystick(direction, 0.0);
    h.runner().setActualSpeedSps(0.0);
    h.run_steps(200, 1.0 / 400.0);
    return h.telemetry().back();
  };

  const auto positive_telemetry = run_direction(1.0);
  const auto negative_telemetry = run_direction(-1.0);
  EXPECT_GT(positive_telemetry.nominal_acceleration_mps2, 0.0);
  EXPECT_LT(negative_telemetry.nominal_acceleration_mps2, 0.0);
  EXPECT_NEAR(positive_telemetry.velocity_damping_acceleration_mps2, 0.0, 1e-9);
  EXPECT_NEAR(negative_telemetry.velocity_damping_acceleration_mps2, 0.0, 1e-9);
  EXPECT_GT(positive_telemetry.pitch_sp_deg, 0.0);
  EXPECT_LT(negative_telemetry.pitch_sp_deg, 0.0);
  EXPECT_NEAR(positive_telemetry.pitch_sp_deg, -negative_telemetry.pitch_sp_deg, 1e-6);
}

TEST(RateControllerCoreTest, TargetAccelerationPitchFeedForwardMatchesPlantPolarity) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();
  ConfigPid::drive_max_acceleration_mps2 = 1.5;

  const auto run_direction = [](double direction) {
    RateControllerHarness h;
    h.setJoystick(direction, 0.0);
    h.runner().setActualSpeedSps(0.0);
    h.run_steps(8, 1.0 / 400.0);
    return h.telemetry().back();
  };

  const auto positive = run_direction(1.0);
  const auto negative = run_direction(-1.0);
  const double expected_pitch_deg = std::atan2(0.12, Config::g0) * 180.0 / M_PI;
  EXPECT_GT(positive.nominal_acceleration_mps2, 0.0);
  EXPECT_LT(negative.nominal_acceleration_mps2, 0.0);
  EXPECT_NEAR(positive.velocity_damping_acceleration_mps2, 0.0, 1e-9);
  EXPECT_NEAR(negative.velocity_damping_acceleration_mps2, 0.0, 1e-9);
  EXPECT_NEAR(positive.pitch_sp_deg, expected_pitch_deg, 1e-6);
  EXPECT_NEAR(negative.pitch_sp_deg, -expected_pitch_deg, 1e-6);
}

TEST(RateControllerCoreTest, CorrectedAxleVelocityRemovesPitchOnlyCompletedStepMotion) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();
  constexpr double kPitchRateRadS = 0.4;
  constexpr double kStepsPerRad = Config::steps_per_rev / (2.0 * M_PI);

  RateControllerHarness h;
  h.setJoystick(0.0, 0.0);
  // q = u - r theta: fixed axle motion therefore has qdot = -r thetadot.
  h.runner().setActualSpeedSps(-kStepsPerRad * kPitchRateRadS);
  for (int i = 0; i < 400; ++i) {
    const uint64_t time_us = static_cast<uint64_t>((i + 1) * 2500);
    const double pitch = kPitchRateRadS * static_cast<double>(time_us) / 1e6;
    h.setImu(pitch, kPitchRateRadS, time_us);
    h.tick(1.0 / 400.0, time_us);
  }

  const auto& t = h.telemetry().back();
  EXPECT_GT(std::abs(t.raw_completed_velocity_sps), 100.0);
  EXPECT_NEAR(t.corrected_axle_velocity_sps, 0.0, 5.0);
}

TEST(RateControllerCoreTest, CorrectedAxleVelocityUsesWrappedPitchDifference) {
  const double delta = rate_controller_detail::wrap_angle_delta(-358.0 * M_PI / 180.0);
  EXPECT_NEAR(delta, 2.0 * M_PI / 180.0, 1e-12);
  EXPECT_NEAR(rate_controller_detail::wrap_angle_delta(-delta), -delta, 1e-12);
}

TEST(RateControllerCoreTest, CorrectedVelocityUpdatesEveryFourTicksWithSymmetricMotion) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();

  const auto observe_step = [](double completed_steps) {
    RateControllerHarness h;
    h.run_steps(4, 1.0 / 400.0);
    h.runner().setActualSteps(completed_steps);
    h.run_steps(3, 1.0 / 400.0);
    EXPECT_NEAR(h.telemetry().back().corrected_axle_velocity_sps, 0.0, 1e-12);
    h.run_steps(1, 1.0 / 400.0);
    return h.telemetry().back().corrected_axle_velocity_sps;
  };

  const double positive = observe_step(40.0);
  const double negative = observe_step(-40.0);
  EXPECT_GT(positive, 0.0);
  EXPECT_LT(negative, 0.0);
  EXPECT_NEAR(positive, -negative, 1e-12);
}

TEST(RateControllerCoreTest, CorrectedVelocityRejectsCompletedStepCounterReset) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();

  RateControllerHarness h;
  h.run_steps(4, 1.0 / 400.0);
  h.runner().setActualSteps(40.0);
  h.run_steps(4, 1.0 / 400.0);
  ASSERT_GT(h.telemetry().back().corrected_axle_velocity_sps, 0.0);

  h.runner().setActualSteps(1'000'000.0);
  h.run_steps(4, 1.0 / 400.0);
  EXPECT_NEAR(h.telemetry().back().raw_completed_velocity_sps, 0.0, 1e-12);
  EXPECT_NEAR(h.telemetry().back().corrected_axle_velocity_sps, 0.0, 1e-12);
}

TEST(RateControllerCoreTest, AccelerationRequestIsJerkLimitedAndVelocityDampingBrakes) {
  ScopedConfigPidRestore restore;
  set_zeroed_gain_audit_config();
  ConfigPid::drive_max_acceleration_mps2 = 1.5;
  ConfigPid::velocity_damping_per_s = 2.0;

  RateControllerHarness h;
  h.setJoystick(1.0, 0.0);
  h.run_steps(8, 1.0 / 400.0);
  EXPECT_NEAR(h.telemetry().back().nominal_acceleration_mps2, 0.12, 1e-9);
  h.runner().setActualSpeedSps(800.0);
  h.run_steps(8, 1.0 / 400.0);
  EXPECT_LT(h.telemetry().back().velocity_damping_acceleration_mps2, 0.0);
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
  EXPECT_NEAR(h.telemetry().back().vel_i_term_deg, learned_trim_deg, 0.01);
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
  ConfigPid::drive_max_acceleration_mps2 = 1.5;
  ConfigPid::velocity_damping_per_s = 200.0;
  ConfigPid::velocity_I = 1.0;
  ConfigPid::velocity_I_limit_deg = 2.0;

  RateControllerHarness h;
  h.setJoystick(1.0, 0.0);
  h.runner().setActualSpeedSps(4000.0);
  h.run_steps(400, 1.0 / 400.0);
  const double expected_limit_deg =
      std::atan2(ConfigPid::drive_max_acceleration_mps2, Config::g0) * 180.0 / M_PI;
  EXPECT_NEAR(std::abs(h.telemetry().back().pitch_sp_deg), expected_limit_deg, 1e-6);
  EXPECT_NE(h.telemetry().back().controller_saturation_flags & ControllerSaturationPitch, 0u);
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
  const double drive_limit_deg =
      std::atan2(ConfigPid::drive_max_acceleration_mps2, Config::g0) * 180.0 / M_PI;
  EXPECT_LE(std::abs(h.telemetry().back().pitch_sp_deg), drive_limit_deg + 0.1);
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

TEST(RateControllerCoreTest, CorrectedVelocityDampingAffectsTelemetryWhenCommanded) {
  RateControllerHarness h;
  h.setJoystick(0.2, 0.0);
  h.run_steps(40, 1.0 / 400.0);

  constexpr double kMeasuredVelocitySps = 1234.0;
  h.runner().setActualSpeedSps(kMeasuredVelocitySps);
  h.run_steps(80, 1.0 / 400.0);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_NEAR(h.telemetry().back().nominal_acceleration_mps2, 0.0, 1e-9);
  EXPECT_GT(h.telemetry().back().corrected_axle_velocity_sps, 0.0);
  EXPECT_LT(h.telemetry().back().velocity_damping_acceleration_mps2, 0.0);
  EXPECT_NE(h.telemetry().back().pitch_sp_deg, 0.0);
}

TEST(ControlServiceTest, UsesZeroCorrectedAxleVelocityWhenNoFeedbackExists) {
  ControlServiceHarness h;
  h.sendJoystick(0.2, 0.0);

  for (int i = 0; i < 200; ++i) {
    const uint64_t sim_time_us = static_cast<uint64_t>((i + 1) * 2500);
    h.step_with_imu(1.0 / 400.0, sim_time_us);
  }

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_GT(h.telemetry().back().nominal_acceleration_mps2, 0.0);
  EXPECT_NEAR(h.telemetry().back().corrected_axle_velocity_sps, 0.0, 1e-6);
}

TEST(ImuServiceTest, ConditionsRawImuAndPreservesRawVectors) {
  ImuServiceHarness h;
  const double pitch_rad = 4.0 * M_PI / 180.0;
  ipc::ImuRawPayload raw{};
  raw.gyr = {0.1, 0.0, 0.3};
  raw.acc = accel_for_pitch(pitch_rad);

  for (int sample = 0;
       sample <= static_cast<int>(4.0 * Config::sampling_hz); ++sample) {
    raw.timestamp_us = 123456 + static_cast<uint64_t>(sample * 1200);
    h.publish_raw(raw);
  }

  ASSERT_EQ(h.fused_samples().size(),
            static_cast<size_t>(4.0 * Config::sampling_hz) + 1u);
  EXPECT_DOUBLE_EQ(h.fused_samples().front().pitch_rad, 0.0);
  const auto& fused = h.fused_samples().back();
  EXPECT_TRUE(fused.estimate_valid);
  EXPECT_NEAR(fused.pitch_rad, pitch_rad, 1e-4);
  EXPECT_NEAR(fused.pitch_rate_rad_s, 0.0, 3e-6);
  EXPECT_NEAR(fused.pitch_accel_rad_s2, 0.0, 2e-5);
  EXPECT_EQ(fused.acc, raw.acc);
  EXPECT_EQ(fused.gyr, raw.gyr);
  EXPECT_EQ(fused.timestamp_us, raw.timestamp_us);
}

TEST(ControlServiceTest, InvalidEstimateImmediatelyClearsImuAndSafeZeroesNextTick) {
  ControlServiceHarness h;
  h.step_with_imu(1.0 / 400.0, 2500, 5.0 * M_PI / 180.0);
  ASSERT_FALSE(h.motor_targets().empty());
  ASSERT_NE(h.motor_targets().back().left_sps, 0.0);

  h.step_with_imu(1.0 / 400.0, 5000, 0.0, 0.0, 0.0, 0.0, false);
  ASSERT_GE(h.motor_targets().size(), 2u);
  EXPECT_EQ(h.motor_targets().back().left_sps, 0.0);
  EXPECT_EQ(h.motor_targets().back().right_sps, 0.0);
  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_NE(h.telemetry().back().controller_fault_flags & ControllerFaultNoImu, 0u);
}

TEST(ImuServiceTest, MotorFeedbackCannotAffectPureImuPitch) {
  ImuServiceHarness without_feedback;
  ImuServiceHarness with_feedback;
  ipc::ImuRawPayload raw{};
  raw.acc = accel_for_pitch(6.0 * M_PI / 180.0);

  for (int sample = 0; sample <= 10; ++sample) {
    raw.timestamp_us = 2000 + static_cast<uint64_t>(sample * 2000);
    ipc::MotorFeedbackPayload feedback{};
    feedback.left_actual_steps = sample * sample * 1000;
    feedback.right_actual_steps = sample * sample * 1000;
    with_feedback.publish_feedback(feedback);
    without_feedback.publish_raw(raw);
    with_feedback.publish_raw(raw);
  }

  ASSERT_FALSE(without_feedback.fused_samples().empty());
  ASSERT_EQ(without_feedback.fused_samples().size(), with_feedback.fused_samples().size());
  const auto& expected = without_feedback.fused_samples().back();
  const auto& actual = with_feedback.fused_samples().back();
  ASSERT_TRUE(expected.estimate_valid);
  ASSERT_TRUE(actual.estimate_valid);
  EXPECT_EQ(actual.pitch_rad, expected.pitch_rad);
  EXPECT_EQ(actual.pitch_rate_rad_s, expected.pitch_rate_rad_s);
  EXPECT_EQ(actual.pitch_accel_rad_s2, expected.pitch_accel_rad_s2);
}

TEST(ControlServiceTest, KeepsMotorSpeedProxyOutOfCorrectedVelocity) {
  ControlServiceHarness h;
  h.sendJoystick(0.2, 0.0);

  for (int i = 0; i < 200; ++i) {
    const uint64_t sim_time_us = static_cast<uint64_t>((i + 1) * 2500);
    h.sendMotorFeedback(200.0, 46.0, 123.0, 0, 0);
    h.step_with_imu(1.0 / 400.0, sim_time_us);
  }

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_NEAR(h.telemetry().back().corrected_axle_velocity_sps, 0.0, 1e-6);
  EXPECT_NEAR(h.telemetry().back().raw_completed_velocity_sps, 0.0, 1e-6);
  EXPECT_GT(h.telemetry().back().nominal_acceleration_mps2, 0.0);
  EXPECT_EQ(h.telemetry().back().left_actual_steps, 0);
  EXPECT_EQ(h.telemetry().back().right_actual_steps, 0);
}

TEST(ControlServiceTest, VelocityObserverSeedsWithoutCounterSpike) {
  ControlServiceHarness h;
  h.sendJoystick(0.0, 0.0);
  h.sendMotorFeedback(0.0, 0.0, 0.0, 0, 0);

  h.step_with_imu(1.0 / 400.0, 2500);

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_NEAR(h.telemetry().back().corrected_axle_velocity_sps, 0.0, 1e-6);

  for (int i = 1; i <= 8; ++i) {
    h.sendMotorFeedback(0.0, 0.0, 1000.0, 0, 0);
    h.step_with_imu(1.0 / 400.0, static_cast<uint64_t>((i + 1) * 2500));
  }
  EXPECT_NEAR(h.telemetry().back().corrected_axle_velocity_sps, 0.0, 1e-6);
}

TEST(ControlServiceTest, VelocityObserverRejectsDifferentialTurningSteps) {
  ControlServiceHarness h;
  h.sendJoystick(0.0, 0.5);
  h.sendMotorFeedback(0.0, 0.0, 0.0, 0, 0);
  for (int i = 1; i <= 8; ++i) {
    h.step_with_imu(1.0 / 400.0, static_cast<uint64_t>(i * 2500));
  }
  h.sendMotorFeedback(0.0, 0.0, 0.0, 100, -100);
  for (int i = 9; i <= 16; ++i) {
    h.step_with_imu(1.0 / 400.0, static_cast<uint64_t>(i * 2500));
  }

  ASSERT_FALSE(h.telemetry().empty());
  EXPECT_NEAR(h.telemetry().back().raw_completed_velocity_sps, 0.0, 1e-6);
  EXPECT_NEAR(h.telemetry().back().corrected_axle_velocity_sps, 0.0, 1e-6);
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
    h.sendMotorFeedback(140.0, 100.0, 120.0, 123, 87, 2.5, 160.0, 120.0,
                        ActuatorSaturationLeftSlew);
    h.step_with_imu(1.0 / 400.0, sim_time_us, 2.0 * M_PI / 180.0, 0.1);
  }

  ASSERT_FALSE(h.telemetry().empty());
  const auto& t = h.telemetry().back();
  EXPECT_GT(t.nominal_acceleration_mps2, 0.0);
  EXPECT_NEAR(t.com_trim_deg, 0.0, 1e-9);
  EXPECT_NE(t.rate_setpoint_dps, 0.0);
  EXPECT_NE(t.pitch_error_deg, 0.0);
  EXPECT_NEAR(t.left_slewed_sps, 160.0, 1e-3);
  EXPECT_NEAR(t.right_slewed_sps, 120.0, 1e-3);
  EXPECT_EQ(t.actuator_saturation_flags, ActuatorSaturationLeftSlew);
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
  EXPECT_NEAR(h.feedback().back().left_slewed_sps, expected.left_slewed_sps, 1e-3);
  EXPECT_NEAR(h.feedback().back().right_slewed_sps, expected.right_slewed_sps, 1e-3);
  EXPECT_GE(h.feedback().back().update_dt_ms, 0.0);
  EXPECT_GE(h.feedback().back().feedback_age_ms, 0.0);
  EXPECT_EQ(h.feedback().back().left_actual_steps, expected.left_actual_steps);
  EXPECT_EQ(h.feedback().back().right_actual_steps, expected.right_actual_steps);
  EXPECT_EQ(h.feedback().back().actuator_saturation_flags,
            expected.actuator_saturation_flags);
}

}  // namespace
