#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <random>
#include <utility>

#include "messages/balancer_msgs.h"
#include "publisher.h"
#include "services/control/control_service.h"
#include "services/imu/imu_service.h"
#include "services/main/config.h"
#include "services/motor/motor_service.h"
#include "services/time/time_service.h"
#include "simulator/simulator_runner.h"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kControlDtS = 1.0 / 400.0;
constexpr double kImuPeriodUs = 1e6 / 833.0;
constexpr double kAccelQuantum = 0.000598205;
constexpr double kGyroQuantum = 0.000152716;

struct DisturbanceValue {
  double force_n{0.0};
  double com_bias_rad{0.0};
};

DisturbanceValue disturbanceAt(const SimulatorScenario& scenario, double time_s) {
  DisturbanceValue total{};
  for (const auto& disturbance : scenario.disturbances) {
    if (time_s < disturbance.start_s) continue;
    const double end_s = disturbance.start_s + disturbance.duration_s;
    switch (disturbance.kind) {
      case SimulatorDisturbanceKind::Step:
        if (disturbance.duration_s > 0.0 && time_s < end_s) {
          total.force_n += disturbance.force_n;
          total.com_bias_rad += disturbance.com_bias_rad;
        }
        break;
      case SimulatorDisturbanceKind::Ramp:
        if (disturbance.duration_s > 0.0 && time_s < end_s) {
          const double progress =
              std::clamp((time_s - disturbance.start_s) / disturbance.duration_s, 0.0, 1.0);
          total.force_n +=
              disturbance.force_n + (disturbance.force_n_end - disturbance.force_n) * progress;
          total.com_bias_rad +=
              disturbance.com_bias_rad +
              (disturbance.com_bias_rad_end - disturbance.com_bias_rad) * progress;
        }
        break;
      case SimulatorDisturbanceKind::HoldBias:
        if (disturbance.duration_s <= 0.0 || time_s < end_s) {
          total.force_n += disturbance.force_n;
          total.com_bias_rad += disturbance.com_bias_rad;
        }
        break;
    }
  }
  return total;
}

ipc::JoystickCommandPayload joystickAt(const SimulatorScenario& scenario, double time_s) {
  ipc::JoystickCommandPayload command{};
  for (const auto& segment : scenario.joy_segments) {
    if (time_s < segment.start_s) continue;
    const bool active = segment.duration_s <= 0.0 || time_s < segment.start_s + segment.duration_s;
    if (!active) continue;
    double progress = 0.0;
    if (segment.duration_s > 0.0) {
      progress = std::clamp((time_s - segment.start_s) / segment.duration_s, 0.0, 1.0);
    }
    command.forward =
        std::clamp(segment.forward + (segment.forward_end - segment.forward) * progress, -1.0, 1.0);
    command.turn =
        std::clamp(segment.turn + (segment.turn_end - segment.turn) * progress, -1.0, 1.0);
  }
  return command;
}

double rawPitchDeg(const std::array<double, 3>& acc) {
  return std::atan2(-acc[0], std::sqrt(acc[1] * acc[1] + acc[2] * acc[2])) * 180.0 / kPi;
}

class SimulationWaveBackend final : public WaveFrameBackend {
 public:
  int queueFrame(unsigned, unsigned, bool) override {
    return next_id_++;
  }
  void deleteFrame(int) override {
  }
  void stop() override {
  }

 private:
  int next_id_{1};
};

struct EngineObserver {
  using Subscribes =
      ipc::MsgList<MsgId::SystemTelemetry, MsgId::MotorFeedback, MsgId::MotorTargets>;

  template <MsgId Id>
  void on_message(const typename MessageTraits<Id>::Payload&) {
  }

  ipc::SystemTelemetryPayload telemetry{};
  ipc::MotorFeedbackPayload feedback{};
  ipc::MotorTargetsPayload targets{};
  bool have_telemetry{false};
  bool have_feedback{false};
};

template <>
void EngineObserver::on_message<MsgId::SystemTelemetry>(
    const ipc::SystemTelemetryPayload& payload) {
  telemetry = payload;
  have_telemetry = true;
}

template <>
void EngineObserver::on_message<MsgId::MotorFeedback>(const ipc::MotorFeedbackPayload& payload) {
  feedback = payload;
  have_feedback = true;
}

template <>
void EngineObserver::on_message<MsgId::MotorTargets>(const ipc::MotorTargetsPayload& payload) {
  targets = payload;
}

struct EngineServices {
  SimulationWaveBackend wave_backend;
  Stepper left;
  Stepper right;
  MotorRunner motors;
  sil::ImuService imu;
  sil::ControlService control;
  sil::MotorService motor;
  sil::TimeService time;
  EngineObserver observer;

  explicit EngineServices(ipc::MessageBus& bus)
      : left(1, Stepper::Pins{12, 19, 13}, false, false),
        right(1, Stepper::Pins{4, 18, 24}, false, false),
        motors(left, right, Config::control_hz, Config::motor_slew_sps_per_s, &wave_backend),
        imu(bus, false),
        control(bus),
        motor(bus, &motors),
        time(bus, kControlDtS) {
  }
};

struct EnginePipeline {
  ipc::MessageBus bus;
  EngineServices services;

  explicit EnginePipeline(const SimulatorScenario& scenario)
      : bus(this, &EnginePipeline::dispatch), services(bus) {
    (void)scenario;
  }

  static void dispatch(void* context, MsgId id, const void* payload) {
    auto* self = static_cast<EnginePipeline*>(context);
    ipc::dispatch_to_service(self->services.imu, id, payload);
    ipc::dispatch_to_service(self->services.motor, id, payload);

    if (id == MsgId::MotorFeedback) {
      const auto feedback = unpack_payload<MsgId::MotorFeedback>(payload);
      self->services.control.on_message<MsgId::MotorFeedback>(feedback);
      self->services.observer.on_message<MsgId::MotorFeedback>(feedback);
      return;
    }

    ipc::dispatch_to_service(self->services.control, id, payload);
    ipc::dispatch_to_service(self->services.observer, id, payload);
  }
};

}  // namespace

struct SimulatorEngine::Impl {
  struct DelayedImu {
    uint64_t release_us{0};
    ipc::ImuRawPayload payload{};
  };

  SimulatorScenario scenario;
  BalancerSimulator simulator;
  EnginePipeline pipeline;
  std::mt19937 rng;
  std::normal_distribution<double> accel_noise;
  std::normal_distribution<double> gyro_noise;
  std::uniform_real_distribution<double> jitter_unit{-1.0, 1.0};
  std::uniform_real_distribution<double> loss_unit{0.0, 1.0};
  std::deque<DelayedImu> delayed_imu;
  ipc::ImuRawPayload latest_raw{};
  ipc::JoystickCommandPayload external_joystick{};
  bool use_external_joystick{false};
  double plant_time_us{0.0};
  double next_imu_time_us{kImuPeriodUs};
  uint64_t last_imu_timestamp_us{0};

  explicit Impl(const SimulatorScenario& input)
      : scenario(input),
        simulator(makeSimulatorConfig(input)),
        pipeline(input),
        rng(input.imu_noise_seed),
        accel_noise(0.0, input.accel_noise_std_mps2 > 0.0 ? input.accel_noise_std_mps2 : 1.0),
        gyro_noise(0.0, input.gyro_noise_std_rad_s > 0.0 ? input.gyro_noise_std_rad_s : 1.0) {
  }

  static BalancerSimulator::Config makeSimulatorConfig(const SimulatorScenario& input) {
    BalancerSimulator::Config config;
    config.com_angle_offset_rad = input.com_angle_offset_rad;
    config.initial_pitch_deg = input.initial_pitch_deg;
    config.physics_profile = input.physics_profile;
    config.physics_override = input.physics_override;
    config.mass_scale = input.mass_scale;
    config.com_height_scale = input.com_height_scale;
    config.inertia_scale = input.inertia_scale;
    return config;
  }

  void setDisturbance(double time_s) {
    const auto disturbance = disturbanceAt(scenario, time_s);
    simulator.set_external_force_n(disturbance.force_n);
    simulator.set_external_com_bias_rad(disturbance.com_bias_rad);
  }

  void publishReleasedImu(uint64_t now_us) {
    while (!delayed_imu.empty() && delayed_imu.front().release_us <= now_us) {
      latest_raw = delayed_imu.front().payload;
      pipeline.bus.publish<MsgId::ImuRawData>(latest_raw);
      delayed_imu.pop_front();
    }
  }

  void sampleImu(uint64_t base_timestamp_us) {
    if (scenario.imu_sample_loss_rate > 0.0 &&
        loss_unit(rng) < std::clamp(scenario.imu_sample_loss_rate, 0.0, 1.0)) {
      return;
    }

    auto raw = simulator.make_raw_imu_payload(base_timestamp_us);
    for (size_t axis = 0; axis < 3; ++axis) {
      raw.acc[axis] += scenario.accel_bias_mps2[axis];
      raw.gyr[axis] += scenario.gyro_bias_rad_s[axis];
      if (scenario.accel_noise_std_mps2 > 0.0) raw.acc[axis] += accel_noise(rng);
      if (scenario.gyro_noise_std_rad_s > 0.0) raw.gyr[axis] += gyro_noise(rng);
      raw.acc[axis] = std::round(raw.acc[axis] / kAccelQuantum) * kAccelQuantum;
      raw.gyr[axis] = std::round(raw.gyr[axis] / kGyroQuantum) * kGyroQuantum;
    }

    const double jitter = jitter_unit(rng) * std::max(0.0, scenario.imu_timestamp_jitter_us);
    const int64_t jittered =
        static_cast<int64_t>(base_timestamp_us) + static_cast<int64_t>(std::llround(jitter));
    raw.timestamp_us =
        std::max(last_imu_timestamp_us + 1, static_cast<uint64_t>(std::max<int64_t>(1, jittered)));
    last_imu_timestamp_us = raw.timestamp_us;
    const uint64_t lag_us =
        static_cast<uint64_t>(std::llround(std::max(0.0, scenario.imu_pitch_lag_s) * 1e6));
    delayed_imu.push_back(DelayedImu{base_timestamp_us + lag_us, raw});
  }

  void advancePlantTo(double target_time_us) {
    while (next_imu_time_us <= target_time_us + 1e-9) {
      const double dt_s = std::max(0.0, next_imu_time_us - plant_time_us) / 1e6;
      setDisturbance(plant_time_us / 1e6);
      const auto emitted = pipeline.services.motors.getScheduledStepPosition(
          static_cast<uint64_t>(std::llround(next_imu_time_us)));
      simulator.set_emitted_steps(emitted.left_steps, emitted.right_steps);
      simulator.step(dt_s);
      plant_time_us = next_imu_time_us;
      const uint64_t timestamp_us = static_cast<uint64_t>(std::llround(next_imu_time_us));
      sampleImu(timestamp_us);
      publishReleasedImu(timestamp_us);
      next_imu_time_us += kImuPeriodUs;
    }

    const double remaining_s = std::max(0.0, target_time_us - plant_time_us) / 1e6;
    setDisturbance(plant_time_us / 1e6);
    const auto emitted = pipeline.services.motors.getScheduledStepPosition(
        static_cast<uint64_t>(std::llround(target_time_us)));
    simulator.set_emitted_steps(emitted.left_steps, emitted.right_steps);
    simulator.step(remaining_s);
    plant_time_us = target_time_us;
    publishReleasedImu(static_cast<uint64_t>(std::llround(target_time_us)));
  }

  SimulatorTimelineRow step() {
    const uint64_t start_us = pipeline.services.time.current_time_us();
    const uint64_t end_us = start_us + static_cast<uint64_t>(std::llround(kControlDtS * 1e6));
    advancePlantTo(static_cast<double>(end_us));

    const auto joystick = use_external_joystick
                              ? external_joystick
                              : joystickAt(scenario, static_cast<double>(end_us) / 1e6);
    pipeline.bus.publish<MsgId::JoystickCommand>(joystick);
    pipeline.services.time.advance(kControlDtS);

    if (pipeline.services.observer.have_feedback) {
      const auto& feedback = pipeline.services.observer.feedback;
      simulator.set_motor_targets(feedback.left_applied_sps, feedback.right_applied_sps);
    }
    return timelineRow();
  }

  SimulatorTimelineRow timelineRow() const {
    const auto& state = simulator.state();
    const auto& diagnostics = simulator.diagnostics();
    const auto& observer = pipeline.services.observer;
    SimulatorTimelineRow row{};
    row.sim_time_s = static_cast<double>(pipeline.services.time.current_time_us()) / 1e6;
    row.plant_pitch_deg = state.pitch * 180.0 / kPi;
    row.plant_pitch_rate_dps = state.pitch_rate * 180.0 / kPi;
    row.plant_position = state.position;
    row.plant_velocity = state.velocity;
    row.target_wheel_velocity = diagnostics.target_wheel_velocity;
    row.actual_wheel_velocity = diagnostics.actual_wheel_velocity;
    row.velocity_error = diagnostics.velocity_error;
    row.f_cmd = diagnostics.f_cmd;
    row.f_app = diagnostics.f_app;
    row.external_force_n = diagnostics.external_force_n;
    row.external_com_bias_rad = diagnostics.external_com_bias_rad;
    row.x_ddot = diagnostics.x_ddot;
    row.theta_ddot = diagnostics.theta_ddot;
    row.force_saturated = diagnostics.command_saturated ? 1.0 : 0.0;
    row.phase_error_steps = diagnostics.phase_error_steps;
    row.missed_steps = diagnostics.missed_steps;
    row.traction_limit_n = diagnostics.traction_limit_n;
    row.motor_force_limit_n = diagnostics.motor_force_limit_n;
    row.seed = scenario.imu_noise_seed;
    row.mass_scale = scenario.mass_scale;
    row.com_height_scale = scenario.com_height_scale;
    row.inertia_scale = scenario.inertia_scale;
    row.imu_timestamp_us = latest_raw.timestamp_us;
    row.raw_acc_pitch_deg = rawPitchDeg(latest_raw.acc);
    row.gyro_pitch_rate_dps = latest_raw.gyr[1] * 180.0 / kPi;
    row.left_sps = observer.targets.left_sps;
    row.right_sps = observer.targets.right_sps;
    row.command_saturated =
        (std::abs(row.left_sps) >= 0.99 * kMaxSps || std::abs(row.right_sps) >= 0.99 * kMaxSps)
            ? 1.0
            : 0.0;

    if (observer.have_feedback) {
      row.left_applied_sps = observer.feedback.left_applied_sps;
      row.right_applied_sps = observer.feedback.right_applied_sps;
      row.left_actual_steps = static_cast<double>(observer.feedback.left_actual_steps);
      row.right_actual_steps = static_cast<double>(observer.feedback.right_actual_steps);
      row.motor_update_dt_ms = observer.feedback.update_dt_ms;
      row.motor_feedback_age_ms = observer.feedback.feedback_age_ms;
    }
    if (observer.have_telemetry) {
      const auto& telemetry = observer.telemetry;
      row.pitch_deg = telemetry.pitch_deg;
      row.pitch_rate_dps = telemetry.pitch_rate_dps;
      row.filtered_pitch_rate_dps = telemetry.filtered_pitch_rate_dps;
      row.fused_pitch_deg = telemetry.fused_pitch_deg;
      row.pitch_sp_deg = telemetry.pitch_sp_deg;
      row.u_sps = telemetry.u_sps;
      row.turn_sps = telemetry.turn_sps;
      row.vel_error = telemetry.vel_error;
      row.target_velocity_sps = telemetry.target_velocity_sps;
      row.velocity_p_term_deg = telemetry.velocity_p_term_deg;
      row.velocity_i_term_deg = telemetry.velocity_i_term_deg;
      row.pitch_error_deg = telemetry.pitch_error_deg;
      row.rate_setpoint_dps = telemetry.rate_setpoint_dps;
      row.rate_error_dps = telemetry.rate_error_dps;
      row.command_saturated = telemetry.command_saturated;
      row.controller_fault_flags = telemetry.controller_fault_flags;
      row.controller_saturation_flags = telemetry.controller_saturation_flags;
      row.actuator_fault = telemetry.actuator_fault;
      row.measured_vel_sps = telemetry.measured_vel_sps;
    }
    return row;
  }
};

SimulatorEngine::SimulatorEngine(const SimulatorScenario& scenario)
    : impl_(std::make_unique<Impl>(scenario)) {
}
SimulatorEngine::~SimulatorEngine() = default;
SimulatorEngine::SimulatorEngine(SimulatorEngine&&) noexcept = default;
SimulatorEngine& SimulatorEngine::operator=(SimulatorEngine&&) noexcept = default;

SimulatorTimelineRow SimulatorEngine::step() {
  return impl_->step();
}

void SimulatorEngine::set_joystick(double forward, double turn) {
  impl_->external_joystick.forward = std::clamp(forward, -1.0, 1.0);
  impl_->external_joystick.turn = std::clamp(turn, -1.0, 1.0);
  impl_->use_external_joystick = true;
}

uint64_t SimulatorEngine::current_time_us() const {
  return impl_->pipeline.services.time.current_time_us();
}

const BalancerSimulator& SimulatorEngine::simulator() const {
  return impl_->simulator;
}
const SimulatorPhysics& SimulatorEngine::physics() const {
  return impl_->simulator.physics();
}
