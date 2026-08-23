#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <random>
#include <utility>

#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>

#include "messages/balancer_msgs.h"
#include "publisher.h"
#include "services/control/control_service.h"
#include "services/imu/imu_service.h"
#include "services/main/config.h"
#include "services/motor/motor_service.h"
#include "services/time/time_service.h"
#include "simulator/simulator_scheduler.h"
#include "simulator/simulator_runner.h"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kControlDtS = 1.0 / 400.0;
constexpr double kImuPeriodUs = 1e6 / Config::sampling_hz;
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

struct PitchAuthorityValue {
  bool active{false};
  double target_deg{0.0};
  double com_trim_deg{0.0};
};

PitchAuthorityValue pitchAuthorityAt(const SimulatorScenario& scenario, double time_s) {
  if (scenario.pitch_authority_segments.empty()) return {};

  // Keep the diagnostic path active for the entire run, including between
  // pulses, so the ordinary drive/velocity/COM paths cannot contaminate a
  // direct-target measurement. The first segment chooses the frozen trim;
  // later segments may explicitly repeat or change it for a documented test.
  PitchAuthorityValue value{true, 0.0, scenario.pitch_authority_segments.front().com_trim_deg};
  for (const auto& segment : scenario.pitch_authority_segments) {
    if (time_s < segment.start_s) continue;
    value.com_trim_deg = segment.com_trim_deg;
    if (segment.duration_s > 0.0 && time_s < segment.start_s + segment.duration_s) {
      value.target_deg = segment.target_deg;
    }
  }
  return value;
}

double rawPitchDeg(const std::array<double, 3>& acc) {
  return std::atan2(-acc[0], -acc[2]) * 180.0 / kPi;
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
  ipc::MotorFeedbackPayload controller_feedback{};
  ipc::MotorTargetsPayload targets{};
  bool have_telemetry{false};
  bool have_feedback{false};
  bool have_controller_feedback{false};
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

  explicit EngineServices(ipc::MessageBus& bus, bool legacy_reference_sensor_path)
      : left(1, Stepper::Pins{12, 19, 13}, false, false),
        right(1, Stepper::Pins{4, 18, 24}, false, false),
        motors(left, right, Config::control_hz, Config::motor_slew_sps_per_s, &wave_backend),
        imu(bus, false,
            legacy_reference_sensor_path
                ? sil::ImuService::EstimatorPath::LegacySimulationReference
                : sil::ImuService::EstimatorPath::Production),
        control(bus),
        motor(bus, &motors),
        time(bus, kControlDtS) {
  }
};

struct EnginePipeline {
  ipc::MessageBus bus;
  // The hardware ISM330 applies gyro LPF1 before the samples reach the
  // estimator.  Keep that sensor-side stage in the SIL input path as well;
  // the estimator itself intentionally contains no generic gyro LPF.
  math::LowPassFilter2p<float> chip_gyro_lpf1;
  bool use_chip_gyro_lpf1{false};
  EngineServices services;

  // This residual exists only for explicit simulator estimator-perturbation
  // scenarios.  Nominal operation delivers the emitted STEP counters to the
  // controller unchanged.
  double synthetic_residual_velocity_mps = 0.0;
  double rolling_residual_steps = 0.0;
  double emitted_step_velocity_sps = 0.0;
  double controller_feedback_velocity_sps = 0.0;
  double previous_emitted_common_steps = 0.0;
  double previous_controller_common_steps = 0.0;
  bool have_velocity_sample = false;

  explicit EnginePipeline(double initial_fused_pitch_deg, PhysicsProfile physics_profile)
      : bus(this, &EnginePipeline::dispatch),
        chip_gyro_lpf1(static_cast<float>(Config::sampling_hz),
                       static_cast<float>(Config::imu_gyro_lpf1_bandwidth_hz)),
        use_chip_gyro_lpf1(physics_profile == PhysicsProfile::StepperPhaseElectrical),
        services(bus, !use_chip_gyro_lpf1) {
    if (std::abs(initial_fused_pitch_deg) > 1e-12) {
      services.imu.setInitialPitchForSimulation(initial_fused_pitch_deg * M_PI / 180.0);
    }
    // Seed the production observer at the physical zero-time wheel position.
    // Initial chassis velocity is then represented by the first subsequent
    // rolling displacement rather than being silently invisible to control.
    ipc::MotorFeedbackPayload initial_feedback{};
    services.control.on_message<MsgId::MotorFeedback>(initial_feedback);
  }

  void applyChipGyroFilter(ipc::ImuRawPayload& raw) {
    if (!use_chip_gyro_lpf1) return;
    for (double& value : raw.gyr) {
      value = chip_gyro_lpf1.apply(static_cast<float>(value));
    }
  }

  void setSyntheticResidualVelocity(double velocity_mps) {
    synthetic_residual_velocity_mps = velocity_mps;
  }

  double emitted_step_velocity_sps_for_telemetry() const {
    return emitted_step_velocity_sps;
  }

  double synthetic_residual_velocity_sps_for_telemetry() const {
    return synthetic_residual_velocity_mps / BalancerSimulator::HardwareNominal::meters_per_step;
  }

  double controller_feedback_velocity_sps_for_telemetry() const {
    return controller_feedback_velocity_sps;
  }

  static void dispatch(void* context, MsgId id, const void* payload) {
    auto* self = static_cast<EnginePipeline*>(context);
    if (id == MsgId::ImuRawData) {
      // The bus owns the published payload, so condition a copy before
      // delivering it to the same estimator used by the hardware service.
      // Timeline raw gyro fields intentionally remain the pre-chip-filter
      // simulator sensor values for diagnostics.
      auto raw = unpack_payload<MsgId::ImuRawData>(payload);
      self->applyChipGyroFilter(raw);
      ipc::dispatch_to_service(self->services.imu, id, &raw);
    } else {
      ipc::dispatch_to_service(self->services.imu, id, payload);
    }
    ipc::dispatch_to_service(self->services.motor, id, payload);

    if (id == MsgId::MotorFeedback) {
      const auto feedback = unpack_payload<MsgId::MotorFeedback>(payload);
      // The observer records the physical feedback. Only the controller-facing
      // copy is delivered after the plant/actuator has published it, so the
      // telemetry remains a ground truth.
      self->services.observer.on_message<MsgId::MotorFeedback>(feedback);
      const double dt_s = std::max(1e-6, feedback.update_dt_ms) / 1000.0;
      self->rolling_residual_steps += self->synthetic_residual_velocity_mps * dt_s /
                                      BalancerSimulator::HardwareNominal::meters_per_step;
      auto ground_feedback = feedback;
      ground_feedback.left_actual_steps +=
          static_cast<int64_t>(std::llround(self->rolling_residual_steps));
      ground_feedback.right_actual_steps +=
          static_cast<int64_t>(std::llround(self->rolling_residual_steps));
      const double emitted_common_steps =
          0.5 * (static_cast<double>(feedback.left_actual_steps) +
                 static_cast<double>(feedback.right_actual_steps));
      const double controller_common_steps =
          0.5 * (static_cast<double>(ground_feedback.left_actual_steps) +
                 static_cast<double>(ground_feedback.right_actual_steps));
      if (dt_s > 0.0 && self->have_velocity_sample) {
        self->emitted_step_velocity_sps =
            (emitted_common_steps - self->previous_emitted_common_steps) / dt_s;
        self->controller_feedback_velocity_sps =
            (controller_common_steps - self->previous_controller_common_steps) / dt_s;
      }
      self->previous_emitted_common_steps = emitted_common_steps;
      self->previous_controller_common_steps = controller_common_steps;
      self->have_velocity_sample = dt_s > 0.0;
      self->services.observer.controller_feedback = ground_feedback;
      self->services.observer.have_controller_feedback = true;
      self->services.control.on_message<MsgId::MotorFeedback>(ground_feedback);
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
  std::deque<std::pair<double, double>> velocity_history;
  SimulatorTimeScheduler scheduler{static_cast<uint64_t>(std::llround(kControlDtS * 1e6))};
  std::vector<ScheduledStepEvent> scheduled_step_events;
  std::vector<SimulatorEvent> events_at_time;
  ipc::ImuRawPayload latest_raw{};
  ipc::JoystickCommandPayload external_joystick{};
  bool use_external_joystick{false};
  double plant_time_us{0.0};
  double next_imu_time_us{kImuPeriodUs};
  uint64_t last_imu_timestamp_us{0};

  explicit Impl(const SimulatorScenario& input)
      : scenario(input),
        simulator(makeSimulatorConfig(input)),
        pipeline(input.initial_fused_pitch_deg, input.physics_profile),
        rng(input.imu_noise_seed),
        accel_noise(0.0, input.accel_noise_std_mps2 > 0.0 ? input.accel_noise_std_mps2 : 1.0),
        gyro_noise(0.0, input.gyro_noise_std_rad_s > 0.0 ? input.gyro_noise_std_rad_s : 1.0) {
    // Publish the ordinary time-zero sensor sample before integrating the
    // plant, matching the hardware filter's first-sample initialization.
    latest_raw = simulator.make_raw_imu_payload(0);
    pipeline.bus.publish<MsgId::ImuRawData>(latest_raw);
    velocity_history.emplace_back(0.0, simulator.state().velocity);
  }

  static BalancerSimulator::Config makeSimulatorConfig(const SimulatorScenario& input) {
    BalancerSimulator::Config config;
    config.com_angle_offset_rad = input.com_angle_offset_rad;
    config.initial_pitch_deg = input.initial_pitch_deg;
    config.initial_pitch_rate_dps = input.initial_pitch_rate_dps;
    config.initial_velocity_mps = input.initial_velocity_mps;
    config.physics_profile = input.physics_profile;
    config.physics_override = input.physics_override;
    config.total_mass_scale = input.total_mass_scale;
    config.pitch_inertia_scale = input.pitch_inertia_scale;
    config.first_mass_moment_scale = input.first_mass_moment_scale;
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
      if (axis == 1 && scenario.gyro_pitch_disturbance_frequency_hz > 0.0 &&
          scenario.gyro_pitch_disturbance_amplitude_rad_s != 0.0) {
        const double time_s = static_cast<double>(base_timestamp_us) / 1e6;
        raw.gyr[axis] += scenario.gyro_pitch_disturbance_amplitude_rad_s *
                         std::sin(2.0 * M_PI * scenario.gyro_pitch_disturbance_frequency_hz *
                                  time_s);
      }
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

  void recordVelocitySample(double time_s) {
    velocity_history.emplace_back(time_s, simulator.state().velocity);
    while (velocity_history.size() > 20001) velocity_history.pop_front();
  }

  double delayedPhysicalVelocity(double time_s) const {
    const double requested_time =
        time_s - std::max(0.0, scenario.velocity_estimator_latency_s);
    if (velocity_history.empty() || requested_time <= velocity_history.front().first) {
      return velocity_history.empty() ? simulator.state().velocity : velocity_history.front().second;
    }
    // The history is time-ordered.  A forward scan made long-horizon
    // scenarios quadratic: with zero latency every controller tick walked
    // through the entire 20,001-sample history before reaching the newest
    // sample.  lower_bound preserves the same interpolation while making
    // delayed velocity lookup logarithmic.
    const auto next = std::lower_bound(
        velocity_history.begin(), velocity_history.end(), requested_time,
        [](const std::pair<double, double>& sample, double value) {
          return sample.first < value;
        });
    if (next == velocity_history.end()) return velocity_history.back().second;
    if (next == velocity_history.begin()) return next->second;
    const auto previous = next - 1;
    const double span = next->first - previous->first;
    if (span <= 0.0) return next->second;
    const double alpha = std::clamp((requested_time - previous->first) / span, 0.0, 1.0);
    return previous->second + (next->second - previous->second) * alpha;
  }

  void setControllerVelocityEstimate(double time_s) {
    const bool perturbation_requested =
        std::abs(scenario.velocity_estimator_bias_mps) > 1e-12 ||
        std::abs(scenario.velocity_estimator_bias_drift_mps_per_s) > 1e-12 ||
        std::abs(scenario.velocity_estimator_scale - 1.0) > 1e-12 ||
        scenario.velocity_estimator_latency_s > 1e-12;
    if (!perturbation_requested) {
      pipeline.setSyntheticResidualVelocity(0.0);
      return;
    }
    const double estimated_velocity_mps =
        scenario.velocity_estimator_bias_mps +
        scenario.velocity_estimator_bias_drift_mps_per_s * time_s +
        scenario.velocity_estimator_scale * delayedPhysicalVelocity(time_s);
    const double emitted_velocity_mps =
        pipeline.emitted_step_velocity_sps_for_telemetry() * Config::meters_per_step;
    pipeline.setSyntheticResidualVelocity(estimated_velocity_mps - emitted_velocity_mps);
  }

  void scheduleScenarioBoundaries(uint64_t start_us, uint64_t target_us) {
    for (const auto& disturbance : scenario.disturbances) {
      const auto schedule_boundary = [&](double time_s) {
        if (time_s <= 0.0) return;
        const uint64_t timestamp_us =
            static_cast<uint64_t>(std::llround(time_s * 1e6));
        if (timestamp_us > start_us && timestamp_us <= target_us) {
          scheduler.schedule(
              SimulatorEvent{timestamp_us, SimulatorEventKind::Scenario, 0, 0});
        }
      };
      schedule_boundary(disturbance.start_s);
      if (disturbance.duration_s > 0.0) {
        schedule_boundary(disturbance.start_s + disturbance.duration_s);
      }
    }
  }

  void scheduleImuSamples(uint64_t start_us, uint64_t target_us) {
    while (true) {
      const uint64_t timestamp_us =
          static_cast<uint64_t>(std::llround(next_imu_time_us));
      if (timestamp_us > target_us) {
        break;
      }
      // Preserve the legacy behavior when rounding places an IMU sample
      // exactly on the current plant boundary.  The old polling loop emitted
      // that zero-duration sample before continuing; the event scheduler
      // intentionally rejects events at or before its current timestamp.
      if (timestamp_us <= start_us) {
        sampleImu(timestamp_us);
        publishReleasedImu(timestamp_us);
      } else {
        scheduler.schedule(
            SimulatorEvent{timestamp_us, SimulatorEventKind::ImuSample, 0, 0});
      }
      next_imu_time_us += kImuPeriodUs;
    }
  }

  void advancePlantTo(double target_time_us) {
    const uint64_t start_us = static_cast<uint64_t>(std::llround(plant_time_us));
    const uint64_t target_us = static_cast<uint64_t>(std::llround(target_time_us));
    if (target_us <= start_us) {
      return;
    }

    scheduler.clear_pending_events();
    scheduler.advance_to(start_us);
    const bool uses_step_events = !simulator.physics().direct_force;
    if (uses_step_events) {
      pipeline.services.motors.getScheduledStepEvents(start_us, target_us,
                                                      scheduled_step_events);
      scheduler.schedule_step_events(scheduled_step_events);
    }
    scheduleScenarioBoundaries(start_us, target_us);
    scheduleImuSamples(start_us, target_us);

    // The cumulative position at the interval start includes all events that
    // were retired by MotorRunner before this interval.  Subsequent updates
    // are then driven only by individual timestamped STEP events.
    double emitted_left_steps = 0.0;
    double emitted_right_steps = 0.0;
    std::int64_t emitted_left_step_index = 0;
    std::int64_t emitted_right_step_index = 0;
    const bool use_integer_step_positions =
        uses_step_events && !simulator.physics().stepper_phase_continuous_field;
    if (uses_step_events) {
      auto emitted = pipeline.services.motors.getScheduledStepPosition(start_us);
      emitted_left_steps = emitted.left_steps;
      emitted_right_steps = emitted.right_steps;
      if (use_integer_step_positions) {
        emitted_left_step_index = static_cast<std::int64_t>(emitted_left_steps);
        emitted_right_step_index = static_cast<std::int64_t>(emitted_right_steps);
        simulator.set_emitted_motor_step_indices(emitted_left_step_index,
                                                 emitted_right_step_index);
      } else {
        simulator.set_emitted_motor_steps(emitted_left_steps, emitted_right_steps);
      }
    }

    while (scheduler.current_time_us() < target_us) {
      const uint64_t current_us = scheduler.current_time_us();
      const uint64_t next_us = scheduler.next_event_time_us(target_us).value_or(target_us);
      if (next_us > current_us) {
        setDisturbance(static_cast<double>(current_us) / 1e6);
        if (uses_step_events) {
          if (use_integer_step_positions) {
            simulator.set_emitted_motor_step_indices(emitted_left_step_index,
                                                     emitted_right_step_index);
          } else {
            simulator.set_emitted_motor_steps(emitted_left_steps, emitted_right_steps);
          }
        }
        simulator.step(static_cast<double>(next_us - current_us) / 1e6);
        scheduler.advance_to(next_us);
        plant_time_us = static_cast<double>(next_us);
      }

      scheduler.pop_events_at(scheduler.current_time_us(), events_at_time);
      for (const auto& event : events_at_time) {
        if (event.kind == SimulatorEventKind::Step) {
          if (use_integer_step_positions) {
            emitted_left_step_index += event.left_step_delta;
            emitted_right_step_index += event.right_step_delta;
            simulator.set_emitted_motor_step_indices(emitted_left_step_index,
                                                     emitted_right_step_index);
          } else {
            emitted_left_steps += static_cast<double>(event.left_step_delta);
            emitted_right_steps += static_cast<double>(event.right_step_delta);
            simulator.set_emitted_motor_steps(emitted_left_steps, emitted_right_steps);
          }
        } else if (event.kind == SimulatorEventKind::Scenario) {
          setDisturbance(static_cast<double>(scheduler.current_time_us()) / 1e6);
        } else if (event.kind == SimulatorEventKind::ImuSample) {
          const uint64_t timestamp_us = scheduler.current_time_us();
          sampleImu(timestamp_us);
          publishReleasedImu(timestamp_us);
        }
      }
    }

    plant_time_us = static_cast<double>(target_us);
    publishReleasedImu(target_us);
  }

  SimulatorTimelineRow step() {
    const uint64_t end_us = scheduler.next_controller_time_us();
    setControllerVelocityEstimate(plant_time_us / 1e6);
    advancePlantTo(static_cast<double>(end_us));
    // Keep the residual current for the next control interval.  It represents
    // the difference between physical ground displacement and the commanded
    // wheel-step displacement in the no-slip reference.
    recordVelocitySample(plant_time_us / 1e6);
    setControllerVelocityEstimate(plant_time_us / 1e6);
    const auto joystick = use_external_joystick
                              ? external_joystick
                              : joystickAt(scenario, static_cast<double>(end_us) / 1e6);
    pipeline.bus.publish<MsgId::JoystickCommand>(joystick);
    const auto pitch_authority =
        pitchAuthorityAt(scenario, static_cast<double>(end_us) / 1e6);
    const double end_time_s = static_cast<double>(end_us) / 1e6;
    const bool refresh_dropout =
        scenario.pitch_authority_refresh_dropout_duration_s > 0.0 &&
        end_time_s >= scenario.pitch_authority_refresh_dropout_start_s &&
        end_time_s < scenario.pitch_authority_refresh_dropout_start_s +
                          scenario.pitch_authority_refresh_dropout_duration_s;
    if (pitch_authority.active && !refresh_dropout) {
      // Refresh before the deterministic PhysicsTick. The controller's
      // watchdog is intentionally short-lived so a future hardware client
      // cannot leave a diagnostic target latched after disconnect.
      ipc::PitchAuthorityDiagnosticCommandPayload command{};
      command.request_id = static_cast<uint32_t>(end_us / 1000U);
      command.active = 1;
      command.target_deg = pitch_authority.target_deg;
      command.com_trim_deg = pitch_authority.com_trim_deg;
      command.duration_s = 0.050;
      pipeline.bus.publish<MsgId::PitchAuthorityDiagnosticCommand>(command);
    }
    pipeline.services.time.advance(kControlDtS);
    scheduler.controller_sample_processed(end_us);

    if (pipeline.services.observer.have_feedback) {
      const auto& feedback = pipeline.services.observer.feedback;
      // The continuous post-slew command drives the motor's field-speed term.
      // Exact pulse-frame quantization already enters the plant independently
      // through set_emitted_motor_steps(), so using the active-frame average here
      // would count the same quantization twice.
      simulator.set_motor_targets(feedback.left_slewed_sps, feedback.right_slewed_sps);
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
    row.desired_drive_force = diagnostics.desired_drive_force;
    row.limited_drive_force = diagnostics.limited_drive_force;
    row.applied_drive_force = diagnostics.applied_drive_force;
    row.desired_tire_force = diagnostics.desired_tire_force;
    row.external_force_n = diagnostics.external_force_n;
    row.external_com_bias_rad = diagnostics.external_com_bias_rad;
    row.x_ddot = diagnostics.x_ddot;
    row.theta_ddot = diagnostics.theta_ddot;
    row.force_saturated = diagnostics.command_saturated ? 1.0 : 0.0;
    row.phase_saturated = diagnostics.phase_saturated ? 1.0 : 0.0;
    row.motor_force_saturated = diagnostics.motor_force_saturated ? 1.0 : 0.0;
    row.traction_saturated = diagnostics.traction_saturated ? 1.0 : 0.0;
    row.phase_error_steps = diagnostics.phase_error_steps;
    row.missed_steps = diagnostics.missed_steps;
    row.traction_limit_n = diagnostics.traction_limit_n;
    row.motor_force_limit_n = diagnostics.motor_force_limit_n;
    row.stepper_commanded_microsteps_left = diagnostics.stepper_commanded_microsteps_left;
    row.stepper_commanded_microsteps_right = diagnostics.stepper_commanded_microsteps_right;
    row.stepper_commanded_field_angle_left_rad =
        diagnostics.stepper_commanded_field_angle_left_rad;
    row.stepper_commanded_field_angle_right_rad =
        diagnostics.stepper_commanded_field_angle_right_rad;
    row.stepper_commanded_field_electrical_angle_left_rad =
        diagnostics.stepper_commanded_field_electrical_angle_left_rad;
    row.stepper_commanded_field_electrical_angle_right_rad =
        diagnostics.stepper_commanded_field_electrical_angle_right_rad;
    row.stepper_commanded_field_velocity_mps =
        diagnostics.stepper_commanded_field_velocity_mps;
    row.stepper_actual_relative_angle_left_rad =
        diagnostics.stepper_actual_relative_angle_left_rad;
    row.stepper_actual_relative_angle_right_rad =
        diagnostics.stepper_actual_relative_angle_right_rad;
    row.stepper_actual_rotor_electrical_angle_left_rad =
        diagnostics.stepper_actual_rotor_electrical_angle_left_rad;
    row.stepper_actual_rotor_electrical_angle_right_rad =
        diagnostics.stepper_actual_rotor_electrical_angle_right_rad;
    row.stepper_electrical_phase_error_left_rad =
        diagnostics.stepper_electrical_phase_error_left_rad;
    row.stepper_electrical_phase_error_right_rad =
        diagnostics.stepper_electrical_phase_error_right_rad;
    row.stepper_torque_left_nm = diagnostics.stepper_torque_left_nm;
    row.stepper_torque_right_nm = diagnostics.stepper_torque_right_nm;
    row.stepper_summed_torque_nm = diagnostics.stepper_summed_torque_nm;
    row.stepper_actual_wheel_velocity_mps = diagnostics.stepper_actual_wheel_velocity_mps;
    row.stepper_chassis_velocity_mps = diagnostics.stepper_chassis_velocity_mps;
    row.stepper_current_ref_a_left = diagnostics.stepper_current_ref_a_left;
    row.stepper_current_ref_b_left = diagnostics.stepper_current_ref_b_left;
    row.stepper_current_a_left = diagnostics.stepper_current_a_left;
    row.stepper_current_b_left = diagnostics.stepper_current_b_left;
    row.stepper_phase_voltage_a_left = diagnostics.stepper_phase_voltage_a_left;
    row.stepper_phase_voltage_b_left = diagnostics.stepper_phase_voltage_b_left;
    row.stepper_back_emf_a_left = diagnostics.stepper_back_emf_a_left;
    row.stepper_back_emf_b_left = diagnostics.stepper_back_emf_b_left;
    row.stepper_electrical_power_left_w = diagnostics.stepper_electrical_power_left_w;
    row.stepper_mechanical_power_left_w = diagnostics.stepper_mechanical_power_left_w;
    row.stepper_resistive_loss_left_w = diagnostics.stepper_resistive_loss_left_w;
    row.stepper_magnetic_energy_left_j = diagnostics.stepper_magnetic_energy_left_j;
    row.stepper_current_ref_a_right = diagnostics.stepper_current_ref_a_right;
    row.stepper_current_ref_b_right = diagnostics.stepper_current_ref_b_right;
    row.stepper_current_a_right = diagnostics.stepper_current_a_right;
    row.stepper_current_b_right = diagnostics.stepper_current_b_right;
    row.stepper_phase_voltage_a_right = diagnostics.stepper_phase_voltage_a_right;
    row.stepper_phase_voltage_b_right = diagnostics.stepper_phase_voltage_b_right;
    row.stepper_back_emf_a_right = diagnostics.stepper_back_emf_a_right;
    row.stepper_back_emf_b_right = diagnostics.stepper_back_emf_b_right;
    row.stepper_electrical_power_right_w = diagnostics.stepper_electrical_power_right_w;
    row.stepper_mechanical_power_right_w = diagnostics.stepper_mechanical_power_right_w;
    row.stepper_resistive_loss_right_w = diagnostics.stepper_resistive_loss_right_w;
    row.stepper_magnetic_energy_right_j = diagnostics.stepper_magnetic_energy_right_j;
    row.stepper_voltage_saturated_left = diagnostics.stepper_voltage_saturated_left ? 1.0 : 0.0;
    row.stepper_voltage_saturated_right = diagnostics.stepper_voltage_saturated_right ? 1.0 : 0.0;
    row.seed = scenario.imu_noise_seed;
    row.total_mass_scale = scenario.total_mass_scale;
    row.pitch_inertia_scale = scenario.pitch_inertia_scale;
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
      row.left_slewed_sps = observer.feedback.left_slewed_sps;
      row.right_slewed_sps = observer.feedback.right_slewed_sps;
      row.left_actual_steps = static_cast<double>(observer.feedback.left_actual_steps);
      row.right_actual_steps = static_cast<double>(observer.feedback.right_actual_steps);
      row.motor_update_dt_ms = observer.feedback.update_dt_ms;
      row.motor_feedback_age_ms = observer.feedback.feedback_age_ms;
      row.actuator_saturation_flags = observer.feedback.actuator_saturation_flags;
    }
    row.emitted_step_velocity_sps = pipeline.emitted_step_velocity_sps_for_telemetry();
    row.synthetic_estimator_velocity_sps =
        pipeline.synthetic_residual_velocity_sps_for_telemetry();
    row.controller_feedback_velocity_sps =
        pipeline.controller_feedback_velocity_sps_for_telemetry();
    if (observer.have_telemetry) {
      const auto& telemetry = observer.telemetry;
      row.pitch_deg = telemetry.pitch_deg;
      row.pitch_rate_dps = telemetry.pitch_rate_dps;
      row.filtered_pitch_rate_dps = telemetry.filtered_pitch_rate_dps;
      row.fused_pitch_deg = telemetry.fused_pitch_deg;
      row.pitch_sp_deg = telemetry.pitch_sp_deg;
      row.u_sps = telemetry.u_sps;
      row.turn_sps = telemetry.turn_sps;
      row.nominal_acceleration_mps2 = telemetry.nominal_acceleration_mps2;
      row.raw_completed_velocity_sps = telemetry.raw_completed_velocity_sps;
      row.completed_step_acceleration_sps2 = telemetry.completed_step_acceleration_sps2;
      row.corrected_axle_velocity_sps = telemetry.corrected_axle_velocity_sps;
      row.velocity_control_sps = telemetry.velocity_control_sps;
      row.velocity_damping_acceleration_mps2 = telemetry.velocity_damping_acceleration_mps2;
      row.com_trim_deg = telemetry.com_trim_deg;
      row.user_velocity_mps = telemetry.user_velocity_mps;
      row.reference_velocity_mps = telemetry.reference_velocity_mps;
      row.reference_acceleration_mps2 = telemetry.reference_acceleration_mps2;
      row.reference_jerk_mps3 = telemetry.reference_jerk_mps3;
      row.velocity_feedback_estimate_mps = telemetry.velocity_feedback_estimate_mps;
      row.velocity_error_mps = telemetry.velocity_error_mps;
      row.velocity_feedback_acceleration_mps2 =
          telemetry.velocity_feedback_acceleration_mps2;
      row.velocity_p_acceleration_mps2 = telemetry.velocity_p_acceleration_mps2;
      row.velocity_i_acceleration_mps2 = telemetry.velocity_i_acceleration_mps2;
      row.velocity_integral_state_mps_s = telemetry.velocity_integral_state_mps_s;
      row.acceleration_raw_mps2 = telemetry.acceleration_raw_mps2;
      row.acceleration_cmd_mps2 = telemetry.acceleration_cmd_mps2;
      row.drive_pitch_target_deg = telemetry.drive_pitch_target_deg;
      row.fixed_com_trim_deg = telemetry.fixed_com_trim_deg;
      row.velocity_feedback_valid = telemetry.velocity_feedback_valid ? 1.0 : 0.0;
      row.velocity_feedback_active = telemetry.velocity_feedback_active ? 1.0 : 0.0;
      row.outer_acceleration_limited = telemetry.outer_acceleration_limited ? 1.0 : 0.0;
      row.outer_pitch_target_limited = telemetry.outer_pitch_target_limited ? 1.0 : 0.0;
      row.active_drive_max_velocity_mps = telemetry.active_drive_max_velocity_mps;
      row.active_drive_max_acceleration_mps2 = telemetry.active_drive_max_acceleration_mps2;
      row.active_drive_max_deceleration_mps2 = telemetry.active_drive_max_deceleration_mps2;
      row.active_velocity_gain_per_s = telemetry.active_velocity_gain_per_s;
      row.active_velocity_feedback_cutoff_hz = telemetry.active_velocity_feedback_cutoff_hz;
      row.active_outer_pitch_limit_deg = telemetry.active_outer_pitch_limit_deg;
      row.active_fixed_com_trim_deg = telemetry.active_fixed_com_trim_deg;
      row.adaptive_com_trim_enabled = telemetry.adaptive_com_trim_enabled ? 1.0 : 0.0;
      row.legacy_outer_fields_valid = telemetry.legacy_outer_fields_valid ? 1.0 : 0.0;
      row.final_pitch_target_deg = telemetry.final_pitch_target_deg;
      row.active_planner_max_acceleration_mps2 =
          telemetry.active_planner_max_acceleration_mps2;
      row.active_planner_max_deceleration_mps2 =
          telemetry.active_planner_max_deceleration_mps2;
      row.active_planner_max_jerk_mps3 = telemetry.active_planner_max_jerk_mps3;
      row.active_velocity_i_gain_per_s2 = telemetry.active_velocity_i_gain_per_s2;
      row.active_velocity_i_leak_time_s = telemetry.active_velocity_i_leak_time_s;
      row.active_velocity_i_acceleration_limit_mps2 =
          telemetry.active_velocity_i_acceleration_limit_mps2;
      row.planner_acceleration_limited = telemetry.planner_acceleration_limited ? 1.0 : 0.0;
      row.planner_jerk_limited = telemetry.planner_jerk_limited ? 1.0 : 0.0;
      row.velocity_integral_limited = telemetry.velocity_integral_limited ? 1.0 : 0.0;
      row.velocity_anti_windup_active = telemetry.velocity_anti_windup_active ? 1.0 : 0.0;
      row.trim_learning_enabled = telemetry.trim_learning_enabled ? 1.0 : 0.0;
      row.trim_learning_block_reason = telemetry.trim_learning_block_reason;
      row.pitch_error_deg = telemetry.pitch_error_deg;
      row.pitch_feedback_sps = telemetry.pitch_feedback_sps;
      row.pitch_rate_feedback_sps = telemetry.pitch_rate_feedback_sps;
      row.pitch_accel_feedback_sps = telemetry.pitch_accel_feedback_sps;
      row.velocity_pitch_target_deg = telemetry.velocity_pitch_target_deg;
      row.balance_unclamped_sps = telemetry.balance_unclamped_sps;
      row.active_pitch_gain_sps_per_rad = telemetry.active_pitch_gain_sps_per_rad;
      row.active_pitch_rate_gain_sps_per_rad_s = telemetry.active_pitch_rate_gain_sps_per_rad_s;
      row.active_pitch_accel_gain_sps_per_rad_s2 =
          telemetry.active_pitch_accel_gain_sps_per_rad_s2;
      row.active_velocity_pitch_gain_rad_per_sps =
          telemetry.active_velocity_pitch_gain_rad_per_sps;
      row.active_velocity_control_cutoff_hz = telemetry.active_velocity_control_cutoff_hz;
      row.active_velocity_observer_cutoff_hz = telemetry.active_velocity_observer_cutoff_hz;
      row.active_com_trim_gain_deg_per_sps_s = telemetry.active_com_trim_gain_deg_per_sps_s;
      row.active_com_trim_limit_deg = telemetry.active_com_trim_limit_deg;
      row.active_velocity_pitch_limit_deg = telemetry.active_velocity_pitch_limit_deg;
      row.active_accel_lpf_hz = telemetry.active_accel_lpf_hz;
      row.active_gyro_lpf_hz = telemetry.active_gyro_lpf_hz;
      row.active_gyro_derivative_lpf_hz = telemetry.active_gyro_derivative_lpf_hz;
      row.active_config_generation = telemetry.active_config_generation;
      row.velocity_pitch_request_unclamped_deg =
          telemetry.velocity_pitch_request_unclamped_deg;
      row.velocity_pitch_request_limited_deg = telemetry.velocity_pitch_request_limited_deg;
      row.velocity_authority_limited = telemetry.velocity_authority_limited ? 1.0 : 0.0;
      row.pitch_target_unclamped_deg = telemetry.pitch_target_unclamped_deg;
      row.pitch_target_limit_reason = telemetry.pitch_target_limit_reason;
      row.trim_trusted = telemetry.trim_trusted ? 1.0 : 0.0;
      row.trim_learning_allowed = telemetry.trim_learning_allowed ? 1.0 : 0.0;
      row.trim_quiet_rate_rms_dps = telemetry.trim_quiet_rate_rms_dps;
      row.pitch_authority_diagnostic_active =
          telemetry.pitch_authority_diagnostic_active ? 1.0 : 0.0;
      row.pitch_authority_diagnostic_target_deg =
          telemetry.pitch_authority_diagnostic_target_deg;
      row.pitch_authority_diagnostic_com_trim_deg =
          telemetry.pitch_authority_diagnostic_com_trim_deg;
      row.pitch_authority_diagnostic_remaining_s =
          telemetry.pitch_authority_diagnostic_remaining_s;
      row.pitch_authority_diagnostic_request_id =
          static_cast<double>(telemetry.pitch_authority_diagnostic_request_id);
      row.pitch_authority_diagnostic_command_age_ms =
          telemetry.pitch_authority_diagnostic_command_age_ms;
      row.command_saturated = telemetry.command_saturated;
      row.controller_fault_flags = telemetry.controller_fault_flags;
      row.controller_saturation_flags = telemetry.controller_saturation_flags;
      row.actuator_saturation_flags = telemetry.actuator_saturation_flags;
      row.actuator_fault = telemetry.actuator_fault;
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
