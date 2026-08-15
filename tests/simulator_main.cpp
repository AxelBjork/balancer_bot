#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "ipc/message_bus.h"
#include "messages/balancer_msgs.h"
#include "messages/types.h"
#include "services/control/control_service.h"
#include "services/imu/imu_service.h"
#include "services/main/config.h"
#include "services/motor/motor_runner.h"
#include "services/motor/motor_service.h"
#include "services/time/time_service.h"
#include "simulator/balancer_simulator.h"
#include "simulator/simulator_runner.h"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTickDtS = 1.0 / 400.0;
constexpr double kFallPitchDeg = 75.0;
constexpr std::size_t kMaxDatagram = 4096;
constexpr std::size_t kTailWindowSamples = static_cast<std::size_t>(2.0 / kTickDtS);

constexpr uint8_t kPhysicsSimplified = 0;
constexpr uint8_t kPhysicsRealistic = 1;
constexpr uint8_t kPhysicsActuatorStress = 2;
constexpr uint8_t kPhysicsIdealForce = 3;
constexpr uint8_t kPhysicsSimpleForce = 4;

constexpr uint8_t kAckAccepted = 0;
constexpr uint8_t kAckBusy = 1;
constexpr uint8_t kAckInvalid = 2;

constexpr uint8_t kDoneCompleted = 0;
constexpr uint8_t kDoneStoppedByClient = 1;
constexpr uint8_t kDoneFell = 2;
constexpr uint8_t kDoneInternalError = 3;
constexpr uint8_t kDoneAcceptanceFailed = 4;

std::atomic<bool> g_stop{false};

void signal_handler(int) {
  g_stop = true;
}

std::string trim_c_string(const std::array<char, 128>& bytes) {
  std::size_t len = 0;
  while (len < bytes.size() && bytes[len] != '\0') {
    ++len;
  }
  return std::string(bytes.data(), len);
}

PhysicsProfile parse_profile(uint8_t raw) {
  if (raw == kPhysicsSimplified) {
    return PhysicsProfile::Simplified;
  }
  if (raw == kPhysicsRealistic) {
    return PhysicsProfile::Realistic;
  }
  if (raw == kPhysicsActuatorStress) {
    return PhysicsProfile::ActuatorStress;
  }
  if (raw == kPhysicsIdealForce) {
    return PhysicsProfile::IdealForce;
  }
  if (raw == kPhysicsSimpleForce) {
    return PhysicsProfile::SimpleForce;
  }
  throw std::runtime_error("invalid physics profile");
}

struct PeerAddress {
  sockaddr_in addr{};
  bool valid = false;
};

struct TailSample {
  double pitch_deg = 0.0;
  double velocity_mps = 0.0;
  double force_saturated = 0.0;
};

struct ServiceDisturbance {
  uint8_t kind = ipc::kSimDisturbanceStep;
  double start_s = 0.0;
  double duration_s = 0.0;
  double force_n = 0.0;
  double com_bias_rad = 0.0;
  double force_n_end = 0.0;
  double com_bias_rad_end = 0.0;
};

struct DisturbanceSample {
  double force_n = 0.0;
  double com_bias_rad = 0.0;
};

struct RunSummary {
  uint32_t sample_count = 0;
  double final_pitch_deg = 0.0;
  double max_abs_pitch_deg = 0.0;
  double tail_rms_pitch_deg = 0.0;
  double tail_rail_fraction = 0.0;
  double tail_mean_abs_pitch_deg = 0.0;
  double max_abs_position_m = 0.0;
  double tail_mean_abs_velocity_mps = 0.0;
};

class UdpEndpoint {
 public:
  explicit UdpEndpoint(uint16_t port) {
    port_ = port;
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
      throw std::runtime_error("socket() failed");
    }

    int opt = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    const int send_buffer_bytes = 4 * 1024 * 1024;
    ::setsockopt(fd_, SOL_SOCKET, SO_SNDBUF, &send_buffer_bytes, sizeof(send_buffer_bytes));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      ::close(fd_);
      throw std::runtime_error("bind() failed");
    }

    const int flags = ::fcntl(fd_, F_GETFL, 0);
    ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
  }

  uint16_t port() const {
    return port_;
  }

  ~UdpEndpoint() {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

  std::optional<std::pair<MsgId, std::vector<uint8_t>>> recv(PeerAddress& peer) {
    std::array<uint8_t, kMaxDatagram> buf{};
    sockaddr_in sender{};
    socklen_t slen = sizeof(sender);
    const ssize_t n =
        ::recvfrom(fd_, buf.data(), buf.size(), 0, reinterpret_cast<sockaddr*>(&sender), &slen);
    if (n < 0) {
      return std::nullopt;
    }

    peer.addr = sender;
    peer.valid = true;

    if (n < static_cast<ssize_t>(sizeof(uint16_t))) {
      return std::nullopt;
    }

    uint16_t raw = 0;
    std::memcpy(&raw, buf.data(), sizeof(raw));
    std::vector<uint8_t> payload(buf.begin() + sizeof(uint16_t), buf.begin() + n);
    return std::make_pair(static_cast<MsgId>(raw), std::move(payload));
  }

  template <typename Payload>
  void send(const PeerAddress& peer, MsgId id, const Payload& payload) {
    if (!peer.valid) {
      return;
    }

    static_assert(std::is_trivially_copyable_v<Payload>);
    iovec iov[2];
    uint16_t raw_id = static_cast<uint16_t>(id);
    iov[0].iov_base = &raw_id;
    iov[0].iov_len = sizeof(raw_id);
    iov[1].iov_base = const_cast<Payload*>(&payload);
    iov[1].iov_len = sizeof(Payload);

    msghdr msg{};
    msg.msg_name = const_cast<sockaddr*>(reinterpret_cast<const sockaddr*>(&peer.addr));
    msg.msg_namelen = sizeof(peer.addr);
    msg.msg_iov = iov;
    msg.msg_iovlen = 2;
    ::sendmsg(fd_, &msg, 0);
  }

 private:
  int fd_{-1};
  uint16_t port_{0};
};

struct PigpioCtx {
  PigpioCtx() {
    pi = pigpio_start(nullptr, nullptr);
    if (pi < 0) {
      throw std::runtime_error("pigpio_start failed");
    }
  }

  ~PigpioCtx() {
    pigpio_stop(pi);
  }

  int handle() const {
    return pi;
  }

  int pi{-1};
};

struct ScenarioObserver {
  using Subscribes = ipc::MsgList<MsgId::SystemTelemetry, MsgId::MotorFeedback>;

  template <MsgId Id>
  void on_message(const typename MessageTraits<Id>::Payload& p) {
    (void)p;
  }

  ipc::SystemTelemetryPayload latest_telemetry{};
  ipc::MotorFeedbackPayload latest_motor_feedback{};
  bool have_telemetry = false;
  bool have_motor_feedback = false;
};

template <>
inline void ScenarioObserver::on_message<MsgId::SystemTelemetry>(
    const ipc::SystemTelemetryPayload& p) {
  latest_telemetry = p;
  have_telemetry = true;
}

template <>
inline void ScenarioObserver::on_message<MsgId::MotorFeedback>(const ipc::MotorFeedbackPayload& p) {
  latest_motor_feedback = p;
  have_motor_feedback = true;
}

struct ScenarioServices {
  PigpioCtx pigpio;
  Stepper left;
  Stepper right;
  sil::TimeService time;
  MotorRunner motors;
  sil::ImuService imu;
  sil::ControlService control;
  sil::MotorService motor_service;
  ScenarioObserver observer;

  ScenarioServices(ipc::MessageBus& bus)
      : left(pigpio.handle(), Stepper::Pins{12, 19, 13}, false, true),
        right(pigpio.handle(), Stepper::Pins{4, 18, 24}, false, true),
        time(bus, kTickDtS),
        motors(left, right, Config::control_hz, Config::motor_slew_sps_per_s),
        imu(bus, false),
        control(bus),
        motor_service(bus, &motors) {
  }
};

struct ScenarioBusContainer {
  ipc::MessageBus bus;
  ScenarioServices services;

  ScenarioBusContainer() : bus(this, &ScenarioBusContainer::dispatch), services(bus) {
  }

  static void dispatch(void* ctx, MsgId id, const void* payload) {
    auto* self = static_cast<ScenarioBusContainer*>(ctx);
    ipc::dispatch_to_services(id, payload, self->services.imu, self->services.motor_service,
                              self->services.control, self->services.observer);
  }
};

class SimulatorService {
 public:
  explicit SimulatorService(uint16_t port, std::string default_pid_config)
      : endpoint_(port), default_pid_config_(std::move(default_pid_config)) {
  }

  void run() {
    std::cout << "Starting balancer_simulator service on UDP port " << endpoint_.port()
              << std::endl;
    while (!g_stop.load()) {
      pump_messages();
      if (run_.has_value()) {
        step_active_run();
        // Full-rate artifact capture needs light pacing so the UDP consumer can
        // drain every row. Downsampled and summary-only validation runs remain
        // fully deterministic and run without wall-clock sleeps.
        if (run_.has_value() && run_->telemetry_stride == 1) {
          std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
  }

 private:
  struct ActiveRun {
    uint32_t run_id = 0;
    std::string pid_config_path;
    SimulatorScenario scenario;
    SimulatorEngine engine;
    uint64_t sim_time_us = 0;
    int steps_total = 0;
    int steps_done = 0;
    uint16_t telemetry_stride = 1;
    bool transfer_validation = false;
    double max_abs_pitch_deg = 0.0;
    double max_abs_position_m = 0.0;
    double current_saturation_s = 0.0;
    double max_continuous_saturation_s = 0.0;
    uint32_t actuator_fault_count = 0;
    uint32_t controller_fault_flags = 0;
    uint64_t timeline_hash = 1469598103934665603ULL;
    std::deque<TailSample> tail_samples;
    std::vector<SimulatorTimelineRow> transfer_rows;
    explicit ActiveRun(uint32_t id, std::string pid_path, SimulatorScenario scenario_in,
                       bool is_transfer_validation)
        : run_id(id),
          pid_config_path(std::move(pid_path)),
          scenario(std::move(scenario_in)),
          engine(scenario),
          transfer_validation(is_transfer_validation) {
      if (transfer_validation) {
        transfer_rows.reserve(static_cast<size_t>(std::llround(scenario.duration_s / kTickDtS)));
      }
    }
  };

  void pump_messages() {
    while (true) {
      auto msg = endpoint_.recv(active_peer_);
      if (!msg.has_value()) {
        return;
      }

      const auto [id, payload] = *msg;
      if (id == static_cast<MsgId>(0)) {
        continue;
      }

      try {
        if (id == MsgId::SimStartRun) {
          handle_start(payload);
        } else if (id == MsgId::SimStopRun) {
          handle_stop(payload);
        } else if (id == MsgId::JoystickCommand) {
          handle_joystick(payload);
        }
      } catch (const std::exception&) {
        if (id == MsgId::SimStartRun && payload.size() >= sizeof(ipc::SimStartRunPayload)) {
          ipc::SimStartRunPayload request{};
          std::memcpy(&request, payload.data(), sizeof(request));
          send_ack(request.run_id, false, kAckInvalid);
        }
      }
    }
  }

  void handle_start(const std::vector<uint8_t>& payload) {
    if (payload.size() != sizeof(ipc::SimStartRunPayload)) {
      throw std::runtime_error("bad start payload");
    }

    ipc::SimStartRunPayload request{};
    std::memcpy(&request, payload.data(), sizeof(request));
    if (run_.has_value()) {
      send_ack(request.run_id, false, kAckBusy);
      return;
    }

    const PhysicsProfile profile = parse_profile(request.physics_profile);

    const std::string requested_pid = trim_c_string(request.pid_config_path);
    const std::string pid_path = requested_pid.empty() ? default_pid_config_ : requested_pid;
    ConfigPid::load(pid_path);

    SimulatorScenario scenario;
    const bool transfer_validation =
        request.transfer_scenario_index != std::numeric_limits<uint16_t>::max();
    if (transfer_validation) {
      const auto transfer_scenarios = transfer_scenario_set();
      if (request.transfer_scenario_index >= transfer_scenarios.size()) {
        throw std::runtime_error("invalid transfer scenario index");
      }
      scenario = transfer_scenarios[request.transfer_scenario_index];
    } else {
      scenario.name = "udp_run_" + std::to_string(request.run_id);
      scenario.physics_profile = profile;
      scenario.duration_s = request.duration_s;
      scenario.initial_pitch_deg = request.initial_pitch_deg;
      scenario.initial_pitch_rate_dps = request.initial_pitch_rate_dps;
      scenario.initial_velocity_mps = request.initial_velocity_mps;
      scenario.com_angle_offset_rad = request.com_angle_offset_rad;
      scenario.total_mass_scale = request.total_mass_scale;
      scenario.pitch_inertia_scale = request.pitch_inertia_scale;
      // The low-frequency mass variation is a correlated mechanical case:
      // total mass and first mass moment change together so the COM height is
      // not accidentally moved by a request that only intended to vary mass.
      scenario.first_mass_moment_scale = request.total_mass_scale;
      if (request.has_physics_override != 0) {
        SimulatorPhysics physics = BalancerSimulator::physics_for_profile(profile);
        if (request.motor_max_force_n > 0.0) physics.max_force_n = request.motor_max_force_n;
        physics.no_load_speed_mps = request.motor_no_load_speed_mps;
        physics.motor_velocity_damping = request.motor_velocity_damping;
        physics.motor_tau_s = request.motor_tau_s;
        physics.traction_coefficient = request.traction_coefficient;
        physics.pitch_damping = request.pitch_damping;
        physics.cart_damping = request.cart_damping;
        physics.phase_error_limit_steps = request.phase_error_limit_steps;
        physics.tire_stiffness_n_per_m = request.tire_stiffness_n_per_m;
        physics.tire_damping_n_s_per_m = request.tire_damping_n_s_per_m;
        physics.wheel_equivalent_mass_kg = request.wheel_equivalent_mass_kg;
        scenario.physics_override = physics;
      }
      scenario.imu_pitch_lag_s = request.imu_pitch_lag_s;
      scenario.imu_noise_seed = request.imu_noise_seed;
      scenario.accel_noise_std_mps2 = request.accel_noise_std_mps2;
      scenario.gyro_noise_std_rad_s = request.gyro_noise_std_rad_s;
      scenario.imu_timestamp_jitter_us = request.imu_timestamp_jitter_us;
      scenario.imu_sample_loss_rate = request.imu_sample_loss_rate;
      scenario.accel_bias_mps2 = request.accel_bias_mps2;
      scenario.gyro_bias_rad_s = request.gyro_bias_rad_s;
      scenario.velocity_estimator_bias_mps = request.velocity_estimator_bias_mps;
      scenario.velocity_estimator_bias_drift_mps_per_s =
          request.velocity_estimator_bias_drift_mps_per_s;
      scenario.velocity_estimator_scale = request.velocity_estimator_scale;
      scenario.velocity_estimator_latency_s = request.velocity_estimator_latency_s;
      for (std::size_t i = 0; i < request.disturbances.size(); ++i) {
        const auto& wire = request.disturbances[i];
        SimulatorDisturbanceKind kind = SimulatorDisturbanceKind::Step;
        if (wire.kind == ipc::kSimDisturbanceRamp) kind = SimulatorDisturbanceKind::Ramp;
        if (wire.kind == ipc::kSimDisturbanceHoldBias) kind = SimulatorDisturbanceKind::HoldBias;
        scenario.disturbances.push_back(SimulatorDisturbance{
            .kind = kind,
            .start_s = wire.start_s,
            .duration_s = wire.duration_s,
            .force_n = wire.force_n,
            .com_bias_rad = wire.com_bias_rad,
            .force_n_end = wire.force_n_end,
            .com_bias_rad_end = wire.com_bias_rad_end,
        });
      }
      for (const auto& wire : request.joy_segments) {
        if (wire.duration_s <= 0.0) continue;
        scenario.joy_segments.push_back(SimulatorJoySegment{
            .start_s = wire.start_s,
            .duration_s = wire.duration_s,
            .forward = wire.forward,
            .turn = wire.turn,
            .forward_end = wire.forward_end,
            .turn_end = wire.turn_end,
        });
      }
      for (const auto& wire : request.pitch_authority_segments) {
        if (wire.duration_s <= 0.0) continue;
        scenario.pitch_authority_segments.push_back(SimulatorPitchAuthoritySegment{
            .start_s = wire.start_s,
            .duration_s = wire.duration_s,
            .target_deg = wire.target_deg,
            .com_trim_deg = wire.com_trim_deg,
        });
      }
      scenario.pitch_authority_refresh_dropout_start_s =
          request.pitch_authority_refresh_dropout_start_s;
      scenario.pitch_authority_refresh_dropout_duration_s =
          request.pitch_authority_refresh_dropout_duration_s;
    }

    run_.emplace(request.run_id, pid_path, std::move(scenario), transfer_validation);
    run_->telemetry_stride = request.telemetry_stride;
    run_->steps_total =
        std::max(1, static_cast<int>(std::llround(run_->scenario.duration_s / kTickDtS)));
    run_->max_abs_pitch_deg = std::abs(run_->engine.simulator().get_pitch()) * 180.0 / kPi;

    send_ack(request.run_id, true, kAckAccepted);
  }

  void handle_stop(const std::vector<uint8_t>& payload) {
    if (payload.size() != sizeof(ipc::SimStopRunPayload)) {
      return;
    }

    ipc::SimStopRunPayload request{};
    std::memcpy(&request, payload.data(), sizeof(request));
    if (run_.has_value() && run_->run_id == request.run_id) {
      finish_active_run(kDoneStoppedByClient);
    }
  }

  void handle_joystick(const std::vector<uint8_t>& payload) {
    if (!run_.has_value() || payload.size() != sizeof(ipc::JoystickCommandPayload)) {
      return;
    }

    ipc::JoystickCommandPayload request{};
    std::memcpy(&request, payload.data(), sizeof(request));
    run_->engine.set_joystick(request.forward, request.turn);
  }

  void step_active_run() {
    ActiveRun& run = *run_;
    const SimulatorTimelineRow row = run.engine.step();
    run.timeline_hash = update_simulator_timeline_hash(run.timeline_hash, row);
    if (run.transfer_validation) run.transfer_rows.push_back(row);
    run.sim_time_us = run.engine.current_time_us();
    ++run.steps_done;

    if (run.telemetry_stride > 0 &&
        ((run.steps_done % run.telemetry_stride) == 0 || run.steps_done == run.steps_total)) {
      publish_telemetry(run, row);
    }

    const double plant_pitch_deg = row.plant_pitch_deg;
    run.max_abs_pitch_deg = std::max(run.max_abs_pitch_deg, std::abs(plant_pitch_deg));
    run.max_abs_position_m = std::max(run.max_abs_position_m, std::abs(row.plant_position));
    run.controller_fault_flags |= row.controller_fault_flags;
    if (row.actuator_fault > 0.5) ++run.actuator_fault_count;
    if (row.command_saturated > 0.5) {
      run.current_saturation_s += kTickDtS;
      run.max_continuous_saturation_s =
          std::max(run.max_continuous_saturation_s, run.current_saturation_s);
    } else {
      run.current_saturation_s = 0.0;
    }
    run.tail_samples.push_back(TailSample{
        .pitch_deg = row.plant_pitch_deg,
        .velocity_mps = row.plant_velocity,
        .force_saturated = row.force_saturated,
    });
    while (run.tail_samples.size() > kTailWindowSamples) run.tail_samples.pop_front();

    // Transfer scenarios must run to their requested duration so the UDP path
    // remains exactly comparable with run_simulator_scenario(), which records
    // the complete timeline even after the simulated plant has fallen.
    if (run.max_abs_pitch_deg > kFallPitchDeg && !run.transfer_validation) {
      finish_active_run(kDoneFell);
      return;
    }

    if (run.steps_done >= run.steps_total) {
      finish_active_run(kDoneCompleted);
    }
  }

  static float accel_pitch_deg(const std::array<double, 3>& acc) {
    return static_cast<float>(std::atan2(-acc[0], -acc[2]) * (180.0 / kPi));
  }

  void publish_telemetry(ActiveRun& run, const SimulatorTimelineRow& row) {
    ipc::SimulatorTelemetryPayload payload{};
    auto& system = payload.system;
    system.run_id = run.run_id;
    payload.seed = row.seed;
    system.controller_fault_flags = row.controller_fault_flags;
    system.controller_saturation_flags = row.controller_saturation_flags;
    system.imu_timestamp_us = row.imu_timestamp_us;
    system.t_sec = static_cast<float>(row.sim_time_s);
    system.pitch_deg = static_cast<float>(row.pitch_deg);
    system.pitch_rate_dps = static_cast<float>(row.pitch_rate_dps);
    system.raw_acc_pitch_deg = static_cast<float>(row.raw_acc_pitch_deg);
    system.fused_pitch_deg = static_cast<float>(row.fused_pitch_deg);
    system.gyro_pitch_rate_dps = static_cast<float>(row.gyro_pitch_rate_dps);
    system.filtered_pitch_rate_dps = static_cast<float>(row.filtered_pitch_rate_dps);
    system.u_sps = static_cast<float>(row.u_sps);
    system.turn_sps = static_cast<float>(row.turn_sps);
    system.nominal_acceleration_mps2 = static_cast<float>(row.nominal_acceleration_mps2);
    system.raw_completed_velocity_sps = static_cast<float>(row.raw_completed_velocity_sps);
    system.completed_step_acceleration_sps2 =
        static_cast<float>(row.completed_step_acceleration_sps2);
    system.corrected_axle_velocity_sps = static_cast<float>(row.corrected_axle_velocity_sps);
    system.velocity_control_sps = static_cast<float>(row.velocity_control_sps);
    system.velocity_damping_acceleration_mps2 =
        static_cast<float>(row.velocity_damping_acceleration_mps2);
    system.com_trim_deg = static_cast<float>(row.com_trim_deg);
    system.pitch_error_deg = static_cast<float>(row.pitch_error_deg);
    system.pitch_sp_deg = static_cast<float>(row.pitch_sp_deg);
    system.pitch_feedback_sps = static_cast<float>(row.pitch_feedback_sps);
    system.pitch_rate_feedback_sps = static_cast<float>(row.pitch_rate_feedback_sps);
    system.pitch_accel_feedback_sps = static_cast<float>(row.pitch_accel_feedback_sps);
    system.velocity_pitch_target_deg = static_cast<float>(row.velocity_pitch_target_deg);
    system.balance_unclamped_sps = static_cast<float>(row.balance_unclamped_sps);
    system.active_pitch_gain_sps_per_rad = static_cast<float>(row.active_pitch_gain_sps_per_rad);
    system.active_pitch_rate_gain_sps_per_rad_s =
        static_cast<float>(row.active_pitch_rate_gain_sps_per_rad_s);
    system.active_pitch_accel_gain_sps_per_rad_s2 =
        static_cast<float>(row.active_pitch_accel_gain_sps_per_rad_s2);
    system.active_velocity_pitch_gain_rad_per_sps =
        static_cast<float>(row.active_velocity_pitch_gain_rad_per_sps);
    system.active_velocity_control_cutoff_hz =
        static_cast<float>(row.active_velocity_control_cutoff_hz);
    system.active_velocity_observer_cutoff_hz =
        static_cast<float>(row.active_velocity_observer_cutoff_hz);
    system.active_com_trim_gain_deg_per_sps_s =
        static_cast<float>(row.active_com_trim_gain_deg_per_sps_s);
    system.active_com_trim_limit_deg = static_cast<float>(row.active_com_trim_limit_deg);
    system.active_velocity_pitch_limit_deg =
        static_cast<float>(row.active_velocity_pitch_limit_deg);
    system.active_accel_lpf_hz = static_cast<float>(row.active_accel_lpf_hz);
    system.active_gyro_lpf_hz = static_cast<float>(row.active_gyro_lpf_hz);
    system.active_gyro_derivative_lpf_hz = static_cast<float>(row.active_gyro_derivative_lpf_hz);
    system.active_config_generation = row.active_config_generation;
    system.velocity_pitch_request_unclamped_deg =
        static_cast<float>(row.velocity_pitch_request_unclamped_deg);
    system.velocity_pitch_request_limited_deg =
        static_cast<float>(row.velocity_pitch_request_limited_deg);
    system.pitch_target_unclamped_deg = static_cast<float>(row.pitch_target_unclamped_deg);
    system.trim_quiet_rate_rms_dps = static_cast<float>(row.trim_quiet_rate_rms_dps);
    system.velocity_authority_limited = row.velocity_authority_limited > 0.5;
    system.trim_trusted = row.trim_trusted > 0.5;
    system.trim_learning_allowed = row.trim_learning_allowed > 0.5;
    system.pitch_target_limit_reason = static_cast<uint8_t>(row.pitch_target_limit_reason);
    system.pitch_authority_diagnostic_active =
        row.pitch_authority_diagnostic_active > 0.5;
    system.pitch_authority_diagnostic_target_deg =
        static_cast<float>(row.pitch_authority_diagnostic_target_deg);
    system.pitch_authority_diagnostic_com_trim_deg =
        static_cast<float>(row.pitch_authority_diagnostic_com_trim_deg);
    system.pitch_authority_diagnostic_remaining_s =
        static_cast<float>(row.pitch_authority_diagnostic_remaining_s);
    system.pitch_authority_diagnostic_request_id =
        static_cast<uint32_t>(row.pitch_authority_diagnostic_request_id);
    system.pitch_authority_diagnostic_command_age_ms =
        static_cast<float>(row.pitch_authority_diagnostic_command_age_ms);
    system.command_saturated = row.command_saturated != 0.0;
    system.actuator_fault = row.actuator_fault != 0.0;
    system.trim_learning_enabled = row.trim_learning_enabled != 0.0 ? 1u : 0u;
    system.trim_learning_block_reason =
        static_cast<uint8_t>(row.trim_learning_block_reason);
    system.trim_learning_reserved = 0u;
    system.left_target_sps = static_cast<float>(row.left_sps);
    system.right_target_sps = static_cast<float>(row.right_sps);
    system.left_slewed_sps = static_cast<float>(row.left_slewed_sps);
    system.right_slewed_sps = static_cast<float>(row.right_slewed_sps);
    system.motor_update_dt_ms = static_cast<float>(row.motor_update_dt_ms);
    system.motor_feedback_age_ms = static_cast<float>(row.motor_feedback_age_ms);
    system.left_actual_steps = static_cast<int32_t>(row.left_actual_steps);
    system.right_actual_steps = static_cast<int32_t>(row.right_actual_steps);
    system.actuator_saturation_flags = row.actuator_saturation_flags;
    payload.plant_pitch_deg = static_cast<float>(row.plant_pitch_deg);
    payload.plant_pitch_rate_dps = static_cast<float>(row.plant_pitch_rate_dps);
    payload.plant_position_m = static_cast<float>(row.plant_position);
    payload.plant_velocity_mps = static_cast<float>(row.plant_velocity);
    payload.target_wheel_velocity = static_cast<float>(row.target_wheel_velocity);
    payload.actual_wheel_velocity = static_cast<float>(row.actual_wheel_velocity);
    payload.plant_velocity_error = static_cast<float>(row.velocity_error);
    payload.f_cmd = static_cast<float>(row.f_cmd);
    payload.f_app = static_cast<float>(row.f_app);
    payload.external_force_n = static_cast<float>(row.external_force_n);
    payload.external_com_bias_rad = static_cast<float>(row.external_com_bias_rad);
    payload.x_ddot = static_cast<float>(row.x_ddot);
    payload.theta_ddot = static_cast<float>(row.theta_ddot);
    payload.force_saturated = row.force_saturated != 0.0;
    payload.phase_error_steps = static_cast<float>(row.phase_error_steps);
    payload.missed_steps = static_cast<float>(row.missed_steps);
    payload.traction_limit_n = static_cast<float>(row.traction_limit_n);
    payload.motor_force_limit_n = static_cast<float>(row.motor_force_limit_n);
    payload.total_mass_scale = static_cast<float>(row.total_mass_scale);
    payload.pitch_inertia_scale = static_cast<float>(row.pitch_inertia_scale);
    const auto& physics = run.engine.simulator().physics();
    payload.motor_max_force_n = static_cast<float>(physics.max_force_n);
    payload.motor_no_load_speed_mps = static_cast<float>(physics.no_load_speed_mps);
    payload.motor_velocity_damping = static_cast<float>(physics.motor_velocity_damping);
    payload.motor_tau_s = static_cast<float>(physics.motor_tau_s);
    payload.traction_coefficient = static_cast<float>(physics.traction_coefficient);
    payload.pitch_damping = static_cast<float>(physics.pitch_damping);
    payload.cart_damping = static_cast<float>(physics.cart_damping);
    payload.phase_error_limit_steps = static_cast<float>(physics.phase_error_limit_steps);
    payload.tire_stiffness_n_per_m = static_cast<float>(physics.tire_stiffness_n_per_m);
    payload.tire_damping_n_s_per_m = static_cast<float>(physics.tire_damping_n_s_per_m);
    payload.wheel_equivalent_mass_kg = static_cast<float>(physics.wheel_equivalent_mass_kg);
    endpoint_.send(active_peer_, MsgId::SimulatorTelemetry, payload);
  }

  RunSummary summarize(const ActiveRun& run) const {
    RunSummary out{};
    out.sample_count = static_cast<uint32_t>(run.steps_done);
    out.final_pitch_deg = run.engine.simulator().get_pitch() * 180.0 / kPi;
    out.max_abs_pitch_deg = run.max_abs_pitch_deg;
    out.max_abs_position_m = run.max_abs_position_m;

    if (run.tail_samples.empty()) {
      return out;
    }

    double pitch_sq_sum = 0.0;
    double abs_pitch_sum = 0.0;
    double abs_velocity_sum = 0.0;
    double rail_sum = 0.0;
    for (const auto& sample : run.tail_samples) {
      pitch_sq_sum += sample.pitch_deg * sample.pitch_deg;
      abs_pitch_sum += std::abs(sample.pitch_deg);
      abs_velocity_sum += std::abs(sample.velocity_mps);
      rail_sum += sample.force_saturated >= 0.5 ? 1.0 : 0.0;
    }

    const double count = static_cast<double>(run.tail_samples.size());
    out.tail_rms_pitch_deg = std::sqrt(pitch_sq_sum / count);
    out.tail_mean_abs_pitch_deg = abs_pitch_sum / count;
    out.tail_mean_abs_velocity_mps = abs_velocity_sum / count;
    out.tail_rail_fraction = rail_sum / count;
    return out;
  }

  void finish_active_run(uint8_t reason_code) {
    if (!run_.has_value()) {
      return;
    }

    const RunSummary summary = summarize(*run_);
    if (reason_code == kDoneCompleted && run_->transfer_validation) {
      SimulatorRunResult result;
      result.scenario = run_->scenario;
      result.physics = run_->engine.physics();
      result.rows = run_->transfer_rows;
      result.final_pitch_deg = summary.final_pitch_deg;
      result.max_abs_pitch_deg = summary.max_abs_pitch_deg;
      result.tail_rms_pitch_deg = summary.tail_rms_pitch_deg;
      result.max_continuous_saturation_s = run_->max_continuous_saturation_s;
      result.actuator_fault_count = run_->actuator_fault_count;
      result.controller_fault_flags = run_->controller_fault_flags;
      result.timeline_hash = run_->timeline_hash;
      result.fell = summary.max_abs_pitch_deg > kFallPitchDeg;
      if (!evaluate_transfer_scenario(result).accepted) {
        reason_code = kDoneAcceptanceFailed;
      }
    }
    ipc::SimRunDonePayload done{};
    done.run_id = run_->run_id;
    done.reason_code = reason_code;
    done.sample_count = summary.sample_count;
    done.elapsed_s = static_cast<double>(run_->sim_time_us) / 1e6;
    done.final_pitch_deg = summary.final_pitch_deg;
    done.max_abs_pitch_deg = summary.max_abs_pitch_deg;
    done.tail_rms_pitch_deg = summary.tail_rms_pitch_deg;
    done.tail_rail_fraction = summary.tail_rail_fraction;
    done.tail_mean_abs_pitch_deg = summary.tail_mean_abs_pitch_deg;
    done.max_abs_position_m = summary.max_abs_position_m;
    done.tail_mean_abs_velocity_mps = summary.tail_mean_abs_velocity_mps;
    done.max_continuous_saturation_s = run_->max_continuous_saturation_s;
    done.actuator_fault_count = run_->actuator_fault_count;
    done.controller_fault_flags = run_->controller_fault_flags;
    done.timeline_hash = run_->timeline_hash;
    endpoint_.send(active_peer_, MsgId::SimRunDone, done);
    run_.reset();
  }

  void send_ack(uint32_t run_id, bool accepted, uint8_t status_code) {
    ipc::SimStartAckPayload ack{};
    ack.run_id = run_id;
    ack.accepted = accepted ? 1 : 0;
    ack.status_code = status_code;
    endpoint_.send(active_peer_, MsgId::SimStartAck, ack);
  }

  UdpEndpoint endpoint_;
  PeerAddress active_peer_{};
  std::string default_pid_config_;
  std::optional<ActiveRun> run_;
};

void print_transfer_catalog_json() {
  const auto scenarios = transfer_scenario_set();
  std::cout << std::setprecision(17) << '[';
  for (size_t index = 0; index < scenarios.size(); ++index) {
    const auto& scenario = scenarios[index];
    const auto physics = scenario.physics_override.value_or(
        BalancerSimulator::physics_for_profile(scenario.physics_profile));
    if (index != 0) std::cout << ',';
    std::cout << "{\"name\":\"" << scenario.name << "\""
              << ",\"physics_profile\":\"" << BalancerSimulator::profile_name(
                     scenario.physics_profile)
              << "\""
              << ",\"duration_s\":" << scenario.duration_s
              << ",\"initial_pitch_deg\":" << scenario.initial_pitch_deg
              << ",\"com_angle_offset_rad\":" << scenario.com_angle_offset_rad
              << ",\"total_mass_scale\":" << scenario.total_mass_scale
              << ",\"pitch_inertia_scale\":" << scenario.pitch_inertia_scale
              << ",\"imu_pitch_lag_s\":" << scenario.imu_pitch_lag_s
              << ",\"imu_noise_seed\":" << scenario.imu_noise_seed
              << ",\"accel_noise_std_mps2\":" << scenario.accel_noise_std_mps2
              << ",\"gyro_noise_std_rad_s\":" << scenario.gyro_noise_std_rad_s
              << ",\"imu_timestamp_jitter_us\":" << scenario.imu_timestamp_jitter_us
              << ",\"imu_sample_loss_rate\":" << scenario.imu_sample_loss_rate
              << ",\"accel_bias_mps2\":[" << scenario.accel_bias_mps2[0] << ','
              << scenario.accel_bias_mps2[1] << ',' << scenario.accel_bias_mps2[2] << ']'
              << ",\"gyro_bias_rad_s\":[" << scenario.gyro_bias_rad_s[0] << ','
              << scenario.gyro_bias_rad_s[1] << ',' << scenario.gyro_bias_rad_s[2] << ']'
              << ",\"physics\":{\"motor_max_force_n\":" << physics.max_force_n
              << ",\"motor_no_load_speed_mps\":" << physics.no_load_speed_mps
              << ",\"motor_velocity_damping\":" << physics.motor_velocity_damping
              << ",\"motor_tau_s\":" << physics.motor_tau_s
              << ",\"traction_coefficient\":" << physics.traction_coefficient
              << ",\"pitch_damping\":" << physics.pitch_damping
              << ",\"cart_damping\":" << physics.cart_damping
              << ",\"phase_error_limit_steps\":" << physics.phase_error_limit_steps
              << ",\"tire_stiffness_n_per_m\":" << physics.tire_stiffness_n_per_m
              << ",\"tire_damping_n_s_per_m\":" << physics.tire_damping_n_s_per_m
              << ",\"wheel_equivalent_mass_kg\":" << physics.wheel_equivalent_mass_kg
              << "},\"disturbances\":[";
    for (size_t disturbance_index = 0; disturbance_index < scenario.disturbances.size();
         ++disturbance_index) {
      const auto& disturbance = scenario.disturbances[disturbance_index];
      if (disturbance_index != 0) std::cout << ',';
      std::cout << "{\"kind\":" << static_cast<int>(disturbance.kind)
                << ",\"start_s\":" << disturbance.start_s
                << ",\"duration_s\":" << disturbance.duration_s
                << ",\"force_n\":" << disturbance.force_n
                << ",\"com_bias_rad\":" << disturbance.com_bias_rad
                << ",\"force_n_end\":" << disturbance.force_n_end
                << ",\"com_bias_rad_end\":" << disturbance.com_bias_rad_end << '}';
    }
    std::cout << "],\"joy_segments\":[";
    for (size_t joy_index = 0; joy_index < scenario.joy_segments.size(); ++joy_index) {
      const auto& joy = scenario.joy_segments[joy_index];
      if (joy_index != 0) std::cout << ',';
      std::cout << "{\"start_s\":" << joy.start_s << ",\"duration_s\":" << joy.duration_s
                << ",\"forward\":" << joy.forward << ",\"turn\":" << joy.turn
                << ",\"forward_end\":" << joy.forward_end << ",\"turn_end\":" << joy.turn_end
                << '}';
    }
    std::cout << "]}";
  }
  std::cout << "]\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  uint16_t port = 9001;
  std::string pid_config_path = ConfigPid::resolve_path("pid.conf");
  std::optional<size_t> direct_summary_index;
  bool catalog_json = false;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--port" && (i + 1) < argc) {
      port = static_cast<uint16_t>(std::stoul(argv[++i]));
    } else if (arg == "--pid-config" && (i + 1) < argc) {
      pid_config_path = argv[++i];
    } else if (arg == "--direct-summary" && (i + 1) < argc) {
      direct_summary_index = std::stoul(argv[++i]);
    } else if (arg == "--catalog-json") {
      catalog_json = true;
    } else if (arg == "--help") {
      std::cout << "Usage: balancer_simulator [--port <udp-port>] [--pid-config <path>] "
                   "[--direct-summary <transfer-index>] [--catalog-json]\n";
      return 0;
    } else {
      std::cerr << "Unknown argument: " << arg << std::endl;
      return 1;
    }
  }

  try {
    if (catalog_json) {
      print_transfer_catalog_json();
      return 0;
    }
    if (direct_summary_index.has_value()) {
      const auto scenarios = transfer_scenario_set();
      if (*direct_summary_index >= scenarios.size()) {
        throw std::runtime_error("invalid transfer scenario index");
      }
      const auto result = run_simulator_scenario(scenarios[*direct_summary_index], pid_config_path);
      const auto acceptance = evaluate_transfer_scenario(result);
      std::cout << std::setprecision(17) << "{\"sample_count\":" << result.rows.size()
                << ",\"elapsed_s\":" << scenarios[*direct_summary_index].duration_s
                << ",\"final_pitch_deg\":" << result.final_pitch_deg
                << ",\"max_abs_pitch_deg\":" << result.max_abs_pitch_deg
                << ",\"tail_rms_pitch_deg\":" << result.tail_rms_pitch_deg
                << ",\"max_continuous_saturation_s\":" << result.max_continuous_saturation_s
                << ",\"actuator_fault_count\":" << result.actuator_fault_count
                << ",\"controller_fault_flags\":" << result.controller_fault_flags
                << ",\"accepted\":" << (acceptance.accepted ? "true" : "false")
                << ",\"failures\":[";
      for (size_t failure_index = 0; failure_index < acceptance.failures.size(); ++failure_index) {
        if (failure_index != 0) std::cout << ',';
        std::cout << '\"' << acceptance.failures[failure_index] << '\"';
      }
      std::cout << ']'
                << ",\"timeline_hash\":" << result.timeline_hash << "}\n";
      return 0;
    }
    SimulatorService service(port, pid_config_path);
    service.run();
    return 0;
  } catch (const std::exception& exc) {
    std::cerr << "Simulator service failed: " << exc.what() << std::endl;
    return 1;
  }
}
