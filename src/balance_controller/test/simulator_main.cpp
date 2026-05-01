#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iostream>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>

#include "messages/balancer_msgs.h"
#include "config.h"
#include "simulator/balancer_simulator.h"
#include "types.h"
#include "services/imu/pitch_lpf.h"
#include "services/control/rate_controller_core.h"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTickDtS = 1.0 / 400.0;
constexpr double kFallPitchDeg = 75.0;
constexpr std::size_t kMaxDatagram = 4096;
constexpr std::size_t kTailWindowSamples = static_cast<std::size_t>(2.0 / kTickDtS);
constexpr int kTelemetryStride = 80;

constexpr uint8_t kPhysicsSimplified = 0;
constexpr uint8_t kPhysicsRealistic = 1;

constexpr uint8_t kAckAccepted = 0;
constexpr uint8_t kAckBusy = 1;
constexpr uint8_t kAckInvalid = 2;

constexpr uint8_t kDoneCompleted = 0;
constexpr uint8_t kDoneStoppedByClient = 1;
constexpr uint8_t kDoneFell = 2;
constexpr uint8_t kDoneInternalError = 3;

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
  float force_n = 0.0f;
  float com_bias_rad = 0.0f;
  float force_n_end = 0.0f;
  float com_bias_rad_end = 0.0f;
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

  uint16_t port() const { return port_; }

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
    struct iovec iov[2];
    uint16_t raw_id = static_cast<uint16_t>(id);
    iov[0].iov_base = &raw_id;
    iov[0].iov_len = sizeof(raw_id);
    iov[1].iov_base = const_cast<Payload*>(&payload);
    iov[1].iov_len = sizeof(Payload);

    struct msghdr msg {};
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

class SimulatorService {
 public:
  explicit SimulatorService(uint16_t port, std::string default_pid_config)
      : endpoint_(port), default_pid_config_(std::move(default_pid_config)) {}

  void run() {
    std::cout << "Starting balancer_simulator service on UDP port "
              << endpoint_.port() << std::endl;
    while (!g_stop.load()) {
      pump_messages();
      if (run_.has_value()) {
        step_active_run();
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
  }

 private:
  struct ActiveRun {
    uint32_t run_id = 0;
    std::string pid_config_path;
    std::array<ServiceDisturbance, ipc::kMaxSimDisturbances> disturbances{};
    BalancerSimulator sim;
    RateControllerCore core;
    Telemetry latest_telemetry{};
    bool have_telemetry = false;
    float left_sps = 0.0f;
    float right_sps = 0.0f;
    double left_actual_steps = 0.0;
    double right_actual_steps = 0.0;
    uint64_t sim_time_us = 0;
    int steps_total = 0;
    int steps_done = 0;
    double max_abs_pitch_deg = 0.0;
    double max_abs_position_m = 0.0;
    std::deque<TailSample> tail_samples;
    std::array<double, 3> accel_bias_mps2{};
    std::array<double, 3> gyro_bias_rad_s{};
    PitchComplementaryFilter imu_filter{};
    std::mt19937 imu_rng{};
    std::normal_distribution<double> accel_noise{0.0, 0.0};
    std::normal_distribution<double> gyro_noise{0.0, 0.0};

    explicit ActiveRun(uint32_t id,
                       std::string pid_path,
                       std::array<ServiceDisturbance, ipc::kMaxSimDisturbances> disturbances_in,
                       BalancerSimulator&& sim_in)
        : run_id(id),
          pid_config_path(std::move(pid_path)),
          disturbances(std::move(disturbances_in)),
          sim(std::move(sim_in)) {}
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

    BalancerSimulator::Config sim_cfg;
    sim_cfg.physics_profile = profile;
    sim_cfg.initial_pitch_deg = request.initial_pitch_deg;
    sim_cfg.com_angle_offset_rad = request.com_angle_offset_rad;
    sim_cfg.wheel_slip_factor = request.wheel_slip_factor;
    sim_cfg.velocity_feedback_scale = request.velocity_feedback_scale;
    sim_cfg.velocity_feedback_tau_s = request.velocity_feedback_tau_s;
    sim_cfg.imu_pitch_lag_s = request.imu_pitch_lag_s;
    sim_cfg.imu_noise_seed = request.imu_noise_seed;
    sim_cfg.accel_noise_std_mps2 = request.accel_noise_std_mps2;
    sim_cfg.gyro_noise_std_rad_s = request.gyro_noise_std_rad_s;
    sim_cfg.accel_bias_mps2 = request.accel_bias_mps2;
    sim_cfg.gyro_bias_rad_s = request.gyro_bias_rad_s;
    BalancerSimulator sim(sim_cfg);

    std::array<ServiceDisturbance, ipc::kMaxSimDisturbances> disturbances{};
    for (std::size_t i = 0; i < disturbances.size(); ++i) {
      const auto& wire = request.disturbances[i];
      disturbances[i] = ServiceDisturbance{
          .kind = wire.kind,
          .start_s = wire.start_s,
          .duration_s = wire.duration_s,
          .force_n = wire.force_n,
          .com_bias_rad = wire.com_bias_rad,
          .force_n_end = wire.force_n_end,
          .com_bias_rad_end = wire.com_bias_rad_end,
      };
    }

    run_.emplace(request.run_id, pid_path, disturbances, std::move(sim));
    run_->steps_total = std::max(1, static_cast<int>(std::llround(request.duration_s / kTickDtS)));
    run_->max_abs_pitch_deg = std::abs(run_->sim.get_pitch()) * 180.0 / kPi;
    run_->accel_bias_mps2 = request.accel_bias_mps2;
    run_->gyro_bias_rad_s = request.gyro_bias_rad_s;
    run_->imu_rng.seed(request.imu_noise_seed);
    run_->accel_noise = std::normal_distribution<double>(0.0, request.accel_noise_std_mps2);
    run_->gyro_noise = std::normal_distribution<double>(0.0, request.gyro_noise_std_rad_s);

    run_->core.setMotorOutputs([this](float left, float right) {
      if (!run_.has_value()) {
        return;
      }
      run_->left_sps = left;
      run_->right_sps = right;
      run_->sim.set_motor_targets(left, right);
    });
    run_->core.setVelocityFeedback([this]() {
      return run_.has_value() ? run_->sim.get_actual_speed_sps() : 0.0f;
    });
    run_->core.setJoystick(JoyCmd{0.0f, 0.0f});
    run_->core.setPositionFeedback([this]() {
      return run_.has_value() ? static_cast<float>(run_->sim.get_position()) : 0.0f;
    });
    run_->core.setTelemetrySink([this](const Telemetry& t) {
      if (!run_.has_value()) {
        return;
      }
      run_->latest_telemetry = t;
      run_->have_telemetry = true;
    });

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

  DisturbanceSample disturbance_for_time(const ActiveRun& run, double sim_time_s) const {
    DisturbanceSample total{};
    for (const auto& disturbance : run.disturbances) {
      if (sim_time_s < disturbance.start_s) {
        continue;
      }
      switch (disturbance.kind) {
        case ipc::kSimDisturbanceStep:
          if (disturbance.duration_s > 0.0 &&
              sim_time_s < (disturbance.start_s + disturbance.duration_s)) {
            total.force_n += disturbance.force_n;
            total.com_bias_rad += disturbance.com_bias_rad;
          }
          break;
        case ipc::kSimDisturbanceRamp:
          if (disturbance.duration_s > 0.0 &&
              sim_time_s < (disturbance.start_s + disturbance.duration_s)) {
            const double progress =
                std::clamp((sim_time_s - disturbance.start_s) / disturbance.duration_s, 0.0, 1.0);
            total.force_n += disturbance.force_n +
                             (disturbance.force_n_end - disturbance.force_n) * progress;
            total.com_bias_rad += disturbance.com_bias_rad +
                                  (disturbance.com_bias_rad_end - disturbance.com_bias_rad) * progress;
          }
          break;
        case ipc::kSimDisturbanceHoldBias:
          if (disturbance.duration_s <= 0.0 ||
              sim_time_s < (disturbance.start_s + disturbance.duration_s)) {
            total.force_n += disturbance.force_n;
            total.com_bias_rad += disturbance.com_bias_rad;
          }
          break;
        default:
          break;
      }
    }
    return total;
  }

  void step_active_run() {
    ActiveRun& run = *run_;
    const double current_sim_time_s = static_cast<double>(run.sim_time_us) / 1e6;
    const DisturbanceSample disturbance = disturbance_for_time(run, current_sim_time_s);
    run.sim.set_external_force_n(disturbance.force_n);
    run.sim.set_external_com_bias_rad(disturbance.com_bias_rad);

    run.sim.step(kTickDtS);
    run.sim_time_us += static_cast<uint64_t>(kTickDtS * 1e6);
    ++run.steps_done;

    const auto imu = make_controller_imu(run);
    ImuSample sample{};
    sample.angle_rad = imu.pitch_rad;
    sample.gyro_rad_s = imu.filtered_pitch_rate_rad_s;
    sample.yaw_rate_z = imu.gyr[2];
    sample.t = std::chrono::steady_clock::time_point(std::chrono::microseconds(imu.timestamp_us));
    run.core.pushImu(sample);
    run.core.step(kTickDtS, sample.t);
    run.left_actual_steps += static_cast<double>(run.left_sps) * kTickDtS;
    run.right_actual_steps += static_cast<double>(run.right_sps) * kTickDtS;

    if ((run.steps_done % kTelemetryStride) == 0 || run.steps_done == run.steps_total) {
      publish_telemetry(run, sample, imu);
    }

    const double plant_pitch_deg = run.sim.get_pitch() * 180.0 / kPi;
    run.max_abs_pitch_deg = std::max(run.max_abs_pitch_deg, std::abs(plant_pitch_deg));
    run.max_abs_position_m = std::max(run.max_abs_position_m, std::abs(run.sim.get_position()));

    if (run.max_abs_pitch_deg > kFallPitchDeg) {
      finish_active_run(kDoneFell);
      return;
    }

    if (run.steps_done >= run.steps_total) {
      finish_active_run(kDoneCompleted);
    }
  }

  static float accel_pitch_deg(const std::array<double, 3>& acc) {
    return static_cast<float>(
        std::atan2(-acc[0], std::sqrt(acc[1] * acc[1] + acc[2] * acc[2])) * (180.0 / kPi));
  }

  ipc::ImuSamplePayload make_controller_imu(ActiveRun& run) {
    return run.sim.make_imu_payload(run.sim_time_us);
  }

  void publish_telemetry(ActiveRun& run, const ImuSample& sample, const ipc::ImuSamplePayload& imu) {
    ipc::SystemTelemetryPayload payload{};
    const auto& diag = run.sim.diagnostics();
    const auto& state = run.sim.state();

    payload.run_id = run.run_id;
    payload.sim_time_s = static_cast<float>(static_cast<double>(run.sim_time_us) / 1e6);
    payload.t_sec = run.have_telemetry ? static_cast<float>(run.latest_telemetry.t_sec) : payload.sim_time_s;
    payload.age_ms = run.have_telemetry
                         ? static_cast<float>(run.latest_telemetry.age_ms)
                         : static_cast<float>(
                               std::chrono::duration<double, std::milli>(
                                   std::chrono::steady_clock::time_point(std::chrono::microseconds(run.sim_time_us)) -
                                   sample.t)
                                   .count());
    payload.pitch_deg = run.have_telemetry ? static_cast<float>(run.latest_telemetry.pitch_deg)
                                           : static_cast<float>(state.pitch * 180.0 / kPi);
    payload.pitch_rate_dps =
        run.have_telemetry ? static_cast<float>(run.latest_telemetry.pitch_rate_dps)
                           : static_cast<float>(state.pitch_rate * 180.0 / kPi);
    payload.raw_acc_pitch_deg = accel_pitch_deg(imu.acc);
    payload.fused_pitch_deg = static_cast<float>(sample.angle_rad * 180.0 / kPi);
    payload.gyro_pitch_rate_dps = static_cast<float>(imu.gyr[1] * 180.0 / kPi);
    payload.filtered_pitch_rate_dps =
        run.have_telemetry ? static_cast<float>(run.latest_telemetry.filtered_pitch_rate_dps)
                           : static_cast<float>(imu.filtered_pitch_rate_rad_s * 180.0 / kPi);
    payload.rate_sp_dps = run.have_telemetry ? static_cast<float>(run.latest_telemetry.rate_sp_dps) : 0.0f;
    payload.out_norm = run.have_telemetry ? static_cast<float>(run.latest_telemetry.out_norm) : 0.0f;
    payload.u_sps = run.have_telemetry ? static_cast<float>(run.latest_telemetry.u_sps) : 0.0f;
    payload.integ_pitch = run.have_telemetry ? static_cast<float>(run.latest_telemetry.integ_pitch) : 0.0f;
    payload.vel_error = run.have_telemetry ? static_cast<float>(run.latest_telemetry.vel_error) : 0.0f;
    payload.vel_p_term = run.have_telemetry ? static_cast<float>(run.latest_telemetry.vel_p_term) : 0.0f;
    payload.vel_i_term = run.have_telemetry ? static_cast<float>(run.latest_telemetry.vel_i_term) : 0.0f;
    payload.target_vel_sps =
        run.have_telemetry ? static_cast<float>(run.latest_telemetry.target_vel_sps) : 0.0f;
    payload.measured_vel_sps =
        run.have_telemetry ? static_cast<float>(run.latest_telemetry.measured_vel_sps) : 0.0f;
    payload.filtered_vel_sps =
        run.have_telemetry ? static_cast<float>(run.latest_telemetry.filtered_vel_sps) : 0.0f;
    payload.position_target_vel_sps =
        run.have_telemetry ? static_cast<float>(run.latest_telemetry.position_target_vel_sps) : 0.0f;
    payload.pitch_ref_from_vel_deg =
        run.have_telemetry ? static_cast<float>(run.latest_telemetry.pitch_ref_from_vel_deg) : 0.0f;
    payload.pitch_ref_from_pos_deg =
        run.have_telemetry ? static_cast<float>(run.latest_telemetry.pitch_ref_from_pos_deg) : 0.0f;
    payload.pitch_sp_deg = run.have_telemetry ? static_cast<float>(run.latest_telemetry.pitch_sp_deg) : 0.0f;
    payload.effective_pitch_sp_deg =
        run.have_telemetry ? static_cast<float>(run.latest_telemetry.effective_pitch_sp_deg) : 0.0f;
    payload.pitch_error_deg =
        run.have_telemetry ? static_cast<float>(run.latest_telemetry.pitch_error_deg)
                           : (payload.pitch_sp_deg - payload.pitch_deg);
    payload.rate_error_dps =
        run.have_telemetry ? static_cast<float>(run.latest_telemetry.rate_error_dps)
                           : (payload.rate_sp_dps - payload.filtered_pitch_rate_dps);
    payload.pitch_trim_deg =
        run.have_telemetry ? static_cast<float>(run.latest_telemetry.pitch_trim_deg) : 0.0f;
    payload.trim_active = run.have_telemetry ? static_cast<float>(run.latest_telemetry.trim_active) : 0.0f;
    payload.command_saturated =
        run.have_telemetry ? static_cast<float>(run.latest_telemetry.command_saturated)
                           : (diag.command_saturated ? 1.0f : 0.0f);
    payload.left_applied_sps = run.left_sps;
    payload.right_applied_sps = run.right_sps;
    payload.left_actual_steps = static_cast<int64_t>(std::llround(run.left_actual_steps));
    payload.right_actual_steps = static_cast<int64_t>(std::llround(run.right_actual_steps));
    payload.plant_pitch_deg = static_cast<float>(state.pitch * 180.0 / kPi);
    payload.plant_pitch_rate_dps = static_cast<float>(state.pitch_rate * 180.0 / kPi);
    payload.plant_position_m = static_cast<float>(state.position);
    payload.plant_velocity_mps = static_cast<float>(state.velocity);
    payload.target_wheel_velocity = static_cast<float>(diag.target_wheel_velocity);
    payload.actual_wheel_velocity = static_cast<float>(diag.actual_wheel_velocity);
    payload.plant_velocity_error = static_cast<float>(diag.velocity_error);
    payload.f_cmd = static_cast<float>(diag.f_cmd);
    payload.f_app = static_cast<float>(diag.f_app);
    payload.external_force_n = static_cast<float>(diag.external_force_n);
    payload.external_com_bias_rad = static_cast<float>(diag.external_com_bias_rad);
    payload.x_ddot = static_cast<float>(diag.x_ddot);
    payload.theta_ddot = static_cast<float>(diag.theta_ddot);
    payload.force_saturated = diag.command_saturated ? 1.0f : 0.0f;
    endpoint_.send(active_peer_, MsgId::SystemTelemetry, payload);

    run.tail_samples.push_back(TailSample{
        .pitch_deg = payload.plant_pitch_deg,
        .velocity_mps = payload.plant_velocity_mps,
        .force_saturated = payload.force_saturated,
    });
    while (run.tail_samples.size() > kTailWindowSamples) {
      run.tail_samples.pop_front();
    }
  }

  RunSummary summarize(const ActiveRun& run) const {
    RunSummary out{};
    out.sample_count = static_cast<uint32_t>(run.steps_done);
    out.final_pitch_deg = run.sim.get_pitch() * 180.0 / kPi;
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
    ipc::SimRunDonePayload done{};
    done.run_id = run_->run_id;
    done.reason_code = reason_code;
    done.sample_count = summary.sample_count;
    done.elapsed_s = static_cast<float>(static_cast<double>(run_->sim_time_us) / 1e6);
    done.final_pitch_deg = static_cast<float>(summary.final_pitch_deg);
    done.max_abs_pitch_deg = static_cast<float>(summary.max_abs_pitch_deg);
    done.tail_rms_pitch_deg = static_cast<float>(summary.tail_rms_pitch_deg);
    done.tail_rail_fraction = static_cast<float>(summary.tail_rail_fraction);
    done.tail_mean_abs_pitch_deg = static_cast<float>(summary.tail_mean_abs_pitch_deg);
    done.max_abs_position_m = static_cast<float>(summary.max_abs_position_m);
    done.tail_mean_abs_velocity_mps = static_cast<float>(summary.tail_mean_abs_velocity_mps);
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

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  uint16_t port = 9001;
  std::string pid_config_path = ConfigPid::resolve_path("pid_sim.conf");

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--port" && (i + 1) < argc) {
      port = static_cast<uint16_t>(std::stoul(argv[++i]));
    } else if (arg == "--pid-config" && (i + 1) < argc) {
      pid_config_path = argv[++i];
    } else if (arg == "--help") {
      std::cout << "Usage: balancer_simulator [--port <udp-port>] [--pid-config <path>]\n";
      return 0;
    } else {
      std::cerr << "Unknown argument: " << arg << std::endl;
      return 1;
    }
  }

  try {
    SimulatorService service(port, pid_config_path);
    service.run();
    return 0;
  } catch (const std::exception& exc) {
    std::cerr << "Simulator service failed: " << exc.what() << std::endl;
    return 1;
  }
}
