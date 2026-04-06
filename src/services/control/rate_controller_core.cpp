#include "rate_controller_core.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "config.h"

#include <uORB/topics/rate_ctrl_status.h>

#include <matrix/matrix/math.hpp>
#include <pid/PID.hpp>
#include <rate_control.hpp>

using matrix::Vector3f;

namespace {

constexpr float kVelocityLoopTiltPriorityRad = static_cast<float>(3.0 * M_PI / 180.0);
constexpr float kPositionHoldEnablePitchRad = static_cast<float>(3.0 * M_PI / 180.0);
constexpr float kMaxPositionTargetSps = 800.0f;
constexpr float kMaxPositionTrimPitchBiasRad = 0.15f;
constexpr float kLeanTrimEnablePitchRad = static_cast<float>(10.0 * M_PI / 180.0);
constexpr float kLeanTrimResetPitchRad = static_cast<float>(20.0 * M_PI / 180.0);
constexpr float kLeanTrimVelocityDeadbandSps = 25.0f;
constexpr float kLeanTrimVelocityFcHz = 0.5f;

}  // namespace

struct RateControllerCore::Impl {
  std::function<void(float, float)> motors_cb;
  std::function<void(const Telemetry&)> tel_cb;
  std::function<float()> velocity_cb;
  std::function<float()> position_cb;

  ImuSample latest_imu{};
  JoyCmd latest_joy{0.0f, 0.0f};
  bool have_imu{false};
  bool initialized{false};

  std::chrono::steady_clock::time_point start_ts{};
  RateControl rc{};
  PID velocity_pid{};

  float pitch_setpoint_rad{0.0f};
  float filtered_velocity_sps{0.0f};
  float filtered_trim_velocity_sps{0.0f};
  float position_anchor_m{0.0f};
  float angle_trim_pitch_bias_rad{0.0f};
  float lean_trim_rad{0.0f};
  bool lean_trim_active{false};
  bool position_anchor_initialized{false};
};

RateControllerCore::RateControllerCore() : p_(new Impl) {
  p_->rc.setPidGains(
      Vector3f(0.f, ConfigPid::rate_P, 0.f),
      Vector3f(0.f, ConfigPid::rate_I, 0.f),
      Vector3f(0.f, ConfigPid::rate_D, 0.f));
  p_->rc.setIntegratorLimit(Vector3f(0.f, ConfigPid::rate_I_lim, 0.f));
  p_->rc.setFeedForwardGain(Vector3f(0.f, ConfigPid::rate_FF, 0.f));

  p_->velocity_pid.setGains(ConfigPid::vel_P, ConfigPid::vel_I, ConfigPid::vel_D);
  p_->velocity_pid.setIntegralLimit(ConfigPid::vel_I_lim);
  p_->velocity_pid.setOutputLimit(kMaxPitchSetpointRad);
  p_->velocity_pid.setSetpoint(0.0f);
}

RateControllerCore::~RateControllerCore() {
  stop();
  delete p_;
}

void RateControllerCore::start() {}

void RateControllerCore::stop() {}

void RateControllerCore::step(double dt_s, std::chrono::steady_clock::time_point now) {
  if (!p_->have_imu) {
    return;
  }

  const float dt = std::clamp(static_cast<float>(dt_s), 1.0f / 2000.0f, 0.05f);
  if (!p_->initialized) {
    p_->initialized = true;
    p_->start_ts = now;
  }

  const float pitch_rad = static_cast<float>(p_->latest_imu.angle_rad);
  const float gyro_rad_s = static_cast<float>(p_->latest_imu.gyro_rad_s);
  const float measured_velocity_sps = p_->velocity_cb ? p_->velocity_cb() : 0.0f;
  const float velocity_alpha = std::clamp(
      static_cast<float>((2.0 * M_PI * Config::fc_velocity_hz * dt) /
                         (1.0 + 2.0 * M_PI * Config::fc_velocity_hz * dt)),
      0.0f,
      1.0f);
  p_->filtered_velocity_sps += velocity_alpha * (measured_velocity_sps - p_->filtered_velocity_sps);
  const float current_velocity_sps = p_->filtered_velocity_sps;
  const float trim_velocity_alpha = std::clamp(
      static_cast<float>((2.0 * M_PI * kLeanTrimVelocityFcHz * dt) /
                         (1.0 + 2.0 * M_PI * kLeanTrimVelocityFcHz * dt)),
      0.0f,
      1.0f);
  p_->filtered_trim_velocity_sps +=
      trim_velocity_alpha * (measured_velocity_sps - p_->filtered_trim_velocity_sps);
  const bool have_position_feedback = static_cast<bool>(p_->position_cb);
  const float current_position_m = have_position_feedback ? p_->position_cb() : 0.0f;
  const float lean_trim_max_rad = static_cast<float>(
      std::clamp(ConfigPid::lean_trim_max_deg * M_PI / 180.0, 0.0, kMaxPitchSetpointRad));

  const JoyCmd joy = p_->latest_joy;
  const bool user_velocity_active = std::abs(joy.forward) > Config::deadzone;
  if (have_position_feedback) {
    if (!p_->position_anchor_initialized) {
      p_->position_anchor_m = current_position_m;
      p_->position_anchor_initialized = true;
    }
    if (user_velocity_active) {
      p_->position_anchor_m = current_position_m;
      p_->angle_trim_pitch_bias_rad = 0.0f;
    }
  }

  float position_target_vel_sps = 0.0f;
  if (have_position_feedback && !user_velocity_active && ConfigPid::pos_P != 0.0 &&
      std::abs(pitch_rad) < kPositionHoldEnablePitchRad) {
    const float position_error_m = p_->position_anchor_m - current_position_m;
    position_target_vel_sps = std::clamp(
        static_cast<float>(ConfigPid::pos_P * position_error_m),
        -kMaxPositionTargetSps,
        kMaxPositionTargetSps);
  }

  // Let balance recovery own the zero-command case. The velocity loop only
  // participates for operator translation or explicit position-hold correction.
  const bool enable_velocity_loop =
      user_velocity_active || std::abs(position_target_vel_sps) > 1e-3f;
  float target_vel_sps =
      user_velocity_active ? joy.forward * static_cast<float>(kMaxSps) : 0.0f;
  target_vel_sps += position_target_vel_sps;
  p_->velocity_pid.setSetpoint(target_vel_sps);

  const float velocity_loop_blend = std::clamp(
      1.0f - (std::abs(pitch_rad) / kVelocityLoopTiltPriorityRad),
      0.0f,
      1.0f);
  const bool update_velocity_integral = enable_velocity_loop && velocity_loop_blend > 0.25f;
  if (!update_velocity_integral) {
    p_->velocity_pid.resetIntegral();
  }

  const float velocity_pitch_setpoint_rad = enable_velocity_loop
                                                ? p_->velocity_pid.update(
                                                      current_velocity_sps, dt, update_velocity_integral)
                                                : 0.0f;
  if (!user_velocity_active && ConfigPid::angle_I != 0.0 &&
      std::abs(pitch_rad) < kPositionHoldEnablePitchRad) {
    p_->angle_trim_pitch_bias_rad = std::clamp(
        p_->angle_trim_pitch_bias_rad - static_cast<float>(ConfigPid::angle_I * pitch_rad * dt),
        -kMaxPositionTrimPitchBiasRad,
        kMaxPositionTrimPitchBiasRad);
  }

  p_->pitch_setpoint_rad =
      velocity_pitch_setpoint_rad * velocity_loop_blend + p_->angle_trim_pitch_bias_rad + p_->lean_trim_rad;
  p_->pitch_setpoint_rad = std::clamp(p_->pitch_setpoint_rad,
                                      -static_cast<float>(kMaxPitchSetpointRad),
                                      static_cast<float>(kMaxPitchSetpointRad));

  const float pitch_error_rad = p_->pitch_setpoint_rad - pitch_rad;
  const float rate_sp_rad_s = static_cast<float>(ConfigPid::angle_to_rate_k * pitch_error_rad);

  const Vector3f rate{0.f, gyro_rad_s, 0.f};
  const Vector3f rate_sp{0.f, rate_sp_rad_s, 0.f};
  const Vector3f ang_acc{0.f, 0.f, 0.f};
  const Vector3f u = p_->rc.update(rate, rate_sp, ang_acc, dt, false);

  float u_sps = u(1) * static_cast<float>(kPitchOutToSps);
  u_sps = std::clamp(u_sps, -static_cast<float>(kMaxSps), static_cast<float>(kMaxSps));
  const bool command_saturated = std::abs(u_sps) >= (0.99f * static_cast<float>(kMaxSps));

  if (std::abs(pitch_rad) > kLeanTrimResetPitchRad) {
    p_->lean_trim_rad = 0.0f;
    p_->lean_trim_active = false;
  } else {
    const bool lean_trim_window = !user_velocity_active && std::abs(pitch_rad) < kLeanTrimEnablePitchRad;
    if (ConfigPid::lean_trim_I != 0.0 && lean_trim_window && !command_saturated) {
      const float velocity_for_trim = std::abs(p_->filtered_trim_velocity_sps) > kLeanTrimVelocityDeadbandSps
                                          ? p_->filtered_trim_velocity_sps
                                          : 0.0f;
      const float normalized_effort =
          std::clamp(-velocity_for_trim / static_cast<float>(kMaxSps), -1.0f, 1.0f);
      p_->lean_trim_rad = std::clamp(
          p_->lean_trim_rad + static_cast<float>(ConfigPid::lean_trim_I * normalized_effort * dt),
          -lean_trim_max_rad,
          lean_trim_max_rad);
      p_->lean_trim_active = true;
    } else {
      p_->lean_trim_active = false;
      if (ConfigPid::lean_trim_decay_s > 0.0) {
        const float decay = std::clamp(
            static_cast<float>(dt / ConfigPid::lean_trim_decay_s),
            0.0f,
            1.0f);
        p_->lean_trim_rad *= (1.0f - decay);
      }
    }
  }

  const float turn_sps = joy.turn * static_cast<float>(kMaxSps) * 0.5f;
  const float left_sps = u_sps + turn_sps;
  const float right_sps = u_sps - turn_sps;

  if (p_->motors_cb) {
    p_->motors_cb(left_sps, right_sps);
  }

  if (p_->tel_cb) {
    rate_ctrl_status_s st{};
    p_->rc.getRateControlStatus(st);

    Telemetry t{};
    t.t_sec = std::chrono::duration<double>(now - p_->start_ts).count();
    t.age_ms = std::chrono::duration<double, std::milli>(now - p_->latest_imu.t).count();
    t.pitch_deg = pitch_rad * 180.0 / M_PI;
    t.pitch_rate_dps = gyro_rad_s * 180.0 / M_PI;
    t.rate_sp_dps = rate_sp_rad_s * 180.0 / M_PI;
    t.out_norm = u(1);
    t.u_sps = u_sps;
    t.integ_pitch = st.pitchspeed_integ;
    t.vel_error = enable_velocity_loop ? (target_vel_sps - current_velocity_sps) : 0.0f;
    t.vel_i_term = p_->velocity_pid.getIntegral();
    t.vel_p_term = enable_velocity_loop ? (t.vel_error * ConfigPid::vel_P) : 0.0f;
    t.pitch_sp_deg = p_->pitch_setpoint_rad * 180.0 / M_PI;
    t.effective_pitch_sp_deg = p_->pitch_setpoint_rad * 180.0 / M_PI;
    t.pitch_trim_deg = p_->lean_trim_rad * 180.0 / M_PI;
    t.trim_active = p_->lean_trim_active ? 1.0 : 0.0;

    p_->tel_cb(std::move(t));
  }
}

void RateControllerCore::pushImu(const ImuSample& s) {
  p_->latest_imu = s;
  p_->have_imu = true;
}

void RateControllerCore::setJoystick(const JoyCmd& j) {
  p_->latest_joy = j;
}

void RateControllerCore::setTelemetrySink(std::function<void(const Telemetry&)> cb) {
  p_->tel_cb = std::move(cb);
}

void RateControllerCore::setMotorOutputs(std::function<void(float, float)> motors_cb) {
  p_->motors_cb = std::move(motors_cb);
}

void RateControllerCore::setVelocityFeedback(std::function<float()> velocity_cb) {
  p_->velocity_cb = std::move(velocity_cb);
}

void RateControllerCore::setPositionFeedback(std::function<float()> position_cb) {
  p_->position_cb = std::move(position_cb);
}
