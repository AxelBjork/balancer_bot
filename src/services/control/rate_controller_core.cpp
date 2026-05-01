#include "rate_controller_core.h"

#include <uORB/topics/rate_ctrl_status.h>

#include <algorithm>
#include <cmath>
#include <matrix/matrix/math.hpp>
#include <rate_control.hpp>
#include <utility>

#include "config.h"

using matrix::Vector3f;

namespace {

constexpr double kPositionHoldEnablePitchRad = 3.0 * M_PI / 180.0;
constexpr double kMaxPositionTargetSps = 800.0;
constexpr double kMaxPositionTrimPitchBiasRad = 0.15;
constexpr double kLeanTrimEnablePitchRad = 10.0 * M_PI / 180.0;
constexpr double kLeanTrimResetPitchRad = 20.0 * M_PI / 180.0;
constexpr double kLeanTrimVelocityDeadbandSps = 10.0;
constexpr double kLeanTrimVelocityFcHz = 0.5;
constexpr double kLeanTrimNormalizationSps = 4000.0;

}  // namespace

struct RateControllerCore::Impl {
  std::function<void(double, double)> motors_cb;
  std::function<void(const Telemetry&)> tel_cb;
  std::function<double()> velocity_cb;
  std::function<double()> position_cb;

  ImuSample latest_imu{};
  JoyCmd latest_joy{0.0f, 0.0f};
  bool have_imu{false};
  bool initialized{false};

  std::chrono::steady_clock::time_point start_ts{};
  RateControl rc{};

  double pitch_setpoint_rad{0.0};
  double filtered_velocity_sps{0.0};
  double filtered_trim_velocity_sps{0.0};
  double position_anchor_m{0.0};
  double angle_trim_pitch_bias_rad{0.0};
  double lean_trim_rad{0.0};
  bool lean_trim_active{false};
  bool position_anchor_initialized{false};
};

RateControllerCore::RateControllerCore() : p_(new Impl) {
  p_->rc.setPidGains(Vector3f(0.0f, static_cast<float>(ConfigPid::rate_P), 0.0f),
                     Vector3f(0.0f, static_cast<float>(ConfigPid::rate_I), 0.0f),
                     Vector3f(0.0f, static_cast<float>(ConfigPid::rate_D), 0.0f));
  p_->rc.setIntegratorLimit(Vector3f(0.0f, static_cast<float>(ConfigPid::rate_I_lim), 0.0f));
  p_->rc.setFeedForwardGain(Vector3f(0.0f, static_cast<float>(ConfigPid::rate_FF), 0.0f));
}

RateControllerCore::~RateControllerCore() {
  stop();
  delete p_;
}

void RateControllerCore::start() {
}

void RateControllerCore::stop() {
}

void RateControllerCore::step(double dt_s, std::chrono::steady_clock::time_point now) {
  if (!p_->have_imu) {
    return;
  }

  const double dt = std::clamp(dt_s, 1.0 / 2000.0, 0.05);
  if (!p_->initialized) {
    p_->initialized = true;
    p_->start_ts = now;
  }

  const double pitch_rad = p_->latest_imu.angle_rad;
  const double filtered_pitch_rate_rad_s = p_->latest_imu.gyro_rad_s;
  const double measured_velocity_sps = p_->velocity_cb ? p_->velocity_cb() : 0.0;
  const double velocity_alpha =
      std::clamp((2.0 * M_PI * Config::fc_velocity_hz * dt) / (1.0 + 2.0 * M_PI * Config::fc_velocity_hz * dt),
                 0.0, 1.0);
  p_->filtered_velocity_sps += velocity_alpha * (measured_velocity_sps - p_->filtered_velocity_sps);
  const double current_velocity_sps = p_->filtered_velocity_sps;
  const double trim_velocity_alpha =
      std::clamp((2.0 * M_PI * kLeanTrimVelocityFcHz * dt) / (1.0 + 2.0 * M_PI * kLeanTrimVelocityFcHz * dt),
                 0.0, 1.0);
  p_->filtered_trim_velocity_sps +=
      trim_velocity_alpha * (measured_velocity_sps - p_->filtered_trim_velocity_sps);
  const bool have_position_feedback = static_cast<bool>(p_->position_cb);
  const double current_position_m = have_position_feedback ? p_->position_cb() : 0.0;
  const double lean_trim_max_rad =
      std::clamp(ConfigPid::lean_trim_max_deg * M_PI / 180.0, 0.0, kMaxPitchSetpointRad);

  const JoyCmd joy = p_->latest_joy;
  const bool user_velocity_active = std::abs(joy.forward) > Config::deadzone;
  if (have_position_feedback) {
    if (!p_->position_anchor_initialized) {
      p_->position_anchor_m = current_position_m;
      p_->position_anchor_initialized = true;
    }
    if (user_velocity_active) {
      p_->position_anchor_m = current_position_m;
      p_->angle_trim_pitch_bias_rad = 0.0;
    }
  }

  const double base_target_vel_sps = user_velocity_active ? static_cast<double>(joy.forward) * kMaxSps : 0.0;
  double position_pitch_setpoint_rad = 0.0;
  const bool position_hold_enabled = have_position_feedback && !user_velocity_active &&
                                     ConfigPid::outer_k_pos != 0.0 &&
                                     std::abs(pitch_rad) < kPositionHoldEnablePitchRad;
  if (position_hold_enabled) {
    const double position_error_m = p_->position_anchor_m - current_position_m;
    position_pitch_setpoint_rad = ConfigPid::outer_k_pos * position_error_m;
  }
  double position_target_vel_sps = 0.0;
  if (std::abs(ConfigPid::outer_k_vel) > 1e-9) {
    position_target_vel_sps = std::clamp(position_pitch_setpoint_rad / ConfigPid::outer_k_vel,
                                         -kMaxPositionTargetSps, kMaxPositionTargetSps);
  }

  const double target_vel_sps = base_target_vel_sps;
  const double vel_error_sps = target_vel_sps - current_velocity_sps;
  const double velocity_pitch_setpoint_rad = ConfigPid::outer_k_vel * vel_error_sps;
  if (!user_velocity_active && ConfigPid::angle_I != 0.0 &&
      std::abs(pitch_rad) < kPositionHoldEnablePitchRad) {
    p_->angle_trim_pitch_bias_rad =
        std::clamp(p_->angle_trim_pitch_bias_rad - (ConfigPid::angle_I * pitch_rad * dt),
                   -kMaxPositionTrimPitchBiasRad, kMaxPositionTrimPitchBiasRad);
  }

  p_->pitch_setpoint_rad = velocity_pitch_setpoint_rad + position_pitch_setpoint_rad +
                           p_->angle_trim_pitch_bias_rad + p_->lean_trim_rad;
  const double pitch_setpoint_limit_rad =
      user_velocity_active ? kMaxPitchSetpointRad : Config::max_tilt_rad;
  p_->pitch_setpoint_rad =
      std::clamp(p_->pitch_setpoint_rad, -pitch_setpoint_limit_rad, pitch_setpoint_limit_rad);

  const double pitch_error_rad = p_->pitch_setpoint_rad - pitch_rad;
  const double rate_sp_rad_s = ConfigPid::outer_k_pitch * pitch_error_rad -
                               ConfigPid::outer_k_pitch_rate * filtered_pitch_rate_rad_s;
  const double rate_error_rad_s = rate_sp_rad_s - filtered_pitch_rate_rad_s;

  const Vector3f rate{0.0f, static_cast<float>(filtered_pitch_rate_rad_s), 0.0f};
  const Vector3f rate_sp{0.0f, static_cast<float>(rate_sp_rad_s), 0.0f};
  const Vector3f ang_acc{0.0f, 0.0f, 0.0f};
  const Vector3f u = p_->rc.update(rate, rate_sp, ang_acc, static_cast<float>(dt), false);

  double u_sps = static_cast<double>(u(1)) * kPitchOutToSps;
  u_sps = std::clamp(u_sps, -kMaxSps, kMaxSps);
  const bool command_saturated = std::abs(u_sps) >= (0.99 * kMaxSps);

  if (std::abs(pitch_rad) > kLeanTrimResetPitchRad) {
    p_->lean_trim_rad = 0.0;
    p_->lean_trim_active = false;
  } else if (user_velocity_active) {
    p_->lean_trim_active = false;
    if (ConfigPid::lean_trim_decay_s > 0.0) {
      const double decay = std::clamp(dt / ConfigPid::lean_trim_decay_s, 0.0, 1.0);
      p_->lean_trim_rad *= (1.0 - decay);
    }
  } else {
    const bool lean_trim_window = std::abs(pitch_rad) < kLeanTrimEnablePitchRad;
    if (ConfigPid::lean_trim_I != 0.0 && lean_trim_window && !command_saturated) {
      if (std::abs(p_->filtered_trim_velocity_sps) > kLeanTrimVelocityDeadbandSps) {
        const double normalized_effort =
            std::clamp(-p_->filtered_trim_velocity_sps / kLeanTrimNormalizationSps, -1.0, 1.0);
        p_->lean_trim_rad =
            std::clamp(p_->lean_trim_rad + (ConfigPid::lean_trim_I * normalized_effort * dt),
                       -lean_trim_max_rad, lean_trim_max_rad);
        p_->lean_trim_active = true;
      } else {
        p_->lean_trim_active = false;
      }
    } else {
      p_->lean_trim_active = false;
    }
  }

  const double turn_sps = static_cast<double>(joy.turn) * kMaxSps * 0.5;

  const double left_sps = u_sps + turn_sps;
  const double right_sps = u_sps - turn_sps;

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
    t.pitch_rate_dps = filtered_pitch_rate_rad_s * 180.0 / M_PI;
    t.filtered_pitch_rate_dps = filtered_pitch_rate_rad_s * 180.0 / M_PI;
    t.rate_sp_dps = rate_sp_rad_s * 180.0 / M_PI;
    t.out_norm = u(1);
    t.u_sps = u_sps;
    t.turn_sps = turn_sps;
    t.integ_pitch = st.pitchspeed_integ;
    t.vel_error = vel_error_sps;
    t.vel_i_term = p_->lean_trim_rad;
    t.vel_p_term = velocity_pitch_setpoint_rad;
    t.target_vel_sps = target_vel_sps;
    t.measured_vel_sps = measured_velocity_sps;
    t.filtered_vel_sps = current_velocity_sps;
    t.position_target_vel_sps = position_target_vel_sps;
    t.pitch_ref_from_vel_deg = velocity_pitch_setpoint_rad * 180.0 / M_PI;
    t.pitch_ref_from_pos_deg = position_pitch_setpoint_rad * 180.0 / M_PI;
    t.pitch_error_deg = pitch_error_rad * 180.0 / M_PI;
    t.rate_error_dps = rate_error_rad_s * 180.0 / M_PI;
    t.pitch_sp_deg = p_->pitch_setpoint_rad * 180.0 / M_PI;
    t.effective_pitch_sp_deg = p_->pitch_setpoint_rad * 180.0 / M_PI;
    t.pitch_trim_deg = p_->lean_trim_rad * 180.0 / M_PI;
    t.trim_active = p_->lean_trim_active ? 1.0 : 0.0;
    t.command_saturated = command_saturated ? 1.0 : 0.0;

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

void RateControllerCore::setMotorOutputs(std::function<void(double, double)> motors_cb) {
  p_->motors_cb = std::move(motors_cb);
}

void RateControllerCore::setVelocityFeedback(std::function<double()> velocity_cb) {
  p_->velocity_cb = std::move(velocity_cb);
}

void RateControllerCore::setPositionFeedback(std::function<double()> position_cb) {
  p_->position_cb = std::move(position_cb);
}
