// pitch_lpf.cpp
#include "pitch_lpf.h"

#include <algorithm>
#include <cmath>

#include "services/main/config.h"

namespace {

// Normal balancing keeps the original slow accelerometer correction because
// longitudinal acceleration is indistinguishable from tilt. If a large
// gravity disagreement persists while angular motion is quiet, smoothly ramp
// to a stronger correction so a missed/handled rotation cannot survive as a
// false lean for tens of seconds.
constexpr double kGravityErrorLpfHz = 1.0;
constexpr double kGravityRecoveryDwellS = 0.5;
constexpr double kGravityRecoveryRampS = 0.5;
constexpr double kGravityRecoveryCorrectionHz = 0.5;
constexpr double kGravityRecoveryEnterErrorRad = 5.0 * M_PI / 180.0;
constexpr double kGravityRecoveryExitErrorRad = 1.0 * M_PI / 180.0;
constexpr double kGravityRecoveryMaxGyroRateRadS = 10.0 * M_PI / 180.0;

}  // namespace

PitchComplementaryFilter::PitchComplementaryFilter() {
  push_data = [this](const Acc3& acc, const Gyr3& gyrv, TimePoint ts) {
    this->push_sample(acc, gyrv, ts);
  };
}

void PitchComplementaryFilter::push_sample(const Acc3& acc, const Gyr3& gyrv, TimePoint ts) {
  const double dt = compute_dt(ts);
  // Debug
  // printf("LPF Raw Acc: %.2f %.2f %.2f\n", acc[0], acc[1], acc[2]);
  const double a_acc = exp_coeff(Config::fc_acc_prefilt_hz, dt);
  const double a_gyr = exp_coeff(Config::fc_gyro_lpf_hz, dt);
  const double a_gyr_accel = exp_coeff(Config::fc_gyro_accel_lpf_hz, dt);

  if (!init_) {
    acc_f_ = acc;
    pitch_ = acc_pitch(acc);
    gyro_lpf_ = gyrv[1];
    gyro_accel_lpf_ = 0.0;
    prev_gyro_lpf_ = gyro_lpf_;
    have_prev_gyro_lpf_ = true;
    have_gravity_error_lpf_ = true;
    gravity_error_lpf_ = 0.0;
    gravity_recovery_elapsed_s_ = 0.0;
    gravity_recovery_active_ = false;
    init_ = true;
    pub_.t = ts;
    publish(pitch_, gyro_lpf_, gyro_accel_lpf_, gyrv[2], ts);
    return;
  }

  // 1) Prefilter accel
  for (int i = 0; i < 3; ++i) {
    acc_f_[i] = a_acc * acc_f_[i] + (1.0 - a_acc) * acc[i];
  }

  // 2) LPF gyro. The mounted axis is fixed and gyro bias is intentionally not
  // learned online: a slowly changing bias state must not become an attitude
  // reference or absorb a real control motion.
  gyro_lpf_ = a_gyr * gyro_lpf_ + (1.0 - a_gyr) * gyrv[1];
  const double gyro_accel = have_prev_gyro_lpf_ ? (gyro_lpf_ - prev_gyro_lpf_) / dt : 0.0;
  gyro_accel_lpf_ = a_gyr_accel * gyro_accel_lpf_ + (1.0 - a_gyr_accel) * gyro_accel;
  prev_gyro_lpf_ = gyro_lpf_;
  have_prev_gyro_lpf_ = true;

  // 3) Predict with gyro
  const double pred = wrap_pi(pitch_ + gyro_lpf_ * dt);

  // 4) Compute the signed gravity angle. Fuse on the circle so crossing the
  // +/-pi boundary cannot turn a small orientation correction into a full
  // revolution. Large innovations are rate-limited rather than rejected;
  // gravity therefore remains a continuous long-term reference.
  const double ap = acc_pitch(acc_f_);  // [-pi, pi]
  if (accel_reliable(acc_f_)) {
    const double gravity_error = wrap_pi(ap - pred);
    const double a_gravity_error = exp_coeff(kGravityErrorLpfHz, dt);
    if (!have_gravity_error_lpf_) {
      gravity_error_lpf_ = gravity_error;
      have_gravity_error_lpf_ = true;
    } else {
      gravity_error_lpf_ = wrap_pi(
          gravity_error_lpf_ +
          (1.0 - a_gravity_error) * wrap_pi(gravity_error - gravity_error_lpf_));
    }

    const bool angular_motion_quiet =
        std::abs(gyro_lpf_) <= kGravityRecoveryMaxGyroRateRadS;
    if (!gravity_recovery_active_) {
      if (angular_motion_quiet &&
          std::abs(gravity_error_lpf_) >= kGravityRecoveryEnterErrorRad) {
        gravity_recovery_elapsed_s_ += dt;
        if (gravity_recovery_elapsed_s_ >= kGravityRecoveryDwellS) {
          gravity_recovery_active_ = true;
        }
      } else {
        gravity_recovery_elapsed_s_ = 0.0;
      }
    } else if (!angular_motion_quiet ||
               std::abs(gravity_error_lpf_) <= kGravityRecoveryExitErrorRad) {
      gravity_recovery_active_ = false;
      gravity_recovery_elapsed_s_ = 0.0;
    } else {
      gravity_recovery_elapsed_s_ += dt;
    }

    const double recovery_weight =
        gravity_recovery_active_
            ? std::clamp((gravity_recovery_elapsed_s_ - kGravityRecoveryDwellS) /
                             kGravityRecoveryRampS,
                         0.0, 1.0)
            : 0.0;
    const double correction_hz =
        Config::fc_acc_corr_hz +
        recovery_weight * (kGravityRecoveryCorrectionHz - Config::fc_acc_corr_hz);
    const double correction_error =
        wrap_pi(gravity_error +
                recovery_weight * wrap_pi(gravity_error_lpf_ - gravity_error));
    const double max_innovation_rad =
        Config::accel_correction_max_innovation_deg * M_PI / 180.0;
    const double innovation =
        std::clamp(correction_error, -max_innovation_rad, max_innovation_rad);
    pitch_ = wrap_pi(pred + (1.0 - exp_coeff(correction_hz, dt)) * innovation);
  } else {
    have_gravity_error_lpf_ = false;
    gravity_recovery_elapsed_s_ = 0.0;
    gravity_recovery_active_ = false;
    pitch_ = pred;  // gyro-only during high dynamics
  }

  publish(pitch_, gyro_lpf_, gyro_accel_lpf_, gyrv[2], ts);
}

ImuSample PitchComplementaryFilter::read_latest() const {
  ImuSample out;
  for (;;) {
    uint64_t s1 = seq_.load(std::memory_order_acquire);
    if (s1 & 1u) continue;
    out = pub_;
    uint64_t s2 = seq_.load(std::memory_order_acquire);
    if (s1 == s2 && !(s2 & 1u)) break;
  }
  return out;
}

void PitchComplementaryFilter::reset() {
  init_ = false;
  have_last_ts_ = false;
  acc_f_ = {0.0, 0.0, 0.0};
  pitch_ = 0.0;
  gyro_lpf_ = 0.0;
  gyro_accel_lpf_ = 0.0;
  have_prev_gyro_lpf_ = false;
  prev_gyro_lpf_ = 0.0;
  have_gravity_error_lpf_ = false;
  gravity_error_lpf_ = 0.0;
  gravity_recovery_elapsed_s_ = 0.0;
  gravity_recovery_active_ = false;
  uint64_t s = seq_.load(std::memory_order_relaxed);
  seq_.store(s + 1, std::memory_order_release);
  pub_ = ImuSample{};
  seq_.store(s + 2, std::memory_order_release);
}

// ----- helpers -----
double PitchComplementaryFilter::exp_coeff(double fc_hz, double dt) {
  return std::exp(-2.0 * M_PI * std::max(0.0, fc_hz) * dt);
}

double PitchComplementaryFilter::compute_dt(TimePoint ts) {
  double dt;
  if (!have_last_ts_) {
    dt = Config::fallback_dt_s;
    last_ts_ = ts;
    have_last_ts_ = true;
  } else {
    dt = std::chrono::duration<double>(ts - last_ts_).count();
    if (dt <= 0.0 || dt > 1.0) dt = Config::fallback_dt_s;
    last_ts_ = ts;
  }
  return dt;
}

double PitchComplementaryFilter::wrap_pi(double x) {
  while (x > M_PI) x -= 2 * M_PI;
  while (x < -M_PI) x += 2 * M_PI;
  return x;
}

double PitchComplementaryFilter::norm(const Acc3& v) {
  return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

double PitchComplementaryFilter::acc_pitch(const Acc3& a) {
  // The configured hardware axis map presents gravity on mounted -Z when
  // upright. Negating Z makes upright zero while retaining signed full-circle
  // pitch and the existing positive-pitch direction.
  return std::atan2(-a[0], -a[2]);
}

bool PitchComplementaryFilter::accel_reliable(const Acc3& a) {
  const double n = norm(a);
  const double g_lo = Config::g0 * (1.0 - Config::g_band_rel);
  const double g_hi = Config::g0 * (1.0 + Config::g_band_rel);
  return n >= g_lo && n <= g_hi;  // reject high linear acceleration and free-fall
}

void PitchComplementaryFilter::publish(double pitch, double gyro_pitch, double gyro_pitch_accel,
                                       double yaw_rate_z, TimePoint ts) {
  uint64_t s = seq_.load(std::memory_order_relaxed);
  seq_.store(s + 1, std::memory_order_release);  // begin publish
  pub_.t = ts;
  pub_.angle_rad = pitch;
  pub_.gyro_rad_s = gyro_pitch;
  pub_.pitch_accel_rad_s2 = gyro_pitch_accel;
  pub_.yaw_rate_z = yaw_rate_z;
  seq_.store(s + 2, std::memory_order_release);  // end publish
}
