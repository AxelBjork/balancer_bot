// PitchComplementaryFilter.hpp
#pragma once
#include "config.h"
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>

class PitchComplementaryFilter {
public:
  using Acc3 = std::array<double, 3>;
  using Gyr3 = std::array<double, 3>;
  using TimePoint = std::chrono::steady_clock::time_point;

  explicit PitchComplementaryFilter() {
    push_data = [this](const Acc3 &acc, const Gyr3 &gyrv, TimePoint ts) {
      this->push_sample(acc, gyrv, ts);
    };
  }

  // Writer (single producer)
  void push_sample(const Acc3 &acc, const Gyr3 &gyrv, TimePoint ts) {
    const double dt = compute_dt(ts);

    const double a_acc = exp_coeff(Config::fc_acc_prefilt_hz, dt);
    const double a_gyr = exp_coeff(Config::fc_gyro_lpf_hz, dt);
    const double a_corr = exp_coeff(Config::fc_acc_corr_hz, dt);
    const double a_bias = exp_coeff(Config::fc_gyro_bias_hz, dt);

    if (!init_) {
      acc_f_ = acc;
      pitch_ = acc_pitch(acc);
      gyro_lpf_ = gyrv[1];
      gyro_bias_ = 0.0;
      init_ = true;
      pub_.t = ts;
      publish(pitch_, gyro_lpf_, gyrv[2], ts);
      return;
    }

    // 1) Prefilter accel
    for (int i = 0; i < 3; ++i)
      acc_f_[i] = a_acc * acc_f_[i] + (1.0 - a_acc) * acc[i];

    // 2) LPF gyro + bias subtract
    gyro_lpf_ = a_gyr * gyro_lpf_ + (1.0 - a_gyr) * gyrv[1];
    const double gyro_corr = gyro_lpf_ - gyro_bias_;

    // 3) Predict with gyro
    const double pred = pitch_ + gyro_corr * dt;

    // 4) Compute accel pitch + decide whether to trust it
    const double ap = acc_pitch(acc_f_); // [-pi, pi]
    const bool accel_ok = accel_reliable(acc_f_, pred);

    // 5) Complementary blend (accel correction only when reliable)
    if (accel_ok) {
      pitch_ = a_corr * pred + (1.0 - a_corr) * ap;
    } else {
      pitch_ = pred; // gyro-only during high dynamics
    }

    // 6) Gyro bias learning when very still and estimates agree
    if (accel_ok && std::abs(rad2deg(gyro_lpf_)) < Config::still_max_rate_dps &&
        std::abs(rad2deg(wrap_pi(ap - pitch_))) < Config::still_max_err_deg) {
      gyro_bias_ = a_bias * gyro_bias_ + (1.0 - a_bias) * gyro_lpf_;
    }
    printf("Publish, accel_reliable %d, pitch pitch_rad=%.3f°, "
           "gyro_lpf=%.3f°/s, yaw=%.3f°/s\n",
           accel_ok ? 1 : 0, pitch_ * 180.0 / M_PI, gyro_lpf_ * 180.0 / M_PI,
           gyrv[2] * 180.0 / M_PI);

    publish(pitch_, gyro_lpf_, gyrv[2], ts);
  }

  // Adapter-friendly member
  std::function<void(const Acc3 &, const Gyr3 &, TimePoint)> push_data;

  // Readers (multi-consumer)
  ImuSample read_latest() const {
    ImuSample out;
    for (;;) {
      uint64_t s1 = seq_.load(std::memory_order_acquire);
      if (s1 & 1u)
        continue;
      out = pub_;
      uint64_t s2 = seq_.load(std::memory_order_acquire);
      if (s1 == s2 && !(s2 & 1u))
        break;
    }
    return out;
  }

  void reset() {
    init_ = false;
    have_last_ts_ = false;
    acc_f_ = {0.0, 0.0, 0.0};
    pitch_ = 0.0;
    gyro_lpf_ = 0.0;
    gyro_bias_ = 0.0;
    uint64_t s = seq_.load(std::memory_order_relaxed);
    seq_.store(s + 1, std::memory_order_release);
    pub_ = ImuSample{};
    seq_.store(s + 2, std::memory_order_release);
  }

private:
  bool init_{false};
  bool have_last_ts_{false};
  TimePoint last_ts_{};
  Acc3 acc_f_{0.0, 0.0, 0.0};
  double pitch_{0.0};     // fused estimate (rad, +forward)
  double gyro_lpf_{0.0};  // rad/s (pitch axis)
  double gyro_bias_{0.0}; // rad/s

  mutable std::atomic<uint64_t> seq_{0};
  alignas(64) ImuSample pub_{};

  // ----- helpers -----
  static inline double exp_coeff(double fc_hz, double dt) {
    return std::exp(-2.0 * M_PI * std::max(0.0, fc_hz) * dt);
  }

  double compute_dt(TimePoint ts) {
    double dt;
    if (!have_last_ts_) {
      dt = Config::fallback_dt_s;
      last_ts_ = ts;
      have_last_ts_ = true;
    } else {
      dt = std::chrono::duration<double>(ts - last_ts_).count();
      if (dt <= 0.0 || dt > 1.0)
        dt = Config::fallback_dt_s;
      last_ts_ = ts;
    }
    return dt;
  }

  static inline double rad2deg(double x) { return x * (180.0 / M_PI); }
  static inline double wrap_pi(double x) {
    while (x > M_PI)
      x -= 2 * M_PI;
    while (x < -M_PI)
      x += 2 * M_PI;
    return x;
  }

  static inline double norm(const Acc3 &v) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  }

  static inline double acc_pitch(const Acc3 &a) {
    const double ax = a[0], ay = a[1], az = a[2];
    return std::atan2(-ax,
                      std::sqrt(ay * ay + az * az)); // matches your convention
  }

  bool accel_reliable(const Acc3 &a, double pitch_pred) const {
    const double n = norm(a);
    const double g_lo = Config::g0 * (1.0 - Config::g_band_rel);
    const double g_hi = Config::g0 * (1.0 + Config::g_band_rel);
    if (!(n >= g_lo && n <= g_hi))
      return false; // high linear accel or free-fall
    if (std::abs(rad2deg(std::cos(pitch_pred))) < 1e-6)
      return false; // avoid divide-by-zero edge cases
    if (std::abs(rad2deg(pitch_pred)) > Config::max_use_pitch_deg)
      return false; // near +/-90°, accel pitch is ill-conditioned
    return true;
  }

  void publish(double pitch, double gyro_pitch, double yaw_rate_z,
               TimePoint ts) {
    uint64_t s = seq_.load(std::memory_order_relaxed);
    seq_.store(s + 1, std::memory_order_release);
    pub_.t = ts;
    pub_.angle_rad = pitch;
    pub_.gyro_rad_s = gyro_pitch;
    pub_.yaw_rate_z = yaw_rate_z;
    seq_.store(s + 2, std::memory_order_release);
  }
};
