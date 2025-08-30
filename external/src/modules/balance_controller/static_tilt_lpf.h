// StaticTiltLPF.hpp
#pragma once
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>


class StaticTiltLPF {
public:
  using Acc3 = std::array<double, 3>;
  using Gyr3 = std::array<double, 3>;
  using TimePoint = std::chrono::steady_clock::time_point;


  explicit StaticTiltLPF() {
    push_data = [this](const Acc3& acc, const Gyr3& gyrv, TimePoint ts) {
      this->push_sample(acc, gyrv, ts);
    };
  }

  // ---------------- Writer (single producer) ----------------
  void push_sample(const Acc3& acc, const Gyr3& gyrv, TimePoint ts) {
    const double dt = compute_dt(ts);
    const double a_acc   = std::exp(-2.0 * M_PI * C.fc_acc_hz   * dt);
    const double a_g     = std::exp(-2.0 * M_PI * C.fc_g_hz     * dt);
    const double a_angle = std::exp(-2.0 * M_PI * C.fc_angle_hz * dt);
    // gyro LPF coefficient (single-pole at ~50 Hz by default)
    const double a_gyro  = std::exp(-2.0 * M_PI * C.fc_gyro_pitch_hz * dt);

    if (!initialized) {
      ax_f = acc;
      g_est = norm(ax_f);
      if (!std::isfinite(g_est) || g_est <= 1e-9) { g_est = C.g0_guess; }
      const auto rp = tilt_from_acc(normalize(ax_f, g_est));
      roll_rad = rp[0];
      pitch_rad = rp[1];
      gyro_pitch_f = gyrv[1];
      initialized = true;
    }
    else {
      for (int i = 0; i < 3; ++i) {
        ax_f[i] = a_acc * ax_f[i] + (1.0 - a_acc) * acc[i];
      }
      const double a_norm = norm(ax_f);
      const double band = C.g_rel_band * g_est;
      const double a_clamped = clamp(a_norm, g_est - band, g_est + band);
      g_est = a_g * g_est + (1.0 - a_g) * a_clamped;

      const double g_safe = (g_est > 1e-9 && std::isfinite(g_est)) ? g_est : C.g0_guess;
      const auto a_n = normalize(ax_f, g_safe);
      const auto rp = tilt_from_acc(a_n);

      roll_rad  = a_angle * roll_rad  + (1.0 - a_angle) * rp[0];
      pitch_rad = a_angle * pitch_rad + (1.0 - a_angle) * rp[1];

      const double gyro_pitch = gyrv[1];
      gyro_pitch_f = a_gyro * gyro_pitch_f + (1.0 - a_gyro) * gyro_pitch;
    }

    // Publish coherent snapshot (seqlock)
    const double yaw_rate_z = gyrv[2];

    uint64_t s1 = seq.load(std::memory_order_relaxed);
    seq.store(s1 + 1, std::memory_order_release);
    pub.t          = ts;
    pub.angle_rad  = pitch_rad;
    pub.gyro_rad_s = gyro_pitch_f;
    pub.yaw_rate_z = yaw_rate_z;
    seq.store(s1 + 2, std::memory_order_release);
  }

  // Adapter-friendly member
  std::function<void(const Acc3&, const Gyr3&, TimePoint)> push_data;

  // ---------------- Readers (multi-consumer) ----------------
  ImuSample read_latest() const {
    ImuSample out;
    for (;;) {
      uint64_t s1 = seq.load(std::memory_order_acquire);
      if (s1 & 1u) { continue; }
      out = pub;
      uint64_t s2 = seq.load(std::memory_order_acquire);
      if (s1 == s2 && !(s2 & 1u)) { break; }
    }
    return out;
  }

  void reset() {
    initialized = false;
    have_last_ts = false;
    ax_f = {0.0, 0.0, 0.0};
    g_est = C.g0_guess;
    roll_rad = 0.0;
    pitch_rad = 0.0;
    gyro_pitch_f = 0.0;

    uint64_t s1 = seq.load(std::memory_order_relaxed);
    seq.store(s1 + 1, std::memory_order_release);
    pub = ImuSample{};
    seq.store(s1 + 2, std::memory_order_release);
  }

private:
  static constexpr Config C{};

  bool initialized{false};
  bool have_last_ts{false};
  TimePoint last_ts{};
  std::array<double, 3> ax_f{0.0, 0.0, 0.0};
  double g_est{9.81};
  double roll_rad{0.0};
  double pitch_rad{0.0};
  double gyro_pitch_f{0.0};

  mutable std::atomic<uint64_t> seq{0};
  alignas(64) ImuSample pub{};

  double compute_dt(TimePoint ts) {
    double dt;
    if (!have_last_ts) {
      dt = C.fallback_dt_s;
      last_ts = ts;
      have_last_ts = true;
    }
    else {
      dt = std::chrono::duration<double>(ts - last_ts).count();
      if (dt <= 0.0 || dt > 1.0) { dt = C.fallback_dt_s; }
      last_ts = ts;
    }
    return dt;
  }

  static double norm(const Acc3& v) {
    return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
  }

  static Acc3 normalize(const Acc3& v, double s) {
    const double inv = (s > 1e-12) ? (1.0 / s) : 0.0;
    return { v[0]*inv, v[1]*inv, v[2]*inv };
  }

  static double clamp(double x, double lo, double hi) {
    if (x < lo) { return lo; }
    if (x > hi) { return hi; }
    return x;
  }

  // roll = atan2(ay, az), pitch = atan2(-ax, sqrt(ay^2 + az^2))
  static std::array<double, 2> tilt_from_acc(const Acc3& a) {
    const double ay = a[1];
    const double az = a[2];
    const double ax = a[0];
    const double roll  = std::atan2(ay, az);
    const double pitch = std::atan2(-ax, std::sqrt(ay*ay + az*az));
    return { roll, pitch };
  }
};
