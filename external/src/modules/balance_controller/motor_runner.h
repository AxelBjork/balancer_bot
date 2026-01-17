// MotorRunner.h
#pragma once
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <mutex>

#include "stepper.h"

// DualWave.h
#pragma once
#include <pigpiod_if2.h>

#include <cstdint>
#include <mutex>

// ---- DualWave (constexpr config, no templates) ----
// Rebuilds a repeating frame each time you call apply(hzL, hzR).
// Uses fixed-capacity buffers (no heap) and merges two sorted edge lists (O(N)).

class DualWave {
 public:
  // Compile-time configuration
  static constexpr unsigned kFrameUs = 5000;  // total frame length
  static constexpr unsigned kMinPulseUs = 2;  // per-pulse "high" width
  // Derived maxima
  static constexpr unsigned kMinPulse = (kMinPulseUs ? kMinPulseUs : 1u);
  static constexpr unsigned kMaxN = (kFrameUs / (2u * kMinPulse));  // pulses/channel
  static_assert(kMaxN > 0, "kFrameUs/kMinPulseUs too small to schedule any pulses.");

  DualWave(int pi, unsigned stepL, unsigned stepR, unsigned /*min_pulse_us*/ = kMinPulseUs,
           unsigned /*frame_us*/ = kFrameUs)
      : pi_(pi), stepL_(stepL), stepR_(stepR) {
  }

  void apply(unsigned hzL, unsigned hzR) {
    std::lock_guard<std::mutex> lk(mu_);

    if (hzL == 0 && hzR == 0) {
      wave_tx_stop(pi_);
      if (wave_id_ >= 0) {
        wave_delete(pi_, wave_id_);
        wave_id_ = -1;
      }
      wave_clear(pi_);
      cur_hzL_ = cur_hzR_ = 0;
      return;
    }

    const unsigned nEL = scheduleChannel(stepL_, hzL, edges_L_);
    const unsigned nER = scheduleChannel(stepR_, hzR, edges_R_);
    if (nEL == 0 && nER == 0) {
      wave_tx_stop(pi_);
      if (wave_id_ >= 0) {
        wave_delete(pi_, wave_id_);
        wave_id_ = -1;
      }
      wave_clear(pi_);
      cur_hzL_ = cur_hzR_ = 0;
      return;
    }

    const unsigned nE = mergeEdges(edges_L_, nEL, edges_R_, nER, edges_);
    const unsigned nP = buildFramePulses(edges_, nE, pulses_);

    wave_tx_stop(pi_);
    if (wave_id_ >= 0) {
      wave_delete(pi_, wave_id_);
      wave_id_ = -1;
    }
    wave_clear(pi_);

    wave_add_generic(pi_, nP, pulses_);
    wave_id_ = wave_create(pi_);
    if (wave_id_ >= 0) {
      wave_send_repeat(pi_, wave_id_);
      cur_hzL_ = hzL;
      cur_hzR_ = hzR;
    } else {
      cur_hzL_ = cur_hzR_ = 0;
    }
  }

  void stop() {
    std::lock_guard<std::mutex> lk(mu_);
    wave_tx_stop(pi_);
    if (wave_id_ >= 0) {
      wave_delete(pi_, wave_id_);
      wave_id_ = -1;
    }
    wave_clear(pi_);
    cur_hzL_ = cur_hzR_ = 0;
  }

 private:
  struct Edge {
    uint32_t t;
    uint32_t on_mask;
    uint32_t off_mask;
  };

  static inline uint32_t bit(unsigned gpio) {
    return (1u << gpio);
  }

  static unsigned scheduleChannel(unsigned gpio, unsigned hz, Edge* out) {
    if (hz == 0) return 0;

    const double pulses_f = double(hz) * double(kFrameUs) / 1e6;
    unsigned n = (unsigned)llround(pulses_f);
    if (n == 0) n = 1;
    if (n > kMaxN) n = kMaxN;

    unsigned m = 0;
    for (unsigned i = 0; i < n; ++i) {
      const double slot = (i + 0.5) * double(kFrameUs) / double(n);
      uint32_t t_on = (uint32_t)llround(slot);
      if (t_on >= kFrameUs) t_on = kFrameUs - 1;

      uint32_t t_off = t_on + kMinPulse;
      if (t_off >= kFrameUs) t_off = kFrameUs - 1;

      out[m++] = Edge{t_on, bit(gpio), 0u};
      out[m++] = Edge{t_off, 0u, bit(gpio)};
    }
    return m;
  }

  static unsigned mergeEdges(const Edge* L, unsigned nL, const Edge* R, unsigned nR, Edge* out) {
    unsigned i = 0, j = 0, k = 0;
    while (i < nL || j < nR) {
      const uint32_t tL = (i < nL) ? L[i].t : UINT32_MAX;
      const uint32_t tR = (j < nR) ? R[j].t : UINT32_MAX;
      const uint32_t t = (tL < tR) ? tL : tR;

      uint32_t on_mask = 0, off_mask = 0;
      while (i < nL && L[i].t == t) {
        on_mask |= L[i].on_mask;
        off_mask |= L[i].off_mask;
        ++i;
      }
      while (j < nR && R[j].t == t) {
        on_mask |= R[j].on_mask;
        off_mask |= R[j].off_mask;
        ++j;
      }

      out[k++] = Edge{t, on_mask, off_mask};
    }
    return k;
  }

  static unsigned buildFramePulses(const Edge* edges, unsigned nE, gpioPulse_t* pulses) {
    if (nE == 0) {
      pulses[0] = gpioPulse_t{0u, 0u, kFrameUs};
      return 1;
    }

    unsigned p = 0;
    uint32_t last_t = 0;

    for (unsigned idx = 0; idx < nE; ++idx) {
      const uint32_t t = edges[idx].t;

      const uint32_t dt = (t > last_t) ? (t - last_t) : 1u;
      pulses[p++] = gpioPulse_t{0u, 0u, dt};
      last_t = t;

      pulses[p++] = gpioPulse_t{edges[idx].on_mask, edges[idx].off_mask, 1u};
      last_t += 1u;
    }

    if (last_t < kFrameUs) {
      pulses[p++] = gpioPulse_t{0u, 0u, (uint32_t)(kFrameUs - last_t)};
    }
    return p;
  }

  int pi_;
  unsigned stepL_;
  unsigned stepR_;

  std::mutex mu_;
  int wave_id_{-1};
  unsigned cur_hzL_{0}, cur_hzR_{0};

  // Capacities from constexpr
  static constexpr unsigned kMaxEdgesPerChan = 2u * kMaxN;
  static constexpr unsigned kMaxEdgesMerged = 2u * kMaxEdgesPerChan;
  static constexpr unsigned kMaxPulses = (kMaxEdgesMerged * 2u) + 4u;

  // Fixed-capacity buffers
  Edge edges_L_[kMaxEdgesPerChan]{};
  Edge edges_R_[kMaxEdgesPerChan]{};
  Edge edges_[kMaxEdgesMerged]{};
  gpioPulse_t pulses_[kMaxPulses]{};
};

// -------------------- MotorRunner (paired updates, no retune) --------------------

class MotorRunner {
 public:
  // kFrameUs/kMinPulseUs come from DualWave's constexprs
  MotorRunner(Stepper& left, Stepper& right, double control_hz = 1000.0,
              double max_slew_sps_per_s = 250000.0)
      : L_(left),
        R_(right),
        wave_(left.pi(), left.stepPin(), right.stepPin()),
        slew_per_call_(max_slew_sps_per_s / ((control_hz > 0.0) ? control_hz : 1.0)) {
  }

  // Only entry point: always rebuild the repeating frame for both channels.
  void setTargets(double left_sps, double right_sps) {
    tgt_left_.store(left_sps, std::memory_order_relaxed);
    tgt_right_.store(right_sps, std::memory_order_relaxed);
    applyOnce();
  }

  // Kept for completeness; prefer setTargets() from your controller.
  void setLeft(double sps) {
    setTargets(sps, tgt_right_.load(std::memory_order_relaxed));
  }

  void setRight(double sps) {
    setTargets(tgt_left_.load(std::memory_order_relaxed), sps);
  }

  void stop() {
    std::lock_guard<std::mutex> lk(mu_);
    wave_.stop();
    last_cmd_L_ = last_cmd_R_ = 0.0;
    last_applied_hzL_ = last_applied_hzR_ = 0;
    // Reset time to avoid large jump on restart?
    // Or just let it be. If we stop, rate is 0, so integration adds 0.
    // But we should reset last_call_time_ if we want to restart cleanly?
    // Actually, if we stop, we just set rates to 0. The integration loop handles 0 rate fine.
  }

  int64_t getLeftSteps() const {
    return steps_left_.load(std::memory_order_relaxed);
  }
  int64_t getRightSteps() const {
    return steps_right_.load(std::memory_order_relaxed);
  }

  int64_t getActualLeftSteps() const {
    return actual_steps_left_.load(std::memory_order_relaxed);
  }
  int64_t getActualRightSteps() const {
    return actual_steps_right_.load(std::memory_order_relaxed);
  }

  double getAverageSpeedSps() const {
    // Return average of CURRENT commanded targets (no lag)
    // Reading atomics is thread-safe and gives immediate feedback
    double L = tgt_left_.load(std::memory_order_relaxed);
    double R = tgt_right_.load(std::memory_order_relaxed);
    return (L + R) / 2.0;
  }

  // Stateful velocity feedback based on actual steps
  // Call this periodically to get the average speed.
  // For open-loop steppers, "Actual" speed is best approximated by the slew-limited command.
  // Differentiating integer steps adds quantization noise (e.g. 400 SPS steps at 400Hz).
  float getActualSpeedSps() {
    std::lock_guard<std::mutex> lk(mu_);  // Lock consistent with applyOnce
    return (float)(last_cmd_L_ + last_cmd_R_) / 2.0f;
  }

 private:
  static inline double clampDelta(double from, double to, double max_delta) {
    const double d = to - from;
    if (d > max_delta) return from + max_delta;
    if (d < -max_delta) return from - max_delta;
    return to;
  }

  // Sigma-Delta Error State
  double sd_error_L_{0.0};
  double sd_error_R_{0.0};

  static inline std::pair<unsigned, unsigned> calculatePulsesPerFrame(
      double sps, double& error_acc, unsigned kFrameUs = DualWave::kFrameUs,
      unsigned kMinPulse = DualWave::kMinPulse) {
    double f = std::fabs(sps);
    // Max theoretical pulses per frame
    const double max_pulses_possible = double(kFrameUs) / (2.0 * double(kMinPulse));

    // Desired pulses this frame
    double desired_pulses = f * double(kFrameUs) / 1e6;
    if (desired_pulses > max_pulses_possible) desired_pulses = max_pulses_possible;

    // Sigma-Delta modulation
    double total = desired_pulses + error_acc;
    unsigned pulses = (unsigned)std::llround(total);
    // Clamp to valid range
    if (pulses > (unsigned)max_pulses_possible) pulses = (unsigned)max_pulses_possible;

    // Update error for next frame
    error_acc = total - pulses;

    // Convert back to "Hz" for DualWave API
    // DualWave does: n = llround(hz * kFrameUs / 1e6)
    // We want n = pulses. So hz = pulses * 1e6 / kFrameUs.
    // This results in exact integer pulses in DualWave.
    if (pulses == 0) return {0, 0};
    unsigned hz = (unsigned)std::llround(double(pulses) * 1e6 / double(kFrameUs));
    return {hz, pulses};
  }

  void applyOnce() {
    std::lock_guard<std::mutex> lk(mu_);

    const double tgtL =
        clampDelta(last_cmd_L_, tgt_left_.load(std::memory_order_relaxed), slew_per_call_);
    const double tgtR =
        clampDelta(last_cmd_R_, tgt_right_.load(std::memory_order_relaxed), slew_per_call_);

    const bool fwdL = L_.forwardFromSps(tgtL);
    const bool fwdR = R_.forwardFromSps(tgtR);

    // Calculate Hz using Sigma-Delta
    auto resL = calculatePulsesPerFrame(tgtL, sd_error_L_);
    auto resR = calculatePulsesPerFrame(tgtR, sd_error_R_);
    const unsigned hzL = resL.first;
    const unsigned pulsesL = resL.second;
    const unsigned hzR = resR.first;
    const unsigned pulsesR = resR.second;

    // Actual steps are now integrated below based on active Hz

    if (hzL == 0u && hzR == 0u) {
      wave_.stop();
      last_cmd_L_ = tgtL;
      last_cmd_R_ = tgtR;
      return;
    }

    const bool dirFlip = (fwdL != L_.dirForward()) || (fwdR != R_.dirForward());
    if (dirFlip) {
      wave_.stop();
      L_.setDirNoWait(fwdL);
      R_.setDirNoWait(fwdR);
      time_sleep(0.00005);  // ~50 µs settle
    }

    wave_.apply(hzL, hzR);

    // Step tracking
    auto now = std::chrono::steady_clock::now();
    if (last_call_time_.time_since_epoch().count() > 0) {
      std::chrono::duration<double> dt = now - last_call_time_;
      double d_sec = dt.count();

      // Integrate the commanded rate (smooth), not the quantized Hz
      // This matches the velocity feedback which also uses commanded values
      double stepsL = last_cmd_L_ * d_sec;
      double stepsR = last_cmd_R_ * d_sec;

      accum_L_ += stepsL;
      accum_R_ += stepsR;

      int64_t totalL = static_cast<int64_t>(accum_L_);
      int64_t totalR = static_cast<int64_t>(accum_R_);
      steps_left_.store(totalL, std::memory_order_relaxed);
      steps_right_.store(totalR, std::memory_order_relaxed);

      // Integrate actual rate (based on applied Hz)
      double actL = (double)last_applied_hzL_ * d_sec;
      double actR = (double)last_applied_hzR_ * d_sec;

      if (last_cmd_L_ < 0.0) actL = -actL;
      if (last_cmd_R_ < 0.0) actR = -actR;

      accum_actual_L_ += actL;
      accum_actual_R_ += actR;

      actual_steps_left_.store((int64_t)accum_actual_L_, std::memory_order_relaxed);
      actual_steps_right_.store((int64_t)accum_actual_R_, std::memory_order_relaxed);
    }
    last_call_time_ = now;
    last_applied_hzL_ = hzL;
    last_applied_hzR_ = hzR;
    last_applied_fwdL_ = fwdL;
    last_applied_fwdR_ = fwdR;

    last_cmd_L_ = tgtL;
    last_cmd_R_ = tgtR;
  }

  Stepper& L_;
  Stepper& R_;
  DualWave wave_;
  std::mutex mu_;

  std::atomic<double> tgt_left_{0.0}, tgt_right_{0.0};
  double last_cmd_L_{0.0}, last_cmd_R_{0.0};
  const double slew_per_call_;

  // Step tracking state
  std::chrono::steady_clock::time_point last_call_time_;
  unsigned last_applied_hzL_{0}, last_applied_hzR_{0};
  bool last_applied_fwdL_{true}, last_applied_fwdR_{true};
  std::atomic<int64_t> steps_left_{0}, steps_right_{0};

  std::atomic<int64_t> actual_steps_left_{0}, actual_steps_right_{0};

  double accum_L_{0.0}, accum_R_{0.0};
  double accum_actual_L_{0.0}, accum_actual_R_{0.0};

  // State for getActualSpeedSps
  std::mutex vel_mu_;
  std::chrono::steady_clock::time_point last_vel_time_;
  int64_t last_vel_left_{0}, last_vel_right_{0};
  float last_vel_val_{0.0f};
};
