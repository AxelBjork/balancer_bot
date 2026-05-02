// MotorRunner.h
#pragma once
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <utility>

#include "services/motor/stepper.h"

struct MotorFeedbackSample {
  double left_applied_sps{0.0};
  double right_applied_sps{0.0};
  double measured_avg_sps{0.0};
  double update_dt_ms{0.0};
  double feedback_age_ms{0.0};
  int64_t left_actual_steps{0};
  int64_t right_actual_steps{0};
};

inline constexpr size_t kVelocityHistorySize = 64;
inline constexpr double kVelocityEstimateWindowS = 0.05;

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
  static constexpr unsigned kFrameUs = 100000;  // total frame length; 1 pulse/frame = 50 sps
  static constexpr unsigned kMinPulseUs = 2;  // per-pulse "high" width
  static constexpr unsigned kMaxScheduledHz = 100000;
  // Derived maxima
  static constexpr unsigned kMinPulse = (kMinPulseUs ? kMinPulseUs : 1u);
  static constexpr unsigned kMaxN =
      (kMaxScheduledHz * kFrameUs + 999999u) / 1000000u;  // pulses/channel
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
    if (hz > kMaxScheduledHz) hz = kMaxScheduledHz;

    const double pulses_f = double(hz) * double(kFrameUs) / 1e6;
    unsigned n = static_cast<unsigned>(llround(pulses_f));
    if (n == 0) n = 1;
    if (n > kMaxN) n = kMaxN;

    unsigned m = 0;
    for (unsigned i = 0; i < n; ++i) {
      const double slot = (i + 0.5) * double(kFrameUs) / double(n);
      uint32_t t_on = static_cast<uint32_t>(llround(slot));
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
      pulses[p++] = gpioPulse_t{0u, 0u, kFrameUs - last_t};
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
  using FeedbackSample = MotorFeedbackSample;

  // kFrameUs/kMinPulseUs come from DualWave's constexprs
  MotorRunner(Stepper& left, Stepper& right, double control_hz = 1000.0,
              double max_slew_sps_per_s = 250000.0)
      : L_(left),
        R_(right),
        wave_(left.pi(), left.stepPin(), right.stepPin()),
        slew_per_call_(max_slew_sps_per_s / ((control_hz > 0.0) ? control_hz : 1.0)) {
  }

  // Only entry point: always rebuild the repeating frame for both channels.
  void setTargets(double left_sps, double right_sps, uint64_t timestamp_us) {
    current_time_us_.store(timestamp_us, std::memory_order_relaxed);
    tgt_left_.store(left_sps, std::memory_order_relaxed);
    tgt_right_.store(right_sps, std::memory_order_relaxed);
    applyOnce();
  }

  // Kept for completeness; prefer setTargets() from your controller.
  void setLeft(double sps) {
    setTargets(sps, tgt_right_.load(std::memory_order_relaxed),
               current_time_us_.load(std::memory_order_relaxed));
  }

  void setRight(double sps) {
    setTargets(tgt_left_.load(std::memory_order_relaxed), sps,
               current_time_us_.load(std::memory_order_relaxed));
  }

  void stop() {
    std::lock_guard<std::mutex> lk(mu_);
    wave_.stop();
    last_cmd_L_ = last_cmd_R_ = 0.0;
    last_applied_hzL_ = last_applied_hzR_ = 0;
    measured_avg_sps_ = 0.0;
    vel_hist_count_ = 0;
    vel_hist_head_ = 0;
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

  FeedbackSample getFeedbackSample() const {
    std::lock_guard<std::mutex> lk(mu_);
    FeedbackSample sample{};
    const double left_positive_dir = L_.forwardFromSps(1.0) ? 1.0 : -1.0;
    const double right_positive_dir = R_.forwardFromSps(1.0) ? 1.0 : -1.0;
    sample.left_applied_sps =
        (last_applied_hzL_ == 0u) ? 0.0 : static_cast<double>(last_applied_hzL_) *
                                                 (last_applied_fwdL_ ? 1.0 : -1.0) *
                                                 left_positive_dir;
    sample.right_applied_sps =
        (last_applied_hzR_ == 0u) ? 0.0 : static_cast<double>(last_applied_hzR_) *
                                                  (last_applied_fwdR_ ? 1.0 : -1.0) *
                                                  right_positive_dir;
    sample.update_dt_ms = last_update_dt_s_ * 1000.0;
    if (last_call_time_us_ > 0u) {
      const uint64_t now_us = current_time_us_.load(std::memory_order_relaxed);
      sample.feedback_age_ms =
          (now_us >= last_call_time_us_) ? static_cast<double>(now_us - last_call_time_us_) / 1000.0
                                         : 0.0;
    }

    const double avg_actual_steps = 0.5 * (accum_actual_L_ + accum_actual_R_);
    if (vel_hist_count_ < 2u) {
      measured_avg_sps_ = 0.0;
    } else {
      const uint64_t now_us = current_time_us_.load(std::memory_order_relaxed);
      size_t selected_idx = (vel_hist_head_ + kVelocityHistorySize - vel_hist_count_) %
                            kVelocityHistorySize;
      bool found_window = false;
      for (size_t offset = 1; offset < vel_hist_count_; ++offset) {
        const size_t idx =
            (vel_hist_head_ + kVelocityHistorySize - 1u - offset) % kVelocityHistorySize;
        if (now_us <= vel_hist_time_us_[idx]) {
          continue;
        }
        const uint64_t age_us = now_us - vel_hist_time_us_[idx];
        if (age_us >= static_cast<uint64_t>(kVelocityEstimateWindowS * 1e6)) {
          selected_idx = idx;
          found_window = true;
          break;
        }
        selected_idx = idx;
      }

      if (now_us <= vel_hist_time_us_[selected_idx]) {
        sample.measured_avg_sps = measured_avg_sps_;
        sample.left_actual_steps = actual_steps_left_.load(std::memory_order_relaxed);
        sample.right_actual_steps = actual_steps_right_.load(std::memory_order_relaxed);
        return sample;
      }
      const double dt_s = static_cast<double>(now_us - vel_hist_time_us_[selected_idx]) / 1e6;
      if ((found_window || dt_s > 1e-6) && dt_s > 1e-6) {
        measured_avg_sps_ = (avg_actual_steps - vel_hist_avg_steps_[selected_idx]) / dt_s;
      }
    }

    sample.measured_avg_sps = measured_avg_sps_;
    sample.left_actual_steps = actual_steps_left_.load(std::memory_order_relaxed);
    sample.right_actual_steps = actual_steps_right_.load(std::memory_order_relaxed);
    return sample;
  }

  double getActualSpeedSps() {
    const auto sample = getFeedbackSample();
    return sample.measured_avg_sps;
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
    unsigned pulses = static_cast<unsigned>(std::llround(total));
    // Clamp to valid range
    if (pulses > static_cast<unsigned>(max_pulses_possible)) {
      pulses = static_cast<unsigned>(max_pulses_possible);
    }
    // Update error for next frame
    error_acc = total - pulses;

    // Convert back to "Hz" for DualWave API
    // DualWave does: n = llround(hz * kFrameUs / 1e6)
    // We want n = pulses. So hz = pulses * 1e6 / kFrameUs.
    // This results in exact integer pulses in DualWave.
    if (pulses == 0) {
      return {0, 0};
    }
    unsigned hz = static_cast<unsigned>(std::llround(double(pulses) * 1e6 / double(kFrameUs)));
    return {hz, pulses};
  }

  void integrateTrackingLocked(uint64_t now_us) {
    if (last_call_time_us_ == 0u) {
      last_call_time_us_ = now_us;
      return;
    }

    const double d_sec = (now_us > last_call_time_us_)
                             ? static_cast<double>(now_us - last_call_time_us_) / 1e6
                             : 0.0;
    if (d_sec <= 0.0) {
      last_call_time_us_ = now_us;
      return;
    }
    last_update_dt_s_ = d_sec;

    accum_L_ += last_cmd_L_ * d_sec;
    accum_R_ += last_cmd_R_ * d_sec;

    steps_left_.store(static_cast<int64_t>(accum_L_), std::memory_order_relaxed);
    steps_right_.store(static_cast<int64_t>(accum_R_), std::memory_order_relaxed);

    const double left_positive_dir = L_.forwardFromSps(1.0) ? 1.0 : -1.0;
    const double right_positive_dir = R_.forwardFromSps(1.0) ? 1.0 : -1.0;
    const double actL = static_cast<double>(last_applied_hzL_) * d_sec *
                        (last_applied_fwdL_ ? 1.0 : -1.0) * left_positive_dir;
    const double actR = static_cast<double>(last_applied_hzR_) * d_sec *
                        (last_applied_fwdR_ ? 1.0 : -1.0) * right_positive_dir;

    accum_actual_L_ += actL;
    accum_actual_R_ += actR;

    recordVelocityHistoryLocked(now_us, 0.5 * (accum_actual_L_ + accum_actual_R_));

    actual_steps_left_.store(static_cast<int64_t>(accum_actual_L_), std::memory_order_relaxed);
    actual_steps_right_.store(static_cast<int64_t>(accum_actual_R_), std::memory_order_relaxed);
    last_call_time_us_ = now_us;
  }

  void recordVelocityHistoryLocked(uint64_t now_us, double avg_actual_steps) {
    vel_hist_time_us_[vel_hist_head_] = now_us;
    vel_hist_avg_steps_[vel_hist_head_] = avg_actual_steps;
    vel_hist_head_ = (vel_hist_head_ + 1u) % kVelocityHistorySize;
    if (vel_hist_count_ < kVelocityHistorySize) {
      ++vel_hist_count_;
    }
  }

  void applyOnce() {
    std::lock_guard<std::mutex> lk(mu_);
    const uint64_t now_us = current_time_us_.load(std::memory_order_relaxed);
    integrateTrackingLocked(now_us);

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
    const unsigned hzR = resR.first;

    // Actual steps are now integrated below based on active Hz

    if (hzL == 0u && hzR == 0u) {
      if (last_applied_hzL_ != 0u || last_applied_hzR_ != 0u) {
        wave_.stop();
      }
      last_applied_hzL_ = 0u;
      last_applied_hzR_ = 0u;
      last_applied_fwdL_ = fwdL;
      last_applied_fwdR_ = fwdR;
      last_cmd_L_ = tgtL;
      last_cmd_R_ = tgtR;
      return;
    }

    const bool dirFlip = (fwdL != L_.dirForward()) || (fwdR != R_.dirForward());
    const bool scheduleChanged = (hzL != last_applied_hzL_) || (hzR != last_applied_hzR_) ||
                                 (fwdL != last_applied_fwdL_) ||
                                 (fwdR != last_applied_fwdR_);
    if (dirFlip) {
      wave_.stop();
      L_.setDirNoWait(fwdL);
      R_.setDirNoWait(fwdR);
      time_sleep(0.00005);  // ~50 µs settle
    }

    if (scheduleChanged || dirFlip) {
      wave_.apply(hzL, hzR);
    }
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
  mutable std::mutex mu_;

  std::atomic<double> tgt_left_{0.0}, tgt_right_{0.0};
  std::atomic<uint64_t> current_time_us_{0};
  double last_cmd_L_{0.0}, last_cmd_R_{0.0};
  const double slew_per_call_;

  // Step tracking state
  uint64_t last_call_time_us_{0};
  unsigned last_applied_hzL_{0}, last_applied_hzR_{0};
  bool last_applied_fwdL_{true}, last_applied_fwdR_{true};
  std::atomic<int64_t> steps_left_{0}, steps_right_{0};

  std::atomic<int64_t> actual_steps_left_{0}, actual_steps_right_{0};

  double accum_L_{0.0}, accum_R_{0.0};
  double accum_actual_L_{0.0}, accum_actual_R_{0.0};
  double last_update_dt_s_{0.0};
  mutable std::array<uint64_t, kVelocityHistorySize> vel_hist_time_us_{};
  mutable std::array<double, kVelocityHistorySize> vel_hist_avg_steps_{};
  mutable size_t vel_hist_head_{0};
  mutable size_t vel_hist_count_{0};
  mutable double measured_avg_sps_{0.0};
};
