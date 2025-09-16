// MotorRunner.h
#pragma once
#include "stepper.h"
#include <atomic>
#include <mutex>
#include <cmath>
#include <chrono>

// DualWave.h
#pragma once
#include <pigpiod_if2.h>
#include <vector>
#include <algorithm>
#include <mutex>
#include <cstdint>
#include <cmath>

class DualWave {
public:
  DualWave(int pi, unsigned stepL, unsigned stepR,
           unsigned min_pulse_us = 2, unsigned frame_us = 5000)
  : pi_(pi), stepL_(stepL), stepR_(stepR),
    min_pulse_us_(min_pulse_us), frame_us_(frame_us) {}

  // Start/retime both channels together.
  void apply(unsigned hzL, unsigned hzR) {
    std::lock_guard<std::mutex> lk(mu_);
    if (hzL == cur_hzL_ && hzR == cur_hzR_) return;

    // Stop and clear any prior wave
    wave_tx_stop(pi_);
    if (wave_id_ >= 0) { wave_delete(pi_, wave_id_); wave_id_ = -1; }
    wave_clear(pi_);

    cur_hzL_ = hzL; cur_hzR_ = hzR;

    if (hzL == 0 && hzR == 0) return; // nothing to send

    std::vector<gpioPulse_t> pulses;
    buildFramePulses(hzL, hzR, pulses);

    if (pulses.empty()) return;

    wave_add_generic(pi_, pulses.size(), pulses.data());
    wave_id_ = wave_create(pi_);
    if (wave_id_ >= 0) {
      wave_send_repeat(pi_, wave_id_);
    }
  }

  void stop() {
    std::lock_guard<std::mutex> lk(mu_);
    wave_tx_stop(pi_);
    if (wave_id_ >= 0) { wave_delete(pi_, wave_id_); wave_id_ = -1; }
    wave_clear(pi_);
    cur_hzL_ = cur_hzR_ = 0;
  }

private:
  struct Edge { uint32_t t; uint32_t on_mask; uint32_t off_mask; };

  static inline uint32_t bit(unsigned gpio) { return (1u << gpio); }

  // Evenly distribute N pulses over [0, frame_us), each pulse high for min_pulse_us_
  void scheduleChannel(unsigned gpio, unsigned hz,
                       std::vector<Edge>& edges) const {
    if (hz == 0) return;

    const double pulses_f = (double)hz * (double)frame_us_ / 1e6;
    unsigned n = (unsigned)std::llround(pulses_f);
    if (n == 0) n = 1;

    // ensure we can fit (2*min_pulse per pulse)
    const unsigned max_n = frame_us_ / (2u * std::max(1u, min_pulse_us_));
    if (n > max_n) n = max_n;

    for (unsigned i = 0; i < n; ++i) {
      // Centered slots for better dither
      const double slot = (i + 0.5) * (double)frame_us_ / (double)n;
      uint32_t t_on  = (uint32_t)std::llround(slot);
      if (t_on >= frame_us_) t_on = frame_us_ - 1;

      uint32_t t_off = t_on + std::max(1u, min_pulse_us_);
      if (t_off >= frame_us_) t_off = frame_us_ - 1;

      edges.push_back({t_on,  bit(gpio), 0});
      edges.push_back({t_off, 0,         bit(gpio)});
    }
  }

  void buildFramePulses(unsigned hzL, unsigned hzR,
                        std::vector<gpioPulse_t>& pulses) const {
    std::vector<Edge> edges;
    edges.reserve(2 * ((hzL * frame_us_) / 1000 + (hzR * frame_us_) / 1000) + 8);

    scheduleChannel(stepL_, hzL, edges);
    scheduleChannel(stepR_, hzR, edges);

    if (edges.empty()) return;

    // Sort by time; coalesce same-time edges
    std::sort(edges.begin(), edges.end(),
              [](const Edge& a, const Edge& b){
                if (a.t != b.t) return a.t < b.t;
                // apply OFF before ON if same time (avoids transient high-high glitches)
                return a.off_mask > b.off_mask;
              });

    // Build gpioPulse_t segments
    uint32_t last_t = 0;
    size_t i = 0;
    while (i < edges.size()) {
      uint32_t t = edges[i].t;

      // delay from last_t to this event time
      uint32_t dt = (t > last_t) ? (t - last_t) : 1u; // never 0 delay
      pulses.push_back(gpioPulse_t{0, 0, dt});
      last_t = t;

      // collect all edges at time t
      uint32_t on_mask = 0, off_mask = 0;
      while (i < edges.size() && edges[i].t == t) {
        on_mask  |= edges[i].on_mask;
        off_mask |= edges[i].off_mask;
        ++i;
      }

      // apply edges and hold at least 1 us
      pulses.push_back(gpioPulse_t{ on_mask, off_mask, 1u });
      last_t += 1u;
    }

    // finish out the frame
    if (last_t < frame_us_) {
      pulses.push_back(gpioPulse_t{0,0, frame_us_ - last_t});
    }
  }

  int pi_;
  unsigned stepL_, stepR_;
  unsigned min_pulse_us_;
  unsigned frame_us_;

  std::mutex mu_;
  int wave_id_{-1};
  unsigned cur_hzL_{0}, cur_hzR_{0};
};


class MotorRunner {
public:
  MotorRunner(Stepper& left, Stepper& right,
              double control_hz = 1000.0,
              double max_slew_sps_per_s = 250000.0,
              double abs_deadband_sps = 20.0)
  : L_(left), R_(right),
    wave_(left.pi(), left.stepPin(), right.stepPin()),
    slew_per_call_(max_slew_sps_per_s / std::max(1.0, control_hz)),
    deadband_sps_(abs_deadband_sps) {}

  void setLeft(double sps)  { tgt_left_.store(sps,  std::memory_order_relaxed);  applyPair(); }
  void setRight(double sps) { tgt_right_.store(sps, std::memory_order_relaxed);  applyPair(); }
  void setTargets(double left_sps, double right_sps) {
    tgt_left_.store(left_sps,  std::memory_order_relaxed);
    tgt_right_.store(right_sps, std::memory_order_relaxed);
    applyPair();
  }

  void stop() {
    std::lock_guard<std::mutex> lk(mu_);
    wave_.stop();
    last_cmd_L_ = last_cmd_R_ = 0.0;
    last_tune_tp_ = {};
  }

private:
  // ---- helpers (same as before) ----
  static inline double clampDelta(double from, double to, double max_delta) {
    const double d = to - from;
    if (d >  max_delta) return from + max_delta;
    if (d < -max_delta) return from - max_delta;
    return to;
  }
  static inline unsigned round_uint(double x) {
    return (x < 0.5) ? 0u : static_cast<unsigned>(std::llround(x));
  }
  static inline unsigned quantizeHz(unsigned hz) {
    if (hz == 0u) return 0u;
    if (hz < 100u)   return hz;
    if (hz < 2000u)  return ((hz + 2u) / 5u) * 5u;
    return ((hz + 25u) / 50u) * 50u;
  }
  static inline unsigned clampSpsToHzRounded(double sps,
                                             unsigned kMinPulseUs = 2,
                                             double   kMaxFreqHz  = 50'000.0) {
    double f = std::fabs(sps);
    const double max_by_pulse = 1e6 / (2.0 * kMinPulseUs);
    if (f > kMaxFreqHz)   f = kMaxFreqHz;
    if (f > max_by_pulse) f = max_by_pulse;
    return (f < 1.0) ? 0u : round_uint(f);
  }
  static inline bool needRetune(unsigned old_hz, unsigned new_hz) {
    if (old_hz == new_hz) return false;
    if (old_hz == 0u)     return new_hz > 0u;
    const unsigned abs_thresh = 3u;
    const unsigned rel_thresh = static_cast<unsigned>(old_hz * 0.01);
    const unsigned thresh = (abs_thresh > rel_thresh) ? abs_thresh : rel_thresh;
    const unsigned delta = (old_hz > new_hz) ? (old_hz - new_hz) : (new_hz - old_hz);
    return delta >= thresh;
  }

  void applyPair() {
    using clock = std::chrono::steady_clock;
    std::lock_guard<std::mutex> lk(mu_);

    double tgtL = clampDelta(last_cmd_L_, tgt_left_.load(std::memory_order_relaxed),  slew_per_call_);
    double tgtR = clampDelta(last_cmd_R_, tgt_right_.load(std::memory_order_relaxed), slew_per_call_);

    if (std::fabs(tgtL - last_cmd_L_) < deadband_sps_ &&
        std::fabs(tgtR - last_cmd_R_) < deadband_sps_)
      return;

    const bool fwdL = L_.forwardFromSps(tgtL);
    const bool fwdR = R_.forwardFromSps(tgtR);

    unsigned hzL = quantizeHz(Stepper::clampSpsToHz(tgtL));
    unsigned hzR = quantizeHz(Stepper::clampSpsToHz(tgtR));

    const bool dirFlip = (fwdL != L_.dirForward()) || (fwdR != R_.dirForward());

    const auto now = clock::now();
    constexpr auto kMinRetuneInterval = std::chrono::milliseconds(5);
    const bool time_ok = (last_tune_tp_.time_since_epoch().count() == 0) ||
                         (now - last_tune_tp_ >= kMinRetuneInterval);

    // only rebuild the wave when needed
    const bool changeL = needRetune(last_hzL_, hzL);
    const bool changeR = needRetune(last_hzR_, hzR);
    const bool need_wave_update = dirFlip || ((changeL || changeR) && time_ok);

    if (!need_wave_update) {
      last_cmd_L_ = tgtL; last_cmd_R_ = tgtR;
      return;
    }

    if (dirFlip) {
      // stop the wave, flip DIRs, short setup delay
      wave_.stop();
      L_.setDirNoWait(fwdL);
      R_.setDirNoWait(fwdR);
      time_sleep(0.00005); // 50 µs is plenty for typical drivers
    }

    wave_.apply(hzL, hzR);

    last_cmd_L_ = tgtL; last_cmd_R_ = tgtR;
    last_hzL_ = hzL;    last_hzR_ = hzR;
    last_tune_tp_ = now;
  }

  Stepper& L_;
  Stepper& R_;
  DualWave wave_;
  std::mutex mu_;

  std::atomic<double> tgt_left_{0.0}, tgt_right_{0.0};
  double   last_cmd_L_{0.0}, last_cmd_R_{0.0};
  unsigned last_hzL_{0},    last_hzR_{0};
  const double slew_per_call_;
  const double deadband_sps_;
  std::chrono::steady_clock::time_point last_tune_tp_{};
};
