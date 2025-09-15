// MotorRunner.h
#pragma once
#include "stepper.h"
#include <atomic>
#include <mutex>
#include <cmath>
#include <pigpiod_if2.h>
#include <stdexcept>

#include "config_pid.h"

class MotorRunner {
public:
  // control_hz only used to derive slew_per_call (no timers/threads here)
  MotorRunner(Stepper& left, Stepper& right,
              double control_hz = 1000.0,
              double max_slew_sps_per_s = 250000.0,  // jerk limiter
              double abs_deadband_sps = 20.0)        // ignore tiny jitters (in SPS domain)
  : L_(left), R_(right),
    slew_per_call_(max_slew_sps_per_s / std::max(1.0, control_hz)),
    deadband_sps_(abs_deadband_sps) {}

  // Your cascaded controller can call either the per-side or combined API.
  void setLeft(double sps)  { tgt_left_.store(sps,  std::memory_order_relaxed);  applyPair(); }
  void setRight(double sps) { tgt_right_.store(sps, std::memory_order_relaxed);  applyPair(); }
  void setTargets(double left_sps, double right_sps) {
    tgt_left_.store(left_sps,  std::memory_order_relaxed);
    tgt_right_.store(right_sps, std::memory_order_relaxed);
    applyPair();
  }

  void stop() {
    std::lock_guard<std::mutex> lk(mu_);
    L_.stop();
    R_.stop();
    last_cmd_L_ = 0.0;
    last_cmd_R_ = 0.0;
    last_tune_tp_ = {};
  }

private:
  // ---- small helpers ----
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
    if (hz < 100u)   return hz;                         // 1 Hz steps under 100
    if (hz < 2000u)  return ((hz + 2u) / 5u) * 5u;      // 5 Hz steps 100–2k
    return ((hz + 25u) / 50u) * 50u;                    // 50 Hz steps ≥2k
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
    if (old_hz == 0u)     return new_hz > 0u; // allow starting from zero even at 1–2 Hz
    const unsigned abs_thresh = 3u;                           // absolute threshold
    const unsigned rel_thresh = static_cast<unsigned>(old_hz * 0.01); // 1%
    const unsigned thresh = (abs_thresh > rel_thresh) ? abs_thresh : rel_thresh;
    const unsigned delta = (old_hz > new_hz) ? (old_hz - new_hz) : (new_hz - old_hz);
    return delta >= thresh;
  }

  void applyPair() {
    using clock = std::chrono::steady_clock;

    std::lock_guard<std::mutex> lk(mu_);

    // 1) Read targets and slew-limit in SPS domain
    double tgtL = clampDelta(last_cmd_L_, tgt_left_.load(std::memory_order_relaxed),  slew_per_call_);
    double tgtR = clampDelta(last_cmd_R_, tgt_right_.load(std::memory_order_relaxed), slew_per_call_);

    // Deadband in SPS to avoid tiny noise
    const bool smallL = std::fabs(tgtL - last_cmd_L_) < deadband_sps_;
    const bool smallR = std::fabs(tgtR - last_cmd_R_) < deadband_sps_;
    if (smallL && smallR) return;

    // 2) Planned directions (logical)
    const bool fwdL = L_.forwardFromSps(tgtL);
    const bool fwdR = R_.forwardFromSps(tgtR);

    // 3) Convert to Hz, round + quantize
    unsigned hzL = quantizeHz(clampSpsToHzRounded(tgtL));
    unsigned hzR = quantizeHz(clampSpsToHzRounded(tgtR));

    // 4) Decide what to do
    const bool dirFlip = (fwdL != L_.dirForward()) || (fwdR != R_.dirForward());
    const unsigned oldL = L_.currentHz();
    const unsigned oldR = R_.currentHz();
    const bool changeL = needRetune(oldL, hzL);
    const bool changeR = needRetune(oldR, hzR);

    // Rate-limit retunes to avoid thrashing PWM
    const auto now = clock::now();
    constexpr auto kMinRetuneInterval = std::chrono::milliseconds(5);
    const bool time_ok = (last_tune_tp_.time_since_epoch().count() == 0) ||
                         (now - last_tune_tp_ >= kMinRetuneInterval);

    if (!dirFlip && !( (changeL || changeR) && time_ok )) {
      // No meaningful change to apply right now
      last_cmd_L_ = tgtL;  // still advance our internal ramp even if we don't retime yet
      last_cmd_R_ = tgtR;
      return;
    }

    // 5) If any DIR flips, stop both, set DIRs, short delay once.
    if (dirFlip) {
      L_.stop();
      R_.stop();
      L_.setDirNoWait(fwdL);
      R_.setDirNoWait(fwdR);
      // Typical stepper drivers need ≥1–2 µs DIR setup. 50 µs is conservative but fast.
      time_sleep(0.00005);
    }

    // 6) Start/retime both back-to-back (µs skew)
    L_.startPwmHz(hzL);
    R_.startPwmHz(hzR);

    last_cmd_L_ = tgtL;
    last_cmd_R_ = tgtR;
    last_tune_tp_ = now;
  }

  Stepper& L_;
  Stepper& R_;
  std::mutex mu_;

  std::atomic<double> tgt_left_{0.0}, tgt_right_{0.0};
  double last_cmd_L_{0.0}, last_cmd_R_{0.0};

  const double slew_per_call_;
  const double deadband_sps_;
  std::chrono::steady_clock::time_point last_tune_tp_{};
};
