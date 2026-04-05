// Stepper.h
#pragma once
#include <pigpiod_if2.h>
#include <atomic>
#include <cmath>
#include <stdexcept>
#include <mutex>

class Stepper {
public:
  struct Pins { unsigned ena, step, dir; };

  Stepper(int pi, const Pins& pins, bool invert=false, bool energize_now=true)
  : pi_(pi), pins_(pins), invert_(invert) {
    if (pi_ < 0) throw std::runtime_error("pigpio not initialized");
    set_mode(pi_, pins_.ena,  PI_OUTPUT);
    set_mode(pi_, pins_.step, PI_OUTPUT);
    set_mode(pi_, pins_.dir,  PI_OUTPUT);

    // Initialize DIR to logical "forward"
    const bool forward = true;
    const bool level = invert_ ? 0 : 1;
    gpio_write(pi_, pins_.dir, level);
    last_dir_forward_ = forward;

    if (energize_now) { gpio_write(pi_, pins_.ena, 1); time_sleep(0.1); }
  }

  ~Stepper() { gpio_write(pi_, pins_.ena, 0); }

  // ---- helpers used by MotorRunner ----
  bool forwardFromSps(double sps) const noexcept {
    bool fwd = (sps >= 0.0);
    return invert_ ? !fwd : fwd;
  }
  void setDirNoWait(bool forward) {
    gpio_write(pi_, pins_.dir, forward ? 1 : 0);
    last_dir_forward_ = forward;
  }
  bool dirForward() const noexcept { return last_dir_forward_; }

  static unsigned clampSpsToHz(double sps,
                               unsigned kMinPulseUs = 2,
                               double   kMaxFreqHz  = 50'000.0) {
    double f = std::fabs(sps);
    const double max_by_pulse = 1e6 / (2.0 * kMinPulseUs);
    if (f > kMaxFreqHz)   f = kMaxFreqHz;
    if (f > max_by_pulse) f = max_by_pulse;
    return (f < 1.0) ? 0u : static_cast<unsigned>(std::llround(f));
  }

  // accessors for the wave engine
  int      pi()       const noexcept { return pi_; }
  unsigned stepPin()  const noexcept { return pins_.step; }

private:
  int pi_;
  Pins pins_;
  bool invert_{false};
  bool last_dir_forward_{true};
};
