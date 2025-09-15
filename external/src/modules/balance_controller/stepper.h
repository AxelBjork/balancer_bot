// Stepper.h
#pragma once
#include <pigpiod_if2.h>
#include <atomic>
#include <cmath>
#include <stdexcept>
#include <atomic>
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
    gpio_write(pi_, pins_.dir, 1);
    last_dir_forward_ = true;
    if (energize_now) { gpio_write(pi_, pins_.ena, 1); time_sleep(0.1); }
  }

  ~Stepper() { stop(); gpio_write(pi_, pins_.ena, 0); }

  void stop() {
    if (running_) { hardware_PWM(pi_, pins_.step, 0, 0); running_ = false; last_freq_ = 0; }
  }

  // ---- helpers used by the pair-apply shim (no logging) ----
  bool forwardFromSps(double sps) const noexcept {
    bool fwd = (sps >= 0.0);
    return invert_ ? !fwd : fwd;
  }
  void setDirNoWait(bool forward) {
    gpio_write(pi_, pins_.dir, forward ? 1 : 0);
    last_dir_forward_ = forward;
  }
  void startPwmHz(unsigned hz) {
    if (hz == 0u) { hardware_PWM(pi_, pins_.step, 0, 0); running_ = false; last_freq_ = 0; }
    else { hardware_PWM(pi_, pins_.step, hz, 500'000); running_ = true; last_freq_ = hz; }
  }
  bool dirForward() const noexcept { return last_dir_forward_; }

  static unsigned clampSpsToHz(double sps,
                               unsigned kMinPulseUs = 2,
                               double   kMaxFreqHz  = 50'000.0) {
    double f = std::fabs(sps);
    const double max_by_pulse = 1e6 / (2.0 * kMinPulseUs);
    if (f > kMaxFreqHz)   f = kMaxFreqHz;
    if (f > max_by_pulse) f = max_by_pulse;
    return (f < 1.0) ? 0u : static_cast<unsigned>(f);
  }

  unsigned currentHz() const noexcept { return last_freq_; }


private:
  int pi_;
  Pins pins_;
  bool invert_{false};
  bool running_{false};
  bool last_dir_forward_{true};
  unsigned last_freq_{0};
  static constexpr unsigned kDirSetupDelayUs = 2000; // 2 ms
};
