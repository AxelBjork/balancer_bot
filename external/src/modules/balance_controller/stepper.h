// Stepper.h  (unchanged API)
#pragma once
#include <atomic>
#include <pigpiod_if2.h>
#include <stdexcept>
#include <cmath>

class Stepper {
public:
  struct Pins { unsigned ena; unsigned step; unsigned dir; };

  Stepper(int pi, const Pins& pins, bool invert=false, bool energize_now=true)
  : pi_(pi), pins_(pins), invert_(invert) {
    if (pi_<0) throw std::runtime_error("pigpio not initialized");
    set_mode(pi_, pins_.ena,  PI_OUTPUT);
    set_mode(pi_, pins_.step, PI_OUTPUT);
    set_mode(pi_, pins_.dir,  PI_OUTPUT);
    gpio_write(pi_, pins_.dir, 1);
    last_dir_forward_ = true;
    if (energize_now) { gpio_write(pi_, pins_.ena, 1); time_sleep(0.1); energized_=true; }
  }

  ~Stepper(){ stop(); if (energized_) gpio_write(pi_, pins_.ena, 0); }

  // Called by PID thread (cheap): just stores.
  void setTarget(double sps){ target_sps_.store(sps, std::memory_order_relaxed); }

  double target()   const { return target_sps_.load(std::memory_order_relaxed); }
  double actual()   const { return actual_sps_.load(std::memory_order_relaxed); }

  // Called by coordinator to (re)apply PWM settings NOW (no smoothing here).
  // Returns the pwm frequency actually set (signed sps after invert).
  double applyNow() {
    double sps = target();
    bool forward = (sps >= 0.0);
    if (invert_) forward = !forward;

    double freq = std::fabs(sps);
    const double max_by_pulse = 1e6 / (2.0 * kMinPulseUs);
    if (freq > kMaxFreqHz)   freq = kMaxFreqHz;
    if (freq > max_by_pulse) freq = max_by_pulse;

    // If direction changed while running, stop, flip DIR, wait setup
    if (running_ && (forward != last_dir_forward_)) {
      hardware_PWM(pi_, pins_.step, 0, 0); running_ = false;
      gpio_write(pi_, pins_.dir, forward ? 1 : 0);
      time_sleep(kDirSetupDelayUs / 1e6);
      last_dir_forward_ = forward;
    } else {
      gpio_write(pi_, pins_.dir, forward ? 1 : 0);
      last_dir_forward_ = forward;
    }

    if (freq < 1.0) {
      hardware_PWM(pi_, pins_.step, 0, 0); running_ = false;
      actual_sps_.store(0.0, std::memory_order_relaxed);
      return 0.0;
    }

    const unsigned pwm_freq = static_cast<unsigned>(freq);
    hardware_PWM(pi_, pins_.step, pwm_freq, 500'000); // 50% duty
    running_ = true;

    const double signed_sps = (forward ? +1.0 : -1.0) * static_cast<double>(pwm_freq) * (invert_ ? -1.0 : 1.0);
    actual_sps_.store(signed_sps, std::memory_order_relaxed);
    return signed_sps;
  }

  void stop(){
    if (running_) { hardware_PWM(pi_, pins_.step, 0, 0); running_=false; actual_sps_.store(0.0,std::memory_order_relaxed); }
  }

private:
  int pi_;
  Pins pins_;
  bool invert_{false}, energized_{false}, running_{false}, last_dir_forward_{true};
  std::atomic<double> target_sps_{0.0}, actual_sps_{0.0};
  static constexpr unsigned kDirSetupDelayUs = 200;
  static constexpr unsigned kMinPulseUs      = 2;
  static constexpr double   kMaxFreqHz       = 50'000.0;
};
