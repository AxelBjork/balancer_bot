#pragma once
#include <algorithm>
#include <chrono>
#include <pigpiod_if2.h>
#include <thread>

class Stepper {
public:
  struct Pins {
    unsigned ena;
    unsigned step;
    unsigned dir;
  };

  Stepper(int pi, const Pins &pins) : pi_(pi), pins_(pins) {
    set_mode(pi_, pins_.ena, PI_OUTPUT);
    set_mode(pi_, pins_.step, PI_OUTPUT);
    set_mode(pi_, pins_.dir, PI_OUTPUT);

    // Energize driver; use pigpio time_sleep so unit-test stubs skip the delay.
    gpio_write(pi_, pins_.ena, 1);
    time_sleep(kWakeDelayUs / 1e6);

    // Initialize DIR to a known state and remember it so we only delay on real
    // changes.
    gpio_write(pi_, pins_.dir, 1);
    last_dir_forward_ = true;
  }

  virtual ~Stepper() {
    gpio_write(pi_, pins_.ena, 0); // de-energize
  }

  inline void stepOnce(unsigned periodUs) const {
    // Respect minimum pulse width; split the remaining time evenly.
    const unsigned half_hi = std::max<unsigned>(periodUs / 2, kMinPulseUs);
    const unsigned half_lo =
        (periodUs > half_hi) ? (periodUs - half_hi) : kMinPulseUs;

    gpio_write(pi_, pins_.step, 1);
    std::this_thread::sleep_for(std::chrono::microseconds(half_hi));
    gpio_write(pi_, pins_.step, 0);
    std::this_thread::sleep_for(std::chrono::microseconds(half_lo));
  }

  virtual void stepN(unsigned steps, unsigned periodUs, bool dirForward) const {
    // Only pay DIR setup delay if direction actually changed.
    if (dirForward != last_dir_forward_) {
      gpio_write(pi_, pins_.dir, dirForward ? 1 : 0);
      std::this_thread::sleep_for(std::chrono::microseconds(kDirSetupDelayUs));
      last_dir_forward_ = dirForward;
    } else {
      // Keep GPIO in the requested state in case something else touched it.
      gpio_write(pi_, pins_.dir, dirForward ? 1 : 0);
    }

    for (unsigned i = 0; i < steps; ++i) {
      stepOnce(periodUs);
    }
  }

private:
  int pi_;
  Pins pins_;
  static constexpr unsigned kWakeDelayUs = 100'000;   // 100 ms
  static constexpr unsigned kDirSetupDelayUs = 2'000; // 2 ms (on dir changes)
  static constexpr unsigned kMinPulseUs = 2;          // STEP high/low minimum

  // Mutable so stepN can remember last direction with a const interface.
  mutable bool last_dir_forward_ = true;
};
