#pragma once
#include <pigpiod_if2.h>
#include <algorithm>

class Stepper {
public:
  struct Pins {
    unsigned ena;
    unsigned step;
    unsigned dir;
  };

  Stepper(int pi, const Pins& pins) : pi_(pi), pins_(pins) {
    set_mode(pi_, pins_.ena,  PI_OUTPUT);
    set_mode(pi_, pins_.step, PI_OUTPUT);
    set_mode(pi_, pins_.dir,  PI_OUTPUT);

    gpio_write(pi_, pins_.ena, 1);     // enable this motor
    sleep_us(kWakeDelayUs);
  }

  virtual ~Stepper() {
    gpio_write(pi_, pins_.ena, 0);     // de-energize
  }

  void stepOnce(unsigned periodUs) const {
    constexpr unsigned kMinPulse = 2;
    const unsigned half = std::max(periodUs / 2, kMinPulse);

    gpio_write(pi_, pins_.step, 1);
    sleep_us(half);
    gpio_write(pi_, pins_.step, 0);
    sleep_us(half);
  }

  virtual void stepN(unsigned steps, unsigned periodUs, bool dirForward) const {
    gpio_write(pi_, pins_.dir, dirForward ? 1 : 0);
    sleep_us(kDirSetupDelayUs);
    for (unsigned i = 0; i < steps; ++i) stepOnce(periodUs);
  }

private:
  static inline void sleep_us(unsigned us) { time_sleep(us / 1e6); }

  int  pi_;
  Pins pins_;
  static constexpr unsigned kWakeDelayUs    = 100'000; // 100 ms
  static constexpr unsigned kDirSetupDelayUs=   2'000; // 2 ms
};
