#include <pigpio.h>
#include <chrono>
#include <thread>
#include <string_view>

class Stepper {
public:
  struct Pins {
    unsigned ena;
    unsigned step;
    unsigned dir;
  };

  Stepper(const Pins& pins) : pins_(pins) {
    // Set modes
    gpioSetMode(pins_.ena,      PI_OUTPUT);
    gpioSetMode(pins_.step,     PI_OUTPUT);
    gpioSetMode(pins_.dir,      PI_OUTPUT);

    gpioWrite(pins_.ena, 1);      // enable this motor
    gpioDelay(kWakeDelayUs);
  }

  ~Stepper() {
    // Ensure motor is de-energized
    gpioWrite(pins_.ena, 0);
  }

  void stepOnce(unsigned periodUs) const {
    // Ensure at least minimal pulse width
    constexpr unsigned kMinPulse = 2;
    const unsigned half = std::max(periodUs / 2, kMinPulse);

    gpioWrite(pins_.step, 1);
    gpioDelay(half);
    gpioWrite(pins_.step, 0);
    gpioDelay(half);
  }

  void stepN(unsigned steps, unsigned periodUs, bool dirForward) const {
    gpioWrite(pins_.dir, dirForward ? 1 : 0);
    gpioDelay(kDirSetupDelayUs);
    for (unsigned i = 0; i < steps; ++i) {
      stepOnce(periodUs);
    }
  }

private:
  Pins pins_;
  static constexpr unsigned kWakeDelayUs    = 100'000; // 100 ms
  static constexpr unsigned kDirSetupDelayUs=   2'000; // 2 ms
};
