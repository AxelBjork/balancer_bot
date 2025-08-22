// drive_by_xbox.cpp
#include "stepper.h"

#include <pigpio.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

  // RAII for pigpio
  struct PigpioCtx {
    PigpioCtx() {
      if (gpioInitialise() < 0) throw std::runtime_error("pigpio init failed");
    }
    ~PigpioCtx() { gpioTerminate(); }
  };


  // App orchestrates controller → target mapping; keeps main() tiny
  class App {
  public:
    int run() {

      // Pins (BCM): left=M1, right=M2
      Stepper::Pins leftPins  {12, 19, 13};
      Stepper::Pins rightPins { 4, 18, 24};
      Stepper left(leftPins), right(rightPins);
      return 0;
  };
};


int main() {
  PigpioCtx _ctx;       // pigpio RAII
  App app;              // orchestration
  return app.run();
}
