// drive_by_xbox.cpp
#include "../motor_runner.h"
#include "../stepper.h"
#include <atomic>
#include <chrono>
#include <cmath>
#include <pigpio.h>
#include <stdexcept>
#include <thread>

// App orchestrates controller → target mapping; keeps main() tiny
class App {
public:
  int run() {
    PigpioCtx _ctx;

    // Pins (BCM): left=M1, right=M2
    Stepper::Pins leftPins{12, 19, 13};
    Stepper::Pins rightPins{4, 18, 24};
    Stepper left(_ctx.handle(), leftPins), right(_ctx.handle(), rightPins);
    return 0;
  };
};

int main() {
  PigpioCtx _ctx; // pigpio RAII
  App app;        // orchestration
  return app.run();
}
