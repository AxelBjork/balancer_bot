// motor_hat_m1_once_clean.cpp
#include "stepper.h"
#include <stdexcept>

int main() {
  if (gpioInitialise() < 0) {
    throw std::runtime_error("pigpio init failed");
  }

  {
    Stepper::Pins m1Pins{12, 19, 13};   // ENA, STEP, DIR
    Stepper::Pins m2Pins{4, 18, 24};   // ENB, STEP, DIR

    Stepper motor1(m1Pins);
    Stepper motor2(m2Pins);

    // Motor 1 forward/backward
    motor1.stepN(400, 1000, true);
    gpioDelay(200'000);
    motor1.stepN(400, 1000, false);

    gpioDelay(500'000); // pause between motors

    // Motor 2 forward/backward
    motor2.stepN(400, 1000, true);
    gpioDelay(200'000);
    motor2.stepN(400, 1000, false);
  }

  gpioTerminate();
  return 0;
}
