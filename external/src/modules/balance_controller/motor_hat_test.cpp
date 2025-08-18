// motor_hat_m1_once_clean.cpp
#include "stepper.h"
#include <stdexcept>

int main() {
  if (gpioInitialise() < 0) {
    throw std::runtime_error("pigpio init failed");
  }

  {
    Stepper::Pins m1Pins{12, 19, 13, 4};
    Stepper motor(m1Pins);

    // Forward ~2 revs
    motor.stepN(600, 1200, true);

    gpioDelay(50'000); // pause

    // Backward ~2 revs
    motor.stepN(600, 1200, false);
  } // motor destructor de-energizes coils

  gpioTerminate();
  return 0;
}
