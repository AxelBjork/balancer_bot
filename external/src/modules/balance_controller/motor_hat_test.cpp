// motor_hat_m1_ramp.cpp
#include <cstdio>
#include <cmath>
#include <pigpio.h>

static void step_pulse(int stepPin, int usDelay) {
  gpioWrite(stepPin, 1); gpioDelay(usDelay);
  gpioWrite(stepPin, 0); gpioDelay(usDelay);
}

static void move_with_ramp(int stepPin, int dirPin,
                           int total_steps,
                           int us_start,  // e.g. 4000 (≈125 Hz)
                           int us_cruise) // e.g. 2000 (≈250 Hz)
{
  if (total_steps < 4) total_steps = 4;
  int accel = total_steps / 4;
  int decel = total_steps / 4;
  int cruise = total_steps - accel - decel;

  // forward
  gpioWrite(dirPin, 1);

  // accel
  for (int i = 0; i < accel; ++i) {
    int us = us_start - (us_start - us_cruise) * (i + 1) / accel;
    step_pulse(stepPin, us);
  }
  // cruise
  for (int i = 0; i < cruise; ++i) step_pulse(stepPin, us_cruise);
  // decel
  for (int i = 0; i < decel; ++i) {
    int us = us_cruise + (us_start - us_cruise) * (i + 1) / decel;
    step_pulse(stepPin, us);
  }

  gpioDelay(200000); // 200 ms pause

  // reverse
  gpioWrite(dirPin, 0);
  for (int i = 0; i < accel; ++i) {
    int us = us_start - (us_start - us_cruise) * (i + 1) / accel;
    step_pulse(stepPin, us);
  }
  for (int i = 0; i < cruise; ++i) step_pulse(stepPin, us_cruise);
  for (int i = 0; i < decel; ++i) {
    int us = us_cruise + (us_start - us_cruise) * (i + 1) / decel;
    step_pulse(stepPin, us);
  }
}

int main() {
  if (gpioInitialise() < 0) { std::fprintf(stderr, "pigpio init failed\n"); return 1; }

  // --- Pins (BCM) ---
  const int ENA  = 12;  // M1 enable (HIGH = enabled)
  const int STEP1= 19;  // M1 step
  const int DIR1 = 13;  // M1 dir

  const int ENB  = 4;   // M2 enable (keep low)
  const int STEP2= 18;  // M2 step (unused)
  const int DIR2 = 24;  // M2 dir  (unused)

  gpioSetMode(ENA,  PI_OUTPUT);
  gpioSetMode(STEP1,PI_OUTPUT);
  gpioSetMode(DIR1, PI_OUTPUT);
  gpioSetMode(ENB,  PI_OUTPUT);
  gpioWrite(ENB, 0);       // ensure M2 disabled
  gpioWrite(ENA, 1);       // enable M1
  gpioDelay(100000);

  // 400 steps each way; with full-step that's ~2 revs total
  move_with_ramp(STEP1, DIR1, /*total_steps=*/400, /*us_start=*/4000, /*us_cruise=*/2000);

  // de-energize coils
  gpioWrite(ENA, 0);
  gpioTerminate();
  return 0;
}
