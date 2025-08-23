// pigpiod_if2.h — stub for building/tests on systems without pigpio/pigpiod
// Place this early on your include path, e.g. -Itests/stubs

#ifndef PIGPIOD_IF2_STUB_H
#define PIGPIOD_IF2_STUB_H

// Minimal constants your code uses
#define PI_INPUT  0
#define PI_OUTPUT 1

// If you want time_sleep to actually sleep during non-test builds,
// define PIGPIOD_STUB_SLEEP_REAL before including this header.
#ifdef PIGPIOD_STUB_SLEEP_REAL
  #include <chrono>
  #include <thread>
#endif

// All functions are no-ops that return success by default.
// Signatures match what your code uses.

static inline int set_mode(int /*pi*/, unsigned /*gpio*/, unsigned /*mode*/) {
  return 0;
}

static inline int gpio_write(int /*pi*/, unsigned /*gpio*/, unsigned /*level*/) {
  return 0;
}

// pigpio's time_sleep takes seconds as double.
static inline void time_sleep(double seconds) {
#ifdef PIGPIOD_STUB_SLEEP_REAL
  if (seconds > 0.0) {
    std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
  }
#else
  (void)seconds; // no-op for fast tests
#endif
}

#endif // PIGPIOD_IF2_STUB_H
