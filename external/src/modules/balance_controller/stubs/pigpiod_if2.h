// tests/stubs/pigpiod_if2.h
#ifndef PIGPIOD_IF2_STUB_H
#define PIGPIOD_IF2_STUB_H

#ifdef __cplusplus
extern "C" {
#endif

#define PI_INPUT 0
#define PI_OUTPUT 1

#ifndef PIGPIOD_STUB_IMPL
// ---- Declarations only, real lib expected at link time ----
int set_mode(int pi, unsigned gpio, unsigned mode);
int gpio_write(int pi, unsigned gpio, unsigned level);
void time_sleep(double seconds);

int pigpio_start(const char *addr, const char *port);
void pigpio_stop(int pi);

#else
// ---- Inline stub implementations for tests ----
static inline int set_mode(int, unsigned, unsigned) { return 0; }
static inline int gpio_write(int, unsigned, unsigned) { return 0; }
static inline void time_sleep(double) {}

static inline int pigpio_start(const char * /*addr*/, const char * /*port*/) {
  // Return a dummy handle >=0 so PigpioCtx sees success
  return 1;
}
static inline void pigpio_stop(int /*pi*/) {
  // no-op
}
#endif

#ifdef __cplusplus
}
#endif
#endif
