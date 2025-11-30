// tests/stubs/pigpiod_if2.h
#ifndef PIGPIOD_IF2_STUB_H
#define PIGPIOD_IF2_STUB_H

#ifdef __cplusplus
extern "C" {
#endif

#define PI_INPUT 0
#define PI_OUTPUT 1

typedef struct {
  uint32_t gpioOn;
  uint32_t gpioOff;
  uint32_t usDelay;
} gpioPulse_t;

#ifndef PIGPIOD_STUB_IMPL
// ---- Declarations only, real lib expected at link time ----
int set_mode(int pi, unsigned gpio, unsigned mode);
int gpio_write(int pi, unsigned gpio, unsigned level);
void time_sleep(double seconds);

int pigpio_start(const char *addr, const char *port);
void pigpio_stop(int pi);

typedef struct {
  uint32_t gpioOn;
  uint32_t gpioOff;
  uint32_t usDelay;
} gpioPulse_t;

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

// Wave stubs
static inline int wave_clear(int) { return 0; }
static inline int wave_add_generic(int, unsigned, gpioPulse_t *) { return 0; }
static inline int wave_create(int) { return 0; }
static inline int wave_delete(int, int) { return 0; }
static inline int wave_send_repeat(int, int) { return 0; }
static inline int wave_tx_stop(int) { return 0; }
#endif

#ifdef __cplusplus
}
#endif
#endif
