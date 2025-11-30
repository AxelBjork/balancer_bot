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

int pigpio_start(const char* addr, const char* port);
void pigpio_stop(int pi);

typedef struct {
  uint32_t gpioOn;
  uint32_t gpioOff;
  uint32_t usDelay;
} gpioPulse_t;

#else
// ---- Declarations for test stubs (implemented in pigpiod_stub.cpp) ----
int set_mode(int pi, unsigned gpio, unsigned mode);
int gpio_write(int pi, unsigned gpio, unsigned level);
void time_sleep(double seconds);

int pigpio_start(const char* addr, const char* port);
void pigpio_stop(int pi);

// Wave stubs
int wave_clear(int pi);
int wave_add_generic(int pi, unsigned numPulses, gpioPulse_t* pulses);
int wave_create(int pi);
int wave_delete(int pi, int wave_id);
int wave_send_repeat(int pi, int wave_id);
int wave_tx_stop(int pi);

// Helper to reset stub state (not part of pigpio API)
void pigpio_stub_reset();
#endif

#ifdef __cplusplus
}
#endif
#endif
