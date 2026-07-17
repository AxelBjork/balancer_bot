#include <algorithm>
#include <iostream>
#include <deque>
#include <map>
#include <mutex>
#include <vector>

#include "pigpiod_if2.h"

// Minimal state for testing
struct StubState {
  std::vector<gpioPulse_t> current_wave_pulses;
  std::map<int, std::vector<gpioPulse_t>> waves;
  int next_wave_id = 0;
  int active_wave_id = -1;
  std::deque<int> queued_wave_ids;
  bool fail_next_wave_create = false;
  std::map<int, int> pin_states;
};

static StubState g_state;
static std::mutex g_mutex;

void pigpio_stub_reset() {
  std::lock_guard<std::mutex> lk(g_mutex);
  g_state = StubState();
}

void pigpio_stub_fail_next_wave_create() {
  std::lock_guard<std::mutex> lk(g_mutex);
  g_state.fail_next_wave_create = true;
}

int set_mode(int, unsigned, unsigned) {
  return 0;
}
int gpio_write(int, unsigned gpio, unsigned level) {
  std::lock_guard<std::mutex> lk(g_mutex);
  g_state.pin_states[static_cast<int>(gpio)] = static_cast<int>(level);
  return 0;
}
void time_sleep(double) {
}

int pigpio_start(const char*, const char*) {
  return 1;
}
void pigpio_stop(int) {
}

int wave_clear(int) {
  std::lock_guard<std::mutex> lk(g_mutex);
  g_state.current_wave_pulses.clear();
  g_state.waves.clear();
  g_state.active_wave_id = -1;
  g_state.queued_wave_ids.clear();
  return 0;
}

int wave_add_new(int) {
  std::lock_guard<std::mutex> lk(g_mutex);
  g_state.current_wave_pulses.clear();
  return 0;
}

int wave_add_generic(int, unsigned numPulses, gpioPulse_t* pulses) {
  std::lock_guard<std::mutex> lk(g_mutex);
  for (unsigned i = 0; i < numPulses; ++i) {
    g_state.current_wave_pulses.push_back(pulses[i]);
  }
  return static_cast<int>(numPulses);
}

int wave_create(int) {
  std::lock_guard<std::mutex> lk(g_mutex);
  if (g_state.fail_next_wave_create) {
    g_state.fail_next_wave_create = false;
    g_state.current_wave_pulses.clear();
    return -1;
  }
  if (g_state.current_wave_pulses.empty()) return -1;

  int id = g_state.next_wave_id++;
  g_state.waves[id] = g_state.current_wave_pulses;
  g_state.current_wave_pulses.clear();
  return id;
}

int wave_delete(int, int wave_id) {
  std::lock_guard<std::mutex> lk(g_mutex);
  g_state.waves.erase(wave_id);
  if (g_state.active_wave_id == wave_id) {
    g_state.active_wave_id = -1;
  }
  g_state.queued_wave_ids.erase(
      std::remove(g_state.queued_wave_ids.begin(), g_state.queued_wave_ids.end(), wave_id),
      g_state.queued_wave_ids.end());
  return 0;
}

int wave_send_using_mode(int, unsigned wave_id, unsigned mode) {
  std::lock_guard<std::mutex> lk(g_mutex);
  const int id = static_cast<int>(wave_id);
  if (g_state.waves.find(id) == g_state.waves.end()) return -1;
  if (mode == PI_WAVE_MODE_ONE_SHOT_SYNC || mode == PI_WAVE_MODE_REPEAT_SYNC) {
    g_state.queued_wave_ids.push_back(id);
  } else {
    g_state.active_wave_id = id;
  }
  return 0;
}

int wave_send_once(int pi, unsigned wave_id) {
  return wave_send_using_mode(pi, wave_id, PI_WAVE_MODE_ONE_SHOT);
}

int wave_send_repeat(int, int wave_id) {
  std::lock_guard<std::mutex> lk(g_mutex);
  if (g_state.waves.find(wave_id) == g_state.waves.end()) return -1;
  g_state.active_wave_id = wave_id;
  return 0;
}

int wave_tx_stop(int) {
  std::lock_guard<std::mutex> lk(g_mutex);
  g_state.active_wave_id = -1;
  g_state.queued_wave_ids.clear();
  return 0;
}

int wave_tx_busy(int) {
  std::lock_guard<std::mutex> lk(g_mutex);
  return g_state.active_wave_id >= 0 ? 1 : 0;
}

int wave_tx_at(int) {
  std::lock_guard<std::mutex> lk(g_mutex);
  return g_state.active_wave_id;
}

std::vector<gpioPulse_t> pigpio_stub_get_wave_pulses(int wave_id) {
  std::lock_guard<std::mutex> lk(g_mutex);
  if (g_state.waves.count(wave_id)) return g_state.waves[wave_id];
  return {};
}

int pigpio_stub_get_active_wave() {
  std::lock_guard<std::mutex> lk(g_mutex);
  return g_state.active_wave_id;
}

int pigpio_stub_get_gpio_level(int pin) {
  std::lock_guard<std::mutex> lk(g_mutex);
  if (g_state.pin_states.count(pin)) return g_state.pin_states[pin];
  return 0;
}
