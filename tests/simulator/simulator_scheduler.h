#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "services/motor/motor_runner.h"

// Simulated-time events are ordered independently of wall-clock execution.
// The controller clock is represented explicitly by controller_period_us;
// STEP events are supplied by MotorRunner's timestamped wave schedule.
enum class SimulatorEventKind : uint8_t {
  Step,
  Scenario,
  ImuSample,
};

struct SimulatorEvent {
  uint64_t timestamp_us{0};
  SimulatorEventKind kind{SimulatorEventKind::Scenario};
  int left_step_delta{0};
  int right_step_delta{0};
};

class SimulatorTimeScheduler {
 public:
  explicit SimulatorTimeScheduler(uint64_t controller_period_us);

  void reset(uint64_t start_time_us = 0);

  uint64_t current_time_us() const {
    return current_time_us_;
  }

  uint64_t controller_period_us() const {
    return controller_period_us_;
  }

  uint64_t next_controller_time_us() const {
    return next_controller_time_us_;
  }

  void controller_sample_processed(uint64_t timestamp_us);

  void schedule(SimulatorEvent event);

  void clear_pending_events();

  void schedule_step_events(const std::vector<ScheduledStepEvent>& events);

  std::optional<uint64_t> next_event_time_us(uint64_t inclusive_limit_us);

  std::vector<SimulatorEvent> pop_events_at(uint64_t timestamp_us);

  // Reuse the caller's storage in the simulator hot path.  The returning
  // overload remains convenient for small scheduler unit tests.
  void pop_events_at(uint64_t timestamp_us, std::vector<SimulatorEvent>& result);

  void advance_to(uint64_t timestamp_us);

 private:
  uint64_t controller_period_us_{2500};
  uint64_t current_time_us_{0};
  uint64_t next_controller_time_us_{2500};
  std::vector<SimulatorEvent> events_;
  bool events_sorted_{true};
};
