#include "simulator_scheduler.h"

#include <algorithm>

SimulatorTimeScheduler::SimulatorTimeScheduler(uint64_t controller_period_us)
    : controller_period_us_(std::max<uint64_t>(1, controller_period_us)) {
  reset();
}

void SimulatorTimeScheduler::reset(uint64_t start_time_us) {
  current_time_us_ = start_time_us;
  next_controller_time_us_ = start_time_us + controller_period_us_;
  events_.clear();
  events_sorted_ = true;
}

void SimulatorTimeScheduler::controller_sample_processed(uint64_t timestamp_us) {
  advance_to(timestamp_us);
  // Preserve the fixed controller cadence even if a caller reports a sample
  // at an exact event boundary or after a diagnostic pause.
  if (timestamp_us >= next_controller_time_us_) {
    const uint64_t elapsed = timestamp_us - next_controller_time_us_;
    const uint64_t periods = elapsed / controller_period_us_ + 1;
    next_controller_time_us_ += periods * controller_period_us_;
  }
}

void SimulatorTimeScheduler::schedule(SimulatorEvent event) {
  if (event.timestamp_us <= current_time_us_) {
    return;
  }
  events_.push_back(event);
  events_sorted_ = false;
}

void SimulatorTimeScheduler::clear_pending_events() {
  events_.clear();
  events_sorted_ = true;
}

void SimulatorTimeScheduler::schedule_step_events(
    const std::vector<ScheduledStepEvent>& events) {
  for (const auto& event : events) {
    schedule(SimulatorEvent{event.timestamp_us, SimulatorEventKind::Step,
                            event.left_step_delta, event.right_step_delta});
  }
}

std::optional<uint64_t> SimulatorTimeScheduler::next_event_time_us(
    uint64_t inclusive_limit_us) {
  if (!events_sorted_) {
    std::stable_sort(events_.begin(), events_.end(), [](const SimulatorEvent& lhs,
                                                       const SimulatorEvent& rhs) {
      return lhs.timestamp_us < rhs.timestamp_us;
    });
    events_sorted_ = true;
  }
  if (events_.empty() || events_.front().timestamp_us > inclusive_limit_us) {
    return std::nullopt;
  }
  return events_.front().timestamp_us;
}

std::vector<SimulatorEvent> SimulatorTimeScheduler::pop_events_at(uint64_t timestamp_us) {
  std::vector<SimulatorEvent> result;
  pop_events_at(timestamp_us, result);
  return result;
}

void SimulatorTimeScheduler::pop_events_at(uint64_t timestamp_us,
                                            std::vector<SimulatorEvent>& result) {
  // next_event_time_us() normally sorted the batch.  Keep this method robust
  // for direct callers as well.
  if (!events_sorted_) {
    std::stable_sort(events_.begin(), events_.end(), [](const SimulatorEvent& lhs,
                                                       const SimulatorEvent& rhs) {
      return lhs.timestamp_us < rhs.timestamp_us;
    });
    events_sorted_ = true;
  }
  result.clear();
  const auto first = events_.begin();
  const auto split = std::find_if(first, events_.end(), [timestamp_us](const SimulatorEvent& event) {
    return event.timestamp_us > timestamp_us;
  });
  result.insert(result.end(), first, split);
  events_.erase(first, split);
}

void SimulatorTimeScheduler::advance_to(uint64_t timestamp_us) {
  if (timestamp_us < current_time_us_) {
    return;
  }
  current_time_us_ = timestamp_us;
}
