#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <vector>

#include <pigpiod_if2.h>

#include "messages/types.h"
#include "services/main/config.h"
#include "services/motor/stepper.h"

struct MotorFeedbackSample {
  double left_slewed_sps{0.0};
  double right_slewed_sps{0.0};
  double measured_avg_sps{0.0};
  double update_dt_ms{0.0};
  double feedback_age_ms{0.0};
  int64_t left_actual_steps{0};
  int64_t right_actual_steps{0};
  uint32_t actuator_saturation_flags{0};
  bool actuator_fault{false};
};

struct ScheduledStepPosition {
  double left_steps{0.0};
  double right_steps{0.0};
};

// A physical STEP edge in the simulator's wave timeline.  The controller
// still queues 2.5 ms frames, but the simulator can consume these edges at
// their actual timestamps instead of applying an aggregate frame displacement
// at the next controller sample.
struct ScheduledStepEvent {
  uint64_t timestamp_us{0};
  int left_step_delta{0};
  int right_step_delta{0};
};

inline constexpr size_t kVelocityHistorySize = 64;
inline constexpr double kVelocityEstimateWindowS = 0.05;

// Backend boundary for one paired, fixed-duration pulse frame. Tests can inject a deterministic
// recorder while production uses pigpio waves.
class WaveFrameBackend {
 public:
  virtual ~WaveFrameBackend() = default;
  virtual int queueFrame(unsigned left_pulses, unsigned right_pulses, bool synchronous) = 0;
  virtual void deleteFrame(int frame_id) = 0;
  virtual void stop() = 0;
};

class DualWave final : public WaveFrameBackend {
 public:
  static constexpr unsigned kFrameUs = 2500;
  static constexpr unsigned kMinPulseUs = 2;
  static constexpr unsigned kMaxScheduledHz =
      static_cast<unsigned>(Config::max_step_rate_sps);
  static constexpr unsigned kMinPulse = kMinPulseUs;
  static constexpr unsigned kMaxN =
      (kMaxScheduledHz * kFrameUs + 999999u) / 1000000u;

  DualWave(int pi, unsigned step_left, unsigned step_right)
      : pi_(pi), step_left_(step_left), step_right_(step_right) {
    wave_clear(pi_);
  }

  int queueFrame(unsigned left_pulses, unsigned right_pulses, bool synchronous) override {
    std::lock_guard<std::mutex> lock(mu_);
    left_pulses = std::min(left_pulses, kMaxN);
    right_pulses = std::min(right_pulses, kMaxN);

    const unsigned left_edges = scheduleChannel(step_left_, left_pulses, edges_left_);
    const unsigned right_edges = scheduleChannel(step_right_, right_pulses, edges_right_);
    const unsigned edge_count =
        mergeEdges(edges_left_, left_edges, edges_right_, right_edges, edges_);
    const unsigned pulse_count = buildFramePulses(edges_, edge_count, pulses_);

    if (wave_add_new(pi_) < 0 || wave_add_generic(pi_, pulse_count, pulses_) < 0) {
      return -1;
    }
    const int frame_id = wave_create(pi_);
    if (frame_id < 0) {
      return -1;
    }

    const unsigned mode = synchronous ? PI_WAVE_MODE_ONE_SHOT_SYNC : PI_WAVE_MODE_ONE_SHOT;
    if (wave_send_using_mode(pi_, static_cast<unsigned>(frame_id), mode) < 0) {
      wave_delete(pi_, frame_id);
      return -1;
    }
    return frame_id;
  }

  void deleteFrame(int frame_id) override {
    if (frame_id < 0) {
      return;
    }
    std::lock_guard<std::mutex> lock(mu_);
    wave_delete(pi_, frame_id);
  }

  void stop() override {
    std::lock_guard<std::mutex> lock(mu_);
    wave_tx_stop(pi_);
    wave_clear(pi_);
  }

 private:
  struct Edge {
    uint32_t time_us;
    uint32_t on_mask;
    uint32_t off_mask;
  };

  static uint32_t bit(unsigned gpio) {
    return 1u << gpio;
  }

  static unsigned scheduleChannel(unsigned gpio, unsigned pulse_count, Edge* out) {
    if (pulse_count == 0) {
      return 0;
    }
    unsigned edge_count = 0;
    for (unsigned index = 0; index < pulse_count; ++index) {
      const double slot = (static_cast<double>(index) + 0.5) *
                          static_cast<double>(kFrameUs) / static_cast<double>(pulse_count);
      uint32_t on_time = static_cast<uint32_t>(std::llround(slot));
      on_time = std::min(on_time, kFrameUs - kMinPulse - 1u);
      out[edge_count++] = Edge{on_time, bit(gpio), 0u};
      out[edge_count++] = Edge{on_time + kMinPulse, 0u, bit(gpio)};
    }
    return edge_count;
  }

  static unsigned mergeEdges(const Edge* left, unsigned left_count, const Edge* right,
                             unsigned right_count, Edge* out) {
    unsigned left_index = 0;
    unsigned right_index = 0;
    unsigned result_count = 0;
    while (left_index < left_count || right_index < right_count) {
      const uint32_t left_time =
          left_index < left_count ? left[left_index].time_us : UINT32_MAX;
      const uint32_t right_time =
          right_index < right_count ? right[right_index].time_us : UINT32_MAX;
      const uint32_t time_us = std::min(left_time, right_time);
      uint32_t on_mask = 0;
      uint32_t off_mask = 0;
      while (left_index < left_count && left[left_index].time_us == time_us) {
        on_mask |= left[left_index].on_mask;
        off_mask |= left[left_index].off_mask;
        ++left_index;
      }
      while (right_index < right_count && right[right_index].time_us == time_us) {
        on_mask |= right[right_index].on_mask;
        off_mask |= right[right_index].off_mask;
        ++right_index;
      }
      out[result_count++] = Edge{time_us, on_mask, off_mask};
    }
    return result_count;
  }

  static unsigned buildFramePulses(const Edge* edges, unsigned edge_count, gpioPulse_t* pulses) {
    if (edge_count == 0) {
      pulses[0] = gpioPulse_t{0u, 0u, kFrameUs};
      return 1;
    }

    unsigned pulse_count = 0;
    uint32_t cursor_us = 0;
    for (unsigned index = 0; index < edge_count; ++index) {
      if (edges[index].time_us > cursor_us) {
        pulses[pulse_count++] = gpioPulse_t{0u, 0u, edges[index].time_us - cursor_us};
        cursor_us = edges[index].time_us;
      }
      const uint32_t next_time =
          (index + 1u < edge_count) ? edges[index + 1u].time_us : kFrameUs;
      const uint32_t delay_us = std::max(1u, next_time - cursor_us);
      pulses[pulse_count++] =
          gpioPulse_t{edges[index].on_mask, edges[index].off_mask, delay_us};
      cursor_us += delay_us;
    }
    if (cursor_us < kFrameUs) {
      pulses[pulse_count++] = gpioPulse_t{0u, 0u, kFrameUs - cursor_us};
    }
    return pulse_count;
  }

  int pi_;
  unsigned step_left_;
  unsigned step_right_;
  std::mutex mu_;

  static constexpr unsigned kMaxEdgesPerChannel = 2u * kMaxN;
  static constexpr unsigned kMaxMergedEdges = 2u * kMaxEdgesPerChannel;
  static constexpr unsigned kMaxPulses = kMaxMergedEdges + 2u;
  Edge edges_left_[kMaxEdgesPerChannel]{};
  Edge edges_right_[kMaxEdgesPerChannel]{};
  Edge edges_[kMaxMergedEdges]{};
  gpioPulse_t pulses_[kMaxPulses]{};
};

class MotorRunner {
 public:
  using FeedbackSample = MotorFeedbackSample;

  MotorRunner(Stepper& left, Stepper& right, double control_hz = 1000.0,
              double max_slew_sps_per_s = 200000.0, WaveFrameBackend* backend = nullptr)
      : left_(left),
        right_(right),
        pigpio_backend_(left.pi(), left.stepPin(), right.stepPin()),
        backend_(backend != nullptr ? *backend : static_cast<WaveFrameBackend&>(pigpio_backend_)),
        nominal_control_dt_s_(1.0 / (control_hz > 0.0 ? control_hz : 1.0)),
        max_slew_sps_per_s_(std::max(0.0, max_slew_sps_per_s)) {
  }

  void setTargets(double left_sps, double right_sps, uint64_t timestamp_us) {
    current_time_us_.store(timestamp_us, std::memory_order_relaxed);
    target_left_sps_.store(std::clamp(left_sps, -static_cast<double>(DualWave::kMaxScheduledHz),
                                     static_cast<double>(DualWave::kMaxScheduledHz)),
                           std::memory_order_relaxed);
    target_right_sps_.store(std::clamp(right_sps, -static_cast<double>(DualWave::kMaxScheduledHz),
                                      static_cast<double>(DualWave::kMaxScheduledHz)),
                            std::memory_order_relaxed);
    applyOnce(timestamp_us);
  }

  void setLeft(double sps) {
    setTargets(sps, target_right_sps_.load(std::memory_order_relaxed),
               current_time_us_.load(std::memory_order_relaxed));
  }

  void setRight(double sps) {
    setTargets(target_left_sps_.load(std::memory_order_relaxed), sps,
               current_time_us_.load(std::memory_order_relaxed));
  }

  void stop() {
    std::lock_guard<std::mutex> lock(mu_);
    backend_.stop();
    frame_count_ = 0;
    next_frame_start_us_ = 0;
    last_command_left_sps_ = 0.0;
    last_command_right_sps_ = 0.0;
    phase_left_ = 0.0;
    phase_right_ = 0.0;
    measured_avg_sps_ = 0.0;
    actuator_saturation_flags_ = ActuatorSaturationNone;
    velocity_history_count_ = 0;
    velocity_history_head_ = 0;
    have_last_call_time_ = false;
    actuator_fault_ = false;
  }

  // Simulator-only transient reset. Step counters remain physical history;
  // queued pulses, slew phase, and velocity-estimator history do not.
  void resetTransientStateForSimulation() {
    std::lock_guard<std::mutex> lock(mu_);
    backend_.stop();
    frame_count_ = 0;
    next_frame_start_us_ = 0;
    last_command_left_sps_ = 0.0;
    last_command_right_sps_ = 0.0;
    phase_left_ = 0.0;
    phase_right_ = 0.0;
    measured_avg_sps_ = 0.0;
    actuator_saturation_flags_ = ActuatorSaturationNone;
    velocity_history_count_ = 0;
    velocity_history_head_ = 0;
    have_last_call_time_ = false;
    last_completed_frame_us_ = 0;
    last_update_dt_s_ = 0.0;
    actuator_fault_ = false;
    target_left_sps_.store(0.0, std::memory_order_relaxed);
    target_right_sps_.store(0.0, std::memory_order_relaxed);
  }

  int64_t getLeftSteps() const {
    return actual_steps_left_.load(std::memory_order_relaxed);
  }
  int64_t getRightSteps() const {
    return actual_steps_right_.load(std::memory_order_relaxed);
  }
  int64_t getActualLeftSteps() const {
    return actual_steps_left_.load(std::memory_order_relaxed);
  }
  int64_t getActualRightSteps() const {
    return actual_steps_right_.load(std::memory_order_relaxed);
  }

  double getAverageSpeedSps() const {
    return 0.5 * (target_left_sps_.load(std::memory_order_relaxed) +
                  target_right_sps_.load(std::memory_order_relaxed));
  }

  FeedbackSample getFeedbackSample() const {
    std::lock_guard<std::mutex> lock(mu_);
    FeedbackSample sample{};
    sample.left_slewed_sps = last_command_left_sps_;
    sample.right_slewed_sps = last_command_right_sps_;
    sample.update_dt_ms = last_update_dt_s_ * 1000.0;
    const uint64_t now_us = current_time_us_.load(std::memory_order_relaxed);
    if (last_completed_frame_us_ > 0 && now_us >= last_completed_frame_us_) {
      sample.feedback_age_ms =
          static_cast<double>(now_us - last_completed_frame_us_) / 1000.0;
    }

    updateVelocityEstimateLocked(now_us);
    sample.measured_avg_sps = measured_avg_sps_;
    sample.left_actual_steps = actual_steps_left_.load(std::memory_order_relaxed);
    sample.right_actual_steps = actual_steps_right_.load(std::memory_order_relaxed);
    sample.actuator_saturation_flags = actuator_saturation_flags_;
    sample.actuator_fault = actuator_fault_;
    return sample;
  }

  double getActualSpeedSps() {
    return getFeedbackSample().measured_avg_sps;
  }

  // Physical pulse position at an arbitrary time, including pulses already
  // emitted inside the active frame. Completed-pulse feedback deliberately
  // remains unchanged until the whole frame retires.
  ScheduledStepPosition getScheduledStepPosition(uint64_t timestamp_us) const {
    std::lock_guard<std::mutex> lock(mu_);
    ScheduledStepPosition position{
        static_cast<double>(actual_steps_left_.load(std::memory_order_relaxed)),
        static_cast<double>(actual_steps_right_.load(std::memory_order_relaxed)),
    };
    for (size_t index = 0; index < frame_count_; ++index) {
      const ScheduledFrame& frame = frames_[index];
      if (timestamp_us <= frame.start_us) continue;
      const uint64_t elapsed_us = std::min(timestamp_us, frame.end_us) - frame.start_us;
      position.left_steps += frame.left_direction * emittedPulses(frame.left_pulses, elapsed_us);
      position.right_steps += frame.right_direction * emittedPulses(frame.right_pulses, elapsed_us);
    }
    return position;
  }

  // Returns all STEP edges in (start_us, end_us].  Events are derived from
  // the same queued frame schedule used by the production wave backend; no
  // second SPS integrator is introduced.  The half-open lower bound prevents
  // an edge already consumed by the preceding simulator interval from being
  // applied twice.
  std::vector<ScheduledStepEvent> getScheduledStepEvents(uint64_t start_us,
                                                         uint64_t end_us) const {
    std::vector<ScheduledStepEvent> events;
    getScheduledStepEvents(start_us, end_us, events);
    return events;
  }

  void getScheduledStepEvents(uint64_t start_us, uint64_t end_us,
                              std::vector<ScheduledStepEvent>& events) const {
    std::lock_guard<std::mutex> lock(mu_);
    events.clear();
    if (end_us <= start_us) {
      return;
    }

    for (size_t index = 0; index < frame_count_; ++index) {
      const ScheduledFrame& frame = frames_[index];
      appendFrameEvents(frame, start_us, end_us, true, events);
      appendFrameEvents(frame, start_us, end_us, false, events);
    }

    std::sort(events.begin(), events.end(), [](const ScheduledStepEvent& lhs,
                                               const ScheduledStepEvent& rhs) {
      return lhs.timestamp_us < rhs.timestamp_us;
    });

    // Both motors can have an edge at the same wave timestamp.  Present one
    // deterministic scheduler event so the plant sees the simultaneous
    // paired update.
    size_t merged_size = 0;
    for (const auto& event : events) {
      if (merged_size > 0 && events[merged_size - 1].timestamp_us == event.timestamp_us) {
        events[merged_size - 1].left_step_delta += event.left_step_delta;
        events[merged_size - 1].right_step_delta += event.right_step_delta;
      } else {
        events[merged_size++] = event;
      }
    }
    events.resize(merged_size);
  }

 private:
  struct ScheduledFrame {
    int backend_id{-1};
    uint64_t start_us{0};
    uint64_t end_us{0};
    unsigned left_pulses{0};
    unsigned right_pulses{0};
    double left_direction{1.0};
    double right_direction{1.0};
  };

  static void appendFrameEvents(const ScheduledFrame& frame, uint64_t start_us,
                                uint64_t end_us, bool left_channel,
                                std::vector<ScheduledStepEvent>& events) {
    const unsigned pulse_count = left_channel ? frame.left_pulses : frame.right_pulses;
    if (pulse_count == 0) {
      return;
    }
    const int direction = static_cast<int>(left_channel ? frame.left_direction
                                                         : frame.right_direction);
    for (unsigned pulse = 0; pulse < pulse_count; ++pulse) {
      // Match DualWave::scheduleChannel exactly, including its rounded event
      // placement and the one-microsecond pulse width guard.
      const double slot = (static_cast<double>(pulse) + 0.5) *
                          static_cast<double>(DualWave::kFrameUs) /
                          static_cast<double>(pulse_count);
      uint64_t offset_us = static_cast<uint64_t>(std::llround(slot));
      offset_us = std::min<uint64_t>(offset_us,
                                     DualWave::kFrameUs - DualWave::kMinPulse - 1u);
      const uint64_t timestamp_us = frame.start_us + offset_us;
      if (timestamp_us > start_us && timestamp_us <= end_us) {
        ScheduledStepEvent event{};
        event.timestamp_us = timestamp_us;
        if (left_channel) {
          event.left_step_delta = direction;
        } else {
          event.right_step_delta = direction;
        }
        events.push_back(event);
      }
    }
  }

  static double clampDelta(double from, double to, double max_delta) {
    return std::clamp(to, from - max_delta, from + max_delta);
  }

  static unsigned pulsesForFrame(double sps, double& phase) {
    const double requested = std::abs(sps) * static_cast<double>(DualWave::kFrameUs) / 1e6;
    const double total = requested + phase;
    const unsigned pulses = std::min(
        static_cast<unsigned>(std::floor(total + 1e-12)), DualWave::kMaxN);
    phase = std::clamp(total - static_cast<double>(pulses), 0.0, 1.0 - 1e-12);
    return pulses;
  }

  static double emittedPulses(unsigned pulses, uint64_t elapsed_us) {
    if (pulses == 0) return 0.0;
    const double slots = static_cast<double>(pulses) *
                         static_cast<double>(elapsed_us) /
                         static_cast<double>(DualWave::kFrameUs);
    return std::min(static_cast<double>(pulses), std::floor(slots + 0.5));
  }

  static double logicalDirection(double sps, double fallback) {
    if (sps > 0.0) return 1.0;
    if (sps < 0.0) return -1.0;
    return fallback;
  }

  void clearQueuedFramesLocked() {
    for (size_t index = 0; index < frame_count_; ++index) {
      backend_.deleteFrame(frames_[index].backend_id);
    }
    frame_count_ = 0;
  }

  // Stopping a one-shot wave does not undo pulses that have already reached
  // STEP. Retire that emitted prefix before changing DIR so both plant motion
  // and completed-pulse feedback remain monotonic across a reversal.
  void accountEmittedAndClearLocked(uint64_t now_us) {
    double left_delta = 0.0;
    double right_delta = 0.0;
    for (size_t index = 0; index < frame_count_; ++index) {
      const ScheduledFrame& frame = frames_[index];
      if (now_us <= frame.start_us) continue;
      const uint64_t elapsed_us = std::min(now_us, frame.end_us) - frame.start_us;
      left_delta += frame.left_direction * emittedPulses(frame.left_pulses, elapsed_us);
      right_delta += frame.right_direction * emittedPulses(frame.right_pulses, elapsed_us);
    }
    clearQueuedFramesLocked();
    actual_steps_left_.fetch_add(static_cast<int64_t>(left_delta), std::memory_order_relaxed);
    actual_steps_right_.fetch_add(static_cast<int64_t>(right_delta), std::memory_order_relaxed);
    if (left_delta != 0.0 || right_delta != 0.0) {
      last_completed_frame_us_ = now_us;
      const double average_steps = 0.5 * static_cast<double>(
          actual_steps_left_.load(std::memory_order_relaxed) +
          actual_steps_right_.load(std::memory_order_relaxed));
      recordVelocityHistoryLocked(now_us, average_steps);
    }
  }

  void retireCompletedFramesLocked(uint64_t now_us) {
    while (frame_count_ > 0 && frames_[0].end_us <= now_us) {
      const ScheduledFrame completed = frames_[0];
      backend_.deleteFrame(completed.backend_id);
      actual_steps_left_.fetch_add(
          static_cast<int64_t>(completed.left_direction * completed.left_pulses),
          std::memory_order_relaxed);
      actual_steps_right_.fetch_add(
          static_cast<int64_t>(completed.right_direction * completed.right_pulses),
          std::memory_order_relaxed);
      last_completed_frame_us_ = completed.end_us;
      const double average_steps =
          0.5 * static_cast<double>(actual_steps_left_.load(std::memory_order_relaxed) +
                                    actual_steps_right_.load(std::memory_order_relaxed));
      recordVelocityHistoryLocked(completed.end_us, average_steps);
      for (size_t index = 1; index < frame_count_; ++index) {
        frames_[index - 1] = frames_[index];
      }
      --frame_count_;
    }
  }

  void handleDirectionChangeLocked(uint64_t now_us) {
    const bool desired_left = std::abs(last_command_left_sps_) > 1e-9
                                  ? left_.forwardFromSps(last_command_left_sps_)
                                  : left_.dirForward();
    const bool desired_right = std::abs(last_command_right_sps_) > 1e-9
                                   ? right_.forwardFromSps(last_command_right_sps_)
                                   : right_.dirForward();
    if (desired_left == left_.dirForward() && desired_right == right_.dirForward()) {
      return;
    }

    backend_.stop();
    accountEmittedAndClearLocked(now_us);
    left_.setStepLow();
    right_.setStepLow();
    if (desired_left != left_.dirForward()) {
      left_.setDirNoWait(desired_left);
      phase_left_ = 0.0;
    }
    if (desired_right != right_.dirForward()) {
      right_.setDirNoWait(desired_right);
      phase_right_ = 0.0;
    }
    time_sleep(0.00005);
    next_frame_start_us_ = now_us;
  }

  void fillFrameQueueLocked(uint64_t now_us) {
    while (frame_count_ < frames_.size() && !actuator_fault_) {
      ScheduledFrame frame{};
      frame.start_us = std::max(now_us, next_frame_start_us_);
      frame.end_us = frame.start_us + DualWave::kFrameUs;
      frame.left_direction = logicalDirection(last_command_left_sps_, last_left_direction_);
      frame.right_direction = logicalDirection(last_command_right_sps_, last_right_direction_);
      frame.left_pulses = pulsesForFrame(last_command_left_sps_, phase_left_);
      frame.right_pulses = pulsesForFrame(last_command_right_sps_, phase_right_);
      frame.backend_id = backend_.queueFrame(frame.left_pulses, frame.right_pulses,
                                             frame_count_ > 0);
      if (frame.backend_id < 0) {
        actuator_fault_ = true;
        backend_.stop();
        accountEmittedAndClearLocked(now_us);
        return;
      }
      frames_[frame_count_++] = frame;
      next_frame_start_us_ = frame.end_us;
      last_left_direction_ = frame.left_direction;
      last_right_direction_ = frame.right_direction;
    }
  }

  void applyOnce(uint64_t now_us) {
    std::lock_guard<std::mutex> lock(mu_);
    actuator_saturation_flags_ = ActuatorSaturationNone;
    retireCompletedFramesLocked(now_us);
    if (actuator_fault_) {
      return;
    }

    double dt_s = nominal_control_dt_s_;
    if (have_last_call_time_ && now_us > last_call_time_us_) {
      dt_s = static_cast<double>(now_us - last_call_time_us_) / 1e6;
    }
    last_update_dt_s_ = dt_s;
    last_call_time_us_ = now_us;
    have_last_call_time_ = true;

    const double max_delta = max_slew_sps_per_s_ * dt_s;
    const double target_left_sps = target_left_sps_.load(std::memory_order_relaxed);
    const double target_right_sps = target_right_sps_.load(std::memory_order_relaxed);
    const double next_left_sps = clampDelta(last_command_left_sps_, target_left_sps, max_delta);
    const double next_right_sps = clampDelta(last_command_right_sps_, target_right_sps, max_delta);
    if (next_left_sps != target_left_sps) {
      actuator_saturation_flags_ |= ActuatorSaturationLeftSlew;
    }
    if (next_right_sps != target_right_sps) {
      actuator_saturation_flags_ |= ActuatorSaturationRightSlew;
    }
    last_command_left_sps_ = next_left_sps;
    last_command_right_sps_ = next_right_sps;

    handleDirectionChangeLocked(now_us);
    if (velocity_history_count_ == 0) {
      recordVelocityHistoryLocked(now_us, 0.5 * static_cast<double>(
          actual_steps_left_.load(std::memory_order_relaxed) +
          actual_steps_right_.load(std::memory_order_relaxed)));
    }
    fillFrameQueueLocked(now_us);
  }

  void recordVelocityHistoryLocked(uint64_t timestamp_us, double average_steps) const {
    velocity_history_time_us_[velocity_history_head_] = timestamp_us;
    velocity_history_steps_[velocity_history_head_] = average_steps;
    velocity_history_head_ = (velocity_history_head_ + 1u) % kVelocityHistorySize;
    if (velocity_history_count_ < kVelocityHistorySize) {
      ++velocity_history_count_;
    }
  }

  void updateVelocityEstimateLocked(uint64_t now_us) const {
    if (velocity_history_count_ < 2 || now_us == 0) {
      measured_avg_sps_ = 0.0;
      return;
    }

    const size_t newest =
        (velocity_history_head_ + kVelocityHistorySize - 1u) % kVelocityHistorySize;
    size_t selected = newest;
    for (size_t offset = 1; offset < velocity_history_count_; ++offset) {
      const size_t index =
          (velocity_history_head_ + kVelocityHistorySize - 1u - offset) % kVelocityHistorySize;
      selected = index;
      if (velocity_history_time_us_[newest] >= velocity_history_time_us_[index] +
                                                   static_cast<uint64_t>(kVelocityEstimateWindowS * 1e6)) {
        break;
      }
    }
    if (velocity_history_time_us_[newest] <= velocity_history_time_us_[selected]) {
      measured_avg_sps_ = 0.0;
      return;
    }
    const double dt_s = static_cast<double>(velocity_history_time_us_[newest] -
                                            velocity_history_time_us_[selected]) / 1e6;
    measured_avg_sps_ =
        (velocity_history_steps_[newest] - velocity_history_steps_[selected]) / dt_s;
  }

  Stepper& left_;
  Stepper& right_;
  DualWave pigpio_backend_;
  WaveFrameBackend& backend_;
  mutable std::mutex mu_;

  std::atomic<double> target_left_sps_{0.0};
  std::atomic<double> target_right_sps_{0.0};
  std::atomic<uint64_t> current_time_us_{0};
  const double nominal_control_dt_s_;
  const double max_slew_sps_per_s_;
  double last_command_left_sps_{0.0};
  double last_command_right_sps_{0.0};
  double phase_left_{0.0};
  double phase_right_{0.0};
  double last_left_direction_{1.0};
  double last_right_direction_{1.0};

  std::array<ScheduledFrame, 2> frames_{};
  size_t frame_count_{0};
  uint64_t next_frame_start_us_{0};
  uint64_t last_completed_frame_us_{0};
  bool actuator_fault_{false};

  uint64_t last_call_time_us_{0};
  bool have_last_call_time_{false};
  double last_update_dt_s_{0.0};
  std::atomic<int64_t> actual_steps_left_{0};
  std::atomic<int64_t> actual_steps_right_{0};

  mutable std::array<uint64_t, kVelocityHistorySize> velocity_history_time_us_{};
  mutable std::array<double, kVelocityHistorySize> velocity_history_steps_{};
  mutable size_t velocity_history_head_{0};
  mutable size_t velocity_history_count_{0};
  mutable double measured_avg_sps_{0.0};
  uint32_t actuator_saturation_flags_{ActuatorSaturationNone};
};
