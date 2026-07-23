#include "services/time/time_service.h"

#include <algorithm>
#include <cmath>
#include <thread>

namespace sil {

TimeService::TimeService(ipc::MessageBus& bus, double dt_s) : bus_(bus), default_dt_s_(dt_s) {
}

TimeService::~TimeService() {
  stop();
}

void TimeService::start() {
  if (running_.exchange(true, std::memory_order_acq_rel)) {
    return;
  }

  worker_ = std::thread([this]() {
    using clock = std::chrono::steady_clock;
    auto previous = clock::now();
    auto next = previous;
    const auto period = std::chrono::duration<double>(default_dt_s_);

    while (running_.load(std::memory_order_relaxed)) {
      next += std::chrono::duration_cast<clock::duration>(period);
      std::this_thread::sleep_until(next);
      if (!running_.load(std::memory_order_relaxed)) {
        break;
      }

      const auto now = clock::now();
      const double measured_dt_s = std::chrono::duration<double>(now - previous).count();
      previous = now;
      const auto wall_timestamp_us = static_cast<uint64_t>(
          std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count());
      const auto timestamp_us = std::max(
          wall_timestamp_us, current_time_us_.load(std::memory_order_relaxed) + uint64_t{1});
      publish_tick(measured_dt_s, timestamp_us);
    }
  });
}

void TimeService::stop() {
  if (!running_.exchange(false, std::memory_order_acq_rel)) {
    return;
  }

  if (worker_.joinable()) {
    worker_.join();
  }
}

void TimeService::advance(double dt_s) {
  if (!std::isfinite(dt_s) || dt_s <= 0.0) {
    return;
  }
  const uint64_t delta_us = static_cast<uint64_t>(std::llround(dt_s * 1e6));
  const uint64_t timestamp_us = current_time_us_.fetch_add(delta_us, std::memory_order_relaxed) + delta_us;
  publish_tick(dt_s, timestamp_us);
}

uint64_t TimeService::current_time_us() const {
  return current_time_us_.load(std::memory_order_relaxed);
}

void TimeService::publish_tick(double dt_s, uint64_t timestamp_us) {
  current_time_us_.store(timestamp_us, std::memory_order_relaxed);
  PhysicsTickPayload payload{};
  payload.dt_s = dt_s;
  payload.timestamp_us = timestamp_us;
  bus_.publish<MsgId::PhysicsTick>(payload);
}

}  // namespace sil
