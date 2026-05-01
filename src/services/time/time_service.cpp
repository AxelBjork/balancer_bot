#include "services/time/time_service.h"

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
    auto next = clock::now();
    const auto period = std::chrono::duration<double>(default_dt_s_);

    while (running_.load(std::memory_order_relaxed)) {
      next += std::chrono::duration_cast<clock::duration>(period);
      publish_tick(default_dt_s_);
      std::this_thread::sleep_until(next);
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
  publish_tick(dt_s);
}

void TimeService::publish_tick(double dt_s) {
  sim_time_us_ += static_cast<uint64_t>(dt_s * 1e6);

  PhysicsTickPayload payload{};
  payload.dt_s = dt_s;
  payload.sim_time_us = sim_time_us_;
  bus_.publish<MsgId::PhysicsTick>(payload);
}

}  // namespace sil
