#pragma once

#include "balancer_msgs.h"
#include "publisher.h"

#include <atomic>
#include <chrono>
#include <thread>

namespace sil {

class DOC_DESC("Publishes the global runtime tick used to drive deterministic controller execution. It can run from wall clock for production or be advanced explicitly for simulation.") TimeService {
 public:
  using Publishes = ipc::MsgList<MsgId::PhysicsTick>;
  using Subscribes = ipc::MsgList<>;

  explicit TimeService(ipc::MessageBus& bus, double dt_s = 1.0 / 400.0);
  ~TimeService();

  void start();
  void stop();
  void advance(double dt_s);

 private:
  ipc::TypedPublisher<TimeService> bus_;
  const double default_dt_s_;
  std::atomic<bool> running_{false};
  std::thread worker_;
  uint64_t sim_time_us_{0};

  void publish_tick(double dt_s);
};

}  // namespace sil
