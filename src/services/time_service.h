#pragma once

#include <atomic>
#include <chrono>
#include <thread>

#include "messages/balancer_msgs.h"
#include "publisher.h"

namespace sil {

inline constexpr char kTimeServiceDoc[] =
    "Publishes the global `PhysicsTick` heartbeat that drives deterministic controller execution "
    "and advances the shared simulation clock.\n\n"
    "The service supports two operating modes. In runtime mode it owns a worker thread that sleeps "
    "against `std::chrono::steady_clock` and emits ticks at the configured default cadence. In SIL "
    "or test mode it can instead be advanced explicitly by callers, which lets the rest of the "
    "system run from a fully deterministic external timeline instead of wall clock time. The "
    "default timestep is currently `1 / 400 s`, so the nominal scheduler frequency is about "
    "`400 Hz`.\n\n"
    "Each tick increments the monotonically increasing simulation timestamp and publishes\n\n"
    "$$ t_{sim,us} \\leftarrow t_{sim,us} + \\Delta t \\cdot 10^6 $$\n\n"
    "with the exact `dt_s` used for that step embedded in the payload. `ControlService` consumes "
    "these ticks as the authoritative integration step, so keeping this service as the sole owner "
    "of tick publication prevents divergent notions of time across hardware, SIL replay, and unit "
    "tests.";

class DOC_DESC(kTimeServiceDoc) TimeService {
 public:
  static constexpr const char* kDocDescription = kTimeServiceDoc;

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
