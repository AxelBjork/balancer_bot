#pragma once

#include <atomic>
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
    "Deterministic ticks increment a zero-based timestamp. Runtime ticks publish an absolute "
    "`std::chrono::steady_clock` timestamp and the elapsed wall-clock `dt_s`. `ControlService` consumes "
    "these ticks as the authoritative integration step, so keeping this service as the sole owner "
    "of tick publication prevents divergent notions of time across hardware, SIL replay, and unit "
    "tests.";

class DOC_DESC(kTimeServiceDoc) TimeService {
 public:
  static constexpr const char* kDocDescription = kTimeServiceDoc;

  using Publishes = ipc::MsgList<MsgId::PhysicsTick>;

  explicit TimeService(ipc::MessageBus& bus, double dt_s = 1.0 / 400.0);
  ~TimeService();

  void start();
  void stop();
  void advance(double dt_s);
  uint64_t current_time_us() const;

 private:
  ipc::TypedPublisher<TimeService> bus_;
  const double default_dt_s_;
  std::atomic<bool> running_{false};
  std::thread worker_;
  std::atomic<uint64_t> current_time_us_{0};

  void publish_tick(double dt_s, uint64_t timestamp_us);
};

}  // namespace sil
