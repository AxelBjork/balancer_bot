#pragma once

#include "config.h"
#include "stepper.h"
#include "xbox_controller.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <pigpiod_if2.h>
#include <stdexcept>
#include <thread>

struct PigpioCtx {
  PigpioCtx(const char *host = nullptr, const char *port = nullptr) {
    pi = pigpio_start(host, port);
    if (pi < 0)
      throw std::runtime_error("pigpio_start failed");
  }
  ~PigpioCtx() { pigpio_stop(pi); }
  int handle() const { return pi; }

private:
  int pi{};
};

class MotorRunner {
public:
  explicit MotorRunner(Stepper &m, bool invert = false)
      : motor_(m), alive_(true),
        slice_sec_(1.0 / static_cast<double>(Config::control_hz)),
        invert_dir(invert), worker_(&MotorRunner::loop, this) {}

  ~MotorRunner() { stop(); }

  void setTarget(double sps) noexcept {
    target_sps_.store(sps, std::memory_order_relaxed);
  }

  double actualSps() const noexcept {
    return actual_sps_.load(std::memory_order_relaxed) * (invert_dir ? -1.0 : 1.0);
  }

  void stop() {
    bool was = true;
    if (alive_.compare_exchange_strong(was, false) && worker_.joinable())
      worker_.join();
  }

private:

  void loop() {
    double residual = 0.0;
    // EMA for measured sps (fast but smooth)
    double ema_sps = 0.0;
    const double tau_s = 0.02; // 20 ms
    const double a = std::exp(-slice_sec_ / tau_s);

    while (alive_.load(std::memory_order_relaxed) &&
           !g_stop.load(std::memory_order_relaxed)) {
      const double t = target_sps_.load(std::memory_order_relaxed);
      bool fwd = (t >= 0.0);
      if (invert_dir) fwd = !fwd;

      const double sps = std::fabs(t);

      // --- fixed slice budgeting (keeps both motors in lockstep) ---
      const double want = sps * slice_sec_ + residual;
      const unsigned n = (want > 0.0) ? static_cast<unsigned>(want) : 0u;
      residual = want - static_cast<double>(n);

      if (n == 0u) {
        // update measured sps toward zero smoothly
        ema_sps = a * ema_sps; // pulls toward 0 when idle
        actual_sps_.store(ema_sps, std::memory_order_relaxed);
        std::this_thread::sleep_for(std::chrono::duration<double>(slice_sec_));
        continue;
      }

      const unsigned period_us =
          static_cast<unsigned>(std::lround(1e6 / std::max(1.0, sps)));
      motor_.stepN(n, period_us, fwd);

      // measured sps this slice = executed steps / slice time
      const double signed_steps = fwd ? static_cast<double>(n) : -static_cast<double>(n);
      const double inst_sps = signed_steps / slice_sec_;

      // EMA update
      ema_sps = a * ema_sps + (1.0 - a) * inst_sps;
      // include invert_dir in the sign we publish
      const double signed_ema = invert_dir ? -ema_sps : ema_sps;
      actual_sps_.store(signed_ema, std::memory_order_relaxed);

      std::this_thread::sleep_for(std::chrono::duration<double>(slice_sec_));
    }
  }

  Stepper &motor_;
  std::atomic<bool> alive_;
  std::atomic<double> target_sps_{0.0};
  std::atomic<double> actual_sps_{0.0};
  const double slice_sec_;
  bool invert_dir;
  std::thread worker_;
};