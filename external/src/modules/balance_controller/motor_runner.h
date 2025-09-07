#pragma once
#include "stepper.h"
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <chrono>
#include <cmath>

#include <pigpiod_if2.h>
#include <stdexcept>

#include "config.h"
#include "config_pid.h"

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
  MotorRunner(Stepper& left, Stepper& right,
              double control_hz = 1000.0,
              double max_slew_sps_per_s = 250000.0,  // jerk limiter
              double abs_deadband_sps = 20.0)        // ignore tiny jitters
  : L_(left), R_(right),
    slice_s_(1.0 / control_hz),
    max_delta_per_tick_(max_slew_sps_per_s * slice_s_),
    abs_deadband_(abs_deadband_sps) {
    alive_.store(true, std::memory_order_relaxed);
    worker_ = std::thread(&MotorRunner::loop, this);
    tick_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(slice_s_));
  }

  ~MotorRunner() { stop(); }

  void stop() {
    bool was = true;
    if (alive_.compare_exchange_strong(was,false)) {
      {
        std::lock_guard<std::mutex> lk(mu_);
        new_data_ = true;
      }
      cv_.notify_one();
      if (worker_.joinable()) worker_.join();
      L_.stop(); R_.stop();
    }
  }

  // Set both together (ideal if your controller can call once per tick)
  void setTargets(double left_sps, double right_sps) {
    L_.setTarget(left_sps);
    R_.setTarget(right_sps);
    {
      std::lock_guard<std::mutex> lk(mu_);
      new_data_ = true;
    }
    cv_.notify_one();
  }

  // Convenience per-wheel setters for your existing two-callback shape.
  void setLeft(double sps)  { pending_left_.store(sps,  std::memory_order_relaxed); nudge(); }
  void setRight(double sps) { pending_right_.store(sps, std::memory_order_relaxed); nudge(); }

private:
  void nudge() {
    // Batch both sides by copying both pending values each nudge.
    const double l = pending_left_.load(std::memory_order_relaxed);
    const double r = pending_right_.load(std::memory_order_relaxed);
    setTargets(l, r);
  }

  static double slewToward(double from, double to, double max_delta) {
    const double d = to - from;
    if (d >  max_delta) return from + max_delta;
    if (d < -max_delta) return from - max_delta;
    return to;
  }

  void loop() {
    using clock = std::chrono::steady_clock;
    auto next = clock::now();
    double lastL = 0.0, lastR = 0.0;

    while (alive_.load(std::memory_order_relaxed)) {
      next += tick_;
      { // wait for either new data or the next cadence tick
        std::unique_lock<std::mutex> lk(mu_);
        cv_.wait_until(lk, next, [&]{ return new_data_ || !alive_.load(); });
        new_data_ = false;
      }
      if (!alive_.load()) break;

      double tgtL = L_.target();
      double tgtR = R_.target();

      // Jerk control (slew-limit)
      tgtL = slewToward(lastL, tgtL, max_delta_per_tick_);
      tgtR = slewToward(lastR, tgtR, max_delta_per_tick_);

      // Deadband
      if (std::fabs(tgtL - lastL) < abs_deadband_ &&
          std::fabs(tgtR - lastR) < abs_deadband_) {
        std::this_thread::sleep_until(next);
        continue;
      }

      // If either direction flips, pause both so they restart in phase
      const bool dirL_now = (tgtL >= 0.0), dirL_prev = (lastL >= 0.0);
      const bool dirR_now = (tgtR >= 0.0), dirR_prev = (lastR >= 0.0);
      if ((dirL_now != dirL_prev) || (dirR_now != dirR_prev)) {
        L_.stop(); R_.stop();
      }

      // Apply back-to-back (µs skew)
      L_.setTarget(tgtL);
      R_.setTarget(tgtR);
      L_.applyNow();
      R_.applyNow();

      lastL = tgtL; lastR = tgtR;
      std::this_thread::sleep_until(next);
    }
  }

  Stepper& L_;
  Stepper& R_;
  std::atomic<bool> alive_{false};
  std::thread worker_;
  std::mutex mu_;
  std::condition_variable cv_;
  bool new_data_{false};

  const double slice_s_;
  const double max_delta_per_tick_;
  const double abs_deadband_;
  std::chrono::steady_clock::duration tick_{};

  // For compatibility with your two-lambda wiring
  std::atomic<double> pending_left_{0.0}, pending_right_{0.0};
};
