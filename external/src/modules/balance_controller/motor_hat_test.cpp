// drive_by_xbox.cpp
#include "stepper.h"
#include "xbox_controller.h"

#include <pigpiod_if2.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <stdexcept>
#include <thread>

// ---------------------- Compile-time configuration ---------------------------
struct Config {
  static inline constexpr int   run_seconds   = 30;
  static inline constexpr int   control_hz    = 100;
  static inline constexpr int   max_sps       = 1200;
  static inline constexpr float deadzone      = 0.05f;
  static inline constexpr bool  invert_left   = false;
  static inline constexpr bool  invert_right  = false;
};
// ---------------------------------------------------------------------------

namespace {
using clock = std::chrono::steady_clock;

static std::atomic<bool> g_stop{false};
static void on_signal(int) { g_stop.store(true, std::memory_order_relaxed); }

struct PigpioCtx {
  PigpioCtx(const char* host=nullptr, const char* port=nullptr) {
    pi = pigpio_start(host, port);
    if (pi < 0) throw std::runtime_error("pigpio_start failed");
  }
  ~PigpioCtx() { pigpio_stop(pi); }
  int handle() const { return pi; }
private: int pi{};
};

// ---------------------- Motor control runner --------------------------------

class MotorRunner {
public:
  explicit MotorRunner(Stepper& m)
    : motor_(m),
      alive_(true),
      slice_sec_(1.0 / static_cast<double>(Config::control_hz)),
      worker_(&MotorRunner::loop, this) {}

  ~MotorRunner() { stop(); }

  void setTarget(double sps) noexcept { target_sps_.store(sps, std::memory_order_relaxed); }

  void stop() {
    bool was = true;
    if (alive_.compare_exchange_strong(was, false) && worker_.joinable()) worker_.join();
  }

private:
  void loop() {
    double residual = 0.0;

    while (alive_.load(std::memory_order_relaxed) && !g_stop.load(std::memory_order_relaxed)) {
      const double t   = target_sps_.load(std::memory_order_relaxed);
      const bool   fwd = (t >= 0.0);
      const double sps = std::fabs(t);

      if (sps < 1e-3) {
        std::this_thread::sleep_for(std::chrono::duration<double>(slice_sec_));
        continue;
      }

      const double want  = sps * slice_sec_ + residual;
      const unsigned n   = (want > 0.0) ? static_cast<unsigned>(want) : 0u;
      residual           = want - static_cast<double>(n);

      if (n == 0u) {
        std::this_thread::sleep_for(std::chrono::duration<double>(slice_sec_));
        continue;
      }

      const unsigned period_us =
          static_cast<unsigned>(std::lround(1e6 / std::max(1.0, sps)));

      motor_.stepN(n, period_us, fwd);
    }
  }

  Stepper&            motor_;
  std::atomic<bool>   alive_;
  std::atomic<double> target_sps_{0.0};
  const double        slice_sec_;
  std::thread         worker_;
};

class App {
public:
  int run(PigpioCtx& _ctx) {
    XboxController pad;

    Stepper::Pins leftPins  {12, 19, 13}; // ENA, STEP, DIR
    Stepper::Pins rightPins { 4, 18, 24}; // ENB, STEP, DIR
    Stepper left(_ctx.handle(), leftPins), right(_ctx.handle(), rightPins);
    MotorRunner L(left), R(right);

    const auto t_end = clock::now() + std::chrono::seconds(Config::run_seconds);
    const auto tick  = std::chrono::milliseconds(1000 / Config::control_hz);

    while (clock::now() < t_end && !g_stop.load(std::memory_order_relaxed)) {
      pad.update();

      float ly = pad.leftY();
      float ry = pad.rightY();
      if (Config::invert_left)  ly = -ly;
      if (Config::invert_right) ry = -ry;

      L.setTarget(static_cast<double>(ly) * Config::max_sps);
      R.setTarget(static_cast<double>(ry) * Config::max_sps);

      std::this_thread::sleep_for(tick);
    }

    L.stop();
    R.stop();
    return 0;
  }
};
} // namespace


int main() {
  std::signal(SIGINT,  on_signal);
  std::signal(SIGTERM, on_signal);

  PigpioCtx _ctx;
  App app;
  
  app.run(_ctx);
  return 0;
}
