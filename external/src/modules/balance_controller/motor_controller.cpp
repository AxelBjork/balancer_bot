// drive_by_xbox.cpp
#include "stepper.h"
#include "xbox_controller.h"
#include "config.h"

#include <pigpiod_if2.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <thread>


namespace {
using clock = std::chrono::steady_clock;



// ---------------------- Motor control runner --------------------------------

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
