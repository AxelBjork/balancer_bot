#include <csignal>

#include "control_app.h"
#include "types.h"

int main() {
  ConfigPid::load("pid.conf");
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  PigpioCtx _ctx;
  ControlApp app;
  return app.run(_ctx, true);
}