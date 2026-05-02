#include <csignal>

#include "services/main/control_app.h"
#include "messages/types.h"

int main() {
  ConfigPid::load("pid.conf");
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  PigpioCtx _ctx;
  ControlApp app;
  return app.run(_ctx);
}
