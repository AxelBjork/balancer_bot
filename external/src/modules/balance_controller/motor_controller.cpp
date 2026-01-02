#include <csignal>
#include "control_app.h"

// Note: g_stop and on_signal are in config.h (via control_app.h)

int main() {
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  PigpioCtx _ctx; 
  ControlApp app;
  return app.run(_ctx, false);
}