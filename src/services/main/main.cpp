#include <csignal>

#include "services/main/control_app.h"
#include "messages/types.h"

// The dashboard cannot execute this cross-built ELF on the host. Keep the PID
// schema version discoverable so it can reject incompatible pid.conf files
// without tying deployment to every gain edit.
extern "C" [[gnu::used, gnu::visibility("default")]]
const char* balancer_pid_config_version_marker() {
  return ConfigPid::config_version_marker;
}

int main() {
  ConfigPid::load("pid.conf");
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  PigpioCtx _ctx;
  ControlApp app;
  return app.run(_ctx);
}
