#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

#include "services/main/control_app.h"
#include "messages/types.h"

namespace {

std::string getenv_or_empty(const char* name) {
  if (const char* value = std::getenv(name)) {
    return value;
  }
  return {};
}

void print_usage() {
  std::cout
      << "Usage: balancer_pi [--pid-config <path>] [--run-seconds <seconds>]\n"
      << "                  [--capture-dir <path>] [--capture-run-id <id>]\n";
}

}  // namespace

int main(int argc, char** argv) {
  AppRunOptions options;
  std::string pid_config_path = getenv_or_empty("BALANCER_PID_CONF");
  if (pid_config_path.empty()) {
    pid_config_path = "pid.conf";
  }
  options.run_seconds = Config::run_seconds;
  options.capture_dir = getenv_or_empty("BALANCER_CAPTURE_DIR");
  options.capture_run_id = getenv_or_empty("BALANCER_CAPTURE_RUN_ID");
  if (const std::string env_run_seconds = getenv_or_empty("BALANCER_RUN_SECONDS");
      !env_run_seconds.empty()) {
    options.run_seconds = std::stod(env_run_seconds);
  }
  options.binary_path = (argc > 0 && argv[0] != nullptr) ? argv[0] : "balancer_pi";

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      print_usage();
      return 0;
    }
    if (arg == "--pid-config" && (i + 1) < argc) {
      pid_config_path = argv[++i];
      continue;
    }
    if (arg == "--run-seconds" && (i + 1) < argc) {
      options.run_seconds = std::stod(argv[++i]);
      continue;
    }
    if (arg == "--capture-dir" && (i + 1) < argc) {
      options.capture_dir = argv[++i];
      options.capture_enabled = true;
      continue;
    }
    if (arg == "--capture-run-id" && (i + 1) < argc) {
      options.capture_run_id = argv[++i];
      continue;
    }
    std::cerr << "Unknown or incomplete argument: " << arg << '\n';
    print_usage();
    return 1;
  }

  if (!options.capture_dir.empty()) {
    options.capture_enabled = true;
  }

  ConfigPid::load(pid_config_path);
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  PigpioCtx _ctx;
  ControlApp app;
  return app.run(_ctx, options);
}
