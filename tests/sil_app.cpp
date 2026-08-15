#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

#include "messages/balancer_msgs.h"
#include "messages/types.h"
#include "services/control/control_service.h"
#include "services/imu/imu_service.h"
#include "services/motor/motor_service.h"
#include "services/input/input_service.h"
#include "udp_bridge.h"

// Reuse the AppServices logic from control_app.h if possible,
// but for SIL we might want a cleaner separation.
// For now, let's define a similar structure here.

struct AppServices {
  sil::MotorService ms;
  sil::ControlService cs;
  sil::ImuService is;
  sil::InputService ins;
  ipc::UdpBridge udp;

  AppServices(ipc::MessageBus& bus)
      : ms(bus, nullptr), cs(bus), is(bus, false), ins(bus), udp(bus) {
  }
};

void sil_dispatcher(void* ctx, MsgId id, const void* payload) {
  auto* s = static_cast<AppServices*>(ctx);
  if (!s) return;

  ipc::dispatch_to_services(id, payload, s->is, s->ms, s->cs, s->ins, s->udp);
}

struct BusContainer {
  ipc::MessageBus bus;
  AppServices services;

  BusContainer() : bus(&services, sil_dispatcher), services(bus) {
  }
};

std::atomic<bool> sil_g_stop{false};
void signal_handler(int) {
  sil_g_stop = true;
}

int main() {
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  std::cout << "Starting sil_app (SIL Mode)..." << std::endl;
  ConfigPid::load(ConfigPid::resolve_path("pid.conf"));

  BusContainer container;

  // Start services
  container.services.is.start();
  container.services.cs.start();
  container.services.ms.start();
  container.services.ins.start();

  try {
    container.services.udp.start();
    std::cout << "UDP Bridge listening on port " << ipc::UdpBridge::kDefaultPort << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Failed to start UDP bridge: " << e.what() << std::endl;
    return 1;
  }

  std::cout << "SIL App running. Press Ctrl+C to stop." << std::endl;

  while (!sil_g_stop) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  std::cout << "Shutting down..." << std::endl;
  container.services.is.stop();
  container.services.cs.stop();
  container.services.ms.stop();
  container.services.ins.stop();

  return 0;
}
