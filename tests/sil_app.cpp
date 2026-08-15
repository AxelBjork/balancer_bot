#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <string>
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

  AppServices(ipc::MessageBus& bus, uint16_t udp_port)
      : ms(bus, nullptr), cs(bus), is(bus, false), ins(bus), udp(bus, udp_port) {
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

  explicit BusContainer(uint16_t udp_port)
      : bus(&services, sil_dispatcher), services(bus, udp_port) {
  }
};

std::atomic<bool> sil_g_stop{false};
void signal_handler(int) {
  sil_g_stop = true;
}

int main(int argc, char** argv) {
  uint16_t udp_port = ipc::UdpBridge::kDefaultPort;
  for (int index = 1; index < argc; ++index) {
    if (std::string(argv[index]) != "--port" || index + 1 >= argc) continue;
    const unsigned long parsed = std::stoul(argv[++index]);
    if (parsed > 65535UL) throw std::invalid_argument("SIL UDP port is out of range");
    udp_port = static_cast<uint16_t>(parsed);
  }

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  std::cout << "Starting sil_app (SIL Mode)..." << std::endl;
  ConfigPid::load(ConfigPid::resolve_path("pid.conf"));

  BusContainer container(udp_port);

  // Start services
  container.services.is.start();
  container.services.cs.start();
  container.services.ms.start();
  container.services.ins.start();

  try {
    container.services.udp.start();
    std::cout << "UDP Bridge listening on port " << udp_port << std::endl;
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
