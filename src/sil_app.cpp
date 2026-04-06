#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>

#include "message_bus.h"
#include "messages/balancer_msgs.h"
#include "udp_bridge.h"
#include "types.h"
#include "services/control_service.h"
#include "services/motor_service.h"
#include "services/imu_service.h"

// Reuse the AppServices logic from control_app.h if possible, 
// but for SIL we might want a cleaner separation. 
// For now, let's define a similar structure here.

struct AppServices {
    sil::MotorService ms;
    sil::ControlService cs;
    sil::ImuService is;
    ipc::UdpBridge udp;

    AppServices(ipc::MessageBus& bus) 
        : ms(bus, nullptr), 
          cs(bus), 
          is(bus, false), 
          udp(bus) 
    {}
};

void sil_dispatcher(void* ctx, MsgId id, const void* payload) {
    auto* s = static_cast<AppServices*>(ctx);
    if (!s) return;

    if (id == MsgId::ImuData) {
        s->cs.on_message<MsgId::ImuData>(*static_cast<const ipc::ImuSamplePayload*>(payload));
        s->udp.on_message<MsgId::ImuData>(*static_cast<const ipc::ImuSamplePayload*>(payload));
    } else if (id == MsgId::PhysicsTick) {
        s->cs.on_message<MsgId::PhysicsTick>(*static_cast<const PhysicsTickPayload*>(payload));
    } else if (id == MsgId::JoystickCommand) {
        s->cs.on_message<MsgId::JoystickCommand>(*static_cast<const ipc::JoystickCommandPayload*>(payload));
    } else if (id == MsgId::MotorFeedback) {
        s->cs.on_message<MsgId::MotorFeedback>(*static_cast<const ipc::MotorFeedbackPayload*>(payload));
    } else if (id == MsgId::MotorTargets) {
        const auto& p = *static_cast<const ipc::MotorTargetsPayload*>(payload);
        s->ms.on_message<MsgId::MotorTargets>(p);
        s->udp.on_message<MsgId::MotorTargets>(p);
    } else if (id == MsgId::SystemTelemetry) {
        const auto& p = *static_cast<const ipc::SystemTelemetryPayload*>(payload);
        s->udp.on_message<MsgId::SystemTelemetry>(p);
    }
}

struct BusContainer {
    ipc::MessageBus bus;
    AppServices services;

    BusContainer() : bus(&services, sil_dispatcher), services(bus) {}
};

std::atomic<bool> sil_g_stop{false};
void signal_handler(int) { sil_g_stop = true; }

int main() {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "Starting sil_app (SIL Mode)..." << std::endl;
    ConfigPid::load(ConfigPid::resolve_path("pid_sim.conf"));

    BusContainer container;

    // Start services
    container.services.is.start();
    container.services.cs.start();
    container.services.ms.start();
    
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

    return 0;
}
