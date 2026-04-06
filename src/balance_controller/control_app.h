#pragma once

#include <pigpiod_if2.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <thread>

#include "config.h"
#include "services/control/rate_controller_core.h"
#include "motor_runner.h"
#include "stepper.h"
#include "xbox_controller.h"

#include "message_bus.h"
#include "messages/balancer_msgs.h"
#include "services/control_service.h"
#include "services/motor_service.h"
#include "services/imu_service.h"
#include "services/time_service.h"
#include "udp_bridge.h"

struct PigpioCtx {
  explicit PigpioCtx(const char* host = nullptr, const char* port = nullptr) {
    pi = pigpio_start(const_cast<char*>(host), const_cast<char*>(port));
    if (pi < 0) throw std::runtime_error("pigpio_start failed");
  }
  ~PigpioCtx() {
    pigpio_stop(pi);
  }
  int handle() const {
    return pi;
  }

 private:
  int pi{};
};

struct AppServices {
  sil::MotorService ms;
  sil::ControlService cs;
  sil::ImuService is;
  sil::TimeService ts;
  ipc::UdpBridge udp;
  AppServices(ipc::MessageBus& bus, MotorRunner* runner)
      : ms(bus, runner), cs(bus), is(bus, true), ts(bus, 1.0 / Config::control_hz), udp(bus) {}
};

struct BusContainer {
  ipc::MessageBus bus;
  AppServices services;
  BusContainer(MotorRunner* runner, ipc::MessageBus::DispatchFn disp)
      : bus(&services, disp), services(bus, runner) {}
};

void app_dispatcher(void* ctx, MsgId id, const void* payload) {
  auto* s = static_cast<AppServices*>(ctx);
  static int telemetry_count = 0;
  if (id == MsgId::ImuData) {
    s->cs.on_message<MsgId::ImuData>(*static_cast<const ipc::ImuSamplePayload*>(payload));
  } else if (id == MsgId::PhysicsTick) {
    s->cs.on_message<MsgId::PhysicsTick>(*static_cast<const PhysicsTickPayload*>(payload));
  } else if (id == MsgId::JoystickCommand) {
    s->cs.on_message<MsgId::JoystickCommand>(*static_cast<const ipc::JoystickCommandPayload*>(payload));
  } else if (id == MsgId::MotorFeedback) {
    s->cs.on_message<MsgId::MotorFeedback>(*static_cast<const ipc::MotorFeedbackPayload*>(payload));
  } else if (id == MsgId::MotorTargets) {
    const auto& p = *static_cast<const ipc::MotorTargetsPayload*>(payload);
    s->ms.on_message<MsgId::MotorTargets>(p);
    s->udp.on_message<MsgId::MotorTargets>(p); // relay to Python
  } else if (id == MsgId::SystemTelemetry) {
    const auto& p = *static_cast<const ipc::SystemTelemetryPayload*>(payload);
    if constexpr (Config::kPrintEvery != -1) {
      if ((++telemetry_count % Config::kPrintEvery) == 0) {
        std::printf(
            "t=%7.3f  theta=%6.2f deg  theta_dot=%6.2f dps  r_sp=%6.2f dps  out=%6.3f  "
            "u=%6.0f%s  I=%7.3f  psp=%5.2f  ve=%5.1f  vi=%5.3f  vp=%5.3f\n",
            p.t_sec, p.pitch_deg, p.pitch_rate_dps, p.rate_sp_dps, p.out_norm, p.u_sps,
            (std::abs(p.u_sps) >= 0.99f * static_cast<float>(kMaxSps)) ? "*" : "",
            p.integ_pitch, p.pitch_sp_deg, p.vel_error, p.vel_i_term, p.vel_p_term);
      }
    }
    s->udp.on_message<MsgId::SystemTelemetry>(p); // relay to Python
  }
}

template <class MotorRunnerT>
class CascadedController {
 public:
  CascadedController(MotorRunnerT& motors) : motors_(motors) {
    core_.setMotorOutputs(
        [this](float left_sps, float right_sps) { motors_.setTargets(left_sps, right_sps); });

    // Velocity feedback from motor commanded targets (actual step tracking)
    core_.setVelocityFeedback([this]() -> float { return motors_.getActualSpeedSps(); });
    core_.setPositionFeedback([this]() -> float {
      const float average_steps =
          0.5f * static_cast<float>(motors_.getActualLeftSteps() + motors_.getActualRightSteps());
      return average_steps * static_cast<float>(Config::meters_per_step);
    });

  }

  ~CascadedController() = default;

  void pushImu(const ImuSample& s) {
    core_.pushImu(s);
  }
  void setJoystick(const JoyCmd& j) {
    core_.setJoystick(j);
  }
  void setTelemetrySink(std::function<void(const Telemetry&)> cb) {
    core_.setTelemetrySink(std::move(cb));
  }
  void step(double dt_s, std::chrono::steady_clock::time_point now) {
    core_.step(dt_s, now);
  }

 private:
  MotorRunnerT& motors_;
  RateControllerCore core_;
};

// ---------------------- Motor control runner --------------------------------
class ControlApp {
 public:
  int run(PigpioCtx& _ctx, bool xbox_control = true) {
    if (xbox_control) {
      pad = std::make_unique<XboxController>();
    }
    // Hardware setup
    Stepper::Pins leftPins{12, 19, 13};  // ENA, STEP(PWM1), DIR
    Stepper::Pins rightPins{4, 18, 24};  // ENB, STEP(PWM0), DIR

    Stepper left(_ctx.handle(), leftPins, Config::invert_left, /*energize_now=*/true);
    Stepper right(_ctx.handle(), rightPins, Config::invert_right, /*energize_now=*/true);

    // Coordinator at 1 kHz
    MotorRunner motors(left, right, Config::control_hz, 250000.0);

    // Start MessageBus and Services
    BusContainer app_bus(&motors, app_dispatcher);
    app_bus.services.cs.start();
    app_bus.services.ms.start();
    app_bus.services.is.start();
    app_bus.services.ts.start();

    // Start UDP Bridge if not in testing, or depending on config. For now, try to start it on port 9000
    try {
        app_bus.services.udp.start();
        std::cout << "UDP Bridge started on default port 9000\n";
    } catch (const std::exception& e) {
        std::cerr << "Failed to start UDP Bridge (expected if port in use): " << e.what() << "\n";
    }

    // Note: ImuService internally configures its IioReader, which runs its own thread.
    // Telemetry is now published to the bus as SystemTelemetry, so we can ignore it here 
    // or add a dummy subscriber if we want printouts. For now, printouts disabled as requested.

    // Main app loop: read gamepad and feed controller setpoints
    // Run for Config::run_seconds of SIMULATION time (not wall-clock time)
    // speedup affects wall-clock execution speed, not simulation duration
    const auto t_end =
        std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::nanoseconds>(
                                               std::chrono::duration<double>(Config::run_seconds));
    const auto tick = std::chrono::duration<double, std::milli>(1000.0 / Config::command_hz);

    while (std::chrono::steady_clock::now() < t_end && !g_stop.load(std::memory_order_relaxed)) {
      float ly = 0.0;
      float ry = 0;

      if (xbox_control) {
        pad->update();
        // Arcade Drive: Left Stick Y = Forward/Swap, Right Stick X = Turn
        // Y-axis is often -1 (Up) to +1 (Down), so invert for Forward.
        ly = -pad->leftY();
        ry = pad->rightX();
      }
      ipc::JoystickCommandPayload j;
      j.forward = ly;
      j.turn = ry;
      app_bus.bus.publish<MsgId::JoystickCommand>(j);

      std::this_thread::sleep_for(tick);
    }

    app_bus.services.ts.stop();
    g_stop.store(true, std::memory_order_relaxed);
    return 0;
  }
  std::unique_ptr<XboxController> pad;
};
