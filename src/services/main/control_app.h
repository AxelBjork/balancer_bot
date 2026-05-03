#pragma once

#include <pigpiod_if2.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <memory>
#include <thread>

#include "messages/balancer_msgs.h"
#include "services/control/control_service.h"
#include "services/control/rate_controller_core.h"
#include "services/imu/imu_service.h"
#include "services/input/input_service.h"
#include "services/main/config.h"
#include "services/main/telemetry_capture.h"
#include "services/motor/motor_runner.h"
#include "services/motor/motor_service.h"
#include "services/motor/stepper.h"
#include "services/time/time_service.h"
#include "udp_bridge.h"

struct AppRunOptions {
  double run_seconds = Config::run_seconds;
  bool capture_enabled = false;
  std::filesystem::path capture_dir;
  std::string capture_run_id;
  std::string pid_profile = "pid.conf";
  std::string binary_path;
};

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
  sil::InputService ins;
  ipc::UdpBridge udp;
  sil::TelemetryCapture* capture = nullptr;
  AppServices(ipc::MessageBus& bus, MotorRunner* runner)
      : ms(bus, runner),
        cs(bus),
        is(bus, true),
        ts(bus, 1.0 / Config::control_hz),
        ins(bus),
        udp(bus) {
  }
};

struct BusContainer {
  ipc::MessageBus bus;
  AppServices services;
  BusContainer(MotorRunner* runner, ipc::MessageBus::DispatchFn disp)
      : bus(&services, disp), services(bus, runner) {
  }
};

inline void app_dispatcher(void* ctx, MsgId id, const void* payload) {
  auto* s = static_cast<AppServices*>(ctx);
  static int telemetry_count = 0;

  ipc::dispatch_to_services(id, payload, s->is, s->ms, s->cs, s->udp, s->ts);

  if (id == MsgId::SystemTelemetry) {
    const ipc::SystemTelemetryPayload& p = unpack_payload<MsgId::SystemTelemetry>(payload);
    if (s->capture != nullptr) {
      s->capture->record(p);
    }
    if constexpr (Config::kPrintEvery != -1) {
      if ((++telemetry_count % Config::kPrintEvery) == 0) {
        const bool motor_dt_warning = p.motor_update_dt_ms > (1500.0 / Config::control_hz);
        const double applied_avg_sps = 0.5 * (p.left_applied_sps + p.right_applied_sps);
        std::printf(
            "t=%7.3f  th=%6.2f deg  dth=%7.2f dps  u=%6.0f%s  "
            "sp=%6.2f (%+5.2f/%+5.2f)  perr=%6.2f  v=%7.1f/%7.1f  "
            "ap=%6.0f%s  trn=%6.0f\n",
            p.t_sec, p.pitch_deg, p.pitch_rate_dps, p.u_sps,
            (std::abs(p.u_sps) >= 0.99 * kMaxSps) ? "*" : "", p.pitch_sp_deg,
            p.pitch_ref_from_vel_deg, p.pitch_trim_deg, p.pitch_error_deg, p.vel_error,
            p.measured_vel_sps, applied_avg_sps, motor_dt_warning ? "  MOTOR_DT!" : "", p.turn_sps);
      }
    }
  }
}


// ---------------------- Motor control runner --------------------------------
class ControlApp {
 public:
  int run(PigpioCtx& _ctx, const AppRunOptions& options = {}) {
    // Hardware setup
    Stepper::Pins leftPins{12, 19, 13};  // ENA, STEP(PWM1), DIR
    Stepper::Pins rightPins{4, 18, 24};  // ENB, STEP(PWM0), DIR

    Stepper left(_ctx.handle(), leftPins, Config::invert_left, /*energize_now=*/true);
    Stepper right(_ctx.handle(), rightPins, Config::invert_right, /*energize_now=*/true);

    // Coordinator at 1 kHz
    MotorRunner motors(left, right, Config::control_hz, 250000.0);

    // Start MessageBus and Services
    BusContainer app_bus(&motors, app_dispatcher);
    std::unique_ptr<sil::TelemetryCapture> capture;
    if (options.capture_enabled) {
      sil::TelemetryCaptureOptions capture_options;
      capture_options.output_dir = options.capture_dir;
      capture_options.run_id = options.capture_run_id;
      if (capture_options.run_id.empty()) {
        capture_options.run_id = options.capture_dir.filename().empty()
                                     ? std::string("capture")
                                     : options.capture_dir.filename().string();
      }
      capture_options.mode = "real_app";
      capture_options.pid_profile = options.pid_profile;
      capture_options.run_seconds = options.run_seconds;
      capture_options.binary_path = options.binary_path;
      capture_options.working_directory = std::filesystem::current_path().string();
      capture = std::make_unique<sil::TelemetryCapture>(std::move(capture_options));
      app_bus.services.capture = capture.get();
    }
    app_bus.services.cs.start();
    app_bus.services.ms.start();
    app_bus.services.is.start();
    app_bus.services.ts.start();
    app_bus.services.ins.start();

    // Start UDP Bridge if not in testing, or depending on config. For now, try to start it on port
    // 9000
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
                                               std::chrono::duration<double>(options.run_seconds));
    const auto tick = std::chrono::duration<double, std::milli>(1000.0 / Config::command_hz);

    while (std::chrono::steady_clock::now() < t_end && !g_stop.load(std::memory_order_relaxed)) {
      std::this_thread::sleep_for(tick);
    }

    app_bus.services.ins.stop();
    app_bus.services.ts.stop();
    if (capture != nullptr) {
      const bool stopped_by_signal = g_stop.load(std::memory_order_relaxed);
      capture->finish(stopped_by_signal ? "stopped_by_signal" : "completed",
                      stopped_by_signal ? 130 : 0);
    }
    g_stop.store(true, std::memory_order_relaxed);
    return 0;
  }
};
