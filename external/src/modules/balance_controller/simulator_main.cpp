#include <csignal>
#include <cstdlib>
#include <iostream>
#include <thread>

#include "control_app.h"
#include "simulator/balancer_simulator.h"

int main() {
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  // Set IIO_ROOT to point to tmp dir
  setenv("IIO_ROOT", "/tmp/balancer_sim_root", 1);

  std::cout << "Starting Balancer Simulator..." << std::endl;

  // Initialize Simulator
  BalancerSimulator::Config sim_cfg;
  // COM angular offset (positive = COM forward)
  sim_cfg.com_angle_offset_rad = 0.005; 
  // Initial pitch angle (positive = forward)
  sim_cfg.initial_pitch_deg = 5.0;
  
  BalancerSimulator sim(sim_cfg);
  sim.start();

  // Run the Control App (using stubs)
  // PigpioCtx will use pigpiod_stub (linked) which does nothing but is required.
  // Wait, if we use pigpiod_stub, we don't need real pigpiod running.
  // But PigpioCtx calls pigpio_start(). 
  // In stub, pigpio_start returns 1 (success-ish, usually >0 is handle).
  
  try {
      PigpioCtx _ctx;
      ControlApp app;
      
      // Run app without xbox (simulator auto-drive or just balance?)
      // We can enable xbox control if we want to test inputs too, 
      // but usually for sim we might want a script or just balance.
      // App::run defaults loop.
      app.run(_ctx, false);
  } catch (const std::exception& e) {
      std::cerr << "Error: " << e.what() << std::endl;
  }

  std::cout << "Stopping Simulator..." << std::endl;
  sim.stop();
  return 0;
}
