#include <iostream>

#include "config.h"
#include "services/control/rate_controller_core.h"
#include "simulator/balancer_simulator.h"
#include "types.h"

int main() {
  ConfigPid::load("pid.conf");

  BalancerSimulator::Config sim_cfg;
  sim_cfg.com_angle_offset_rad = 0.0;
  sim_cfg.initial_pitch_deg = 0.1;

  BalancerSimulator sim(sim_cfg);
  RateControllerCore core;

  float left_sps = 0.0f;
  float right_sps = 0.0f;
  core.setMotorOutputs([&](float left, float right) {
    left_sps = left;
    right_sps = right;
    sim.set_motor_targets(left, right);
  });
  core.setVelocityFeedback([&]() { return sim.get_actual_speed_sps(); });
  core.setJoystick(JoyCmd{0.0f, 0.0f});

  constexpr double dt_s = 1.0 / 400.0;
  uint64_t sim_time_us = 0;
  for (int i = 0; i < static_cast<int>(Config::run_seconds * Config::control_hz); ++i) {
    sim.step(dt_s);
    sim_time_us += static_cast<uint64_t>(dt_s * 1e6);
    const auto imu = sim.make_imu_payload(sim_time_us);

    ImuSample sample{};
    sample.angle_rad = imu.pitch_rad;
    sample.gyro_rad_s = imu.gyr[1];
    sample.yaw_rate_z = imu.gyr[2];
    sample.t = std::chrono::steady_clock::time_point(std::chrono::microseconds(imu.timestamp_us));

    core.pushImu(sample);
    core.step(dt_s, sample.t);
  }

  std::cout << "Final Pitch: " << sim.get_pitch() * 180.0 / 3.14159265358979323846 << " deg\n";
  std::cout << "Final Commands: left=" << left_sps << " right=" << right_sps << "\n";

  return std::abs(sim.get_pitch()) > (75.0 * 3.14159265358979323846 / 180.0) ? 1 : 0;
}
