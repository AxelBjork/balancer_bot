#pragma once

#include <atomic>
#include <filesystem>
#include <thread>
#include <vector>

#include "pigpiod_if2.h" /* for gpioPulse_t */

// Forward declaration of the stub accessors
std::vector<gpioPulse_t> pigpio_stub_get_wave_pulses(int wave_id);
int pigpio_stub_get_active_wave();
int pigpio_stub_get_gpio_level(int pin);

struct SimulatorConfig {
  std::filesystem::path iio_root = "/tmp/balancer_sim_root";
  double dt = 0.001;                    // Simulation step 1ms (1000Hz)
  double com_angle_offset_rad = 0.001;  // COM angular offset (positive = COM forward)
  double initial_pitch_deg = 2.0;       // Initial pitch angle (positive = forward)
};

class BalancerSimulator {
 public:
  using Config = SimulatorConfig;

  BalancerSimulator(const Config& cfg = Config());
  ~BalancerSimulator();

  void start();
  void stop();
  void step();  // Run one physics step

  // Accessors for state visualization
  double get_pitch() const {
    return state_.pitch;
  }
  double get_position() const {
    return state_.position;
  }

 private:
  void setup_mock_hw();
  void teardown_mock_hw();
  void update_imu_files();

  struct State {
    double pitch = 0.0;
    double pitch_rate = 0.0;
    double position = 0.0;
    double velocity = 0.0;
  };

  Config cfg_;
  State state_;
  std::atomic<bool> running_{false};
  std::thread sim_thread_;

  // Physics constants (User provided: 1.032kg, 14cm tall, 80mm wheels, COM 6cm)
  static constexpr double gravity = 9.81;
  static constexpr double wheel_radius = 0.080 / 2.0;    // 80mm diameter
  static constexpr double robot_mass = 1.032;            // kg
  static constexpr double center_of_mass_height = 0.06;  // 6cm COM
  static constexpr double robot_height = 0.14;           // 14cm total height

  // Moment of Inertia about Axle: I = I_com + m*l^2.
  // Modeling as a rod of height h: I_com = 1/12 * m * h^2.
  // I_axle = m * (h^2/12 + (h/2)^2) = m * (h^2/12 + h^2/4) = m * h^2 * (1+3)/12 = m * h^2 / 3.
  static constexpr double moment_of_inertia = robot_mass * (robot_height * robot_height) / 3.0;

  // Motor constants for ideal stepper
  static constexpr double steps_per_rev = 200.0 * 16.0;  // 1/16 microstepping

  // Simulation State
  double last_wheel_velocity_ = 0.0;  // m/s

  // Paths
  std::filesystem::path accel_dir_;
  std::filesystem::path gyro_dir_;
  std::filesystem::path trigger_dir_;
  std::filesystem::path trigger_config_dir_;

  // FDs
  int fd_accel_ = -1;
  int fd_gyro_ = -1;

  std::chrono::system_clock::time_point simulated_clock_;
};
