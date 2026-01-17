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
  double dt = 0.0005;                    // Simulation step 1ms (1000Hz)
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

    // Helper accessors via references
    double& theta() {
      return pitch;
    }
    double& theta_dot() {
      return pitch_rate;
    }
    double& x() {
      return position;
    }
    double& v() {
      return velocity;
    }
  };

  Config cfg_;
  State state_;
  std::atomic<bool> running_{false};
  std::thread sim_thread_;

  // Physics constants (User provided: 1.032kg, 14cm tall, 80mm wheels, COM 6cm)
  static constexpr double gravity = 9.81;
  static constexpr double wheel_radius = 0.080 / 2.0;          // 80mm diameter
  static constexpr double robot_mass = 1.032;                  // kg (Total)
  static constexpr double wheel_mass = 0.050;                  // kg (per wheel, approx)
  static constexpr double cart_mass = 2.0 * wheel_mass;        // Mass of "cart" (wheels)
  static constexpr double body_mass = robot_mass - cart_mass;  // Mass of pendulum
  static constexpr double center_of_mass_height = 0.06;        // 6cm COM (length l)

  // Inertia about COM. Modeling as rod + discrete mass? Or just user provided?
  // User estimated I_pivot. We need I_com.
  // I_pivot = I_com + m*l^2  => I_com = I_pivot - m*l^2
  // Previous I_pivot was 1.032 * 0.14^2 / 3 ~ 0.00674
  // l = 0.06. m*l^2 = 0.932 * 0.0036 ~ 0.00335
  // I_com ~ 0.0034.
  static constexpr double I_com = 0.0034;

  // Motor Constants (Nema 17 approx)
  static constexpr double kMotorR = 3.6;          // Ohms
  static constexpr double kMotorKe = 0.35;        // V/(rad/s) Back EMF constant (approx for Nema17)
  static constexpr double kMotorKt = kMotorKe;    // Torque constant (Nm/A)
  static constexpr double kSupplyVoltage = 12.0;  // Volts

  // Simulation State
  double last_stepper_position_ = 0.0;  // To track velocity command from steps
  double current_wheel_omega_ = 0.0;    // Current wheel angular velocity (rad/s)
  double last_x_ddot_ = 0.0;            // For IMU specific force

  // Motor constants for ideal stepper
  static constexpr double steps_per_rev = 200.0 * 16.0;  // 1/16 microstepping

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
