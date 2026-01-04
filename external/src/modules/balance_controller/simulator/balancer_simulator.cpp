#include "simulator/balancer_simulator.h"

#include <iostream>
#include <fstream>
#include <cmath>
#include <thread>
#include <chrono>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>

namespace fs = std::filesystem;

BalancerSimulator::BalancerSimulator(const Config& cfg) : cfg_(cfg) {
  state_.pitch = cfg_.initial_pitch_deg * M_PI / 180.0; // Initial tilt 1.0 degrees
  setup_mock_hw();
}

BalancerSimulator::~BalancerSimulator() {
  stop();
  if (fd_accel_ >= 0) close(fd_accel_);
  if (fd_gyro_ >= 0) close(fd_gyro_);
  teardown_mock_hw();
}

void BalancerSimulator::setup_mock_hw() {
  fs::create_directories(cfg_.iio_root / "bus/iio/devices");
  fs::create_directories(cfg_.iio_root / "dev");
  fs::create_directories(cfg_.iio_root / "kernel/config/iio/triggers");
  
  // Create trigger device (trigger0) logic
  // The reader looks in bus/iio/devices/trigger*/name = imu833
  trigger_dir_ = cfg_.iio_root / "bus/iio/devices/trigger0";
  fs::create_directories(trigger_dir_);
  std::ofstream(trigger_dir_ / "name") << "imu833";
  std::ofstream(trigger_dir_ / "sampling_frequency") << "833"; // Allow writing
  
  // Create accel device (iio:device0)

  // Create accel device (iio:device0)
  accel_dir_ = cfg_.iio_root / "bus/iio/devices/iio:device0";
  fs::create_directories(accel_dir_ / "scan_elements");
  fs::create_directories(accel_dir_ / "buffer");
  fs::create_directories(accel_dir_ / "trigger");

  std::ofstream(accel_dir_ / "name") << "ism330dhcx_accel";
  std::ofstream(accel_dir_ / "scan_elements/in_accel_x_type") << "le:s16/16>>0";
  std::ofstream(accel_dir_ / "scan_elements/in_accel_y_type") << "le:s16/16>>0";
  std::ofstream(accel_dir_ / "scan_elements/in_accel_z_type") << "le:s16/16>>0";
  std::ofstream(accel_dir_ / "scan_elements/in_timestamp_type") << "le:s64/64>>0";
  std::ofstream(accel_dir_ / "scan_elements/in_accel_x_index") << "0";
  std::ofstream(accel_dir_ / "scan_elements/in_accel_y_index") << "1";
  std::ofstream(accel_dir_ / "scan_elements/in_accel_z_index") << "2";
  std::ofstream(accel_dir_ / "scan_elements/in_accel_z_index") << "2";
  std::ofstream(accel_dir_ / "scan_elements/in_timestamp_index") << "3";
  // Create current_trigger for reader to write to
  std::ofstream(accel_dir_ / "trigger/current_trigger") << "";

  // Create gyro device (iio:device1)
  gyro_dir_ = cfg_.iio_root / "bus/iio/devices/iio:device1";
  fs::create_directories(gyro_dir_ / "scan_elements");
  fs::create_directories(gyro_dir_ / "buffer");
  fs::create_directories(gyro_dir_ / "trigger");

  std::ofstream(gyro_dir_ / "name") << "ism330dhcx_gyro";
  std::ofstream(gyro_dir_ / "scan_elements/in_anglvel_x_type") << "le:s16/16>>0";
  std::ofstream(gyro_dir_ / "scan_elements/in_anglvel_y_type") << "le:s16/16>>0";
  std::ofstream(gyro_dir_ / "scan_elements/in_anglvel_z_type") << "le:s16/16>>0";
  std::ofstream(gyro_dir_ / "scan_elements/in_timestamp_type") << "le:s64/64>>0";
  std::ofstream(gyro_dir_ / "scan_elements/in_anglvel_x_index") << "0";
  std::ofstream(gyro_dir_ / "scan_elements/in_anglvel_y_index") << "1";
  std::ofstream(gyro_dir_ / "scan_elements/in_anglvel_z_index") << "2";
  std::ofstream(gyro_dir_ / "scan_elements/in_timestamp_index") << "3";
  // Create current_trigger for reader to write to
  std::ofstream(gyro_dir_ / "trigger/current_trigger") << "";
  
  // Create FIFOs for data
  mkfifo((cfg_.iio_root / "dev/iio:device0").c_str(), 0666);
  mkfifo((cfg_.iio_root / "dev/iio:device1").c_str(), 0666);
}

void BalancerSimulator::teardown_mock_hw() {
  fs::remove_all(cfg_.iio_root);
}

void BalancerSimulator::start() {
  if (running_) return;
  running_ = true;
  sim_thread_ = std::thread([this]() {
    while (running_) {
      step();
      std::this_thread::sleep_for(std::chrono::duration<double>(cfg_.dt));
    }
  });
}

void BalancerSimulator::stop() {
  running_ = false;
  if (sim_thread_.joinable()) sim_thread_.join();
}

void BalancerSimulator::step() {
  // 1. Read Motor Input (Mock) and Determine Base Velocity
  double wheel_velocity = 0.0;
  int wave = pigpio_stub_get_active_wave();
  
  if (wave >= 0) {
    auto pulses = pigpio_stub_get_wave_pulses(wave);
    // Simple frequency estimation:
    // Count pulses in the wave, divide by total duration of wave.
    // Or just look at pulse length.
    // Stub is simple: a wave is a sequence of pulses. 
    // Pulse struct: {gpioOn, gpioOff, usDelay}
    
    if (!pulses.empty()) {
      // Calculate period of one cycle (assuming uniform pulsetrain for now)
      // cycle_us = sum(usDelay) / num_on_off_pairs?
      // Or just take the first pulse delay if it's a square wave?
      // A standard stepper wave might be: ON(bit), delay(half_period), OFF(bit), delay(half_period).
      // So period = sum of delays.
      
      uint32_t total_us = 0;
      for (auto& p : pulses) total_us += p.usDelay;
      
      double duration_s = total_us / 1000000.0;
      if (duration_s > 1e-6) {
         // pulses.size() is number of pulse structs. 
         // If generic wave, might be transitions. 
         // Assuming simple stepper driver logic: 1 pulse = 1 step.
         // Usually 1 pulse struct = 1 change. 
         // A SET/CLEAR pair is often 2 structs or 1 struct with both if simultaneous (unlikely for PWM).
         // Let's approximate: 2 structs per step (On -> Delay -> Off -> Delay).
         double steps = pulses.size() / 2.0; 
         double steps_per_sec = steps / duration_s;
         
         // Direction Check using Stub GPIO state
         // Left Motor (Pin 13): Config::invert_left = true.
         //   Logic: If Inverted, Stepper driver gets !(dir). 
         //   If Controller wants FWD (Speed > 0), Stepper logic: if(inv) dir=!1=0.
         //   If Controller wants BWD (Speed < 0) -> "Forward" for Balance?
         //   Analysis of loop instability: Negative u (L=1, R=0) MUST mean Forward Velocity.
         //   So Pin 13 == 1 => Forward. Pin 13 == 0 => Backward.
         
         // Right Motor (Pin 24): Config::invert_right = false.
         //   Logic: If !Inverted, Stepper driver gets (dir).
         //   Negative u -> R=0.
         //   So Pin 24 == 0 => Forward. Pin 24 == 1 => Backward.
         
         int level_left = pigpio_stub_get_gpio_level(13);
         int level_right = pigpio_stub_get_gpio_level(24);
         
         double dir_left = (level_left == 1) ? 1.0 : -1.0;
         double dir_right = (level_right == 0) ? 1.0 : -1.0;
         
         // Average direction (ignore turning for 2D physics)
         double avg_dir = (dir_left + dir_right) / 2.0;

         wheel_velocity = (steps_per_sec / steps_per_rev) * 2.0 * M_PI * wheel_radius * avg_dir;
      }
    }
  }
  
  // Smooth/Low-pass the velocity change to simulate inertia/finite torque?
  // Ideally steppers are stiff, so velocity is exactly as commanded.
  
  state_.velocity = wheel_velocity;
  state_.position += state_.velocity * cfg_.dt;

  // 2. Physics Update (Inverted Pendulum)
  // Base acceleration
  double base_accel = (state_.velocity - last_wheel_velocity_) / cfg_.dt;
  last_wheel_velocity_ = state_.velocity;

  // Equation of Motion:
  // I_pivot * alpha = m * g * l * sin(theta) - m * l * a_base * cos(theta)
  // alpha = (m*g*l*sin(theta) - m*l*a_base*cos(theta)) / I_pivot
  
  double m = robot_mass;
  double l = center_of_mass_height;
  double I = moment_of_inertia;
  
  // Calculate torques
  // Gravity torque: depends on angle of COM vector relative to vertical.
  // theta = 0 means chassis is vertical.
  // If COM offset is positive (forward), effective angle is theta + offset.
  double torque_grav = m * gravity * l * std::sin(state_.pitch + cfg_.com_angle_offset_rad);
  double torque_inertial = -m * l * base_accel * std::cos(state_.pitch);
  double alpha = (torque_grav + torque_inertial) / I;

  state_.pitch_rate += alpha * cfg_.dt;
  state_.pitch += state_.pitch_rate * cfg_.dt;

  // Simple ground collision
  if (std::abs(state_.pitch) > M_PI / 2.0) {
      state_.pitch = (state_.pitch > 0) ? M_PI / 2.0 : -M_PI / 2.0;
      state_.pitch_rate = 0;
  }
  
  // 3. Update IMU data
  update_imu_files();
}

void BalancerSimulator::update_imu_files() {
  // Write to FIFOs
  // Format: 3x s16 (accel/gyro) + s64 (timestamp)
  // Total 16 bytes (aligned)
  
  struct {
    int16_t x, y, z;
    int16_t pad;
    int64_t ts;
  } packet;
  
  // Accel: gravity vector rotated by pitch
  // If robot pitches FWD (positive pitch?), gravity (down) vector in body frame:
  // Body X is forward, Z is up? 
  // If upright: Z sees +1g (or -1g?), X sees 0.
  // If pitch=+90deg (nose down), X sees +1g
  
  // Accel scale: 1 unit = kAccelScale m/s^2? No, kAccelScale converts raw to m/s^2.
  // Standard gravity 9.81 m/s^2.
  // raw values approx 16384 per g (if +/- 2g scale)
  
  const double g = 9.81;
  const double scale_accel = 16384.0 / 9.81; // approx
  
  double ax = -g * std::sin(state_.pitch);
  double az = g * std::cos(state_.pitch);
  
  packet.x = (int16_t)(ax * scale_accel);
  packet.y = 0;
  packet.z = (int16_t)(az * scale_accel);
  packet.ts = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  
  // Write to Accel FIFO
  if (fd_accel_ < 0) {
      fd_accel_ = open((cfg_.iio_root / "dev/iio:device0").c_str(), O_WRONLY | O_NONBLOCK);
  }
  if (fd_accel_ >= 0) {
      int ret = write(fd_accel_, &packet, 16);
      if (ret != 16) {
          if (errno != EAGAIN) { // EAGAIN is just buffer full
              close(fd_accel_);
              fd_accel_ = -1;
          }
      }
  }

  // Gyro
  double scale_gyro = 1000.0; // approx raw per rad/s
  packet.x = 0;
  packet.y = (int16_t)(state_.pitch_rate * scale_gyro); // Pitch rate is usually Y axis in some frames?
  
  packet.z = 0;
  packet.ts = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  
  if (fd_gyro_ < 0) {
      fd_gyro_ = open((cfg_.iio_root / "dev/iio:device1").c_str(), O_WRONLY | O_NONBLOCK);
  }
  if (fd_gyro_ >= 0) {
      int ret = write(fd_gyro_, &packet, 16);
      if (ret != 16) {
          if (errno != EAGAIN) {
             close(fd_gyro_);
             fd_gyro_ = -1;
          }
      }
  }
}
