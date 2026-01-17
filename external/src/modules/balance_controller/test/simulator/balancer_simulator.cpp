#include "balancer_simulator.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <thread>

namespace fs = std::filesystem;

BalancerSimulator::BalancerSimulator(const Config& cfg) : cfg_(cfg) {
  state_.pitch = cfg_.initial_pitch_deg * M_PI / 180.0;  // Initial tilt 1.0 degrees
  simulated_clock_ = std::chrono::system_clock::now();
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
  std::ofstream(trigger_dir_ / "sampling_frequency") << "833";  // Allow writing

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
  // 1. Determine Target Wheel Velocity from Stepper Pulses
  double target_wheel_velocity = 0.0;
  int wave = pigpio_stub_get_active_wave();

  if (wave >= 0) {
    auto pulses = pigpio_stub_get_wave_pulses(wave);
    if (!pulses.empty()) {
      uint32_t total_us = 0;
      for (auto& p : pulses) total_us += p.usDelay;
      double duration_s = total_us / 1000000.0;

      if (duration_s > 1e-6) {
        double steps = pulses.size() / 2.0;
        double steps_per_sec = steps / duration_s;

        // Determine direction from GPIO pins
        int level_left = pigpio_stub_get_gpio_level(13);   // L: inv=true -> level 1=FWD
        int level_right = pigpio_stub_get_gpio_level(24);  // R: inv=false -> level 0=FWD

        double dir_left = (level_left == 1) ? 1.0 : -1.0;
        double dir_right = (level_right == 0) ? 1.0 : -1.0;
        double avg_dir = (dir_left + dir_right) / 2.0;

        target_wheel_velocity =
            (steps_per_sec / steps_per_rev) * 2.0 * M_PI * wheel_radius * avg_dir;
      }
    }
  }

  // 2. Motor / Wheel Dynamics (Torque Generation)
  double current_wheel_v = state_.v();
  double v_err = target_wheel_velocity - current_wheel_v;

  // Driver Stiffness (Proportional Control)
  double K_driver_p = 500.0;  // N / (m/s)
  double F_cmd = K_driver_p * v_err;

  // Saturation (Max Force ~ 20N approx for Nema17 pair)
  double F_max = 20.0;
  double F_app = std::max(-F_max, std::min(F_max, F_cmd));

  // 3. Coupled Equations of Motion
  double Q = state_.theta();
  double Q_dot = state_.theta_dot();
  double sQ = std::sin(Q);
  double cQ = std::cos(Q);

  double M = cart_mass;
  double m = body_mass;
  double l = center_of_mass_height;
  double I = I_com;

  double d11 = M + m;
  double d12 = m * l * cQ;
  double d21 = m * l * cQ;
  double d22 = I + m * l * l;

  // Friction / Damping
  double c_x = 2.0;    // N / (m/s)  Viscous friction cart
  double c_th = 0.05;  // Nm / (rad/s) Viscous friction pendulum

  double rhs1 = F_app + m * l * Q_dot * Q_dot * sQ - c_x * state_.v();
  double rhs2 = m * gravity * l * sQ - c_th * state_.theta_dot();  // Gravity - Damping

  double det = d11 * d22 - d12 * d21;
  double x_ddot = (d22 * rhs1 - d12 * rhs2) / det;
  double theta_ddot = (d11 * rhs2 - d21 * rhs1) / det;

  // 4. Integration (Semi-Implicit Euler)
  state_.v() += x_ddot * cfg_.dt;
  state_.x() += state_.v() * cfg_.dt;

  state_.theta_dot() += theta_ddot * cfg_.dt;
  state_.theta() += state_.theta_dot() * cfg_.dt;

  // Ground Collision
  if (std::abs(state_.theta()) > M_PI / 2.0) {
    state_.theta() = (state_.theta() > 0) ? M_PI / 2.0 : -M_PI / 2.0;
    state_.theta_dot() = 0.0;
    state_.v() = 0.0;
  }

  // Store x_ddot for IMU
  last_x_ddot_ = x_ddot;

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

  // IMU Accelerometer Model (Specific Force)
  // f_meas = R^T * (a_world - g_world)
  // a_world = [x_ddot, 0, 0]
  // g_world = [0, 0, -g]
  // term (a - g) = [x_ddot, 0, g]

  double Q = state_.theta();
  double x_ddot = last_x_ddot_;  // Retrieves stored accel
  const double g = 9.81;

  // Body Frame Specific Force (X=Fwd, Z=Up)
  double ax_mps2 = x_ddot * std::cos(Q) + g * std::sin(Q);
  double az_mps2 = -x_ddot * std::sin(Q) + g * std::cos(Q);

  const double scale_accel = 16384.0 / 9.81;

  // Mapping to "Raw" Packet Indices based on config.h:
  // accel_cfg = {x=0, y=2, z=1, inv_x=1, inv_z=1}
  // BodyX = -src[0] => src[0] = -BodyX
  // BodyZ = -src[1] => src[1] = -BodyZ
  // BodyY = src[2]  => src[2] = BodyY (0)

  packet.x = (int16_t)(-ax_mps2 * scale_accel);  // Index 0
  packet.y = (int16_t)(-az_mps2 * scale_accel);  // Index 1
  packet.z = 0;                                  // Index 2
  packet.ts =
      std::chrono::duration_cast<std::chrono::nanoseconds>(simulated_clock_.time_since_epoch())
          .count();

  // Write to Accel FIFO
  if (fd_accel_ < 0) {
    fd_accel_ = open((cfg_.iio_root / "dev/iio:device0").c_str(), O_WRONLY | O_NONBLOCK);
  }
  if (fd_accel_ >= 0) {
    int ret = write(fd_accel_, &packet, 16);
    if (ret != 16) {
      if (errno != EAGAIN) {
        close(fd_accel_);
        fd_accel_ = -1;
      }
    }
  }

  // Gyro
  // gyro_cfg = {x=0, y=2, z=1, inv_x=1, inv_z=1}
  // BodyY (PitchRate) = src[2] => src[2] = BodyY
  double scale_gyro = 6550.0;  // Match Reader kGyroScale
  packet.x = 0;
  packet.y = 0;

  double val_pitch = -state_.theta_dot() * scale_gyro;
  val_pitch = std::max(-32767.0, std::min(32767.0, val_pitch));
  packet.z = (int16_t)val_pitch;  // Index 2
  simulated_clock_ += std::chrono::nanoseconds((int64_t)(cfg_.dt * 1e9));
  packet.ts =
      std::chrono::duration_cast<std::chrono::nanoseconds>(simulated_clock_.time_since_epoch())
          .count();

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
