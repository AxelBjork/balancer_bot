#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "config.h"
#include "services/control/rate_controller_core.h"
#include "simulator/balancer_simulator.h"
#include "types.h"

namespace {

constexpr double kPi = 3.14159265358979323846;

struct TimelineRow {
  double sim_time_s{};
  double pitch_deg{};
  double pitch_rate_dps{};
  double pitch_sp_deg{};
  double rate_sp_dps{};
  double u_sps{};
  double left_sps{};
  double right_sps{};
  double vel_error{};
  double vel_i_term{};
  double vel_p_term{};
  double out_norm{};
  double plant_pitch_deg{};
  double plant_pitch_rate_dps{};
  double plant_position{};
  double plant_velocity{};
};

std::string shell_quote(const std::filesystem::path& path) {
  std::string s = path.string();
  std::string out = "'";
  for (char ch : s) {
    if (ch == '\'') {
      out += "'\\''";
    } else {
      out.push_back(ch);
    }
  }
  out.push_back('\'');
  return out;
}

std::filesystem::path output_dir_from_env() {
  if (const char* env = std::getenv("BALANCER_RUN_DIR")) {
    return env;
  }
  return std::filesystem::path("build") / "sim_runs" / "latest";
}

void write_timeline_csv(const std::filesystem::path& path, const std::vector<TimelineRow>& rows) {
  std::ofstream out(path);
  out << "sim_time_s,pitch_deg,pitch_rate_dps,pitch_sp_deg,rate_sp_dps,u_sps,left_sps,right_sps,"
         "vel_error,vel_i_term,vel_p_term,out_norm,plant_pitch_deg,plant_pitch_rate_dps,"
         "plant_position,plant_velocity\n";
  for (const auto& row : rows) {
    out << row.sim_time_s << ','
        << row.pitch_deg << ','
        << row.pitch_rate_dps << ','
        << row.pitch_sp_deg << ','
        << row.rate_sp_dps << ','
        << row.u_sps << ','
        << row.left_sps << ','
        << row.right_sps << ','
        << row.vel_error << ','
        << row.vel_i_term << ','
        << row.vel_p_term << ','
        << row.out_norm << ','
        << row.plant_pitch_deg << ','
        << row.plant_pitch_rate_dps << ','
        << row.plant_position << ','
        << row.plant_velocity << '\n';
  }
}

void write_metadata_json(const std::filesystem::path& path,
                         const BalancerSimulator::Config& cfg,
                         double dt_s) {
  std::ofstream out(path);
  out << "{\n";
  out << "  \"run_id\": \"balancer_simulator\",\n";
  out << "  \"source\": \"simulator_main\",\n";
  out << "  \"initial_pitch_deg\": " << cfg.initial_pitch_deg << ",\n";
  out << "  \"com_angle_offset_rad\": " << cfg.com_angle_offset_rad << ",\n";
  out << "  \"dt_s\": " << dt_s << "\n";
  out << "}\n";
}

}  // namespace

int main() {
  ConfigPid::load("pid.conf");

  BalancerSimulator::Config sim_cfg;
  sim_cfg.com_angle_offset_rad = 0.0;
  sim_cfg.initial_pitch_deg = 0.1;

  const auto output_dir = output_dir_from_env();
  std::filesystem::create_directories(output_dir);

  BalancerSimulator sim(sim_cfg);
  RateControllerCore core;

  float left_sps = 0.0f;
  float right_sps = 0.0f;
  Telemetry latest_telemetry{};
  bool have_telemetry = false;
  std::vector<TimelineRow> rows;
  rows.reserve(static_cast<size_t>(Config::run_seconds * Config::control_hz));

  core.setMotorOutputs([&](float left, float right) {
    left_sps = left;
    right_sps = right;
    sim.set_motor_targets(left, right);
  });
  core.setVelocityFeedback([&]() { return sim.get_actual_speed_sps(); });
  core.setJoystick(JoyCmd{0.0f, 0.0f});
  core.setTelemetrySink([&](const Telemetry& t) {
    latest_telemetry = t;
    have_telemetry = true;
  });

  constexpr double dt_s = 1.0 / 400.0;
  uint64_t sim_time_us = 0;
  double max_abs_pitch_deg = std::abs(sim.get_pitch()) * 180.0 / kPi;
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

    TimelineRow row{};
    row.sim_time_s = static_cast<double>(sim_time_us) / 1e6;
    row.left_sps = left_sps;
    row.right_sps = right_sps;
    row.plant_pitch_deg = sim.get_pitch() * 180.0 / kPi;
    row.plant_pitch_rate_dps = sim.state().pitch_rate * 180.0 / kPi;
    row.plant_position = sim.state().position;
    row.plant_velocity = sim.state().velocity;
    if (have_telemetry) {
      row.pitch_deg = latest_telemetry.pitch_deg;
      row.pitch_rate_dps = latest_telemetry.pitch_rate_dps;
      row.pitch_sp_deg = latest_telemetry.pitch_sp_deg;
      row.rate_sp_dps = latest_telemetry.rate_sp_dps;
      row.u_sps = latest_telemetry.u_sps;
      row.vel_error = latest_telemetry.vel_error;
      row.vel_i_term = latest_telemetry.vel_i_term;
      row.vel_p_term = latest_telemetry.vel_p_term;
      row.out_norm = latest_telemetry.out_norm;
    }
    rows.push_back(row);

    max_abs_pitch_deg = std::max(max_abs_pitch_deg, std::abs(row.plant_pitch_deg));
  }

  const auto timeline_path = output_dir / "timeline.csv";
  const auto metadata_path = output_dir / "metadata.json";
  write_timeline_csv(timeline_path, rows);
  write_metadata_json(metadata_path, sim_cfg, dt_s);

  std::ostringstream cmd;
  cmd << "python3 -m tools.run_artifacts"
      << " --csv " << shell_quote(timeline_path)
      << " --metadata " << shell_quote(metadata_path)
      << " --output-dir " << shell_quote(output_dir);
  const int tool_rc = std::system(cmd.str().c_str());
  if (tool_rc != 0) {
    std::cerr << "Artifact generation failed with rc=" << tool_rc << "\n";
  }

  const double final_pitch_deg = sim.get_pitch() * 180.0 / kPi;
  std::cout << "Final Pitch: " << final_pitch_deg << " deg\n";
  std::cout << "Max Abs Pitch: " << max_abs_pitch_deg << " deg\n";
  std::cout << "Final Commands: left=" << left_sps << " right=" << right_sps << "\n";

  if (max_abs_pitch_deg > 75.0) {
    std::cerr << "TEST FAILED: Robot fell (angle > 75 deg)\n";
    return 1;
  }
  std::cout << "TEST PASSED: Robot stayed upright.\n";
  return 0;
}
