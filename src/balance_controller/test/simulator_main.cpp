#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "simulator/simulator_runner.h"
#include "types.h"

namespace {

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
  return std::filesystem::path("build") / "sim" / "latest";
}

PhysicsProfile parse_physics_profile(std::string_view value) {
  if (value == "simplified") {
    return PhysicsProfile::Simplified;
  }
  if (value == "realistic") {
    return PhysicsProfile::Realistic;
  }
  throw std::runtime_error("Unknown physics profile: " + std::string(value));
}

void print_usage() {
  std::cout
      << "Usage: balancer_simulator [--scenario <name> | --scenario-set <required|capability|all>]\n"
      << "                          [--physics-profile <simplified|realistic>]\n"
      << "                          [--driver-kp <value>] [--max-force <value>]\n"
      << "                          [--cart-damping <value>] [--pitch-damping <value>]\n"
      << "                          [--motor-tau <value>]\n"
      << "                          [--initial-pitch-deg <value>] [--com-angle-offset-rad <value>]\n"
      << "                          [--duration-s <value>]\n"
      << "                          [--disturbance-start-s <value>] [--disturbance-duration-s <value>]\n"
      << "                          [--disturbance-forward <value>] [--disturbance-turn <value>]\n"
      << "                          [--pid-config <path>] [--output-dir <dir>]\n";
}

void write_timeline_csv(const std::filesystem::path& path, const SimulatorRunResult& result) {
  std::ofstream out(path);
  out << "sim_time_s,pitch_deg,pitch_rate_dps,pitch_sp_deg,rate_sp_dps,u_sps,left_sps,right_sps,"
         "vel_error,vel_i_term,vel_p_term,out_norm,plant_pitch_deg,plant_pitch_rate_dps,"
         "plant_position,plant_velocity,target_wheel_velocity,actual_wheel_velocity,"
         "velocity_error,f_cmd,f_app,x_ddot,theta_ddot,command_saturated,force_saturated\n";
  for (const auto& row : result.rows) {
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
        << row.plant_velocity << ','
        << row.target_wheel_velocity << ','
        << row.actual_wheel_velocity << ','
        << row.velocity_error << ','
        << row.f_cmd << ','
        << row.f_app << ','
        << row.x_ddot << ','
        << row.theta_ddot << ','
        << row.command_saturated << ','
        << row.force_saturated << '\n';
  }
}

void write_metadata_json(const std::filesystem::path& path, const SimulatorRunResult& result) {
  std::ofstream out(path);
  out << "{\n";
  out << "  \"run_id\": \"" << result.scenario.name << "\",\n";
  out << "  \"source\": \"simulator_main\",\n";
  out << "  \"scenario_name\": \"" << result.scenario.name << "\",\n";
  out << "  \"initial_pitch_deg\": " << result.scenario.initial_pitch_deg << ",\n";
  out << "  \"com_angle_offset_rad\": " << result.scenario.com_angle_offset_rad << ",\n";
  out << "  \"duration_s\": " << result.scenario.duration_s << ",\n";
  out << "  \"dt_s\": " << (1.0 / 400.0) << ",\n";
  out << "  \"physics_profile\": \"" << BalancerSimulator::profile_name(result.scenario.physics_profile)
      << "\",\n";
  out << "  \"pid_profile\": \"" << result.pid_config_path << "\",\n";
  out << "  \"max_command_sps\": " << kMaxSps << ",\n";
  out << "  \"physics\": {\n";
  out << "    \"driver_kp\": " << result.physics.driver_kp << ",\n";
  out << "    \"max_force_n\": " << result.physics.max_force_n << ",\n";
  out << "    \"cart_damping\": " << result.physics.cart_damping << ",\n";
  out << "    \"pitch_damping\": " << result.physics.pitch_damping << ",\n";
  out << "    \"motor_tau_s\": " << result.physics.motor_tau_s << "\n";
  out << "  }";
  if (result.scenario.disturbance.has_value()) {
    const auto& d = *result.scenario.disturbance;
    out << ",\n  \"disturbance\": {\n";
    out << "    \"start_s\": " << d.start_s << ",\n";
    out << "    \"duration_s\": " << d.duration_s << ",\n";
    out << "    \"forward\": " << d.forward << ",\n";
    out << "    \"turn\": " << d.turn << "\n";
    out << "  }\n";
  } else {
    out << "\n";
  }
  out << "}\n";
}

int generate_artifacts(const std::filesystem::path& output_dir) {
  const auto script = std::filesystem::path(BALANCER_REPO_ROOT) / "tools" / "run_artifacts.py";
  const auto timeline = output_dir / "timeline.csv";
  const auto metadata = output_dir / "metadata.json";

  std::ostringstream cmd;
  cmd << "python3 " << shell_quote(script)
      << " --csv " << shell_quote(timeline)
      << " --metadata " << shell_quote(metadata)
      << " --output-dir " << shell_quote(output_dir);
  return std::system(cmd.str().c_str());
}

void write_scenario_set_summary(const std::filesystem::path& path,
                                const std::vector<SimulatorRunResult>& results) {
  std::ofstream out(path);
  out << "{\n  \"scenarios\": [\n";
  for (size_t i = 0; i < results.size(); ++i) {
    const auto& result = results[i];
    out << "    {\n";
    out << "      \"name\": \"" << result.scenario.name << "\",\n";
    out << "      \"physics_profile\": \"" << BalancerSimulator::profile_name(result.scenario.physics_profile)
        << "\",\n";
    out << "      \"pid_profile\": \"" << result.pid_config_path << "\",\n";
    out << "      \"fell\": " << (result.fell ? "true" : "false") << ",\n";
    out << "      \"final_pitch_deg\": " << result.final_pitch_deg << ",\n";
    out << "      \"max_abs_pitch_deg\": " << result.max_abs_pitch_deg << "\n";
    out << "    }";
    out << (i + 1 < results.size() ? ",\n" : "\n");
  }
  out << "  ]\n}\n";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    std::optional<std::string> scenario_name;
    std::optional<std::string> scenario_set;
    std::optional<std::string> pid_config_arg;
    auto physics_profile = PhysicsProfile::Simplified;
    std::filesystem::path output_dir = output_dir_from_env();
    std::optional<double> driver_kp_override;
    std::optional<double> max_force_override;
    std::optional<double> cart_damping_override;
    std::optional<double> pitch_damping_override;
    std::optional<double> motor_tau_override;
    std::optional<double> initial_pitch_override;
    std::optional<double> com_angle_offset_override;
    std::optional<double> duration_override;
    std::optional<double> disturbance_start_override;
    std::optional<double> disturbance_duration_override;
    std::optional<float> disturbance_forward_override;
    std::optional<float> disturbance_turn_override;

    for (int i = 1; i < argc; ++i) {
      std::string_view arg = argv[i];
      auto next_value = [&](std::string_view flag) -> std::string {
        if (i + 1 >= argc) {
          throw std::runtime_error("Missing value for " + std::string(flag));
        }
        return argv[++i];
      };

      if (arg == "--scenario") {
        scenario_name = next_value(arg);
      } else if (arg == "--scenario-set") {
        scenario_set = next_value(arg);
      } else if (arg == "--physics-profile") {
        physics_profile = parse_physics_profile(next_value(arg));
      } else if (arg == "--driver-kp") {
        driver_kp_override = std::stod(next_value(arg));
      } else if (arg == "--max-force") {
        max_force_override = std::stod(next_value(arg));
      } else if (arg == "--cart-damping") {
        cart_damping_override = std::stod(next_value(arg));
      } else if (arg == "--pitch-damping") {
        pitch_damping_override = std::stod(next_value(arg));
      } else if (arg == "--motor-tau") {
        motor_tau_override = std::stod(next_value(arg));
      } else if (arg == "--initial-pitch-deg") {
        initial_pitch_override = std::stod(next_value(arg));
      } else if (arg == "--com-angle-offset-rad") {
        com_angle_offset_override = std::stod(next_value(arg));
      } else if (arg == "--duration-s") {
        duration_override = std::stod(next_value(arg));
      } else if (arg == "--disturbance-start-s") {
        disturbance_start_override = std::stod(next_value(arg));
      } else if (arg == "--disturbance-duration-s") {
        disturbance_duration_override = std::stod(next_value(arg));
      } else if (arg == "--disturbance-forward") {
        disturbance_forward_override = std::stof(next_value(arg));
      } else if (arg == "--disturbance-turn") {
        disturbance_turn_override = std::stof(next_value(arg));
      } else if (arg == "--pid-config") {
        pid_config_arg = next_value(arg);
      } else if (arg == "--output-dir") {
        output_dir = next_value(arg);
      } else if (arg == "--help") {
        print_usage();
        return 0;
      } else {
        throw std::runtime_error("Unknown argument: " + std::string(arg));
      }
    }

    if (scenario_name.has_value() && scenario_set.has_value()) {
      throw std::runtime_error("Use either --scenario or --scenario-set, not both");
    }

    const std::string pid_config_path =
        pid_config_arg.value_or(ConfigPid::resolve_path("pid_sim.conf"));
    std::optional<SimulatorPhysics> physics_override;
    if (driver_kp_override || max_force_override || cart_damping_override || pitch_damping_override ||
        motor_tau_override) {
      auto physics = BalancerSimulator::physics_for_profile(physics_profile);
      if (driver_kp_override) physics.driver_kp = *driver_kp_override;
      if (max_force_override) physics.max_force_n = *max_force_override;
      if (cart_damping_override) physics.cart_damping = *cart_damping_override;
      if (pitch_damping_override) physics.pitch_damping = *pitch_damping_override;
      if (motor_tau_override) physics.motor_tau_s = *motor_tau_override;
      physics_override = physics;
    }
    std::filesystem::create_directories(output_dir);

    if (scenario_set.has_value()) {
      std::vector<SimulatorRunResult> results;
      for (const auto& scenario : simulator_scenario_set(*scenario_set, physics_profile)) {
        auto scenario_with_override = scenario;
        scenario_with_override.physics_override = physics_override;
        if (initial_pitch_override) scenario_with_override.initial_pitch_deg = *initial_pitch_override;
        if (com_angle_offset_override) scenario_with_override.com_angle_offset_rad = *com_angle_offset_override;
        if (duration_override) scenario_with_override.duration_s = *duration_override;
        if (disturbance_start_override || disturbance_duration_override || disturbance_forward_override ||
            disturbance_turn_override) {
          auto disturbance = scenario_with_override.disturbance.value_or(SimulatorDisturbance{});
          if (disturbance_start_override) disturbance.start_s = *disturbance_start_override;
          if (disturbance_duration_override) disturbance.duration_s = *disturbance_duration_override;
          if (disturbance_forward_override) disturbance.forward = *disturbance_forward_override;
          if (disturbance_turn_override) disturbance.turn = *disturbance_turn_override;
          scenario_with_override.disturbance = disturbance;
        }
        const auto scenario_dir = output_dir / scenario.name;
        std::filesystem::create_directories(scenario_dir);
        const auto result = run_simulator_scenario(scenario_with_override, pid_config_path);
        write_timeline_csv(scenario_dir / "timeline.csv", result);
        write_metadata_json(scenario_dir / "metadata.json", result);
        const int artifact_rc = generate_artifacts(scenario_dir);
        if (artifact_rc != 0) {
          std::cerr << "Artifact generation failed for " << scenario.name << " rc=" << artifact_rc << "\n";
          return 2;
        }
        results.push_back(result);
      }

      write_scenario_set_summary(output_dir / "scenario_set_summary.json", results);

      bool any_fell = false;
      for (const auto& result : results) {
        any_fell = any_fell || result.fell;
        std::cout << result.scenario.name << ": max_abs_pitch_deg=" << result.max_abs_pitch_deg
                  << " final_pitch_deg=" << result.final_pitch_deg
                  << " fell=" << (result.fell ? "true" : "false") << "\n";
      }
      return any_fell ? 1 : 0;
    }

    const auto scenario = scenario_name.has_value()
                              ? simulator_named_scenario(*scenario_name, physics_profile)
                              : simulator_named_scenario("pitch_bias_pos", physics_profile);
    if (!scenario.has_value()) {
      throw std::runtime_error("Unknown simulator scenario");
    }

    auto scenario_with_override = *scenario;
    scenario_with_override.physics_override = physics_override;
    if (initial_pitch_override) scenario_with_override.initial_pitch_deg = *initial_pitch_override;
    if (com_angle_offset_override) scenario_with_override.com_angle_offset_rad = *com_angle_offset_override;
    if (duration_override) scenario_with_override.duration_s = *duration_override;
    if (disturbance_start_override || disturbance_duration_override || disturbance_forward_override ||
        disturbance_turn_override) {
      auto disturbance = scenario_with_override.disturbance.value_or(SimulatorDisturbance{});
      if (disturbance_start_override) disturbance.start_s = *disturbance_start_override;
      if (disturbance_duration_override) disturbance.duration_s = *disturbance_duration_override;
      if (disturbance_forward_override) disturbance.forward = *disturbance_forward_override;
      if (disturbance_turn_override) disturbance.turn = *disturbance_turn_override;
      scenario_with_override.disturbance = disturbance;
    }
    const auto result = run_simulator_scenario(scenario_with_override, pid_config_path);
    write_timeline_csv(output_dir / "timeline.csv", result);
    write_metadata_json(output_dir / "metadata.json", result);
    const int artifact_rc = generate_artifacts(output_dir);
    if (artifact_rc != 0) {
      std::cerr << "Artifact generation failed with rc=" << artifact_rc << "\n";
      return 2;
    }

    std::cout << "Scenario: " << result.scenario.name << "\n";
    std::cout << "Physics Profile: " << BalancerSimulator::profile_name(result.scenario.physics_profile) << "\n";
    std::cout << "PID Profile: " << result.pid_config_path << "\n";
    std::cout << "Final Pitch: " << result.final_pitch_deg << " deg\n";
    std::cout << "Max Abs Pitch: " << result.max_abs_pitch_deg << " deg\n";

    if (result.fell) {
      std::cerr << "TEST FAILED: Robot fell (angle > 75 deg)\n";
      return 1;
    }

    std::cout << "TEST PASSED: Robot stayed upright.\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << "\n";
    print_usage();
    return 2;
  }
}
