#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

#include "services/main/config.h"
#include "simulator/simulator_runner.h"
#include "simulator/tuner_support.h"

namespace {

constexpr double kControlDtS = 1.0 / 400.0;

struct Variant {
  std::string name;
  bool endpoint_continuity_enabled;
  bool matched_reference_filter_enabled;
  bool drive_feedforward_enabled;
};

std::string file_component(std::string value) {
  for (char& character : value) {
    if (!std::isalnum(static_cast<unsigned char>(character)) && character != '_' &&
        character != '-' && character != '.') {
      character = '_';
    }
  }
  return value;
}

void write_timeline(const std::filesystem::path& path,
                    const std::vector<SimulatorTimelineRow>& rows) {
  std::ofstream output(path);
  output << "time_s,user_velocity_mps,reference_velocity_mps,"
            "velocity_feedback_reference_mps,reference_acceleration_mps2,reference_jerk_mps3,"
            "plant_position_m,plant_velocity_mps,velocity_feedback_estimate_mps,velocity_error_mps,"
            "velocity_feedback_acceleration_mps2,velocity_p_acceleration_mps2,"
            "velocity_i_acceleration_mps2,acceleration_raw_mps2,acceleration_cmd_mps2,"
            "drive_pitch_target_deg,drive_feedforward_sps,balance_correction_sps,"
            "common_unclamped_sps,pitch_feedback_sps,pitch_rate_feedback_sps,"
            "pitch_accel_feedback_sps,final_pitch_target_deg,pitch_error_deg,"
            "plant_pitch_deg,plant_pitch_rate_dps,pitch_rate_dps,"
            "filtered_pitch_rate_dps,u_sps,"
            "balance_unclamped_sps,left_sps,left_slewed_sps,right_slewed_sps,"
            "command_saturated,actuator_saturation_flags,controller_fault_flags,"
            "outer_acceleration_limited,outer_pitch_target_limited,"
            "stepper_actual_wheel_velocity_mps,stepper_chassis_velocity_mps,"
            "stepper_electrical_phase_error_left_rad,stepper_electrical_phase_error_right_rad,"
            "stepper_voltage_saturated_left,stepper_voltage_saturated_right,"
            "stepper_traction_utilization,stepper_traction_saturated,"
            "stepper_accumulated_slip_distance_m\n";
  output << std::setprecision(12);
  for (const auto& row : rows) {
    output << row.sim_time_s << ',' << row.user_velocity_mps << ','
           << row.reference_velocity_mps << ',' << row.velocity_feedback_reference_mps << ','
           << row.reference_acceleration_mps2 << ',' << row.reference_jerk_mps3 << ','
           << row.plant_position << ',' << row.plant_velocity << ','
           << row.velocity_feedback_estimate_mps << ','
           << row.velocity_error_mps << ',' << row.velocity_feedback_acceleration_mps2 << ','
           << row.velocity_p_acceleration_mps2 << ',' << row.velocity_i_acceleration_mps2 << ','
           << row.acceleration_raw_mps2 << ',' << row.acceleration_cmd_mps2 << ','
           << row.drive_pitch_target_deg << ',' << row.drive_feedforward_sps << ','
           << row.balance_correction_sps << ',' << row.common_unclamped_sps << ','
           << row.pitch_feedback_sps << ',' << row.pitch_rate_feedback_sps << ','
           << row.pitch_accel_feedback_sps << ',' << row.final_pitch_target_deg << ','
           << row.pitch_error_deg << ','
           << row.plant_pitch_deg << ','
           << row.plant_pitch_rate_dps << ',' << row.pitch_rate_dps << ','
           << row.filtered_pitch_rate_dps << ',' << row.u_sps << ','
           << row.balance_unclamped_sps << ',' << row.left_sps << ',' << row.left_slewed_sps
           << ',' << row.right_slewed_sps << ',' << row.command_saturated << ','
           << row.actuator_saturation_flags << ',' << row.controller_fault_flags << ','
           << row.outer_acceleration_limited << ',' << row.outer_pitch_target_limited << ','
           << row.stepper_actual_wheel_velocity_mps << ',' << row.stepper_chassis_velocity_mps
           << ',' << row.stepper_electrical_phase_error_left_rad << ','
           << row.stepper_electrical_phase_error_right_rad << ','
           << row.stepper_voltage_saturated_left << ',' << row.stepper_voltage_saturated_right
           << ',' << row.stepper_traction_utilization << ','
           << row.stepper_traction_saturated << ','
           << row.stepper_accumulated_slip_distance_m << '\n';
  }
}

void write_variant_metrics(std::ofstream& output, const Variant& variant,
                           const SimulatorScenario& scenario,
                           const SimulatorRunResult& result) {
  const auto metrics = calculate_tuning_metrics(result);
  double drive_feedforward_sum = 0.0;
  double balance_correction_abs_sum = 0.0;
  double pitch_error_abs_sum = 0.0;
  size_t steady_count = 0;
  for (const auto& row : result.rows) {
    if (std::abs(row.reference_velocity_mps) < 0.25) continue;
    drive_feedforward_sum += row.drive_feedforward_sps;
    balance_correction_abs_sum += std::abs(row.balance_correction_sps);
    pitch_error_abs_sum += std::abs(row.pitch_error_deg);
    ++steady_count;
  }
  const double steady_scale = steady_count > 0 ? 1.0 / static_cast<double>(steady_count) : 0.0;
  output << variant.name << ',' << scenario.name << ','
         << result.fell << ',' << result.controller_fault_flags << ','
         << result.actuator_fault_count << ',' << result.max_abs_pitch_deg << ','
         << result.final_pitch_deg << ',' << result.tail_rms_pitch_deg << ','
         << result.max_continuous_saturation_s << ','
         << metrics.mechanical_velocity_hold_target_fraction << ','
         << metrics.mechanical_velocity_hold_abs_error_mps << ','
         << metrics.release_distance_m << ',' << metrics.rebound_velocity_mps << ','
         << drive_feedforward_sum * steady_scale << ','
         << balance_correction_abs_sum * steady_scale << ','
         << pitch_error_abs_sum * steady_scale << ',' << metrics.slew_active_fraction << ','
         << metrics.requested_applied_error_p95_sps << ','
         << metrics.requested_command_delta_p95_sps << ','
         << metrics.integrated_slew_excess_sps << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  std::filesystem::path output_dir = "build/sim/outer_loop_ab";
  std::string pid_path = "pid.conf";
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--output" && index + 1 < argc) {
      output_dir = argv[++index];
    } else if (argument == "--pid" && index + 1 < argc) {
      pid_path = argv[++index];
    } else {
      std::cerr << "Usage: " << argv[0] << " [--output DIR] [--pid FILE]\n";
      return 1;
    }
  }

  std::filesystem::create_directories(output_dir);
  ConfigPid::load(pid_path);
  const std::vector<Variant> variants = {
      {"baseline", false, false, false},
      {"planner_only", true, false, false},
      {"matched_reference_only", false, true, false},
      {"combined", true, true, false},
      {"new_architecture", true, true, true},
  };
  const auto scenarios = tuning_root_controller_scenario_set(
      PhysicsProfile::StepperPhaseElectrical);

  std::ofstream summary(output_dir / "summary.csv");
  summary << "variant,scenario,fell,controller_fault_flags,actuator_fault_count,"
             "max_abs_pitch_deg,final_pitch_deg,tail_rms_pitch_deg,"
             "max_continuous_saturation_s,hold_target_fraction,hold_abs_error_mps,"
             "release_distance_m,rebound_velocity_mps,steady_drive_feedforward_sps,"
             "steady_abs_balance_correction_sps,steady_abs_pitch_error_deg,"
             "slew_active_fraction,requested_applied_error_p95_sps,"
             "requested_command_delta_p95_sps,integrated_slew_excess_sps\n";
  std::ofstream metadata(output_dir / "metadata.txt");
  metadata << "Velocity-reference outer-loop architecture A/B\n"
           << "pid=" << pid_path << "\n"
           << "profile=StepperPhaseElectrical\n"
           << "endpoint_continuity, matched_reference_filter, and drive feedforward are simulator-only hooks\n"
           << "all variants use identical root scenario definitions and gains\n"
           << "controller MotorRunner slew behavior is unchanged\n";

  for (const auto& variant : variants) {
    const auto variant_dir = output_dir / variant.name;
    std::filesystem::create_directories(variant_dir);
    for (const auto& source_scenario : scenarios) {
      auto scenario = source_scenario;
      scenario.simulation_endpoint_continuity_enabled =
          variant.endpoint_continuity_enabled;
      scenario.simulation_matched_reference_filter_enabled =
          variant.matched_reference_filter_enabled;
      scenario.simulation_drive_feedforward_enabled = variant.drive_feedforward_enabled;
      const auto result = run_simulator_scenario_with_loaded_pid(scenario);
      write_timeline(variant_dir / (file_component(scenario.name) + ".csv"), result.rows);
      write_variant_metrics(summary, variant, scenario, result);
    }
  }
  std::cout << "variants=" << variants.size() << " scenarios=" << scenarios.size()
            << " output=" << output_dir << '\n';
  return 0;
}
