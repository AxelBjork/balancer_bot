#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include "messages/types.h"
#include "simulator/simulator_runner.h"

namespace {

struct Candidate {
  double rate_p{};
  double rate_i{};
  double angle_p{};
  double angle_d{};
  double balance_limit{};
  double velocity_p{};
  double velocity_i{};
  double score{};
  double peak_pitch{};
  double tail_rms{};
  double survival_s{};
  double saturation_fraction{};
  double drive_error_mps{};
  double stop_speed_mps{};
  bool safe{};
};

Candidate evaluate(double rate_p, double rate_i, double angle_p, double angle_d,
                   double balance_limit, double velocity_p, double velocity_i) {
  ConfigPid::rate_P = rate_p;
  ConfigPid::rate_I = rate_i;
  ConfigPid::rate_D = 0.0;
  ConfigPid::angle_P = angle_p;
  ConfigPid::angle_D = angle_d;
  ConfigPid::velocity_P = velocity_p;
  ConfigPid::velocity_I = velocity_i;
  ConfigPid::pitch_max_deg = 10.0;
  ConfigPid::balance_max_sps = balance_limit;
  ConfigPid::output_scale_sps = 3200.0;

  SimulatorScenario scenario;
  scenario.physics_profile = PhysicsProfile::Realistic;
  scenario.duration_s = 20.0;
  scenario.imu_noise_seed = 2026;
  scenario.accel_noise_std_mps2 = 0.20;
  scenario.gyro_noise_std_rad_s = 0.015;
  scenario.imu_pitch_lag_s = 0.01;
  scenario.disturbances.push_back(SimulatorDisturbance{
      .kind = SimulatorDisturbanceKind::Step, .start_s = 1.0, .duration_s = 0.1, .force_n = 3.0});

  SimulatorEngine engine(scenario);
  double peak = 0.0;
  double tail_squared = 0.0;
  int tail_count = 0;
  double survival_s = scenario.duration_s;
  int saturated_samples = 0;
  constexpr int sample_count = 8000;
  for (int step = 0; step < sample_count; ++step) {
    const auto row = engine.step();
    peak = std::max(peak, std::abs(row.plant_pitch_deg));
    if (std::abs(row.plant_pitch_deg) >= 15.0 && survival_s == scenario.duration_s) {
      survival_s = row.sim_time_s;
    }
    if (row.sim_time_s >= 18.0) {
      tail_squared += row.plant_pitch_deg * row.plant_pitch_deg;
      ++tail_count;
    }
    saturated_samples += row.command_saturated > 0.5 ? 1 : 0;
  }
  const double tail_rms = tail_count > 0 ? std::sqrt(tail_squared / tail_count) : 90.0;
  const bool safe = survival_s == scenario.duration_s;
  const double saturation_fraction = saturated_samples / static_cast<double>(sample_count);
  SimulatorScenario drive;
  drive.physics_profile = PhysicsProfile::Realistic;
  drive.duration_s = 12.0;
  const double forward = 0.05 + 0.95 * (800.0 / 1200.0);
  drive.joy_segments.push_back(SimulatorJoySegment{
      .start_s = 1.0, .duration_s = 4.0, .forward = 0.0, .forward_end = forward});
  drive.joy_segments.push_back(SimulatorJoySegment{
      .start_s = 5.0, .duration_s = 4.0, .forward = forward, .forward_end = forward});
  SimulatorEngine drive_engine(drive);
  double drive_peak = 0.0;
  double drive_speed_sum = 0.0;
  int drive_speed_count = 0;
  double stop_speed_max = 0.0;
  bool drive_safe = true;
  const double target_speed = 800.0 * BalancerSimulator::HardwareNominal::meters_per_step;
  for (int step = 0; step < 4800; ++step) {
    const auto row = drive_engine.step();
    drive_peak = std::max(drive_peak, std::abs(row.plant_pitch_deg));
    if (drive_peak >= 15.0) drive_safe = false;
    if (row.sim_time_s >= 8.0 && row.sim_time_s <= 9.0) {
      drive_speed_sum += row.plant_velocity;
      ++drive_speed_count;
    }
    if (row.sim_time_s >= 11.0 && row.sim_time_s <= 12.0) {
      stop_speed_max = std::max(stop_speed_max, std::abs(row.plant_velocity));
    }
  }
  const double drive_speed =
      drive_speed_count > 0 ? drive_speed_sum / static_cast<double>(drive_speed_count) : 0.0;
  const double drive_error = std::abs(drive_speed - target_speed);
  const bool push_accepted = safe && tail_rms < 1.0;
  const bool drive_accepted =
      drive_safe && drive_error <= 0.30 * target_speed && stop_speed_max < 0.05;
  const bool all_safe = push_accepted && drive_accepted;
  const double score = all_safe
                           ? (peak + drive_peak + 30.0 * tail_rms + 20.0 * saturation_fraction +
                              200.0 * drive_error + 200.0 * stop_speed_max)
                           : std::numeric_limits<double>::infinity();
  return Candidate{rate_p,        rate_i,         angle_p,    angle_d,
                   balance_limit, velocity_p,     velocity_i, score,
                   peak,          tail_rms,       survival_s, saturation_fraction,
                   drive_error,   stop_speed_max, all_safe};
}

void apply_candidate(const Candidate& candidate) {
  ConfigPid::rate_P = candidate.rate_p;
  ConfigPid::rate_I = candidate.rate_i;
  ConfigPid::angle_P = candidate.angle_p;
  ConfigPid::angle_D = candidate.angle_d;
  ConfigPid::balance_max_sps = candidate.balance_limit;
  ConfigPid::velocity_P = candidate.velocity_p;
  ConfigPid::velocity_I = candidate.velocity_i;
}

void rank(std::vector<Candidate>& candidates) {
  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& left, const Candidate& right) {
              if (left.safe != right.safe) return left.safe > right.safe;
              return left.score < right.score;
            });
}

void write_candidates(const std::filesystem::path& path, const std::vector<Candidate>& candidates) {
  std::ofstream output(path);
  output << "rank,rate_P,rate_I,angle_P,angle_D,balance_max_sps,velocity_P,velocity_I,"
            "accepted,survival_s,peak_pitch_deg,tail_rms_deg,saturation_fraction,"
            "drive_error_mps,stop_mps,score\n";
  for (size_t index = 0; index < candidates.size(); ++index) {
    const auto& c = candidates[index];
    output << (index + 1) << ',' << c.rate_p << ',' << c.rate_i << ',' << c.angle_p << ','
           << c.angle_d << ',' << c.balance_limit << ',' << c.velocity_p << ',' << c.velocity_i
           << ',' << c.safe << ',' << c.survival_s << ',' << c.peak_pitch << ',' << c.tail_rms
           << ',' << c.saturation_fraction << ',' << c.drive_error_mps << ',' << c.stop_speed_mps
           << ',' << c.score << '\n';
  }
}

void write_grid(const std::filesystem::path& path, const std::vector<Candidate>& candidates) {
  std::ofstream output(path);
  output << "rate_P,rate_I,angle_P,angle_D,balance_max_sps,velocity_P,velocity_I\n";
  for (const auto& c : candidates) {
    output << c.rate_p << ',' << c.rate_i << ',' << c.angle_p << ',' << c.angle_d << ','
           << c.balance_limit << ',' << c.velocity_p << ',' << c.velocity_i << '\n';
  }
}

void write_stage_results(const std::filesystem::path& output_dir, std::string_view stage,
                         const std::vector<Candidate>& candidates) {
  write_grid(output_dir / (std::string(stage) + "_grid.csv"), candidates);
  write_candidates(output_dir / (std::string(stage) + "_results.csv"), candidates);
}

void write_stage_selection(const std::filesystem::path& output_dir, std::string_view stage,
                           const Candidate& selected) {
  apply_candidate(selected);
  ConfigPid::save((output_dir / (std::string(stage) + "_selected_pid.conf")).string());
  std::ofstream output(output_dir / (std::string(stage) + "_selected.json"));
  output << "{\n"
         << "  \"rate_P\": " << selected.rate_p << ",\n"
         << "  \"rate_I\": " << selected.rate_i << ",\n"
         << "  \"angle_P\": " << selected.angle_p << ",\n"
         << "  \"angle_D\": " << selected.angle_d << ",\n"
         << "  \"balance_max_sps\": " << selected.balance_limit << ",\n"
         << "  \"velocity_P\": " << selected.velocity_p << ",\n"
         << "  \"velocity_I\": " << selected.velocity_i << ",\n"
         << "  \"hard_safety_accepted\": " << (selected.safe ? "true" : "false") << ",\n"
         << "  \"score\": " << selected.score << "\n"
         << "}\n";
}

bool passes_transfer_matrix(const Candidate& candidate) {
  apply_candidate(candidate);
  for (const auto& scenario : transfer_scenario_set()) {
    if (!evaluate_transfer_scenario(run_simulator_scenario_with_loaded_pid(scenario)).accepted) {
      return false;
    }
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  std::filesystem::path base = "pid_sim.conf";
  std::filesystem::path output_dir = "build/sim/tuning";
  std::string requested_stage = "all";
  size_t top_k = 20;
  for (int index = 1; index < argc; ++index) {
    const std::string arg = argv[index];
    if (arg == "--base" && index + 1 < argc)
      base = argv[++index];
    else if (arg == "--output" && index + 1 < argc)
      output_dir = argv[++index];
    else if (arg == "--stage" && index + 1 < argc)
      requested_stage = argv[++index];
    else if (arg == "--top-k" && index + 1 < argc)
      top_k = std::stoul(argv[++index]);
    else if (arg == "--help") {
      std::cout << "Usage: balancer_simulator_tuner [--base FILE] [--output DIR] "
                   "[--stage inner|outer|joint|all] [--top-k N]\n";
      return 0;
    } else {
      std::cerr << "Unknown or incomplete argument: " << arg << '\n';
      return 2;
    }
  }
  if (requested_stage != "inner" && requested_stage != "outer" && requested_stage != "joint" &&
      requested_stage != "all") {
    std::cerr << "Invalid stage: " << requested_stage << '\n';
    return 2;
  }
  ConfigPid::load(base.string());
  std::filesystem::create_directories(output_dir);
  for (const auto* legacy_name : {"inner.csv", "outer.csv", "joint.csv"}) {
    std::filesystem::remove(output_dir / legacy_name);
  }

  // Stage 1: find the inner rate/attitude basin while holding the accepted
  // outer-loop gains fixed.
  const std::vector<double> rate_p_values{0.20, 0.30, 0.40, 0.50};
  const std::vector<double> rate_i_values{0.0, 0.01};
  const std::vector<double> angle_p_values{20.0, 28.0, 36.0, 44.0};
  const std::vector<double> angle_d_values{0.1, 0.25, 0.5, 0.8};
  std::vector<Candidate> inner;
  for (double rate_p : rate_p_values) {
    for (double rate_i : rate_i_values) {
      for (double angle_p : angle_p_values) {
        for (double angle_d : angle_d_values) {
          inner.push_back(evaluate(rate_p, rate_i, angle_p, angle_d, ConfigPid::balance_max_sps,
                                   ConfigPid::velocity_P, ConfigPid::velocity_I));
        }
      }
    }
  }
  rank(inner);
  write_stage_results(output_dir, "inner", inner);
  write_stage_selection(output_dir, "inner", inner.front());
  if (requested_stage == "inner") return inner.empty() ? 1 : 0;

  // Stage 2: tune velocity regulation with the best safe inner candidate.
  const Candidate inner_best = inner.front();
  const std::vector<double> velocity_p_values{0.001, 0.002, 0.004};
  const std::vector<double> velocity_i_values{0.0, 0.00001};
  std::vector<Candidate> outer;
  for (double velocity_p : velocity_p_values) {
    for (double velocity_i : velocity_i_values) {
      outer.push_back(evaluate(inner_best.rate_p, inner_best.rate_i, inner_best.angle_p,
                               inner_best.angle_d, inner_best.balance_limit, velocity_p,
                               velocity_i));
    }
  }
  rank(outer);
  write_stage_results(output_dir, "outer", outer);
  write_stage_selection(output_dir, "outer", outer.front());
  if (requested_stage == "outer") return outer.empty() ? 1 : 0;

  // Stage 3: refine locally around the selected inner/outer combination.
  const Candidate outer_best = outer.front();
  const std::vector<double> local_rate_p{std::max(0.05, outer_best.rate_p - 0.10),
                                         outer_best.rate_p, outer_best.rate_p + 0.10};
  const std::vector<double> local_angle_p{std::max(1.0, outer_best.angle_p - 8.0),
                                          outer_best.angle_p, outer_best.angle_p + 8.0};
  const std::vector<double> local_angle_d{std::max(0.0, outer_best.angle_d - 0.15),
                                          outer_best.angle_d, outer_best.angle_d + 0.25};
  std::vector<Candidate> joint;
  for (double rate_p : local_rate_p) {
    for (double angle_p : local_angle_p) {
      for (double angle_d : local_angle_d) {
        for (double velocity_p : velocity_p_values) {
          for (double velocity_i : velocity_i_values) {
            joint.push_back(evaluate(rate_p, outer_best.rate_i, angle_p, angle_d,
                                     outer_best.balance_limit, velocity_p, velocity_i));
          }
        }
      }
    }
  }
  rank(joint);
  write_stage_results(output_dir, "joint", joint);

  const size_t validation_count = std::min(top_k, joint.size());
  const Candidate* selected = nullptr;
  for (size_t index = 0; index < validation_count; ++index) {
    if (joint[index].safe && passes_transfer_matrix(joint[index])) {
      selected = &joint[index];
      break;
    }
  }
  if (selected == nullptr) {
    std::cerr << "No top candidate passed the complete transfer matrix\n";
    return 1;
  }
  apply_candidate(*selected);
  write_stage_selection(output_dir, "joint", *selected);
  ConfigPid::save((output_dir / "selected_pid.conf").string());
  std::ofstream provenance(output_dir / "provenance.json");
  provenance << "{\n  \"base\": \"" << base.string() << "\",\n"
             << "  \"stages\": [\"inner\", \"outer\", \"joint\"],\n"
             << "  \"validated_scenarios\": 20,\n"
             << "  \"hard_safety_rejection_before_scoring\": true,\n"
             << "  \"simulation_qualified_only\": true,\n"
             << "  \"selected\": \"joint_selected.json\",\n"
             << "  \"selected_score\": " << selected->score << "\n}\n";
  std::cout << "Selected transfer-qualified candidate: " << (output_dir / "selected_pid.conf")
            << '\n';
  return 0;
}
