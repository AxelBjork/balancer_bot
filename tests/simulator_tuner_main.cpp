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

enum class TuneStage { Inner, Outer, Joint };

bool scenario_for_stage(const SimulatorScenario& scenario, TuneStage stage) {
  const bool release = scenario.name.find("release") != std::string::npos;
  const bool push = scenario.name.find("push") != std::string::npos;
  const bool drive = scenario.name.find("drive") != std::string::npos;
  if (stage == TuneStage::Inner) return release || push;
  if (stage == TuneStage::Outer) return release || drive;
  return true;
}

Candidate evaluate(double rate_p, double rate_i, double angle_p, double angle_d,
                   double balance_limit, double velocity_p, double velocity_i, TuneStage stage) {
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

  double peak = 0.0;
  double tail_rms = 0.0;
  double survival_s = std::numeric_limits<double>::infinity();
  double saturation_fraction = 0.0;
  double drive_error = 0.0;
  double stop_speed = 0.0;
  bool all_safe = true;

  for (SimulatorScenario scenario : transfer_scenario_set()) {
    if (!scenario_for_stage(scenario, stage)) continue;
    if (stage == TuneStage::Inner) scenario.com_angle_offset_rad = 0.0;

    const auto result = run_simulator_scenario_with_loaded_pid(scenario);
    const auto acceptance = evaluate_transfer_scenario(result);
    // Intermediate stages reject physical/controller safety failures before
    // scoring, but leave complete response and settling qualification to the
    // joint stage. Otherwise a close outer candidate can never reach the
    // local refinement which is specifically intended to fix those margins.
    const bool hard_safe =
        std::none_of(acceptance.failures.begin(), acceptance.failures.end(), [](const auto& failure) {
          return failure == "fell" || failure == "peak_pitch" || failure == "actuator_fault" ||
                 failure == "controller_fault" || failure == "continuous_saturation" ||
                 failure == "non_finite";
        });
    all_safe = all_safe && (stage == TuneStage::Joint ? acceptance.accepted : hard_safe);
    peak = std::max(peak, result.max_abs_pitch_deg);
    tail_rms = std::max(tail_rms, result.tail_rms_pitch_deg);
    if (result.fell) {
      const auto fall = std::find_if(result.rows.begin(), result.rows.end(), [](const auto& row) {
        return std::abs(row.plant_pitch_deg) >= 20.0;
      });
      if (fall != result.rows.end()) survival_s = std::min(survival_s, fall->sim_time_s);
    }
    if (!result.rows.empty()) {
      const size_t saturated =
          static_cast<size_t>(std::count_if(result.rows.begin(), result.rows.end(),
                                            [](const auto& row) {
                                              return row.command_saturated > 0.5;
                                            }));
      saturation_fraction =
          std::max(saturation_fraction,
                   static_cast<double>(saturated) / static_cast<double>(result.rows.size()));
    }

    if (scenario.name.find("drive") != std::string::npos) {
      const auto mean = [&](double start_s, double end_s) {
        double sum = 0.0;
        size_t count = 0;
        for (const auto& row : result.rows) {
          if (row.sim_time_s >= start_s && row.sim_time_s < end_s) {
            sum += row.measured_vel_sps;
            ++count;
          }
        }
        return count > 0 ? sum / static_cast<double>(count) : 0.0;
      };
      const auto rms = [&](double start_s, double end_s) {
        double squared = 0.0;
        size_t count = 0;
        for (const auto& row : result.rows) {
          if (row.sim_time_s >= start_s && row.sim_time_s < end_s) {
            squared += row.measured_vel_sps * row.measured_vel_sps;
            ++count;
          }
        }
        return count > 0 ? std::sqrt(squared / static_cast<double>(count)) : 12000.0;
      };
      drive_error = std::max(
          drive_error,
          std::max(std::abs(mean(9.5, 10.8) - 800.0), std::abs(mean(16.5, 17.8) + 800.0)));
      stop_speed = std::max(stop_speed, std::max(rms(12.0, 12.5), rms(19.0, 19.5)));
    }
  }

  if (!std::isfinite(survival_s)) survival_s = 20.0;
  const double score =
      all_safe ? (5.0 * peak + 100.0 * tail_rms + 0.05 * drive_error + 0.05 * stop_speed)
               : std::numeric_limits<double>::infinity();
  return Candidate{rate_p,        rate_i,         angle_p,    angle_d,
                   balance_limit, velocity_p,     velocity_i, score,
                   peak,          tail_rms,       survival_s, saturation_fraction,
                   drive_error,   stop_speed, all_safe};
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
            "drive_error_sps,stop_sps,score\n";
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

bool passes_gain_neighborhood(const Candidate& candidate) {
  if (!passes_transfer_matrix(candidate)) return false;
  const auto check_scaled = [&](std::string_view name, double Candidate::*member) {
    for (double scale : {0.90, 1.10}) {
      Candidate varied = candidate;
      varied.*member *= scale;
      if (!passes_transfer_matrix(varied)) {
        std::cerr << "Gain neighborhood rejected candidate=" << candidate.rate_p << '/'
                  << candidate.angle_p << '/' << candidate.angle_d << '/'
                  << candidate.velocity_p << " gain=" << name << " x " << scale << '\n';
        return false;
      }
    }
    return true;
  };
  return check_scaled("rate_P", &Candidate::rate_p) &&
         check_scaled("angle_P", &Candidate::angle_p) &&
         check_scaled("angle_D", &Candidate::angle_d) &&
         check_scaled("velocity_P", &Candidate::velocity_p);
}

}  // namespace

int main(int argc, char** argv) {
  std::filesystem::path base = "pid.conf";
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
  for (const auto* stale_name : {"inner.csv", "outer.csv", "joint.csv", "selected_pid.conf",
                                 "joint_selected_pid.conf", "joint_selected.json",
                                 "provenance.json"}) {
    std::filesystem::remove(output_dir / stale_name);
  }

  // Stage 1 disables joystick motion, COM trim, and every rate-loop I/D/FF
  // term. A small velocity-P grid remains because completed wheel speed is a
  // required balance-state input in the velocity-actuated plant; forcing it
  // to zero has no safe release/push candidate in this gain range.
  const std::vector<double> rate_p_values{0.15, 0.20, 0.25, 0.30, 0.40};
  const std::vector<double> angle_p_values{20.0, 24.0, 28.0, 32.0, 36.0};
  const std::vector<double> angle_d_values{0.10, 0.25, 0.40, 0.60};
  const std::vector<double> balance_limits{8000.0, 10000.0, 12000.0};
  const std::vector<double> inner_velocity_p_values{0.0035, 0.0040, 0.0045};
  std::vector<Candidate> inner;
  for (double rate_p : rate_p_values) {
    for (double angle_p : angle_p_values) {
      for (double angle_d : angle_d_values) {
        for (double balance_limit : balance_limits) {
          for (double velocity_p : inner_velocity_p_values) {
            inner.push_back(evaluate(rate_p, 0.0, angle_p, angle_d, balance_limit, velocity_p,
                                     0.0, TuneStage::Inner));
          }
        }
      }
    }
  }
  rank(inner);
  write_stage_results(output_dir, "inner", inner);
  if (inner.empty() || !inner.front().safe) {
    std::cerr << "No safe inner-loop candidate found\n";
    return 1;
  }
  write_stage_selection(output_dir, "inner", inner.front());
  if (requested_stage == "inner") return 0;

  // Stage 2: keep the safe inner shortlist. Release/push scores can be tied or
  // nearly tied while their command transients differ materially, so choosing
  // one before exercising the outer loop makes the staged search brittle.
  const std::vector<double> velocity_p_values{0.0035, 0.00375, 0.0040, 0.00425, 0.0045};
  constexpr double velocity_i = 0.001;
  std::vector<Candidate> outer;
  const size_t inner_shortlist_count = std::min<size_t>(30, inner.size());
  for (size_t inner_index = 0; inner_index < inner_shortlist_count; ++inner_index) {
    const Candidate& inner_candidate = inner[inner_index];
    if (!inner_candidate.safe) continue;
    for (double velocity_p : velocity_p_values) {
      outer.push_back(evaluate(inner_candidate.rate_p, 0.0, inner_candidate.angle_p,
                               inner_candidate.angle_d, inner_candidate.balance_limit, velocity_p,
                               velocity_i, TuneStage::Outer));
    }
  }
  rank(outer);
  write_stage_results(output_dir, "outer", outer);
  if (outer.empty() || !outer.front().safe) {
    std::cerr << "No safe outer-loop candidate found\n";
    return 1;
  }
  write_stage_selection(output_dir, "outer", outer.front());
  if (requested_stage == "outer") return 0;

  // Stage 3: refine locally around the selected inner/outer combination.
  const Candidate outer_best = outer.front();
  const std::vector<double> local_rate_p{0.25, 0.26, 0.27, 0.28};
  const std::vector<double> local_angle_p{std::max(1.0, outer_best.angle_p - 4.0),
                                          outer_best.angle_p, outer_best.angle_p + 4.0};
  const std::vector<double> local_angle_d{0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40};
  const std::vector<double> local_velocity_p{
      std::max(0.00010, outer_best.velocity_p - 0.0002),
      std::max(0.00010, outer_best.velocity_p - 0.0001), outer_best.velocity_p,
      std::min(0.00800, outer_best.velocity_p + 0.0001),
      std::min(0.00800, outer_best.velocity_p + 0.0002)};
  std::vector<Candidate> joint;
  for (double rate_p : local_rate_p) {
    for (double angle_p : local_angle_p) {
      for (double angle_d : local_angle_d) {
        for (double velocity_p : local_velocity_p) {
          joint.push_back(evaluate(rate_p, 0.0, angle_p, angle_d, outer_best.balance_limit,
                                   velocity_p, velocity_i, TuneStage::Joint));
        }
      }
    }
  }
  rank(joint);
  write_stage_results(output_dir, "joint", joint);

  const size_t validation_count = std::min(top_k, joint.size());
  const Candidate* selected = nullptr;
  for (size_t index = 0; index < validation_count; ++index) {
    if (joint[index].safe && passes_gain_neighborhood(joint[index])) {
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
             << "  \"validated_scenarios\": 10,\n"
             << "  \"gain_neighborhood_percent\": 10,\n"
             << "  \"hard_safety_rejection_before_scoring\": true,\n"
             << "  \"simulation_qualified_only\": true,\n"
             << "  \"selected\": \"joint_selected.json\",\n"
             << "  \"selected_score\": " << selected->score << "\n}\n";
  std::cout << "Selected transfer-qualified candidate: " << (output_dir / "selected_pid.conf")
            << '\n';
  return 0;
}
