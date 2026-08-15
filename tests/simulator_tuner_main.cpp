#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "messages/types.h"
#include "simulator/simulator_runner.h"
#include "simulator/tuner_support.h"

namespace {

struct Gains {
  double pitch_gain{};
  double pitch_rate_gain{};
  double pitch_accel_gain{};
  double velocity_damping_per_s{};
  double drive_max_acceleration_mps2{};
  double velocity_i{};
  double velocity_control_cutoff_hz{};
};

struct CaseResult {
  std::string name;
  ScenarioMetrics metrics;
  TransferAcceptance acceptance;
  bool fell = false;
  double peak_pitch_deg = 0.0;
  double tail_rms_pitch_deg = 0.0;
  uint32_t controller_fault_flags = 0;
  uint32_t actuator_fault_count = 0;
};

struct Candidate {
  size_t id{};
  size_t round{};
  Gains gains{};
  std::vector<CaseResult> cases;
  size_t passed_cases = 0;
  size_t hard_failures = 0;
  double score = std::numeric_limits<double>::infinity();
};

Gains loaded_gains() {
  return {
      .pitch_gain = ConfigPid::values.pitch_gain,
      .pitch_rate_gain = ConfigPid::values.pitch_rate_gain,
      .pitch_accel_gain = ConfigPid::values.pitch_accel_gain,
      .velocity_damping_per_s = ConfigPid::values.velocity_damping_per_s,
      .drive_max_acceleration_mps2 = ConfigPid::values.drive_max_acceleration_mps2,
      .velocity_i = ConfigPid::values.velocity_I,
      .velocity_control_cutoff_hz = ConfigPid::values.velocity_control_cutoff_hz,
  };
}

void apply_gains(const Gains& gains) {
  ConfigPidValues values = ConfigPid::values;
  values.pitch_gain = gains.pitch_gain;
  values.pitch_rate_gain = gains.pitch_rate_gain;
  values.pitch_accel_gain = gains.pitch_accel_gain;
  values.velocity_damping_per_s = gains.velocity_damping_per_s;
  values.drive_max_acceleration_mps2 = gains.drive_max_acceleration_mps2;
  values.velocity_I = gains.velocity_i;
  values.velocity_I_limit_deg = 4.0;
  values.balance_max_sps = 8000.0;
  values.drive_max_sps = 1200.0;
  values.turn_max_sps = 1600.0;
  values.velocity_control_cutoff_hz = gains.velocity_control_cutoff_hz;
  ConfigPid::values = values;
}

double case_cost(const SimulatorRunResult& result, const TransferAcceptance& acceptance,
                 const ScenarioMetrics& metrics) {
  // Acceptance failures are primary, but the continuous quantities make the
  // ordering useful when every candidate fails at least one hard gate.
  double cost = 250.0 * static_cast<double>(acceptance.failures.size());
  cost += 4.0 * result.max_abs_pitch_deg;
  cost += 80.0 * result.tail_rms_pitch_deg;
  cost += 20.0 * result.max_continuous_saturation_s;
  cost += 0.002 * metrics.command_total_variation_sps;
  cost += 0.03 * std::abs(metrics.final_velocity_mean_sps);
  if (result.fell) cost += 100000.0;
  if (result.actuator_fault_count != 0) cost += 25000.0;
  if (result.controller_fault_flags != ControllerFaultNone) cost += 25000.0;
  if (!std::isfinite(cost)) return 1e12;
  return cost;
}

Candidate evaluate_candidate(size_t id, size_t round, const Gains& gains,
                             const std::vector<SimulatorScenario>& scenarios) {
  apply_gains(gains);
  Candidate candidate;
  candidate.id = id;
  candidate.round = round;
  candidate.gains = gains;
  candidate.score = 0.0;
  for (const auto& scenario : scenarios) {
    const auto result = run_simulator_scenario_with_loaded_pid(scenario);
    const auto acceptance = evaluate_transfer_scenario(result);
    const auto metrics = calculate_tuning_metrics(result);
    candidate.score += case_cost(result, acceptance, metrics);
    candidate.passed_cases += acceptance.accepted ? 1u : 0u;
    candidate.hard_failures += result.fell ? 1u : 0u;
    candidate.hard_failures += result.actuator_fault_count != 0 ? 1u : 0u;
    candidate.hard_failures += result.controller_fault_flags != ControllerFaultNone ? 1u : 0u;
    candidate.cases.push_back({
        .name = scenario.name,
        .metrics = metrics,
        .acceptance = acceptance,
        .fell = result.fell,
        .peak_pitch_deg = result.max_abs_pitch_deg,
        .tail_rms_pitch_deg = result.tail_rms_pitch_deg,
        .controller_fault_flags = result.controller_fault_flags,
        .actuator_fault_count = result.actuator_fault_count,
    });
  }
  return candidate;
}

bool candidate_less(const Candidate& left, const Candidate& right) {
  if (left.passed_cases != right.passed_cases) return left.passed_cases > right.passed_cases;
  if (left.hard_failures != right.hard_failures) return left.hard_failures < right.hard_failures;
  if (left.score != right.score) return left.score < right.score;
  return left.id < right.id;
}

void rank(std::vector<Candidate>& candidates) {
  std::sort(candidates.begin(), candidates.end(), candidate_less);
}

std::string gains_key(const Gains& gains) {
  std::ostringstream output;
  output << std::setprecision(12) << gains.pitch_gain << ':' << gains.pitch_rate_gain << ':'
         << gains.pitch_accel_gain << ':' << gains.velocity_damping_per_s << ':'
         << gains.drive_max_acceleration_mps2 << ':'
         << gains.velocity_i << ':' << gains.velocity_control_cutoff_hz;
  return output.str();
}

Gains clamp_gains(Gains gains) {
  gains.pitch_gain = std::clamp(gains.pitch_gain, 0.0, 200000.0);
  gains.pitch_rate_gain = std::clamp(gains.pitch_rate_gain, 0.0, 5000.0);
  gains.pitch_accel_gain = std::clamp(gains.pitch_accel_gain, 0.0, 200.0);
  gains.velocity_damping_per_s = std::clamp(gains.velocity_damping_per_s, 0.0, 16.0);
  gains.drive_max_acceleration_mps2 = std::clamp(gains.drive_max_acceleration_mps2, 0.50, 3.0);
  gains.velocity_i = std::clamp(gains.velocity_i, 0.0, 0.005);
  gains.velocity_control_cutoff_hz = std::clamp(gains.velocity_control_cutoff_hz, 0.5, 10.0);
  return gains;
}

enum class Field {
  PitchGain,
  PitchRateGain,
  PitchAccelGain,
  Damping,
  Accel,
  VelocityI,
  VelocityCutoff
};

struct Move {
  Field field;
  double delta;
};

Gains apply_move(Gains gains, const Move& move) {
  switch (move.field) {
    case Field::PitchGain: gains.pitch_gain += move.delta; break;
    case Field::PitchRateGain: gains.pitch_rate_gain += move.delta; break;
    case Field::PitchAccelGain: gains.pitch_accel_gain += move.delta; break;
    case Field::Damping: gains.velocity_damping_per_s += move.delta; break;
    case Field::Accel: gains.drive_max_acceleration_mps2 += move.delta; break;
    case Field::VelocityI: gains.velocity_i += move.delta; break;
    case Field::VelocityCutoff: gains.velocity_control_cutoff_hz += move.delta; break;
  }
  return clamp_gains(gains);
}

void add_unique(std::vector<Gains>& destination, std::set<std::string>& seen, Gains gains) {
  gains = clamp_gains(gains);
  if (seen.insert(gains_key(gains)).second) destination.push_back(gains);
}

std::vector<Gains> coarse_candidates(const Gains& base) {
  std::vector<Gains> candidates;
  std::set<std::string> seen;
  add_unique(candidates, seen, base);
  for (double value : {4000.0, 6000.0, 8000.0, 10000.0, 14000.0, 20000.0}) {
    auto gains = base;
    gains.pitch_gain = value;
    add_unique(candidates, seen, gains);
  }
  for (double value : {250.0, 400.0, 500.0, 700.0, 1000.0, 1400.0}) {
    auto gains = base;
    gains.pitch_rate_gain = value;
    add_unique(candidates, seen, gains);
  }
  for (double value : {0.0, 2.0, 4.0, 8.0, 16.0, 32.0}) {
    auto gains = base;
    gains.pitch_accel_gain = value;
    add_unique(candidates, seen, gains);
  }
  for (double value : {0.0, 2.0, 3.0, 5.0, 7.0, 8.0, 10.0, 12.0, 15.0}) {
    auto gains = base;
    gains.velocity_damping_per_s = value;
    add_unique(candidates, seen, gains);
  }
  for (double value : {0.50, 0.75, 1.0, 1.5, 2.0, 2.5}) {
    auto gains = base;
    gains.drive_max_acceleration_mps2 = value;
    add_unique(candidates, seen, gains);
  }
  for (double value : {0.0, 0.0005, 0.0010, 0.0015, 0.0020, 0.0030}) {
    auto gains = base;
    gains.velocity_i = value;
    add_unique(candidates, seen, gains);
  }
  for (double value : {1.0, 2.0, 3.0, 5.0}) {
    auto gains = base;
    gains.velocity_control_cutoff_hz = value;
    add_unique(candidates, seen, gains);
  }
  return candidates;
}

std::vector<Gains> local_candidates(const std::vector<Candidate>& seeds, size_t seed_count,
                                    size_t limit, bool fine) {
  const double scale = fine ? 0.5 : 1.0;
  const std::vector<Move> moves = {
      {Field::PitchGain, -1000.0 * scale}, {Field::PitchGain, 1000.0 * scale},
      {Field::PitchRateGain, -50.0 * scale}, {Field::PitchRateGain, 50.0 * scale},
      {Field::PitchAccelGain, -4.0 * scale}, {Field::PitchAccelGain, 4.0 * scale},
      {Field::Damping, -2.0 * scale}, {Field::Damping, 2.0 * scale},
      {Field::Accel, -0.25 * scale}, {Field::Accel, 0.25 * scale},
      {Field::VelocityI, -0.0005 * scale}, {Field::VelocityI, 0.0005 * scale},
      {Field::VelocityCutoff, -0.5 * scale}, {Field::VelocityCutoff, 0.5 * scale},
  };
  std::vector<Gains> candidates;
  std::set<std::string> seen;
  for (size_t seed_index = 0; seed_index < std::min(seed_count, seeds.size()); ++seed_index) {
    const auto& seed = seeds[seed_index].gains;
    add_unique(candidates, seen, seed);
    for (const auto& move : moves) add_unique(candidates, seen, apply_move(seed, move));
    for (size_t first = 0; first < moves.size() && candidates.size() < limit; ++first) {
      for (size_t second = first + 1; second < moves.size() && candidates.size() < limit; ++second) {
        add_unique(candidates, seen, apply_move(apply_move(seed, moves[first]), moves[second]));
      }
    }
    if (candidates.size() >= limit) break;
  }
  if (candidates.size() > limit) candidates.resize(limit);
  return candidates;
}

std::string failures(const TransferAcceptance& acceptance) {
  std::ostringstream output;
  for (size_t index = 0; index < acceptance.failures.size(); ++index) {
    if (index != 0) output << ';';
    output << acceptance.failures[index];
  }
  return output.str();
}

void write_artifacts(const std::filesystem::path& output_dir, const std::vector<Candidate>& candidates) {
  std::ofstream summary(output_dir / "candidate_summary.csv");
  summary << "rank,candidate_id,round,pitch_gain,pitch_rate_gain,pitch_accel_gain,"
             "velocity_damping_per_s,drive_max_acceleration_mps2,velocity_I,velocity_control_cutoff_hz,passed_cases,"
             "hard_failures,score\n";
  std::ofstream validation(output_dir / "validation_results.csv");
  validation << "rank,candidate_id,scenario,accepted,failures,peak_pitch_deg,tail_rms_pitch_deg,"
                "fell,controller_fault_flags,actuator_fault_count\n";
  std::ofstream metrics(output_dir / "scenario_metrics.csv");
  metrics << "rank,candidate_id,scenario," << tuning_metrics_csv_header() << '\n';
  for (size_t rank_index = 0; rank_index < candidates.size(); ++rank_index) {
    const auto& candidate = candidates[rank_index];
    const auto& gains = candidate.gains;
    summary << rank_index + 1 << ',' << candidate.id << ',' << candidate.round << ','
            << std::setprecision(12) << gains.pitch_gain << ',' << gains.pitch_rate_gain << ','
            << gains.pitch_accel_gain << ','
            << gains.velocity_damping_per_s << ',' << gains.drive_max_acceleration_mps2 << ','
            << gains.velocity_i << ',' << gains.velocity_control_cutoff_hz << ','
            << candidate.passed_cases << ',' << candidate.hard_failures << ','
            << candidate.score << '\n';
    for (const auto& result : candidate.cases) {
      validation << rank_index + 1 << ',' << candidate.id << ',' << result.name << ','
                 << result.acceptance.accepted << ',' << failures(result.acceptance) << ','
                 << result.peak_pitch_deg << ',' << result.tail_rms_pitch_deg << ',' << result.fell << ','
                 << result.controller_fault_flags << ',' << result.actuator_fault_count << '\n';
      metrics << rank_index + 1 << ',' << candidate.id << ',' << result.name << ','
              << tuning_metrics_csv_row(result.metrics) << '\n';
    }
  }
}

void write_best_observed(const std::filesystem::path& output_dir, const Candidate& candidate,
                         size_t case_count) {
  apply_gains(candidate.gains);
  ConfigPid::save((output_dir / "best_observed.pid.conf").string());
  std::ofstream note(output_dir / "best_observed.txt");
  note << "This is the best simulator-observed candidate, not an authorization to apply it to hardware.\n"
       << "transfer cases passed: " << candidate.passed_cases << '/' << case_count << '\n'
       << "hard failures: " << candidate.hard_failures << '\n'
       << "score: " << candidate.score << '\n';
  if (candidate.passed_cases == case_count) {
    ConfigPid::save((output_dir / "transfer_qualified.pid.conf").string());
  }
}

void usage() {
  std::cout << "Usage: balancer_simulator_tuner [--base FILE] [--output DIR] "
               "[--top-k N] [--budget N]\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::filesystem::path base = "pid.conf";
  std::filesystem::path output_dir = "build/sim/tuning";
  size_t top_k = 12;
  size_t budget = 350;
  for (int index = 1; index < argc; ++index) {
    const std::string arg = argv[index];
    if (arg == "--base" && index + 1 < argc) base = argv[++index];
    else if (arg == "--output" && index + 1 < argc) output_dir = argv[++index];
    else if (arg == "--top-k" && index + 1 < argc) top_k = std::stoul(argv[++index]);
    else if (arg == "--budget" && index + 1 < argc) budget = std::stoul(argv[++index]);
    else if (arg == "--help") { usage(); return 0; }
    else { std::cerr << "Unknown or incomplete argument: " << arg << '\n'; return 2; }
  }
  if (top_k == 0 || budget < 20) {
    std::cerr << "--top-k must be positive and --budget must be at least 20\n";
    return 2;
  }

  ConfigPid::load(base.string());
  const Gains base_gains = loaded_gains();
  const auto scenarios = transfer_scenario_set();
  std::filesystem::create_directories(output_dir);
  for (const auto* stale : {"selected_pid.conf", "best_observed.pid.conf", "best_observed.txt",
                            "transfer_qualified.pid.conf", "candidate_summary.csv",
                            "scenario_metrics.csv", "validation_results.csv"}) {
    std::filesystem::remove(output_dir / stale);
  }

  std::vector<Candidate> all;
  size_t next_id = 1;
  const auto evaluate = [&](const std::vector<Gains>& gains, size_t round) {
    for (const auto& value : gains) all.push_back(evaluate_candidate(next_id++, round, value, scenarios));
    rank(all);
  };

  evaluate(coarse_candidates(base_gains), 0);
  const size_t first_limit = std::min(budget, static_cast<size_t>(220));
  evaluate(local_candidates(all, top_k, first_limit, false), 1);
  rank(all);
  if (all.size() < budget) {
    evaluate(local_candidates(all, std::min(top_k, static_cast<size_t>(8)), budget - all.size(), true), 2);
  }
  rank(all);
  write_artifacts(output_dir, all);
  write_best_observed(output_dir, all.front(), scenarios.size());

  const auto& best = all.front();
  std::cout << "Best observed candidate " << best.id << " passed " << best.passed_cases << '/'
            << scenarios.size() << " transfer cases with " << best.hard_failures
            << " hard failures; artifacts written to " << output_dir << '\n';
  if (best.passed_cases != scenarios.size()) {
    std::cout << "No transfer-qualified PID was found. best_observed.pid.conf is simulation-only.\n";
  }
  return 0;
}
