#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "messages/types.h"
#include "services/main/config.h"
#include "simulator/simulator_runner.h"
#include "simulator/tuner_support.h"

namespace {

// This tuner deliberately stays in the simulator support target. It changes only ConfigPid
// numeric values and uses the production controller/plant implementation for every evaluation.
// The fixed balance/drive/turn limits below are authority limits, not search variables.
struct Gains {
  double pitch_gain{};
  double pitch_rate_gain{};
  double pitch_accel_gain{};
  double drive_max_acceleration_mps2{};
  double velocity_damping_per_s{};
  double velocity_pitch_limit_deg{};
  double velocity_i{};
  double velocity_i_limit_deg{};
  double velocity_control_cutoff_hz{};
};

enum class SearchStage { Inner, Outer, Com, Joint };

struct CaseResult {
  std::string name;
  ScenarioMetrics metrics;
  bool fell = false;
  uint32_t controller_fault_flags = 0;
  uint32_t actuator_fault_count = 0;
  double tail_rms_pitch_deg = 0.0;
  double peak_command_sps = 0.0;
  double command_p95_sps = 0.0;
  double command_p99_sps = 0.0;
  bool trim_trusted = false;
  double final_trim_deg = 0.0;
  double trim_error_deg = 0.0;
  std::string quality_failures;
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
      .drive_max_acceleration_mps2 = ConfigPid::values.drive_max_acceleration_mps2,
      .velocity_damping_per_s = ConfigPid::values.velocity_damping_per_s,
      .velocity_pitch_limit_deg = ConfigPid::values.velocity_pitch_limit_deg,
      .velocity_i = ConfigPid::values.velocity_I,
      .velocity_i_limit_deg = ConfigPid::values.velocity_I_limit_deg,
      .velocity_control_cutoff_hz = ConfigPid::values.velocity_control_cutoff_hz,
  };
}

void apply_gains(const Gains& gains) {
  ConfigPidValues values = ConfigPid::values;
  values.pitch_gain = gains.pitch_gain;
  values.pitch_rate_gain = gains.pitch_rate_gain;
  values.pitch_accel_gain = gains.pitch_accel_gain;
  values.drive_max_acceleration_mps2 = gains.drive_max_acceleration_mps2;
  values.velocity_damping_per_s = gains.velocity_damping_per_s;
  values.velocity_pitch_limit_deg = gains.velocity_pitch_limit_deg;
  values.velocity_I = gains.velocity_i;
  values.velocity_I_limit_deg = gains.velocity_i_limit_deg;
  values.velocity_control_cutoff_hz = gains.velocity_control_cutoff_hz;

  // Keep the comparison surface fixed. In particular, the 16000-SPS balance ceiling is not
  // allowed to become an optimizer escape hatch.
  values.balance_max_sps = Config::max_step_rate_sps;
  values.drive_max_sps = 1200.0;
  values.turn_max_sps = 1600.0;
  ConfigPid::values = values;
}

Gains clamp_gains(Gains gains) {
  // These are deliberately much wider than the historical DirectActuator-oriented priors.
  // They remain inside the numeric configuration validator's supported range.
  gains.pitch_gain = std::clamp(gains.pitch_gain, 0.0, 1.0e6);
  gains.pitch_rate_gain = std::clamp(gains.pitch_rate_gain, 0.0, 1.0e6);
  gains.pitch_accel_gain = std::clamp(gains.pitch_accel_gain, 0.0, 1.0e6);
  gains.drive_max_acceleration_mps2 = std::clamp(gains.drive_max_acceleration_mps2, 0.0, 10.0);
  gains.velocity_damping_per_s = std::clamp(gains.velocity_damping_per_s, 0.0, 64.0);
  gains.velocity_pitch_limit_deg = std::clamp(gains.velocity_pitch_limit_deg, 0.0, 30.0);
  gains.velocity_i = std::clamp(gains.velocity_i, 0.0, 0.02);
  gains.velocity_i_limit_deg = std::clamp(gains.velocity_i_limit_deg, 0.0, 20.0);
  gains.velocity_control_cutoff_hz = std::clamp(gains.velocity_control_cutoff_hz, 0.10, 30.0);
  return gains;
}

std::string gains_key(const Gains& gains) {
  std::ostringstream output;
  output << std::setprecision(12) << gains.pitch_gain << ':' << gains.pitch_rate_gain << ':'
         << gains.pitch_accel_gain << ':' << gains.drive_max_acceleration_mps2 << ':'
         << gains.velocity_damping_per_s << ':' << gains.velocity_pitch_limit_deg << ':'
         << gains.velocity_i << ':' << gains.velocity_i_limit_deg << ':'
         << gains.velocity_control_cutoff_hz;
  return output.str();
}

void add_unique(std::vector<Gains>& destination, std::set<std::string>& seen, Gains gains) {
  gains = clamp_gains(gains);
  if (seen.insert(gains_key(gains)).second) destination.push_back(gains);
}

uint64_t next_random(uint64_t& state) {
  // Deterministic, dependency-free search sampling. Reproducibility matters more than random
  // generator sophistication for this diagnostic tuner.
  state = state * 6364136223846793005ULL + 1442695040888963407ULL;
  return state;
}

double unit_random(uint64_t& state) {
  return static_cast<double>(next_random(state) >> 11) /
         static_cast<double>(1ULL << 53);
}

double log_sample(uint64_t& state, double low, double high) {
  return std::exp(std::log(low) + unit_random(state) * (std::log(high) - std::log(low)));
}

double linear_sample(uint64_t& state, double low, double high) {
  return low + unit_random(state) * (high - low);
}

SearchStage parse_stage(std::string_view value) {
  if (value == "inner") return SearchStage::Inner;
  if (value == "outer") return SearchStage::Outer;
  if (value == "com" || value == "trim") return SearchStage::Com;
  if (value == "joint") return SearchStage::Joint;
  throw std::runtime_error("Unknown tuning stage: " + std::string(value));
}

const char* stage_name(SearchStage stage) {
  switch (stage) {
    case SearchStage::Inner: return "inner";
    case SearchStage::Outer: return "outer";
    case SearchStage::Com: return "com";
    case SearchStage::Joint: return "joint";
  }
  return "unknown";
}

std::vector<SimulatorScenario> stage_scenarios(SearchStage stage) {
  constexpr auto profile = PhysicsProfile::StepperPhaseElectrical;
  switch (stage) {
    case SearchStage::Inner:
      return tuning_inner_scenario_set(profile);
    case SearchStage::Outer:
      return tuning_velocity_scenario_set(profile);
    case SearchStage::Com:
      return tuning_trim_scenario_set(profile);
    case SearchStage::Joint: {
      auto scenarios = tuning_inner_scenario_set(profile);
      auto outer = tuning_velocity_scenario_set(profile);
      auto trim = tuning_trim_scenario_set(profile);
      scenarios.insert(scenarios.end(), outer.begin(), outer.end());
      scenarios.insert(scenarios.end(), trim.begin(), trim.end());
      return scenarios;
    }
  }
  return {};
}

std::vector<Gains> candidates_for_stage(const Gains& base, SearchStage stage, size_t budget) {
  std::vector<Gains> candidates;
  std::set<std::string> seen;
  add_unique(candidates, seen, base);

  if (stage == SearchStage::Inner || stage == SearchStage::Joint) {
    const std::vector<double> pitch = {20000.0, 50000.0, 100000.0, 180000.0,
                                       280000.0, 420000.0, 650000.0, 1000000.0};
    const std::vector<double> rate = {1000.0, 3000.0, 6000.0, 12000.0,
                                      24000.0, 40000.0, 70000.0};
    const std::vector<double> accel = {0.0, 10.0, 50.0, 200.0};
    for (double pitch_gain : pitch) {
      for (double pitch_rate_gain : rate) {
        for (double pitch_accel_gain : accel) {
          auto value = base;
          value.pitch_gain = pitch_gain;
          value.pitch_rate_gain = pitch_rate_gain;
          value.pitch_accel_gain = pitch_accel_gain;
          add_unique(candidates, seen, value);
        }
      }
    }
  }

  if (stage == SearchStage::Outer || stage == SearchStage::Joint) {
    const std::vector<double> acceleration = {0.25, 0.50, 0.75, 1.00, 1.25, 1.75, 2.50, 4.00};
    const std::vector<double> damping = {0.25, 0.50, 1.00, 2.00, 3.00, 4.00,
                                         6.00, 8.00, 12.00, 16.00, 24.00, 40.00};
    const std::vector<double> pitch_limit = {1.0, 1.5, 2.0, 3.0, 4.0, 5.0, 6.0, 10.0};
    const std::vector<double> cutoff = {0.25, 0.50, 1.0, 2.0, 3.0, 5.0, 8.0, 15.0};
    // A diagonal coarse grid covers interactions without spending the entire budget on a
    // Cartesian product. The random tail below fills the cross terms uniformly in log space.
    for (size_t index = 0; index < damping.size(); ++index) {
      auto value = base;
      value.velocity_damping_per_s = damping[index];
      value.velocity_pitch_limit_deg = pitch_limit[index % pitch_limit.size()];
      value.drive_max_acceleration_mps2 = acceleration[index % acceleration.size()];
      value.velocity_control_cutoff_hz = cutoff[index % cutoff.size()];
      add_unique(candidates, seen, value);
    }
    for (double value : damping) {
      auto candidate = base;
      candidate.velocity_damping_per_s = value;
      add_unique(candidates, seen, candidate);
    }
    for (double value : pitch_limit) {
      auto candidate = base;
      candidate.velocity_pitch_limit_deg = value;
      add_unique(candidates, seen, candidate);
    }
    for (double value : acceleration) {
      auto candidate = base;
      candidate.drive_max_acceleration_mps2 = value;
      add_unique(candidates, seen, candidate);
    }
    for (double value : cutoff) {
      auto candidate = base;
      candidate.velocity_control_cutoff_hz = value;
      add_unique(candidates, seen, candidate);
    }
  }

  if (stage == SearchStage::Com || stage == SearchStage::Joint) {
    const std::vector<double> trim_gain = {0.0, 0.00005, 0.0001, 0.00025,
                                           0.0005, 0.001, 0.002, 0.004, 0.008, 0.016};
    const std::vector<double> trim_limit = {0.5, 1.0, 2.0, 4.0, 6.0, 8.0, 12.0, 20.0};
    for (double gain : trim_gain) {
      for (double limit : trim_limit) {
        auto value = base;
        value.velocity_i = gain;
        value.velocity_i_limit_deg = limit;
        add_unique(candidates, seen, value);
      }
    }
  }

  uint64_t state = 0x5eed5eed12345678ULL + static_cast<uint64_t>(stage) * 0x9e3779b9ULL;
  while (candidates.size() < budget) {
    auto value = base;
    if (stage == SearchStage::Inner || stage == SearchStage::Joint) {
      value.pitch_gain = log_sample(state, 20000.0, 1.0e6);
      value.pitch_rate_gain = log_sample(state, 1000.0, 70000.0);
      value.pitch_accel_gain = linear_sample(state, 0.0, 500.0);
    }
    if (stage == SearchStage::Outer || stage == SearchStage::Joint) {
      value.drive_max_acceleration_mps2 = log_sample(state, 0.25, 4.0);
      value.velocity_damping_per_s = log_sample(state, 0.25, 40.0);
      value.velocity_pitch_limit_deg = log_sample(state, 0.5, 12.0);
      value.velocity_control_cutoff_hz = log_sample(state, 0.25, 15.0);
    }
    if (stage == SearchStage::Com || stage == SearchStage::Joint) {
      value.velocity_i = unit_random(state) < 0.08 ? 0.0 : log_sample(state, 0.00002, 0.02);
      value.velocity_i_limit_deg = log_sample(state, 0.5, 20.0);
    }
    add_unique(candidates, seen, value);
  }
  if (candidates.size() > budget) candidates.resize(budget);
  return candidates;
}

double percentile(std::vector<double> values, double fraction) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const double position = fraction * static_cast<double>(values.size() - 1);
  const size_t lower = static_cast<size_t>(std::floor(position));
  const size_t upper = std::min(values.size() - 1, lower + 1);
  const double weight = position - static_cast<double>(lower);
  return values[lower] * (1.0 - weight) + values[upper] * weight;
}

std::string join_failures(const std::vector<std::string>& failures) {
  std::ostringstream output;
  for (size_t index = 0; index < failures.size(); ++index) {
    if (index != 0) output << ';';
    output << failures[index];
  }
  return output.str();
}

bool case_is_good(SearchStage stage, const SimulatorRunResult& result,
                  const ScenarioMetrics& metrics, std::vector<std::string>& failures) {
  const auto reject = [&](bool condition, const char* reason) {
    if (condition) failures.emplace_back(reason);
  };
  reject(result.fell, "fell");
  reject(result.controller_fault_flags != ControllerFaultNone, "controller_fault");
  reject(result.actuator_fault_count != 0, "actuator_fault");
  reject(!std::isfinite(metrics.peak_pitch_deg) || !std::isfinite(metrics.peak_rate_dps),
         "non_finite");
  reject(metrics.peak_pitch_deg >= 35.0, "peak_pitch");
  reject(metrics.peak_rate_dps >= 450.0, "peak_rate");
  reject(metrics.growing_oscillation, "growing_oscillation");
  reject(metrics.max_continuous_saturation_s >= 1.0, "continuous_saturation");
  if (stage == SearchStage::Inner || stage == SearchStage::Joint) {
    reject(result.tail_rms_pitch_deg >= 2.0, "inner_tail_rms");
  }
  if (stage == SearchStage::Outer || stage == SearchStage::Joint) {
    reject(std::abs(metrics.final_velocity_mean_sps) >= 400.0, "residual_velocity");
    reject(metrics.stop_speed_rms_sps >= 500.0, "stop_speed");
  }
  if ((stage == SearchStage::Com || stage == SearchStage::Joint) &&
      std::abs(result.scenario.com_angle_offset_rad) > 1.0e-12) {
    reject(std::abs(metrics.final_velocity_mean_sps) >= 250.0, "trim_residual_velocity");
    reject(result.rows.empty() || result.rows.back().trim_trusted < 0.5, "trim_not_trusted");
    const double expected_trim_deg =
        -result.scenario.com_angle_offset_rad * 180.0 / 3.14159265358979323846;
    const double final_trim_deg = result.rows.empty() ? 0.0 : result.rows.back().com_trim_deg;
    reject(std::abs(final_trim_deg - expected_trim_deg) > 0.08, "trim_error");
  }
  return failures.empty();
}

double case_cost(SearchStage stage, const SimulatorRunResult& result,
                 const ScenarioMetrics& metrics) {
  double cost = 0.0;
  if (result.fell) cost += 1.0e7;
  if (result.actuator_fault_count != 0) cost += 1.0e6;
  if (result.controller_fault_flags != ControllerFaultNone) cost += 1.0e6;
  if (!std::isfinite(metrics.peak_pitch_deg) || !std::isfinite(metrics.peak_rate_dps)) {
    return 1.0e12;
  }

  // Stability and oscillation dominate. These terms deliberately remain continuous when a
  // candidate is just on the wrong side of a behavioral gate.
  cost += 2.0 * metrics.peak_pitch_deg;
  cost += 0.08 * metrics.peak_rate_dps;
  cost += 120.0 * metrics.pitch_iae_deg_s;
  cost += 220.0 * result.tail_rms_pitch_deg;
  cost += 0.02 * metrics.command_rms_sps;
  cost += 0.0005 * metrics.command_total_variation_sps;
  cost += 25.0 * metrics.saturation_time_s;
  cost += 40.0 * metrics.max_continuous_saturation_s;
  if (metrics.growing_oscillation) cost += 4000.0;

  if (stage == SearchStage::Outer || stage == SearchStage::Joint) {
    cost += 0.80 * metrics.velocity_iae_sps_s;
    cost += 0.20 * std::abs(metrics.final_velocity_mean_sps);
    cost += 0.40 * metrics.drive_tracking_mae_sps;
    cost += 0.20 * metrics.stop_speed_rms_sps;
    if (!metrics.settled) cost += 250.0;
  }
  if ((stage == SearchStage::Com || stage == SearchStage::Joint) &&
      std::abs(result.scenario.com_angle_offset_rad) > 1.0e-12) {
    cost += 0.10 * std::abs(metrics.final_velocity_mean_sps);
    const double expected_trim_deg =
        -result.scenario.com_angle_offset_rad * 180.0 / 3.14159265358979323846;
    const double final_trim_deg = result.rows.empty() ? 0.0 : result.rows.back().com_trim_deg;
    const double trim_error_deg = std::abs(final_trim_deg - expected_trim_deg);
    cost += 100.0 * trim_error_deg;
    if (result.rows.empty() || result.rows.back().trim_trusted < 0.5) cost += 10000.0;
  }
  return std::isfinite(cost) ? cost : 1.0e12;
}

Candidate evaluate_candidate(size_t id, size_t round, const Gains& gains, SearchStage stage,
                             const std::vector<SimulatorScenario>& scenarios) {
  apply_gains(gains);
  Candidate candidate;
  candidate.id = id;
  candidate.round = round;
  candidate.gains = gains;
  candidate.score = 0.0;
  for (const auto& scenario : scenarios) {
    const auto result = run_simulator_scenario_with_loaded_pid(scenario);
    const auto metrics = calculate_tuning_metrics(result);
    std::vector<double> commands;
    commands.reserve(result.rows.size());
    for (const auto& row : result.rows) commands.push_back(std::abs(row.u_sps));
    const double peak_command = commands.empty() ? 0.0
                                                  : *std::max_element(commands.begin(), commands.end());
    const bool trim_trusted = !result.rows.empty() && result.rows.back().trim_trusted > 0.5;
    const double final_trim_deg = result.rows.empty() ? 0.0 : result.rows.back().com_trim_deg;
    const double expected_trim_deg =
        -scenario.com_angle_offset_rad * 180.0 / 3.14159265358979323846;
    std::vector<std::string> failures;
    const bool good = case_is_good(stage, result, metrics, failures);
    candidate.score += case_cost(stage, result, metrics);
    candidate.passed_cases += good ? 1u : 0u;
    candidate.hard_failures += result.fell ? 1u : 0u;
    candidate.hard_failures += result.actuator_fault_count != 0 ? 1u : 0u;
    candidate.hard_failures += result.controller_fault_flags != ControllerFaultNone ? 1u : 0u;
    candidate.cases.push_back({
        .name = scenario.name,
        .metrics = metrics,
        .fell = result.fell,
        .controller_fault_flags = result.controller_fault_flags,
        .actuator_fault_count = result.actuator_fault_count,
        .tail_rms_pitch_deg = result.tail_rms_pitch_deg,
        .peak_command_sps = peak_command,
        .command_p95_sps = percentile(commands, 0.95),
        .command_p99_sps = percentile(commands, 0.99),
        .trim_trusted = trim_trusted,
        .final_trim_deg = final_trim_deg,
        .trim_error_deg = std::abs(final_trim_deg - expected_trim_deg),
        .quality_failures = join_failures(failures),
    });
  }
  return candidate;
}

bool candidate_less(const Candidate& left, const Candidate& right) {
  if (left.hard_failures != right.hard_failures) return left.hard_failures < right.hard_failures;
  if (left.passed_cases != right.passed_cases) return left.passed_cases > right.passed_cases;
  if (left.score != right.score) return left.score < right.score;
  return left.id < right.id;
}

void rank(std::vector<Candidate>& candidates) {
  std::sort(candidates.begin(), candidates.end(), candidate_less);
}

void write_bounds(const std::filesystem::path& output_dir, SearchStage stage, size_t budget) {
  std::ofstream output(output_dir / "search_bounds.txt");
  output << "stage=" << stage_name(stage) << "\nprofile=StepperPhaseElectrical\n";
  output << "budget=" << budget << "\n";
  output << "pitch_gain=20000..1000000 (log coarse/random; validator max 1000000)\n";
  output << "pitch_rate_gain=1000..70000 (log coarse/random; validator max 1000000)\n";
  output << "pitch_accel_gain=0..500 (linear coarse/random; validator max 1000000)\n";
  output << "drive_max_acceleration_mps2=0.25..4 (log coarse/random)\n";
  output << "velocity_damping_per_s=0.25..40 (log coarse/random)\n";
  output << "velocity_pitch_limit_deg=0.5..12 (log coarse/random; validator max 90)\n";
  output << "velocity_I=0..0.02 (zero plus log coarse/random)\n";
  output << "velocity_I_limit_deg=0.5..20 (log coarse/random)\n";
  output << "velocity_control_cutoff_hz=0.25..15 (log coarse/random)\n";
  output << "balance_max_sps=16000 (fixed)\ndrive_max_sps=1200 (fixed)\nturn_max_sps=1600 (fixed)\n";
}

void write_artifacts(const std::filesystem::path& output_dir, const std::vector<Candidate>& candidates) {
  std::ofstream summary(output_dir / "candidate_summary.csv");
  summary << "rank,candidate_id,round,pitch_gain,pitch_rate_gain,pitch_accel_gain,"
             "drive_max_acceleration_mps2,velocity_damping_per_s,velocity_pitch_limit_deg,"
             "velocity_I,velocity_I_limit_deg,velocity_control_cutoff_hz,passed_cases,"
             "hard_failures,score\n";
  std::ofstream validation(output_dir / "validation_results.csv");
  validation << "rank,candidate_id,scenario,accepted,quality_failures,fell,controller_fault_flags,"
                "actuator_fault_count,peak_command_sps,command_p95_sps,command_p99_sps,"
                "peak_pitch_deg,peak_rate_dps,tail_rms_pitch_deg,saturation_time_s,"
                "max_continuous_saturation_s,final_velocity_mean_sps,trim_trusted,"
                "final_trim_deg,trim_error_deg\n";
  std::ofstream metrics(output_dir / "scenario_metrics.csv");
  metrics << "rank,candidate_id,scenario," << tuning_metrics_csv_header() << '\n';
  for (size_t rank_index = 0; rank_index < candidates.size(); ++rank_index) {
    const auto& candidate = candidates[rank_index];
    const auto& gains = candidate.gains;
    summary << rank_index + 1 << ',' << candidate.id << ',' << candidate.round << ','
            << std::setprecision(12) << gains.pitch_gain << ',' << gains.pitch_rate_gain << ','
            << gains.pitch_accel_gain << ',' << gains.drive_max_acceleration_mps2 << ','
            << gains.velocity_damping_per_s << ',' << gains.velocity_pitch_limit_deg << ','
            << gains.velocity_i << ',' << gains.velocity_i_limit_deg << ','
            << gains.velocity_control_cutoff_hz << ',' << candidate.passed_cases << ','
            << candidate.hard_failures << ',' << candidate.score << '\n';
    for (const auto& result : candidate.cases) {
      validation << rank_index + 1 << ',' << candidate.id << ',' << result.name << ','
                 << result.quality_failures.empty() << ',' << result.quality_failures << ','
                 << result.fell << ',' << result.controller_fault_flags << ','
                 << result.actuator_fault_count << ',' << result.peak_command_sps << ','
                 << result.command_p95_sps << ',' << result.command_p99_sps << ','
                 << result.metrics.peak_pitch_deg << ',' << result.metrics.peak_rate_dps << ','
                 << result.tail_rms_pitch_deg << ',' << result.metrics.saturation_time_s << ','
                 << result.metrics.max_continuous_saturation_s << ','
                 << result.metrics.final_velocity_mean_sps << ',' << result.trim_trusted << ','
                 << result.final_trim_deg << ',' << result.trim_error_deg << '\n';
      metrics << rank_index + 1 << ',' << candidate.id << ',' << result.name << ','
              << tuning_metrics_csv_row(result.metrics) << '\n';
    }
  }
}

void write_best_config(const std::filesystem::path& output_dir, const Candidate& candidate,
                       size_t case_count) {
  apply_gains(candidate.gains);
  ConfigPid::save((output_dir / "best_observed.pid.conf").string());
  std::ofstream note(output_dir / "best_observed.txt");
  note << "StepperPhaseElectrical simulator tuning candidate; not hardware authorization.\n"
       << "proxy cases passed: " << candidate.passed_cases << '/' << case_count << '\n'
       << "hard failures: " << candidate.hard_failures << '\n'
       << "continuous score: " << candidate.score << '\n';
}

void usage() {
  std::cout << "Usage: balancer_simulator_tuner --stage inner|outer|com|joint "
               "[--base FILE] [--output DIR] [--budget N]\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::filesystem::path base = "tests/data/stepper_phase_electrical_pid.conf";
  std::filesystem::path output_dir = "build/sim/stepper_tuning";
  SearchStage stage = SearchStage::Inner;
  size_t budget = 600;
  for (int index = 1; index < argc; ++index) {
    const std::string arg = argv[index];
    if (arg == "--base" && index + 1 < argc) {
      base = argv[++index];
    } else if (arg == "--output" && index + 1 < argc) {
      output_dir = argv[++index];
    } else if (arg == "--stage" && index + 1 < argc) {
      stage = parse_stage(argv[++index]);
    } else if (arg == "--budget" && index + 1 < argc) {
      budget = std::stoul(argv[++index]);
    } else if (arg == "--help") {
      usage();
      return 0;
    } else {
      std::cerr << "Unknown or incomplete argument: " << arg << '\n';
      return 2;
    }
  }
  if (budget < 20) {
    std::cerr << "--budget must be at least 20\n";
    return 2;
  }

  ConfigPid::load(base.string());
  const Gains base_gains = loaded_gains();
  const auto scenarios = stage_scenarios(stage);
  std::filesystem::create_directories(output_dir);
  for (const auto* stale : {"best_observed.pid.conf", "best_observed.txt", "candidate_summary.csv",
                            "scenario_metrics.csv", "search_bounds.txt", "validation_results.csv"}) {
    std::filesystem::remove(output_dir / stale);
  }
  write_bounds(output_dir, stage, budget);

  const auto gains = candidates_for_stage(base_gains, stage, budget);
  std::vector<Candidate> candidates;
  candidates.reserve(gains.size());
  for (size_t index = 0; index < gains.size(); ++index) {
    candidates.push_back(evaluate_candidate(index + 1, 0, gains[index], stage, scenarios));
  }
  rank(candidates);
  write_artifacts(output_dir, candidates);
  write_best_config(output_dir, candidates.front(), scenarios.size());

  const auto& best = candidates.front();
  std::cout << "Stage " << stage_name(stage) << " evaluated " << candidates.size()
            << " candidates over " << scenarios.size() << " StepperPhaseElectrical scenarios.\n"
            << "Best proxy candidate " << best.id << " passed " << best.passed_cases << '/'
            << scenarios.size() << " with " << best.hard_failures << " hard failures; score "
            << best.score << ". Artifacts: " << output_dir << '\n';
  return 0;
}
