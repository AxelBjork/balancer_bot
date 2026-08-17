#include <algorithm>
#include <array>
#include <cctype>
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

// This tuner deliberately stays in the simulator support target. It changes
// only the new translational ConfigPid values and uses the production
// controller/plant implementation for every evaluation. Inner attitude gains,
// adaptive COM trim, and the fixed balance/turn ceilings are not search
// variables; motion pitch authority is deliberately searched.
struct Gains {
  double drive_max_velocity_mps{};
  double planner_max_acceleration_mps2{};
  double planner_max_deceleration_mps2{};
  double planner_max_jerk_mps3{};
  double outer_pitch_limit_deg{};
  double velocity_feedback_cutoff_hz{};
  double velocity_gain_per_s{};
  double velocity_i_gain_per_s2{};
  double velocity_i_leak_time_s{};
  double velocity_i_acceleration_limit_mps2{};
};

enum class SearchStage {
  Feedforward,
  Feedback,
  Integral,
  Observer,
  Joint,
  Motion,
  Boundary,
  MotionIntegral,
};

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
      .drive_max_velocity_mps = ConfigPid::values.drive_max_velocity_mps,
      .planner_max_acceleration_mps2 = ConfigPid::values.planner_max_acceleration_mps2,
      .planner_max_deceleration_mps2 = ConfigPid::values.planner_max_deceleration_mps2,
      .planner_max_jerk_mps3 = ConfigPid::values.planner_max_jerk_mps3,
      .outer_pitch_limit_deg = ConfigPid::values.outer_pitch_limit_deg,
      .velocity_feedback_cutoff_hz = ConfigPid::values.velocity_feedback_cutoff_hz,
      .velocity_gain_per_s = ConfigPid::values.velocity_gain_per_s,
      .velocity_i_gain_per_s2 = ConfigPid::values.velocity_i_gain_per_s2,
      .velocity_i_leak_time_s = ConfigPid::values.velocity_i_leak_time_s,
      .velocity_i_acceleration_limit_mps2 =
          ConfigPid::values.velocity_i_acceleration_limit_mps2,
  };
}

void apply_gains(const Gains& gains) {
  ConfigPidValues values = ConfigPid::values;
  values.drive_max_velocity_mps = gains.drive_max_velocity_mps;
  values.planner_max_acceleration_mps2 = gains.planner_max_acceleration_mps2;
  values.planner_max_deceleration_mps2 = gains.planner_max_deceleration_mps2;
  values.planner_max_jerk_mps3 = gains.planner_max_jerk_mps3;
  values.outer_pitch_limit_deg = gains.outer_pitch_limit_deg;
  values.velocity_feedback_cutoff_hz = gains.velocity_feedback_cutoff_hz;
  values.velocity_gain_per_s = gains.velocity_gain_per_s;
  values.velocity_i_gain_per_s2 = gains.velocity_i_gain_per_s2;
  values.velocity_i_leak_time_s = gains.velocity_i_leak_time_s;
  values.velocity_i_acceleration_limit_mps2 = gains.velocity_i_acceleration_limit_mps2;
  values.adaptive_com_trim_enabled = 0.0;
  values.adaptive_com_trim_gain_deg_per_mps_s = 0.0;

  // Keep the comparison surface fixed. In particular, the 16000-SPS balance
  // ceiling and turn allocation are not optimizer escape hatches.
  values.balance_max_sps = Config::max_step_rate_sps;
  values.turn_max_sps = 1600.0;
  ConfigPid::values = values;
}

Gains clamp_gains(Gains gains) {
  // The initial speed range is deliberately wide but stays inside the
  // explicit 25%-of-available-SPS headroom validator.
  const double headroom_speed_mps =
      0.25 * (16000.0 - 1600.0) * Config::meters_per_step;
  gains.drive_max_velocity_mps =
      std::clamp(gains.drive_max_velocity_mps, 0.005, headroom_speed_mps);
  gains.planner_max_acceleration_mps2 =
      std::clamp(gains.planner_max_acceleration_mps2, 0.01, 6.0);
  gains.planner_max_deceleration_mps2 =
      std::clamp(gains.planner_max_deceleration_mps2, 0.01, 6.0);
  gains.planner_max_jerk_mps3 = std::clamp(gains.planner_max_jerk_mps3, 0.05, 60.0);
  gains.outer_pitch_limit_deg =
      std::clamp(gains.outer_pitch_limit_deg, 0.5, Config::max_motion_pitch_setpoint_deg);
  gains.velocity_feedback_cutoff_hz =
      std::clamp(gains.velocity_feedback_cutoff_hz, 0.10, 5.0);
  gains.velocity_gain_per_s = std::clamp(gains.velocity_gain_per_s, 0.0, 20.0);
  gains.velocity_i_gain_per_s2 = std::clamp(gains.velocity_i_gain_per_s2, 0.0, 5.0);
  gains.velocity_i_leak_time_s = std::clamp(gains.velocity_i_leak_time_s, 0.1, 20.0);
  gains.velocity_i_acceleration_limit_mps2 =
      std::clamp(gains.velocity_i_acceleration_limit_mps2, 0.01, 3.0);
  return gains;
}

std::string gains_key(const Gains& gains) {
  std::ostringstream output;
  output << std::setprecision(12) << gains.drive_max_velocity_mps << ':'
         << gains.planner_max_acceleration_mps2 << ':' << gains.planner_max_deceleration_mps2
         << ':' << gains.planner_max_jerk_mps3 << ':'
         << gains.outer_pitch_limit_deg << ':' << gains.velocity_feedback_cutoff_hz << ':'
         << gains.velocity_gain_per_s << ':' << gains.velocity_i_gain_per_s2 << ':'
         << gains.velocity_i_leak_time_s << ':' << gains.velocity_i_acceleration_limit_mps2;
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

SearchStage parse_stage(std::string_view value) {
  if (value == "feedforward" || value == "inner") return SearchStage::Feedforward;
  if (value == "integral" || value == "pi") return SearchStage::Integral;
  if (value == "observer") return SearchStage::Observer;
  if (value == "feedback" || value == "outer") return SearchStage::Feedback;
  if (value == "joint") return SearchStage::Joint;
  if (value == "motion" || value == "outer-motion") return SearchStage::Motion;
  if (value == "boundary" || value == "authority-boundary") return SearchStage::Boundary;
  if (value == "motion-integral" || value == "outer-motion-integral" ||
      value == "leaky-integral" || value == "leaky-motion") {
    return SearchStage::MotionIntegral;
  }
  throw std::runtime_error("Unknown tuning stage: " + std::string(value));
}

const char* stage_name(SearchStage stage) {
  switch (stage) {
    case SearchStage::Feedforward: return "feedforward";
    case SearchStage::Observer: return "observer";
    case SearchStage::Feedback: return "feedback";
    case SearchStage::Integral: return "integral";
    case SearchStage::Joint: return "joint";
    case SearchStage::Motion: return "motion";
    case SearchStage::Boundary: return "boundary";
    case SearchStage::MotionIntegral: return "motion-integral";
  }
  return "unknown";
}

std::vector<SimulatorScenario> stage_scenarios(SearchStage stage) {
  constexpr auto profile = PhysicsProfile::StepperPhaseElectrical;
  switch (stage) {
    case SearchStage::Feedforward:
      return tuning_motion_scenario_set(profile);
    case SearchStage::Observer:
    case SearchStage::Feedback:
    case SearchStage::Integral:
      return tuning_velocity_scenario_set(profile);
    case SearchStage::Joint: {
      return tuning_velocity_scenario_set(profile);
    }
    case SearchStage::Motion:
    case SearchStage::Boundary:
      return tuning_outer_motion_scenario_set(profile);
    case SearchStage::MotionIntegral:
      return tuning_leaky_integral_scenario_set(profile);
  }
  return {};
}

std::vector<Gains> candidates_for_stage(const Gains& base, SearchStage stage, size_t budget) {
  std::vector<Gains> candidates;
  std::set<std::string> seen;
  Gains stage_base = base;
  const bool real_motion_stage = stage == SearchStage::Motion ||
                                 stage == SearchStage::Boundary ||
                                 stage == SearchStage::MotionIntegral;
  if (stage == SearchStage::Feedforward) {
    stage_base.velocity_gain_per_s = 0.0;
    stage_base.velocity_i_gain_per_s2 = 0.0;
  }
  if (!real_motion_stage) add_unique(candidates, seen, stage_base);
  if (real_motion_stage) {
    constexpr double kFixedUserSpeedMps = 0.12;
    stage_base.drive_max_velocity_mps = kFixedUserSpeedMps;
    if (stage == SearchStage::Boundary) {
      // The boundary experiment must actually ask for more than the
      // historical 2.5--5 degree region. Hold the controller parent fixed,
      // but use the allowed planner envelope so the configured authority is
      // observable rather than an unused ceiling.
      stage_base.planner_max_acceleration_mps2 = 6.0;
      stage_base.planner_max_deceleration_mps2 = 6.0;
      stage_base.planner_max_jerk_mps3 = 60.0;
    }
    if (stage == SearchStage::Motion) {
      stage_base.velocity_i_gain_per_s2 = 0.0;
    }
    if (stage == SearchStage::MotionIntegral) {
      // The focused experiment varies only P/I behavior. Planner dynamics,
      // cutoff, speed cap, and total outer authority stay fixed.
      stage_base.planner_max_acceleration_mps2 = 0.25;
      stage_base.planner_max_deceleration_mps2 = 0.25;
      stage_base.planner_max_jerk_mps3 = 1.0;
      stage_base.outer_pitch_limit_deg = 15.0;
      stage_base.velocity_feedback_cutoff_hz = base.velocity_feedback_cutoff_hz;
      stage_base.velocity_gain_per_s = 0.5;
      stage_base.velocity_i_gain_per_s2 = 0.0;
    }
    add_unique(candidates, seen, stage_base);
    if (stage == SearchStage::MotionIntegral) {
      auto high_p = stage_base;
      high_p.velocity_gain_per_s = 8.0;
      add_unique(candidates, seen, high_p);
    }

    const std::vector<double> velocity_gain = {
        0.25, 0.5, 1.0, 2.0, 3.0, 4.0, 6.0, 8.0, 10.0, 12.0, 16.0, 20.0};
    const std::vector<double> cutoff = {0.25, 0.5, 0.7, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0};
    const std::vector<double> authority = {5.0, 10.0, 15.0, 20.0, 25.0, 30.0};
    const std::vector<double> acceleration = {0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 6.0};
    const std::vector<double> jerk = {0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 30.0, 60.0};

    if (stage == SearchStage::Boundary) {
      for (double value : authority) {
        auto candidate = stage_base;
        candidate.outer_pitch_limit_deg = value;
        add_unique(candidates, seen, candidate);
      }
    } else if (stage == SearchStage::Motion) {
      // First cover the cross-product of the three controller dimensions that
      // most directly expose the attainable frontier. Planner parameters are
      // then covered independently and in the random tail below.
      const size_t cross_product_budget = budget > 100 ? budget - 100 : budget;
      for (size_t index = 0;
           index < velocity_gain.size() * cutoff.size() * authority.size() &&
           candidates.size() < cross_product_budget;
           ++index) {
        auto candidate = stage_base;
        candidate.velocity_gain_per_s = velocity_gain[index % velocity_gain.size()];
        candidate.velocity_feedback_cutoff_hz =
            cutoff[(index / velocity_gain.size()) % cutoff.size()];
        candidate.outer_pitch_limit_deg =
            authority[(index / (velocity_gain.size() * cutoff.size())) % authority.size()];
        add_unique(candidates, seen, candidate);
        if (candidates.size() >= budget) break;
      }
      for (double value : acceleration) {
        auto candidate = stage_base;
        candidate.planner_max_acceleration_mps2 = value;
        add_unique(candidates, seen, candidate);
      }
      for (double value : acceleration) {
        auto candidate = stage_base;
        candidate.planner_max_deceleration_mps2 = value;
        add_unique(candidates, seen, candidate);
      }
      for (double value : jerk) {
        auto candidate = stage_base;
        candidate.planner_max_jerk_mps3 = value;
        add_unique(candidates, seen, candidate);
      }
    } else {
      const std::vector<double> proportional = {0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0};
      const std::vector<double> integral_gain = {0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 4.0};
      const std::vector<double> leak = {0.25, 0.5, 1.0, 2.0, 4.0, 8.0};
      const std::vector<double> limit = {0.3, 0.5, 0.8, 1.0, 1.4};
      if (stage == SearchStage::MotionIntegral) {
        // Interleave the authority slices. A budget smaller than the full
        // Cartesian product must still test every requested integral lean
        // level rather than exhausting the first 0.3 m/s2 slice.
        const std::vector<double> coarse_integral_gain = {0.1, 0.5, 1.0, 2.0, 4.0};
        const std::vector<double> coarse_leak = {0.5, 2.0, 8.0};
        for (size_t limit_index = 0;
             limit_index < limit.size() && candidates.size() < budget; ++limit_index) {
          for (size_t p_index = 0;
               p_index < proportional.size() && candidates.size() < budget; ++p_index) {
            for (size_t gain_index = 0;
                 gain_index < coarse_integral_gain.size() && candidates.size() < budget;
                 ++gain_index) {
              auto candidate = stage_base;
              candidate.velocity_gain_per_s = proportional[p_index];
              candidate.velocity_i_gain_per_s2 = coarse_integral_gain[gain_index];
              candidate.velocity_i_leak_time_s =
                  coarse_leak[(p_index + gain_index + limit_index) % coarse_leak.size()];
              candidate.velocity_i_acceleration_limit_mps2 = limit[limit_index];
              add_unique(candidates, seen, candidate);
            }
          }
        }
      } else {
        for (size_t index = 0;
             index < proportional.size() * integral_gain.size() * leak.size() * limit.size();
             ++index) {
          auto candidate = stage_base;
          candidate.velocity_gain_per_s = proportional[index % proportional.size()];
          candidate.velocity_i_gain_per_s2 =
              integral_gain[(index / proportional.size()) % integral_gain.size()];
          candidate.velocity_i_leak_time_s =
              leak[(index / (proportional.size() * integral_gain.size())) % leak.size()];
          candidate.velocity_i_acceleration_limit_mps2 =
              limit[(index / (proportional.size() * integral_gain.size() * leak.size())) %
                    limit.size()];
          add_unique(candidates, seen, candidate);
          if (candidates.size() >= budget) break;
        }
      }
    }

    uint64_t motion_state = 0x6d6f74696f6eULL + static_cast<uint64_t>(stage) * 0x9e3779b9ULL;
    while (candidates.size() < budget) {
      auto candidate = stage_base;
      candidate.drive_max_velocity_mps = kFixedUserSpeedMps;
      if (stage == SearchStage::Boundary) {
        candidate.outer_pitch_limit_deg =
            5.0 + unit_random(motion_state) * (Config::max_motion_pitch_setpoint_deg - 5.0);
      } else if (stage == SearchStage::Motion) {
        candidate.velocity_gain_per_s = log_sample(motion_state, 0.25, 20.0);
        candidate.velocity_feedback_cutoff_hz = log_sample(motion_state, 0.25, 5.0);
        candidate.outer_pitch_limit_deg =
            5.0 + unit_random(motion_state) * (Config::max_motion_pitch_setpoint_deg - 5.0);
        candidate.planner_max_acceleration_mps2 = log_sample(motion_state, 0.1, 6.0);
        candidate.planner_max_deceleration_mps2 = log_sample(motion_state, 0.1, 6.0);
        candidate.planner_max_jerk_mps3 = log_sample(motion_state, 0.1, 60.0);
        candidate.velocity_i_gain_per_s2 = 0.0;
      } else if (stage == SearchStage::MotionIntegral) {
        candidate.velocity_gain_per_s = log_sample(motion_state, 0.25, 3.0);
        candidate.velocity_i_gain_per_s2 = log_sample(motion_state, 0.05, 4.0);
        candidate.velocity_i_leak_time_s = log_sample(motion_state, 0.25, 8.0);
        candidate.velocity_i_acceleration_limit_mps2 =
            log_sample(motion_state, 0.3, 1.4);
      } else {
        candidate.velocity_i_gain_per_s2 = log_sample(motion_state, 0.01, 4.0);
        candidate.velocity_i_leak_time_s = log_sample(motion_state, 0.25, 8.0);
        candidate.velocity_i_acceleration_limit_mps2 = log_sample(motion_state, 0.05, 1.0);
      }
      add_unique(candidates, seen, candidate);
    }
    if (candidates.size() > budget) candidates.resize(budget);
    return candidates;
  }

  if (stage == SearchStage::Feedforward || stage == SearchStage::Joint) {
    const std::vector<double> speed = {0.02, 0.04, 0.06, 0.08, 0.10, 0.12, 0.14};
    const std::vector<double> acceleration = {0.05, 0.10, 0.20, 0.35, 0.60, 1.0, 2.0};
    const std::vector<double> deceleration = {0.05, 0.10, 0.20, 0.35, 0.60, 1.0, 2.0};
    const std::vector<double> jerk = {0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0};
    const std::vector<double> pitch_limit = {5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 30.0};
    for (size_t index = 0; index < speed.size(); ++index) {
      auto value = base;
      value.drive_max_velocity_mps = speed[index];
      value.planner_max_acceleration_mps2 = acceleration[index];
      value.planner_max_deceleration_mps2 = deceleration[index];
      value.planner_max_jerk_mps3 = jerk[index];
      value.outer_pitch_limit_deg = pitch_limit[index];
      value.velocity_gain_per_s = 0.0;
      value.velocity_i_gain_per_s2 = 0.0;
      add_unique(candidates, seen, value);
    }
    for (double value : speed) {
      auto candidate = base;
      candidate.drive_max_velocity_mps = value;
      candidate.velocity_gain_per_s = 0.0;
      add_unique(candidates, seen, candidate);
    }
    for (double value : acceleration) {
      auto candidate = base;
      candidate.planner_max_acceleration_mps2 = value;
      candidate.velocity_gain_per_s = 0.0;
      add_unique(candidates, seen, candidate);
    }
    for (double value : deceleration) {
      auto candidate = base;
      candidate.planner_max_deceleration_mps2 = value;
      candidate.velocity_gain_per_s = 0.0;
      add_unique(candidates, seen, candidate);
    }
    for (double value : jerk) {
      auto candidate = base;
      candidate.planner_max_jerk_mps3 = value;
      candidate.velocity_gain_per_s = 0.0;
      candidate.velocity_i_gain_per_s2 = 0.0;
      add_unique(candidates, seen, candidate);
    }
    for (double value : pitch_limit) {
      auto candidate = base;
      candidate.outer_pitch_limit_deg = value;
      candidate.velocity_gain_per_s = 0.0;
      add_unique(candidates, seen, candidate);
    }
  }

  if (stage == SearchStage::Observer) {
    const std::vector<double> cutoff = {0.25, 0.35, 0.50, 0.68, 0.85, 1.0, 1.5, 2.0};
    for (double value : cutoff) {
      auto candidate = base;
      candidate.velocity_feedback_cutoff_hz = value;
      candidate.velocity_gain_per_s = 0.0;
      add_unique(candidates, seen, candidate);
    }
  }

  if (stage == SearchStage::Feedback || stage == SearchStage::Joint) {
    const std::vector<double> gain = {0.10, 0.25, 0.50, 1.0, 2.0, 4.0, 7.0, 10.0, 12.0};
    for (double value : gain) {
      auto candidate = base;
      candidate.velocity_gain_per_s = value;
      add_unique(candidates, seen, candidate);
    }
  }

  if (stage == SearchStage::Integral || stage == SearchStage::Joint) {
    const std::vector<double> gain = {0.02, 0.05, 0.10, 0.25, 0.50, 1.0, 2.0};
    const std::vector<double> leak = {0.25, 0.5, 1.0, 2.0, 4.0, 8.0};
    const std::vector<double> limit = {0.05, 0.10, 0.20, 0.35, 0.50};
    for (double value : gain) {
      auto candidate = base;
      candidate.velocity_i_gain_per_s2 = value;
      add_unique(candidates, seen, candidate);
    }
    for (double value : leak) {
      auto candidate = base;
      candidate.velocity_i_leak_time_s = value;
      add_unique(candidates, seen, candidate);
    }
    for (double value : limit) {
      auto candidate = base;
      candidate.velocity_i_acceleration_limit_mps2 = value;
      add_unique(candidates, seen, candidate);
    }
  }

  uint64_t state = 0x5eed5eed12345678ULL + static_cast<uint64_t>(stage) * 0x9e3779b9ULL;
  while (candidates.size() < budget) {
    auto value = base;
    if (stage == SearchStage::Feedforward || stage == SearchStage::Joint) {
      value.drive_max_velocity_mps =
          log_sample(state, 0.01, 0.25 * (16000.0 - 1600.0) * Config::meters_per_step);
      value.planner_max_acceleration_mps2 = log_sample(state, 0.02, 6.0);
      value.planner_max_deceleration_mps2 = log_sample(state, 0.02, 6.0);
      value.planner_max_jerk_mps3 = log_sample(state, 0.05, 30.0);
      value.outer_pitch_limit_deg = log_sample(state, 5.0, 30.0);
      value.velocity_gain_per_s = 0.0;
      value.velocity_i_gain_per_s2 = 0.0;
    }
    if (stage == SearchStage::Observer || stage == SearchStage::Feedback ||
        stage == SearchStage::Joint) {
      value.velocity_feedback_cutoff_hz = log_sample(state, 0.25, 2.0);
    }
    if (stage == SearchStage::Feedback || stage == SearchStage::Joint) {
      value.velocity_gain_per_s = log_sample(state, 0.1, 12.0);
    }
    if (stage == SearchStage::Integral || stage == SearchStage::Joint) {
      value.velocity_i_gain_per_s2 = log_sample(state, 0.02, 2.0);
      value.velocity_i_leak_time_s = log_sample(state, 0.25, 8.0);
      value.velocity_i_acceleration_limit_mps2 = log_sample(state, 0.03, 0.6);
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

bool is_real_motion_stage(SearchStage stage) {
  return stage == SearchStage::Motion || stage == SearchStage::Boundary ||
         stage == SearchStage::MotionIntegral;
}

bool is_motion_guard(const std::string& scenario_name) {
  return scenario_name.rfind("guard_", 0) == 0;
}

bool is_motion_tracking_case(const std::string& scenario_name) {
  return scenario_name.rfind("motion_target_", 0) == 0 ||
         scenario_name.rfind("motion_pi_", 0) == 0 ||
         scenario_name == "motion_fixed_drive_stop";
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
  if (is_real_motion_stage(stage)) {
    // Do not reuse the old proxy gates here. They rejected the very high
    // authority region this experiment is meant to measure. The genuine
    // simulator fall/fault path remains authoritative; these are only
    // numerical runaway limits for ranking a candidate before full-suite
    // promotion.
    reject(metrics.peak_pitch_deg >= 65.0, "motion_peak_pitch");
    reject(metrics.peak_rate_dps >= 1200.0, "motion_peak_rate");
    reject(metrics.max_continuous_saturation_s >= 2.0, "motion_continuous_saturation");
    reject(metrics.growing_oscillation, "motion_growing_oscillation");
    if (is_motion_tracking_case(result.scenario.name) &&
        stage != SearchStage::Boundary) {
      // A motion candidate is not useful merely because it remains upright.
      // Require the simulated chassis to move in the requested direction and
      // to develop a material fraction of the requested mechanical speed.
      // Boundary runs deliberately use an intentionally excessive planner
      // demand and are scored as authority-stress experiments instead.
      reject(metrics.mechanical_velocity_direction_fraction < 0.80,
             "mechanical_velocity_direction");
      reject(metrics.mechanical_velocity_target_fraction < 0.20,
             "mechanical_velocity_target");
      reject(metrics.outer_pitch_target_limited_time_s > 0.05 * result.scenario.duration_s,
             "motion_outer_authority_duty");
      reject(metrics.max_continuous_saturation_s > 0.5, "motion_balance_rail");
    }
    if (is_motion_guard(result.scenario.name)) {
      reject(result.tail_rms_pitch_deg >= 6.0, "guard_tail_rms");
    }
    return failures.empty();
  }
  reject(metrics.peak_pitch_deg >= 35.0, "peak_pitch");
  reject(metrics.peak_rate_dps >= 450.0, "peak_rate");
  reject(metrics.growing_oscillation, "growing_oscillation");
  reject(metrics.max_continuous_saturation_s >= 1.0, "continuous_saturation");
  if (stage == SearchStage::Feedforward || stage == SearchStage::Joint) {
    reject(result.tail_rms_pitch_deg >= 2.0, "inner_tail_rms");
  }
  if (stage == SearchStage::Feedback || stage == SearchStage::Integral ||
      stage == SearchStage::Joint) {
    reject(std::abs(metrics.final_velocity_mean_sps) >= 400.0, "residual_velocity");
    reject(metrics.stop_speed_rms_sps >= 500.0, "stop_speed");
  }
  if (stage == SearchStage::Observer) {
    reject(!std::isfinite(metrics.velocity_feedback_mae_mps), "observer_invalid");
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
  cost += 20.0 * metrics.command_near_rail_time_s;
  cost += 15.0 * metrics.outer_acceleration_limited_time_s;
  if (metrics.growing_oscillation) cost += 4000.0;

  if (is_real_motion_stage(stage)) {
    // Mechanical plant velocity is deliberately the dominant objective. A
    // candidate cannot win by matching completed STEP activity while the
    // simulated chassis remains stationary.
    cost += 30000.0 * metrics.mechanical_velocity_target_iae_m_s;
    cost += 50000.0 * metrics.mechanical_velocity_late_error_mps;
    cost += 5000.0 * (1.0 - metrics.mechanical_velocity_target_fraction);
    cost += 3000.0 * (1.0 - metrics.mechanical_velocity_direction_fraction);
    cost += 14000.0 * metrics.release_distance_m;
    cost += 18000.0 * metrics.rebound_velocity_mps;
    cost += 500.0 * metrics.velocity_feedback_mae_mps;
    cost += 25.0 * metrics.outer_pitch_target_limited_time_s;
    if (is_motion_guard(result.scenario.name)) {
      cost += 300.0 * result.tail_rms_pitch_deg;
      cost += 2.0 * metrics.peak_pitch_deg;
    }
    return std::isfinite(cost) ? cost : 1.0e12;
  }

  if (stage == SearchStage::Observer || stage == SearchStage::Feedback ||
      stage == SearchStage::Integral ||
      stage == SearchStage::Joint) {
    cost += 0.80 * metrics.velocity_iae_sps_s;
    cost += 0.20 * std::abs(metrics.final_velocity_mean_sps);
    cost += 0.40 * metrics.drive_tracking_mae_sps;
    cost += 0.20 * metrics.stop_speed_rms_sps;
    cost += 1200.0 * metrics.release_distance_m;
    cost += 600.0 * metrics.rebound_velocity_mps;
    if (!metrics.settled) cost += 250.0;
  }
  if (stage == SearchStage::Observer) {
    cost += 2500.0 * metrics.velocity_feedback_mae_mps;
    cost += 800.0 * std::abs(metrics.velocity_feedback_bias_mps);
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
  if (stage == SearchStage::MotionIntegral) {
    output << "drive_max_velocity_mps=0.12 (fixed)\n";
    output << "planner_max_acceleration_mps2=0.25 (fixed)\n";
    output << "planner_max_deceleration_mps2=0.25 (fixed)\n";
    output << "planner_max_jerk_mps3=1 (fixed)\n";
    output << "outer_pitch_limit_deg=15 (fixed)\n";
    output << "velocity_feedback_cutoff_hz=base value (fixed)\n";
    output << "velocity_gain_per_s=0.25..3 for low/moderate P; 0.5 baseline; 8 high-P reference\n";
    output << "velocity_i_gain_per_s2=0.05..4 (P/I search)\n";
    output << "velocity_i_leak_time_s=0.25..8\n";
    output << "velocity_i_acceleration_limit_mps2=0.3,0.5,0.8,1.0,1.4\n";
  } else if (is_real_motion_stage(stage)) {
    output << "drive_max_velocity_mps=0.12 (fixed; no trivial low-speed solutions)\n";
    output << "planner_max_acceleration_mps2=0.1..6 (log coarse/random)\n";
    output << "planner_max_deceleration_mps2=0.1..6 (log coarse/random)\n";
    output << "planner_max_jerk_mps3=0.1..60 (log coarse/random)\n";
    output << "outer_pitch_limit_deg=5,10,15,20,25,30 plus random 5..45\n";
    output << "velocity_feedback_cutoff_hz=0.25..5 (log coarse/random)\n";
    output << "velocity_gain_per_s=0.25..20 (log coarse/random)\n";
    output << "velocity_i_gain_per_s2=0 for P-only; 0.01..4 for motion-integral\n";
    output << "velocity_i_leak_time_s=0.25..8 (motion-integral)\n";
    output << "velocity_i_acceleration_limit_mps2=0.05..1 (motion-integral)\n";
  } else {
    output << "drive_max_velocity_mps=0.005.."
           << 0.25 * (16000.0 - 1600.0) * Config::meters_per_step
           << " (log coarse/random; 25% headroom validated)\n";
    output << "planner_max_acceleration_mps2=0.01..6 (log coarse/random)\n";
    output << "planner_max_deceleration_mps2=0.01..6 (log coarse/random)\n";
    output << "planner_max_jerk_mps3=0.05..60 (log coarse/random)\n";
    output << "outer_pitch_limit_deg=5..45 (log coarse/random)\n";
    output << "velocity_feedback_cutoff_hz=0.25..5 (observer stage; log coarse/random)\n";
    output << "velocity_gain_per_s=0..20 (feedback stage; log coarse/random)\n";
    output << "velocity_i_gain_per_s2=0..5 (integral stage; log coarse/random)\n";
    output << "velocity_i_leak_time_s=0.1..20 (integral stage; log coarse/random)\n";
    output << "velocity_i_acceleration_limit_mps2=0.01..3 (integral stage; log coarse/random)\n";
  }
  output << "adaptive_com_trim_enabled=0 (fixed)\nbalance_max_sps=16000 (fixed)\nturn_max_sps=1600 (fixed)\n";
}

struct MotionFrontierMetrics {
  double target_iae_m_s = 0.0;
  double late_error_mps = 0.0;
  double direction_fraction = 0.0;
  double target_fraction = 0.0;
  double rebound_velocity_mps = 0.0;
  double peak_pitch_deg = 0.0;
  double peak_rate_dps = 0.0;
  double saturation_time_s = 0.0;
  double outer_limit_fraction = 0.0;
  size_t target_case_count = 0;
};

MotionFrontierMetrics motion_frontier_metrics(const Candidate& candidate) {
  MotionFrontierMetrics metrics;
  for (const auto& result : candidate.cases) {
    if (result.name.rfind("motion_", 0) != 0) continue;
    metrics.peak_pitch_deg = std::max(metrics.peak_pitch_deg, result.metrics.peak_pitch_deg);
    metrics.peak_rate_dps = std::max(metrics.peak_rate_dps, result.metrics.peak_rate_dps);
    metrics.saturation_time_s += result.metrics.saturation_time_s;
    metrics.outer_limit_fraction =
        std::max(metrics.outer_limit_fraction, result.metrics.outer_limit_fraction);
    metrics.rebound_velocity_mps =
        std::max(metrics.rebound_velocity_mps, result.metrics.rebound_velocity_mps);
    if (is_motion_tracking_case(result.name)) {
      metrics.target_iae_m_s += result.metrics.mechanical_velocity_target_iae_m_s;
      metrics.late_error_mps += result.metrics.mechanical_velocity_late_error_mps;
      metrics.direction_fraction += result.metrics.mechanical_velocity_direction_fraction;
      metrics.target_fraction += result.metrics.mechanical_velocity_target_fraction;
      ++metrics.target_case_count;
    }
  }
  if (metrics.target_case_count > 0) {
    const double count = static_cast<double>(metrics.target_case_count);
    metrics.target_iae_m_s /= count;
    metrics.late_error_mps /= count;
    metrics.direction_fraction /= count;
    metrics.target_fraction /= count;
  }
  return metrics;
}

bool motion_frontier_dominates(const MotionFrontierMetrics& left,
                               const MotionFrontierMetrics& right) {
  const std::array<double, 9> left_values = {
      left.target_iae_m_s,       left.late_error_mps,
      1.0 - left.direction_fraction, 1.0 - left.target_fraction,
      left.rebound_velocity_mps,   left.peak_pitch_deg,
      left.peak_rate_dps,          left.saturation_time_s,
      left.outer_limit_fraction};
  const std::array<double, 9> right_values = {
      right.target_iae_m_s,       right.late_error_mps,
      1.0 - right.direction_fraction, 1.0 - right.target_fraction,
      right.rebound_velocity_mps,   right.peak_pitch_deg,
      right.peak_rate_dps,          right.saturation_time_s,
      right.outer_limit_fraction};
  bool strictly_better = false;
  for (size_t index = 0; index < left_values.size(); ++index) {
    if (left_values[index] > right_values[index] + 1e-12) return false;
    strictly_better = strictly_better || left_values[index] + 1e-12 < right_values[index];
  }
  return strictly_better;
}

void write_artifacts(const std::filesystem::path& output_dir,
                     const std::vector<Candidate>& candidates, SearchStage stage) {
  std::ofstream summary(output_dir / "candidate_summary.csv");
  summary << "rank,candidate_id,round,drive_max_velocity_mps,planner_max_acceleration_mps2,"
             "planner_max_deceleration_mps2,planner_max_jerk_mps3,outer_pitch_limit_deg,"
             "velocity_feedback_cutoff_hz,velocity_gain_per_s,velocity_i_gain_per_s2,"
             "velocity_i_leak_time_s,velocity_i_acceleration_limit_mps2,passed_cases,"
             "hard_failures,score\n";
  std::ofstream validation(output_dir / "validation_results.csv");
  validation << "rank,candidate_id,scenario,accepted,quality_failures,fell,controller_fault_flags,"
                "actuator_fault_count,peak_command_sps,command_p95_sps,command_p99_sps,"
                "peak_pitch_deg,peak_rate_dps,tail_rms_pitch_deg,saturation_time_s,"
                "max_continuous_saturation_s,command_near_rail_time_s,"
                "outer_acceleration_limited_time_s,release_distance_m,rebound_velocity_mps,"
                "final_velocity_mean_sps,final_velocity_mean_mps,velocity_feedback_mae_mps,"
                "velocity_feedback_bias_mps,mechanical_velocity_target_iae_m_s,"
                "mechanical_velocity_late_error_mps,mechanical_velocity_peak_mps,"
                "mechanical_velocity_direction_fraction,mechanical_velocity_target_fraction,"
                "hold_user_mean_mps,hold_reference_mean_mps,hold_actual_mean_mps,"
                "hold_abs_error_mps,hold_direction_fraction,hold_target_fraction,"
                "hold_duration_s,"
                "drive_pitch_peak_deg,outer_limit_fraction,"
                "trim_trusted,final_trim_deg,trim_error_deg\n";
  std::ofstream metrics(output_dir / "scenario_metrics.csv");
  metrics << "rank,candidate_id,scenario," << tuning_metrics_csv_header() << '\n';
  for (size_t rank_index = 0; rank_index < candidates.size(); ++rank_index) {
    const auto& candidate = candidates[rank_index];
    const auto& gains = candidate.gains;
    summary << rank_index + 1 << ',' << candidate.id << ',' << candidate.round << ','
            << std::setprecision(12) << gains.drive_max_velocity_mps << ','
            << gains.planner_max_acceleration_mps2 << ','
            << gains.planner_max_deceleration_mps2 << ',' << gains.planner_max_jerk_mps3 << ','
            << gains.outer_pitch_limit_deg << ',' << gains.velocity_feedback_cutoff_hz << ','
            << gains.velocity_gain_per_s << ',' << gains.velocity_i_gain_per_s2 << ','
            << gains.velocity_i_leak_time_s << ','
            << gains.velocity_i_acceleration_limit_mps2 << ',' << candidate.passed_cases << ','
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
                 << result.metrics.command_near_rail_time_s << ','
                 << result.metrics.outer_acceleration_limited_time_s << ','
                 << result.metrics.release_distance_m << ','
                 << result.metrics.rebound_velocity_mps << ','
                 << result.metrics.final_velocity_mean_sps << ','
                 << result.metrics.final_velocity_mean_mps << ','
                 << result.metrics.velocity_feedback_mae_mps << ','
                 << result.metrics.velocity_feedback_bias_mps << ','
                 << result.metrics.mechanical_velocity_target_iae_m_s << ','
                 << result.metrics.mechanical_velocity_late_error_mps << ','
                 << result.metrics.mechanical_velocity_peak_mps << ','
                 << result.metrics.mechanical_velocity_direction_fraction << ','
                 << result.metrics.mechanical_velocity_target_fraction << ','
                 << result.metrics.mechanical_velocity_hold_user_mean_mps << ','
                 << result.metrics.mechanical_velocity_hold_reference_mean_mps << ','
                 << result.metrics.mechanical_velocity_hold_actual_mean_mps << ','
                 << result.metrics.mechanical_velocity_hold_abs_error_mps << ','
                 << result.metrics.mechanical_velocity_hold_direction_fraction << ','
                 << result.metrics.mechanical_velocity_hold_target_fraction << ','
                 << result.metrics.mechanical_velocity_hold_duration_s << ','
                 << result.metrics.drive_pitch_peak_deg << ','
                 << result.metrics.outer_limit_fraction << ','
                 << result.trim_trusted << ',' << result.final_trim_deg << ','
                 << result.trim_error_deg << '\n';
      metrics << rank_index + 1 << ',' << candidate.id << ',' << result.name << ','
              << tuning_metrics_csv_row(result.metrics) << '\n';
    }
  }
  if (is_real_motion_stage(stage)) {
    std::ofstream frontier(output_dir / "pareto_frontier.csv");
    frontier << "rank,candidate_id,velocity_gain_per_s,velocity_feedback_cutoff_hz,"
                 "outer_pitch_limit_deg,planner_max_acceleration_mps2,"
                 "planner_max_deceleration_mps2,planner_max_jerk_mps3,passed_cases,"
                 "hard_failures,target_iae_m_s,late_error_mps,direction_fraction,"
                 "target_fraction,rebound_velocity_mps,peak_pitch_deg,peak_rate_dps,"
                 "saturation_time_s,outer_limit_fraction,score\n";
    for (size_t index = 0; index < candidates.size(); ++index) {
      const auto left = motion_frontier_metrics(candidates[index]);
      bool dominated = false;
      for (size_t other = 0; other < candidates.size(); ++other) {
        if (other == index) continue;
        if (motion_frontier_dominates(motion_frontier_metrics(candidates[other]), left)) {
          dominated = true;
          break;
        }
      }
      if (dominated) continue;
      const auto& candidate = candidates[index];
      frontier << index + 1 << ',' << candidate.id << ',' << candidate.gains.velocity_gain_per_s
               << ',' << candidate.gains.velocity_feedback_cutoff_hz << ','
               << candidate.gains.outer_pitch_limit_deg << ','
               << candidate.gains.planner_max_acceleration_mps2 << ','
               << candidate.gains.planner_max_deceleration_mps2 << ','
               << candidate.gains.planner_max_jerk_mps3 << ',' << candidate.passed_cases << ','
               << candidate.hard_failures << ',' << left.target_iae_m_s << ','
               << left.late_error_mps << ',' << left.direction_fraction << ','
               << left.target_fraction << ',' << left.rebound_velocity_mps << ','
               << left.peak_pitch_deg << ',' << left.peak_rate_dps << ','
               << left.saturation_time_s << ',' << left.outer_limit_fraction << ','
               << candidate.score << '\n';
    }
  }
}

void write_best_config(const std::filesystem::path& output_dir, const Candidate& candidate,
                       size_t case_count) {
  apply_gains(candidate.gains);
  ConfigPid::save((output_dir / "best_observed.pid.conf").string());
  std::ofstream note(output_dir / "best_observed.txt");
  note << "StepperPhaseElectrical simulator tuning candidate; not hardware authorization.\n"
       << "motion/guard cases passed: " << candidate.passed_cases << '/' << case_count << '\n'
       << "hard failures: " << candidate.hard_failures << '\n'
       << "continuous score: " << candidate.score << '\n';
}

void write_best_n_configs(const std::filesystem::path& output_dir,
                          const std::vector<Candidate>& candidates, size_t count) {
  const size_t write_count = std::min(count, candidates.size());
  for (size_t index = 0; index < write_count; ++index) {
    apply_gains(candidates[index].gains);
    std::ostringstream filename;
    filename << "best_" << std::setw(2) << std::setfill('0') << (index + 1)
             << ".pid.conf";
    ConfigPid::save((output_dir / filename.str()).string());
  }
  if (!candidates.empty()) apply_gains(candidates.front().gains);
}

std::string timeline_file_component(std::string value) {
  for (char& character : value) {
    if (!std::isalnum(static_cast<unsigned char>(character)) && character != '_' &&
        character != '-') {
      character = '_';
    }
  }
  return value;
}

void write_timeline_csv(const std::filesystem::path& path, const SimulatorRunResult& result) {
  std::ofstream output(path);
  output << "t_sec,user_velocity_mps,reference_velocity_mps,reference_acceleration_mps2,"
            "reference_jerk_mps3,plant_velocity_mps,velocity_feedback_estimate_mps,"
            "velocity_error_mps,velocity_feedback_acceleration_mps2,"
            "velocity_p_acceleration_mps2,velocity_i_acceleration_mps2,"
            "velocity_integral_state_mps_s,acceleration_raw_mps2,acceleration_cmd_mps2,"
            "drive_pitch_target_deg,final_pitch_target_deg,plant_pitch_deg,"
            "plant_pitch_rate_dps,u_sps,balance_unclamped_sps,command_saturated,"
            "outer_acceleration_limited,outer_pitch_target_limited,velocity_integral_limited,"
            "velocity_anti_windup_active,active_outer_pitch_limit_deg,"
            "active_velocity_gain_per_s,active_velocity_i_gain_per_s2,"
            "active_velocity_i_leak_time_s,active_velocity_i_acceleration_limit_mps2,"
            "left_slewed_sps,right_slewed_sps,actuator_saturation_flags,phase_saturated,"
            "motor_force_saturated,missed_steps\n";
  output << std::setprecision(12);
  for (const auto& row : result.rows) {
    output << row.sim_time_s << ',' << row.user_velocity_mps << ',' << row.reference_velocity_mps
           << ',' << row.reference_acceleration_mps2 << ',' << row.reference_jerk_mps3 << ','
           << row.plant_velocity << ',' << row.velocity_feedback_estimate_mps << ','
           << row.velocity_error_mps << ',' << row.velocity_feedback_acceleration_mps2 << ','
           << row.velocity_p_acceleration_mps2 << ',' << row.velocity_i_acceleration_mps2 << ','
           << row.velocity_integral_state_mps_s << ',' << row.acceleration_raw_mps2 << ','
           << row.acceleration_cmd_mps2 << ',' << row.drive_pitch_target_deg << ','
           << row.final_pitch_target_deg << ',' << row.plant_pitch_deg << ','
           << row.plant_pitch_rate_dps << ',' << row.u_sps << ',' << row.balance_unclamped_sps
           << ',' << row.command_saturated << ',' << row.outer_acceleration_limited << ','
           << row.outer_pitch_target_limited << ',' << row.velocity_integral_limited << ','
           << row.velocity_anti_windup_active << ',' << row.active_outer_pitch_limit_deg << ','
           << row.active_velocity_gain_per_s << ',' << row.active_velocity_i_gain_per_s2 << ','
           << row.active_velocity_i_leak_time_s << ','
           << row.active_velocity_i_acceleration_limit_mps2 << ',' << row.left_slewed_sps << ','
           << row.right_slewed_sps << ',' << row.actuator_saturation_flags << ','
           << row.phase_saturated << ',' << row.motor_force_saturated << ',' << row.missed_steps
           << '\n';
  }
}

void write_focused_timelines(const std::filesystem::path& output_dir,
                             const std::vector<Candidate>& candidates,
                             const std::vector<SimulatorScenario>& scenarios,
                             const Gains& base_gains) {
  const auto timeline_dir = output_dir / "timelines";
  std::filesystem::create_directories(timeline_dir);
  std::vector<size_t> selected_indices;
  for (size_t index = 0; index < candidates.size() && selected_indices.size() < 3; ++index) {
    selected_indices.push_back(index);
  }

  std::vector<std::pair<std::string, Gains>> references;
  auto conservative = base_gains;
  conservative.drive_max_velocity_mps = 0.12;
  conservative.planner_max_acceleration_mps2 = 0.25;
  conservative.planner_max_deceleration_mps2 = 0.25;
  conservative.planner_max_jerk_mps3 = 1.0;
  conservative.outer_pitch_limit_deg = 15.0;
  conservative.velocity_i_gain_per_s2 = 0.0;
  conservative.velocity_gain_per_s = 0.5;
  references.emplace_back("reference_conservative_p", conservative);
  auto high_p = conservative;
  high_p.velocity_gain_per_s = 8.0;
  references.emplace_back("reference_high_p", high_p);

  const auto dump = [&](const std::string& label, const Gains& gains) {
    apply_gains(gains);
    const auto candidate_dir = timeline_dir / label;
    std::filesystem::create_directories(candidate_dir);
    ConfigPid::save((candidate_dir / "config.pid.conf").string());
    for (const auto& scenario : scenarios) {
      const auto result = run_simulator_scenario_with_loaded_pid(scenario);
      write_timeline_csv(
          candidate_dir / (timeline_file_component(scenario.name) + ".csv"), result);
    }
  };
  for (const auto& reference : references) dump(reference.first, reference.second);
  for (const size_t index : selected_indices) {
    const auto& candidate = candidates[index];
    std::ostringstream label;
    label << "rank_" << std::setw(2) << std::setfill('0') << (index + 1) << "_candidate_"
          << candidate.id;
    dump(label.str(), candidate.gains);
  }
}

void usage() {
  std::cout << "Usage: balancer_simulator_tuner --stage feedforward|feedback|integral|observer|joint|motion|boundary|motion-integral "
               "[--base FILE] [--output DIR] [--budget N]\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::filesystem::path base = "tests/data/stepper_phase_electrical_pid.conf";
  std::filesystem::path output_dir = "build/sim/stepper_tuning";
  SearchStage stage = SearchStage::Feedforward;
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
                            "scenario_metrics.csv", "search_bounds.txt", "validation_results.csv",
                            "pareto_frontier.csv"}) {
    std::filesystem::remove(output_dir / stale);
  }
  for (const auto& entry : std::filesystem::directory_iterator(output_dir)) {
    const std::string filename = entry.path().filename().string();
    if (filename.rfind("best_", 0) == 0 && entry.path().extension() == ".conf") {
      std::filesystem::remove(entry.path());
    }
  }
  write_bounds(output_dir, stage, budget);

  const auto gains = candidates_for_stage(base_gains, stage, budget);
  std::vector<Candidate> candidates;
  candidates.reserve(gains.size());
  for (size_t index = 0; index < gains.size(); ++index) {
    candidates.push_back(evaluate_candidate(index + 1, 0, gains[index], stage, scenarios));
  }
  rank(candidates);
  write_artifacts(output_dir, candidates, stage);
  write_best_config(output_dir, candidates.front(), scenarios.size());
  write_best_n_configs(output_dir, candidates, 10);
  if (stage == SearchStage::MotionIntegral) {
    write_focused_timelines(output_dir, candidates, scenarios, base_gains);
  }

  const auto& best = candidates.front();
  std::cout << "Stage " << stage_name(stage) << " evaluated " << candidates.size()
            << " candidates over " << scenarios.size() << " StepperPhaseElectrical scenarios.\n"
            << "Best motion candidate " << best.id << " passed " << best.passed_cases << '/'
            << scenarios.size() << " with " << best.hard_failures << " hard failures; score "
            << best.score << ". Artifacts: " << output_dir << '\n';
  return 0;
}
