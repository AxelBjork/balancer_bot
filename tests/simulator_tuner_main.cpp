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
#include <optional>
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

constexpr double kTuningBalanceMaxSps = 32000.0;
constexpr double kTuningTurnMaxSps = 1600.0;

// This tuner deliberately stays in the simulator support target. It changes
// only the selected ConfigPid values and uses the production controller/plant
// implementation for every evaluation. The root-controller stages search the
// attitude gains as well as the translational gains; adaptive COM trim and the
// fixed balance/turn ceilings remain outside the search space.
struct Gains {
  double pitch_gain{};
  double pitch_rate_gain{};
  double pitch_accel_gain{};
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
  RootInner,
  RootOuter,
  RootJoint,
  Feedforward,
  Feedback,
  Integral,
  Observer,
  Joint,
  Motion,
  Boundary,
  MotionIntegral,
  Distance,
  Envelope,
  LowDampingMotion,
};

double g_balance_ceiling_sps = kTuningBalanceMaxSps;
SearchStage g_active_stage = SearchStage::Feedforward;

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
  values.pitch_gain = gains.pitch_gain;
  values.pitch_rate_gain = gains.pitch_rate_gain;
  values.pitch_accel_gain = gains.pitch_accel_gain;
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
  values.balance_max_sps = g_balance_ceiling_sps;
  values.turn_max_sps = kTuningTurnMaxSps;
  ConfigPid::values = values;
}

Gains clamp_gains(Gains gains) {
  gains.pitch_gain = std::clamp(gains.pitch_gain, 20000.0, 220000.0);
  gains.pitch_rate_gain = std::clamp(gains.pitch_rate_gain, 150.0, 2000.0);
  gains.pitch_accel_gain = std::clamp(gains.pitch_accel_gain, 0.0, 300.0);
  // The initial speed range is deliberately wide but stays inside the
  // explicit 25%-of-available-SPS headroom validator.
  const double headroom_speed_mps =
      0.25 * (g_balance_ceiling_sps - kTuningTurnMaxSps) * Config::meters_per_step;
  const double experimental_speed_mps = g_active_stage == SearchStage::Envelope ? 0.5 : 0.0;
  gains.drive_max_velocity_mps =
      std::clamp(gains.drive_max_velocity_mps, 0.005,
                 std::max(headroom_speed_mps, experimental_speed_mps));
  gains.planner_max_acceleration_mps2 =
      std::clamp(gains.planner_max_acceleration_mps2, 0.01, 6.0);
  gains.planner_max_deceleration_mps2 =
      std::clamp(gains.planner_max_deceleration_mps2, 0.01, 6.0);
  gains.planner_max_jerk_mps3 = std::clamp(gains.planner_max_jerk_mps3, 0.05, 60.0);
  gains.outer_pitch_limit_deg =
      std::clamp(gains.outer_pitch_limit_deg, 0.5, Config::max_motion_pitch_setpoint_deg);
  gains.velocity_feedback_cutoff_hz =
      std::clamp(gains.velocity_feedback_cutoff_hz, 0.10, 5.0);
  gains.velocity_gain_per_s = std::clamp(gains.velocity_gain_per_s, 0.0, 30.0);
  gains.velocity_i_gain_per_s2 = std::clamp(gains.velocity_i_gain_per_s2, 0.0, 30.0);
  gains.velocity_i_leak_time_s = std::clamp(gains.velocity_i_leak_time_s, 0.05, 60.0);
  gains.velocity_i_acceleration_limit_mps2 =
      std::clamp(gains.velocity_i_acceleration_limit_mps2, 0.01, 8.0);
  return gains;
}

std::string gains_key(const Gains& gains) {
  std::ostringstream output;
  output << std::setprecision(12) << gains.pitch_gain << ':' << gains.pitch_rate_gain << ':'
         << gains.pitch_accel_gain << ':'
         << gains.drive_max_velocity_mps << ':'
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
  if (value == "inner-loop" || value == "root-inner" || value == "inner-tune") {
    return SearchStage::RootInner;
  }
  if (value == "outer-loop" || value == "root-outer" || value == "outer-tune") {
    return SearchStage::RootOuter;
  }
  if (value == "joint-refine" || value == "root-joint" || value == "joint-root") {
    return SearchStage::RootJoint;
  }
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
  if (value == "distance" || value == "drive-distance" ||
      value == "isolated-distance") {
    return SearchStage::Distance;
  }
  if (value == "envelope" || value == "speed-envelope" || value == "drive-envelope") {
    return SearchStage::Envelope;
  }
  if (value == "low-damping" || value == "damping1" ||
      value == "low-damping-motion") {
    return SearchStage::LowDampingMotion;
  }
  throw std::runtime_error("Unknown tuning stage: " + std::string(value));
}

const char* stage_name(SearchStage stage) {
  switch (stage) {
    case SearchStage::RootInner: return "root-inner";
    case SearchStage::RootOuter: return "root-outer";
    case SearchStage::RootJoint: return "root-joint";
    case SearchStage::Feedforward: return "feedforward";
    case SearchStage::Observer: return "observer";
    case SearchStage::Feedback: return "feedback";
    case SearchStage::Integral: return "integral";
    case SearchStage::Joint: return "joint";
    case SearchStage::Motion: return "motion";
    case SearchStage::Boundary: return "boundary";
    case SearchStage::MotionIntegral: return "motion-integral";
    case SearchStage::Distance: return "distance";
    case SearchStage::Envelope: return "envelope";
    case SearchStage::LowDampingMotion: return "low-damping-motion";
  }
  return "unknown";
}

std::vector<SimulatorScenario> stage_scenarios(
    SearchStage stage, std::optional<double> cart_damping_override) {
  constexpr auto profile = PhysicsProfile::StepperPhaseElectrical;
  switch (stage) {
    case SearchStage::RootInner:
    case SearchStage::RootOuter:
    case SearchStage::RootJoint:
      return tuning_root_controller_scenario_set(profile);
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
      return tuning_outer_motion_scenario_set(profile, cart_damping_override);
    case SearchStage::MotionIntegral:
      return tuning_leaky_integral_scenario_set(profile, cart_damping_override);
    case SearchStage::Distance:
      return tuning_distance_scenario_set(profile, cart_damping_override);
    case SearchStage::Envelope:
      return tuning_speed_envelope_scenario_set(profile, cart_damping_override);
    case SearchStage::LowDampingMotion:
      return tuning_outer_motion_scenario_set(profile, cart_damping_override);
  }
  return {};
}

std::vector<Gains> candidates_for_stage(const Gains& base, SearchStage stage, size_t budget) {
  std::vector<Gains> candidates;
  std::set<std::string> seen;
  Gains stage_base = base;
  const bool real_motion_stage = stage == SearchStage::Motion ||
                                 stage == SearchStage::Boundary ||
                                 stage == SearchStage::MotionIntegral ||
                                 stage == SearchStage::Distance ||
                                 stage == SearchStage::Envelope ||
                                 stage == SearchStage::LowDampingMotion;
  const bool root_controller_stage = stage == SearchStage::RootInner ||
                                     stage == SearchStage::RootOuter ||
                                     stage == SearchStage::RootJoint;
  if (root_controller_stage) {
    // Root tuning deliberately keeps every non-selected root value fixed.
    // In particular, no candidate may quietly alter speed, planner dynamics,
    // outer authority, I, contact parameters, or balance/turn limits.
    stage_base.velocity_i_gain_per_s2 = 0.0;

    if (stage == SearchStage::RootInner) {
      // Stage A uses a deliberately modest outer loop so inner-loop
      // candidates are compared against the same moving operating point.
      // Include P=0 as an explicit diagnostic, but keep P=1/cutoff=1.7 as
      // the main inner-loop surface.
      const std::vector<double> outer_p = {0.0, 1.0};
      const std::vector<double> outer_cutoff = {1.7, 2.0};
      const std::vector<double> pitch = {20000.0, 30000.0, 45000.0, 65000.0, 90000.0,
                                         120000.0, 150000.0, 180000.0, 203550.0, 220000.0};
      const std::vector<double> rate = {150.0, 200.0, 300.0, 400.0, 500.0, 700.0,
                                        900.0, 1190.0, 1500.0, 1800.0, 2000.0};
      const std::vector<double> accel = {0.0, 0.1, 0.3, 1.0, 3.0, 10.0, 30.0, 100.0, 300.0};
      for (double velocity_gain : outer_p) {
        for (double feedback_cutoff : outer_cutoff) {
          auto candidate = stage_base;
          candidate.velocity_gain_per_s = velocity_gain;
          candidate.velocity_feedback_cutoff_hz = feedback_cutoff;
          add_unique(candidates, seen, candidate);
        }
      }
      // Fully cover the zero-acceleration inner surface. This makes the
      // effect of the acceleration-feedback signal separable from the basic
      // pitch/rate damping choice.
      for (double pitch_gain : pitch) {
        for (double pitch_rate_gain : rate) {
          auto candidate = stage_base;
          candidate.pitch_gain = pitch_gain;
          candidate.pitch_rate_gain = pitch_rate_gain;
          candidate.pitch_accel_gain = 0.0;
          candidate.velocity_gain_per_s = 1.0;
          candidate.velocity_feedback_cutoff_hz = 1.7;
          add_unique(candidates, seen, candidate);
        }
      }
      // Add deterministic cross-sections for every acceleration scale before
      // random exploration. The index mixing spreads pitch/rate values across
      // the whole table rather than exhausting one corner of a Cartesian grid.
      for (size_t index = 0; index < 180; ++index) {
        auto candidate = stage_base;
        candidate.pitch_gain = pitch[(index * 7 + 3) % pitch.size()];
        candidate.pitch_rate_gain = rate[(index * 5 + 2) % rate.size()];
        candidate.pitch_accel_gain = accel[(index % (accel.size() - 1)) + 1];
        candidate.velocity_gain_per_s = 1.0;
        candidate.velocity_feedback_cutoff_hz = 1.7;
        add_unique(candidates, seen, candidate);
      }
      uint64_t state = 0x726f6f745f696e6eULL;
      while (candidates.size() < budget) {
        auto candidate = stage_base;
        candidate.pitch_gain = log_sample(state, 20000.0, 220000.0);
        candidate.pitch_rate_gain = log_sample(state, 150.0, 2000.0);
        candidate.pitch_accel_gain = unit_random(state) < 0.15
                                         ? 0.0
                                         : log_sample(state, 0.1, 300.0);
        candidate.velocity_gain_per_s = 1.0;
        candidate.velocity_feedback_cutoff_hz = 1.7;
        add_unique(candidates, seen, candidate);
      }
    } else if (stage == SearchStage::RootOuter) {
      const std::vector<double> proportional = {0.0, 0.2, 0.25, 0.5, 0.75, 1.0, 1.5,
                                                2.0, 3.0, 4.0, 5.0, 6.0};
      const std::vector<double> cutoff = {0.5, 0.7, 1.0, 1.5, 1.7, 2.0, 2.5, 3.0, 3.5, 4.0};
      for (double velocity_gain : proportional) {
        for (double feedback_cutoff : cutoff) {
          auto candidate = stage_base;
          candidate.velocity_gain_per_s = velocity_gain;
          candidate.velocity_feedback_cutoff_hz = feedback_cutoff;
          add_unique(candidates, seen, candidate);
        }
      }
      uint64_t state = 0x726f6f745f6f7574ULL;
      while (candidates.size() < budget) {
        auto candidate = stage_base;
        candidate.velocity_gain_per_s = unit_random(state) < 0.08
                                           ? 0.0
                                           : log_sample(state, 0.2, 6.0);
        candidate.velocity_feedback_cutoff_hz = log_sample(state, 0.5, 4.0);
        add_unique(candidates, seen, candidate);
      }
    } else {
      const std::array<double, 3> scale = {0.85, 1.0, 1.15};
      for (double pitch_scale : scale) {
        for (double rate_scale : scale) {
          for (double velocity_scale : scale) {
            for (double cutoff_scale : scale) {
              auto candidate = stage_base;
              candidate.pitch_gain = base.pitch_gain * pitch_scale;
              candidate.pitch_rate_gain = base.pitch_rate_gain * rate_scale;
              candidate.pitch_accel_gain = base.pitch_accel_gain * rate_scale;
              candidate.velocity_gain_per_s = base.velocity_gain_per_s * velocity_scale;
              candidate.velocity_feedback_cutoff_hz =
                  base.velocity_feedback_cutoff_hz * cutoff_scale;
              add_unique(candidates, seen, candidate);
            }
          }
        }
      }
      // A zero acceleration-feedback parent has no useful multiplicative
      // neighbourhood. Add explicit small positive probes before random
      // local refinement so a short joint budget still tests them.
      if (base.pitch_accel_gain == 0.0) {
        for (double accel_gain : {0.1, 0.3, 1.0, 3.0, 10.0}) {
          auto candidate = stage_base;
          candidate.pitch_accel_gain = accel_gain;
          add_unique(candidates, seen, candidate);
        }
      }
      uint64_t state = 0x726f6f745f6a6f69ULL;
      while (candidates.size() < budget) {
        auto candidate = stage_base;
        candidate.pitch_gain = base.pitch_gain * (0.85 + 0.30 * unit_random(state));
        candidate.pitch_rate_gain = base.pitch_rate_gain * (0.85 + 0.30 * unit_random(state));
        candidate.pitch_accel_gain = base.pitch_accel_gain == 0.0
                                         ? (unit_random(state) < 0.2
                                                ? 0.0
                                                : log_sample(state, 0.1, 30.0))
                                         : base.pitch_accel_gain *
                                               (0.85 + 0.30 * unit_random(state));
        candidate.velocity_gain_per_s = base.velocity_gain_per_s *
                                        (0.85 + 0.30 * unit_random(state));
        candidate.velocity_feedback_cutoff_hz = base.velocity_feedback_cutoff_hz *
                                                (0.85 + 0.30 * unit_random(state));
        add_unique(candidates, seen, candidate);
      }
    }
    if (candidates.size() > budget) candidates.resize(budget);
    return candidates;
  }
  if (stage == SearchStage::Feedforward) {
    stage_base.velocity_gain_per_s = 0.0;
    stage_base.velocity_i_gain_per_s2 = 0.0;
  }
  if (!real_motion_stage) add_unique(candidates, seen, stage_base);
  if (real_motion_stage) {
    constexpr double kFixedUserSpeedMps = 0.12;
    stage_base.drive_max_velocity_mps = kFixedUserSpeedMps;
    if (stage == SearchStage::Distance) {
      // This stage has one fixed full-forward scenario and a distance-only
      // objective. No stability-quality score or unrelated behavioral
      // scenario may pull the search back toward tiny motion.
      stage_base.outer_pitch_limit_deg = 15.0;
      stage_base.velocity_feedback_cutoff_hz = 0.68;
      stage_base.planner_max_acceleration_mps2 = 0.25;
      stage_base.planner_max_deceleration_mps2 = 0.25;
      stage_base.planner_max_jerk_mps3 = 1.0;

      const std::vector<double> proportional = {
          0.1, 0.25, 0.5, 1.0, 2.0, 3.0, 5.0, 8.0, 10.0, 15.0, 20.0, 30.0};
      const std::vector<double> integral_gain = {
          0.0, 0.02, 0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 30.0};
      const std::vector<double> leak = {0.05, 0.1, 0.25, 0.5, 1.0, 2.0,
                                        4.0, 8.0, 16.0, 30.0, 60.0};
      const std::vector<double> limit = {
          0.01, 0.25, 0.5, 0.8, 1.0, 1.4, 2.0, 3.0, 5.0, 8.0};
      const std::vector<double> cutoff = {0.2, 0.25, 0.5, 0.68, 1.0, 1.5,
                                          2.0, 3.0, 4.0, 5.0};
      const std::vector<double> acceleration = {0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 6.0};
      const std::vector<double> jerk = {0.05, 0.1, 0.25, 0.5, 1.0, 2.0,
                                        4.0, 8.0, 16.0, 30.0, 60.0};
      const std::vector<double> authority = {10.0, 15.0, 20.0, 30.0};
      const std::vector<double> selected_p = {0.25, 0.5, 1.0, 2.0, 3.0, 8.0};
      const std::vector<double> selected_i = {0.05, 0.25, 0.5, 1.0, 2.0, 5.0};
      const std::vector<double> selected_limit = {0.25, 0.5, 1.0, 1.4, 2.0};
      const std::vector<double> selected_leak = {0.1, 0.5, 1.0, 4.0, 16.0};

      auto add_distance_candidate = [&](Gains candidate) {
        candidate.drive_max_velocity_mps = kFixedUserSpeedMps;
        add_unique(candidates, seen, candidate);
      };
      add_distance_candidate(stage_base);
      for (double p : proportional) {
        auto candidate = stage_base;
        candidate.velocity_gain_per_s = p;
        candidate.velocity_i_gain_per_s2 = 0.0;
        add_distance_candidate(candidate);
      }
      for (double authority_deg : authority) {
        auto candidate = stage_base;
        candidate.outer_pitch_limit_deg = authority_deg;
        add_distance_candidate(candidate);
      }
      for (double p : selected_p) {
        for (double i : selected_i) {
          for (double i_limit : selected_limit) {
            auto candidate = stage_base;
            candidate.velocity_gain_per_s = p;
            candidate.velocity_i_gain_per_s2 = i;
            candidate.velocity_i_acceleration_limit_mps2 = i_limit;
            candidate.velocity_i_leak_time_s =
                selected_leak[static_cast<size_t>(p + i + i_limit) % selected_leak.size()];
            add_distance_candidate(candidate);
          }
        }
      }
      for (double value : cutoff) {
        auto candidate = stage_base;
        candidate.velocity_feedback_cutoff_hz = value;
        candidate.velocity_gain_per_s = 8.0;
        candidate.velocity_i_gain_per_s2 = 0.0;
        add_distance_candidate(candidate);
      }
      for (double value : acceleration) {
        auto candidate = stage_base;
        candidate.planner_max_acceleration_mps2 = value;
        candidate.velocity_gain_per_s = 8.0;
        candidate.velocity_i_gain_per_s2 = 0.0;
        add_distance_candidate(candidate);
      }
      for (double value : acceleration) {
        auto candidate = stage_base;
        candidate.planner_max_deceleration_mps2 = value;
        candidate.velocity_gain_per_s = 8.0;
        candidate.velocity_i_gain_per_s2 = 0.0;
        add_distance_candidate(candidate);
      }
      for (double value : jerk) {
        auto candidate = stage_base;
        candidate.planner_max_jerk_mps3 = value;
        candidate.velocity_gain_per_s = 8.0;
        candidate.velocity_i_gain_per_s2 = 0.0;
        add_distance_candidate(candidate);
      }
      for (double value : authority) {
        auto candidate = stage_base;
        candidate.outer_pitch_limit_deg = value;
        candidate.velocity_gain_per_s = 8.0;
        candidate.velocity_i_gain_per_s2 = 0.0;
        add_distance_candidate(candidate);
      }

      uint64_t state = 0x64697374616e6365ULL;
      while (candidates.size() < budget) {
        auto candidate = stage_base;
        candidate.velocity_gain_per_s =
            unit_random(state) < 0.08 ? 0.0 : log_sample(state, 0.1, 30.0);
        candidate.velocity_feedback_cutoff_hz = log_sample(state, 0.2, 5.0);
        candidate.outer_pitch_limit_deg =
            authority[static_cast<size_t>(
                          unit_random(state) * static_cast<double>(authority.size())) %
                      authority.size()];
        candidate.planner_max_acceleration_mps2 = log_sample(state, 0.1, 6.0);
        candidate.planner_max_deceleration_mps2 = log_sample(state, 0.1, 6.0);
        candidate.planner_max_jerk_mps3 = log_sample(state, 0.05, 60.0);
        if (unit_random(state) < 0.20) {
          candidate.velocity_i_gain_per_s2 = 0.0;
          candidate.velocity_i_acceleration_limit_mps2 = 0.01;
        } else {
          candidate.velocity_i_gain_per_s2 = log_sample(state, 0.02, 30.0);
          candidate.velocity_i_leak_time_s = log_sample(state, 0.05, 60.0);
          candidate.velocity_i_acceleration_limit_mps2 = log_sample(state, 0.25, 8.0);
        }
        add_distance_candidate(candidate);
      }
      if (candidates.size() > budget) candidates.resize(budget);
      return candidates;
    }
    if (stage == SearchStage::Envelope) {
      // This is an operating-envelope diagnostic, not a low-speed tuning
      // stage.  Keep the requested 0.5 m/s trajectory fixed and compare a
      // small set of controller parents and authority values at the selected
      // balance ceiling.  The speed can intentionally exceed the normal
      // 25%-headroom policy at low diagnostic ceilings; apply_gains keeps the
      // values in memory so this stage does not silently clip the experiment.
      stage_base.drive_max_velocity_mps = 0.5;
      const std::vector<double> proportional = {0.25, 0.5, 1.0, 2.0, 4.0, 8.0};
      const std::vector<double> integral_gain = {0.0, 0.1, 0.5, 1.0, 2.0};
      const std::vector<double> integral_limit = {0.5, 1.0, 2.0};
      const std::vector<double> authority = {10.0, 15.0, 20.0, 30.0};
      add_unique(candidates, seen, stage_base);
      for (double p : proportional) {
        auto candidate = stage_base;
        candidate.velocity_gain_per_s = p;
        candidate.velocity_i_gain_per_s2 = 0.0;
        add_unique(candidates, seen, candidate);
      }
      for (double authority_deg : authority) {
        auto candidate = stage_base;
        candidate.outer_pitch_limit_deg = authority_deg;
        add_unique(candidates, seen, candidate);
      }
      for (double p : proportional) {
        for (double i : integral_gain) {
          for (double limit_value : integral_limit) {
            auto candidate = stage_base;
            candidate.velocity_gain_per_s = p;
            candidate.velocity_i_gain_per_s2 = i;
            candidate.velocity_i_acceleration_limit_mps2 = limit_value;
            candidate.velocity_i_leak_time_s = i == 0.0 ? 1.0 : 2.0;
            add_unique(candidates, seen, candidate);
          }
        }
      }
      if (candidates.size() > budget) candidates.resize(budget);
      return candidates;
    }
    if (stage == SearchStage::LowDampingMotion) {
      // Broad low-damping retune: unlike the focused leaky-I experiment,
      // include observer bandwidth and planner dynamics so the controller is
      // not co-tuned to the old damping-40 response time.
      stage_base.drive_max_velocity_mps = 0.12;
      const std::vector<double> proportional = {
          0.1, 0.25, 0.5, 1.0, 2.0, 3.0, 5.0, 8.0, 12.0, 16.0, 20.0};
      const std::vector<double> integral_gain = {
          0.0, 0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0};
      const std::vector<double> leak = {0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0,
                                        16.0};
      const std::vector<double> limit = {0.25, 0.5, 0.8, 1.0, 1.4, 2.0, 4.0};
      const std::vector<double> cutoff = {0.2, 0.35, 0.5, 0.68, 1.0, 1.5, 2.0, 3.0, 4.0,
                                          5.0};
      const std::vector<double> acceleration = {0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 6.0};
      const std::vector<double> jerk = {0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 30.0,
                                        60.0};
      const std::vector<double> authority = {10.0, 15.0, 20.0, 30.0};
      add_unique(candidates, seen, stage_base);
      // Cover P-only bandwidth/authority regions first.
      for (double p : proportional) {
        for (double filter : cutoff) {
          for (double authority_deg : authority) {
            auto candidate = stage_base;
            candidate.velocity_gain_per_s = p;
            candidate.velocity_i_gain_per_s2 = 0.0;
            candidate.velocity_feedback_cutoff_hz = filter;
            candidate.outer_pitch_limit_deg = authority_deg;
            add_unique(candidates, seen, candidate);
          }
        }
      }
      // Then interleave meaningful I authority, memory, observer, planner,
      // and authority slices.  A deliberately mixed-radix order keeps every
      // dimension represented even when the requested budget is smaller than
      // the full Cartesian product.
      const size_t structured_count = std::min<size_t>(320, budget);
      for (size_t index = 0; index < structured_count && candidates.size() < budget; ++index) {
        auto candidate = stage_base;
        candidate.velocity_gain_per_s = proportional[index % proportional.size()];
        candidate.velocity_i_gain_per_s2 =
            integral_gain[(index * 3 / proportional.size()) % integral_gain.size()];
        candidate.velocity_i_acceleration_limit_mps2 =
            limit[(index * 5 / (proportional.size() * integral_gain.size())) % limit.size()];
        candidate.velocity_i_leak_time_s =
            leak[(index * 7 / (proportional.size() * integral_gain.size() * limit.size())) %
                 leak.size()];
        candidate.velocity_feedback_cutoff_hz =
            cutoff[(index * 11 / (proportional.size() * integral_gain.size() * limit.size() *
                                  leak.size())) % cutoff.size()];
        candidate.outer_pitch_limit_deg = authority[(index * 13) % authority.size()];
        candidate.planner_max_acceleration_mps2 = acceleration[(index * 17) % acceleration.size()];
        candidate.planner_max_deceleration_mps2 = acceleration[(index * 19) % acceleration.size()];
        candidate.planner_max_jerk_mps3 = jerk[(index * 23) % jerk.size()];
        add_unique(candidates, seen, candidate);
      }
      uint64_t state = 0x6c6f7764616d7069ULL;
      while (candidates.size() < budget) {
        auto candidate = stage_base;
        candidate.velocity_gain_per_s =
            unit_random(state) < 0.05 ? 0.0 : log_sample(state, 0.1, 20.0);
        candidate.velocity_i_gain_per_s2 =
            unit_random(state) < 0.20 ? 0.0 : log_sample(state, 0.05, 8.0);
        candidate.velocity_i_leak_time_s = log_sample(state, 0.05, 16.0);
        candidate.velocity_i_acceleration_limit_mps2 = log_sample(state, 0.25, 4.0);
        candidate.velocity_feedback_cutoff_hz = log_sample(state, 0.2, 5.0);
        candidate.outer_pitch_limit_deg =
            authority[static_cast<size_t>(unit_random(state) *
                                          static_cast<double>(authority.size())) %
                      authority.size()];
        candidate.planner_max_acceleration_mps2 = log_sample(state, 0.1, 6.0);
        candidate.planner_max_deceleration_mps2 = log_sample(state, 0.1, 6.0);
        candidate.planner_max_jerk_mps3 = log_sample(state, 0.1, 60.0);
        add_unique(candidates, seen, candidate);
      }
      if (candidates.size() > budget) candidates.resize(budget);
      return candidates;
    }
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
          log_sample(state, 0.01,
                     0.25 * (kTuningBalanceMaxSps - kTuningTurnMaxSps) *
                         Config::meters_per_step);
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
  return stage == SearchStage::RootInner || stage == SearchStage::RootOuter ||
         stage == SearchStage::RootJoint || stage == SearchStage::Motion ||
         stage == SearchStage::Boundary ||
         stage == SearchStage::MotionIntegral || stage == SearchStage::Distance ||
         stage == SearchStage::Envelope || stage == SearchStage::LowDampingMotion;
}

bool is_motion_guard(const std::string& scenario_name) {
  return scenario_name.rfind("guard_", 0) == 0 ||
         scenario_name.rfind("root_neutral_", 0) == 0 ||
         scenario_name.rfind("root_pitch_", 0) == 0 ||
         scenario_name.rfind("root_push_", 0) == 0 ||
         scenario_name.rfind("root_initial_velocity_", 0) == 0;
}

bool is_motion_tracking_case(const std::string& scenario_name) {
  return scenario_name.rfind("motion_target_", 0) == 0 ||
         scenario_name.rfind("motion_pi_", 0) == 0 ||
         scenario_name == "motion_fixed_drive_stop" ||
         scenario_name.rfind("root_motion_", 0) == 0;
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
  if (stage == SearchStage::Distance) {
    const bool complete =
        !result.rows.empty() &&
        result.rows.back().sim_time_s + 1e-6 >= metrics.active_command_end_s;
    reject(!complete || !metrics.active_distance_valid, "distance_window");
    if (metrics.active_distance_valid) {
      reject(metrics.signed_distance_m * metrics.reference_distance_m < -1e-7,
             "distance_wrong_direction");
    }
    return failures.empty();
  }
  if (stage == SearchStage::Envelope) {
    const bool complete =
        !result.rows.empty() &&
        result.rows.back().sim_time_s + 1e-6 >= metrics.active_command_end_s;
    reject(!complete || !metrics.active_distance_valid, "envelope_window");
    if (metrics.active_distance_valid) {
      reject(metrics.signed_distance_m * metrics.reference_distance_m < -1e-7,
             "envelope_wrong_direction");
    }
    return failures.empty();
  }
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
  if (stage == SearchStage::Distance) {
    if (!metrics.active_distance_valid) return 1.0e12;
    if (metrics.signed_distance_m * metrics.reference_distance_m < -1e-7) {
      return 1.0e9 - metrics.signed_distance_m;
    }
    // Signed active-command displacement is the sole primary objective for
    // this diagnostic experiment. Release quality, pitch quality, and
    // unrelated scenario behavior are retained in the artifacts only.
    return -metrics.signed_distance_m;
  }
  if (stage == SearchStage::Envelope) {
    if (!metrics.active_distance_valid) return 1.0e12;
    if (metrics.signed_distance_m * metrics.reference_distance_m < -1e-7) {
      return 1.0e9 - metrics.signed_distance_m;
    }
    // The envelope stage ranks actual signed travel first.  Its detailed
    // electrical, pitch, and release metrics remain diagnostic columns.
    return -metrics.signed_distance_m;
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
    // The 200 kSPS/s MotorRunner envelope is fixed. Penalize ordinary
    // command demand above its 500-SPS/frame allowance, but keep this below
    // the motion/stability terms so useful translation is not optimized away.
    cost += 0.015 * metrics.slew_active_fraction;
    cost += 0.002 * metrics.requested_command_delta_p95_sps;
    cost += 0.001 * metrics.requested_applied_error_p95_sps;
    cost += 0.0001 * metrics.integrated_slew_excess_sps;
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
    if ((stage == SearchStage::Distance || stage == SearchStage::Envelope) && !good) {
      ++candidate.hard_failures;
    }
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

void write_bounds(const std::filesystem::path& output_dir, SearchStage stage, size_t budget,
                  std::optional<double> cart_damping_override) {
  std::ofstream output(output_dir / "search_bounds.txt");
  output << "stage=" << stage_name(stage) << "\nprofile=StepperPhaseElectrical\n";
  output << "budget=" << budget << "\n";
  output << "balance_ceiling_sps=" << g_balance_ceiling_sps << "\n";
  output << "cart_damping=";
  if (cart_damping_override.has_value()) {
    output << *cart_damping_override;
  } else {
    output << "profile_default";
  }
  output << " N*s/m\n";
  if (stage == SearchStage::RootInner || stage == SearchStage::RootOuter ||
      stage == SearchStage::RootJoint) {
    output << "scenario_set=root_controller_scenario_set (root pid.conf speed and plant)\n";
    output << "pitch_gain=20000..220000 (log/coarse root-inner search; root outer stages hold it)\n";
    output << "pitch_rate_gain=150..2000 (log/coarse root-inner search; root outer stages hold it)\n";
    output << "pitch_accel_gain=0 plus 0.1..300 (explicit/log root-inner search; root outer stages hold it)\n";
    output << "velocity_gain_per_s=0, 0.2..6 (root-outer grid/log search; root inner stages hold it)\n";
    output << "velocity_feedback_cutoff_hz=0.5..4 (root-outer grid/log search; root inner stages hold it)\n";
    output << "root-joint=+/-15 percent local refinement over five active gains\n";
    output << "drive_max_velocity_mps/planner/authority/I fixed from root pid.conf\n";
    output << "optimizer_constraints=fall/fault/growing-oscillation/poor-motion only\n";
  } else if (stage == SearchStage::Distance) {
    output << "scenario=distance_full_forward_then_stop (fixed full-forward command)\n";
    output << "active_command_interval=1..6 s (signed plant-position delta)\n";
    output << "drive_max_velocity_mps=0.12 (fixed)\n";
    output << "velocity_gain_per_s=0.1..30 (explicit points plus log-random)\n";
    output << "velocity_i_gain_per_s2=0 or 0.02..30 (explicit points plus log-random)\n";
    output << "velocity_i_leak_time_s=0.05..60 (explicit points plus log-random)\n";
    output << "velocity_i_acceleration_limit_mps2=0.01..8 (explicit points plus log-random)\n";
    output << "velocity_feedback_cutoff_hz=0.2..5 (explicit points plus log-random)\n";
    output << "planner_max_acceleration_mps2=0.1..6 (log-random)\n";
    output << "planner_max_deceleration_mps2=0.1..6 (log-random)\n";
    output << "planner_max_jerk_mps3=0.05..60 (log-random)\n";
    output << "outer_pitch_limit_deg=10,15,20,30 (fixed per candidate)\n";
    output << "optimizer_constraints=fall/fault/incomplete/wrong-direction only\n";
  } else if (stage == SearchStage::Envelope) {
    output << "scenario=speed_envelope_05mps_ramp_hold_stop\n";
    output << "active_command_interval=1..23 s (signed plant-position delta)\n";
    output << "drive_max_velocity_mps=0.5 (fixed diagnostic request)\n";
    output << "balance_ceiling_sps=" << g_balance_ceiling_sps << "\n";
    output << "optimizer_constraints=fall/fault/incomplete/wrong-direction only\n";
  } else if (stage == SearchStage::MotionIntegral) {
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
  } else if (stage == SearchStage::LowDampingMotion) {
    output << "scenario_set=tuning_outer_motion_scenario_set (real motion plus recovery guards)\n";
    output << "drive_max_velocity_mps=0.12 (fixed)\n";
    output << "planner_max_acceleration_mps2=0.1..6\n";
    output << "planner_max_deceleration_mps2=0.1..6\n";
    output << "planner_max_jerk_mps3=0.1..60\n";
    output << "outer_pitch_limit_deg=10,15,20,30\n";
    output << "velocity_feedback_cutoff_hz=0.2..5\n";
    output << "velocity_gain_per_s=0.1..20\n";
    output << "velocity_i_gain_per_s2=0..8\n";
    output << "velocity_i_leak_time_s=0.05..16\n";
    output << "velocity_i_acceleration_limit_mps2=0.25..4\n";
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
           << 0.25 * (kTuningBalanceMaxSps - kTuningTurnMaxSps) * Config::meters_per_step
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
  output << "adaptive_com_trim_enabled=0 (fixed)\nbalance_max_sps=" << g_balance_ceiling_sps << "\n"
             "turn_max_sps=1600 (fixed)\nimplementation_max_step_rate_sps=64000\n";
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
    if (result.name.rfind("motion_", 0) != 0 &&
        result.name.rfind("root_motion_", 0) != 0) {
      continue;
    }
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
  summary << "rank,candidate_id,round,pitch_gain,pitch_rate_gain,pitch_accel_gain,drive_max_velocity_mps,"
             "planner_max_acceleration_mps2,"
             "planner_max_deceleration_mps2,planner_max_jerk_mps3,outer_pitch_limit_deg,"
             "velocity_feedback_cutoff_hz,velocity_gain_per_s,velocity_i_gain_per_s2,"
             "velocity_i_leak_time_s,velocity_i_acceleration_limit_mps2,passed_cases,"
             "hard_failures,score,signed_distance_m,reference_distance_m,"
             "distance_tracking_fraction,active_mean_reference_velocity_mps,"
             "active_mean_mechanical_velocity_mps,active_peak_mechanical_velocity_mps,"
             "active_final_mechanical_velocity_mps,active_mean_a_ref_mps2,"
             "active_mean_a_p_mps2,active_mean_a_i_mps2\n";
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
                "active_command_start_s,active_command_end_s,signed_distance_m,"
                "reference_distance_m,distance_tracking_fraction,"
                "active_mean_reference_velocity_mps,active_mean_mechanical_velocity_mps,"
                "active_peak_mechanical_velocity_mps,active_final_mechanical_velocity_mps,"
                "active_mean_a_ref_mps2,active_mean_a_p_mps2,active_mean_a_i_mps2,"
                "active_peak_a_ref_mps2,active_peak_a_p_mps2,active_peak_a_i_mps2,"
                "active_distance_valid,"
                "drive_pitch_peak_deg,outer_limit_fraction,slew_active_fraction,"
                "requested_applied_error_rms_sps,requested_applied_error_p95_sps,"
                "requested_applied_error_peak_sps,requested_command_delta_p95_sps,"
                "requested_command_delta_p99_sps,requested_command_delta_peak_sps,"
                "pitch_error_command_delta_p95_sps,pitch_error_command_delta_p99_sps,"
                "pitch_error_command_delta_peak_sps,pitch_rate_command_delta_p95_sps,"
                "pitch_rate_command_delta_p99_sps,pitch_rate_command_delta_peak_sps,"
                "pitch_accel_command_delta_p95_sps,pitch_accel_command_delta_p99_sps,"
                "pitch_accel_command_delta_peak_sps,pitch_target_command_delta_p95_sps,"
                "pitch_target_command_delta_p99_sps,pitch_target_command_delta_peak_sps,"
                "integrated_slew_excess_sps,longest_slew_limited_interval_s,"
                "trim_trusted,final_trim_deg,trim_error_deg\n";
  std::ofstream metrics(output_dir / "scenario_metrics.csv");
  metrics << "rank,candidate_id,scenario," << tuning_metrics_csv_header() << '\n';
  for (size_t rank_index = 0; rank_index < candidates.size(); ++rank_index) {
    const auto& candidate = candidates[rank_index];
    const auto& gains = candidate.gains;
    summary << rank_index + 1 << ',' << candidate.id << ',' << candidate.round << ','
            << std::setprecision(12) << gains.pitch_gain << ',' << gains.pitch_rate_gain << ','
            << gains.pitch_accel_gain << ','
            << gains.drive_max_velocity_mps << ','
            << gains.planner_max_acceleration_mps2 << ','
            << gains.planner_max_deceleration_mps2 << ',' << gains.planner_max_jerk_mps3 << ','
            << gains.outer_pitch_limit_deg << ',' << gains.velocity_feedback_cutoff_hz << ','
            << gains.velocity_gain_per_s << ',' << gains.velocity_i_gain_per_s2 << ','
            << gains.velocity_i_leak_time_s << ','
            << gains.velocity_i_acceleration_limit_mps2 << ',' << candidate.passed_cases << ','
            << candidate.hard_failures << ',' << candidate.score << ',';
    if (!candidate.cases.empty()) {
      const auto& primary = candidate.cases.front().metrics;
      summary << primary.signed_distance_m << ',' << primary.reference_distance_m << ','
              << primary.distance_tracking_fraction << ','
              << primary.active_mean_reference_velocity_mps << ','
              << primary.active_mean_mechanical_velocity_mps << ','
              << primary.active_peak_mechanical_velocity_mps << ','
              << primary.active_final_mechanical_velocity_mps << ','
              << primary.active_mean_a_ref_mps2 << ',' << primary.active_mean_a_p_mps2 << ','
              << primary.active_mean_a_i_mps2;
    } else {
      summary << "0,0,0,0,0,0,0,0,0,0";
    }
    summary << '\n';
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
                 << result.metrics.active_command_start_s << ','
                 << result.metrics.active_command_end_s << ','
                 << result.metrics.signed_distance_m << ','
                 << result.metrics.reference_distance_m << ','
                 << result.metrics.distance_tracking_fraction << ','
                 << result.metrics.active_mean_reference_velocity_mps << ','
                 << result.metrics.active_mean_mechanical_velocity_mps << ','
                 << result.metrics.active_peak_mechanical_velocity_mps << ','
                 << result.metrics.active_final_mechanical_velocity_mps << ','
                 << result.metrics.active_mean_a_ref_mps2 << ','
                 << result.metrics.active_mean_a_p_mps2 << ','
                 << result.metrics.active_mean_a_i_mps2 << ','
                 << result.metrics.active_peak_a_ref_mps2 << ','
                 << result.metrics.active_peak_a_p_mps2 << ','
                 << result.metrics.active_peak_a_i_mps2 << ','
                 << result.metrics.active_distance_valid << ','
                 << result.metrics.drive_pitch_peak_deg << ','
                 << result.metrics.outer_limit_fraction << ','
                 << result.metrics.slew_active_fraction << ','
                 << result.metrics.requested_applied_error_rms_sps << ','
                 << result.metrics.requested_applied_error_p95_sps << ','
                 << result.metrics.requested_applied_error_peak_sps << ','
                 << result.metrics.requested_command_delta_p95_sps << ','
                 << result.metrics.requested_command_delta_p99_sps << ','
                 << result.metrics.requested_command_delta_peak_sps << ','
                 << result.metrics.pitch_error_command_delta_p95_sps << ','
                 << result.metrics.pitch_error_command_delta_p99_sps << ','
                 << result.metrics.pitch_error_command_delta_peak_sps << ','
                 << result.metrics.pitch_rate_command_delta_p95_sps << ','
                 << result.metrics.pitch_rate_command_delta_p99_sps << ','
                 << result.metrics.pitch_rate_command_delta_peak_sps << ','
                 << result.metrics.pitch_accel_command_delta_p95_sps << ','
                 << result.metrics.pitch_accel_command_delta_p99_sps << ','
                 << result.metrics.pitch_accel_command_delta_peak_sps << ','
                 << result.metrics.pitch_target_command_delta_p95_sps << ','
                 << result.metrics.pitch_target_command_delta_p99_sps << ','
                 << result.metrics.pitch_target_command_delta_peak_sps << ','
                 << result.metrics.integrated_slew_excess_sps << ','
                 << result.metrics.longest_slew_limited_interval_s << ','
                 << result.trim_trusted << ',' << result.final_trim_deg << ','
                 << result.trim_error_deg << '\n';
      metrics << rank_index + 1 << ',' << candidate.id << ',' << result.name << ','
              << tuning_metrics_csv_row(result.metrics) << '\n';
    }
  }
  if (stage == SearchStage::Distance || stage == SearchStage::Envelope) {
    const auto filename = stage == SearchStage::Distance ? "distance_top10.csv"
                                                          : "speed_envelope_top10.csv";
    std::ofstream frontier(output_dir / filename);
    frontier << "rank,candidate_id,signed_distance_m,reference_distance_m,"
                 "distance_tracking_fraction,mean_reference_velocity_mps,"
                 "mean_mechanical_velocity_mps,peak_mechanical_velocity_mps,"
                 "final_mechanical_velocity_mps,velocity_gain_per_s,"
                 "velocity_i_gain_per_s2,velocity_i_leak_time_s,"
                 "velocity_i_acceleration_limit_mps2,velocity_feedback_cutoff_hz,"
                 "planner_max_acceleration_mps2,planner_max_deceleration_mps2,"
                 "planner_max_jerk_mps3,outer_pitch_limit_deg,peak_pitch_deg,"
                 "peak_rate_dps,peak_command_sps,command_p95_sps,command_p99_sps,"
                 "saturation_time_s,max_continuous_saturation_s,release_distance_m,"
                 "rebound_velocity_mps,mean_a_ref_mps2,mean_a_p_mps2,mean_a_i_mps2\n";
    const size_t top_count = std::min<size_t>(10, candidates.size());
    for (size_t index = 0; index < top_count; ++index) {
      const auto& candidate = candidates[index];
      if (candidate.cases.empty()) continue;
      const auto& value = candidate.cases.front();
      const auto& m = value.metrics;
      frontier << index + 1 << ',' << candidate.id << ',' << m.signed_distance_m << ','
               << m.reference_distance_m << ',' << m.distance_tracking_fraction << ','
               << m.active_mean_reference_velocity_mps << ','
               << m.active_mean_mechanical_velocity_mps << ','
               << m.active_peak_mechanical_velocity_mps << ','
               << m.active_final_mechanical_velocity_mps << ','
               << candidate.gains.velocity_gain_per_s << ','
               << candidate.gains.velocity_i_gain_per_s2 << ','
               << candidate.gains.velocity_i_leak_time_s << ','
               << candidate.gains.velocity_i_acceleration_limit_mps2 << ','
               << candidate.gains.velocity_feedback_cutoff_hz << ','
               << candidate.gains.planner_max_acceleration_mps2 << ','
               << candidate.gains.planner_max_deceleration_mps2 << ','
               << candidate.gains.planner_max_jerk_mps3 << ','
               << candidate.gains.outer_pitch_limit_deg << ',' << m.peak_pitch_deg << ','
               << m.peak_rate_dps << ',' << value.peak_command_sps << ','
               << value.command_p95_sps << ',' << value.command_p99_sps << ','
               << m.saturation_time_s << ',' << m.max_continuous_saturation_s << ','
               << m.release_distance_m << ',' << m.rebound_velocity_mps << ','
               << m.active_mean_a_ref_mps2 << ',' << m.active_mean_a_p_mps2 << ','
               << m.active_mean_a_i_mps2 << '\n';
    }
  } else if (is_real_motion_stage(stage)) {
    std::ofstream frontier(output_dir / "pareto_frontier.csv");
    frontier << "rank,candidate_id,pitch_gain,pitch_rate_gain,velocity_gain_per_s,"
                 "pitch_accel_gain,"
                 "velocity_feedback_cutoff_hz,"
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
      frontier << index + 1 << ',' << candidate.id << ',' << candidate.gains.pitch_gain << ','
               << candidate.gains.pitch_rate_gain << ',' << candidate.gains.velocity_gain_per_s
               << ',' << candidate.gains.pitch_accel_gain
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
            "plant_position_m,"
            "velocity_error_mps,velocity_feedback_acceleration_mps2,"
            "velocity_p_acceleration_mps2,velocity_i_acceleration_mps2,"
            "velocity_integral_state_mps_s,acceleration_raw_mps2,acceleration_cmd_mps2,"
            "drive_pitch_target_deg,drive_feedforward_sps,balance_correction_sps,"
            "common_unclamped_sps,final_pitch_target_deg,plant_pitch_deg,"
            "plant_pitch_rate_dps,pitch_feedback_sps,pitch_rate_feedback_sps,"
            "pitch_accel_feedback_sps,u_sps,balance_unclamped_sps,command_saturated,"
            "outer_acceleration_limited,outer_pitch_target_limited,velocity_integral_limited,"
            "velocity_anti_windup_active,active_outer_pitch_limit_deg,"
            "active_velocity_gain_per_s,active_velocity_i_gain_per_s2,"
            "active_velocity_i_leak_time_s,active_velocity_i_acceleration_limit_mps2,"
            "left_slewed_sps,right_slewed_sps,actuator_saturation_flags,phase_saturated,"
            "motor_force_saturated,missed_steps,stepper_chassis_velocity_mps,"
            "stepper_actual_wheel_velocity_mps,stepper_electrical_phase_error_left_rad,"
            "stepper_electrical_phase_error_right_rad,stepper_torque_left_nm,"
            "stepper_torque_right_nm,stepper_summed_torque_nm,stepper_current_a_left,"
            "stepper_current_b_left,stepper_current_a_right,stepper_current_b_right,"
            "stepper_back_emf_a_left,stepper_back_emf_b_left,stepper_back_emf_a_right,"
            "stepper_back_emf_b_right,stepper_voltage_saturated_left,"
            "stepper_voltage_saturated_right,emitted_step_velocity_sps,"
            "synthetic_estimator_velocity_sps,controller_feedback_velocity_sps,"
            "stepper_wheel_surface_velocity_mps,stepper_chassis_ground_velocity_mps,"
            "stepper_wheel_angle_left_rad,stepper_wheel_angle_right_rad,"
            "stepper_wheel_angular_velocity_left_rad_s,"
            "stepper_wheel_angular_velocity_right_rad_s,"
            "stepper_slip_velocity_mps,stepper_accumulated_slip_distance_m,"
            "stepper_wheel_acceleration_mps2,stepper_requested_contact_force_n,"
            "stepper_actual_contact_force_n,stepper_traction_limit_n,"
            "stepper_traction_utilization,stepper_requested_contact_force_left_n,"
            "stepper_requested_contact_force_right_n,stepper_actual_contact_force_left_n,"
            "stepper_actual_contact_force_right_n,stepper_traction_limit_left_n,"
            "stepper_traction_limit_right_n,stepper_traction_saturated,"
            "stepper_contact_sticking,stepper_static_contact_active,"
            "stepper_unwrapped_phase_error_left_rad,"
            "stepper_unwrapped_phase_error_right_rad,stepper_field_rotor_relative_velocity_left_rad_s,"
            "stepper_field_rotor_relative_velocity_right_rad_s,stepper_electrical_cycle_index_left,"
            "stepper_electrical_cycle_index_right,stepper_accumulated_cycle_slips_left,"
            "stepper_accumulated_cycle_slips_right,stepper_electrical_cycle_slipped_left,"
            "stepper_electrical_cycle_slipped_right\n";
  output << std::setprecision(12);
  for (const auto& row : result.rows) {
    output << row.sim_time_s << ',' << row.user_velocity_mps << ',' << row.reference_velocity_mps
           << ',' << row.reference_acceleration_mps2 << ',' << row.reference_jerk_mps3 << ','
           << row.plant_velocity << ',' << row.velocity_feedback_estimate_mps << ','
           << row.plant_position << ',' << row.velocity_error_mps << ','
           << row.velocity_feedback_acceleration_mps2 << ','
           << row.velocity_p_acceleration_mps2 << ',' << row.velocity_i_acceleration_mps2 << ','
           << row.velocity_integral_state_mps_s << ',' << row.acceleration_raw_mps2 << ','
           << row.acceleration_cmd_mps2 << ',' << row.drive_pitch_target_deg << ','
           << row.drive_feedforward_sps << ',' << row.balance_correction_sps << ','
           << row.common_unclamped_sps << ',' << row.final_pitch_target_deg << ','
           << row.plant_pitch_deg << ','
           << row.plant_pitch_rate_dps << ',' << row.pitch_feedback_sps << ','
           << row.pitch_rate_feedback_sps << ',' << row.pitch_accel_feedback_sps << ','
           << row.u_sps << ',' << row.balance_unclamped_sps
           << ',' << row.command_saturated << ',' << row.outer_acceleration_limited << ','
           << row.outer_pitch_target_limited << ',' << row.velocity_integral_limited << ','
           << row.velocity_anti_windup_active << ',' << row.active_outer_pitch_limit_deg << ','
           << row.active_velocity_gain_per_s << ',' << row.active_velocity_i_gain_per_s2 << ','
           << row.active_velocity_i_leak_time_s << ','
           << row.active_velocity_i_acceleration_limit_mps2 << ',' << row.left_slewed_sps << ','
           << row.right_slewed_sps << ',' << row.actuator_saturation_flags << ','
           << row.phase_saturated << ',' << row.motor_force_saturated << ',' << row.missed_steps
           << ',' << row.stepper_chassis_velocity_mps << ','
           << row.stepper_actual_wheel_velocity_mps << ','
           << row.stepper_electrical_phase_error_left_rad << ','
           << row.stepper_electrical_phase_error_right_rad << ',' << row.stepper_torque_left_nm
           << ',' << row.stepper_torque_right_nm << ',' << row.stepper_summed_torque_nm << ','
           << row.stepper_current_a_left << ',' << row.stepper_current_b_left << ','
           << row.stepper_current_a_right << ',' << row.stepper_current_b_right << ','
           << row.stepper_back_emf_a_left << ',' << row.stepper_back_emf_b_left << ','
           << row.stepper_back_emf_a_right << ',' << row.stepper_back_emf_b_right << ','
           << row.stepper_voltage_saturated_left << ',' << row.stepper_voltage_saturated_right
           << ',' << row.emitted_step_velocity_sps << ',' << row.synthetic_estimator_velocity_sps
            << ',' << row.controller_feedback_velocity_sps
           << ',' << row.stepper_wheel_surface_velocity_mps
           << ',' << row.stepper_chassis_ground_velocity_mps
           << ',' << row.stepper_wheel_angle_left_rad
           << ',' << row.stepper_wheel_angle_right_rad
           << ',' << row.stepper_wheel_angular_velocity_left_rad_s
           << ',' << row.stepper_wheel_angular_velocity_right_rad_s
           << ',' << row.stepper_slip_velocity_mps
           << ',' << row.stepper_accumulated_slip_distance_m
           << ',' << row.stepper_wheel_acceleration_mps2
           << ',' << row.stepper_requested_contact_force_n
           << ',' << row.stepper_actual_contact_force_n
           << ',' << row.stepper_traction_limit_n
           << ',' << row.stepper_traction_utilization
           << ',' << row.stepper_requested_contact_force_left_n
           << ',' << row.stepper_requested_contact_force_right_n
           << ',' << row.stepper_actual_contact_force_left_n
           << ',' << row.stepper_actual_contact_force_right_n
           << ',' << row.stepper_traction_limit_left_n
           << ',' << row.stepper_traction_limit_right_n
           << ',' << row.stepper_traction_saturated
           << ',' << row.stepper_contact_sticking
           << ',' << row.stepper_static_contact_active
           << ',' << row.stepper_unwrapped_electrical_phase_error_left_rad
           << ',' << row.stepper_unwrapped_electrical_phase_error_right_rad
           << ',' << row.stepper_field_rotor_relative_velocity_left_rad_s
           << ',' << row.stepper_field_rotor_relative_velocity_right_rad_s
           << ',' << row.stepper_electrical_cycle_index_left
           << ',' << row.stepper_electrical_cycle_index_right
           << ',' << row.stepper_accumulated_electrical_cycle_slips_left
           << ',' << row.stepper_accumulated_electrical_cycle_slips_right
           << ',' << row.stepper_electrical_cycle_slipped_left
           << ',' << row.stepper_electrical_cycle_slipped_right
           << '\n';
  }
}

void write_focused_timelines(const std::filesystem::path& output_dir,
                             const std::vector<Candidate>& candidates,
                             const std::vector<SimulatorScenario>& scenarios,
                             const Gains& base_gains) {
  const auto timeline_dir = output_dir / "timelines";
  std::filesystem::create_directories(timeline_dir);
  const bool distance_experiment =
      !scenarios.empty() && (scenarios.front().name.rfind("distance_", 0) == 0 ||
                             scenarios.front().name.rfind("speed_envelope_", 0) == 0);
  const bool root_experiment =
      !scenarios.empty() && scenarios.front().name.rfind("root_", 0) == 0;
  std::vector<size_t> selected_indices;
  const size_t selected_count = distance_experiment ? 10 : 3;
  for (size_t index = 0;
       index < candidates.size() && selected_indices.size() < selected_count; ++index) {
    selected_indices.push_back(index);
  }

  std::vector<std::pair<std::string, Gains>> references;
  auto conservative = base_gains;
  if (!root_experiment) {
    conservative.drive_max_velocity_mps = 0.12;
    conservative.planner_max_acceleration_mps2 = 0.25;
    conservative.planner_max_deceleration_mps2 = 0.25;
    conservative.planner_max_jerk_mps3 = 1.0;
    conservative.outer_pitch_limit_deg = 15.0;
  }
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
      if (distance_experiment) {
        auto mirrored = scenario;
        mirrored.name = scenarios.front().name.rfind("speed_envelope_", 0) == 0
                            ? "speed_envelope_reverse"
                            : "distance_full_reverse_then_stop";
        for (auto& segment : mirrored.joy_segments) {
          segment.forward = -segment.forward;
          segment.forward_end = -segment.forward_end;
        }
        const auto reverse_result = run_simulator_scenario_with_loaded_pid(mirrored);
        write_timeline_csv(candidate_dir / (timeline_file_component(mirrored.name) + ".csv"),
                           reverse_result);
      }
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
  std::cout << "Usage: balancer_simulator_tuner --stage root-inner|root-outer|root-joint|feedforward|feedback|integral|observer|joint|motion|boundary|motion-integral|distance|envelope|low-damping-motion "
               "[--base FILE] [--output DIR] [--budget N] [--offset N] [--count N] "
               "[--cart-damping N] [--balance-ceiling N] [--timelines]\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::filesystem::path base = "pid.conf";
  std::filesystem::path output_dir = "build/sim/stepper_tuning";
  SearchStage stage = SearchStage::Feedforward;
  size_t budget = 600;
  size_t offset = 0;
  size_t count = 0;
  bool write_timelines = false;
  std::optional<double> cart_damping_override = 1.0;
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
    } else if (arg == "--offset" && index + 1 < argc) {
      offset = std::stoul(argv[++index]);
    } else if (arg == "--count" && index + 1 < argc) {
      count = std::stoul(argv[++index]);
    } else if (arg == "--cart-damping" && index + 1 < argc) {
      const double value = std::stod(argv[++index]);
      if (!std::isfinite(value) || value < 0.0) {
        std::cerr << "--cart-damping must be a finite non-negative value\n";
        return 2;
      }
      cart_damping_override = value;
    } else if (arg == "--balance-ceiling" && index + 1 < argc) {
      const double value = std::stod(argv[++index]);
      if (!std::isfinite(value) || value <= kTuningTurnMaxSps ||
          value > Config::max_step_rate_sps) {
        std::cerr << "--balance-ceiling must exceed turn_max_sps and not exceed the "
                     "implementation step-rate ceiling\n";
        return 2;
      }
      g_balance_ceiling_sps = value;
    } else if (arg == "--timelines") {
      write_timelines = true;
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
  g_active_stage = stage;
  const Gains base_gains = loaded_gains();
  const auto scenarios = stage_scenarios(stage, cart_damping_override);
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
  const size_t evaluated_count = count == 0 ? budget : count;
  const size_t generation_budget =
      std::max<size_t>(20, std::max(budget, offset + evaluated_count));
  write_bounds(output_dir, stage, evaluated_count, cart_damping_override);
  if (offset != 0 || count != 0) {
    std::ofstream partition(output_dir / "search_partition.txt");
    partition << "candidate_generation_budget=" << generation_budget << '\n'
              << "candidate_offset=" << offset << '\n'
              << "candidate_count=" << evaluated_count << '\n';
  }

  const auto all_gains = candidates_for_stage(base_gains, stage, generation_budget);
  const size_t begin = std::min(offset, all_gains.size());
  const size_t end = std::min(all_gains.size(), begin + evaluated_count);
  const std::vector<Gains> gains(all_gains.begin() + static_cast<std::ptrdiff_t>(begin),
                                 all_gains.begin() + static_cast<std::ptrdiff_t>(end));
  std::vector<Candidate> candidates;
  candidates.reserve(gains.size());
  for (size_t index = 0; index < gains.size(); ++index) {
    candidates.push_back(
        evaluate_candidate(offset + index + 1, 0, gains[index], stage, scenarios));
  }
  if (candidates.empty()) {
    std::cerr << "No candidates selected; offset is beyond the generated search range\n";
    return 2;
  }
  rank(candidates);
  write_artifacts(output_dir, candidates, stage);
  write_best_config(output_dir, candidates.front(), scenarios.size());
  write_best_n_configs(output_dir, candidates, 10);
  if (stage == SearchStage::RootInner || stage == SearchStage::RootOuter ||
      stage == SearchStage::RootJoint || stage == SearchStage::MotionIntegral ||
      stage == SearchStage::LowDampingMotion ||
      ((stage == SearchStage::Distance || stage == SearchStage::Envelope) && write_timelines)) {
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
