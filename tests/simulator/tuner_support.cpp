#include "simulator/tuner_support.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

#include "services/main/config.h"

namespace {

constexpr double kSettledPitchDeg = 0.5;
constexpr double kSettledRateDps = 8.0;
constexpr double kSettledVelocitySps = 75.0;
constexpr double kSettledDurationS = 0.5;

double event_start_s(const SimulatorScenario& scenario) {
  double start = std::numeric_limits<double>::infinity();
  for (const auto& disturbance : scenario.disturbances) start = std::min(start, disturbance.start_s);
  for (const auto& segment : scenario.joy_segments) start = std::min(start, segment.start_s);
  return std::isfinite(start) ? start : 0.0;
}

double dt_at(const std::vector<SimulatorTimelineRow>& rows, size_t index) {
  if (index == 0 || index >= rows.size()) return 0.0;
  return std::max(0.0, rows[index].sim_time_s - rows[index - 1].sim_time_s);
}

struct HoldWindowAccumulator {
  double duration_s = 0.0;
  double user_sum = 0.0;
  double reference_sum = 0.0;
  double actual_sum = 0.0;
  double absolute_error_sum = 0.0;
  double direction_count_s = 0.0;
  double target_fraction_sum = 0.0;
  double target_duration_s = 0.0;
};

void calculate_hold_metrics(const SimulatorRunResult& result, ScenarioMetrics& metrics) {
  constexpr double kHoldWindowS = 1.0;
  HoldWindowAccumulator accumulated;

  for (const auto& segment : result.scenario.joy_segments) {
    if (segment.duration_s <= 0.0) continue;
    const double command_end = std::abs(segment.forward_end) > 1e-9
                                   ? segment.forward_end
                                   : segment.forward;
    // A ramp is not a velocity hold. Constant signed command segments are
    // intentionally measured separately from their planner/release ramps.
    if (std::abs(command_end - segment.forward) > 1e-9 ||
        std::abs(segment.forward) <= 1e-9) {
      continue;
    }

    double active_end_s = segment.start_s + segment.duration_s;
    for (const auto& other : result.scenario.joy_segments) {
      if (other.start_s > segment.start_s + 1e-9) {
        active_end_s = std::min(active_end_s, other.start_s);
      }
    }
    const double hold_end_s = active_end_s;
    const double hold_start_s =
        std::max(segment.start_s, hold_end_s - kHoldWindowS);
    if (hold_end_s <= hold_start_s + 1e-9) continue;

    for (size_t index = 1; index < result.rows.size(); ++index) {
      const auto& row = result.rows[index];
      const double interval_start_s = result.rows[index - 1].sim_time_s;
      const double interval_end_s = row.sim_time_s;
      const double overlap_start_s = std::max(interval_start_s, hold_start_s);
      const double overlap_end_s = std::min(interval_end_s, hold_end_s);
      const double dt_s = overlap_end_s - overlap_start_s;
      if (dt_s <= 0.0) continue;

      accumulated.duration_s += dt_s;
      accumulated.user_sum += row.user_velocity_mps * dt_s;
      accumulated.reference_sum += row.reference_velocity_mps * dt_s;
      accumulated.actual_sum += row.plant_velocity * dt_s;

      if (std::abs(row.reference_velocity_mps) > 0.01) {
        const double error = row.plant_velocity - row.reference_velocity_mps;
        accumulated.absolute_error_sum += std::abs(error) * dt_s;
        accumulated.target_fraction_sum +=
            std::clamp(1.0 - std::abs(error) /
                                 std::max(0.01, std::abs(row.reference_velocity_mps)),
                       0.0, 1.0) *
            dt_s;
        accumulated.target_duration_s += dt_s;
        if (row.plant_velocity * row.reference_velocity_mps > 0.0) {
          accumulated.direction_count_s += dt_s;
        }
      }
    }
  }

  if (accumulated.duration_s <= 0.0) return;
  metrics.mechanical_velocity_hold_duration_s = accumulated.duration_s;
  metrics.mechanical_velocity_hold_user_mean_mps =
      accumulated.user_sum / accumulated.duration_s;
  metrics.mechanical_velocity_hold_reference_mean_mps =
      accumulated.reference_sum / accumulated.duration_s;
  metrics.mechanical_velocity_hold_actual_mean_mps =
      accumulated.actual_sum / accumulated.duration_s;
  if (accumulated.target_duration_s <= 0.0) return;
  metrics.mechanical_velocity_hold_abs_error_mps =
      accumulated.absolute_error_sum / accumulated.target_duration_s;
  metrics.mechanical_velocity_hold_direction_fraction =
      accumulated.direction_count_s / accumulated.target_duration_s;
  metrics.mechanical_velocity_hold_target_fraction =
      accumulated.target_fraction_sum / accumulated.target_duration_s;
}

void calculate_release_metrics(const SimulatorRunResult& result, ScenarioMetrics& metrics) {
  const auto& rows = result.rows;
  if (rows.size() < 2) return;

  for (const auto& segment : result.scenario.joy_segments) {
    const double release_s = segment.start_s + segment.duration_s;
    if (segment.duration_s <= 0.0 || release_s >= result.scenario.duration_s) continue;

    double next_command_s = result.scenario.duration_s;
    for (const auto& other : result.scenario.joy_segments) {
      if (other.start_s > release_s + 1e-9) {
        next_command_s = std::min(next_command_s, other.start_s);
      }
    }
    const double direction_source =
        std::abs(segment.forward_end) > 1e-9 ? segment.forward_end : segment.forward;
    if (std::abs(direction_source) <= 1e-9) continue;
    const double direction = std::copysign(1.0, direction_source);
    double distance_m = 0.0;
    double rebound_velocity_mps = 0.0;
    for (size_t index = 1; index < rows.size(); ++index) {
      const auto& row = rows[index];
      if (row.sim_time_s < release_s) continue;
      if (row.sim_time_s >= next_command_s) break;
      distance_m += std::abs(row.plant_velocity) * dt_at(rows, index);
      rebound_velocity_mps =
          std::max(rebound_velocity_mps, std::max(0.0, -direction * row.plant_velocity));
      if (row.sim_time_s >= release_s + 0.5 &&
          std::abs(row.plant_velocity) <= 0.005 &&
          std::abs(row.reference_velocity_mps) <= 0.005) {
        break;
      }
    }
    metrics.release_distance_m = std::max(metrics.release_distance_m, distance_m);
    metrics.rebound_velocity_mps =
        std::max(metrics.rebound_velocity_mps, rebound_velocity_mps);
  }
}

}  // namespace

ScenarioMetrics calculate_tuning_metrics(const SimulatorRunResult& result) {
  ScenarioMetrics metrics;
  const auto& rows = result.rows;
  if (rows.empty()) return metrics;

  const double start_s = event_start_s(result.scenario);
  const bool drive = result.scenario.name.find("drive") != std::string::npos;
  double command_squared = 0.0;
  size_t command_count = 0;
  double previous_command = rows.front().u_sps;
  double initial_rate = result.scenario.initial_pitch_rate_dps;
  double observed_rate_sign = initial_rate;
  bool arrested = false;
  double continuous_saturation_s = 0.0;
  double first_peak = 0.0;
  double rebound = 0.0;
  double primary_pitch_sign = result.scenario.initial_pitch_deg;
  bool pitch_crossed_zero = false;
  double final_velocity_sum = 0.0;
  size_t final_velocity_count = 0;
  double stop_squared = 0.0;
  size_t stop_count = 0;
  double drive_error_sum = 0.0;
  size_t drive_count = 0;
  double preceding_tail_squared = 0.0;
  size_t preceding_tail_count = 0;
  double final_tail_squared = 0.0;
  size_t final_tail_count = 0;
  double observer_abs_error_sum = 0.0;
  double observer_error_sum = 0.0;
  double observer_error_squared = 0.0;
  size_t observer_count = 0;
  double final_velocity_mps_sum = 0.0;
  size_t final_velocity_mps_count = 0;
  double mechanical_target_error_iae = 0.0;
  double mechanical_target_fraction_sum = 0.0;
  size_t mechanical_target_count = 0;
  size_t mechanical_direction_correct_count = 0;
  double mechanical_late_error_sum = 0.0;
  size_t mechanical_late_count = 0;

  for (size_t index = 0; index < rows.size(); ++index) {
    const auto& row = rows[index];
    const double dt_s = dt_at(rows, index);
    metrics.peak_pitch_deg = std::max(metrics.peak_pitch_deg, std::abs(row.plant_pitch_deg));
    metrics.peak_rate_dps = std::max(metrics.peak_rate_dps, std::abs(row.plant_pitch_rate_dps));
    metrics.drive_pitch_peak_deg =
        std::max(metrics.drive_pitch_peak_deg, std::abs(row.drive_pitch_target_deg));
    command_squared += row.u_sps * row.u_sps;
    ++command_count;
    if (index > 0) metrics.command_total_variation_sps += std::abs(row.u_sps - previous_command);
    previous_command = row.u_sps;

    if (row.sim_time_s >= start_s) {
      metrics.pitch_iae_deg_s += std::abs(row.plant_pitch_deg) * dt_s;
      metrics.velocity_iae_sps_s += std::abs(row.corrected_axle_velocity_sps) * dt_s;
      const double mechanical_target_error =
          std::abs(row.plant_velocity - row.reference_velocity_mps);
      mechanical_target_error_iae += mechanical_target_error * dt_s;
      metrics.mechanical_velocity_peak_mps =
          std::max(metrics.mechanical_velocity_peak_mps, std::abs(row.plant_velocity));
      if (row.sim_time_s >= result.scenario.duration_s - 1.0) {
        mechanical_late_error_sum += mechanical_target_error;
        ++mechanical_late_count;
      }
      if (std::abs(row.reference_velocity_mps) > 0.01) {
        ++mechanical_target_count;
        const double target_fraction = std::clamp(
            1.0 - mechanical_target_error / std::max(0.01, std::abs(row.reference_velocity_mps)),
            0.0, 1.0);
        mechanical_target_fraction_sum += target_fraction;
        if (row.plant_velocity * row.reference_velocity_mps > 0.0) {
          ++mechanical_direction_correct_count;
        }
      }
      const double magnitude = std::abs(row.plant_pitch_deg);
      if (std::abs(primary_pitch_sign) < 0.1 && magnitude >= 0.1) {
        primary_pitch_sign = row.plant_pitch_deg;
      }
      if (std::abs(primary_pitch_sign) >= 0.1 && !pitch_crossed_zero) {
        first_peak = std::max(first_peak, magnitude);
        pitch_crossed_zero = primary_pitch_sign * row.plant_pitch_deg <= 0.0;
      } else {
        rebound = std::max(rebound, magnitude);
      }
      if (!arrested && std::abs(observed_rate_sign) < 1e-6 &&
          std::abs(row.plant_pitch_rate_dps) > 1e-3) {
        observed_rate_sign = row.plant_pitch_rate_dps;
      }
      if (!arrested && std::abs(observed_rate_sign) > 1e-6 &&
          observed_rate_sign * row.plant_pitch_rate_dps <= 0.0) {
        metrics.arrest_time_s = row.sim_time_s - start_s;
        arrested = true;
      }
    }

    if (row.velocity_feedback_valid > 0.5 && std::isfinite(row.plant_velocity) &&
        std::isfinite(row.velocity_feedback_estimate_mps)) {
      const double observer_error = row.velocity_feedback_estimate_mps - row.plant_velocity;
      observer_abs_error_sum += std::abs(observer_error);
      observer_error_sum += observer_error;
      observer_error_squared += observer_error * observer_error;
      ++observer_count;
    }

    if (row.outer_acceleration_limited > 0.5) {
      metrics.outer_acceleration_limited_time_s += dt_s;
    }
    if (row.outer_pitch_target_limited > 0.5) {
      metrics.outer_pitch_target_limited_time_s += dt_s;
    }
    if (std::abs(row.u_sps) >= 0.95 * Config::max_step_rate_sps) {
      metrics.command_near_rail_time_s += dt_s;
    }

    if (row.command_saturated > 0.5) {
      metrics.saturation_time_s += dt_s;
      continuous_saturation_s += dt_s;
      metrics.max_continuous_saturation_s =
          std::max(metrics.max_continuous_saturation_s, continuous_saturation_s);
    } else {
      continuous_saturation_s = 0.0;
    }

    if (row.sim_time_s >= result.scenario.duration_s - 1.0) {
      final_velocity_sum += row.corrected_axle_velocity_sps;
      ++final_velocity_count;
      final_velocity_mps_sum += row.plant_velocity;
      ++final_velocity_mps_count;
    }
    if (row.sim_time_s >= result.scenario.duration_s - 1.0 &&
        row.sim_time_s < result.scenario.duration_s - 0.5) {
      preceding_tail_squared += row.plant_pitch_deg * row.plant_pitch_deg;
      ++preceding_tail_count;
    }
    if (row.sim_time_s >= result.scenario.duration_s - 0.5) {
      final_tail_squared += row.plant_pitch_deg * row.plant_pitch_deg;
      ++final_tail_count;
    }
    if (drive) {
      const bool positive_drive = row.sim_time_s >= 1.0 && row.sim_time_s < 3.0;
      const bool negative_drive = row.sim_time_s >= 5.0 && row.sim_time_s < 7.0;
      if (positive_drive || negative_drive) {
        const double reference_sps = positive_drive ? 600.0 : -600.0;
        drive_error_sum += std::abs(row.corrected_axle_velocity_sps - reference_sps);
        ++drive_count;
      }
      if ((row.sim_time_s >= 3.0 && row.sim_time_s < 5.0) ||
          (row.sim_time_s >= 7.0 && row.sim_time_s <= 9.0)) {
        stop_squared += row.corrected_axle_velocity_sps * row.corrected_axle_velocity_sps;
        ++stop_count;
      }
    }
  }
  metrics.command_rms_sps = std::sqrt(command_squared / static_cast<double>(command_count));
  metrics.rebound_ratio = first_peak > 1e-6 ? rebound / first_peak : 0.0;
  metrics.arrest_time_s = arrested ? metrics.arrest_time_s : std::numeric_limits<double>::infinity();
  metrics.final_velocity_mean_sps = final_velocity_count > 0
                                        ? final_velocity_sum / static_cast<double>(final_velocity_count)
                                        : std::numeric_limits<double>::infinity();
  metrics.final_velocity_mean_mps =
      final_velocity_mps_count > 0
          ? final_velocity_mps_sum / static_cast<double>(final_velocity_mps_count)
          : std::numeric_limits<double>::infinity();
  metrics.velocity_feedback_mae_mps =
      observer_count > 0 ? observer_abs_error_sum / static_cast<double>(observer_count)
                          : std::numeric_limits<double>::infinity();
  metrics.velocity_feedback_bias_mps =
      observer_count > 0 ? observer_error_sum / static_cast<double>(observer_count)
                          : std::numeric_limits<double>::infinity();
  metrics.velocity_feedback_rms_mps =
      observer_count > 0
          ? std::sqrt(observer_error_squared / static_cast<double>(observer_count))
          : std::numeric_limits<double>::infinity();
  metrics.mechanical_velocity_target_iae_m_s = mechanical_target_error_iae;
  metrics.mechanical_velocity_late_error_mps =
      mechanical_late_count > 0
          ? mechanical_late_error_sum / static_cast<double>(mechanical_late_count)
          : std::abs(metrics.final_velocity_mean_mps);
  metrics.mechanical_velocity_direction_fraction =
      mechanical_target_count > 0
          ? static_cast<double>(mechanical_direction_correct_count) /
                static_cast<double>(mechanical_target_count)
          : 1.0;
  metrics.mechanical_velocity_target_fraction =
      mechanical_target_count > 0
          ? mechanical_target_fraction_sum / static_cast<double>(mechanical_target_count)
          : 1.0;
  const double configured_outer_limit_deg =
      result.rows.empty() ? 0.0 : std::abs(result.rows.back().active_outer_pitch_limit_deg);
  metrics.outer_limit_fraction = configured_outer_limit_deg > 1e-9
                                    ? metrics.drive_pitch_peak_deg / configured_outer_limit_deg
                                    : 0.0;
  metrics.drive_tracking_mae_sps = drive_count > 0
                                       ? drive_error_sum / static_cast<double>(drive_count)
                                       : 0.0;
  metrics.stop_speed_rms_sps = stop_count > 0
                                   ? std::sqrt(stop_squared / static_cast<double>(stop_count))
                                   : 0.0;
  if (preceding_tail_count > 0 && final_tail_count > 0) {
    const double preceding_rms =
        std::sqrt(preceding_tail_squared / static_cast<double>(preceding_tail_count));
    const double final_rms = std::sqrt(final_tail_squared / static_cast<double>(final_tail_count));
    metrics.growing_oscillation = final_rms > preceding_rms + 0.05;
  }

  calculate_hold_metrics(result, metrics);
  if (metrics.mechanical_velocity_hold_duration_s > 0.0) {
    // Keep the historical field names usable while making their window
    // explicit: for constant-command scenarios these now describe the final
    // one-second pre-release hold, not the post-release scenario tail.
    metrics.mechanical_velocity_late_error_mps =
        metrics.mechanical_velocity_hold_abs_error_mps;
    metrics.mechanical_velocity_direction_fraction =
        metrics.mechanical_velocity_hold_direction_fraction;
    metrics.mechanical_velocity_target_fraction =
        metrics.mechanical_velocity_hold_target_fraction;
  }

  for (size_t index = 0; index < rows.size(); ++index) {
    const auto& row = rows[index];
    if (row.sim_time_s < start_s) continue;
    const double window_end_s = row.sim_time_s + kSettledDurationS;
    if (window_end_s > result.scenario.duration_s) break;
    const bool stays_settled = std::all_of(rows.begin() + static_cast<std::ptrdiff_t>(index), rows.end(),
        [&](const auto& later) {
          if (later.sim_time_s > window_end_s) return true;
          return std::abs(later.plant_pitch_deg) < kSettledPitchDeg &&
                 std::abs(later.plant_pitch_rate_dps) < kSettledRateDps &&
                 std::abs(later.corrected_axle_velocity_sps) < kSettledVelocitySps;
        });
    if (stays_settled) {
      metrics.settled = true;
      metrics.settling_time_s = row.sim_time_s - start_s;
      break;
    }
  }
  if (!metrics.settled) metrics.settling_time_s = std::numeric_limits<double>::infinity();
  calculate_release_metrics(result, metrics);
  metrics.safe = !result.fell && result.actuator_fault_count == 0 &&
                 result.controller_fault_flags == 0 && std::isfinite(metrics.peak_pitch_deg) &&
                 std::isfinite(metrics.peak_rate_dps);
  return metrics;
}

double normalized_tuning_metric(double value, double baseline) {
  if (!std::isfinite(value)) return 10.0;
  return value / std::max(1e-6, std::abs(baseline));
}

bool tuning_metrics_dominate(const std::vector<double>& left, const std::vector<double>& right) {
  if (left.size() != right.size() || left.empty()) return false;
  bool strictly_better = false;
  for (size_t index = 0; index < left.size(); ++index) {
    if (left[index] > right[index]) return false;
    strictly_better = strictly_better || left[index] < right[index];
  }
  return strictly_better;
}

std::vector<double> tuning_pareto_objectives(TunerRankingStage stage,
                                             const TunerRankingSummary& value) {
  switch (stage) {
    case TunerRankingStage::Inner:
      return {value.worst_settling_time_s, value.total_pitch_iae_deg_s,
              value.worst_peak_pitch_deg, value.worst_peak_rate_dps,
              value.neutral_command_variation_sps};
    case TunerRankingStage::Authority:
      return {value.worst_arrest_time_s, value.worst_settling_time_s,
              value.worst_rebound_ratio, value.max_continuous_saturation_s,
              value.residual_velocity_sps, value.post_recovery_command_variation_sps};
    case TunerRankingStage::Velocity:
      return {value.total_velocity_iae_sps_s, value.total_drive_tracking_mae_sps,
              value.worst_settling_time_s, value.worst_peak_pitch_deg,
              value.post_recovery_command_variation_sps};
    case TunerRankingStage::Drive:
      return {value.total_drive_tracking_mae_sps, value.total_stop_speed_rms_sps,
              value.worst_peak_pitch_deg, value.max_continuous_saturation_s,
              value.post_recovery_command_variation_sps};
    case TunerRankingStage::Trim:
      return {value.trim_speed_magnitude_sps, value.trim_symmetry_sps,
              value.post_recovery_command_variation_sps, value.worst_settling_time_s};
    case TunerRankingStage::Joint:
      return {value.worst_settling_time_s, value.total_pitch_iae_deg_s,
              value.neutral_command_variation_sps, value.total_velocity_iae_sps_s,
              value.total_drive_tracking_mae_sps};
  }
  return {};
}

bool tuning_velocity_scores_equivalent(double score, double best_score) {
  return std::isfinite(score) && std::isfinite(best_score) &&
         score <= best_score * 1.05 + 1e-12;
}

bool tuning_stage_tie_break_less(TunerRankingStage stage, const TunerRankingSummary& left,
                                 const TunerRankingSummary& right) {
  if (stage == TunerRankingStage::Velocity &&
      left.velocity_gain_per_s != right.velocity_gain_per_s) {
    return left.velocity_gain_per_s < right.velocity_gain_per_s;
  }
  return left.score < right.score;
}

std::string tuning_metrics_csv_header() {
  return "peak_pitch_deg,peak_rate_dps,pitch_iae_deg_s,velocity_iae_sps_s,arrest_time_s,"
         "settling_time_s,command_rms_sps,command_total_variation_sps,saturation_time_s,"
         "max_continuous_saturation_s,command_near_rail_time_s,"
         "outer_acceleration_limited_time_s,outer_pitch_target_limited_time_s,rebound_ratio,"
         "rebound_velocity_mps,release_distance_m,drive_tracking_mae_sps,stop_speed_rms_sps,"
         "final_velocity_mean_sps,final_velocity_mean_mps,velocity_feedback_mae_mps,"
         "velocity_feedback_bias_mps,velocity_feedback_rms_mps,"
         "mechanical_velocity_target_iae_m_s,mechanical_velocity_late_error_mps,"
         "mechanical_velocity_peak_mps,mechanical_velocity_direction_fraction,"
         "mechanical_velocity_target_fraction,mechanical_velocity_hold_user_mean_mps,"
         "mechanical_velocity_hold_reference_mean_mps,mechanical_velocity_hold_actual_mean_mps,"
         "mechanical_velocity_hold_abs_error_mps,mechanical_velocity_hold_direction_fraction,"
         "mechanical_velocity_hold_target_fraction,mechanical_velocity_hold_duration_s,"
         "drive_pitch_peak_deg,outer_limit_fraction,"
         "growing_oscillation,settled,safe";
}

std::string tuning_metrics_csv_row(const ScenarioMetrics& m) {
  std::ostringstream output;
  output << std::setprecision(12) << m.peak_pitch_deg << ',' << m.peak_rate_dps << ','
         << m.pitch_iae_deg_s << ',' << m.velocity_iae_sps_s << ',' << m.arrest_time_s << ','
         << m.settling_time_s << ',' << m.command_rms_sps << ',' << m.command_total_variation_sps
         << ',' << m.saturation_time_s << ',' << m.max_continuous_saturation_s << ','
         << m.command_near_rail_time_s << ',' << m.outer_acceleration_limited_time_s << ','
         << m.outer_pitch_target_limited_time_s << ',' << m.rebound_ratio << ','
         << m.rebound_velocity_mps << ',' << m.release_distance_m << ','
         << m.drive_tracking_mae_sps << ',' << m.stop_speed_rms_sps << ','
         << m.final_velocity_mean_sps << ',' << m.final_velocity_mean_mps << ','
         << m.velocity_feedback_mae_mps << ',' << m.velocity_feedback_bias_mps << ','
         << m.velocity_feedback_rms_mps << ',' << m.mechanical_velocity_target_iae_m_s << ','
         << m.mechanical_velocity_late_error_mps << ',' << m.mechanical_velocity_peak_mps << ','
         << m.mechanical_velocity_direction_fraction << ','
         << m.mechanical_velocity_target_fraction << ','
         << m.mechanical_velocity_hold_user_mean_mps << ','
         << m.mechanical_velocity_hold_reference_mean_mps << ','
         << m.mechanical_velocity_hold_actual_mean_mps << ','
         << m.mechanical_velocity_hold_abs_error_mps << ','
         << m.mechanical_velocity_hold_direction_fraction << ','
         << m.mechanical_velocity_hold_target_fraction << ','
         << m.mechanical_velocity_hold_duration_s << ',' << m.drive_pitch_peak_deg << ','
         << m.outer_limit_fraction << ',' << m.growing_oscillation << ',' << m.settled << ','
         << m.safe;
  return output.str();
}
