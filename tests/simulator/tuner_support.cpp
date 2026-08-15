#include "simulator/tuner_support.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

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

  for (size_t index = 0; index < rows.size(); ++index) {
    const auto& row = rows[index];
    const double dt_s = dt_at(rows, index);
    metrics.peak_pitch_deg = std::max(metrics.peak_pitch_deg, std::abs(row.plant_pitch_deg));
    metrics.peak_rate_dps = std::max(metrics.peak_rate_dps, std::abs(row.plant_pitch_rate_dps));
    command_squared += row.u_sps * row.u_sps;
    ++command_count;
    if (index > 0) metrics.command_total_variation_sps += std::abs(row.u_sps - previous_command);
    previous_command = row.u_sps;

    if (row.sim_time_s >= start_s) {
      metrics.pitch_iae_deg_s += std::abs(row.plant_pitch_deg) * dt_s;
      metrics.velocity_iae_sps_s += std::abs(row.corrected_axle_velocity_sps) * dt_s;
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
      left.velocity_damping_per_s != right.velocity_damping_per_s) {
    return left.velocity_damping_per_s < right.velocity_damping_per_s;
  }
  return left.score < right.score;
}

std::string tuning_metrics_csv_header() {
  return "peak_pitch_deg,peak_rate_dps,pitch_iae_deg_s,velocity_iae_sps_s,arrest_time_s,"
         "settling_time_s,command_rms_sps,command_total_variation_sps,saturation_time_s,"
         "max_continuous_saturation_s,rebound_ratio,drive_tracking_mae_sps,"
         "stop_speed_rms_sps,final_velocity_mean_sps,growing_oscillation,settled,safe";
}

std::string tuning_metrics_csv_row(const ScenarioMetrics& m) {
  std::ostringstream output;
  output << std::setprecision(12) << m.peak_pitch_deg << ',' << m.peak_rate_dps << ','
         << m.pitch_iae_deg_s << ',' << m.velocity_iae_sps_s << ',' << m.arrest_time_s << ','
         << m.settling_time_s << ',' << m.command_rms_sps << ',' << m.command_total_variation_sps
         << ',' << m.saturation_time_s << ',' << m.max_continuous_saturation_s << ','
         << m.rebound_ratio << ',' << m.drive_tracking_mae_sps << ',' << m.stop_speed_rms_sps
         << ',' << m.final_velocity_mean_sps << ',' << m.growing_oscillation << ',' << m.settled
         << ',' << m.safe;
  return output.str();
}
