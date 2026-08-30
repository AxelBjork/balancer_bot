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

double rms(const std::vector<double>& values) {
  if (values.empty()) return 0.0;
  double squared = 0.0;
  for (const double value : values) squared += value * value;
  return std::sqrt(squared / static_cast<double>(values.size()));
}

double percentile(std::vector<double> values, double fraction) {
  if (values.empty()) return 0.0;
  fraction = std::clamp(fraction, 0.0, 1.0);
  std::sort(values.begin(), values.end());
  const double position = fraction * static_cast<double>(values.size() - 1);
  const size_t lower = static_cast<size_t>(position);
  const size_t upper = std::min(values.size() - 1, lower + 1);
  const double weight = position - static_cast<double>(lower);
  return values[lower] + weight * (values[upper] - values[lower]);
}

void calculate_slew_metrics(const SimulatorRunResult& result, ScenarioMetrics& metrics) {
  const auto& rows = result.rows;
  if (rows.size() < 2) return;

  std::vector<double> requested_deltas;
  std::vector<double> pitch_error_deltas;
  std::vector<double> pitch_rate_deltas;
  std::vector<double> pitch_accel_deltas;
  std::vector<double> pitch_target_deltas;
  std::vector<double> requested_applied_errors;
  requested_deltas.reserve(rows.size() - 1);
  pitch_error_deltas.reserve(rows.size() - 1);
  pitch_rate_deltas.reserve(rows.size() - 1);
  pitch_accel_deltas.reserve(rows.size() - 1);
  pitch_target_deltas.reserve(rows.size() - 1);
  requested_applied_errors.reserve(rows.size() - 1);

  size_t slew_active_count = 0;
  size_t longest_slew_frames = 0;
  size_t current_slew_frames = 0;
  double previous_pitch_rad = rows.front().pitch_deg * M_PI / 180.0;
  double previous_target_rad = rows.front().final_pitch_target_deg * M_PI / 180.0;
  double previous_rate_term = rows.front().pitch_rate_feedback_sps;
  double previous_accel_term = rows.front().pitch_accel_feedback_sps;
  for (size_t index = 1; index < rows.size(); ++index) {
    const auto& previous = rows[index - 1];
    const auto& row = rows[index];
    const double pitch_rad = row.pitch_deg * M_PI / 180.0;
    const double target_rad = row.final_pitch_target_deg * M_PI / 180.0;
    const double pitch_error_delta =
        row.active_pitch_gain_sps_per_rad * (pitch_rad - previous_pitch_rad);
    const double pitch_rate_delta = row.pitch_rate_feedback_sps - previous_rate_term;
    const double pitch_accel_delta = row.pitch_accel_feedback_sps - previous_accel_term;
    const double pitch_target_delta =
        -row.active_pitch_gain_sps_per_rad * (target_rad - previous_target_rad);
    const double requested_delta = row.u_sps - previous.u_sps;
    const double requested_applied_error = std::max(
        std::abs(row.left_sps - row.left_slewed_sps),
        std::abs(row.right_sps - row.right_slewed_sps));

    requested_deltas.push_back(std::abs(requested_delta));
    pitch_error_deltas.push_back(std::abs(pitch_error_delta));
    pitch_rate_deltas.push_back(std::abs(pitch_rate_delta));
    pitch_accel_deltas.push_back(std::abs(pitch_accel_delta));
    pitch_target_deltas.push_back(std::abs(pitch_target_delta));
    requested_applied_errors.push_back(requested_applied_error);
    metrics.integrated_slew_excess_sps +=
        std::max(0.0, std::abs(requested_delta) -
                          Config::motor_slew_sps_per_s / Config::control_hz);

    const bool slew_active = requested_applied_error > 1e-9 ||
                             (row.actuator_saturation_flags &
                              (ActuatorSaturationLeftSlew | ActuatorSaturationRightSlew));
    if (slew_active) {
      ++slew_active_count;
      ++current_slew_frames;
      longest_slew_frames = std::max(longest_slew_frames, current_slew_frames);
    } else {
      current_slew_frames = 0;
    }
    previous_pitch_rad = pitch_rad;
    previous_target_rad = target_rad;
    previous_rate_term = row.pitch_rate_feedback_sps;
    previous_accel_term = row.pitch_accel_feedback_sps;
  }

  const double frame_count = static_cast<double>(rows.size() - 1);
  metrics.slew_active_fraction = static_cast<double>(slew_active_count) / frame_count;
  metrics.longest_slew_limited_interval_s =
      static_cast<double>(longest_slew_frames) * dt_at(rows, 1);
  metrics.requested_applied_error_rms_sps = rms(requested_applied_errors);
  metrics.requested_applied_error_p95_sps = percentile(requested_applied_errors, 0.95);
  metrics.requested_applied_error_peak_sps =
      *std::max_element(requested_applied_errors.begin(), requested_applied_errors.end());
  metrics.requested_command_delta_p95_sps = percentile(requested_deltas, 0.95);
  metrics.requested_command_delta_p99_sps = percentile(requested_deltas, 0.99);
  metrics.requested_command_delta_peak_sps =
      *std::max_element(requested_deltas.begin(), requested_deltas.end());
  metrics.pitch_error_command_delta_p95_sps = percentile(pitch_error_deltas, 0.95);
  metrics.pitch_error_command_delta_p99_sps = percentile(pitch_error_deltas, 0.99);
  metrics.pitch_error_command_delta_peak_sps =
      *std::max_element(pitch_error_deltas.begin(), pitch_error_deltas.end());
  metrics.pitch_rate_command_delta_p95_sps = percentile(pitch_rate_deltas, 0.95);
  metrics.pitch_rate_command_delta_p99_sps = percentile(pitch_rate_deltas, 0.99);
  metrics.pitch_rate_command_delta_peak_sps =
      *std::max_element(pitch_rate_deltas.begin(), pitch_rate_deltas.end());
  metrics.pitch_accel_command_delta_p95_sps = percentile(pitch_accel_deltas, 0.95);
  metrics.pitch_accel_command_delta_p99_sps = percentile(pitch_accel_deltas, 0.99);
  metrics.pitch_accel_command_delta_peak_sps =
      *std::max_element(pitch_accel_deltas.begin(), pitch_accel_deltas.end());
  metrics.pitch_target_command_delta_p95_sps = percentile(pitch_target_deltas, 0.95);
  metrics.pitch_target_command_delta_p99_sps = percentile(pitch_target_deltas, 0.99);
  metrics.pitch_target_command_delta_peak_sps =
      *std::max_element(pitch_target_deltas.begin(), pitch_target_deltas.end());
}

struct ActiveCommandInterval {
  double start_s = 0.0;
  double end_s = 0.0;
  bool valid = false;
};

ActiveCommandInterval first_active_command_interval(const SimulatorRunResult& result) {
  if (result.scenario.name.rfind("speed_envelope_", 0) == 0) {
    double start_s = std::numeric_limits<double>::infinity();
    double end_s = 0.0;
    for (const auto& segment : result.scenario.joy_segments) {
      if (segment.duration_s <= 0.0 ||
          (std::abs(segment.forward) <= 1e-9 && std::abs(segment.forward_end) <= 1e-9)) {
        continue;
      }
      start_s = std::min(start_s, segment.start_s);
      end_s = std::max(end_s, std::min(result.scenario.duration_s,
                                       segment.start_s + segment.duration_s));
    }
    if (std::isfinite(start_s) && end_s > start_s + 1e-9) {
      return {.start_s = start_s, .end_s = end_s, .valid = true};
    }
    return {};
  }

  for (const auto& segment : result.scenario.joy_segments) {
    if (segment.duration_s <= 0.0 || std::abs(segment.forward) <= 1e-9) continue;
    double end_s = std::min(result.scenario.duration_s, segment.start_s + segment.duration_s);
    for (const auto& other : result.scenario.joy_segments) {
      if (other.start_s > segment.start_s + 1e-9) end_s = std::min(end_s, other.start_s);
    }
    if (end_s > segment.start_s + 1e-9) {
      return {.start_s = segment.start_s, .end_s = end_s, .valid = true};
    }
  }
  return {};
}

template <typename Selector>
double sample_at(const std::vector<SimulatorTimelineRow>& rows, double time_s,
                 Selector selector) {
  if (rows.empty()) return std::numeric_limits<double>::quiet_NaN();
  if (time_s <= rows.front().sim_time_s) return selector(rows.front());
  if (time_s >= rows.back().sim_time_s) return selector(rows.back());
  const auto upper = std::lower_bound(
      rows.begin(), rows.end(), time_s,
      [](const SimulatorTimelineRow& row, double value) { return row.sim_time_s < value; });
  if (upper == rows.begin()) return selector(*upper);
  const auto& before = *(upper - 1);
  const double span_s = upper->sim_time_s - before.sim_time_s;
  if (span_s <= 0.0) return selector(*upper);
  const double fraction = (time_s - before.sim_time_s) / span_s;
  return selector(before) + fraction * (selector(*upper) - selector(before));
}

template <typename Selector>
double integrate_interval(const std::vector<SimulatorTimelineRow>& rows, double start_s,
                          double end_s, Selector selector) {
  double integral = 0.0;
  if (rows.size() < 2 || end_s <= start_s) return integral;
  for (size_t index = 1; index < rows.size(); ++index) {
    const auto& before = rows[index - 1];
    const auto& after = rows[index];
    const double overlap_start = std::max(start_s, before.sim_time_s);
    const double overlap_end = std::min(end_s, after.sim_time_s);
    if (overlap_end <= overlap_start) continue;
    const double span_s = after.sim_time_s - before.sim_time_s;
    if (span_s <= 0.0) continue;
    const auto interpolate = [&](double time_s) {
      const double fraction = (time_s - before.sim_time_s) / span_s;
      return selector(before) + fraction * (selector(after) - selector(before));
    };
    integral += 0.5 * (interpolate(overlap_start) + interpolate(overlap_end)) *
                (overlap_end - overlap_start);
  }
  return integral;
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

void calculate_active_distance_metrics(const SimulatorRunResult& result,
                                       ScenarioMetrics& metrics) {
  const auto interval = first_active_command_interval(result);
  if (!interval.valid || result.rows.size() < 2) return;

  const auto& rows = result.rows;
  const double duration_s = interval.end_s - interval.start_s;
  const auto position = [](const SimulatorTimelineRow& row) { return row.plant_position; };
  const auto reference_velocity =
      [](const SimulatorTimelineRow& row) { return row.reference_velocity_mps; };
  const auto actual_velocity = [](const SimulatorTimelineRow& row) { return row.plant_velocity; };
  const auto reference_acceleration =
      [](const SimulatorTimelineRow& row) { return row.reference_acceleration_mps2; };
  const auto p_acceleration =
      [](const SimulatorTimelineRow& row) { return row.velocity_p_acceleration_mps2; };
  const auto i_acceleration =
      [](const SimulatorTimelineRow& row) { return row.velocity_i_acceleration_mps2; };

  metrics.active_command_start_s = interval.start_s;
  metrics.active_command_end_s = interval.end_s;
  const double start_position = sample_at(rows, interval.start_s, position);
  const double end_position = sample_at(rows, interval.end_s, position);
  metrics.signed_distance_m = end_position - start_position;
  metrics.reference_distance_m =
      integrate_interval(rows, interval.start_s, interval.end_s, reference_velocity);
  metrics.distance_tracking_fraction =
      std::abs(metrics.reference_distance_m) > 1e-9
          ? metrics.signed_distance_m / metrics.reference_distance_m
          : 0.0;
  metrics.active_mean_reference_velocity_mps =
      integrate_interval(rows, interval.start_s, interval.end_s, reference_velocity) / duration_s;
  metrics.active_mean_mechanical_velocity_mps =
      integrate_interval(rows, interval.start_s, interval.end_s, actual_velocity) / duration_s;
  metrics.active_final_mechanical_velocity_mps = sample_at(rows, interval.end_s, actual_velocity);
  metrics.active_mean_a_ref_mps2 =
      integrate_interval(rows, interval.start_s, interval.end_s, reference_acceleration) /
      duration_s;
  metrics.active_mean_a_p_mps2 =
      integrate_interval(rows, interval.start_s, interval.end_s, p_acceleration) / duration_s;
  metrics.active_mean_a_i_mps2 =
      integrate_interval(rows, interval.start_s, interval.end_s, i_acceleration) / duration_s;
  metrics.active_peak_mechanical_velocity_mps = 0.0;
  metrics.active_peak_a_ref_mps2 = 0.0;
  metrics.active_peak_a_p_mps2 = 0.0;
  metrics.active_peak_a_i_mps2 = 0.0;
  for (const auto& row : rows) {
    if (row.sim_time_s < interval.start_s || row.sim_time_s > interval.end_s) continue;
    metrics.active_peak_mechanical_velocity_mps =
        std::max(metrics.active_peak_mechanical_velocity_mps, std::abs(row.plant_velocity));
    metrics.active_peak_a_ref_mps2 =
        std::max(metrics.active_peak_a_ref_mps2, std::abs(row.reference_acceleration_mps2));
    metrics.active_peak_a_p_mps2 =
        std::max(metrics.active_peak_a_p_mps2, std::abs(row.velocity_p_acceleration_mps2));
    metrics.active_peak_a_i_mps2 =
        std::max(metrics.active_peak_a_i_mps2, std::abs(row.velocity_i_acceleration_mps2));
  }
  metrics.active_distance_valid =
      std::isfinite(metrics.signed_distance_m) && std::isfinite(metrics.reference_distance_m) &&
      std::isfinite(metrics.distance_tracking_fraction) &&
      std::isfinite(metrics.active_mean_mechanical_velocity_mps) &&
      std::isfinite(metrics.active_final_mechanical_velocity_mps) &&
      std::abs(metrics.reference_distance_m) > 1e-9;
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
    const double configured_balance_limit =
        std::max(1.0, std::abs(ConfigPid::values.balance_max_sps));
    if (std::abs(row.u_sps) >= 0.95 * configured_balance_limit) {
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
  calculate_active_distance_metrics(result, metrics);
  calculate_slew_metrics(result, metrics);
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
         "active_command_start_s,active_command_end_s,signed_distance_m,reference_distance_m,"
         "distance_tracking_fraction,active_mean_reference_velocity_mps,"
         "active_mean_mechanical_velocity_mps,active_peak_mechanical_velocity_mps,"
         "active_final_mechanical_velocity_mps,active_mean_a_ref_mps2,active_mean_a_p_mps2,"
         "active_mean_a_i_mps2,active_peak_a_ref_mps2,active_peak_a_p_mps2,"
         "active_peak_a_i_mps2,active_distance_valid,"
         "drive_pitch_peak_deg,outer_limit_fraction,"
         "slew_active_fraction,requested_applied_error_rms_sps,"
         "requested_applied_error_p95_sps,requested_applied_error_peak_sps,"
         "requested_command_delta_p95_sps,requested_command_delta_p99_sps,"
         "requested_command_delta_peak_sps,pitch_error_command_delta_p95_sps,"
         "pitch_error_command_delta_p99_sps,pitch_error_command_delta_peak_sps,"
         "pitch_rate_command_delta_p95_sps,pitch_rate_command_delta_p99_sps,"
         "pitch_rate_command_delta_peak_sps,pitch_accel_command_delta_p95_sps,"
         "pitch_accel_command_delta_p99_sps,pitch_accel_command_delta_peak_sps,"
         "pitch_target_command_delta_p95_sps,pitch_target_command_delta_p99_sps,"
         "pitch_target_command_delta_peak_sps,integrated_slew_excess_sps,"
         "longest_slew_limited_interval_s,"
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
         << m.mechanical_velocity_hold_duration_s << ',' << m.active_command_start_s << ','
         << m.active_command_end_s << ',' << m.signed_distance_m << ','
         << m.reference_distance_m << ',' << m.distance_tracking_fraction << ','
         << m.active_mean_reference_velocity_mps << ','
         << m.active_mean_mechanical_velocity_mps << ','
         << m.active_peak_mechanical_velocity_mps << ','
         << m.active_final_mechanical_velocity_mps << ',' << m.active_mean_a_ref_mps2 << ','
         << m.active_mean_a_p_mps2 << ',' << m.active_mean_a_i_mps2 << ','
         << m.active_peak_a_ref_mps2 << ',' << m.active_peak_a_p_mps2 << ','
         << m.active_peak_a_i_mps2 << ',' << m.active_distance_valid << ','
         << m.drive_pitch_peak_deg << ',' << m.outer_limit_fraction << ','
         << m.slew_active_fraction << ',' << m.requested_applied_error_rms_sps << ','
         << m.requested_applied_error_p95_sps << ',' << m.requested_applied_error_peak_sps << ','
         << m.requested_command_delta_p95_sps << ',' << m.requested_command_delta_p99_sps << ','
         << m.requested_command_delta_peak_sps << ','
         << m.pitch_error_command_delta_p95_sps << ','
         << m.pitch_error_command_delta_p99_sps << ','
         << m.pitch_error_command_delta_peak_sps << ','
         << m.pitch_rate_command_delta_p95_sps << ','
         << m.pitch_rate_command_delta_p99_sps << ','
         << m.pitch_rate_command_delta_peak_sps << ','
         << m.pitch_accel_command_delta_p95_sps << ','
         << m.pitch_accel_command_delta_p99_sps << ','
         << m.pitch_accel_command_delta_peak_sps << ','
         << m.pitch_target_command_delta_p95_sps << ','
         << m.pitch_target_command_delta_p99_sps << ','
         << m.pitch_target_command_delta_peak_sps << ','
         << m.integrated_slew_excess_sps << ','
         << m.longest_slew_limited_interval_s << ','
         << m.growing_oscillation << ',' << m.settled << ','
         << m.safe;
  return output.str();
}
