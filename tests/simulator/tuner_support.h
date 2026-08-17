#pragma once

#include <string>
#include <vector>

#include "simulator/simulator_runner.h"

struct ScenarioMetrics {
  double peak_pitch_deg = 0.0;
  double peak_rate_dps = 0.0;
  double pitch_iae_deg_s = 0.0;
  double velocity_iae_sps_s = 0.0;
  double arrest_time_s = 0.0;
  double settling_time_s = 0.0;
  double command_rms_sps = 0.0;
  double command_total_variation_sps = 0.0;
  double saturation_time_s = 0.0;
  double max_continuous_saturation_s = 0.0;
  double command_near_rail_time_s = 0.0;
  double outer_acceleration_limited_time_s = 0.0;
  double outer_pitch_target_limited_time_s = 0.0;
  double rebound_ratio = 0.0;
  double rebound_velocity_mps = 0.0;
  double release_distance_m = 0.0;
  double drive_tracking_mae_sps = 0.0;
  double stop_speed_rms_sps = 0.0;
  double final_velocity_mean_sps = 0.0;
  double final_velocity_mean_mps = 0.0;
  double velocity_feedback_mae_mps = 0.0;
  double velocity_feedback_bias_mps = 0.0;
  double velocity_feedback_rms_mps = 0.0;
  // Mechanical plant velocity is the primary motion objective. The
  // completed-step and feedback estimates remain separate observer metrics.
  double mechanical_velocity_target_iae_m_s = 0.0;
  double mechanical_velocity_late_error_mps = 0.0;
  double mechanical_velocity_peak_mps = 0.0;
  double mechanical_velocity_direction_fraction = 0.0;
  double mechanical_velocity_target_fraction = 0.0;
  // Explicit final pre-release hold-window metrics. These are intentionally
  // separate from the all-active-reference metrics above so post-release
  // settling samples cannot make a weak hold look accurate.
  double mechanical_velocity_hold_user_mean_mps = 0.0;
  double mechanical_velocity_hold_reference_mean_mps = 0.0;
  double mechanical_velocity_hold_actual_mean_mps = 0.0;
  double mechanical_velocity_hold_abs_error_mps = 0.0;
  double mechanical_velocity_hold_direction_fraction = 0.0;
  double mechanical_velocity_hold_target_fraction = 0.0;
  double mechanical_velocity_hold_duration_s = 0.0;
  // Distance-only experiment metrics. These describe the first nonzero
  // joystick segment, not the post-release tail, so the isolated drive
  // search cannot be rewarded by rebound or accumulated wheel travel.
  double active_command_start_s = 0.0;
  double active_command_end_s = 0.0;
  double signed_distance_m = 0.0;
  double reference_distance_m = 0.0;
  double distance_tracking_fraction = 0.0;
  double active_mean_reference_velocity_mps = 0.0;
  double active_mean_mechanical_velocity_mps = 0.0;
  double active_peak_mechanical_velocity_mps = 0.0;
  double active_final_mechanical_velocity_mps = 0.0;
  double active_mean_a_ref_mps2 = 0.0;
  double active_mean_a_p_mps2 = 0.0;
  double active_mean_a_i_mps2 = 0.0;
  double active_peak_a_ref_mps2 = 0.0;
  double active_peak_a_p_mps2 = 0.0;
  double active_peak_a_i_mps2 = 0.0;
  bool active_distance_valid = false;
  double drive_pitch_peak_deg = 0.0;
  double outer_limit_fraction = 0.0;
  bool growing_oscillation = false;
  bool settled = false;
  bool safe = false;
};

enum class TunerRankingStage { Inner, Authority, Velocity, Drive, Trim, Joint };

struct TunerRankingSummary {
  double score = 0.0;
  double worst_settling_time_s = 0.0;
  double total_pitch_iae_deg_s = 0.0;
  double worst_peak_pitch_deg = 0.0;
  double worst_peak_rate_dps = 0.0;
  double neutral_command_variation_sps = 0.0;
  double worst_arrest_time_s = 0.0;
  double worst_rebound_ratio = 0.0;
  double max_continuous_saturation_s = 0.0;
  double residual_velocity_sps = 0.0;
  double post_recovery_command_variation_sps = 0.0;
  double total_velocity_iae_sps_s = 0.0;
  double total_drive_tracking_mae_sps = 0.0;
  double total_stop_speed_rms_sps = 0.0;
  double total_velocity_feedback_mae_mps = 0.0;
  double total_release_distance_m = 0.0;
  double total_rebound_velocity_mps = 0.0;
  double velocity_gain_per_s = 0.0;
  double trim_speed_magnitude_sps = 0.0;
  double trim_symmetry_sps = 0.0;
};

ScenarioMetrics calculate_tuning_metrics(const SimulatorRunResult& result);
double normalized_tuning_metric(double value, double baseline);
bool tuning_metrics_dominate(const std::vector<double>& left, const std::vector<double>& right);
std::vector<double> tuning_pareto_objectives(TunerRankingStage stage,
                                             const TunerRankingSummary& summary);
bool tuning_velocity_scores_equivalent(double score, double best_score);
bool tuning_stage_tie_break_less(TunerRankingStage stage, const TunerRankingSummary& left,
                                 const TunerRankingSummary& right);
std::string tuning_metrics_csv_header();
std::string tuning_metrics_csv_row(const ScenarioMetrics& metrics);
