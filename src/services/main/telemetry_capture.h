#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ios>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "messages/balancer_msgs.h"

namespace sil {

struct TelemetryCaptureOptions {
  std::filesystem::path output_dir;
  std::string run_id = "run";
  std::string mode = "real_app";
  std::string pid_profile = "pid.conf";
  double run_seconds = 0.0;
  std::string binary_path;
  std::string working_directory;
};

class TelemetryCapture {
 public:
  explicit TelemetryCapture(TelemetryCaptureOptions options)
      : options_(std::move(options)),
        started_at_(std::chrono::steady_clock::now()) {
    std::filesystem::create_directories(options_.output_dir);
    write_metadata_json();
  }

  void record(const ipc::SystemTelemetryPayload& p) {
    samples_.push_back(p);
  }

  void finish(std::string_view reason, int returncode) {
    if (finished_) {
      return;
    }
    finished_ = true;
    write_csv();
    write_summary_json(reason, returncode);
    write_done_json(reason, returncode);
  }

  [[nodiscard]] std::size_t sample_count() const {
    return samples_.size();
  }

 private:
  static constexpr std::array<const char*, 44> kColumns = {
      "sim_time_s",
      "pitch_deg",
      "pitch_rate_dps",
      "filtered_pitch_rate_dps",
      "raw_acc_pitch_deg",
      "fused_pitch_deg",
      "gyro_pitch_rate_dps",
      "pitch_sp_deg",
      "rate_sp_dps",
      "u_sps",
      "left_sps",
      "right_sps",
      "vel_error",
      "vel_i_term",
      "vel_p_term",
      "target_vel_sps",
      "measured_vel_sps",
      "filtered_vel_sps",
      "pitch_ref_from_vel_deg",
      "pitch_error_deg",
      "rate_error_dps",
      "out_norm",
      "effective_pitch_sp_deg",
      "pitch_trim_deg",
      "trim_active",
      "left_applied_sps",
      "right_applied_sps",
      "left_actual_steps",
      "right_actual_steps",
      "plant_pitch_deg",
      "plant_pitch_rate_dps",
      "plant_position",
      "plant_velocity",
      "target_wheel_velocity",
      "actual_wheel_velocity",
      "velocity_error",
      "f_cmd",
      "f_app",
      "external_force_n",
      "external_com_bias_rad",
      "x_ddot",
      "theta_ddot",
      "command_saturated",
      "force_saturated",
  };

  static std::string json_escape(std::string_view value) {
    std::string out;
    out.reserve(value.size() + 8);
    for (char ch : value) {
      switch (ch) {
        case '\\':
          out += "\\\\";
          break;
        case '"':
          out += "\\\"";
          break;
        case '\n':
          out += "\\n";
          break;
        case '\r':
          out += "\\r";
          break;
        case '\t':
          out += "\\t";
          break;
        default:
          out.push_back(ch);
          break;
      }
    }
    return out;
  }

  static void write_json_string(std::ostream& os, std::string_view value) {
    os << '"' << json_escape(value) << '"';
  }

  static void write_json_number(std::ostream& os, double value) {
    os << std::setprecision(17) << value;
  }

  static void write_json_integer(std::ostream& os, std::int64_t value) {
    os << value;
  }

  static void write_json_bool(std::ostream& os, bool value) {
    os << (value ? "true" : "false");
  }

  template <typename T>
  static void write_json_key_value(std::ostream& os, std::string_view key, const T& value) {
    write_json_string(os, key);
    os << ':';
    write_json_value(os, value);
  }

  static void write_json_value(std::ostream& os, const std::string& value) {
    write_json_string(os, value);
  }

  static void write_json_value(std::ostream& os, std::string_view value) {
    write_json_string(os, value);
  }

  static void write_json_value(std::ostream& os, const char* value) {
    write_json_string(os, value ? value : "");
  }

  static void write_json_value(std::ostream& os, bool value) {
    write_json_bool(os, value);
  }

  static void write_json_value(std::ostream& os, double value) {
    if (std::isfinite(value)) {
      write_json_number(os, value);
    } else {
      os << "null";
    }
  }

  static void write_json_value(std::ostream& os, std::int64_t value) {
    write_json_integer(os, value);
  }

  static void write_json_value(std::ostream& os, std::size_t value) {
    os << value;
  }

  template <typename T>
  static void write_json_value(std::ostream& os, const std::optional<T>& value) {
    if (value.has_value()) {
      write_json_value(os, *value);
    } else {
      os << "null";
    }
  }

  static void write_json_value(std::ostream& os, std::nullptr_t) {
    os << "null";
  }

  template <typename T>
  static void write_json_array(std::ostream& os, const std::vector<T>& values) {
    os << '[';
    for (std::size_t i = 0; i < values.size(); ++i) {
      if (i != 0) {
        os << ',';
      }
      write_json_value(os, values[i]);
    }
    os << ']';
  }

  static std::vector<double> series(const std::vector<ipc::SystemTelemetryPayload>& rows,
                                    double ipc::SystemTelemetryPayload::*field) {
    std::vector<double> out;
    out.reserve(rows.size());
    for (const auto& row : rows) {
      out.push_back(row.*field);
    }
    return out;
  }

  static std::vector<double> time_series(const std::vector<ipc::SystemTelemetryPayload>& rows) {
    return series(rows, &ipc::SystemTelemetryPayload::t_sec);
  }

  static double max_abs(const std::vector<double>& values) {
    double out = 0.0;
    for (double value : values) {
      out = std::max(out, std::abs(value));
    }
    return out;
  }

  static double mean_abs(const std::vector<double>& values) {
    if (values.empty()) {
      return 0.0;
    }
    double sum = 0.0;
    for (double value : values) {
      sum += std::abs(value);
    }
    return sum / static_cast<double>(values.size());
  }

  static double rms(const std::vector<double>& values) {
    if (values.empty()) {
      return 0.0;
    }
    double sum = 0.0;
    for (double value : values) {
      sum += value * value;
    }
    return std::sqrt(sum / static_cast<double>(values.size()));
  }

  static std::optional<double> median(std::vector<double> values) {
    if (values.empty()) {
      return std::nullopt;
    }
    std::sort(values.begin(), values.end());
    const std::size_t mid = values.size() / 2;
    if ((values.size() % 2) == 0) {
      return 0.5 * (values[mid - 1] + values[mid]);
    }
    return values[mid];
  }

  static std::optional<double> max_value(const std::vector<double>& values) {
    if (values.empty()) {
      return std::nullopt;
    }
    return *std::max_element(values.begin(), values.end());
  }

  static std::optional<double> min_value(const std::vector<double>& values) {
    if (values.empty()) {
      return std::nullopt;
    }
    return *std::min_element(values.begin(), values.end());
  }

  static void write_csv_header(std::ostream& os) {
    for (std::size_t i = 0; i < kColumns.size(); ++i) {
      if (i != 0) {
        os << ',';
      }
      os << kColumns[i];
    }
    os << '\n';
  }

  static void write_csv_row(std::ostream& os, const ipc::SystemTelemetryPayload& row) {
    os << std::setprecision(17) << std::defaultfloat;
    os << row.t_sec << ','
       << row.pitch_deg << ','
       << row.pitch_rate_dps << ','
       << row.filtered_pitch_rate_dps << ','
       << row.raw_acc_pitch_deg << ','
       << row.fused_pitch_deg << ','
       << row.gyro_pitch_rate_dps << ','
       << row.pitch_sp_deg << ','
       << 0.0 << ','
       << row.u_sps << ','
       << row.left_applied_sps << ','
       << row.right_applied_sps << ','
       << row.vel_error << ','
       << 0.0 << ','
       << row.vel_p_term << ','
       << 0.0 << ','
       << row.measured_vel_sps << ','
       << 0.0 << ','
       << row.pitch_ref_from_vel_deg << ','
       << row.pitch_error_deg << ','
       << 0.0 << ','
       << 0.0 << ','
       << 0.0 << ','
       << row.pitch_trim_deg << ','
       << row.trim_active << ','
       << row.left_applied_sps << ','
       << row.right_applied_sps << ','
       << row.left_actual_steps << ','
       << row.right_actual_steps << ','
       << row.plant_pitch_deg << ','
       << row.plant_pitch_rate_dps << ','
       << row.plant_position_m << ','
       << row.plant_velocity_mps << ','
       << row.target_wheel_velocity << ','
       << row.actual_wheel_velocity << ','
       << row.plant_velocity_error << ','
       << row.f_cmd << ','
       << row.f_app << ','
       << row.external_force_n << ','
       << row.external_com_bias_rad << ','
       << row.x_ddot << ','
       << row.theta_ddot << ','
       << 0.0 << ','
       << row.force_saturated << '\n';
  }

  void write_csv() const {
    std::ofstream csv(options_.output_dir / "timeline.csv");
    write_csv_header(csv);
    for (const auto& row : samples_) {
      write_csv_row(csv, row);
    }
  }

  void write_metadata_json() const {
    std::ofstream out(options_.output_dir / "metadata.json");
    out << "{\n";
    write_json_key_value(out, "run_id", options_.run_id);
    out << ",\n";
    write_json_key_value(out, "mode", options_.mode);
    out << ",\n";
    write_json_key_value(out, "pid_profile", options_.pid_profile);
    out << ",\n";
    write_json_key_value(out, "duration_s", options_.run_seconds);
    out << ",\n";
    write_json_key_value(out, "binary_path", options_.binary_path);
    out << ",\n";
    write_json_key_value(out, "working_directory", options_.working_directory);
    out << "\n}\n";
  }

  void write_done_json(std::string_view reason, int returncode) const {
    std::ofstream out(options_.output_dir / "done.json");
    out << "{\n";
    write_json_key_value(out, "run_id", options_.run_id);
    out << ",\n";
    write_json_key_value(out, "reason", reason);
    out << ",\n";
    write_json_key_value(out, "returncode", static_cast<std::int64_t>(returncode));
    out << ",\n";
    write_json_key_value(out, "sample_count", samples_.size());
    out << ",\n";
    write_json_key_value(out, "elapsed_s", elapsed_s());
    out << ",\n";
    write_json_key_value(out, "final_pitch_deg",
                         final_field(&ipc::SystemTelemetryPayload::plant_pitch_deg));
    out << ",\n";
    write_json_key_value(out, "max_abs_pitch_deg", max_abs_pitch());
    out << ",\n";
    write_json_key_value(out, "tail_rms_pitch_deg", tail_rms_pitch());
    out << ",\n";
    write_json_key_value(out, "tail_rail_fraction", tail_rail_fraction());
    out << ",\n";
    write_json_key_value(out, "tail_mean_abs_pitch_deg", tail_mean_abs_pitch());
    out << ",\n";
    write_json_key_value(out, "max_abs_position_m", max_abs_position());
    out << ",\n";
    write_json_key_value(out, "tail_mean_abs_velocity_mps", tail_mean_abs_velocity());
    out << "\n}\n";
  }

  void write_summary_json(std::string_view reason, int returncode) const {
    std::ofstream out(options_.output_dir / "summary.json");
    out << "{\n";
    write_json_key_value(out, "run_id", options_.run_id);
    out << ",\n";
    write_json_key_value(out, "scenario", options_.run_id);
    out << ",\n";
    write_json_key_value(out, "mode", options_.mode);
    out << ",\n";
    write_json_key_value(out, "sample_count", samples_.size());
    out << ",\n";
    write_json_key_value(out, "duration_s", duration_s());
    out << ",\n";
    write_json_key_value(out, "final_pitch_deg",
                         final_field(&ipc::SystemTelemetryPayload::plant_pitch_deg));
    out << ",\n";
    write_json_key_value(out, "max_abs_pitch_deg", max_abs_pitch());
    out << ",\n";
    write_json_key_value(out, "max_abs_u_sps", max_abs_u());
    out << ",\n";
    write_json_key_value(out, "fell", fell());
    out << ",\n";
    write_json_key_value(out, "final_position_m", final_field(&ipc::SystemTelemetryPayload::plant_position_m));
    out << ",\n";
    write_json_key_value(out, "max_abs_position_m", max_abs_position());
    out << ",\n";
    write_json_key_value(out, "max_abs_target_wheel_velocity", max_abs_target_wheel_velocity());
    out << ",\n";
    write_json_key_value(out, "max_abs_actual_wheel_velocity", max_abs_actual_wheel_velocity());
    out << ",\n";
    write_json_key_value(out, "max_abs_f_app", max_abs_f_app());
    out << ",\n";
    write_json_key_value(out, "max_abs_theta_ddot", max_abs_theta_ddot());
    out << ",\n";
    write_json_key_value(out, "tail_rms_pitch_deg", tail_rms_pitch());
    out << ",\n";
    write_json_key_value(out, "tail_mean_abs_pitch_deg", tail_mean_abs_pitch());
    out << ",\n";
    write_json_key_value(out, "tail_mean_abs_velocity_mps", tail_mean_abs_velocity());
    out << ",\n";
    write_json_key_value(out, "tail_command_rail_fraction", tail_command_rail_fraction());
    out << ",\n";
    write_json_key_value(out, "tail_rail_fraction", tail_rail_fraction());
    out << ",\n";
    write_json_key_value(out, "settled_at_s", settled_at_s());
    out << ",\n";
    write_json_key_value(out, "dt_median_s", dt_median_s());
    out << ",\n";
    write_json_key_value(out, "dt_max_s", dt_max_s());
    out << ",\n";
    write_json_key_value(out, "telemetry_continuous", telemetry_continuous());
    out << ",\n";
    write_json_key_value(out, "reason", reason);
    out << ",\n";
    write_json_key_value(out, "returncode", static_cast<std::int64_t>(returncode));
    out << ",\n";
    write_json_key_value(out, "pitch_deg_min",
                         min_value(series(samples_, &ipc::SystemTelemetryPayload::plant_pitch_deg)));
    out << ",\n";
    write_json_key_value(out, "pitch_deg_max",
                         max_value(series(samples_, &ipc::SystemTelemetryPayload::plant_pitch_deg)));
    out << ",\n";
    write_json_key_value(out, "pitch_rate_dps_min",
                         min_value(series(samples_, &ipc::SystemTelemetryPayload::pitch_rate_dps)));
    out << ",\n";
    write_json_key_value(out, "pitch_rate_dps_max",
                         max_value(series(samples_, &ipc::SystemTelemetryPayload::pitch_rate_dps)));
    out << ",\n";
    write_json_key_value(out, "u_sps_min", min_value(series(samples_, &ipc::SystemTelemetryPayload::u_sps)));
    out << ",\n";
    write_json_key_value(out, "u_sps_max", max_value(series(samples_, &ipc::SystemTelemetryPayload::u_sps)));
    out << ",\n";
    write_json_key_value(out, "vel_error_min",
                         min_value(series(samples_, &ipc::SystemTelemetryPayload::vel_error)));
    out << ",\n";
    write_json_key_value(out, "vel_error_max",
                         max_value(series(samples_, &ipc::SystemTelemetryPayload::vel_error)));
    out << ",\n";
    write_json_key_value(out, "rate_sp_dps_min", 0.0);
    out << ",\n";
    write_json_key_value(out, "rate_sp_dps_max", 0.0);
    out << ",\n";
    write_json_key_value(out, "f_cmd_min", min_value(series(samples_, &ipc::SystemTelemetryPayload::f_cmd)));
    out << ",\n";
    write_json_key_value(out, "f_cmd_max", max_value(series(samples_, &ipc::SystemTelemetryPayload::f_cmd)));
    out << ",\n";
    write_json_key_value(out, "f_app_min", min_value(series(samples_, &ipc::SystemTelemetryPayload::f_app)));
    out << ",\n";
    write_json_key_value(out, "f_app_max", max_value(series(samples_, &ipc::SystemTelemetryPayload::f_app)));
    out << "\n}\n";
  }

  double final_field(double ipc::SystemTelemetryPayload::*field) const {
    if (samples_.empty()) {
      return 0.0;
    }
    return samples_.back().*field;
  }

  double duration_s() const {
    if (samples_.size() < 2) {
      return 0.0;
    }
    return samples_.back().t_sec - samples_.front().t_sec;
  }

  double elapsed_s() const {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at_).count();
  }

  double max_abs_pitch() const {
    return max_abs(series(samples_, &ipc::SystemTelemetryPayload::plant_pitch_deg));
  }

  double max_abs_u() const {
    return max_abs(series(samples_, &ipc::SystemTelemetryPayload::u_sps));
  }

  double max_abs_position() const {
    return max_abs(series(samples_, &ipc::SystemTelemetryPayload::plant_position_m));
  }

  double max_abs_target_wheel_velocity() const {
    return max_abs(series(samples_, &ipc::SystemTelemetryPayload::target_wheel_velocity));
  }

  double max_abs_actual_wheel_velocity() const {
    return max_abs(series(samples_, &ipc::SystemTelemetryPayload::actual_wheel_velocity));
  }

  double max_abs_f_app() const {
    return max_abs(series(samples_, &ipc::SystemTelemetryPayload::f_app));
  }

  double max_abs_theta_ddot() const {
    return max_abs(series(samples_, &ipc::SystemTelemetryPayload::theta_ddot));
  }

  double tail_rms_pitch() const {
    return rms(tail_series(&ipc::SystemTelemetryPayload::plant_pitch_deg));
  }

  double tail_mean_abs_pitch() const {
    return mean_abs(tail_series(&ipc::SystemTelemetryPayload::plant_pitch_deg));
  }

  double tail_mean_abs_velocity() const {
    return mean_abs(tail_series(&ipc::SystemTelemetryPayload::plant_velocity_mps));
  }

  double tail_command_rail_fraction() const {
    const auto tail = tail_series(&ipc::SystemTelemetryPayload::force_saturated);
    if (tail.empty()) {
      return 0.0;
    }
    std::size_t count = 0;
    for (double value : tail) {
      if (value >= 0.5) {
        ++count;
      }
    }
    return static_cast<double>(count) / static_cast<double>(tail.size());
  }

  double tail_rail_fraction() const {
    const auto tail = tail_series(&ipc::SystemTelemetryPayload::force_saturated);
    if (tail.empty()) {
      return tail_command_rail_fraction();
    }
    std::size_t count = 0;
    for (double value : tail) {
      if (value >= 0.5) {
        ++count;
      }
    }
    return static_cast<double>(count) / static_cast<double>(tail.size());
  }

  std::optional<double> settled_at_s() const {
    if (samples_.empty()) {
      return std::nullopt;
    }
    std::vector<double> running_max_abs(samples_.size(), 0.0);
    double acc = 0.0;
    for (std::size_t idx = samples_.size(); idx-- > 0;) {
      acc = std::max(acc, std::abs(samples_[idx].plant_pitch_deg));
      running_max_abs[idx] = acc;
    }
    for (std::size_t idx = 0; idx < running_max_abs.size(); ++idx) {
      if (running_max_abs[idx] <= 3.0) {
        return samples_[idx].t_sec;
      }
    }
    return std::nullopt;
  }

  std::optional<double> dt_median_s() const {
    const auto times = time_series(samples_);
    if (times.size() < 2) {
      return std::nullopt;
    }
    std::vector<double> deltas;
    deltas.reserve(times.size() - 1);
    for (std::size_t idx = 1; idx < times.size(); ++idx) {
      const double dt = times[idx] - times[idx - 1];
      if (dt > 0.0) {
        deltas.push_back(dt);
      }
    }
    return median(std::move(deltas));
  }

  std::optional<double> dt_max_s() const {
    const auto times = time_series(samples_);
    if (times.size() < 2) {
      return std::nullopt;
    }
    std::vector<double> deltas;
    deltas.reserve(times.size() - 1);
    for (std::size_t idx = 1; idx < times.size(); ++idx) {
      const double dt = times[idx] - times[idx - 1];
      if (dt > 0.0) {
        deltas.push_back(dt);
      }
    }
    return max_value(deltas);
  }

  bool telemetry_continuous() const {
    const auto times = time_series(samples_);
    if (times.size() < 2) {
      return true;
    }
    std::vector<double> positive;
    positive.reserve(times.size() - 1);
    for (std::size_t idx = 1; idx < times.size(); ++idx) {
      const double dt = times[idx] - times[idx - 1];
      if (dt <= 0.0) {
        return false;
      }
      positive.push_back(dt);
    }
    const auto med = median(positive);
    const double max_gap = *std::max_element(positive.begin(), positive.end());
    if (!med.has_value()) {
      return false;
    }
    return max_gap <= std::max(5.0 * *med, 1e-9);
  }

  bool fell() const {
    return max_abs_pitch() > 75.0;
  }

  template <typename FieldT>
  std::vector<double> tail_series(FieldT ipc::SystemTelemetryPayload::*field) const {
    std::vector<double> out;
    if (samples_.empty()) {
      return out;
    }
    const double tail_start = samples_.back().t_sec - 2.0;
    for (const auto& row : samples_) {
      if (row.t_sec >= tail_start) {
        out.push_back(row.*field);
      }
    }
    return out;
  }

  std::vector<ipc::SystemTelemetryPayload> samples_;
  TelemetryCaptureOptions options_;
  std::chrono::steady_clock::time_point started_at_;
  bool finished_ = false;
};

}  // namespace sil
