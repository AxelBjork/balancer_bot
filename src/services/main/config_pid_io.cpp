#include <algorithm>
#include <atomic>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <stdexcept>
#include <sstream>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <vector>

#include "services/main/config_pid_io.h"
#include "services/main/config.h"

namespace {

std::atomic<uint64_t> g_config_pid_generation{0};

}  // namespace

namespace sil {

ConfigPidValues pid_values_from_payload(const ConfigPidValues& p) {
  return p;
}

void fill_pid_status_values(ConfigPidValues& p, const ConfigPidValues& values) {
  p = values;
}

ipc::PidConfigStatusPayload apply_pid_config_override(const ipc::PidConfigOverridePayload& payload) {
  const ConfigPidValues values = pid_values_from_payload(payload.values);
  const ConfigPidValidationCode validation = ConfigPid::validate_numeric(values);

  ipc::PidConfigStatusPayload status{};
  status.request_id = payload.request_id;
  status.accepted = validation == ConfigPidValidationCode::Accepted ? 1 : 0;
  status.result_code = static_cast<uint8_t>(validation);
  fill_pid_status_values(status.values, ConfigPid::numeric_values());
  if (validation == ConfigPidValidationCode::Accepted) {
    ConfigPid::apply_numeric(values);
    fill_pid_status_values(status.values, ConfigPid::numeric_values());
  }
  return status;
}

}  // namespace sil

static void write_param(std::ofstream& f, const std::string& name, double value) {
  f << std::left << std::setw(20) << name << " = " << value << "\n";
}

ConfigPidValues ConfigPid::numeric_values() {
  return values;
}

ConfigPidValidationCode ConfigPid::validate_numeric(const ConfigPidValues& values) {
  for (const double value : {
           values.drive_max_velocity_mps,
           values.velocity_gain_per_s,
           values.velocity_feedback_cutoff_hz,
           values.outer_pitch_limit_deg,
           values.fixed_com_trim_deg,
           values.adaptive_com_trim_enabled,
           values.adaptive_com_trim_gain_deg_per_mps_s,
           values.adaptive_com_trim_limit_deg,
           values.turn_max_sps,
           values.balance_max_sps,
           values.pitch_gain,
           values.pitch_rate_gain,
           values.pitch_accel_gain,
           values.planner_max_acceleration_mps2,
           values.planner_max_deceleration_mps2,
           values.planner_max_jerk_mps3,
           values.velocity_i_gain_per_s2,
           values.velocity_i_leak_time_s,
           values.velocity_i_acceleration_limit_mps2,
       }) {
    if (!std::isfinite(value)) return ConfigPidValidationCode::NonFinite;
  }

  for (const double value : {
           values.drive_max_velocity_mps,
           values.velocity_gain_per_s,
           values.velocity_feedback_cutoff_hz,
           values.outer_pitch_limit_deg,
           values.adaptive_com_trim_enabled,
           values.adaptive_com_trim_gain_deg_per_mps_s,
           values.adaptive_com_trim_limit_deg,
           values.turn_max_sps,
           values.pitch_gain,
           values.pitch_rate_gain,
           values.pitch_accel_gain,
           values.planner_max_acceleration_mps2,
           values.planner_max_deceleration_mps2,
           values.planner_max_jerk_mps3,
           values.velocity_i_gain_per_s2,
           values.velocity_i_leak_time_s,
           values.velocity_i_acceleration_limit_mps2,
       }) {
    if (value < 0.0) return ConfigPidValidationCode::Negative;
  }

  if (values.drive_max_velocity_mps <= 0.0 || values.balance_max_sps <= 0.0 ||
      values.velocity_feedback_cutoff_hz <= 0.0 || values.outer_pitch_limit_deg <= 0.0) {
    return ConfigPidValidationCode::NonPositive;
  }
  if (values.planner_max_acceleration_mps2 <= 0.0 ||
      values.planner_max_deceleration_mps2 <= 0.0 || values.planner_max_jerk_mps3 <= 0.0 ||
      values.velocity_i_leak_time_s <= 0.0 ||
      values.velocity_i_acceleration_limit_mps2 <= 0.0) {
    return ConfigPidValidationCode::NonPositive;
  }
  if (values.pitch_gain > 1.0e6 || values.pitch_rate_gain > 1.0e6 ||
      values.pitch_accel_gain > 1.0e6) {
    return ConfigPidValidationCode::OutOfRange;
  }
  if (values.outer_pitch_limit_deg > Config::max_motion_pitch_setpoint_deg ||
      std::abs(values.fixed_com_trim_deg) > 45.0 ||
      values.adaptive_com_trim_limit_deg > 45.0 ||
      (values.adaptive_com_trim_enabled != 0.0 && values.adaptive_com_trim_enabled != 1.0)) {
    return ConfigPidValidationCode::OutOfRange;
  }
  if (values.turn_max_sps > Config::max_step_rate_sps ||
      values.balance_max_sps > Config::max_step_rate_sps ||
      values.balance_max_sps <= values.turn_max_sps) {
    return ConfigPidValidationCode::OutOfRange;
  }
  const double user_speed_sps = values.drive_max_velocity_mps / Config::meters_per_step;
  const double available_sps = values.balance_max_sps - values.turn_max_sps;
  if (user_speed_sps > 0.25 * available_sps) {
    return ConfigPidValidationCode::OutOfRange;
  }
  return ConfigPidValidationCode::Accepted;
}

void ConfigPid::apply_numeric(const ConfigPidValues& next_values) {
  if (validate_numeric(next_values) != ConfigPidValidationCode::Accepted) {
    throw std::invalid_argument("Cannot apply invalid numeric PID configuration");
  }
  values = next_values;
  g_config_pid_generation.fetch_add(1, std::memory_order_release);
}

uint64_t ConfigPid::generation() {
  return g_config_pid_generation.load(std::memory_order_acquire);
}

void ConfigPid::load(const std::string& path) {
  namespace fs = std::filesystem;
  if (!fs::exists(path)) {
    throw std::runtime_error("PID configuration does not exist: " + path);
  }

  std::ifstream f(path);
  if (!f.is_open()) {
    throw std::runtime_error("PID configuration cannot be opened: " + path);
  }

  struct ParsedLine {
    std::string key;
    std::string value_text;
  };
  std::vector<ParsedLine> parsed_lines;
  std::unordered_set<std::string> allowed = {
      "config_version", "drive_max_velocity_mps", "velocity_gain_per_s",
      "velocity_feedback_cutoff_hz",
      "outer_pitch_limit_deg", "fixed_com_trim_deg", "adaptive_com_trim_enabled",
      "adaptive_com_trim_gain_deg_per_mps_s", "adaptive_com_trim_limit_deg", "turn_max_sps",
      "balance_max_sps", "pitch_gain", "pitch_rate_gain", "pitch_accel_gain",
      "planner_max_acceleration_mps2", "planner_max_deceleration_mps2",
      "planner_max_jerk_mps3", "velocity_i_gain_per_s2", "velocity_i_leak_time_s",
      "velocity_i_acceleration_limit_mps2",
      "controller_enabled"};
  std::string line;
  size_t line_number = 0;
  while (std::getline(f, line)) {
    ++line_number;
    // Strip comments #
    size_t comment_pos = line.find('#');
    if (comment_pos != std::string::npos) {
      line = line.substr(0, comment_pos);
    }

    // Trim whitespace
    line.erase(0, line.find_first_not_of(" \t\r\n"));
    if (line.empty()) continue;

    const size_t equals = line.find('=');
    if (equals == std::string::npos || line.find('=', equals + 1) != std::string::npos) {
      throw std::runtime_error("Malformed PID configuration line " +
                               std::to_string(line_number));
    }
    auto trim = [](std::string text) {
      const size_t first = text.find_first_not_of(" \t\r\n");
      if (first == std::string::npos) return std::string{};
      const size_t last = text.find_last_not_of(" \t\r\n");
      return text.substr(first, last - first + 1);
    };
    const std::string key = trim(line.substr(0, equals));
    const std::string value_text = trim(line.substr(equals + 1));
    parsed_lines.push_back({key, value_text});
  }

  // Validate the schema version before interpreting any schema-specific key.
  // This makes newer configuration files fail clearly on older binaries.
  const auto version = std::find_if(parsed_lines.begin(), parsed_lines.end(),
                                    [](const ParsedLine& item) { return item.key == "config_version"; });
  if (version == parsed_lines.end()) {
    throw std::runtime_error("Missing PID configuration key: config_version");
  }
  if (std::count_if(parsed_lines.begin(), parsed_lines.end(),
                    [](const ParsedLine& item) { return item.key == "config_version"; }) != 1) {
    throw std::runtime_error("Duplicate PID configuration key: config_version");
  }
  size_t version_parsed = 0;
  double version_value = 0.0;
  try {
    version_value = std::stod(version->value_text, &version_parsed);
  } catch (const std::exception&) {
    throw std::runtime_error("Invalid value for PID configuration key: config_version");
  }
  if (version_parsed != version->value_text.size() || !std::isfinite(version_value)) {
    throw std::runtime_error("Non-finite or malformed PID value for key: config_version");
  }
  if (version_value != static_cast<double>(config_version)) {
    throw std::runtime_error("PID configuration version mismatch: expected " +
                             std::to_string(config_version) + ", got " + version->value_text);
  }

  std::unordered_map<std::string, double> parsed_values;
  for (const auto& item : parsed_lines) {
    if (!allowed.contains(item.key)) {
      throw std::runtime_error("Unknown PID configuration key: " + item.key);
    }
    if (parsed_values.contains(item.key)) {
      throw std::runtime_error("Duplicate PID configuration key: " + item.key);
    }
    size_t parsed = 0;
    double value = 0.0;
    try {
      value = std::stod(item.value_text, &parsed);
    } catch (const std::exception&) {
      throw std::runtime_error("Invalid value for PID configuration key: " + item.key);
    }
    if (parsed != item.value_text.size() || !std::isfinite(value)) {
      throw std::runtime_error("Non-finite or malformed PID value for key: " + item.key);
    }
    parsed_values.emplace(item.key, value);
  }

  for (const auto& key : allowed) {
    if (key == "config_version" || key == "controller_enabled") continue;
    if (!parsed_values.contains(key)) {
      throw std::runtime_error("Missing PID configuration key: " + key);
    }
  }
  if (parsed_values.contains("controller_enabled") && parsed_values.at("controller_enabled") != 0.0 &&
      parsed_values.at("controller_enabled") != 1.0) {
    throw std::runtime_error("controller_enabled must be 0 or 1");
  }

  const ConfigPidValues numeric{
      parsed_values.at("drive_max_velocity_mps"),
      parsed_values.at("velocity_gain_per_s"),
      parsed_values.at("velocity_feedback_cutoff_hz"),
      parsed_values.at("outer_pitch_limit_deg"),
      parsed_values.at("fixed_com_trim_deg"),
      parsed_values.at("adaptive_com_trim_enabled"),
      parsed_values.at("adaptive_com_trim_gain_deg_per_mps_s"),
      parsed_values.at("adaptive_com_trim_limit_deg"),
      parsed_values.at("turn_max_sps"),
      parsed_values.at("balance_max_sps"),
      parsed_values.at("pitch_gain"),
      parsed_values.at("pitch_rate_gain"),
      parsed_values.at("pitch_accel_gain"),
      parsed_values.at("planner_max_acceleration_mps2"),
      parsed_values.at("planner_max_deceleration_mps2"),
      parsed_values.at("planner_max_jerk_mps3"),
      parsed_values.at("velocity_i_gain_per_s2"),
      parsed_values.at("velocity_i_leak_time_s"),
      parsed_values.at("velocity_i_acceleration_limit_mps2"),
  };
  switch (validate_numeric(numeric)) {
    case ConfigPidValidationCode::Accepted:
      break;
    case ConfigPidValidationCode::NonFinite:
      throw std::runtime_error("PID configuration contains a non-finite value");
    case ConfigPidValidationCode::Negative:
      throw std::runtime_error("PID configuration contains a negative value");
    case ConfigPidValidationCode::NonPositive:
      throw std::runtime_error("PID configuration contains a non-positive limit");
    case ConfigPidValidationCode::OutOfRange:
      throw std::runtime_error("PID configuration limit exceeds the supported range");
  }
  const bool next_controller_enabled =
      !parsed_values.contains("controller_enabled") || parsed_values.at("controller_enabled") == 1.0;
  apply_numeric(numeric);
  controller_enabled = next_controller_enabled;
}

void ConfigPid::save(const std::string& path) {
  std::ofstream f(path);
  if (f.is_open()) {
    f << "# Balancer Bot Controller Configuration\n";
    f << "# Modifying this file requires application restart (or reload logic)\n\n";

    f << "config_version       = " << config_version << "\n\n";
    f << "# --- Explicit attitude state feedback ---\n";
    write_param(f, "pitch_gain", values.pitch_gain);
    write_param(f, "pitch_rate_gain", values.pitch_rate_gain);
    write_param(f, "pitch_accel_gain", values.pitch_accel_gain);
    f << "\n# --- Velocity-reference outer loop / fixed COM trim ---\n";
    write_param(f, "drive_max_velocity_mps", values.drive_max_velocity_mps);
    write_param(f, "velocity_gain_per_s", values.velocity_gain_per_s);
    write_param(f, "velocity_feedback_cutoff_hz", values.velocity_feedback_cutoff_hz);
    write_param(f, "outer_pitch_limit_deg", values.outer_pitch_limit_deg);
    write_param(f, "fixed_com_trim_deg", values.fixed_com_trim_deg);
    write_param(f, "adaptive_com_trim_enabled", values.adaptive_com_trim_enabled);
    write_param(f, "adaptive_com_trim_gain_deg_per_mps_s",
                values.adaptive_com_trim_gain_deg_per_mps_s);
    write_param(f, "adaptive_com_trim_limit_deg", values.adaptive_com_trim_limit_deg);
    write_param(f, "turn_max_sps", values.turn_max_sps);
    write_param(f, "balance_max_sps", values.balance_max_sps);
    write_param(f, "planner_max_acceleration_mps2", values.planner_max_acceleration_mps2);
    write_param(f, "planner_max_deceleration_mps2", values.planner_max_deceleration_mps2);
    write_param(f, "planner_max_jerk_mps3", values.planner_max_jerk_mps3);
    write_param(f, "velocity_i_gain_per_s2", values.velocity_i_gain_per_s2);
    write_param(f, "velocity_i_leak_time_s", values.velocity_i_leak_time_s);
    write_param(f, "velocity_i_acceleration_limit_mps2",
                values.velocity_i_acceleration_limit_mps2);
    write_param(f, "controller_enabled", controller_enabled ? 1.0 : 0.0);
    f << "\n";
  } else {
    throw std::runtime_error("PID configuration cannot be written: " + path);
  }
}
