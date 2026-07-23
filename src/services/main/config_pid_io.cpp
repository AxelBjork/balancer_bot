#include <algorithm>
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

#include "messages/types.h"

static void write_param(std::ofstream& f, const std::string& name, double value) {
  f << std::left << std::setw(20) << name << " = " << value << "\n";
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
      "config_version", "rate_P", "rate_I", "rate_D", "rate_I_lim", "rate_FF",
      "velocity_P", "max_longitudinal_accel_mps2", "max_jerk_mps3", "velocity_damping_per_s", "velocity_I", "velocity_I_limit_deg", "angle_P", "angle_D",
      "drive_max_sps", "turn_max_sps", "pitch_max_deg", "balance_max_sps",
      "output_scale_sps", "controller_enabled"};
  controller_enabled = true;
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

  std::unordered_map<std::string, double> values;
  for (const auto& item : parsed_lines) {
    if (!allowed.contains(item.key)) {
      throw std::runtime_error("Unknown PID configuration key: " + item.key);
    }
    if (values.contains(item.key)) {
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
    values.emplace(item.key, value);
  }

  for (const auto& key : allowed) {
    if (key == "controller_enabled") continue;
    if (!values.contains(key)) {
      throw std::runtime_error("Missing PID configuration key: " + key);
    }
  }
  if (values.contains("controller_enabled") && values.at("controller_enabled") != 0.0 &&
      values.at("controller_enabled") != 1.0) {
    throw std::runtime_error("controller_enabled must be 0 or 1");
  }

  const auto nonnegative = [&](const char* key) {
    if (values.at(key) < 0.0) throw std::runtime_error(std::string(key) + " must be non-negative");
  };
  for (const char* key : {"rate_P", "rate_I", "rate_D", "rate_I_lim", "rate_FF",
                          "velocity_P", "max_longitudinal_accel_mps2", "max_jerk_mps3", "velocity_damping_per_s", "velocity_I", "velocity_I_limit_deg", "angle_P",
                          "angle_D", "turn_max_sps"}) {
    nonnegative(key);
  }
  for (const char* key : {"drive_max_sps", "pitch_max_deg", "balance_max_sps",
                          "output_scale_sps"}) {
    if (values.at(key) <= 0.0) throw std::runtime_error(std::string(key) + " must be positive");
  }
  if (values.at("drive_max_sps") > 12000.0 || values.at("turn_max_sps") > 12000.0 ||
      values.at("balance_max_sps") > 12000.0 || values.at("pitch_max_deg") > 45.0) {
    throw std::runtime_error("PID configuration limit exceeds the supported range");
  }

  rate_P = values.at("rate_P");
  rate_I = values.at("rate_I");
  rate_D = values.at("rate_D");
  rate_I_lim = values.at("rate_I_lim");
  rate_FF = values.at("rate_FF");
  velocity_P = values.at("velocity_P");
  max_longitudinal_accel_mps2 = values.at("max_longitudinal_accel_mps2");
  max_jerk_mps3 = values.at("max_jerk_mps3");
  velocity_damping_per_s = values.at("velocity_damping_per_s");
  velocity_I = values.at("velocity_I");
  velocity_I_limit_deg = values.at("velocity_I_limit_deg");
  angle_P = values.at("angle_P");
  angle_D = values.at("angle_D");
  drive_max_sps = values.at("drive_max_sps");
  turn_max_sps = values.at("turn_max_sps");
  pitch_max_deg = values.at("pitch_max_deg");
  balance_max_sps = values.at("balance_max_sps");
  output_scale_sps = values.at("output_scale_sps");
  controller_enabled = !values.contains("controller_enabled") || values.at("controller_enabled") == 1.0;
}

void ConfigPid::save(const std::string& path) {
  std::ofstream f(path);
  if (f.is_open()) {
    f << "# Balancer Bot PID Configuration\n";
    f << "# Modifying this file requires application restart (or reload logic)\n\n";

    f << "config_version       = " << config_version << "\n\n";
    f << "# --- Rate Controller (400 Hz) ---\n";
    write_param(f, "rate_P", rate_P);
    write_param(f, "rate_I", rate_I);
    write_param(f, "rate_D", rate_D);
    write_param(f, "rate_I_lim", rate_I_lim);
    write_param(f, "rate_FF", rate_FF);
    f << "\n";

    f << "# --- Velocity control / stationary COM trim (50 Hz) and allocation ---\n";
    write_param(f, "velocity_P", velocity_P);
    write_param(f, "max_longitudinal_accel_mps2", max_longitudinal_accel_mps2);
    write_param(f, "max_jerk_mps3", max_jerk_mps3);
    write_param(f, "velocity_damping_per_s", velocity_damping_per_s);
    write_param(f, "velocity_I", velocity_I);
    write_param(f, "velocity_I_limit_deg", velocity_I_limit_deg);
    write_param(f, "angle_P", angle_P);
    write_param(f, "angle_D", angle_D);
    write_param(f, "drive_max_sps", drive_max_sps);
    write_param(f, "turn_max_sps", turn_max_sps);
    write_param(f, "pitch_max_deg", pitch_max_deg);
    write_param(f, "balance_max_sps", balance_max_sps);
    write_param(f, "output_scale_sps", output_scale_sps);
    write_param(f, "controller_enabled", controller_enabled ? 1.0 : 0.0);
    f << "\n";
  } else {
    throw std::runtime_error("PID configuration cannot be written: " + path);
  }
}
