#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>

#include "types.h"

static void write_param(std::ofstream& f, const std::string& name, double value) {
  f << std::left << std::setw(20) << name << " = " << value << "\n";
}

void ConfigPid::load(const std::string& path) {
  namespace fs = std::filesystem;
  if (!fs::exists(path)) {
    std::cout << "[Config] File '" << path << "' not found. Creating defaults.\n";
    // Ensure parent directory exists
    fs::path p(path);
    if (p.has_parent_path()) {
      fs::create_directories(p.parent_path());
    }
    save(path);
    return;
  }

  std::ifstream f(path);
  if (!f.is_open()) {
    std::cerr << "[Config] Failed to open existing file: " << path << "\n";
    return;
  }

  // Map string keys to the actual variable addresses
  std::unordered_map<std::string, double*> param_map = {
      {"rate_P", &rate_P},         {"rate_I", &rate_I},   {"rate_D", &rate_D},
      {"rate_I_lim", &rate_I_lim}, {"rate_FF", &rate_FF}, {"angle_to_rate_k", &angle_to_rate_k},
      {"vel_P", &vel_P},           {"vel_I", &vel_I},     {"vel_D", &vel_D},
      {"vel_I_lim", &vel_I_lim}};

  std::cout << "[Config] Loading from " << path << "...\n";
  std::string line;
  while (std::getline(f, line)) {
    // Strip comments #
    size_t comment_pos = line.find('#');
    if (comment_pos != std::string::npos) {
      line = line.substr(0, comment_pos);
    }

    // Trim whitespace
    line.erase(0, line.find_first_not_of(" \t\r\n"));
    if (line.empty()) continue;

    std::stringstream ss(line);
    std::string key, val_str;

    // Split by '='
    if (std::getline(ss, key, '=')) {
      if (std::getline(ss, val_str)) {
        // Trim key and value
        key.erase(0, key.find_first_not_of(" \t"));
        key.erase(key.find_last_not_of(" \t") + 1);
        val_str.erase(0, val_str.find_first_not_of(" \t"));
        val_str.erase(val_str.find_last_not_of(" \t\r\n") + 1);

        auto it = param_map.find(key);
        if (it != param_map.end()) {
          try {
            *it->second = std::stod(val_str);
            std::cout << "Loaded " << key << " = " << *it->second << "\n";
          } catch (...) {
            std::cerr << "[Config] Error parsing value for " << key << ": '" << val_str << "'\n";
          }
        } else {
          // Optional: Warn about unknown keys
          // std::cerr << "[Config] Unknown key: " << key << "\n";
        }
      }
    }
  }
}

void ConfigPid::save(const std::string& path) {
  std::ofstream f(path);
  if (f.is_open()) {
    f << "# Balancer Bot PID Configuration\n";
    f << "# Modifying this file requires application restart (or reload logic)\n\n";

    f << "# --- Rate Controller (Inner Loop) ---\n";
    write_param(f, "rate_P", rate_P);
    write_param(f, "rate_I", rate_I);
    write_param(f, "rate_D", rate_D);
    write_param(f, "rate_I_lim", rate_I_lim);
    write_param(f, "rate_FF", rate_FF);
    f << "\n";

    f << "# --- Angle Controller (Middle Loop) ---\n";
    write_param(f, "angle_to_rate_k", angle_to_rate_k);
    f << "\n";

    f << "# --- Velocity Controller (Outer Loop) ---\n";
    write_param(f, "vel_P", vel_P);
    write_param(f, "vel_I", vel_I);
    write_param(f, "vel_D", vel_D);
    write_param(f, "vel_I_lim", vel_I_lim);

    std::cout << "[Config] Saved defaults to " << path << "\n";
  } else {
    std::cerr << "[Config] Failed to save " << path << "\n";
  }
}
