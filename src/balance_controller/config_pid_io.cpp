#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
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
      {"rate_I_lim", &rate_I_lim}, {"rate_FF", &rate_FF},
      {"angle_to_rate_k", &angle_to_rate_k}, {"vel_P", &vel_P},
      {"vel_I", &vel_I},           {"vel_D", &vel_D},     {"vel_I_lim", &vel_I_lim},
      {"pos_P", &pos_P},           {"outer_k_pos", &outer_k_pos},
      {"outer_k_vel", &outer_k_vel}, {"outer_k_pitch", &outer_k_pitch},
      {"outer_k_pitch_rate", &outer_k_pitch_rate}, {"angle_I", &angle_I},
      {"lean_trim_I", &lean_trim_I}, {"lean_trim_max_deg", &lean_trim_max_deg},
      {"lean_trim_decay_s", &lean_trim_decay_s}};

  bool saw_outer_k_pos = false;
  bool saw_outer_k_vel = false;
  bool saw_outer_k_pitch = false;
  bool saw_outer_k_pitch_rate = false;
  bool saw_angle_to_rate_k = false;
  bool saw_vel_p = false;
  bool saw_vel_i = false;
  bool saw_vel_d = false;
  bool saw_vel_i_lim = false;
  bool saw_pos_p = false;

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
            if (key == "outer_k_pos") saw_outer_k_pos = true;
            else if (key == "outer_k_vel") saw_outer_k_vel = true;
            else if (key == "outer_k_pitch") saw_outer_k_pitch = true;
            else if (key == "outer_k_pitch_rate") saw_outer_k_pitch_rate = true;
            else if (key == "angle_to_rate_k") saw_angle_to_rate_k = true;
            else if (key == "vel_P") saw_vel_p = true;
            else if (key == "vel_I") saw_vel_i = true;
            else if (key == "vel_D") saw_vel_d = true;
            else if (key == "vel_I_lim") saw_vel_i_lim = true;
            else if (key == "pos_P") saw_pos_p = true;
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

  const bool saw_legacy_outer =
      saw_angle_to_rate_k || saw_vel_p || saw_vel_i || saw_vel_d || saw_vel_i_lim || saw_pos_p;
  if (saw_legacy_outer) {
    if (!saw_outer_k_vel && saw_vel_p) {
      outer_k_vel = vel_P;
      std::cout << "[Config] Mapped legacy vel_P -> outer_k_vel = " << outer_k_vel << "\n";
    }
    if (!saw_outer_k_pitch && saw_angle_to_rate_k) {
      outer_k_pitch = angle_to_rate_k;
      std::cout << "[Config] Mapped legacy angle_to_rate_k -> outer_k_pitch = " << outer_k_pitch
                << "\n";
    }
    if (!saw_outer_k_pos && saw_pos_p) {
      outer_k_pos = pos_P * outer_k_vel;
      std::cout << "[Config] Mapped legacy pos_P -> outer_k_pos = " << outer_k_pos
                << " using outer_k_vel = " << outer_k_vel << "\n";
    }
    if (!saw_outer_k_pitch_rate) {
      outer_k_pitch_rate = 0.0;
      std::cout << "[Config] Legacy outer-loop config detected; defaulting outer_k_pitch_rate = 0"
                << "\n";
    }
    if (saw_vel_i || saw_vel_d || saw_vel_i_lim) {
      std::cout << "[Config] Legacy vel_I/vel_D/vel_I_lim are loadable for compatibility but are"
                   " not used by the rewritten outer controller.\n";
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

    f << "# --- Physics-Based Outer Loop ---\n";
    write_param(f, "outer_k_pos", outer_k_pos);
    write_param(f, "outer_k_vel", outer_k_vel);
    write_param(f, "outer_k_pitch", outer_k_pitch);
    write_param(f, "outer_k_pitch_rate", outer_k_pitch_rate);
    f << "\n";

    f << "# --- Trim / Bias Learning ---\n";
    write_param(f, "angle_I", angle_I);
    write_param(f, "lean_trim_I", lean_trim_I);
    write_param(f, "lean_trim_max_deg", lean_trim_max_deg);
    write_param(f, "lean_trim_decay_s", lean_trim_decay_s);
    f << "\n";

    f << "# --- Legacy Outer-Loop Keys (load-only compatibility) ---\n";
    f << "# Older configs may still use:\n";
    f << "#   angle_to_rate_k, vel_P, vel_I, vel_D, vel_I_lim, pos_P\n";
    f << "# The rewritten controller reads those keys if present, but new configs should\n";
    f << "# only tune the outer_k_* fields above.\n";

    std::cout << "[Config] Saved defaults to " << path << "\n";
  } else {
    std::cerr << "[Config] Failed to save " << path << "\n";
  }
}
