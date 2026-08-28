#include <array>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>

#include "simulator/balancer_simulator.h"

namespace {

using Mat4 = std::array<std::array<double, 4>, 4>;
using Vec4 = std::array<double, 4>;

Vec4 mat_vec(const Mat4& a, const Vec4& x) {
  Vec4 out{};
  for (std::size_t row = 0; row < 4; ++row) {
    for (std::size_t col = 0; col < 4; ++col) {
      out[row] += a[row][col] * x[col];
    }
  }
  return out;
}

Vec4 add(const Vec4& left, const Vec4& right) {
  Vec4 out{};
  for (std::size_t i = 0; i < out.size(); ++i) {
    out[i] = left[i] + right[i];
  }
  return out;
}

int controllability_rank(const BalancerSimulator::LinearizedUprightModel& model) {
  std::array<Vec4, 4> columns{};
  columns[0] = add(model.horizontal_force_input, model.motor_force_input);
  for (std::size_t i = 1; i < columns.size(); ++i) {
    columns[i] = mat_vec(model.A, columns[i - 1]);
  }

  double m[4][4]{};
  for (std::size_t row = 0; row < 4; ++row) {
    for (std::size_t col = 0; col < 4; ++col) {
      m[row][col] = columns[col][row];
    }
  }

  constexpr double kEps = 1e-9;
  int rank = 0;
  for (int col = 0; col < 4; ++col) {
    int pivot = rank;
    while (pivot < 4 && std::abs(m[pivot][col]) < kEps) {
      ++pivot;
    }
    if (pivot == 4) {
      continue;
    }
    if (pivot != rank) {
      for (int c = 0; c < 4; ++c) {
        std::swap(m[pivot][c], m[rank][c]);
      }
    }
    const double pivot_value = m[rank][col];
    for (int c = col; c < 4; ++c) {
      m[rank][c] /= pivot_value;
    }
    for (int r = 0; r < 4; ++r) {
      if (r == rank) {
        continue;
      }
      const double factor = m[r][col];
      for (int c = col; c < 4; ++c) {
        m[r][c] -= factor * m[rank][c];
      }
    }
    ++rank;
  }
  return rank;
}

std::string_view profile_name(PhysicsProfile profile) {
  return BalancerSimulator::profile_name(profile);
}

void print_row(const std::array<double, 4>& row) {
  std::cout << "  [";
  for (std::size_t i = 0; i < row.size(); ++i) {
    if (i != 0) {
      std::cout << ", ";
    }
    std::cout << std::setw(11) << row[i];
  }
  std::cout << "]\n";
}

void print_profile_audit(PhysicsProfile profile) {
  const SimulatorPhysics physics = BalancerSimulator::physics_for_profile(profile);
  const auto model = BalancerSimulator::linearized_upright_model(physics);
  const auto poles = BalancerSimulator::overdamped_candidate_poles(physics);
  const int rank = controllability_rank(model);

  std::cout << "Profile: " << profile_name(profile) << "\n";
  std::cout << "Physics:\n";
  std::cout << "  max_force_n         = " << physics.max_force_n << "\n";
  std::cout << "  no_load_speed_mps   = " << physics.no_load_speed_mps << "\n";
  std::cout << "  traction_coefficient= " << physics.traction_coefficient << "\n";
  std::cout << "  motor_vel_damping   = " << physics.motor_velocity_damping << "\n";
  std::cout << "  cart_damping        = " << physics.cart_damping << "\n";
  std::cout << "  rolling_resistance  = " << physics.rolling_resistance_force_n << " N\n";
  std::cout << "  static_breakaway   = " << physics.static_breakaway_force_n << " N\n";
  std::cout << "  pitch_damping       = " << physics.pitch_damping << "\n";
  std::cout << "  motor_tau_s         = " << physics.motor_tau_s << "\n";
  std::cout << "  direct_force_per_sps= " << physics.direct_force_per_sps << "\n";
  std::cout << "  command_delay_s     = " << physics.command_delay_s << "\n";
  std::cout << "  speed_force_limit   = " << physics.speed_dependent_force_limit << "\n";
  std::cout << "  force_from_vel_err  = " << physics.force_from_velocity_error << "\n";
  std::cout << "  phase_limit_steps   = " << physics.phase_error_limit_steps << "\n";
  std::cout << "Linearized A:\n";
  for (const auto& row : model.A) {
    print_row(row);
  }
  std::cout << "Linearized horizontal-force input:\n";
  print_row(model.horizontal_force_input);
  std::cout << "Linearized motor-force input:\n";
  print_row(model.motor_force_input);
  std::cout << "Quasi-static combined drive-force input:\n";
  print_row(add(model.horizontal_force_input, model.motor_force_input));
  std::cout << "Controllability rank: " << rank << "/4\n";
  std::cout << "Candidate overdamped poles:\n";
  std::cout << "  [";
  for (std::size_t i = 0; i < poles.size(); ++i) {
    if (i != 0) {
      std::cout << ", ";
    }
    std::cout << poles[i];
  }
  std::cout << "]\n";
  std::cout << "\n";
}

void print_usage(const char* argv0) {
  std::cout << "Usage: " << argv0
            << " [--all] [--profile simplified|realistic|actuator_stress|direct_actuator|ideal_force(alias)|stepper_phase|stepper_phase_electrical]\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::cout << std::fixed << std::setprecision(6);

  bool all_profiles = true;
  std::optional<PhysicsProfile> selected_profile;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--all") {
      all_profiles = true;
      selected_profile.reset();
      continue;
    }
    if (arg == "--profile") {
      if (i + 1 >= argc) {
        print_usage(argv[0]);
        return EXIT_FAILURE;
      }
      const std::string value = argv[++i];
      if (value == "simplified") {
        selected_profile = PhysicsProfile::Simplified;
      } else if (value == "realistic") {
        selected_profile = PhysicsProfile::Realistic;
      } else if (value == "actuator_stress") {
        selected_profile = PhysicsProfile::ActuatorStress;
      } else if (value == "direct_actuator" || value == "ideal_force") {
        selected_profile = PhysicsProfile::DirectActuator;
      } else if (value == "stepper_phase") {
        selected_profile = PhysicsProfile::StepperPhase;
      } else if (value == "stepper_phase_electrical") {
        selected_profile = PhysicsProfile::StepperPhaseElectrical;
      } else {
        print_usage(argv[0]);
        return EXIT_FAILURE;
      }
      all_profiles = false;
      continue;
    }
    print_usage(argv[0]);
    return EXIT_FAILURE;
  }

  if (all_profiles) {
    print_profile_audit(PhysicsProfile::Simplified);
    print_profile_audit(PhysicsProfile::Realistic);
    print_profile_audit(PhysicsProfile::ActuatorStress);
    print_profile_audit(PhysicsProfile::DirectActuator);
    print_profile_audit(PhysicsProfile::StepperPhase);
    print_profile_audit(PhysicsProfile::StepperPhaseElectrical);
    return EXIT_SUCCESS;
  }

  print_profile_audit(*selected_profile);
  return EXIT_SUCCESS;
}
