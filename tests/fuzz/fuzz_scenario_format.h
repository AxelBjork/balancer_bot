#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace fuzz {

constexpr uint8_t kFuzzScenarioVersion1 = 1;
constexpr std::size_t kMaxFuzzDisturbances = 4;

#pragma pack(push, 1)
struct FuzzDisturbanceV1 {
  uint8_t kind = 0;
  uint8_t reserved0 = 0;
  uint16_t reserved1 = 0;
  float start_s = 0.0f;
  float duration_s = 0.0f;
  float force_n = 0.0f;
  float com_bias_rad = 0.0f;
  float force_n_end = 0.0f;
  float com_bias_rad_end = 0.0f;
};

struct FuzzSimulatorScenarioV1 {
  uint8_t version = kFuzzScenarioVersion1;
  uint8_t physics_profile = 0;
  uint8_t disturbance_count = 0;
  uint8_t reserved0 = 0;
  float duration_s = 0.0f;
  float initial_pitch_deg = 0.0f;
  float com_angle_offset_rad = 0.0f;
  float wheel_slip_factor = 1.0f;
  float velocity_feedback_scale = 1.0f;
  float velocity_feedback_tau_s = 0.0f;
  float imu_pitch_lag_s = 0.0f;
  FuzzDisturbanceV1 disturbances[kMaxFuzzDisturbances]{};
};
#pragma pack(pop)

static_assert(std::is_trivially_copyable_v<FuzzDisturbanceV1>);
static_assert(std::is_trivially_copyable_v<FuzzSimulatorScenarioV1>);
static_assert(sizeof(FuzzDisturbanceV1) == 28);
static_assert(sizeof(FuzzSimulatorScenarioV1) == 144);

}  // namespace fuzz

