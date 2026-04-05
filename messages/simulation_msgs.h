#pragma once

#include <array>
#include <string_view>

#include "msg_base.h"

struct DOC_DESC("One timed motor command step, embedded in MotorSequencePayload.") MotorSubCmd {
  int16_t speed_rpm;
  uint32_t duration_us;
};

static constexpr uint8_t kMaxSubCmds = 5;

template <std::size_t N>
struct DOC_DESC("Deliver a sequence of up to 10 timed motor sub-commands to the simulator. The simulator executes steps[0..num_steps-1] in real time; a new command preempts any currently running sequence.") MotorSequencePayloadTemplate {
  uint32_t cmd_id;
  uint8_t num_steps;
  std::array<MotorSubCmd, N> steps;
};

using MotorSequencePayload = MotorSequencePayloadTemplate<kMaxSubCmds>;

struct DOC_DESC("One-byte sentinel. Send to request a KinematicsData snapshot. The payload value is ignored.") KinematicsRequestPayload {
  uint8_t reserved;
};

struct DOC_DESC("Kinematics snapshot sent in response to a KinematicsRequest. Reflects physics state integrated since the start of the current sequence.") KinematicsPayload {
  uint32_t cmd_id;
  uint32_t elapsed_us;
  float position_m;
  float speed_mps;
};

struct DOC_DESC("One-byte sentinel. Send to request a PowerData snapshot. The payload value is ignored.") PowerRequestPayload {
  uint8_t reserved;
};

struct DOC_DESC("Power-model snapshot sent in response to a PowerRequest. Models a simple battery with internal resistance drain.") PowerPayload {
  uint32_t cmd_id;
  float voltage_v;
  float current_a;
  uint8_t state_of_charge;
};

struct DOC_DESC("One-byte sentinel. Send to request a ThermalData snapshot. The payload value is ignored.") ThermalRequestPayload {
  uint8_t reserved;
};

struct DOC_DESC("Thermal snapshot sent in response to a ThermalRequest. Models temperature of motor and battery based on power metrics.") ThermalPayload {
  float motor_temp_c;
  float battery_temp_c;
};

struct DOC_DESC("Internal IPC: Periodic RPM and activity update from MotorService.") MotorStatusPayload {
  uint32_t cmd_id;
  int16_t speed_rpm;
  bool is_active;
};

template <>
struct MessageTraits<MsgId::MotorSequence> {
  using Payload = MotorSequencePayload;
  static constexpr std::string_view name = "MotorSequence";
};

template <>
struct MessageTraits<MsgId::KinematicsRequest> {
  using Payload = KinematicsRequestPayload;
  static constexpr std::string_view name = "KinematicsRequest";
};

template <>
struct MessageTraits<MsgId::KinematicsData> {
  using Payload = KinematicsPayload;
  static constexpr std::string_view name = "KinematicsData";
};

template <>
struct MessageTraits<MsgId::PowerRequest> {
  using Payload = PowerRequestPayload;
  static constexpr std::string_view name = "PowerRequest";
};

template <>
struct MessageTraits<MsgId::PowerData> {
  using Payload = PowerPayload;
  static constexpr std::string_view name = "PowerData";
};

template <>
struct MessageTraits<MsgId::ThermalRequest> {
  using Payload = ThermalRequestPayload;
  static constexpr std::string_view name = "ThermalRequest";
};

template <>
struct MessageTraits<MsgId::ThermalData> {
  using Payload = ThermalPayload;
  static constexpr std::string_view name = "ThermalData";
};

template <>
struct MessageTraits<MsgId::MotorStatus> {
  using Payload = MotorStatusPayload;
  static constexpr std::string_view name = "MotorStatus";
};
