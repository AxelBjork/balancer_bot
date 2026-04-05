#pragma once

#include <array>
#include <memory>
#include <string_view>

#include "msg_base.h"

struct DOC_DESC("A 2D coordinate.") Point2D {
  float x;
  float y;
};

struct DOC_DESC("An axis-aligned 2D bounding box.") BoundingBox2D {
  Point2D min_pt;
  Point2D max_pt;
};

struct DOC_DESC("A 3D coordinate vector.") Vector3 {
  float x;
  float y;
  float z;
};

struct DOC_DESC("Environment ID Wrapper.") EnvId {
  uint32_t id;
};

struct DOC_DESC("Request conditions for a specific location.") EnvironmentRequestPayload {
  Point2D target_location;
};

struct DOC_DESC("ACK sent by the application when it accepts new environment data.") EnvironmentAckPayload {
  uint32_t region_id;
};

struct DOC_DESC("Environmental conditions delivered to the application from the outside world.") EnvironmentPayload {
  uint32_t region_id;
  BoundingBox2D bounds;
  float ambient_temp_c;
  float incline_percent;
  float surface_friction;
  float max_speed_rpm;
};

enum class DOC_DESC("Control strategy for the autonomous service.") DriveMode : uint8_t {
  Eco,
  Performance,
  ManualTuning
};

struct DOC_DESC("A single target maneuver point.") ManeuverNode {
  Point2D target_pos;
  uint16_t timeout_ms;
};

template <std::size_t MaxNodes>
struct DOC_DESC("High level autonomous driving route and configuration.") AutoDriveCommandTemplate {
  char route_name[32];
  DriveMode mode;
  float p_gain;
  bool use_environment_tuning;
  std::array<Vector3, 3> route_transform;
  uint8_t num_nodes;
  std::array<ManeuverNode, MaxNodes> route;
};

using AutoDriveCommandPayload = AutoDriveCommandTemplate<8>;

struct DOC_DESC("Efficiency metrics for a single traveled node.") ManeuverStats {
  float initial_energy_mj;
  float final_energy_mj;
  float energy_per_meter_mj;
  float total_energy_used_mj;
};

template <std::size_t MaxNodes, std::size_t MaxEnvs>
struct DOC_DESC("Status and efficiency report from the AutonomousService.") AutoDriveStatusTemplate {
  uint32_t cmd_id;
  uint8_t current_node_idx;
  bool route_complete;
  uint8_t num_stats;
  std::array<ManeuverStats, MaxNodes> node_stats;
  uint8_t num_environments_used;
  std::array<EnvId, MaxEnvs> environment_ids;
};

using AutoDriveStatusPayload = AutoDriveStatusTemplate<8, 4>;

struct InternalEnvRequestPayload {
  float x;
  float y;
};

struct InternalEnvDataPayload {
  std::shared_ptr<const EnvironmentPayload> ptr;
};

template <>
struct MessageTraits<MsgId::EnvironmentAck> {
  using Payload = EnvironmentAckPayload;
  static constexpr std::string_view name = "EnvironmentAck";
};

template <>
struct MessageTraits<MsgId::EnvironmentRequest> {
  using Payload = EnvironmentRequestPayload;
  static constexpr std::string_view name = "EnvironmentRequest";
};

template <>
struct MessageTraits<MsgId::EnvironmentData> {
  using Payload = EnvironmentPayload;
  static constexpr std::string_view name = "EnvironmentData";
};

template <>
struct MessageTraits<MsgId::AutoDriveCommand> {
  using Payload = AutoDriveCommandPayload;
  static constexpr std::string_view name = "AutoDriveCommand";
};

template <>
struct MessageTraits<MsgId::AutoDriveStatus> {
  using Payload = AutoDriveStatusPayload;
  static constexpr std::string_view name = "AutoDriveStatus";
};

template <>
struct MessageTraits<MsgId::InternalEnvRequest> {
  using Payload = InternalEnvRequestPayload;
  static constexpr std::string_view name = "InternalEnvRequest";
};

template <>
struct MessageTraits<MsgId::InternalEnvData> {
  using Payload = InternalEnvDataPayload;
  static constexpr std::string_view name = "InternalEnvData";
};

struct DOC_DESC("Explicitly request the SensorService to read terrain from its deterministically generated world map.") SensorRequestPayload {
  Point2D target_location;
};

struct DOC_DESC("ACK sent by SensorService indicating whether a SensorRequest was accepted.") SensorAckPayload {
  uint32_t request_id;
  bool success;
  uint8_t reason;
};

template <>
struct MessageTraits<MsgId::SensorRequest> {
  using Payload = SensorRequestPayload;
  static constexpr std::string_view name = "SensorRequest";
};

template <>
struct MessageTraits<MsgId::SensorAck> {
  using Payload = SensorAckPayload;
  static constexpr std::string_view name = "SensorAck";
};
