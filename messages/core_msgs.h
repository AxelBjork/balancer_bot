#pragma once

#include <string_view>

#include "msg_base.h"

struct DOC_DESC("Unidirectional log/trace message. Emitted by any component at any time; Python receives these passively from the bus.") LogPayload {
  char text[255];
  Severity severity;
  char component[32];
};

struct DOC_DESC("One-byte sentinel. Send to request a StateData snapshot. The payload value is ignored.") StateRequestPayload {
  uint8_t reserved;
};

struct DOC_DESC("State machine snapshot. Carries the current coarse lifecycle SystemState.") StatePayload {
  SystemState state;
};

struct DOC_DESC("Request the current system revision and protocol hash.") RevisionRequestPayload {
  uint8_t reserved;
};

struct DOC_DESC("Response containing the system revision and protocol hash.") RevisionResponsePayload {
  char protocol_hash[65];
};

struct DOC_DESC("Global runtime tick. Published by the time service to advance deterministic simulation and controller execution using an explicit delta time plus accumulated monotonic time.") PhysicsTickPayload {
  double dt_s;
  uint64_t sim_time_us;
};

template <>
struct MessageTraits<MsgId::Log> {
  using Payload = LogPayload;
  static constexpr std::string_view name = "Log";
};

template <>
struct MessageTraits<MsgId::StateRequest> {
  using Payload = StateRequestPayload;
  static constexpr std::string_view name = "StateRequest";
};

template <>
struct MessageTraits<MsgId::StateData> {
  using Payload = StatePayload;
  static constexpr std::string_view name = "StateData";
};

template <>
struct MessageTraits<MsgId::RevisionRequest> {
  using Payload = RevisionRequestPayload;
  static constexpr std::string_view name = "RevisionRequest";
};

template <>
struct MessageTraits<MsgId::RevisionResponse> {
  using Payload = RevisionResponsePayload;
  static constexpr std::string_view name = "RevisionResponse";
};

template <>
struct MessageTraits<MsgId::PhysicsTick> {
  using Payload = PhysicsTickPayload;
  static constexpr std::string_view name = "PhysicsTick";
};
