#pragma once

#include "msg_base.h"

struct DOC_DESC(
    "Global runtime tick. Published by the time service to advance deterministic simulation and "
    "controller execution using an explicit delta time plus accumulated monotonic time.")
    PhysicsTickPayload {
  double dt_s;
  uint64_t timestamp_us;
};

template <>
struct MessageTraits<MsgId::PhysicsTick> {
  using Payload = PhysicsTickPayload;
};
