#pragma once

#include <cstdint>

#if defined(REFLECT_DOCS)
#define DOC_DESC(...) [[= doc::Desc(__VA_ARGS__)]]
#else
#define DOC_DESC(...)
#endif

namespace doc {
struct Desc {
  char text[1024]{};

  constexpr Desc(const char* t) {
    int i = 0;
    while (t[i] != '\0' && i < 1023) {
      text[i] = t[i];
      ++i;
    }
    text[i] = '\0';
  }
};
}  // namespace doc

enum class DOC_DESC("Top-level message type selector. The uint16_t wire value is the first two "
                    "bytes of every UDP datagram.") MsgId : uint16_t {
  PhysicsTick = 1,
  ImuData = 3000,
  JoystickCommand = 3001,
  MotorTargets = 3002,
  SystemTelemetry = 3003,
  MotorFeedback = 3004,
  SimStartRun = 3005,
  SimStartAck = 3006,
  SimStopRun = 3007,
  SimRunDone = 3008,
  ImuRawData = 3009,
};

template <MsgId Id>
struct MessageTraits;

template <MsgId Id>
inline const typename MessageTraits<Id>::Payload& unpack_payload(const void* payload) {
  return *static_cast<const typename MessageTraits<Id>::Payload*>(payload);
}
