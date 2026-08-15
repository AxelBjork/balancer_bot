#pragma once

#include <cstdint>

#if defined(REFLECT_DOCS)
#define DOC_DESC(...) [[= doc::Desc(__VA_ARGS__)]]
#else
#define DOC_DESC(...)
#endif

namespace doc {
struct Desc {
  char text[2048]{};

  constexpr Desc(const char* t) {
    int i = 0;
    while (t[i] != '\0') {
      if (i >= static_cast<int>(sizeof(text) - 1)) {
        throw "doc::Desc exceeds its 2047-character capacity";
      }
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
  SimulatorTelemetry = 3010,
  ExternalJoystickCommand = 3011,
  PidConfigOverride = 3012,
  PidConfigStatus = 3013,
  PitchAuthorityDiagnosticCommand = 3014,
};

template <MsgId Id>
struct MessageTraits;

template <MsgId Id>
inline const typename MessageTraits<Id>::Payload& unpack_payload(const void* payload) {
  return *static_cast<const typename MessageTraits<Id>::Payload*>(payload);
}
