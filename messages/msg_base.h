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

enum class Severity : uint8_t { Debug = 0, Info = 1, Warn = 2, Error = 3 };
enum class SystemState : uint8_t { Init = 0, Ready = 1, Executing = 2, Stopping = 3, Fault = 4 };

enum class DOC_DESC("Top-level message type selector. The uint16_t wire value is the first two bytes of every UDP datagram.") MsgId : uint16_t {
  Log = 0,
  PhysicsTick = 1,
  StateRequest = 2,
  StateData = 3,

  MotorSequence = 10,
  MotorStatus = 11,
  KinematicsRequest = 20,
  KinematicsData = 21,
  PowerRequest = 30,
  PowerData = 31,
  ThermalRequest = 40,
  ThermalData = 41,

  EnvironmentAck = 50,
  EnvironmentRequest = 51,
  EnvironmentData = 52,
  AutoDriveCommand = 60,
  AutoDriveStatus = 61,

  InternalEnvRequest = 1000,
  InternalEnvData = 1001,

  SensorRequest = 1100,
  SensorAck = 1101,

  RevisionRequest = 2000,
  RevisionResponse = 2001,
};

template <MsgId Id>
struct MessageTraits;
