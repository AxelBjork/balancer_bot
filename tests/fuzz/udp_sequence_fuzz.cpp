#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "afl_compat.h"
#include "fuzz_support.h"
#include "messages/balancer_msgs.h"
#include "messages/types.h"
#include "publisher.h"
#include "services/control/control_service.h"
#include "services/imu/imu_service.h"
#include "udp_bridge.h"

namespace {

volatile std::uint64_t g_udp_sink = 0;

std::size_t payload_size_for(MsgId id) {
  switch (id) {
    case MsgId::PhysicsTick:
      return sizeof(PhysicsTickPayload);
    case MsgId::JoystickCommand:
      return sizeof(ipc::JoystickCommandPayload);
    case MsgId::ImuRawData:
      return sizeof(ipc::ImuRawPayload);
    default:
      return 0;
  }
}

struct ServiceHarness {
  ipc::MessageBus bus;
  sil::ControlService control;
  sil::ImuService imu;
  ipc::TypedPublisher<ipc::UdpBridge> ingress;
  std::uint64_t messages_processed = 0;
  std::uint64_t motor_targets_seen = 0;
  std::uint64_t telemetry_seen = 0;
  double last_left_sps = 0.0;
  double last_right_sps = 0.0;
  double last_u_sps = 0.0;

  ServiceHarness()
      : bus(this, &ServiceHarness::dispatch), control(bus), imu(bus, false), ingress(bus) {
    control.start();
  }

  ~ServiceHarness() {
    control.stop();
  }

  static void dispatch(void* ctx, MsgId id, const void* payload) {
    auto* self = static_cast<ServiceHarness*>(ctx);
    switch (id) {
      case MsgId::ImuRawData:
        self->imu.on_message<MsgId::ImuRawData>(*static_cast<const ipc::ImuRawPayload*>(payload));
        ++self->messages_processed;
        break;
      case MsgId::ImuData:
        self->control.on_message<MsgId::ImuData>(
            *static_cast<const ipc::ImuSamplePayload*>(payload));
        ++self->messages_processed;
        break;
      case MsgId::PhysicsTick:
        self->control.on_message<MsgId::PhysicsTick>(
            *static_cast<const PhysicsTickPayload*>(payload));
        ++self->messages_processed;
        break;
      case MsgId::JoystickCommand:
        self->control.on_message<MsgId::JoystickCommand>(
            *static_cast<const ipc::JoystickCommandPayload*>(payload));
        ++self->messages_processed;
        break;
      case MsgId::MotorFeedback:
        self->control.on_message<MsgId::MotorFeedback>(
            *static_cast<const ipc::MotorFeedbackPayload*>(payload));
        break;
      case MsgId::MotorTargets: {
        const auto& target = *static_cast<const ipc::MotorTargetsPayload*>(payload);
        self->last_left_sps = target.left_sps;
        self->last_right_sps = target.right_sps;
        ++self->motor_targets_seen;
        break;
      }
      case MsgId::SystemTelemetry:
        self->last_u_sps = static_cast<const ipc::SystemTelemetryPayload*>(payload)->u_sps;
        ++self->telemetry_seen;
        break;
      default:
        break;
    }
  }
};

void run_sequence(const std::vector<uint8_t>& input) {
  ServiceHarness harness;

  std::size_t offset = 0;
  while ((offset + sizeof(uint16_t)) <= input.size()) {
    uint16_t raw_id = 0;
    std::memcpy(&raw_id, input.data() + offset, sizeof(raw_id));
    offset += sizeof(raw_id);

    const MsgId id = static_cast<MsgId>(raw_id);
    const std::size_t payload_size = payload_size_for(id);
    if (payload_size == 0) {
      break;
    }

    const std::size_t remaining = input.size() - offset;
    const std::size_t actual_size = remaining >= payload_size ? payload_size : remaining;
    harness.ingress.publish_if_authorized(id, input.data() + offset, actual_size);
    if (actual_size != payload_size) {
      break;
    }
    offset += payload_size;
  }

  g_udp_sink = g_udp_sink ^ harness.messages_processed;
  g_udp_sink = g_udp_sink + harness.motor_targets_seen * 3;
  g_udp_sink = g_udp_sink + harness.telemetry_seen * 5;
  g_udp_sink = g_udp_sink + static_cast<std::uint64_t>(std::llround(
                                std::abs(harness.last_left_sps + harness.last_right_sps)));
  g_udp_sink = g_udp_sink + static_cast<std::uint64_t>(std::llround(std::abs(harness.last_u_sps)));
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    return 1;
  }

  ConfigPid::load(fuzz::repo_path("pid.conf"));

  while (__AFL_LOOP(1000)) {
    std::vector<uint8_t> input;
    if (!fuzz::read_binary_file(argv[1], input)) {
      return 1;
    }
    run_sequence(input);
  }

  return 0;
}
