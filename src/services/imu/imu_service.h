#pragma once

#include <memory>

#include "messages/balancer_msgs.h"
#include "publisher.h"
#include "services/imu/pitch_lpf.h"

class Ism330IioReader;

namespace sil {

inline constexpr char kImuServiceDoc[] =
    "Consumes raw accelerometer/gyroscope samples and publishes `ImuData` samples that represent "
    "the controller's current view of body pitch, specific force, angular rate, and sample "
    "time.\n\n"
    "When hardware reading is enabled, the service owns an `Ism330IioReader` that discovers the "
    "split accel/gyro IIO devices, converts raw sensor counts into SI units, timestamps each "
    "sample, and publishes `ImuRawData` onto the internal message bus. The raw accelerometer and "
    "gyroscope vectors are then fused by a complementary filter before `ImuData` reaches "
    "control.\n\n"
    "In SIL mode the hardware reader can be disabled entirely, but Python can still inject "
    "`ImuRawData` through `UdpBridge` to exercise the same filter path. `ImuData` remains an "
    "internal controller-facing contract rather than a UDP payload.";

class DOC_DESC(kImuServiceDoc) ImuService {
 public:
  static constexpr const char* kDocDescription = kImuServiceDoc;

  using Publishes = ipc::MsgList<MsgId::ImuRawData, MsgId::ImuData>;
  using Subscribes = ipc::MsgList<MsgId::ImuRawData>;

  explicit ImuService(ipc::MessageBus& bus, bool enable_hardware_reader = true);
  ~ImuService();

  void start() {
  }
  void stop();

  template <MsgId Id>
  void on_message(const typename MessageTraits<Id>::Payload& p) {
  }

 private:
  void handle_raw_imu(const ipc::ImuRawPayload& p);

  ipc::TypedPublisher<ImuService> bus_;
  PitchComplementaryFilter filter_{};
  std::unique_ptr<Ism330IioReader> reader_;
};

template <>
inline void ImuService::on_message<MsgId::ImuRawData>(const ipc::ImuRawPayload& p) {
  handle_raw_imu(p);
}

}  // namespace sil
