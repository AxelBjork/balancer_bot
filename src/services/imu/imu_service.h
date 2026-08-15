#pragma once

#include <memory>

#include "messages/balancer_msgs.h"
#include "publisher.h"

class Ism330IioReader;
class ImuPitchEstimator;

namespace sil {

inline constexpr char kImuServiceDoc[] =
    "Consumes raw accelerometer/gyroscope samples and publishes `ImuData` samples that represent "
    "the controller's current view of body pitch, angular motion, and sample time.\n\n"
    "The hardware reader converts synchronized sensor samples into SI-valued robot axes and "
    "publishes `ImuRawData`. This service applies 15 Hz accelerometer and 30 Hz gyro two-pole "
    "low-pass filters plus a 10 Hz filtered gyro derivative. Full-circle gravity pitch corrects "
    "short-term gyro prediction at 0.5 Hz, with each innovation limited symmetrically to 2.5 "
    "degrees so translation and motor vibration cannot abruptly steer attitude. The optional "
    "fixed notch and 70 mm lever-arm correction are disabled by default. It never learns gyro "
    "bias, mounting, gravity recovery modes, or COM correction, and marks invalid input invalid.\n\n"
    "SIL can disable the hardware reader and inject `ImuRawData` through `UdpBridge` while using "
    "the same estimator. `ImuData` remains an internal controller-facing contract.";

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
  std::unique_ptr<ImuPitchEstimator> estimator_;
  std::unique_ptr<Ism330IioReader> reader_;
};

template <>
inline void ImuService::on_message<MsgId::ImuRawData>(const ipc::ImuRawPayload& p) {
  handle_raw_imu(p);
}

}  // namespace sil
