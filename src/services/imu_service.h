#pragma once

#include "messages/balancer_msgs.h"
#include "publisher.h"
#include "services/imu/pitch_lpf.h"

#include <memory>

class Ism330IioReader;

namespace sil {

inline constexpr char kImuServiceDoc[] =
    "Publishes `ImuData` samples that represent the controller's current view of body pitch, "
    "specific force, angular rate, and sample time.\n\n"
    "When hardware reading is enabled, the service owns an `Ism330IioReader` that discovers the "
    "split accel/gyro IIO devices, configures their trigger-driven buffers, converts raw sensor "
    "counts into SI units, and timestamps each sample before publishing it onto the internal "
    "message bus. The raw accelerometer and gyroscope vectors are fused by a complementary filter "
    "so the published pitch stays referenced to the balancing frame instead of jumping between "
    "upright and inverted branches of a raw accelerometer angle.\n\n"
    "In SIL mode the hardware reader can be disabled entirely, in which case this service becomes "
    "quiescent and the same `ImuData` payloads are injected externally through `UdpBridge`. That "
    "keeps the controller-facing contract identical across hardware and simulation: `ControlService` "
    "always consumes the same reflected payload shape regardless of whether the source is the "
    "physical ISM330 sensor path or the pytest harness.";

class DOC_DESC(kImuServiceDoc) ImuService {
public:
    static constexpr const char* kDocDescription = kImuServiceDoc;

    using Publishes = ipc::MsgList<MsgId::ImuData>;
    using Subscribes = ipc::MsgList<>;

    explicit ImuService(ipc::MessageBus& bus, bool enable_hardware_reader = true);
    ~ImuService();

    void start() {}
    void stop();

private:
    ipc::TypedPublisher<ImuService> bus_;
    PitchComplementaryFilter filter_{};
    std::unique_ptr<Ism330IioReader> reader_;
};

} // namespace sil
