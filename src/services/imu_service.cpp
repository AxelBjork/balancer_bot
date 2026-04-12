#include "imu_service.h"
#include "services/imu/ism330_iio_reader.h"

namespace sil {

ImuService::ImuService(ipc::MessageBus& bus, bool enable_hardware_reader)
    : bus_(bus)
{
    if (!enable_hardware_reader) {
        return;
    }

    reader_ = std::make_unique<Ism330IioReader>(Ism330IioReader::IMUConfig{
        [this](double pitch, std::array<double, 3> acc, std::array<double, 3> gyr, Ism330IioReader::TimePoint ts) {
            (void)pitch;
            filter_.push_sample(acc, gyr, ts);
            const ImuSample fused = filter_.read_latest();
            ipc::ImuSamplePayload payload{};
            payload.pitch_rad = fused.angle_rad;
            payload.acc = acc;
            payload.gyr = gyr;
            payload.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                       fused.t.time_since_epoch())
                                       .count();
            bus_.publish<MsgId::ImuData>(payload);
        }
    });
}

ImuService::~ImuService() {
    stop();
}

void ImuService::stop() {
    if (reader_) {
        reader_->stop();
    }
}

} // namespace sil
