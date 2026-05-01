#include "imu_service.h"

#include "services/imu/ism330_iio_reader.h"

namespace sil {

ImuService::ImuService(ipc::MessageBus& bus, bool enable_hardware_reader) : bus_(bus) {
  if (!enable_hardware_reader) {
    return;
  }

  reader_ = std::make_unique<Ism330IioReader>(
      Ism330IioReader::IMUConfig{[this](double pitch, std::array<double, 3> acc,
                                        std::array<double, 3> gyr, Ism330IioReader::TimePoint ts) {
        (void)pitch;
        ipc::ImuRawPayload payload{};
        payload.acc = acc;
        payload.gyr = gyr;
        payload.timestamp_us =
            std::chrono::duration_cast<std::chrono::microseconds>(ts.time_since_epoch()).count();
        bus_.publish<MsgId::ImuRawData>(payload);
      }});
}

ImuService::~ImuService() {
  stop();
}

void ImuService::stop() {
  if (reader_) {
    reader_->stop();
  }
}

void ImuService::handle_raw_imu(const ipc::ImuRawPayload& p) {
  const auto ts = std::chrono::steady_clock::time_point(std::chrono::microseconds(p.timestamp_us));
  filter_.push_sample(p.acc, p.gyr, ts);
  const ImuSample fused = filter_.read_latest();

  ipc::ImuSamplePayload payload{};
  payload.pitch_rad = fused.angle_rad;
  payload.filtered_pitch_rate_rad_s = fused.gyro_rad_s;
  payload.acc = p.acc;
  payload.gyr = p.gyr;
  payload.timestamp_us = p.timestamp_us;
  bus_.publish<MsgId::ImuData>(payload);
}

}  // namespace sil
