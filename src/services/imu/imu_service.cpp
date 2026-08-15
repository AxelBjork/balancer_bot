#include "services/imu/imu_service.h"

#include "services/imu/imu_pitch_estimator.h"
#include "services/imu/ism330_iio_reader.h"

namespace sil {

ImuService::ImuService(ipc::MessageBus& bus, bool enable_hardware_reader)
    : bus_(bus), estimator_(std::make_unique<ImuPitchEstimator>()) {
  if (!enable_hardware_reader) {
    return;
  }

  reader_ = std::make_unique<Ism330IioReader>(
      Ism330IioReader::IMUConfig{[this](std::array<double, 3> acc, std::array<double, 3> gyr,
                                        Ism330IioReader::TimePoint ts) {
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
  const ImuPitchEstimate estimate = estimator_->push_sample(p.acc, p.gyr, ts);

  ipc::ImuSamplePayload payload{};
  payload.pitch_rad = estimate.sample.angle_rad;
  payload.pitch_rate_rad_s = estimate.sample.gyro_rad_s;
  payload.pitch_accel_rad_s2 = estimate.sample.pitch_accel_rad_s2;
  payload.acc = p.acc;
  payload.gyr = p.gyr;
  payload.timestamp_us = p.timestamp_us;
  payload.estimate_valid = estimate.valid;
  bus_.publish<MsgId::ImuData>(payload);
}

}  // namespace sil
