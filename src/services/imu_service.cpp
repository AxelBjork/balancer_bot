#include "imu_service.h"

namespace sil {

ImuService::ImuService(ipc::MessageBus& bus)
    : bus_(bus),
      reader_({
          // reader config configures the callback
          [this](double pitch, std::array<double, 3> acc, std::array<double, 3> gyr, Ism330IioReader::TimePoint ts) {
              ipc::ImuSamplePayload payload{};
              payload.pitch_rad = pitch;
              payload.acc = acc;
              payload.gyr = gyr;
              payload.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(ts.time_since_epoch()).count();
              bus_.publish<ipc::ImuData>(payload);
          }
      })
{
}

ImuService::~ImuService() {
    stop();
}

void ImuService::stop() {
    reader_.stop();
}

} // namespace sil
