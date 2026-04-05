#pragma once

#include "ism330_iio_reader.h"
#include "publisher.h"
#include "balancer_msgs.h"

namespace sil {

class ImuService {
public:
    using Publishes = ipc::MsgList<ipc::ImuData>;
    using Subscribes = ipc::MsgList<>;

    explicit ImuService(ipc::MessageBus& bus);
    ~ImuService();

    void start() {}
    void stop();

private:
    ipc::TypedPublisher<ImuService> bus_;
    Ism330IioReader reader_;
};

} // namespace sil
