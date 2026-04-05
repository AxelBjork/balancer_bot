#pragma once

#include "balancer_msgs.h"
#include "publisher.h"

#include <memory>

class Ism330IioReader;

namespace sil {

class ImuService {
public:
    using Publishes = ipc::MsgList<ipc::ImuData>;
    using Subscribes = ipc::MsgList<>;

    explicit ImuService(ipc::MessageBus& bus, bool enable_hardware_reader = true);
    ~ImuService();

    void start() {}
    void stop();

private:
    ipc::TypedPublisher<ImuService> bus_;
    std::unique_ptr<Ism330IioReader> reader_;
};

} // namespace sil
