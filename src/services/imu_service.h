#pragma once

#include "balancer_msgs.h"
#include "publisher.h"

#include <memory>

class Ism330IioReader;

namespace sil {

class DOC_DESC("Publishes fused IMU samples onto the internal message bus. In SIL mode the hardware reader can be disabled and samples are injected over UDP instead.") ImuService {
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
