#pragma once

#include "publisher.h"
#include "balancer_msgs.h"
#include "motor_runner.h"

namespace sil {

class DOC_DESC("Consumes wheel-speed targets and forwards them to the motor runner when hardware is attached.") MotorService {
public:
    using Publishes = ipc::MsgList<>;
    using Subscribes = ipc::MsgList<ipc::MotorTargets>;

    explicit MotorService(ipc::MessageBus& bus, MotorRunner* runner) 
        : bus_(bus), runner_(runner) {}
    ~MotorService() = default;

    void start() {}
    void stop() {}

    template <MsgId Id>
    void on_message(const typename MessageTraits<Id>::Payload& p) {}

private:
    ipc::TypedPublisher<MotorService> bus_;
    MotorRunner* runner_;
};

template <>
inline void MotorService::on_message<ipc::MotorTargets>(const ipc::MotorTargetsPayload& p) {
    if (runner_) {
        runner_->setTargets(p.left_sps, p.right_sps);
    }
}

} // namespace sil
