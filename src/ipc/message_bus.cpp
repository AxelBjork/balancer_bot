#include "message_bus.h"

namespace ipc {

void MessageBus::dispatch(MsgId id, const void* payload) {
  std::lock_guard<std::recursive_mutex> lock(dispatch_mu_);
  dispatcher_(ctx_, id, payload);
}

}  // namespace ipc
