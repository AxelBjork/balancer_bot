#include "message_bus.h"

namespace ipc {

void MessageBus::dispatch(MsgId id, const void* payload) {
  dispatcher_(ctx_, id, payload);
}

}  // namespace ipc
