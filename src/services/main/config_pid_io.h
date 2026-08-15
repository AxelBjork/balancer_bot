#pragma once

#include "messages/balancer_msgs.h"
#include "messages/types.h"

namespace sil {

ConfigPidValues pid_values_from_payload(const ConfigPidValues& payload);
void fill_pid_status_values(ConfigPidValues& payload, const ConfigPidValues& values);
ipc::PidConfigStatusPayload apply_pid_config_override(const ipc::PidConfigOverridePayload& payload);

}  // namespace sil
