
#pragma once
// Minimal standalone stub for PX4's rate_ctrl_status_s
struct rate_ctrl_status_s {
    float rollspeed_integ{0.f};
    float pitchspeed_integ{0.f};
    float yawspeed_integ{0.f};
};