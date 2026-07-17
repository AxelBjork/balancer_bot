import time
import math

from generated_balancer import BalancerMsgId, ImuRawPayload, PhysicsTickPayload, SystemTelemetryPayload


def test_udp_bridge_accepts_raw_imu_data_for_control_telemetry(fresh_udp):
    udp = fresh_udp
    pitch_rad = 0.125
    start_us = time.monotonic_ns() // 1000

    # Rotate over 400 ms with a matching gyro signal, then hold. An accel-only
    # discontinuity is intentionally rejected because horizontal specific
    # force is indistinguishable from tilt on the balancing robot.
    ramp_samples = 160
    ramp_duration_s = ramp_samples * 0.0025
    for i in range(320):
        timestamp_us = start_us + (i + 1) * 2500
        progress = min(1.0, (i + 1) / ramp_samples)
        sample_pitch_rad = pitch_rad * progress
        pitch_rate_rad_s = pitch_rad / ramp_duration_s if i < ramp_samples else 0.0
        sample = ImuRawPayload(
            acc=[
                -9.81 * math.sin(sample_pitch_rad),
                0.0,
                9.81 * math.cos(sample_pitch_rad),
            ],
            gyr=[0.0, pitch_rate_rad_s, 0.0],
            timestamp_us=timestamp_us,
        )
        udp.send(BalancerMsgId.ImuRawData, sample.pack())
        udp.send(BalancerMsgId.PhysicsTick, PhysicsTickPayload(dt_s=0.0025, timestamp_us=timestamp_us).pack())

    latest = None
    deadline = time.monotonic() + 0.01
    while time.monotonic() < deadline:
        try:
            msg_id, payload_bytes = udp.recv(timeout=0.1)
        except TimeoutError:
            continue
        if msg_id != int(BalancerMsgId.SystemTelemetry):
            continue
        latest = SystemTelemetryPayload.unpack(payload_bytes)

    assert latest is not None, "UDP bridge did not produce telemetry from raw IMU input"
    assert math.isclose(latest.pitch_deg, math.degrees(pitch_rad), abs_tol=0.35)
    assert math.isclose(latest.raw_acc_pitch_deg, math.degrees(pitch_rad), abs_tol=1e-4)
