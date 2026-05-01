import time
import math

from generated_balancer import BalancerMsgId, ImuRawPayload, PhysicsTickPayload, SystemTelemetryPayload


def test_udp_bridge_accepts_raw_imu_data_for_control_telemetry(udp):
    pitch_rad = 0.125
    start_us = time.monotonic_ns() // 1000

    for i in range(240):
        timestamp_us = start_us + (i + 1) * 2500
        sample = ImuRawPayload(
            acc=[-9.81 * math.sin(pitch_rad), 0.0, 9.81 * math.cos(pitch_rad)],
            gyr=[0.0, 0.0, 0.0],
            timestamp_us=timestamp_us,
        )
        udp.send(BalancerMsgId.ImuRawData, sample.pack())
        udp.send(BalancerMsgId.PhysicsTick, PhysicsTickPayload(dt_s=0.0025, sim_time_us=timestamp_us).pack())

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
    assert math.isclose(latest.pitch_deg, math.degrees(pitch_rad), abs_tol=0.25)
    assert math.isclose(latest.raw_acc_pitch_deg, math.degrees(pitch_rad), abs_tol=1e-4)
