import time

from generated_balancer import BalancerMsgId, ImuSamplePayload


def test_udp_bridge_relays_imu_data(udp):
    sample = ImuSamplePayload(
        pitch_rad=0.125,
        acc=[0.0, 0.0, 9.81],
        gyr=[0.0, 0.25, 0.0],
        timestamp_us=time.monotonic_ns() // 1000,
    )

    udp.send(BalancerMsgId.ImuData, sample.pack())

    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        msg_id, payload_bytes = udp.recv(timeout=0.1)
        if msg_id != int(BalancerMsgId.ImuData):
            continue

        echoed = ImuSamplePayload.unpack(payload_bytes)
        assert echoed.pitch_rad == sample.pitch_rad
        assert echoed.timestamp_us == sample.timestamp_us
        assert echoed.gyr[1] == sample.gyr[1]
        return

    raise AssertionError("UDP bridge did not relay ImuData back to the Python client")
