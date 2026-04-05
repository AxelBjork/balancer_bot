import time

from generated_balancer import (
    BalancerMsgId,
    ImuSamplePayload,
    JoystickCommandPayload,
    MotorTargetsPayload,
)


def make_imu_sample(timestamp_us: int) -> ImuSamplePayload:
    return ImuSamplePayload(
        pitch_rad=0.0,
        acc=[0.0, 0.0, 9.81],
        gyr=[0.0, 0.0, 0.0],
        timestamp_us=timestamp_us,
    )


def test_joystick_to_motor_targets(udp):
    cmd = JoystickCommandPayload(forward=1.0, turn=0.5)
    udp.send(BalancerMsgId.JoystickCommand, cmd.pack())

    base_ts_us = time.monotonic_ns() // 1000
    for i in range(8):
        imu = make_imu_sample(base_ts_us + i * 5_000)
        udp.send(BalancerMsgId.ImuData, imu.pack())
        time.sleep(0.01)

    start_time = time.time()
    found = False
    while time.time() - start_time < 5.0:
        try:
            msg_id, payload_bytes = udp.recv(timeout=0.25)
            if msg_id == int(BalancerMsgId.MotorTargets):
                payload = MotorTargetsPayload.unpack(payload_bytes)
                if abs(payload.left_sps) > 0 or abs(payload.right_sps) > 0:
                    print(f"Received MotorTargets: left={payload.left_sps}, right={payload.right_sps}")
                    found = True
                    break
        except TimeoutError:
            pass

    assert found, "Did not receive MotorTargets message from sil_app"
