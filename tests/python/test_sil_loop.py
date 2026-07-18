import time

from generated_balancer import (
    BalancerMsgId,
    PhysicsTickPayload,
    ImuRawPayload,
    JoystickCommandPayload,
    MotorTargetsPayload,
    SystemTelemetryPayload,
)


def make_imu_sample(timestamp_us: int) -> ImuRawPayload:
    return ImuRawPayload(
        acc=[0.0, 0.0, -9.81],
        gyr=[0.0, 0.0, 0.0],
        timestamp_us=timestamp_us,
    )


def test_joystick_to_motor_targets(udp):
    cmd = JoystickCommandPayload(forward=1.0, turn=0.5)
    udp.send(BalancerMsgId.JoystickCommand, cmd.pack())

    base_time_us = time.monotonic_ns() // 1000
    for _ in range(32):
        sim_time_us = base_time_us + (_ + 1) * 2_500
        imu = make_imu_sample(sim_time_us)
        udp.send(BalancerMsgId.ImuRawData, imu.pack())
        udp.send(BalancerMsgId.PhysicsTick, PhysicsTickPayload(dt_s=0.0025, timestamp_us=sim_time_us).pack())

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


def test_tick_driven_telemetry_stream(udp):
    udp.send(BalancerMsgId.JoystickCommand, JoystickCommandPayload(forward=0.2, turn=0.0).pack())

    base_time_us = time.monotonic_ns() // 1000
    for _ in range(24):
        sim_time_us = base_time_us + (_ + 1) * 2_500
        udp.send(BalancerMsgId.ImuRawData, make_imu_sample(sim_time_us).pack())
        udp.send(BalancerMsgId.PhysicsTick, PhysicsTickPayload(dt_s=0.0025, timestamp_us=sim_time_us).pack())

    deadline = time.time() + 2.0
    while time.time() < deadline:
        try:
            msg_id, payload_bytes = udp.recv(timeout=0.1)
        except TimeoutError:
            continue

        if msg_id != int(BalancerMsgId.SystemTelemetry):
            continue

        payload = SystemTelemetryPayload.unpack(payload_bytes)
        assert payload.t_sec >= 0.0
        assert payload.age_ms >= 0.0
        assert payload.pitch_deg == payload.pitch_deg
        assert payload.u_sps == payload.u_sps
        return

    raise AssertionError("Did not receive SystemTelemetry message from sil_app")
