from __future__ import annotations

import time

from generated_balancer import BalancerMsgId, SimRunDonePayload, SimStopRunPayload
from tests.python.support.simulator_service import (
    ACK_ACCEPTED,
    ACK_BUSY,
    DONE_STOPPED_BY_CLIENT,
    PHYSICS_SIMPLIFIED,
    make_start_payload,
    wait_for_ack,
    wait_for_done,
)


def test_balancer_simulator_start_ack_and_done(simulator_udp):
    run_id = 5101
    simulator_udp.send(
        BalancerMsgId.SimStartRun,
        make_start_payload(run_id=run_id, physics_profile=PHYSICS_SIMPLIFIED, duration_s=1.0).pack(),
    )

    ack = wait_for_ack(simulator_udp, run_id)
    assert ack.accepted == 1
    assert ack.status_code == ACK_ACCEPTED

    saw_telemetry = False
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        try:
            msg_id, payload = simulator_udp.recv(timeout=0.1)
        except TimeoutError:
            continue
        if msg_id == int(BalancerMsgId.SystemTelemetry):
            saw_telemetry = True
            continue
        if msg_id == int(BalancerMsgId.SimRunDone):
            done = SimRunDonePayload.unpack(payload)
            assert done.run_id == run_id
            assert done.sample_count > 0
            assert saw_telemetry
            return

    raise AssertionError("Did not receive SimRunDone from balancer_simulator")


def test_balancer_simulator_rejects_second_start_while_busy(simulator_udp):
    first_run = 5201
    second_run = 5202
    simulator_udp.send(
        BalancerMsgId.SimStartRun,
        make_start_payload(run_id=first_run, physics_profile=PHYSICS_SIMPLIFIED, duration_s=5.0).pack(),
    )
    first_ack = wait_for_ack(simulator_udp, first_run)
    assert first_ack.accepted == 1

    simulator_udp.send(
        BalancerMsgId.SimStartRun,
        make_start_payload(run_id=second_run, physics_profile=PHYSICS_SIMPLIFIED, duration_s=1.0).pack(),
    )
    second_ack = wait_for_ack(simulator_udp, second_run)
    assert second_ack.accepted == 0
    assert second_ack.status_code == ACK_BUSY

    simulator_udp.send(BalancerMsgId.SimStopRun, SimStopRunPayload(run_id=first_run).pack())
    done = wait_for_done(simulator_udp, first_run)
    assert done.reason_code == DONE_STOPPED_BY_CLIENT


def test_balancer_simulator_stop_run_terminates_active_scenario(simulator_udp):
    run_id = 5301
    simulator_udp.send(
        BalancerMsgId.SimStartRun,
        make_start_payload(run_id=run_id, physics_profile=PHYSICS_SIMPLIFIED, duration_s=20.0).pack(),
    )
    ack = wait_for_ack(simulator_udp, run_id)
    assert ack.accepted == 1

    simulator_udp.send(BalancerMsgId.SimStopRun, SimStopRunPayload(run_id=run_id).pack())
    done = wait_for_done(simulator_udp, run_id)
    assert done.reason_code == DONE_STOPPED_BY_CLIENT
