from __future__ import annotations

import time
import json
import subprocess
from pathlib import Path

from generated_balancer import BalancerMsgId, SimRunDonePayload, SimStopRunPayload
from tests.python.support.simulator_service import (
    ACK_ACCEPTED,
    ACK_BUSY,
    DONE_STOPPED_BY_CLIENT,
    PHYSICS_SIMPLIFIED,
    make_start_payload,
    wait_for_ack,
    wait_for_done,
    run_scenario_live,
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
        if msg_id == int(BalancerMsgId.SimulatorTelemetry):
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


def test_run_summary_is_independent_of_telemetry_stride(simulator_udp, sim_artifact_settings):
    root = Path(sim_artifact_settings["temp_root"])
    common = {
        "physics_profile": PHYSICS_SIMPLIFIED,
        "duration_s": 1.0,
        "disturbances": [{"start_s": 0.2, "duration_s": 0.1, "force_n": 1.0}],
    }
    _summary_full, _metadata_full, full = run_scenario_live(
        simulator_udp,
        run_id=5401,
        output_dir=root / "stride_full",
        telemetry_stride=1,
        **common,
    )
    _summary_sparse, _metadata_sparse, sparse = run_scenario_live(
        simulator_udp,
        run_id=5402,
        output_dir=root / "stride_sparse",
        telemetry_stride=80,
        **common,
    )

    assert full.sample_count == sparse.sample_count
    assert full.max_abs_pitch_deg == sparse.max_abs_pitch_deg
    assert full.tail_rms_pitch_deg == sparse.tail_rms_pitch_deg
    assert full.max_continuous_saturation_s == sparse.max_continuous_saturation_s
    assert full.actuator_fault_count == sparse.actuator_fault_count
    assert full.controller_fault_flags == sparse.controller_fault_flags


def test_udp_transfer_scenario_matches_direct_engine_exactly(
    simulator_udp, simulator_binary, sim_artifact_settings
):
    scenario_index = 1  # nominal_push
    direct = json.loads(
        subprocess.check_output(
            [str(simulator_binary), "--direct-summary", str(scenario_index)], text=True
        )
    )
    _summary, _metadata, udp_done = run_scenario_live(
        simulator_udp,
        run_id=5501,
        output_dir=Path(sim_artifact_settings["temp_root"]) / "direct_udp_equivalence",
        physics_profile=PHYSICS_SIMPLIFIED,
        duration_s=20.0,
        telemetry_stride=0,
        transfer_scenario_index=scenario_index,
    )

    for field in (
        "sample_count",
        "elapsed_s",
        "final_pitch_deg",
        "max_abs_pitch_deg",
        "tail_rms_pitch_deg",
        "max_continuous_saturation_s",
        "actuator_fault_count",
        "controller_fault_flags",
        "timeline_hash",
    ):
        assert getattr(udp_done, field) == direct[field]
