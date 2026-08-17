from __future__ import annotations

import csv
import dataclasses
import json
import logging
import math
import socket
import subprocess
import struct
import sys
import threading
import time
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest.mock import MagicMock, Mock, patch

import pytest

from generated_balancer import (
    ConfigPidValuesPayload,
    JoystickCommandPayload,
    PidConfigOverridePayload,
    PidConfigStatusPayload,
    SimulatorTelemetryPayload,
    SystemTelemetryPayload,
)
from tools.telemetry_dashboard.server import (
    DISPLAY_HISTORY_POINTS,
    DISPLAY_HISTORY_SECONDS,
    DISPLAY_HZ,
    SYSTEM_TELEMETRY_ID,
    JoystickCommandController,
    DeploymentManager,
    CsvLogger,
    CsvPlayback,
    DIAGNOSTIC_LOGGER,
    PiHeartbeat,
    PID_CONFIG_FIELDS,
    PidSessionController,
    SseHub,
    TelemetryState,
    UdpReceiver,
    load_playback_csv,
    make_handler,
    parse_args,
    resolve_ssh_alias,
    resolve_udp_host,
    SourceController,
    configure_diagnostic_logging,
)


def test_dashboard_live_import_does_not_require_pandas():
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            """
import builtins
_import = builtins.__import__
def without_pandas(name, *args, **kwargs):
    if name == "pandas":
        raise ModuleNotFoundError("No module named pandas", name="pandas")
    return _import(name, *args, **kwargs)
builtins.__import__ = without_pandas
import tools.telemetry_dashboard.server
""",
        ],
        cwd=Path(__file__).resolve().parents[2],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr


def telemetry_packet(**changes: float | int) -> bytes:
    base = SystemTelemetryPayload.unpack(bytes(SystemTelemetryPayload.WIRE_SIZE))
    return struct.pack("<H", SYSTEM_TELEMETRY_ID) + dataclasses.replace(base, **changes).pack()


def test_generated_telemetry_wire_sizes_and_round_trip():
    assert SystemTelemetryPayload.WIRE_SIZE == 416
    assert SimulatorTelemetryPayload.WIRE_SIZE == 544
    sample = dataclasses.replace(
        SystemTelemetryPayload.unpack(bytes(SystemTelemetryPayload.WIRE_SIZE)),
        run_id=17,
        packet_seq=23,
        loop_seq=29,
        sender_monotonic_ns=31,
    )
    assert SystemTelemetryPayload.unpack(sample.pack()) == sample
    simulator = SimulatorTelemetryPayload.unpack(bytes(SimulatorTelemetryPayload.WIRE_SIZE))
    simulator.system = sample
    assert SimulatorTelemetryPayload.unpack(simulator.pack()).system == sample


def test_dashboard_decodes_generated_payload_and_latches_flags():
    state = TelemetryState("pi.local")
    packet = telemetry_packet(
        pitch_deg=-1.25,
        corrected_axle_velocity_sps=-50.0,
        left_actual_steps=123,
        right_actual_steps=87,
        u_sps=123.5,
        controller_fault_flags=0x12,
        controller_saturation_flags=0x04,
        actuator_saturation_flags=0x03,
        actuator_fault=1.0,
        trim_learning_enabled=1,
        trim_learning_block_reason=0,
        pitch_feedback_sps=11.0,
        pitch_rate_feedback_sps=12.0,
        active_pitch_gain_sps_per_rad=6000.0,
        active_pitch_rate_gain_sps_per_rad_s=500.0,
        active_config_generation=99,
    )

    assert state.accept(packet, received_at=10.0)
    telemetry, status = state.snapshot(display_rate_hz=25.0)
    assert telemetry["attitude"]["pitch_deg"] == -1.25
    assert telemetry["motion"]["corrected_axle_velocity_sps"] == -50.0
    assert telemetry["motion"]["left_actual_steps"] == 123
    assert telemetry["motion"]["right_actual_steps"] == 87
    assert telemetry["controller"]["command_sps"] == 123.5
    assert telemetry["controller"]["trim_learning_enabled"] is True
    assert telemetry["controller"]["trim_learning_block_reason"] == 0
    assert telemetry["controller"]["pitch_feedback_sps"] == 11.0
    assert telemetry["controller"]["pitch_rate_feedback_sps"] == 12.0
    assert telemetry["controller"]["active_pitch_gain_sps_per_rad"] == 6000.0
    assert telemetry["controller"]["active_config_generation"] == 99
    assert "raw" not in telemetry
    assert status["latched_flags"] == {
        "controller": 0x12,
        "saturation": 0x04,
        "actuator_saturation": 0x03,
        "actuator": 1,
    }
    assert status["telemetry_accept_count"] == 1
    assert status["telemetry_accept_avg_ms"] is not None
    assert status["telemetry_accept_max_ms"] >= status["telemetry_accept_avg_ms"]
    assert status["csv_queue_depth"] is None


def test_dashboard_classifies_sender_receiver_gaps_and_run_resets():
    state = TelemetryState("pi.local")
    assert state.accept(
        telemetry_packet(
            run_id=41,
            packet_seq=1,
            loop_seq=1,
            sender_monotonic_ns=1_000_000_000,
            t_sec=0.0,
        ),
        received_at=10.0,
    )
    assert state.accept(
        telemetry_packet(
            run_id=41,
            packet_seq=2,
            loop_seq=2,
            sender_monotonic_ns=1_002_500_000,
            t_sec=0.0025,
        ),
        received_at=10.0025,
    )

    assert state.accept(
        telemetry_packet(
            run_id=41,
            packet_seq=3,
            loop_seq=3,
            sender_monotonic_ns=1_202_500_000,
            t_sec=0.2025,
        ),
        received_at=10.003,
    )
    _, status = state.snapshot(50.0)
    assert status["telemetry_gap_count"] == 1
    assert status["last_telemetry_gap"]["event"] == "telemetry_packet_gap"
    assert status["last_telemetry_gap"]["classification"] == "sender_control_dispatch_pause"
    assert status["sender_packet_seq"] == 3
    assert status["sender_loop_seq"] == 3

    assert state.accept(
        telemetry_packet(
            run_id=41,
            packet_seq=4,
            loop_seq=4,
            sender_monotonic_ns=1_205_000_000,
            t_sec=0.205,
        ),
        received_at=10.130,
    )
    _, status = state.snapshot(50.0)
    assert status["telemetry_gap_count"] == 2
    assert status["last_telemetry_gap"]["event"] == "udp_receive_pause"
    assert status["last_telemetry_gap"]["classification"] == "receiver_or_network_pause"

    assert state.accept(
        telemetry_packet(
            run_id=42,
            packet_seq=1,
            loop_seq=1,
            sender_monotonic_ns=500_000,
            t_sec=0.0,
        ),
        received_at=10.132,
    )
    _, status = state.snapshot(50.0)
    assert status["telemetry_gap_count"] == 2
    assert status["telemetry_sender_reset_count"] == 1
    assert status["last_telemetry_gap"]["classification"] == "receiver_or_network_pause"


def test_dashboard_reports_new_wire_metadata_and_receive_buffer():
    state = TelemetryState("pi.local")
    packet = telemetry_packet(run_id=7, packet_seq=9, loop_seq=11, sender_monotonic_ns=123456789)
    assert state.accept(packet, received_at=10.0)
    telemetry, status = state.snapshot(50.0)
    assert telemetry["run_id"] == 7
    assert telemetry["packet_seq"] == 9
    assert telemetry["loop_seq"] == 11
    assert telemetry["sender_monotonic_ns"] == 123456789
    assert status["sender_run_id"] == 7
    assert status["sender_packet_seq"] == 9
    assert status["sender_loop_seq"] == 11

    receiver = UdpReceiver(state, None, 9000)
    try:
        assert receiver.receive_buffer_bytes > 0
        assert state.snapshot(0.0)[1]["udp_receive_buffer_bytes"] == receiver.receive_buffer_bytes
    finally:
        receiver.close()


def test_dashboard_counts_packet_and_loop_sequence_gaps():
    state = TelemetryState("pi.local")
    assert state.accept(
        telemetry_packet(
            run_id=9,
            packet_seq=1,
            loop_seq=1,
            sender_monotonic_ns=1_000_000_000,
        ),
        received_at=10.0,
    )
    assert state.accept(
        telemetry_packet(
            run_id=9,
            packet_seq=4,
            loop_seq=6,
            sender_monotonic_ns=1_007_500_000,
        ),
        received_at=10.0075,
    )
    _, status = state.snapshot(50.0)
    assert status["telemetry_packet_sequence_gap_count"] == 2
    assert status["telemetry_loop_sequence_gap_count"] == 4
    assert status["last_telemetry_gap"]["classification"] == "loop_without_packet_gap"


def test_dashboard_classifies_transport_and_combined_gaps():
    transport = TelemetryState("pi.local")
    assert transport.accept(
        telemetry_packet(run_id=3, packet_seq=1, loop_seq=1, sender_monotonic_ns=1_000_000_000),
        received_at=5.0,
    )
    assert transport.accept(
        telemetry_packet(run_id=3, packet_seq=3, loop_seq=3, sender_monotonic_ns=1_005_000_000),
        received_at=5.005,
    )
    assert transport.snapshot(50.0)[1]["last_gap_classification"] == "packet_or_transport_gap"

    combined = TelemetryState("pi.local")
    assert combined.accept(
        telemetry_packet(run_id=4, packet_seq=1, loop_seq=1, sender_monotonic_ns=2_000_000_000),
        received_at=8.0,
    )
    assert combined.accept(
        telemetry_packet(run_id=4, packet_seq=2, loop_seq=2, sender_monotonic_ns=2_250_000_000),
        received_at=8.250,
    )
    assert combined.snapshot(50.0)[1]["last_gap_classification"] == "receiver_and_sender_gap"


def test_browser_cache_rollover_is_batched():
    dashboard = Path(__file__).resolve().parents[2] / "tools" / "telemetry_dashboard" / "static" / "dashboard.js"
    source = dashboard.read_text(encoding="utf-8")
    assert "TRIM_BATCH_POINTS" in source
    assert "store.samples.shift()" not in source
    assert 'id:"freeze-diagnostics"' in source
    assert "dashboard-diagnostics" not in source


def test_status_separates_telemetry_freshness_from_pi_reachability():
    state = TelemetryState("rpi4")
    assert state.accept(telemetry_packet(), received_at=time.monotonic())
    _, status = state.snapshot(0.0)
    assert status["telemetry_connected"]
    assert not status["pi_ready"]


def test_pid_session_exposes_numeric_fields_and_applies_complete_snapshot():
    state = TelemetryState("rpi4")
    receiver = Mock()
    receiver.send_message.return_value = True
    controller = PidSessionController(state, receiver)
    assert not state.run_active

    initial = controller.snapshot()
    assert PID_CONFIG_FIELDS == (
        "pitch_gain",
        "pitch_rate_gain",
        "pitch_accel_gain",
        "drive_max_velocity_mps",
        "velocity_gain_per_s",
        "velocity_feedback_cutoff_hz",
        "outer_pitch_limit_deg",
        "fixed_com_trim_deg",
        "adaptive_com_trim_enabled",
        "adaptive_com_trim_gain_deg_per_mps_s",
        "adaptive_com_trim_limit_deg",
        "turn_max_sps",
        "balance_max_sps",
        "planner_max_acceleration_mps2",
        "planner_max_deceleration_mps2",
        "planner_max_jerk_mps3",
        "velocity_i_gain_per_s2",
        "velocity_i_leak_time_s",
        "velocity_i_acceleration_limit_mps2",
    )
    assert set(initial["values"]) == set(PID_CONFIG_FIELDS)
    assert set(initial["baseline"]) == set(PID_CONFIG_FIELDS)
    values = dict(initial["values"])
    values["pitch_gain"] = 6000.0
    result = controller.update(values)
    assert result["ok"] and result["sent"]
    message_id, encoded = receiver.send_message.call_args.args
    assert message_id == 3012
    payload = PidConfigOverridePayload.unpack(encoded)
    assert payload.request_id == result["request_id"]
    assert payload.values.pitch_gain == 6000.0

    controller.accept_status(
        PidConfigStatusPayload(
            request_id=payload.request_id,
            accepted=1,
            result_code=0,
            reserved=0,
            values=ConfigPidValuesPayload(**values),
        )
    )
    snapshot = controller.snapshot()
    assert snapshot["last_status"]["state"] == "applied"
    assert snapshot["values"]["pitch_gain"] == 6000.0
    assert state.snapshot(0.0)[1]["pid_status"]["accepted"] is True
    assert "values" not in state.snapshot(0.0)[1]["pid_status"]
    assert "values" not in snapshot["last_status"]

    invalid = dict(values)
    invalid["balance_max_sps"] = 16001.0
    with pytest.raises(ValueError, match="16000"):
        controller.update(invalid)


def test_controller_payloads_share_nested_block_with_current_wire_size():
    values = ConfigPidValuesPayload(**{name: float(index + 1) for index, name in enumerate(PID_CONFIG_FIELDS)})
    override = PidConfigOverridePayload(request_id=9, reserved=0, values=values)
    status = PidConfigStatusPayload(request_id=9, accepted=1, result_code=0, reserved=0, values=values)

    assert ConfigPidValuesPayload.WIRE_SIZE == 152
    assert len(override.pack()) == 160
    assert len(status.pack()) == 160
    assert PidConfigOverridePayload.unpack(override.pack()).values == values
    assert PidConfigStatusPayload.unpack(status.pack()).values == values


def test_joystick_command_is_run_gated_until_explicit_release():
    state = TelemetryState("rpi4")
    receiver = Mock()
    receiver.send_message.return_value = True
    controller = JoystickCommandController(state, receiver)

    with pytest.raises(ValueError, match="Start"):
        controller.send({"forward": 0.1, "turn": 0.0})

    state.set_run_active(True)
    result = controller.send({"forward": 0.1, "turn": 0.0})
    assert result["ok"] and result["sent"]
    message_id, encoded = receiver.send_message.call_args.args
    assert message_id == 3011
    payload = JoystickCommandPayload.unpack(encoded)
    assert payload.forward == 0.1 and payload.turn == 0.0

    time.sleep(0.15)
    _, encoded = receiver.send_message.call_args.args
    payload = JoystickCommandPayload.unpack(encoded)
    assert payload.forward == 0.1 and payload.turn == 0.0
    result = controller.send({"release": True})
    assert result["ok"] and result["sent"]
    _, encoded = receiver.send_message.call_args.args
    payload = JoystickCommandPayload.unpack(encoded)
    assert payload.forward == 0.0 and payload.turn == 0.0


def test_dashboard_control_http_endpoints_enforce_pid_snapshot_and_run_gate():
    state = TelemetryState("rpi4")
    hub = SseHub(state)
    receiver = Mock()
    receiver.send_message.return_value = True
    pid_controller = PidSessionController(state, receiver)
    command_controller = JoystickCommandController(state, receiver)
    server = ThreadingHTTPServer(
        ("127.0.0.1", 0),
        make_handler(hub, state, None, None, pid_controller, command_controller),
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        pid = json.load(urllib.request.urlopen(base + "/api/pid", timeout=1))
        assert set(pid["values"]) == set(PID_CONFIG_FIELDS)
        request = urllib.request.Request(
            base + "/api/joystick",
            data=json.dumps({"forward": 0.1, "turn": 0.0}).encode(),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with pytest.raises(urllib.error.HTTPError) as response:
            urllib.request.urlopen(request, timeout=1)
        assert response.value.code == 400

        state.set_run_active(True)
        response = json.load(urllib.request.urlopen(request, timeout=1))
        assert response["ok"] and response["sent"]
    finally:
        command_controller.close()
        server.shutdown()
        server.server_close()
        thread.join()

    state.set_pi_ready(True)
    with state.lock:
        state.last_packet_at = time.monotonic() - 2.0
    _, status = state.snapshot(0.0)
    assert not status["telemetry_connected"]
    assert status["pi_ready"]

    state.set_pi_ready(False)
    _, status = state.snapshot(0.0)
    assert not status["telemetry_connected"]
    assert not status["pi_ready"]


def test_joystick_heartbeat_and_neutral_cleanup_are_explicit():
    state = TelemetryState("rpi4")
    state.set_run_active(True)
    receiver = Mock()
    receiver.send_message.return_value = True
    controller = JoystickCommandController(state, receiver)

    controller.send({"forward": 0.25, "turn": 0.0})
    time.sleep(0.06)
    _, encoded = receiver.send_message.call_args.args
    assert JoystickCommandPayload.unpack(encoded).forward == 0.25
    controller.send({"forward": 0.5, "turn": 0.0})
    time.sleep(0.05)
    _, encoded = receiver.send_message.call_args.args
    assert JoystickCommandPayload.unpack(encoded).forward == 0.5

    time.sleep(0.08)
    _, encoded = receiver.send_message.call_args.args
    assert JoystickCommandPayload.unpack(encoded).forward == 0.5

    receiver.send_message.return_value = False
    with pytest.raises(RuntimeError, match="address"):
        controller.send({"forward": 0.1, "turn": 0.0})
    result = controller.send({"release": True})
    assert result["ok"] and not result["sent"]
    controller.close()


def test_csv_logger_writes_every_raw_field_with_fixed_header(tmp_path):
    logger = CsvLogger(tmp_path, max_bytes=1024 * 1024, retain_count=2)
    state = TelemetryState("pi.local", logger)
    for index in range(400):
        assert state.accept(telemetry_packet(pitch_deg=-1.25 + index), received_at=10.0 + index / 400.0)
    logger.close()
    log_path = next(tmp_path.glob("telemetry_*.csv"))
    with log_path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert "received_at_unix_s" in rows[0]
    assert "motor_feedback_age_ms" in rows[0]
    assert "left_slewed_sps" in rows[0]
    assert "right_slewed_sps" in rows[0]
    assert "actuator_saturation_flags" in rows[0]
    assert "packet_seq" in rows[0]
    assert "loop_seq" in rows[0]
    assert "sender_monotonic_ns" in rows[0]
    assert float(rows[0]["pitch_deg"]) == -1.25
    assert len(rows) == 400
    assert float(rows[-1]["pitch_deg"]) == 397.75


def test_csv_logger_rolls_at_hardware_run_boundary_without_losing_rows(tmp_path):
    logger = CsvLogger(tmp_path, max_bytes=1024 * 1024, retain_count=10)
    state = TelemetryState("pi.local", logger)
    for index in range(2):
        assert state.accept(telemetry_packet(pitch_deg=float(index)), received_at=10.0 + index / 400.0)

    state.begin_hardware_run()

    for index in range(2, 4):
        assert state.accept(telemetry_packet(pitch_deg=float(index)), received_at=10.0 + index / 400.0)
    logger.close()

    logs = sorted(tmp_path.glob("telemetry_*.csv"))
    assert len(logs) == 2
    with logs[0].open(newline="", encoding="utf-8") as handle:
        first_rows = list(csv.DictReader(handle))
    with logs[1].open(newline="", encoding="utf-8") as handle:
        second_rows = list(csv.DictReader(handle))
    assert [float(row["pitch_deg"]) for row in first_rows] == [0.0, 1.0]
    assert [float(row["pitch_deg"]) for row in second_rows] == [2.0, 3.0]


def test_csv_logger_roll_before_first_packet_is_lazy(tmp_path):
    logger = CsvLogger(tmp_path, max_bytes=1024 * 1024, retain_count=10)
    logger.roll()
    assert not list(tmp_path.glob("telemetry_*.csv"))

    state = TelemetryState("pi.local", logger)
    assert state.accept(telemetry_packet(pitch_deg=4.0), received_at=10.0)
    logger.close()
    assert len(list(tmp_path.glob("telemetry_*.csv"))) == 1


def test_csv_logger_keeps_size_rollover_as_fallback(tmp_path):
    logger = CsvLogger(tmp_path, max_bytes=1024, retain_count=100)
    state = TelemetryState("pi.local", logger)
    for index in range(20):
        assert state.accept(telemetry_packet(pitch_deg=float(index)), received_at=10.0 + index / 400.0)
    logger.close()

    logs = sorted(tmp_path.glob("telemetry_*.csv"))
    assert len(logs) > 1
    total_rows = 0
    for log in logs:
        with log.open(newline="", encoding="utf-8") as handle:
            total_rows += len(list(csv.DictReader(handle)))
    assert total_rows == 20


def test_dashboard_rejects_wrong_id_and_wrong_size():
    state = TelemetryState("pi.local")
    assert not state.accept(b"\x00\x00")
    assert not state.accept(struct.pack("<H", 3002) + bytes(SystemTelemetryPayload.WIRE_SIZE))
    assert not state.accept(
        struct.pack("<H", SYSTEM_TELEMETRY_ID) + bytes(SystemTelemetryPayload.WIRE_SIZE - 1)
    )
    _, status = state.snapshot(display_rate_hz=0.0)
    assert status["malformed_packets"] == 3


def test_sse_hub_coalesces_fast_input_and_preserves_short_fault():
    state = TelemetryState("pi.local")
    hub = SseHub(state)
    thread = threading.Thread(target=hub.run, daemon=True)
    thread.start()
    for index in range(20):
        assert state.accept(telemetry_packet(controller_fault_flags=1 if index == 3 else 0))
    time.sleep(0.14)
    version, telemetry, status = hub.wait_for_update(-1, timeout=0.1)
    hub.stop()
    assert version >= 1
    assert telemetry is not None and telemetry["sequence"] == 20
    assert status["display_rate_hz"] == DISPLAY_HZ
    assert status["latched_flags"]["controller"] == 1


def test_sse_hub_retains_only_display_rate_history():
    state = TelemetryState("pi.local")
    hub = SseHub(state)
    thread = threading.Thread(target=hub.run, daemon=True)
    thread.start()
    for index in range(20):
        assert state.accept(telemetry_packet(pitch_deg=float(index)))
    time.sleep(0.14)
    history = hub.history()
    hub.stop()
    assert len(history) == 1
    assert history[0]["sequence"] == 20
    assert "raw" not in history[0]


def test_history_exposes_clamped_timeline_bounds_and_display_runs():
    state = TelemetryState("pi.local")
    hub = SseHub(state)
    first = {"sequence": 1, "received_at": 10.0}
    second = {"sequence": 2, "received_at": 20.0}
    hub.set_static_history([first, second])
    assert hub.timeline_bounds() == (10.0, 20.0)
    assert hub.history(seconds=30.0, end=999.0) == [first, second]
    assert state.reset_display_run() == 1
    assert state.snapshot(0.0)[1]["display_run"] == 1


def test_live_display_history_is_exactly_one_two_minute_50_hz_run():
    state = TelemetryState("pi.local")
    hub = SseHub(state)
    for sequence in range(1, DISPLAY_HISTORY_POINTS + 101):
        hub._append_history({"sequence": sequence, "received_at": sequence / DISPLAY_HZ})
    history, decimated = hub.history_window(DISPLAY_HISTORY_SECONDS, DISPLAY_HISTORY_POINTS)
    assert len(history) == DISPLAY_HISTORY_POINTS
    assert not decimated
    assert history[-1]["sequence"] == DISPLAY_HISTORY_POINTS + 100

    csv_history = [{"sequence": sequence, "received_at": sequence / 100.0} for sequence in range(7000)]
    hub.set_static_history(csv_history)
    reduced, decimated = hub.history_window(120.0, DISPLAY_HISTORY_POINTS)
    assert decimated
    assert len(reduced) <= DISPLAY_HISTORY_POINTS
    assert reduced[-1] is csv_history[-1]


def test_start_boundary_clears_display_history_but_accepts_next_packet():
    state = TelemetryState("pi.local")
    hub = SseHub(state)
    state.accept(telemetry_packet(pitch_deg=1.0), received_at=10.0)
    first, _ = state.snapshot(DISPLAY_HZ)
    assert first is not None
    hub._append_history(first)
    state.reset_display_run()
    hub.begin_display_run()
    hub._append_history(first)
    assert hub.history() == []
    state.accept(telemetry_packet(pitch_deg=2.0), received_at=10.02)
    second, _ = state.snapshot(DISPLAY_HZ)
    assert second is not None
    hub._append_history(second)
    assert [sample["attitude"]["pitch_deg"] for sample in hub.history()] == [2.0]


def test_udp_telemetry_rate_is_measured_from_accepted_packets():
    state = TelemetryState("pi.local")
    base = time.monotonic()
    for index in range(400):
        assert state.accept(telemetry_packet(), received_at=base + index / 400.0)
    _, status = state.snapshot(DISPLAY_HZ)
    assert status["raw_packet_rate_hz"] == 400
    assert status["display_rate_hz"] == 50.0
    assert state.source_info()["run_limit_s"] == 120.0


def test_receive_and_controller_gaps_are_counted_and_written_as_jsonl(tmp_path):
    listener = configure_diagnostic_logging(tmp_path)
    try:
        state = TelemetryState("pi.local")
        assert state.accept(telemetry_packet(t_sec=1.0), received_at=10.0)
        assert state.accept(telemetry_packet(t_sec=1.0025), received_at=10.25)
        assert state.accept(telemetry_packet(t_sec=1.5), received_at=10.251)
        _, status = state.snapshot(50.0)
        assert status["telemetry_gap_count"] == 2
        assert status["last_telemetry_gap"]["event"] == "telemetry_packet_gap"
    finally:
        listener.stop()
        DIAGNOSTIC_LOGGER.handlers = [logging.NullHandler()]
        DIAGNOSTIC_LOGGER.propagate = True
    events = [json.loads(line) for line in (tmp_path / "dashboard_events.jsonl").read_text().splitlines()]
    assert [event["event"] for event in events] == ["udp_receive_pause", "telemetry_packet_gap"]
    assert events[0]["receive_gap_s"] == 0.25
    assert math.isclose(events[1]["controller_gap_s"], 0.4975, abs_tol=1e-6)


def test_playback_csv_normalizes_simulator_rows_without_relogging(tmp_path):
    path = tmp_path / "timeline.csv"
    path.write_text(
        "sim_time_s,pitch_deg,u_sps,controller_fault_flags\n0.0,1.5,20,2\n0.2,2.5,30,0\n",
        encoding="utf-8",
    )
    samples = load_playback_csv(path)
    assert [sample[0] for sample in samples] == [0.0, 0.2]
    assert samples[0][1]["attitude"]["pitch_deg"] == 1.5
    assert samples[1][1]["controller"]["command_sps"] == 30.0
    assert samples[0][1]["flags"]["controller"] == 2

    state = TelemetryState("CSV: timeline.csv")
    hub = SseHub(state)
    playback = CsvPlayback(state, path, speed=0.0, loop=False, history_sink=hub.record_playback)
    playback.run()
    telemetry, _ = state.snapshot(0.0)
    assert telemetry is not None and telemetry["sequence"] == 2
    assert telemetry["controller"]["command_sps"] == 30.0
    assert len(hub.history()) == 2


def test_dashboard_requires_exactly_one_live_or_csv_source(tmp_path):
    assert parse_args(["--pi", "rpi4"]).pi == "rpi4"
    assert parse_args(["--csv", str(tmp_path / "capture.csv")]).csv.name == "capture.csv"
    assert parse_args([]).pi == "rpi4"


def test_ephemeral_csv_source_is_windowed_and_removed_when_returning_live(tmp_path):
    state = TelemetryState()
    hub = SseHub(state)
    receiver = UdpReceiver(state, None, 9000)
    source = SourceController(state, hub, receiver, 9000)
    response = source.upload_csv("capture.csv", b"t_sec,pitch_deg\n0,1\n1,2\n2,3\n")
    assert response["ok"]
    assert state.source_info()["mode"] == "csv"
    assert [sample["attitude"]["pitch_deg"] for sample in hub.history(1, 10, 1.0)] == [1.0, 2.0]
    temporary = source.temp_path
    assert temporary is not None and temporary.exists()
    source.configure_live("rpi4")
    assert not temporary.exists()
    assert state.source_info()["mode"] == "live"
    receiver.close()


def test_receiver_registers_and_accepts_fake_pi_telemetry():
    fake_pi = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    fake_pi.bind(("127.0.0.1", 0))
    state = TelemetryState("127.0.0.1")
    receiver = UdpReceiver(state, "127.0.0.1", fake_pi.getsockname()[1])
    thread = threading.Thread(target=receiver.run, daemon=True)
    thread.start()
    fake_pi.settimeout(1.0)
    registration, dashboard_address = fake_pi.recvfrom(32)
    assert registration == b"\x00\x00"
    fake_pi.sendto(telemetry_packet(pitch_deg=4.5), dashboard_address)
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        telemetry, _ = state.snapshot(0.0)
        if telemetry is not None:
            break
        time.sleep(0.01)
    receiver.close()
    fake_pi.close()
    assert telemetry is not None
    assert telemetry["attitude"]["pitch_deg"] == 4.5


def test_receiver_retries_failed_resolution_but_reuses_successful_address():
    state = TelemetryState("rpi4")
    receiver = UdpReceiver(state, "rpi4", 9)
    resolution = Mock(side_effect=[RuntimeError("offline"), "127.0.0.1"])
    thread = threading.Thread(target=receiver.run, daemon=True)
    try:
        with patch("tools.telemetry_dashboard.server.resolve_udp_host", resolution), patch(
            "tools.telemetry_dashboard.server.RESOLVE_RETRY_S", 0.01
        ):
            thread.start()
            deadline = time.monotonic() + 1.0
            while resolution.call_count < 2 and time.monotonic() < deadline:
                time.sleep(0.01)
            time.sleep(0.25)
    finally:
        receiver.close()
        thread.join(timeout=1.0)
    assert resolution.call_count == 2


def test_pi_heartbeat_reports_ssh_port_reachability_without_authentication():
    state = TelemetryState("rpi4")
    receiver = Mock()
    receiver.current_target.return_value = "rpi4"
    heartbeat = PiHeartbeat(state, receiver)
    with patch("tools.telemetry_dashboard.server.resolve_udp_host", return_value="192.168.1.44"), patch(
        "tools.telemetry_dashboard.server.socket.create_connection", return_value=MagicMock()
    ) as connect:
        assert heartbeat.probe()
    connect.assert_called_once_with(("192.168.1.44", 22), timeout=1.5)
    with patch("tools.telemetry_dashboard.server.resolve_udp_host", side_effect=RuntimeError("offline")):
        assert not heartbeat.probe()


def test_pi_heartbeat_refreshes_receiver_udp_address():
    state = TelemetryState("rpi4")
    receiver = UdpReceiver(state, "rpi4", 9000)
    heartbeat = PiHeartbeat(state, receiver)
    try:
        with patch("tools.telemetry_dashboard.server.resolve_udp_host", return_value="192.168.1.45"), patch(
            "tools.telemetry_dashboard.server.socket.create_connection", return_value=MagicMock()
        ):
            assert heartbeat.probe()
        with receiver.lock:
            assert receiver.pi_address == ("192.168.1.45", 9000)
    finally:
        receiver.close()


def test_ssh_alias_uses_ssh_config_hostname_and_accepts_user_prefix():
    completed = Mock(stdout="host rpi4\nhostname 192.168.1.44\nuser pi\n")
    with patch("tools.telemetry_dashboard.server.subprocess.run", return_value=completed) as run:
        assert resolve_ssh_alias("pi@rpi4") == "192.168.1.44"
    assert run.call_args.args[0] == ["ssh", "-G", "rpi4"]


def test_udp_host_falls_back_to_pi_mdns_name_when_ssh_is_unavailable():
    with patch("tools.telemetry_dashboard.server.resolve_ssh_alias", return_value="rpi4"), patch(
        "tools.telemetry_dashboard.server.socket.getaddrinfo",
        side_effect=[socket.gaierror, [(socket.AF_INET, socket.SOCK_DGRAM, 17, "", ("192.168.1.26", 0))]],
    ):
        assert resolve_udp_host("rpi4") == "192.168.1.26"


def write_versioned_binary(binary: Path, version: int) -> None:
    binary.write_bytes(b"\x7fELF\0BALANCER_PID_CONFIG_VERSION=" + str(version).encode() + b"\0")


def test_deployer_uses_current_build_with_host_scp_and_never_starts_robot(tmp_path, monkeypatch):
    deployer = DeploymentManager("pi@rpi4")
    completed = Mock(returncode=0, stdout="", stderr="")
    binary, config = tmp_path / "balancer_pi", tmp_path / "pid.conf"
    write_versioned_binary(binary, 3)
    config.write_text("config_version = 3\n")
    monkeypatch.setattr("tools.telemetry_dashboard.server.PI_BINARY", binary)
    monkeypatch.setattr("tools.telemetry_dashboard.server.PID_CONFIG", config)
    with patch("tools.telemetry_dashboard.server.subprocess.run", return_value=completed) as run:
        result = deployer.deploy_current()
    assert result["ok"]
    command = run.call_args.args[0]
    assert command[0] == "scp"
    assert command[-1] == "pi@rpi4:~/"
    assert "ssh" not in command


def test_deployer_checks_embedded_pid_config_version_only_for_deploy(tmp_path, monkeypatch):
    deployer = DeploymentManager("pi@rpi4")
    binary, config = tmp_path / "balancer_pi", tmp_path / "pid.conf"
    binary.write_bytes(b"ELF")
    config.write_text("config_version = 3\n")
    monkeypatch.setattr("tools.telemetry_dashboard.server.PI_BINARY", binary)
    monkeypatch.setattr("tools.telemetry_dashboard.server.PID_CONFIG", config)
    with pytest.raises(ValueError, match="no unambiguous PID config version"):
        deployer.deploy_current()
    write_versioned_binary(binary, 2)
    with pytest.raises(ValueError, match="binary expects 2"):
        deployer.deploy_current()
    write_versioned_binary(binary, 3)
    config.write_text("config_version = 3\ncontroller_enabled = 1\n")
    completed = Mock(returncode=0, stdout="", stderr="")
    with patch("tools.telemetry_dashboard.server.subprocess.run", return_value=completed):
        assert deployer.deploy_current()["ok"]


def test_deployer_start_does_not_require_a_current_local_build(tmp_path, monkeypatch):
    deployer = DeploymentManager("pi@rpi4")
    binary, config = tmp_path / "balancer_pi", tmp_path / "pid.conf"
    monkeypatch.setattr("tools.telemetry_dashboard.server.PI_BINARY", binary)
    monkeypatch.setattr("tools.telemetry_dashboard.server.PID_CONFIG", config)
    completed = Mock(returncode=0, stdout="", stderr="")
    with patch("tools.telemetry_dashboard.server.subprocess.run", return_value=completed) as run:
        assert deployer.start()["ok"]
    assert run.call_args.args[0][0] == "ssh"


def test_start_and_abort_use_short_lived_ssh_commands(tmp_path, monkeypatch):
    deployer = DeploymentManager("pi@rpi4")
    completed = Mock(returncode=0, stdout="", stderr="")
    binary, config = tmp_path / "balancer_pi", tmp_path / "pid.conf"
    write_versioned_binary(binary, 3)
    config.write_text("config_version = 3\n")
    monkeypatch.setattr("tools.telemetry_dashboard.server.PI_BINARY", binary)
    monkeypatch.setattr("tools.telemetry_dashboard.server.PID_CONFIG", config)
    with patch("tools.telemetry_dashboard.server.subprocess.run", return_value=completed) as run:
        assert deployer.start()["ok"]
        assert deployer.abort()["ok"]
    commands = [call.args[0] for call in run.call_args_list]
    assert commands[0][:2] == ["ssh", "pi@rpi4"]
    assert "nohup sudo -n ./balancer_pi" in commands[0][2]
    assert "cd ~ || exit 1;" in commands[0][2]
    assert "&& nohup" not in commands[0][2]
    assert "sleep 1" in commands[0][2]
    assert "tail -n 40 ~/balancer_pi.log" in commands[0][2]
    assert "pkill -TERM -x balancer_pi" in commands[1][2]
    assert "pkill -KILL" not in commands[1][2]
    assert "status" in commands[1][2]


def test_start_surfaces_immediate_remote_failure_log(tmp_path, monkeypatch):
    deployer = DeploymentManager("pi@rpi4")
    binary, config = tmp_path / "balancer_pi", tmp_path / "pid.conf"
    write_versioned_binary(binary, 3)
    config.write_text("config_version = 3\n")
    monkeypatch.setattr("tools.telemetry_dashboard.server.PI_BINARY", binary)
    monkeypatch.setattr("tools.telemetry_dashboard.server.PID_CONFIG", config)
    completed = Mock(
        returncode=1,
        stdout="Start failed (exit 134):\nwhat(): PID configuration version mismatch: expected 3, got 2\n",
        stderr="",
    )
    with patch("tools.telemetry_dashboard.server.subprocess.run", return_value=completed):
        with pytest.raises(RuntimeError, match="PID configuration version mismatch: expected 3, got 2"):
            deployer.start()


def test_successful_live_start_rolls_raw_capture(tmp_path):
    logger = CsvLogger(tmp_path, max_bytes=1024 * 1024, retain_count=10)
    state = TelemetryState("rpi4", logger)
    hub = SseHub(state)
    deployer = Mock()
    deployer.start.return_value = {"ok": True, "message": "started"}
    server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(hub, state, deployer))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        assert state.accept(telemetry_packet(pitch_deg=1.0), received_at=10.0)
        request = urllib.request.Request(
            f"http://127.0.0.1:{server.server_address[1]}/api/start", method="POST"
        )
        response = json.load(urllib.request.urlopen(request, timeout=1))
        assert response["ok"]
        assert response["display_run"] == 1
        assert state.snapshot(0.0)[1]["run_active"]
        assert state.accept(telemetry_packet(pitch_deg=2.0), received_at=10.01)
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
        logger.close()

    logs = sorted(tmp_path.glob("telemetry_*.csv"))
    assert len(logs) == 2
    with logs[0].open(newline="", encoding="utf-8") as handle:
        first_rows = list(csv.DictReader(handle))
    with logs[1].open(newline="", encoding="utf-8") as handle:
        second_rows = list(csv.DictReader(handle))
    assert [float(row["pitch_deg"]) for row in first_rows] == [1.0]
    assert [float(row["pitch_deg"]) for row in second_rows] == [2.0]
    deployer.start.assert_called_once_with()


def test_failed_live_start_does_not_roll_raw_capture(tmp_path):
    logger = CsvLogger(tmp_path, max_bytes=1024 * 1024, retain_count=10)
    state = TelemetryState("rpi4", logger)
    hub = SseHub(state)
    deployer = Mock()
    deployer.start.side_effect = RuntimeError("start failed")
    server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(hub, state, deployer))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        assert state.accept(telemetry_packet(pitch_deg=1.0), received_at=10.0)
        request = urllib.request.Request(
            f"http://127.0.0.1:{server.server_address[1]}/api/start", method="POST"
        )
        with pytest.raises(urllib.error.HTTPError) as response:
            urllib.request.urlopen(request, timeout=1)
        assert response.value.code == 400
        assert not state.snapshot(0.0)[1]["run_active"]
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
        logger.close()

    logs = sorted(tmp_path.glob("telemetry_*.csv"))
    assert len(logs) == 1
    with logs[0].open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert [float(row["pitch_deg"]) for row in rows] == [1.0]


def test_dashboard_operation_failures_are_written_to_jsonl(tmp_path):
    listener = configure_diagnostic_logging(tmp_path)
    state = TelemetryState("rpi4")
    hub = SseHub(state)
    deployer = Mock()
    deployer.start.side_effect = subprocess.TimeoutExpired(["ssh", "pi@rpi4"], 15)
    server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(hub, state, deployer))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    request = urllib.request.Request(
        f"http://127.0.0.1:{server.server_address[1]}/api/start", method="POST"
    )
    try:
        with pytest.raises(urllib.error.HTTPError) as response:
            urllib.request.urlopen(request, timeout=1)
        assert response.value.code == 400
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
        listener.stop()
        DIAGNOSTIC_LOGGER.handlers = [logging.NullHandler()]
        DIAGNOSTIC_LOGGER.propagate = True
    events = [json.loads(line) for line in (tmp_path / "dashboard_events.jsonl").read_text().splitlines()]
    assert events[-1]["event"] == "dashboard_operation_failure"
    assert events[-1]["operation"] == "start"
    assert events[-1]["error_type"] == "TimeoutExpired"


def test_deployer_normalizes_plain_pi_hostname_to_pi_user():
    assert DeploymentManager("rpi4").info()["target"] == "pi@rpi4"


def test_dashboard_serves_assets_and_sse_to_multiple_clients():
    state = TelemetryState("pi.local")
    hub = SseHub(state)
    hub_thread = threading.Thread(target=hub.run, daemon=True)
    hub_thread.start()
    server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(hub, state))
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    base = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        assert b"Balancer telemetry" in urllib.request.urlopen(base + "/", timeout=1).read()
        index = urllib.request.urlopen(base + "/", timeout=1).read()
        assert b'src="vendor/uPlot-1.6.32.iife.min.js"' in index
        assert b'src="dashboard.js"' in index
        script = urllib.request.urlopen(base + "/dashboard.js", timeout=1)
        assert script.headers.get_content_type() == "text/javascript"
        script = script.read()
        assert b"EventSource" in script
        assert b"setData" in script
        assert b"updateMetrics(null)" in script
        assert b"updateMetrics(store.samples.at(-1)??null)" in script
        assert b'numberAt(sample,"motion.corrected_axle_velocity_sps")' in script
        assert b'sampleOrStatus("timing.imu_age_ms","imu_age_ms")' in script
        assert b'sampleOrStatus("timing.feedback_age_ms","feedback_age_ms")' in script
        assert b'motion.measured_velocity_sps' not in script
        assert b"function liveConnection(status)" in script
        assert b"telemetry_connected===true" in script
        assert b"pi_ready===true" in script
        assert "Pi online · telemetry stopped".encode() in script
        assert b"joystick-controls" in script
        assert b"drive-pad" in script
        assert b"data-drive-pad" in script
        assert b"drivePadVectorAt" in script
        assert b"forward:-y" in script
        assert b'id:"joystick"' in script
        assert b'joystick.forward' in script
        assert b'joystick.turn' in script
        assert b"joystick_command_valid" in script
        assert b"setPointerCapture" in script
        assert b"JOYSTICK_REPEAT_MS = 100" in script
        assert b"data-joystick-track" not in script
        assert b"data-axis=\"forward\"" not in script
        assert b"data-axis=\"turn\"" not in script
        assert b"Neutral" in script
        assert b"joystick-value" not in script
        assert b"Drag for direction and speed; release to stop" in script
        assert b'id="joystick-stop"' not in script
        assert b'id="pid-load"' in script
        assert b">Load<" in script
        assert b'title:"Command"' not in script
        assert b'new Set(["performance","attitude","contributions","outer-loop"])' in script
        assert b'Balance performance' in script
        assert b'Composite activity' in script
        assert b'PERFORMANCE_WINDOW_S = 1.5' in script
        assert b'Filtered control rate' in script
        assert b'Balance contributions' in script
        assert b'Pitch rate / gyro diagnostics' in script
        assert b'Raw sensor angle' not in script
        assert b"PID_MIN_STEP = 0.0001" in script
        assert b"Math.round(raw/magnitude)" in script
        assert b'<div class="pid-actions"><button id="pid-apply"' in script
        assert b'<div class="pid-fields">${fields}</div>' in script
        assert b"lostpointercapture" in script
        assert b"sanity" not in script.lower()
        assert b"if(status.values)renderPidFields(status.values)" not in script
        assert b"Session PID tuning" in script
        stylesheet = urllib.request.urlopen(base + "/dashboard.css", timeout=1)
        assert stylesheet.headers.get_content_type() == "text/css"
        stylesheet_data = stylesheet.read()
        assert b'grid-template-areas: "left" "right" "main"' in stylesheet_data
        assert b".drive-pad" in stylesheet_data
        assert b"touch-action: none" in stylesheet_data
        assert b"overflow-x: auto" not in stylesheet_data
        assert b"max-width: 760px" in stylesheet_data
        uplot = urllib.request.urlopen(base + "/vendor/uPlot-1.6.32.iife.min.js", timeout=1)
        assert uplot.headers.get_content_type() == "text/javascript"
        assert b"uPlot" in uplot.read()
        uplot_css = urllib.request.urlopen(base + "/vendor/uPlot-1.6.32.min.css", timeout=1)
        assert uplot_css.headers.get_content_type() == "text/css"
        logo = urllib.request.urlopen(base + "/balancer-mark.svg", timeout=1)
        assert logo.headers.get_content_type() == "image/svg+xml"
        with pytest.raises(urllib.error.HTTPError) as missing:
            urllib.request.urlopen(base + "/%2e%2e/server.py", timeout=1)
        assert missing.value.code == 404
        history = json.load(urllib.request.urlopen(base + "/api/history?seconds=30", timeout=1))
        assert history["history_seconds"] == 30.0
        assert history["samples"] == []
        assert history["run_limit_s"] == 120.0
        assert history["display_sample_hz"] == 50.0
        assert history["decimated"] is False
        source = json.load(urllib.request.urlopen(base + "/api/source", timeout=1))
        assert source["mode"] == "live"
        assert source["display_sample_hz"] == 50.0
        first = urllib.request.urlopen(base + "/api/stream", timeout=1)
        second = urllib.request.urlopen(base + "/api/stream", timeout=1)
        state.accept(telemetry_packet(pitch_deg=2.0))
        def has_telemetry(response):
            return any(b"event: telemetry" in response.readline() for _ in range(8))

        assert has_telemetry(first)
        assert has_telemetry(second)
        first.close()
        second.close()
    finally:
        server.shutdown()
        server.server_close()
        hub.stop()


def test_sse_ignores_windows_client_connection_abort():
    state = TelemetryState("pi.local")
    hub = Mock()
    hub.wait_for_update.return_value = (0, None, {"connected": True})
    handler_type = make_handler(hub, state)

    class AbortedWriter:
        def write(self, _data):
            raise ConnectionAbortedError

        def flush(self):
            raise AssertionError("flush should not run after a failed write")

    class Handler:
        wfile = AbortedWriter()

        def send_response(self, _status):
            pass

        def send_header(self, _name, _value):
            pass

        def end_headers(self):
            pass

    handler_type._stream(Handler())
