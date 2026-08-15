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

from generated_balancer import SystemTelemetryPayload
from tools.telemetry_dashboard.server import (
    DISPLAY_HISTORY_POINTS,
    DISPLAY_HISTORY_SECONDS,
    DISPLAY_HZ,
    SYSTEM_TELEMETRY_ID,
    DeploymentManager,
    CsvLogger,
    CsvPlayback,
    DIAGNOSTIC_LOGGER,
    PiHeartbeat,
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
    )

    assert state.accept(packet, received_at=10.0)
    telemetry, status = state.snapshot(display_rate_hz=25.0)
    assert telemetry["attitude"]["pitch_deg"] == -1.25
    assert telemetry["motion"]["corrected_axle_velocity_sps"] == -50.0
    assert telemetry["motion"]["left_actual_steps"] == 123
    assert telemetry["motion"]["right_actual_steps"] == 87
    assert telemetry["controller"]["command_sps"] == 123.5
    assert "raw" not in telemetry
    assert status["latched_flags"] == {
        "controller": 0x12,
        "saturation": 0x04,
        "actuator_saturation": 0x03,
        "actuator": 1,
    }


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
    assert float(rows[0]["pitch_deg"]) == -1.25
    assert len(rows) == 400
    assert float(rows[-1]["pitch_deg"]) == 397.75


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
        stylesheet = urllib.request.urlopen(base + "/dashboard.css", timeout=1)
        assert stylesheet.headers.get_content_type() == "text/css"
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
