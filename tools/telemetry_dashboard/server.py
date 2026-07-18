#!/usr/bin/env python3
"""Serve a read-only live dashboard for the balancer UDP telemetry stream."""
from __future__ import annotations

import argparse
import csv
import dataclasses
import json
import socket
import struct
import subprocess
import sys
import threading
import time
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
GENERATED_BINDINGS = ROOT / "tests" / "python"
if str(GENERATED_BINDINGS) not in sys.path:
    sys.path.insert(0, str(GENERATED_BINDINGS))

from generated_balancer import SystemTelemetryPayload  # noqa: E402

SYSTEM_TELEMETRY_ID = 3003
SYSTEM_TELEMETRY_SIZE = 512
DISPLAY_HZ = 25.0
DISCONNECT_AFTER_S = 1.0
REGISTER_INTERVAL_S = 2.0
STATIC_DIR = Path(__file__).with_name("static")
PI_BINARY = ROOT / "build-pi" / "balancer_pi"
PID_CONFIG = ROOT / "pid.conf"
SERVER_LOG_DIR = ROOT / "data" / "server"
LOG_ROTATE_BYTES = 128 * 1024 * 1024
LOG_RETAIN_COUNT = 10
TELEMETRY_FIELDS = [field.name for field in dataclasses.fields(SystemTelemetryPayload)]


def resolve_ssh_alias(target: str) -> str:
    """Resolve an SSH config alias to the host name that accepts UDP traffic.

    ``ssh -G`` expands the user's normal ``~/.ssh/config`` without connecting.
    This lets ``--pi rpi4`` use the same Host alias as ``ssh pi@rpi4`` and
    ``scp ... pi@rpi4:~/``. A plain DNS name/IP continues to work unchanged.
    """
    alias = target.rsplit("@", 1)[-1]
    try:
        result = subprocess.run(
            ["ssh", "-G", alias],
            check=True,
            capture_output=True,
            text=True,
            timeout=2,
        )
    except (FileNotFoundError, subprocess.CalledProcessError, subprocess.TimeoutExpired):
        return alias
    for line in result.stdout.splitlines():
        key, _, value = line.partition(" ")
        if key.lower() == "hostname" and value.strip():
            return value.strip()
    return alias


def resolve_udp_host(target: str) -> str:
    """Return an IPv4 UDP destination for an SSH alias or mDNS Pi name.

    Development containers do not always contain the ``ssh`` executable or
    the host machine's SSH configuration. Raspberry Pi OS advertises the
    conventional ``<hostname>.local`` mDNS name, so try it after the alias
    itself before producing an actionable startup error.
    """
    host = resolve_ssh_alias(target)
    candidates = [host]
    if "." not in host:
        candidates.append(f"{host}.local")
    for candidate in candidates:
        try:
            return socket.getaddrinfo(candidate, None, socket.AF_INET, socket.SOCK_DGRAM)[0][4][0]
        except socket.gaierror:
            continue
    tried = ", ".join(candidates)
    raise RuntimeError(
        f"Could not resolve Pi UDP address for {target!r} (tried {tried}). "
        "Use the Pi's .local name or LAN IP, or run the dashboard where the SSH alias is available."
    )


def telemetry_view(sample: SystemTelemetryPayload, sequence: int, received_at: float) -> dict[str, Any]:
    """Return the stable, browser-facing subset of the reflected wire payload."""
    return {
        "sequence": sequence,
        "received_at": received_at,
        # Retained unchanged for the fixed-schema CSV export. The visual UI only
        # consumes the curated groups below.
        "raw": dataclasses.asdict(sample),
        "t_sec": sample.t_sec,
        "attitude": {
            "pitch_deg": sample.pitch_deg,
            "fused_pitch_deg": sample.fused_pitch_deg,
            "raw_acc_pitch_deg": sample.raw_acc_pitch_deg,
            "pitch_setpoint_deg": sample.pitch_sp_deg,
        },
        "rate": {
            "pitch_rate_dps": sample.pitch_rate_dps,
            "filtered_pitch_rate_dps": sample.filtered_pitch_rate_dps,
            "gyro_pitch_rate_dps": sample.gyro_pitch_rate_dps,
            "rate_setpoint_dps": sample.rate_setpoint_dps,
        },
        "motion": {
            "target_velocity_sps": sample.target_velocity_sps,
            "measured_velocity_sps": sample.measured_vel_sps,
            "left_target_sps": sample.left_target_sps,
            "right_target_sps": sample.right_target_sps,
            "left_applied_sps": sample.left_applied_sps,
            "right_applied_sps": sample.right_applied_sps,
        },
        "controller": {
            "command_sps": sample.u_sps,
            "velocity_error": sample.vel_error,
            "pitch_error_deg": sample.pitch_error_deg,
            "velocity_p_term_deg": sample.velocity_p_term_deg,
            "velocity_i_term_deg": sample.velocity_i_term_deg,
        },
        "timing": {"imu_age_ms": sample.age_ms, "feedback_age_ms": sample.motor_feedback_age_ms},
        "flags": {
            "controller": sample.controller_fault_flags,
            "saturation": sample.controller_saturation_flags,
            "actuator": int(sample.actuator_fault),
        },
    }


class CsvLogger:
    """Append every valid raw packet to bounded, fixed-schema server CSV files."""
    def __init__(self, directory: Path, max_bytes: int = LOG_ROTATE_BYTES, retain_count: int = LOG_RETAIN_COUNT) -> None:
        self.directory, self.max_bytes, self.retain_count = directory, max_bytes, retain_count
        self.file: Any | None = None
        self.writer: csv.writer | None = None
        self.index = 0

    def _open(self) -> None:
        self.directory.mkdir(parents=True, exist_ok=True)
        while True:
            stamp = time.strftime("%Y%m%d-%H%M%S")
            path = self.directory / f"telemetry_{stamp}_{self.index:02d}.csv"
            self.index += 1
            if not path.exists():
                break
        self.file = path.open("w", newline="", encoding="utf-8", buffering=1)
        self.writer = csv.writer(self.file)
        self.writer.writerow(["received_at_unix_s", "received_at_monotonic_s", *TELEMETRY_FIELDS])
        old_logs = sorted(self.directory.glob("telemetry_*.csv"), key=lambda item: item.stat().st_mtime)
        for old_log in old_logs[:-self.retain_count]:
            old_log.unlink()

    def write(self, sample: SystemTelemetryPayload, monotonic_s: float) -> None:
        if self.file is None or self.file.tell() >= self.max_bytes:
            self.close()
            self._open()
        assert self.writer is not None
        self.writer.writerow([time.time(), monotonic_s, *(getattr(sample, name) for name in TELEMETRY_FIELDS)])

    def close(self) -> None:
        if self.file is not None:
            self.file.close()
        self.file, self.writer = None, None


class TelemetryState:
    def __init__(self, pi_host: str, logger: CsvLogger | None = None) -> None:
        self.pi_host = pi_host
        self.logger = logger
        self.lock = threading.Lock()
        self.sequence = 0
        self.latest: dict[str, Any] | None = None
        self.last_packet_at: float | None = None
        self.malformed_packets = 0
        self.packet_times: deque[float] = deque()
        self.latched_flags = {"controller": 0, "saturation": 0, "actuator": 0}

    def accept(self, datagram: bytes, received_at: float | None = None) -> bool:
        if len(datagram) != 2 + SYSTEM_TELEMETRY_SIZE:
            with self.lock:
                self.malformed_packets += 1
            return False
        message_id = struct.unpack_from("<H", datagram)[0]
        if message_id != SYSTEM_TELEMETRY_ID:
            with self.lock:
                self.malformed_packets += 1
            return False
        try:
            sample = SystemTelemetryPayload.unpack(datagram[2:])
        except (struct.error, ValueError):
            with self.lock:
                self.malformed_packets += 1
            return False

        now = time.monotonic() if received_at is None else received_at
        if self.logger is not None:
            self.logger.write(sample, now)
        with self.lock:
            self.sequence += 1
            self.latest = telemetry_view(sample, self.sequence, now)
            self.last_packet_at = now
            self.packet_times.append(now)
            while self.packet_times and self.packet_times[0] < now - 1.0:
                self.packet_times.popleft()
            for key, value in self.latest["flags"].items():
                self.latched_flags[key] |= value
        return True

    def clear_latched_flags(self) -> None:
        with self.lock:
            self.latched_flags = {"controller": 0, "saturation": 0, "actuator": 0}

    def snapshot(self, display_rate_hz: float) -> tuple[dict[str, Any] | None, dict[str, Any]]:
        now = time.monotonic()
        with self.lock:
            age = None if self.last_packet_at is None else max(0.0, now - self.last_packet_at)
            status = {
                "pi": self.pi_host,
                "connected": age is not None and age <= DISCONNECT_AFTER_S,
                "last_packet_age_ms": None if age is None else age * 1000.0,
                "raw_packet_rate_hz": len(self.packet_times),
                "malformed_packets": self.malformed_packets,
                "display_rate_hz": display_rate_hz,
                "latched_flags": dict(self.latched_flags),
            }
            return self.latest, status


class SseHub:
    """A 25 Hz coalescing fan-out shared by every connected browser tab."""
    def __init__(self, state: TelemetryState) -> None:
        self.state = state
        self.condition = threading.Condition()
        self.version = 0
        self.telemetry: dict[str, Any] | None = None
        self.status: dict[str, Any] = {}
        self._display_times: deque[float] = deque()
        self._stopping = False

    def run(self) -> None:
        interval = 1.0 / DISPLAY_HZ
        while not self._stopping:
            started = time.monotonic()
            self._display_times.append(started)
            while self._display_times and self._display_times[0] < started - 1.0:
                self._display_times.popleft()
            telemetry, status = self.state.snapshot(float(len(self._display_times)))
            with self.condition:
                self.telemetry, self.status = telemetry, status
                self.version += 1
                self.condition.notify_all()
            time.sleep(max(0.0, interval - (time.monotonic() - started)))

    def stop(self) -> None:
        self._stopping = True
        with self.condition:
            self.condition.notify_all()

    def wait_for_update(self, version: int, timeout: float = 15.0) -> tuple[int, dict[str, Any] | None, dict[str, Any]]:
        with self.condition:
            if self.version == version:
                self.condition.wait(timeout)
            return self.version, self.telemetry, self.status


class UdpReceiver:
    def __init__(self, state: TelemetryState, pi_host: str, pi_port: int) -> None:
        self.state, self.pi_address = state, (pi_host, pi_port)
        self.stopping = threading.Event()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("0.0.0.0", 0))
        self.socket.settimeout(0.2)

    def run(self) -> None:
        next_registration = 0.0
        while not self.stopping.is_set():
            now = time.monotonic()
            if now >= next_registration:
                self.socket.sendto(b"\x00\x00", self.pi_address)
                next_registration = now + REGISTER_INTERVAL_S
            try:
                packet, _ = self.socket.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            self.state.accept(packet)

    def close(self) -> None:
        self.stopping.set()
        self.socket.close()


class DeploymentManager:
    """Small host-side SCP deployment helper."""
    def __init__(self, ssh_target: str) -> None:
        self.ssh_target = ssh_target
        self.lock = threading.Lock()

    def info(self) -> dict[str, Any]:
        return {
            "target": self.ssh_target,
            "binary": str(PI_BINARY),
            "binary_exists": PI_BINARY.is_file(),
        }

    def _run(self, command: list[str], timeout: int) -> str:
        with self.lock:
            result = subprocess.run(command, capture_output=True, text=True, timeout=timeout)
        output = (result.stdout + result.stderr).strip()
        if result.returncode:
            raise RuntimeError(output or f"command failed with exit code {result.returncode}")
        return output

    def deploy_current(self) -> dict[str, Any]:
        if not PI_BINARY.is_file():
            raise ValueError(f"Cross-built binary not found: {PI_BINARY}. Run ./build_cmake OFF first.")
        if not PID_CONFIG.is_file():
            raise ValueError(f"PID config not found: {PID_CONFIG}")
        output = self._run(["scp", str(PI_BINARY), str(PID_CONFIG), f"{self.ssh_target}:~/"], 120)
        return {"ok": True, "message": output or "Deployed build-pi/balancer_pi and pid.conf to the Pi."}

    def start(self) -> dict[str, Any]:
        command = (
            "cd ~ && rm -f ~/balancer_pi.pid && "
            "(nohup sudo -n ./balancer_pi </dev/null >~/balancer_pi.log 2>&1 & "
            "echo $! >~/balancer_pi.pid)"
        )
        self._run(["ssh", self.ssh_target, command], 15)
        return {"ok": True, "message": "Start requested. Live output is in ~/balancer_pi.log on the Pi."}

    def abort(self) -> dict[str, Any]:
        command = (
            "status=0; "
            "sudo -n pkill -TERM -x balancer_pi || status=$?; "
            "rm -f ~/balancer_pi.pid; "
            "if test \"$status\" -gt 1; then exit \"$status\"; fi"
        )
        self._run(["ssh", self.ssh_target, command], 15)
        return {"ok": True, "message": "Requested shutdown of all balancer_pi processes on the Pi."}


def make_handler(hub: SseHub, state: TelemetryState, deployer: DeploymentManager | None = None):
    deployer = deployer or DeploymentManager("pi@rpi4")
    class DashboardHandler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, _format: str, *_args: Any) -> None:
            return

        def do_GET(self) -> None:  # noqa: N802
            if self.path == "/api/stream":
                self._stream()
                return
            if self.path == "/api/clear-flags":
                state.clear_latched_flags()
                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
                return
            if self.path == "/api/deploy-info":
                self._json(HTTPStatus.OK, deployer.info())
                return
            target = "index.html" if self.path in ("/", "/index.html") else self.path.lstrip("/")
            path = (STATIC_DIR / target).resolve()
            if STATIC_DIR not in path.parents or not path.is_file():
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            content_type = "text/html; charset=utf-8" if path.suffix == ".html" else "text/javascript; charset=utf-8" if path.suffix == ".js" else "text/css; charset=utf-8"
            data = path.read_bytes()
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_POST(self) -> None:  # noqa: N802
            actions = {
                "/api/deploy": deployer.deploy_current,
                "/api/start": deployer.start,
                "/api/abort": deployer.abort,
            }
            action = actions.get(self.path)
            if action is None:
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            try:
                self._json(HTTPStatus.OK, action())
            except (KeyError, ValueError, RuntimeError, subprocess.TimeoutExpired, FileNotFoundError) as exc:
                self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)})

        def _json(self, status: HTTPStatus, body: dict[str, Any]) -> None:
            data = json.dumps(body).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _stream(self) -> None:
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            version = -1
            try:
                while True:
                    version, telemetry, status = hub.wait_for_update(version)
                    if telemetry is not None:
                        self.wfile.write(b"event: telemetry\ndata: " + json.dumps(telemetry, separators=(",", ":")).encode() + b"\n\n")
                    self.wfile.write(b"event: status\ndata: " + json.dumps(status, separators=(",", ":")).encode() + b"\n\n")
                    self.wfile.flush()
            # A browser can close or replace its EventSource connection at any
            # time.  Windows reports a host-side abort as ConnectionAbortedError,
            # while other platforms commonly use the two exceptions below.
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass

    return DashboardHandler


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--pi",
        required=True,
        help="SSH Host alias (such as rpi4), hostname, IP, or user@SSH-alias",
    )
    parser.add_argument("--pi-port", type=int, default=9000)
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--listen-lan", action="store_true", help="Listen on all interfaces instead of localhost")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    logger = CsvLogger(SERVER_LOG_DIR)
    state = TelemetryState(args.pi, logger)
    udp_host = resolve_udp_host(args.pi)
    receiver = UdpReceiver(state, udp_host, args.pi_port)
    hub = SseHub(state)
    ssh_target = args.pi if "@" in args.pi else f"pi@{args.pi}"
    deployer = DeploymentManager(ssh_target)
    receiver_thread = threading.Thread(target=receiver.run, name="telemetry-udp", daemon=True)
    hub_thread = threading.Thread(target=hub.run, name="telemetry-sse", daemon=True)
    receiver_thread.start()
    hub_thread.start()
    host = "0.0.0.0" if args.listen_lan else "127.0.0.1"
    server = ThreadingHTTPServer((host, args.port), make_handler(hub, state, deployer))
    print(f"Dashboard: http://{host}:{args.port} (Pi UDP {args.pi} -> {udp_host}:{args.pi_port})")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        receiver.close()
        hub.stop()
        logger.close()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
