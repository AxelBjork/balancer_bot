#!/usr/bin/env python3
"""Serve a read-only live dashboard for the balancer UDP telemetry stream."""
from __future__ import annotations

import argparse
import csv
import dataclasses
import hashlib
import json
import logging
import logging.handlers
import queue
import re
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
from urllib.parse import parse_qs, urlsplit
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable

ROOT = Path(__file__).resolve().parents[2]
GENERATED_BINDINGS = ROOT / "tests" / "python"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(GENERATED_BINDINGS) not in sys.path:
    sys.path.insert(0, str(GENERATED_BINDINGS))

from generated_balancer import SystemTelemetryPayload  # noqa: E402

SYSTEM_TELEMETRY_ID = 3003
SYSTEM_TELEMETRY_SIZE = SystemTelemetryPayload.WIRE_SIZE
DISPLAY_HZ = 50.0
DISPLAY_HISTORY_SECONDS = 120.0
DISPLAY_HISTORY_POINTS = 6000
STATUS_HZ = 2.0
DISCONNECT_AFTER_S = 1.0
REGISTER_INTERVAL_S = 2.0
RESOLVE_RETRY_S = 10.0
MAX_UPLOAD_BYTES = 128 * 1024 * 1024
STATIC_DIR = Path(__file__).with_name("static")
PI_BINARY = ROOT / "build-pi" / "balancer_pi"
PID_CONFIG = ROOT / "pid.conf"
SERVER_LOG_DIR = ROOT / "data" / "server"
LOG_ROTATE_BYTES = 128 * 1024 * 1024
LOG_RETAIN_COUNT = 10
EVENT_LOG_ROTATE_BYTES = 4 * 1024 * 1024
TELEMETRY_GAP_S = 0.1
DEFAULT_PI_TARGET = "rpi4"
TELEMETRY_FIELDS = [field.name for field in dataclasses.fields(SystemTelemetryPayload)]
DIAGNOSTIC_LOGGER = logging.getLogger("balancer.telemetry_dashboard")
DIAGNOSTIC_LOGGER.addHandler(logging.NullHandler())


class JsonEventFormatter(logging.Formatter):
    def format(self, record: logging.LogRecord) -> str:
        event = dict(getattr(record, "event_data", {}))
        return json.dumps(
            {
                "logged_at_unix_s": time.time(),
                "level": record.levelname.lower(),
                "message": record.getMessage(),
                **event,
            },
            separators=(",", ":"),
        )


def configure_diagnostic_logging(directory: Path) -> logging.handlers.QueueListener:
    """Write operational warnings without blocking the UDP receiver thread."""
    directory.mkdir(parents=True, exist_ok=True)
    events = logging.handlers.RotatingFileHandler(
        directory / "dashboard_events.jsonl",
        maxBytes=EVENT_LOG_ROTATE_BYTES,
        backupCount=3,
        encoding="utf-8",
    )
    events.setFormatter(JsonEventFormatter())
    console = logging.StreamHandler()
    console.setFormatter(logging.Formatter("Dashboard warning: %(message)s"))
    event_queue: queue.SimpleQueue[logging.LogRecord] = queue.SimpleQueue()
    queue_handler = logging.handlers.QueueHandler(event_queue)
    DIAGNOSTIC_LOGGER.handlers = [queue_handler]
    DIAGNOSTIC_LOGGER.setLevel(logging.INFO)
    DIAGNOSTIC_LOGGER.propagate = False
    listener = logging.handlers.QueueListener(event_queue, events, console, respect_handler_level=True)
    listener.start()
    return listener


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
        self.bytes_written = 0
        self.rows: queue.SimpleQueue[list[Any] | None] = queue.SimpleQueue()
        self.closed = False
        self.error: BaseException | None = None
        self.worker = threading.Thread(target=self._run, name="telemetry-csv", daemon=True)
        self.worker.start()

    def _open(self) -> None:
        self.directory.mkdir(parents=True, exist_ok=True)
        while True:
            stamp = time.strftime("%Y%m%d-%H%M%S")
            path = self.directory / f"telemetry_{stamp}_{self.index:02d}.csv"
            self.index += 1
            if not path.exists():
                break
        # A large userspace buffer prevents a filesystem flush on every 400 Hz
        # telemetry row. close() still flushes the complete raw capture.
        self.file = path.open("w", newline="", encoding="utf-8", buffering=256 * 1024)
        self.writer = csv.writer(self.file)
        self.bytes_written = self.writer.writerow(["received_at_unix_s", "received_at_monotonic_s", *TELEMETRY_FIELDS])
        old_logs = sorted(self.directory.glob("telemetry_*.csv"), key=lambda item: item.stat().st_mtime)
        for old_log in old_logs[:-self.retain_count]:
            old_log.unlink()

    def write(self, sample: SystemTelemetryPayload, monotonic_s: float) -> None:
        if self.closed:
            raise RuntimeError("Cannot write to a closed telemetry CSV logger.")
        if self.error is not None:
            raise RuntimeError("The telemetry CSV writer failed.") from self.error
        self.rows.put([time.time(), monotonic_s, *(getattr(sample, name) for name in TELEMETRY_FIELDS)])

    def _write_row(self, row: list[Any]) -> None:
        if self.file is None or self.bytes_written >= self.max_bytes:
            self._close_file()
            self._open()
        assert self.writer is not None
        self.bytes_written += self.writer.writerow(row)

    def _run(self) -> None:
        try:
            while True:
                row = self.rows.get()
                if row is None:
                    break
                started = time.monotonic()
                self._write_row(row)
                write_s = time.monotonic() - started
                if write_s > TELEMETRY_GAP_S:
                    DIAGNOSTIC_LOGGER.warning(
                        "raw CSV write stalled for %.3f s (%d rows queued)",
                        write_s,
                        self.rows.qsize(),
                        extra={
                            "event_data": {
                                "event": "csv_write_stall",
                                "duration_s": write_s,
                                "queued_rows": self.rows.qsize(),
                            }
                        },
                    )
        except BaseException as exc:  # Preserve the error for the receiver/main thread.
            self.error = exc
            DIAGNOSTIC_LOGGER.exception(
                "raw telemetry CSV writer failed",
                extra={"event_data": {"event": "csv_writer_failure"}},
            )
        finally:
            self._close_file()

    def _close_file(self) -> None:
        if self.file is not None:
            self.file.close()
        self.file, self.writer, self.bytes_written = None, None, 0

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        self.rows.put(None)
        self.worker.join()
        if self.error is not None:
            raise RuntimeError("The telemetry CSV writer failed.") from self.error


class TelemetryState:
    def __init__(self, pi_host: str = "No Pi configured", logger: CsvLogger | None = None, *, source_mode: str = "live", duration_s: float | None = None) -> None:
        self.pi_host = pi_host
        self.logger = logger
        self.source_mode = source_mode
        self.duration_s = duration_s
        self.sample_cadence_s: float | None = None
        self.connection_state = "unconfigured" if not pi_host or pi_host == "No Pi configured" else "configured"
        self.connection_message = "Choose a Pi target to begin receiving telemetry."
        self.display_run = 0
        self.run_active = False
        self.pi_ready = False
        self.lock = threading.Lock()
        self.sequence = 0
        self.latest: dict[str, Any] | None = None
        self.last_packet_at: float | None = None
        self.malformed_packets = 0
        self.packet_times: deque[float] = deque()
        self.latched_flags = {"controller": 0, "saturation": 0, "actuator": 0}
        self.telemetry_gap_count = 0
        self.last_telemetry_gap: dict[str, Any] | None = None

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
        gap_event: dict[str, Any] | None = None
        with self.lock:
            previous = self.latest
            receive_gap = None if self.last_packet_at is None else now - self.last_packet_at
            controller_gap = None if previous is None else sample.t_sec - float(previous["t_sec"])
            if (receive_gap is not None and receive_gap > TELEMETRY_GAP_S) or (
                controller_gap is not None and controller_gap > TELEMETRY_GAP_S
            ):
                if receive_gap is not None and receive_gap > TELEMETRY_GAP_S and (
                    controller_gap is None or controller_gap <= TELEMETRY_GAP_S
                ):
                    event_type = "udp_receive_pause"
                elif controller_gap is not None and controller_gap > TELEMETRY_GAP_S and (
                    receive_gap is None or receive_gap <= TELEMETRY_GAP_S
                ):
                    event_type = "telemetry_packet_gap"
                else:
                    event_type = "telemetry_gap"
                gap_event = {
                    "event": event_type,
                    "receive_gap_s": receive_gap,
                    "controller_gap_s": controller_gap,
                    "previous_t_sec": None if previous is None else previous["t_sec"],
                    "current_t_sec": sample.t_sec,
                    "next_sequence": self.sequence + 1,
                }
                self.telemetry_gap_count += 1
                self.last_telemetry_gap = gap_event
            self.sequence += 1
            self.latest = telemetry_view(sample, self.sequence, now)
            self.last_packet_at = now
            self.packet_times.append(now)
            while self.packet_times and self.packet_times[0] < now - 1.0:
                self.packet_times.popleft()
            for key, value in self.latest["flags"].items():
                self.latched_flags[key] |= value
        if gap_event is not None:
            DIAGNOSTIC_LOGGER.warning(
                "telemetry gap: receive %.3f s, controller %.3f s",
                float(gap_event["receive_gap_s"] or 0.0),
                float(gap_event["controller_gap_s"] or 0.0),
                extra={"event_data": gap_event},
            )
        return True

    def clear_latched_flags(self) -> None:
        with self.lock:
            self.latched_flags = {"controller": 0, "saturation": 0, "actuator": 0}

    def accept_view(self, telemetry: dict[str, Any], received_at: float) -> None:
        """Publish a normalized playback sample without writing a new CSV log."""
        with self.lock:
            self.sequence += 1
            telemetry["sequence"] = self.sequence
            telemetry["received_at"] = received_at
            self.latest = telemetry
            self.last_packet_at = time.monotonic()
            self.packet_times.append(self.last_packet_at)
            while self.packet_times and self.packet_times[0] < self.last_packet_at - 1.0:
                self.packet_times.popleft()
            for key, value in telemetry["flags"].items():
                self.latched_flags[key] |= value

    def snapshot(self, display_rate_hz: float) -> tuple[dict[str, Any] | None, dict[str, Any]]:
        now = time.monotonic()
        with self.lock:
            while self.packet_times and self.packet_times[0] < now - 1.0:
                self.packet_times.popleft()
            age = None if self.last_packet_at is None else max(0.0, now - self.last_packet_at)
            status = {
                "pi": self.pi_host,
                "connected": age is not None and age <= DISCONNECT_AFTER_S,
                "last_packet_age_ms": None if age is None else age * 1000.0,
                "raw_packet_rate_hz": len(self.packet_times),
                "malformed_packets": self.malformed_packets,
                "display_rate_hz": display_rate_hz,
                "latched_flags": dict(self.latched_flags),
                "connection_state": "streaming" if age is not None and age <= DISCONNECT_AFTER_S else self.connection_state,
                "connection_message": self.connection_message,
                "display_run": self.display_run,
                "run_active": self.run_active,
                "telemetry_connected": age is not None and age <= DISCONNECT_AFTER_S,
                "pi_ready": self.pi_ready,
                "imu_age_ms": None if self.latest is None else self.latest["timing"]["imu_age_ms"],
                "feedback_age_ms": None if self.latest is None else self.latest["timing"]["feedback_age_ms"],
                "telemetry_gap_count": self.telemetry_gap_count,
                "last_telemetry_gap": self.last_telemetry_gap,
            }
            return self.latest, status

    def source_info(self) -> dict[str, Any]:
        with self.lock:
            return {
                "mode": self.source_mode,
                "name": self.pi_host,
                "duration_s": self.duration_s,
                "sample_cadence_s": self.sample_cadence_s,
                "configured_pi": None if self.pi_host == "No Pi configured" else self.pi_host,
                "connection_state": self.connection_state,
                "connection_message": self.connection_message,
                "run_limit_s": DISPLAY_HISTORY_SECONDS,
                "display_sample_hz": DISPLAY_HZ,
                "display_run": self.display_run,
            }

    def configure_pi(self, target: str) -> None:
        with self.lock:
            self.pi_host = target
            self.source_mode = "live"
            self.duration_s = None
            self.sample_cadence_s = None
            self.connection_state = "configured"
            self.connection_message = "Waiting to resolve and register with the Pi."

    def set_csv_source(self, name: str, duration_s: float, cadence_s: float | None) -> None:
        with self.lock:
            self.pi_host = name
            self.source_mode = "csv"
            self.duration_s = duration_s
            self.sample_cadence_s = cadence_s
            self.connection_state = "offline"
            self.connection_message = "CSV replay is local to this dashboard session."

    def reset_display_run(self) -> int:
        """Mark a new browser display run without affecting raw capture."""
        with self.lock:
            self.display_run += 1
            return self.display_run

    def set_run_active(self, active: bool) -> None:
        with self.lock:
            self.run_active = active

    def set_pi_ready(self, ready: bool) -> None:
        with self.lock:
            self.pi_ready = ready


class SseHub:
    """A coalescing fan-out with bounded display history for browser clients."""
    def __init__(self, state: TelemetryState) -> None:
        self.state = state
        self.condition = threading.Condition()
        self.version = 0
        self.telemetry: dict[str, Any] | None = None
        self.status: dict[str, Any] = {}
        self._display_times: deque[float] = deque()
        self._history: deque[dict[str, Any]] = deque(maxlen=DISPLAY_HISTORY_POINTS)
        self._history_sequence = 0
        self._static_history: list[dict[str, Any]] | None = None
        self._stopping = False

    def run(self) -> None:
        interval = 1.0 / DISPLAY_HZ
        while not self._stopping:
            started = time.monotonic()
            self._display_times.append(started)
            while self._display_times and self._display_times[0] < started - 1.0:
                self._display_times.popleft()
            telemetry, status = self.state.snapshot(DISPLAY_HZ)
            with self.condition:
                self.status = status
                if self._static_history is not None:
                    self.telemetry = None
                elif telemetry is not None and telemetry["sequence"] != self._history_sequence:
                    self.telemetry = telemetry
                    self._append_history(telemetry)
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

    def history(self, seconds: float = DISPLAY_HISTORY_SECONDS, max_points: int = DISPLAY_HISTORY_POINTS, end: float | None = None) -> list[dict[str, Any]]:
        """Return a stable snapshot of the display-rate history, oldest first."""
        return self.history_window(seconds, max_points, end)[0]

    def history_window(
        self,
        seconds: float = DISPLAY_HISTORY_SECONDS,
        max_points: int = DISPLAY_HISTORY_POINTS,
        end: float | None = None,
    ) -> tuple[list[dict[str, Any]], bool]:
        """Return a bounded history window and whether it was decimated."""
        with self.condition:
            source = self._static_history if self._static_history is not None else list(self._history)
            if not source:
                return [], False
            latest = source[-1]["received_at"] if end is None else min(end, source[-1]["received_at"])
            cutoff = latest - max(0.0, seconds)
            samples = [sample for sample in source if cutoff <= sample["received_at"] <= latest]
            if len(samples) <= max_points:
                return samples, False
            stride = max(1, (len(samples) + max_points - 1) // max_points)
            reduced = samples[::stride]
            if reduced[-1] is not samples[-1]:
                if len(reduced) >= max_points:
                    reduced[-1] = samples[-1]
                else:
                    reduced.append(samples[-1])
            return reduced, True

    def timeline_bounds(self) -> tuple[float | None, float | None]:
        with self.condition:
            source = self._static_history if self._static_history is not None else self._history
            if not source:
                return None, None
            return source[0]["received_at"], source[-1]["received_at"]

    def set_static_history(self, samples: list[dict[str, Any]] | None) -> None:
        """Install a normalized CSV source without retaining an uploaded file."""
        with self.condition:
            self._static_history = samples
            self._history.clear()
            self._history_sequence = 0
            self.version += 1
            self.condition.notify_all()

    def clear_history(self) -> None:
        self.set_static_history(None)

    def begin_display_run(self) -> None:
        """Clear live presentation history without interrupting raw logging."""
        with self.state.lock:
            sequence = self.state.sequence
        with self.condition:
            if self._static_history is None:
                self._history.clear()
                # Skip the last pre-start packet; the next accepted packet is t=0.
                self._history_sequence = sequence
                self.telemetry = None
            self.version += 1
            self.condition.notify_all()

    def record_playback(self, telemetry: dict[str, Any]) -> None:
        """Retain replay samples even when a CSV is intentionally played at full speed."""
        with self.condition:
            self._append_history(telemetry)

    def _append_history(self, telemetry: dict[str, Any]) -> None:
        if telemetry["sequence"] == self._history_sequence:
            return
        if self._history and telemetry["received_at"] < self._history[-1]["received_at"]:
            self._history.clear()
        self._history.append(telemetry)
        self._history_sequence = telemetry["sequence"]
        cutoff = telemetry["received_at"] - DISPLAY_HISTORY_SECONDS
        while self._history and self._history[0]["received_at"] < cutoff:
            self._history.popleft()


def _number(row: dict[str, Any], *names: str) -> float:
    for name in names:
        try:
            value = float(row.get(name, ""))
        except (TypeError, ValueError):
            continue
        if value == value and value not in (float("inf"), float("-inf")):
            return value
    return 0.0


def load_playback_csv(path: Path) -> list[tuple[float, dict[str, Any]]]:
    """Normalize dashboard and simulator CSV rows into browser telemetry views."""
    try:
        from tools.telemetry_analysis.frames import read_telemetry_csv
    except ModuleNotFoundError as exc:
        if exc.name == "pandas":
            raise RuntimeError(
                "CSV playback requires pandas; install requirements-dev.txt to use --csv or upload a CSV."
            ) from exc
        raise
    rows = read_telemetry_csv(path).to_dict(orient="records")
    samples: list[tuple[float, dict[str, Any]]] = []
    time_keys = ("t_sec", "received_at_monotonic_s", "received_at_unix_s")
    for row in rows:
        source_time = _number(row, *time_keys)
        view = {
            "t_sec": _number(row, "t_sec"),
            "attitude": {
                "pitch_deg": _number(row, "pitch_deg", "plant_pitch_deg"),
                "fused_pitch_deg": _number(row, "fused_pitch_deg", "pitch_deg"),
                "raw_acc_pitch_deg": _number(row, "raw_acc_pitch_deg"),
                "pitch_setpoint_deg": _number(row, "pitch_sp_deg"),
            },
            "rate": {
                "pitch_rate_dps": _number(row, "pitch_rate_dps", "plant_pitch_rate_dps"),
                "filtered_pitch_rate_dps": _number(row, "filtered_pitch_rate_dps", "pitch_rate_dps"),
                "gyro_pitch_rate_dps": _number(row, "gyro_pitch_rate_dps"),
                "rate_setpoint_dps": _number(row, "rate_setpoint_dps"),
            },
            "motion": {
                "target_velocity_sps": _number(row, "target_velocity_sps"),
                "measured_velocity_sps": _number(row, "measured_vel_sps"),
                "left_target_sps": _number(row, "left_target_sps"),
                "right_target_sps": _number(row, "right_target_sps"),
                "left_applied_sps": _number(row, "left_applied_sps"),
                "right_applied_sps": _number(row, "right_applied_sps"),
            },
            "controller": {
                "command_sps": _number(row, "u_sps"),
                "velocity_error": _number(row, "vel_error"),
                "pitch_error_deg": _number(row, "pitch_error_deg"),
                "velocity_p_term_deg": _number(row, "velocity_p_term_deg"),
                "velocity_i_term_deg": _number(row, "velocity_i_term_deg"),
            },
            "timing": {"imu_age_ms": _number(row, "age_ms"), "feedback_age_ms": _number(row, "motor_feedback_age_ms")},
            "flags": {
                "controller": int(_number(row, "controller_fault_flags")),
                "saturation": int(_number(row, "controller_saturation_flags")),
                "actuator": int(_number(row, "actuator_fault")),
            },
        }
        samples.append((source_time, view))
    return sorted(samples, key=lambda item: item[0])


class CsvPlayback:
    """Replay normalized CSV data at its recorded cadence without a Pi connection."""
    def __init__(
        self,
        state: TelemetryState,
        path: Path,
        speed: float,
        loop: bool,
        history_sink: Callable[[dict[str, Any]], None] | None = None,
    ) -> None:
        self.state, self.samples, self.speed, self.loop = state, load_playback_csv(path), speed, loop
        self.history_sink = history_sink
        self.stopping = threading.Event()

    def run(self) -> None:
        if not self.samples:
            return
        while not self.stopping.is_set():
            base_source, base_wall = self.samples[0][0], time.monotonic()
            last_recorded_time: float | None = None
            for index, (source_time, view) in enumerate(self.samples):
                if self.stopping.is_set():
                    return
                if self.speed > 0.0:
                    deadline = base_wall + max(0.0, source_time - base_source) / self.speed
                    self.stopping.wait(max(0.0, deadline - time.monotonic()))
                self.state.accept_view(view, source_time - base_source)
                if self.history_sink is not None and (
                    last_recorded_time is None
                    or source_time - last_recorded_time >= 1.0 / DISPLAY_HZ
                    or index == len(self.samples) - 1
                ):
                    self.history_sink(view)
                    last_recorded_time = source_time
            if not self.loop:
                return

    def close(self) -> None:
        self.stopping.set()


class UdpReceiver:
    """Receive telemetry while resolving a configured Pi lazily and repeatedly."""
    def __init__(self, state: TelemetryState, pi_host: str | None, pi_port: int) -> None:
        self.state, self.pi_port = state, pi_port
        self.target = pi_host
        self.pi_address: tuple[str, int] | None = None
        self.lock = threading.Lock()
        self.stopping = threading.Event()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("0.0.0.0", 0))
        self.socket.settimeout(0.2)

    def run(self) -> None:
        next_registration = next_resolution = 0.0
        while not self.stopping.is_set():
            now = time.monotonic()
            with self.lock:
                target = self.target
            if target and (self.pi_address is None or now >= next_resolution):
                try:
                    self.pi_address = (resolve_udp_host(target), self.pi_port)
                    with self.state.lock:
                        self.state.connection_state = "registering"
                        self.state.connection_message = f"Resolved {self.pi_address[0]}; waiting for telemetry."
                except RuntimeError as exc:
                    self.pi_address = None
                    with self.state.lock:
                        self.state.connection_state = "resolving"
                        self.state.connection_message = str(exc)
                next_resolution = now + RESOLVE_RETRY_S
            if self.pi_address and now >= next_registration:
                try:
                    self.socket.sendto(b"\x00\x00", self.pi_address)
                except OSError:
                    self.pi_address = None
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

    def configure(self, target: str) -> None:
        with self.lock:
            self.target = target
            self.pi_address = None

    def retry_now(self) -> None:
        with self.lock:
            self.pi_address = None

    def current_target(self) -> str | None:
        with self.lock:
            return self.target


class PiHeartbeat:
    """Probe SSH reachability without authenticating or blocking UDP reception."""
    def __init__(self, state: TelemetryState, receiver: UdpReceiver) -> None:
        self.state, self.receiver = state, receiver
        self.stopping = threading.Event()

    def probe(self) -> bool:
        target = self.receiver.current_target()
        if not target:
            return False
        try:
            address = resolve_udp_host(target)
            with socket.create_connection((address, 22), timeout=1.5):
                return True
        except (RuntimeError, OSError):
            return False

    def run(self) -> None:
        while not self.stopping.is_set():
            self.state.set_pi_ready(self.probe())
            self.stopping.wait(RESOLVE_RETRY_S)

    def close(self) -> None:
        self.stopping.set()


class DeploymentManager:
    """Small host-side SCP deployment helper."""
    def __init__(self, ssh_target: str | None) -> None:
        self.ssh_target: str | None = None
        if ssh_target:
            self.set_target(ssh_target)
        self.lock = threading.Lock()

    def info(self) -> dict[str, Any]:
        return {
            "enabled": bool(self.ssh_target),
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

    def set_target(self, target: str) -> None:
        self.ssh_target = target if "@" in target else f"pi@{target}"

    def _target(self) -> str:
        if not self.ssh_target:
            raise ValueError("Configure a Pi target before running this operation.")
        return self.ssh_target

    def _validate_current_build(self) -> None:
        if not PI_BINARY.is_file():
            raise ValueError(f"Cross-built binary not found: {PI_BINARY}. Run ./build_cmake OFF first.")
        if not PID_CONFIG.is_file():
            raise ValueError(f"PID config not found: {PID_CONFIG}")
        manifest_path = PI_BINARY.with_name(f"{PI_BINARY.name}.manifest.json")
        stale_message = "Cross-built binary is stale for pid.conf; run ./build_cmake OFF first."
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            config_match = re.search(
                r"^\s*config_version\s*=\s*(\d+)\s*(?:#.*)?$",
                PID_CONFIG.read_text(encoding="utf-8"),
                re.MULTILINE,
            )
            if (
                not isinstance(manifest, dict)
                or manifest.get("format_version") != 1
                or not config_match
                or manifest.get("binary_sha256") != hashlib.sha256(PI_BINARY.read_bytes()).hexdigest()
                or manifest.get("pid_config_sha256") != hashlib.sha256(PID_CONFIG.read_bytes()).hexdigest()
                or manifest.get("pid_config_version") != int(config_match.group(1))
            ):
                raise ValueError(stale_message)
        except (OSError, json.JSONDecodeError, ValueError, TypeError):
            raise ValueError(stale_message) from None

    def deploy_current(self) -> dict[str, Any]:
        self._validate_current_build()
        output = self._run(["scp", str(PI_BINARY), str(PID_CONFIG), f"{self._target()}:~/"], 120)
        return {"ok": True, "message": output or "Deployed build-pi/balancer_pi and pid.conf to the Pi."}

    def start(self) -> dict[str, Any]:
        self._validate_current_build()
        command = (
            "cd ~ || exit 1; rm -f ~/balancer_pi.pid; : >~/balancer_pi.log; "
            "nohup sudo -n ./balancer_pi </dev/null >~/balancer_pi.log 2>&1 & "
            "pid=$!; echo \"$pid\" >~/balancer_pi.pid; sleep 1; "
            "if ! kill -0 \"$pid\" 2>/dev/null; then "
            "wait \"$pid\"; status=$?; rm -f ~/balancer_pi.pid; "
            "echo \"Start failed (exit $status):\"; tail -n 40 ~/balancer_pi.log; exit 1; fi"
        )
        self._run(["ssh", self._target(), command], 15)
        return {"ok": True, "message": "Balancer started successfully. Live output is in ~/balancer_pi.log on the Pi."}

    def abort(self) -> dict[str, Any]:
        command = (
            "status=0; "
            "sudo -n pkill -TERM -x balancer_pi || status=$?; "
            "rm -f ~/balancer_pi.pid; "
            "if test \"$status\" -gt 1; then exit \"$status\"; fi"
        )
        self._run(["ssh", self._target(), command], 15)
        return {"ok": True, "message": "Requested shutdown of all balancer_pi processes on the Pi."}


class SourceController:
    """Own the selectable dashboard source and the lifetime of temporary uploads."""
    def __init__(self, state: TelemetryState, hub: SseHub, receiver: UdpReceiver, pi_port: int, deployer: DeploymentManager | None = None) -> None:
        self.state, self.hub, self.receiver, self.pi_port = state, hub, receiver, pi_port
        self.temp_path: Path | None = None
        self.lock = threading.Lock()
        self.deployer = deployer

    def configure_live(self, target: str) -> dict[str, Any]:
        target = target.strip()
        if not target:
            raise ValueError("Enter a Pi host, SSH alias, or IP address.")
        self._remove_temp()
        self.hub.clear_history()
        self.state.configure_pi(target)
        self.receiver.configure(target)
        if self.deployer:
            self.deployer.set_target(target)
        return {"ok": True, "message": f"Connecting to {target}; retrying every {RESOLVE_RETRY_S:g} seconds."}

    def upload_csv(self, filename: str, data: bytes) -> dict[str, Any]:
        if not filename.lower().endswith(".csv"):
            raise ValueError("Choose a .csv telemetry capture.")
        if not data:
            raise ValueError("The selected CSV is empty.")
        if len(data) > MAX_UPLOAD_BYTES:
            raise ValueError(f"CSV is larger than the {MAX_UPLOAD_BYTES // (1024 * 1024)} MiB session limit.")
        self._remove_temp()
        handle = tempfile.NamedTemporaryFile(prefix="balancer-dashboard-", suffix=".csv", delete=False)
        try:
            handle.write(data)
            handle.close()
            path = Path(handle.name)
            rows = load_playback_csv(path)
            if not rows:
                raise ValueError("The CSV did not contain telemetry rows.")
            base = rows[0][0]
            normalized: list[dict[str, Any]] = []
            for sequence, (source_time, view) in enumerate(rows, start=1):
                view["sequence"] = sequence
                view["received_at"] = source_time - base
                normalized.append(view)
            times = [sample["received_at"] for sample in normalized]
            cadence = (times[-1] - times[0]) / (len(times) - 1) if len(times) > 1 else None
            self.temp_path = path
            self.state.set_csv_source(filename, times[-1] if times else 0.0, cadence)
            self.hub.set_static_history(normalized)
            return {"ok": True, "message": f"Loaded {filename} ({len(normalized):,} samples).", "source": self.state.source_info()}
        except Exception:
            path = Path(handle.name)
            path.unlink(missing_ok=True)
            raise

    def test_connection(self) -> dict[str, Any]:
        source = self.state.source_info()
        target = source["configured_pi"]
        if not target or source["mode"] != "live":
            raise ValueError("Choose a live Pi source before testing the connection.")
        ssh_target = target if "@" in target else f"pi@{target}"
        result = subprocess.run(
            ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=3", ssh_target, "true"],
            capture_output=True, text=True, timeout=5,
        )
        if result.returncode:
            raise RuntimeError((result.stderr or result.stdout).strip() or "SSH connection test failed.")
        return {"ok": True, "message": f"SSH connection to {ssh_target} succeeded."}

    def _remove_temp(self) -> None:
        with self.lock:
            if self.temp_path:
                self.temp_path.unlink(missing_ok=True)
                self.temp_path = None

    def close(self) -> None:
        self._remove_temp()


def make_handler(hub: SseHub, state: TelemetryState, deployer: DeploymentManager | None = None, sources: SourceController | None = None):
    class DashboardHandler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, _format: str, *_args: Any) -> None:
            return

        def handle(self) -> None:
            try:
                super().handle()
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                # Windows can raise WinError 10053 while BaseHTTPRequestHandler
                # is waiting for a browser that just replaced an SSE connection.
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
            if self.path.startswith("/api/history"):
                query = parse_qs(urlsplit(self.path).query)
                source = state.source_info()
                maximum_seconds = (
                    max(DISPLAY_HISTORY_SECONDS, float(source["duration_s"] or 0.0))
                    if source["mode"] == "csv"
                    else DISPLAY_HISTORY_SECONDS
                )
                try:
                    seconds = float(query.get("seconds", [DISPLAY_HISTORY_SECONDS])[0])
                except ValueError:
                    seconds = DISPLAY_HISTORY_SECONDS
                seconds = min(maximum_seconds, max(1.0, seconds))
                try:
                    max_points = int(query.get("max_points", [DISPLAY_HISTORY_POINTS])[0])
                except ValueError:
                    max_points = DISPLAY_HISTORY_POINTS
                max_points = min(DISPLAY_HISTORY_POINTS, max(10, max_points))
                try:
                    end = float(query.get("end", [""])[0])
                except ValueError:
                    end = None
                samples, decimated = hub.history_window(seconds, max_points, end)
                earliest, latest = hub.timeline_bounds()
                requested_end = latest if end is None else end
                requested_start = None if requested_end is None else requested_end - seconds
                self._json(
                    HTTPStatus.OK,
                    {
                        "history_seconds": seconds,
                        "earliest": earliest,
                        "latest": latest,
                        "requested_start": requested_start,
                        "requested_end": requested_end,
                        "window_start": samples[0]["received_at"] if samples else None,
                        "window_end": samples[-1]["received_at"] if samples else None,
                        "cached_start": samples[0]["received_at"] if samples else None,
                        "cached_end": samples[-1]["received_at"] if samples else None,
                        "decimated": decimated,
                        "run_limit_s": DISPLAY_HISTORY_SECONDS,
                        "display_sample_hz": DISPLAY_HZ,
                        "display_run": source["display_run"],
                        "samples": samples,
                    },
                )
                return
            if self.path == "/api/source":
                self._json(HTTPStatus.OK, state.source_info())
                return
            if self.path == "/api/deploy-info":
                self._json(HTTPStatus.OK, deployer.info() if deployer else {"enabled": False})
                return
            target = "index.html" if self.path in ("/", "/index.html") else self.path.lstrip("/")
            path = (STATIC_DIR / target).resolve()
            if STATIC_DIR not in path.parents or not path.is_file():
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            content_types = {
                ".html": "text/html; charset=utf-8",
                ".js": "text/javascript; charset=utf-8",
                ".css": "text/css; charset=utf-8",
                ".json": "application/json; charset=utf-8",
                ".svg": "image/svg+xml",
                ".LICENSE": "text/plain; charset=utf-8",
            }
            content_type = content_types.get(path.suffix, "application/octet-stream")
            data = path.read_bytes()
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_POST(self) -> None:  # noqa: N802
            if self.path == "/api/source/csv":
                if sources is None:
                    self.send_error(HTTPStatus.NOT_IMPLEMENTED)
                    return
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    if length <= 0 or length > MAX_UPLOAD_BYTES:
                        raise ValueError(f"CSV upload must be between 1 byte and {MAX_UPLOAD_BYTES // (1024 * 1024)} MiB.")
                    filename = self.headers.get("X-Filename", "capture.csv")
                    self._json(HTTPStatus.OK, sources.upload_csv(filename, self.rfile.read(length)))
                except (KeyError, ValueError, RuntimeError, OSError) as exc:
                    self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)})
                return
            if self.path == "/api/connect":
                if sources is None:
                    self.send_error(HTTPStatus.NOT_IMPLEMENTED)
                    return
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    body = json.loads(self.rfile.read(length) or b"{}")
                    self._json(HTTPStatus.OK, sources.configure_live(str(body.get("target", ""))))
                except (ValueError, RuntimeError, json.JSONDecodeError) as exc:
                    self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)})
                return
            if self.path == "/api/ping":
                if sources is None:
                    self.send_error(HTTPStatus.NOT_IMPLEMENTED)
                    return
                try:
                    self._json(HTTPStatus.OK, sources.test_connection())
                except (ValueError, RuntimeError, subprocess.TimeoutExpired, FileNotFoundError) as exc:
                    self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)})
                return
            actions = {
                "/api/deploy": deployer.deploy_current if deployer else None,
                "/api/start": deployer.start if deployer else None,
                "/api/abort": deployer.abort if deployer else None,
            }
            action = actions.get(self.path)
            if action is None:
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            try:
                if state.source_info()["mode"] == "csv":
                    result = {"ok": True, "message": "CSV source: no SSH command sent."}
                    if self.path == "/api/start":
                        state.set_run_active(True)
                        result["display_run"] = state.reset_display_run()
                    elif self.path == "/api/abort":
                        state.set_run_active(False)
                    self._json(HTTPStatus.OK, result)
                    return
                result = action()
                if self.path == "/api/start":
                    state.set_run_active(True)
                    result["display_run"] = state.reset_display_run()
                    hub.begin_display_run()
                elif self.path == "/api/abort":
                    state.set_run_active(False)
                self._json(HTTPStatus.OK, result)
            except (KeyError, ValueError, RuntimeError, subprocess.TimeoutExpired, FileNotFoundError) as exc:
                DIAGNOSTIC_LOGGER.error(
                    "dashboard %s failed: %s",
                    self.path,
                    exc,
                    extra={
                        "event_data": {
                            "event": "dashboard_operation_failure",
                            "operation": self.path.removeprefix("/api/"),
                            "error_type": type(exc).__name__,
                            "error": str(exc),
                        }
                    },
                )
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
            last_telemetry_sequence = -1
            last_status_at = 0.0
            last_status_key: tuple[Any, ...] | None = None
            try:
                while True:
                    version, telemetry, status = hub.wait_for_update(version)
                    if telemetry is not None and telemetry.get("sequence") != last_telemetry_sequence:
                        self.wfile.write(b"event: telemetry\ndata: " + json.dumps(telemetry, separators=(",", ":")).encode() + b"\n\n")
                        last_telemetry_sequence = int(telemetry["sequence"])
                    now = time.monotonic()
                    status_key = (
                        status.get("run_active"), status.get("telemetry_connected"), status.get("pi_ready"),
                        status.get("connection_state"), status.get("display_run"), status.get("malformed_packets"),
                        tuple(sorted(status.get("latched_flags", {}).items())),
                    )
                    if status_key != last_status_key or now - last_status_at >= 1.0 / STATUS_HZ:
                        self.wfile.write(b"event: status\ndata: " + json.dumps(status, separators=(",", ":")).encode() + b"\n\n")
                        last_status_key, last_status_at = status_key, now
                    self.wfile.flush()
            # A browser can close or replace its EventSource connection at any
            # time.  Windows reports a host-side abort as ConnectionAbortedError,
            # while other platforms commonly use the two exceptions below.
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass

    return DashboardHandler


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=False)
    source.add_argument(
        "--pi",
        default=DEFAULT_PI_TARGET,
        help="SSH Host alias (such as rpi4), hostname, IP, or user@SSH-alias",
    )
    source.add_argument("--csv", type=Path, help="Replay a dashboard or simulator CSV without a Pi")
    parser.add_argument("--pi-port", type=int, default=9000)
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--playback-speed", type=float, default=1.0, help="CSV playback multiplier; 0 replays as fast as possible")
    parser.add_argument("--playback-loop", action="store_true", help="Loop CSV playback after its final sample")
    parser.add_argument("--deploy-pi", help="Initial Pi target while viewing a CSV")
    parser.add_argument("--listen-lan", action="store_true", help="Listen on all interfaces instead of localhost")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.playback_speed < 0.0:
        raise ValueError("--playback-speed must be zero or positive")
    # Offline captures stay self-contained even though live sessions default to rpi4.
    initial_target = None if args.csv else (args.pi or args.deploy_pi)
    diagnostic_listener = configure_diagnostic_logging(SERVER_LOG_DIR)
    logger = CsvLogger(SERVER_LOG_DIR)
    state = TelemetryState(initial_target or "No Pi configured", logger)
    hub = SseHub(state)
    receiver = UdpReceiver(state, initial_target, args.pi_port)
    heartbeat = PiHeartbeat(state, receiver)
    deployer = DeploymentManager(initial_target)
    sources = SourceController(state, hub, receiver, args.pi_port, deployer)
    source_description = f"Pi target {initial_target or 'not configured'}"
    if args.csv:
        sources.upload_csv(args.csv.name, args.csv.read_bytes())
        source_description = f"CSV session {args.csv}"
    receiver_thread = threading.Thread(target=receiver.run, name="telemetry-udp", daemon=True)
    heartbeat_thread = threading.Thread(target=heartbeat.run, name="pi-heartbeat", daemon=True)
    hub_thread = threading.Thread(target=hub.run, name="telemetry-sse", daemon=True)
    receiver_thread.start()
    heartbeat_thread.start()
    hub_thread.start()
    host = "0.0.0.0" if args.listen_lan else "127.0.0.1"
    server = ThreadingHTTPServer((host, args.port), make_handler(hub, state, deployer, sources))
    print(f"Dashboard: http://{host}:{args.port} ({source_description})")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        receiver.close()
        heartbeat.close()
        hub.stop()
        receiver_thread.join(timeout=2.0)
        heartbeat_thread.join(timeout=2.0)
        hub_thread.join(timeout=2.0)
        sources.close()
        try:
            logger.close()
        finally:
            diagnostic_listener.stop()
            server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
