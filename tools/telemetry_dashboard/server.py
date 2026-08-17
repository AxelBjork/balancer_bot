#!/usr/bin/env python3
"""Serve the balancer telemetry dashboard and its run-gated runtime controls."""
from __future__ import annotations

import argparse
import csv
import dataclasses
import json
import logging
import logging.handlers
import math
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

from generated_balancer import (  # noqa: E402
    ConfigPidValuesPayload,
    JoystickCommandPayload,
    PidConfigOverridePayload,
    PidConfigStatusPayload,
    SystemTelemetryPayload,
)

SYSTEM_TELEMETRY_ID = 3003
SYSTEM_TELEMETRY_SIZE = SystemTelemetryPayload.WIRE_SIZE
EXTERNAL_JOYSTICK_COMMAND_ID = 3011
PID_CONFIG_OVERRIDE_ID = 3012
PID_CONFIG_STATUS_ID = 3013
PID_CONFIG_FIELDS = (
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
PID_CONFIG_NONNEGATIVE_FIELDS = frozenset(
    {
        "drive_max_velocity_mps",
        "velocity_gain_per_s",
        "velocity_feedback_cutoff_hz",
        "outer_pitch_limit_deg",
        "adaptive_com_trim_enabled",
        "adaptive_com_trim_gain_deg_per_mps_s",
        "adaptive_com_trim_limit_deg",
        "turn_max_sps",
        "pitch_gain",
        "pitch_rate_gain",
        "pitch_accel_gain",
        "planner_max_acceleration_mps2",
        "planner_max_deceleration_mps2",
        "planner_max_jerk_mps3",
        "velocity_i_gain_per_s2",
        "velocity_i_leak_time_s",
        "velocity_i_acceleration_limit_mps2",
    }
)
PID_CONFIG_POSITIVE_FIELDS = frozenset(
    {
        "drive_max_velocity_mps",
        "balance_max_sps",
        "velocity_feedback_cutoff_hz",
        "outer_pitch_limit_deg",
        "planner_max_acceleration_mps2",
        "planner_max_deceleration_mps2",
        "planner_max_jerk_mps3",
        "velocity_i_leak_time_s",
        "velocity_i_acceleration_limit_mps2",
    }
)
PID_CONFIG_LIMIT_FIELDS = frozenset({"turn_max_sps", "balance_max_sps"})
# Keep the dashboard validation aligned with Config::max_step_rate_sps.  The
# C++ configuration is authoritative for the runtime; this mirror prevents a
# dashboard request from being rejected before it reaches that validator.
MAX_STEP_RATE_SPS = 16000.0
MAX_MOTION_PITCH_SETPOINT_DEG = 45.0
METERS_PER_STEP = 2.0 * math.pi * 0.0412 / 6400.0
DISPLAY_HZ = 50.0
DISPLAY_HISTORY_SECONDS = 120.0
DISPLAY_HISTORY_POINTS = 6000
UDP_RECEIVE_BUFFER_BYTES = 4 * 1024 * 1024
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


def _validate_pid_values(values: dict[str, Any], *, require_complete: bool = True) -> dict[str, float]:
    if not isinstance(values, dict):
        raise ValueError("PID values must be a JSON object.")
    unknown = sorted(set(values) - set(PID_CONFIG_FIELDS))
    if unknown:
        raise ValueError(f"Unknown PID fields: {', '.join(unknown)}")
    if require_complete and set(values) != set(PID_CONFIG_FIELDS):
        missing = sorted(set(PID_CONFIG_FIELDS) - set(values))
        raise ValueError(f"Missing PID fields: {', '.join(missing)}")

    normalized: dict[str, float] = {}
    for name, value in values.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"PID field {name} must be numeric.")
        value = float(value)
        if not math.isfinite(value):
            raise ValueError(f"PID field {name} must be finite.")
        normalized[name] = value

    for name in PID_CONFIG_NONNEGATIVE_FIELDS:
        if name in normalized and normalized[name] < 0.0:
            raise ValueError(f"PID field {name} must be non-negative.")
    for name in PID_CONFIG_POSITIVE_FIELDS:
        if name in normalized and normalized[name] <= 0.0:
            raise ValueError(f"PID field {name} must be positive.")
    for name in PID_CONFIG_LIMIT_FIELDS:
        if name in normalized and normalized[name] > MAX_STEP_RATE_SPS:
            raise ValueError(
                f"PID field {name} exceeds the supported {MAX_STEP_RATE_SPS:g} limit."
            )
    for name in (
        "pitch_gain",
        "pitch_rate_gain",
        "pitch_accel_gain",
    ):
        if name in normalized and normalized[name] > 1.0e6:
            raise ValueError(f"PID field {name} exceeds the supported range.")
    if normalized.get("outer_pitch_limit_deg", 0.0) > MAX_MOTION_PITCH_SETPOINT_DEG:
        raise ValueError(
            "PID field outer_pitch_limit_deg exceeds the supported "
            f"{MAX_MOTION_PITCH_SETPOINT_DEG:g} degree range."
        )
    if abs(normalized.get("fixed_com_trim_deg", 0.0)) > 45.0:
        raise ValueError("PID field fixed_com_trim_deg exceeds the supported range.")
    if normalized.get("adaptive_com_trim_limit_deg", 0.0) > 45.0:
        raise ValueError("PID field adaptive_com_trim_limit_deg exceeds the supported range.")
    if normalized.get("adaptive_com_trim_enabled", 0.0) not in (0.0, 1.0):
        raise ValueError("PID field adaptive_com_trim_enabled must be 0 or 1.")
    if "turn_max_sps" in normalized and "balance_max_sps" in normalized:
        if normalized["balance_max_sps"] <= normalized["turn_max_sps"]:
            raise ValueError("PID balance_max_sps must exceed turn_max_sps.")
    if "drive_max_velocity_mps" in normalized and "turn_max_sps" in normalized and "balance_max_sps" in normalized:
        user_speed_sps = normalized["drive_max_velocity_mps"] / METERS_PER_STEP
        available_sps = normalized["balance_max_sps"] - normalized["turn_max_sps"]
        if user_speed_sps > 0.25 * available_sps:
            raise ValueError("PID drive_max_velocity_mps exceeds the reserved actuator headroom.")
    return normalized


def _load_pid_values(path: Path) -> dict[str, float]:
    if not path.is_file():
        raise ValueError(f"PID config not found: {path}")
    values: dict[str, float] = {}
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        raise ValueError(f"PID config cannot be read: {exc}") from None
    allowed = set(PID_CONFIG_FIELDS) | {"config_version", "controller_enabled"}
    for line_number, line in enumerate(lines, start=1):
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        if line.count("=") != 1:
            raise ValueError(f"Malformed PID config line {line_number}.")
        key, value_text = (part.strip() for part in line.split("=", 1))
        if key not in allowed:
            raise ValueError(f"Unknown PID config key: {key}")
        if key in values:
            raise ValueError(f"Duplicate PID config key: {key}")
        try:
            value = float(value_text)
        except ValueError:
            raise ValueError(f"Invalid PID config value for {key}.") from None
        if not math.isfinite(value):
            raise ValueError(f"PID config value for {key} must be finite.")
        values[key] = value
    if values.get("config_version") != 12.0:
        raise ValueError(
            f"PID configuration version mismatch: expected 12, got {values.get('config_version', 'missing')}"
        )
    missing = [name for name in PID_CONFIG_FIELDS if name not in values]
    if missing:
        raise ValueError(f"Missing PID config fields: {', '.join(missing)}")
    return _validate_pid_values({name: values[name] for name in PID_CONFIG_FIELDS})


def _pid_status_message(accepted: bool, result_code: int) -> str:
    if accepted:
        return "PID override applied for the current balancer process."
    return {
        1: "PID override rejected: a value was not finite.",
        2: "PID override rejected: a value was negative.",
        3: "PID override rejected: a required limit was not positive.",
        4: "PID override rejected: a supported speed limit was exceeded.",
    }.get(result_code, "PID override rejected by the balancer.")


def telemetry_view(sample: SystemTelemetryPayload, sequence: int, received_at: float) -> dict[str, Any]:
    """Return the stable, browser-facing subset of the reflected wire payload."""
    return {
        "sequence": sequence,
        "received_at": received_at,
        "run_id": int(sample.run_id),
        "packet_seq": int(sample.packet_seq),
        "loop_seq": int(sample.loop_seq),
        "sender_monotonic_ns": int(sample.sender_monotonic_ns),
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
        },
        "motion": {
            "raw_completed_velocity_sps": sample.raw_completed_velocity_sps,
            "corrected_axle_velocity_sps": sample.corrected_axle_velocity_sps,
            "velocity_control_sps": sample.velocity_control_sps,
            "left_target_sps": sample.left_target_sps,
            "right_target_sps": sample.right_target_sps,
            "left_slewed_sps": sample.left_slewed_sps,
            "right_slewed_sps": sample.right_slewed_sps,
            "left_actual_steps": sample.left_actual_steps,
            "right_actual_steps": sample.right_actual_steps,
            "user_velocity_mps": sample.user_velocity_mps,
            "reference_velocity_mps": sample.reference_velocity_mps,
            "reference_acceleration_mps2": sample.reference_acceleration_mps2,
            "reference_jerk_mps3": sample.reference_jerk_mps3,
            "velocity_feedback_estimate_mps": sample.velocity_feedback_estimate_mps,
            "velocity_error_mps": sample.velocity_error_mps,
            "velocity_feedback_valid": bool(sample.velocity_feedback_valid),
            "velocity_feedback_active": bool(sample.velocity_feedback_active),
        },
        "controller": {
            "command_sps": sample.u_sps,
            "reference_acceleration_mps2": sample.reference_acceleration_mps2,
            "velocity_feedback_acceleration_mps2": sample.velocity_feedback_acceleration_mps2,
            "velocity_p_acceleration_mps2": sample.velocity_p_acceleration_mps2,
            "velocity_i_acceleration_mps2": sample.velocity_i_acceleration_mps2,
            "velocity_integral_state_mps_s": sample.velocity_integral_state_mps_s,
            "acceleration_raw_mps2": sample.acceleration_raw_mps2,
            "acceleration_cmd_mps2": sample.acceleration_cmd_mps2,
            "drive_pitch_target_deg": sample.drive_pitch_target_deg,
            "final_pitch_target_deg": sample.final_pitch_target_deg,
            "pitch_error_deg": sample.pitch_error_deg,
            "pitch_feedback_sps": sample.pitch_feedback_sps,
            "pitch_rate_feedback_sps": sample.pitch_rate_feedback_sps,
            "pitch_accel_feedback_sps": sample.pitch_accel_feedback_sps,
            "balance_unclamped_sps": sample.balance_unclamped_sps,
            "outer_acceleration_limited": bool(sample.outer_acceleration_limited),
            "outer_pitch_target_limited": bool(sample.outer_pitch_target_limited),
            "planner_acceleration_limited": bool(sample.planner_acceleration_limited),
            "planner_jerk_limited": bool(sample.planner_jerk_limited),
            "velocity_integral_limited": bool(sample.velocity_integral_limited),
            "velocity_anti_windup_active": bool(sample.velocity_anti_windup_active),
            "pitch_target_unclamped_deg": sample.pitch_target_unclamped_deg,
            "pitch_target_limit_reason": sample.pitch_target_limit_reason,
            "com_trim_deg": sample.com_trim_deg,
            "fixed_com_trim_deg": sample.fixed_com_trim_deg,
            "trim_learning_enabled": bool(sample.trim_learning_enabled),
            "trim_learning_block_reason": sample.trim_learning_block_reason,
            "trim_trusted": bool(sample.trim_trusted),
            "trim_learning_allowed": bool(sample.trim_learning_allowed),
            "trim_quiet_rate_rms_dps": sample.trim_quiet_rate_rms_dps,
            "active_pitch_gain_sps_per_rad": sample.active_pitch_gain_sps_per_rad,
            "active_pitch_rate_gain_sps_per_rad_s": sample.active_pitch_rate_gain_sps_per_rad_s,
            "active_pitch_accel_gain_sps_per_rad_s2": sample.active_pitch_accel_gain_sps_per_rad_s2,
            "active_drive_max_velocity_mps": sample.active_drive_max_velocity_mps,
            "active_drive_max_acceleration_mps2": sample.active_drive_max_acceleration_mps2,
            "active_drive_max_deceleration_mps2": sample.active_drive_max_deceleration_mps2,
            "active_planner_max_acceleration_mps2": sample.active_planner_max_acceleration_mps2,
            "active_planner_max_deceleration_mps2": sample.active_planner_max_deceleration_mps2,
            "active_planner_max_jerk_mps3": sample.active_planner_max_jerk_mps3,
            "active_velocity_gain_per_s": sample.active_velocity_gain_per_s,
            "active_velocity_i_gain_per_s2": sample.active_velocity_i_gain_per_s2,
            "active_velocity_i_leak_time_s": sample.active_velocity_i_leak_time_s,
            "active_velocity_i_acceleration_limit_mps2":
                sample.active_velocity_i_acceleration_limit_mps2,
            "active_velocity_feedback_cutoff_hz": sample.active_velocity_feedback_cutoff_hz,
            "active_outer_pitch_limit_deg": sample.active_outer_pitch_limit_deg,
            "active_fixed_com_trim_deg": sample.active_fixed_com_trim_deg,
            "adaptive_com_trim_enabled": bool(sample.adaptive_com_trim_enabled),
            "legacy_outer_fields_valid": bool(sample.legacy_outer_fields_valid),
            "active_accel_lpf_hz": sample.active_accel_lpf_hz,
            "active_gyro_lpf_hz": sample.active_gyro_lpf_hz,
            "active_gyro_derivative_lpf_hz": sample.active_gyro_derivative_lpf_hz,
            "active_config_generation": sample.active_config_generation,
        },
        "timing": {"imu_age_ms": sample.age_ms, "feedback_age_ms": sample.motor_feedback_age_ms},
        "flags": {
            "controller": sample.controller_fault_flags,
            "saturation": sample.controller_saturation_flags,
            "actuator_saturation": sample.actuator_saturation_flags,
            "actuator": int(sample.actuator_fault),
        },
    }


@dataclasses.dataclass
class _CsvRollRequest:
    completed: threading.Event = dataclasses.field(default_factory=threading.Event)
    error: BaseException | None = None


_CsvRow = tuple[float, float, SystemTelemetryPayload]


class CsvLogger:
    """Append every valid raw packet to fixed-schema server CSV files."""
    def __init__(self, directory: Path, max_bytes: int = LOG_ROTATE_BYTES, retain_count: int = LOG_RETAIN_COUNT) -> None:
        self.directory, self.max_bytes, self.retain_count = directory, max_bytes, retain_count
        self.file: Any | None = None
        self.writer: csv.writer | None = None
        self.index = 0
        self.bytes_written = 0
        self.rows: queue.SimpleQueue[_CsvRow | _CsvRollRequest | None] = queue.SimpleQueue()
        self.closed = False
        self.error: BaseException | None = None
        self.write_stall_count = 0
        self.write_duration_max_s = 0.0
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
        self.rows.put((time.time(), monotonic_s, sample))

    def queue_depth(self) -> int:
        """Return the approximate number of queued rows and control requests."""
        return self.rows.qsize()

    def roll(self) -> None:
        """Finish the current capture before the next accepted telemetry row."""
        if self.closed:
            raise RuntimeError("Cannot roll a closed telemetry CSV logger.")
        if self.error is not None:
            raise RuntimeError("The telemetry CSV writer failed.") from self.error
        request = _CsvRollRequest()
        self.rows.put(request)
        while not request.completed.wait(0.1):
            if self.error is not None:
                raise RuntimeError("The telemetry CSV writer failed.") from self.error
        if request.error is not None:
            raise RuntimeError("The telemetry CSV rollover failed.") from request.error
        if self.error is not None:
            raise RuntimeError("The telemetry CSV writer failed.") from self.error

    def _write_row(self, row: list[Any]) -> None:
        if self.file is None or self.bytes_written >= self.max_bytes:
            self._close_file()
            self._open()
        assert self.writer is not None
        self.bytes_written += self.writer.writerow(row)

    def _run(self) -> None:
        try:
            while True:
                item = self.rows.get()
                if item is None:
                    break
                if isinstance(item, _CsvRollRequest):
                    try:
                        self._close_file()
                    except BaseException as exc:
                        item.error = exc
                        raise
                    finally:
                        item.completed.set()
                    continue
                started = time.monotonic()
                wall_time, monotonic_s, sample = item
                row = [wall_time, monotonic_s, *(getattr(sample, name) for name in TELEMETRY_FIELDS)]
                self._write_row(row)
                write_s = time.monotonic() - started
                self.write_duration_max_s = max(self.write_duration_max_s, write_s)
                if write_s > TELEMETRY_GAP_S:
                    self.write_stall_count += 1
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
        self.pid_status: dict[str, Any] | None = None
        self.udp_receive_buffer_bytes: int | None = None
        self.lock = threading.Lock()
        self.sequence = 0
        self.latest: dict[str, Any] | None = None
        self.last_packet_at: float | None = None
        self.malformed_packets = 0
        self.packet_times: deque[float] = deque()
        self.latched_flags = {
            "controller": 0,
            "saturation": 0,
            "actuator_saturation": 0,
            "actuator": 0,
        }
        self.telemetry_gap_count = 0
        self.last_telemetry_gap: dict[str, Any] | None = None
        self.sender_reset_count = 0
        self.packet_sequence_gap_count = 0
        self.loop_sequence_gap_count = 0
        self.last_sender_time_gap_s: float | None = None
        self.last_receiver_gap_s: float | None = None
        self.last_gap_classification: str | None = None
        self.accept_count = 0
        self.accept_duration_total_s = 0.0
        self.accept_duration_max_s = 0.0
        self.last_accept_stall_log = 0.0

    def accept(self, datagram: bytes, received_at: float | None = None) -> bool:
        started = time.monotonic()
        try:
            return self._accept(datagram, received_at)
        finally:
            duration_s = time.monotonic() - started
            with self.lock:
                self.accept_count += 1
                self.accept_duration_total_s += duration_s
                self.accept_duration_max_s = max(self.accept_duration_max_s, duration_s)
            should_log = False
            if duration_s > TELEMETRY_GAP_S:
                with self.lock:
                    now = time.monotonic()
                    if now - self.last_accept_stall_log >= 1.0:
                        self.last_accept_stall_log = now
                        should_log = True
            if should_log:
                DIAGNOSTIC_LOGGER.warning(
                    "dashboard telemetry accept stalled for %.3f s",
                    duration_s,
                    extra={
                        "event_data": {
                            "event": "dashboard_accept_stall",
                            "duration_s": duration_s,
                        }
                    },
                )

    def _accept(self, datagram: bytes, received_at: float | None = None) -> bool:
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
        reset_event: dict[str, Any] | None = None
        with self.lock:
            previous = self.latest
            receive_gap = None if self.last_packet_at is None else now - self.last_packet_at
            controller_gap = None if previous is None else sample.t_sec - float(previous["t_sec"])
            metadata_available = any(
                int(getattr(sample, name)) != 0
                for name in ("run_id", "packet_seq", "loop_seq", "sender_monotonic_ns")
            )
            previous_metadata_available = bool(previous and previous.get("sender_metadata_available"))
            sender_time_gap = None
            packet_seq_delta = None
            loop_seq_delta = None
            packet_gap_count = 0
            loop_gap_count = 0
            sender_reset = False
            if metadata_available and previous_metadata_available:
                sender_time_gap = (
                    sample.sender_monotonic_ns - int(previous["sender_monotonic_ns"])
                ) / 1e9
                packet_seq_delta = int(sample.packet_seq) - int(previous["packet_seq"])
                loop_seq_delta = int(sample.loop_seq) - int(previous["loop_seq"])
                sender_reset = (
                    int(sample.run_id) != int(previous["run_id"])
                    or packet_seq_delta < 0
                    or loop_seq_delta < 0
                    or sender_time_gap < 0.0
                )
            if sender_reset:
                self.sender_reset_count += 1
                reset_event = {
                    "event": "telemetry_run_reset",
                    "previous_run_id": int(previous["run_id"]),
                    "current_run_id": int(sample.run_id),
                    "previous_packet_seq": int(previous["packet_seq"]),
                    "current_packet_seq": int(sample.packet_seq),
                    "previous_loop_seq": int(previous["loop_seq"]),
                    "current_loop_seq": int(sample.loop_seq),
                    "previous_sender_monotonic_ns": int(previous["sender_monotonic_ns"]),
                    "current_sender_monotonic_ns": int(sample.sender_monotonic_ns),
                    "next_sequence": self.sequence + 1,
                }
                sender_time_gap = packet_seq_delta = loop_seq_delta = None
            elif packet_seq_delta is not None:
                packet_gap_count = max(0, packet_seq_delta - 1)
                loop_gap_count = max(0, (loop_seq_delta or 0) - 1)
                self.packet_sequence_gap_count += packet_gap_count
                self.loop_sequence_gap_count += loop_gap_count

            receiver_gap = receive_gap is not None and receive_gap > TELEMETRY_GAP_S
            sender_gap = (
                (sender_time_gap is not None and sender_time_gap > TELEMETRY_GAP_S)
                if metadata_available and previous_metadata_available and not sender_reset
                else controller_gap is not None and controller_gap > TELEMETRY_GAP_S
            )
            sequence_gap = packet_gap_count > 0 or loop_gap_count > 0
            if not sender_reset and (receiver_gap or sender_gap or sequence_gap):
                if receiver_gap and sender_gap:
                    classification = "receiver_and_sender_gap"
                    event_type = "telemetry_gap"
                elif receiver_gap and not sender_gap:
                    classification = "receiver_or_network_pause"
                    event_type = "udp_receive_pause"
                elif sender_gap:
                    classification = "sender_control_dispatch_pause"
                    event_type = "telemetry_packet_gap"
                elif loop_gap_count > packet_gap_count:
                    classification = "loop_without_packet_gap"
                    event_type = "telemetry_packet_gap"
                else:
                    classification = "packet_or_transport_gap"
                    event_type = "telemetry_packet_gap"
                gap_event = {
                    "event": event_type,
                    "classification": classification,
                    "receive_gap_s": receive_gap,
                    "controller_gap_s": controller_gap,
                    "sender_time_gap_s": sender_time_gap,
                    "packet_seq_delta": packet_seq_delta,
                    "loop_seq_delta": loop_seq_delta,
                    "packet_gap_count": packet_gap_count,
                    "loop_gap_count": loop_gap_count,
                    "previous_t_sec": None if previous is None else previous["t_sec"],
                    "current_t_sec": sample.t_sec,
                    "run_id": int(sample.run_id),
                    "packet_seq": int(sample.packet_seq),
                    "loop_seq": int(sample.loop_seq),
                    "sender_monotonic_ns": int(sample.sender_monotonic_ns),
                    "next_sequence": self.sequence + 1,
                }
                self.telemetry_gap_count += 1
                self.last_telemetry_gap = gap_event
                self.last_gap_classification = classification
            self.last_receiver_gap_s = receive_gap
            self.last_sender_time_gap_s = sender_time_gap
            self.sequence += 1
            self.latest = telemetry_view(sample, self.sequence, now)
            self.latest["sender_metadata_available"] = metadata_available
            self.last_packet_at = now
            self.packet_times.append(now)
            while self.packet_times and self.packet_times[0] < now - 1.0:
                self.packet_times.popleft()
            for key, value in self.latest["flags"].items():
                self.latched_flags[key] |= value
        if reset_event is not None:
            DIAGNOSTIC_LOGGER.info(
                "telemetry sender run reset from %s to %s",
                reset_event["previous_run_id"],
                reset_event["current_run_id"],
                extra={"event_data": reset_event},
            )
        if gap_event is not None:
            DIAGNOSTIC_LOGGER.warning(
                "telemetry gap: receive %.3f s, sender %.3f s, class %s",
                float(gap_event["receive_gap_s"] or 0.0),
                float(gap_event["sender_time_gap_s"] or gap_event["controller_gap_s"] or 0.0),
                gap_event["classification"],
                extra={"event_data": gap_event},
            )
        return True

    def clear_latched_flags(self) -> None:
        with self.lock:
            self.latched_flags = {
                "controller": 0,
                "saturation": 0,
                "actuator_saturation": 0,
                "actuator": 0,
            }

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
                "pid_status": None if self.pid_status is None else dict(self.pid_status),
                "imu_age_ms": None if self.latest is None else self.latest["timing"]["imu_age_ms"],
                "feedback_age_ms": None if self.latest is None else self.latest["timing"]["feedback_age_ms"],
                "telemetry_gap_count": self.telemetry_gap_count,
                "last_telemetry_gap": self.last_telemetry_gap,
                "sender_run_id": None if self.latest is None else self.latest.get("run_id"),
                "sender_packet_seq": None if self.latest is None else self.latest.get("packet_seq"),
                "sender_loop_seq": None if self.latest is None else self.latest.get("loop_seq"),
                "sender_monotonic_ns": (
                    None if self.latest is None else self.latest.get("sender_monotonic_ns")
                ),
                "sender_time_gap_ms": (
                    None if self.last_sender_time_gap_s is None else self.last_sender_time_gap_s * 1000.0
                ),
                "receiver_gap_ms": (
                    None if self.last_receiver_gap_s is None else self.last_receiver_gap_s * 1000.0
                ),
                "telemetry_sender_reset_count": self.sender_reset_count,
                "telemetry_packet_sequence_gap_count": self.packet_sequence_gap_count,
                "telemetry_loop_sequence_gap_count": self.loop_sequence_gap_count,
                "last_gap_classification": self.last_gap_classification,
                "telemetry_accept_count": self.accept_count,
                "telemetry_accept_avg_ms": (
                    None
                    if self.accept_count == 0
                    else self.accept_duration_total_s / self.accept_count * 1000.0
                ),
                "telemetry_accept_max_ms": (
                    None if self.accept_count == 0 else self.accept_duration_max_s * 1000.0
                ),
                "csv_queue_depth": None if self.logger is None else self.logger.queue_depth(),
                "csv_write_stall_count": None if self.logger is None else self.logger.write_stall_count,
                "csv_write_max_ms": (
                    None if self.logger is None else self.logger.write_duration_max_s * 1000.0
                ),
                "udp_receive_buffer_bytes": self.udp_receive_buffer_bytes,
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
            self.pi_ready = False
            self.run_active = False

    def set_csv_source(self, name: str, duration_s: float, cadence_s: float | None) -> None:
        with self.lock:
            self.pi_host = name
            self.source_mode = "csv"
            self.duration_s = duration_s
            self.sample_cadence_s = cadence_s
            self.connection_state = "offline"
            self.connection_message = "CSV replay is local to this dashboard session."
            self.pi_ready = False
            self.run_active = False

    def reset_display_run(self) -> int:
        """Mark a new browser display run without affecting raw capture."""
        with self.lock:
            self.display_run += 1
            return self.display_run

    def begin_hardware_run(self) -> None:
        """Roll the raw capture at the start of a live hardware run."""
        with self.lock:
            logger = self.logger if self.source_mode == "live" else None
        if logger is not None:
            logger.roll()

    def set_run_active(self, active: bool) -> None:
        with self.lock:
            self.run_active = active

    def set_pi_ready(self, ready: bool) -> None:
        with self.lock:
            self.pi_ready = ready

    def set_pid_status(self, status: dict[str, Any] | None) -> None:
        with self.lock:
            self.pid_status = None if status is None else dict(status)


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
        self._last_display_started: float | None = None
        self.display_lateness_max_s = 0.0
        self.display_work_max_s = 0.0
        self.display_stall_count = 0
        self.last_display_stall_log = 0.0
        self.sse_stall_count = 0
        self.sse_flush_max_s = 0.0

    def run(self) -> None:
        interval = 1.0 / DISPLAY_HZ
        while not self._stopping:
            started = time.monotonic()
            if self._last_display_started is not None:
                lateness = max(0.0, started - self._last_display_started - interval)
                self.display_lateness_max_s = max(self.display_lateness_max_s, lateness)
                if lateness > TELEMETRY_GAP_S:
                    self.display_stall_count += 1
                    if started - self.last_display_stall_log >= 1.0:
                        self.last_display_stall_log = started
                        DIAGNOSTIC_LOGGER.warning(
                            "dashboard display loop stalled for %.3f s",
                            lateness,
                            extra={
                                "event_data": {
                                    "event": "dashboard_display_loop_stall",
                                    "duration_s": lateness,
                                }
                            },
                        )
            self._last_display_started = started
            self._display_times.append(started)
            while self._display_times and self._display_times[0] < started - 1.0:
                self._display_times.popleft()
            telemetry, status = self.state.snapshot(DISPLAY_HZ)
            status.update(
                {
                    "dashboard_display_lateness_max_ms": self.display_lateness_max_s * 1000.0,
                    "dashboard_display_work_max_ms": self.display_work_max_s * 1000.0,
                    "dashboard_display_stall_count": self.display_stall_count,
                    "dashboard_sse_stall_count": self.sse_stall_count,
                    "dashboard_sse_flush_max_ms": self.sse_flush_max_s * 1000.0,
                }
            )
            with self.condition:
                self.status = status
                if self._static_history is not None:
                    self.telemetry = None
                elif telemetry is not None and telemetry["sequence"] != self._history_sequence:
                    self.telemetry = telemetry
                    self._append_history(telemetry)
                self.version += 1
                self.condition.notify_all()
            work_s = time.monotonic() - started
            self.display_work_max_s = max(self.display_work_max_s, work_s)
            if work_s > TELEMETRY_GAP_S:
                self.display_stall_count += 1
                if time.monotonic() - self.last_display_stall_log >= 1.0:
                    self.last_display_stall_log = time.monotonic()
                    DIAGNOSTIC_LOGGER.warning(
                        "dashboard display update stalled for %.3f s",
                        work_s,
                        extra={
                            "event_data": {
                                "event": "dashboard_display_work_stall",
                                "duration_s": work_s,
                            }
                        },
                    )
            time.sleep(max(0.0, interval - work_s))

    def record_sse_flush(self, duration_s: float) -> None:
        self.sse_flush_max_s = max(self.sse_flush_max_s, duration_s)
        if duration_s > TELEMETRY_GAP_S:
            self.sse_stall_count += 1

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
            "run_id": int(_number(row, "run_id")),
            "packet_seq": int(_number(row, "packet_seq")),
            "loop_seq": int(_number(row, "loop_seq")),
            "sender_monotonic_ns": int(_number(row, "sender_monotonic_ns")),
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
            },
            "motion": {
                "raw_completed_velocity_sps": _number(row, "raw_completed_velocity_sps", "vel_error"),
                "corrected_axle_velocity_sps": _number(row, "corrected_axle_velocity_sps", "measured_vel_sps"),
                "left_target_sps": _number(row, "left_target_sps"),
                "right_target_sps": _number(row, "right_target_sps"),
                "left_slewed_sps": _number(row, "left_slewed_sps", "left_target_sps"),
                "right_slewed_sps": _number(row, "right_slewed_sps", "right_target_sps"),
                "left_actual_steps": _number(row, "left_actual_steps"),
                "right_actual_steps": _number(row, "right_actual_steps"),
                "user_velocity_mps": _number(row, "user_velocity_mps"),
                "reference_velocity_mps": _number(row, "reference_velocity_mps"),
                "reference_acceleration_mps2": _number(row, "reference_acceleration_mps2"),
                "reference_jerk_mps3": _number(row, "reference_jerk_mps3"),
                "velocity_feedback_estimate_mps": _number(row, "velocity_feedback_estimate_mps"),
                "velocity_error_mps": _number(row, "velocity_error_mps"),
                "velocity_feedback_valid": bool(_number(row, "velocity_feedback_valid")),
                "velocity_feedback_active": bool(_number(row, "velocity_feedback_active")),
            },
            "controller": {
                "command_sps": _number(row, "u_sps"),
                "reference_acceleration_mps2": _number(row, "reference_acceleration_mps2"),
                "velocity_feedback_acceleration_mps2": _number(row, "velocity_feedback_acceleration_mps2"),
                "velocity_p_acceleration_mps2": _number(row, "velocity_p_acceleration_mps2"),
                "velocity_i_acceleration_mps2": _number(row, "velocity_i_acceleration_mps2"),
                "velocity_integral_state_mps_s": _number(row, "velocity_integral_state_mps_s"),
                "acceleration_raw_mps2": _number(row, "acceleration_raw_mps2"),
                "acceleration_cmd_mps2": _number(row, "acceleration_cmd_mps2"),
                "drive_pitch_target_deg": _number(row, "drive_pitch_target_deg"),
                "final_pitch_target_deg": _number(row, "final_pitch_target_deg", "pitch_sp_deg"),
                "pitch_error_deg": _number(row, "pitch_error_deg"),
                "planner_acceleration_limited": bool(_number(row, "planner_acceleration_limited")),
                "planner_jerk_limited": bool(_number(row, "planner_jerk_limited")),
                "velocity_integral_limited": bool(_number(row, "velocity_integral_limited")),
                "velocity_anti_windup_active": bool(_number(row, "velocity_anti_windup_active")),
                "com_trim_deg": _number(row, "com_trim_deg"),
                "fixed_com_trim_deg": _number(row, "fixed_com_trim_deg"),
            },
            "timing": {"imu_age_ms": _number(row, "age_ms"), "feedback_age_ms": _number(row, "motor_feedback_age_ms")},
            "flags": {
                "controller": int(_number(row, "controller_fault_flags")),
                "saturation": int(_number(row, "controller_saturation_flags")),
                "actuator_saturation": int(_number(row, "actuator_saturation_flags")),
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
    """Receive telemetry while resolving a configured Pi lazily."""
    def __init__(self, state: TelemetryState, pi_host: str | None, pi_port: int) -> None:
        self.state, self.pi_port = state, pi_port
        self.target = pi_host
        self.pi_address: tuple[str, int] | None = None
        self.next_resolution = 0.0
        self.lock = threading.Lock()
        self.address_listeners: list[Callable[[str], None]] = []
        self.pid_status_sink: Callable[[PidConfigStatusPayload], None] | None = None
        self.stopping = threading.Event()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, UDP_RECEIVE_BUFFER_BYTES)
        self.receive_buffer_bytes = self.socket.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        self.socket.bind(("0.0.0.0", 0))
        self.socket.settimeout(0.2)
        with self.state.lock:
            self.state.udp_receive_buffer_bytes = self.receive_buffer_bytes

    def run(self) -> None:
        next_registration = 0.0
        while not self.stopping.is_set():
            now = time.monotonic()
            with self.lock:
                target = self.target
                address = self.pi_address
                next_resolution = self.next_resolution
            if target and address is None and now >= next_resolution:
                try:
                    resolved = resolve_udp_host(target)
                    with self.lock:
                        if self.target != target:
                            continue
                        self.pi_address = (resolved, self.pi_port)
                        self.next_resolution = 0.0
                    self._notify_address_ready(resolved)
                    with self.state.lock:
                        self.state.connection_state = "registering"
                        self.state.connection_message = f"Resolved {resolved}; waiting for telemetry."
                except RuntimeError as exc:
                    with self.lock:
                        if self.target != target:
                            continue
                        self.pi_address = None
                        self.next_resolution = now + RESOLVE_RETRY_S
                    with self.state.lock:
                        self.state.connection_state = "resolving"
                        self.state.connection_message = str(exc)
            with self.lock:
                address = self.pi_address
            if address and now >= next_registration:
                try:
                    self.socket.sendto(b"\x00\x00", address)
                except OSError:
                    with self.lock:
                        if self.pi_address == address:
                            self.pi_address = None
                            self.next_resolution = 0.0
                next_registration = now + REGISTER_INTERVAL_S
            try:
                packet, _ = self.socket.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(packet) >= 2 and struct.unpack_from("<H", packet)[0] == PID_CONFIG_STATUS_ID:
                if len(packet) != 2 + PidConfigStatusPayload.WIRE_SIZE:
                    continue
                try:
                    status = PidConfigStatusPayload.unpack(packet[2:])
                except (struct.error, ValueError):
                    continue
                sink = self.pid_status_sink
                if sink is not None:
                    sink(status)
                continue
            self.state.accept(packet)

    def close(self) -> None:
        self.stopping.set()
        self.socket.close()

    def configure(self, target: str) -> None:
        with self.lock:
            self.target = target
            self.pi_address = None
            self.next_resolution = 0.0

    def retry_now(self) -> None:
        with self.lock:
            self.pi_address = None
            self.next_resolution = 0.0

    def update_resolved_address(self, address: str) -> None:
        """Publish a heartbeat-resolved address without blocking UDP reception."""
        with self.lock:
            if self.target:
                self.pi_address = (address, self.pi_port)
                self.next_resolution = 0.0
        self._notify_address_ready(address)

    def current_target(self) -> str | None:
        with self.lock:
            return self.target

    def add_address_listener(self, listener: Callable[[str], None]) -> None:
        with self.lock:
            self.address_listeners.append(listener)

    def set_pid_status_sink(self, sink: Callable[[PidConfigStatusPayload], None] | None) -> None:
        self.pid_status_sink = sink

    def send_message(self, message_id: int, payload: bytes) -> bool:
        datagram = struct.pack("<H", message_id) + payload
        with self.lock:
            address = self.pi_address
        if address is None:
            return False
        try:
            self.socket.sendto(datagram, address)
            return True
        except OSError:
            with self.lock:
                if self.pi_address == address:
                    self.pi_address = None
                    self.next_resolution = 0.0
            return False

    def _notify_address_ready(self, address: str) -> None:
        with self.lock:
            listeners = list(self.address_listeners)
        for listener in listeners:
            try:
                listener(address)
            except Exception:
                DIAGNOSTIC_LOGGER.exception("dashboard address-ready listener failed")


class PidSessionController:
    """Keep live PID edits in memory and deliver complete snapshots over UDP."""

    def __init__(self, state: TelemetryState, receiver: UdpReceiver) -> None:
        self.state, self.receiver = state, receiver
        self.baseline = _load_pid_values(PID_CONFIG)
        self.values = dict(self.baseline)
        self.request_id = 0
        self.pending = False
        self.override_active = False
        self.last_address: str | None = None
        self.last_status: dict[str, Any] | None = None
        self.lock = threading.Lock()
        receiver.set_pid_status_sink(self.accept_status)
        receiver.add_address_listener(self._on_address_ready)

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return {
                "ok": True,
                "values": dict(self.values),
                "baseline": dict(self.baseline),
                "override_active": self.override_active,
                "pending": self.pending,
                "last_status": None if self.last_status is None else dict(self.last_status),
            }

    def reset_for_target(self) -> None:
        with self.lock:
            self.values = dict(self.baseline)
            self.override_active = False
            self.pending = False
            self.last_status = None
            self.last_address = None
        self.state.set_pid_status(None)

    def update(self, values: dict[str, Any]) -> dict[str, Any]:
        with self.state.lock:
            if self.state.source_mode != "live":
                raise ValueError("PID tuning requires a live Pi source.")
        normalized = _validate_pid_values(values)
        with self.lock:
            self.values = normalized
            self.override_active = normalized != self.baseline
            self.request_id += 1
            request_id = self.request_id
            self.pending = True
            self.last_status = {
                "request_id": request_id,
                "state": "pending",
                "accepted": None,
                "result_code": None,
                "message": "Sending PID override to the Pi.",
            }
        self.state.set_pid_status(dict(self.last_status))
        sent = self._send_current(request_id)
        with self.lock:
            pending = self.pending
        return {
            "ok": True,
            "values": dict(normalized),
            "request_id": request_id,
            "sent": sent,
            "pending": pending,
            "message": "PID override sent." if sent else "PID override queued until the Pi address is available.",
        }

    def on_start(self) -> None:
        with self.lock:
            self.request_id += 1
            request_id = self.request_id
            self.pending = True
            self.last_status = {
                "request_id": request_id,
                "state": "pending",
                "accepted": None,
                "result_code": None,
                "message": "Applying the dashboard PID snapshot after Start.",
            }
        self.state.set_pid_status(dict(self.last_status))
        self._send_current(request_id)

    def accept_status(self, payload: PidConfigStatusPayload) -> None:
        values = {name: float(getattr(payload.values, name)) for name in PID_CONFIG_FIELDS}
        accepted = bool(payload.accepted)
        with self.lock:
            if payload.request_id != self.request_id:
                return
            self.values = values
            self.override_active = self.values != self.baseline
            self.pending = False
            status = {
                "request_id": int(payload.request_id),
                "state": "applied" if accepted else "rejected",
                "accepted": accepted,
                "result_code": int(payload.result_code),
                "message": _pid_status_message(accepted, int(payload.result_code)),
            }
            self.last_status = status
        self.state.set_pid_status(status)

    def _on_address_ready(self, address: str) -> None:
        with self.lock:
            address_changed = address != self.last_address
            self.last_address = address
            override_active = self.override_active
            status_to_publish: dict[str, Any] | None = None
            if address_changed:
                self.request_id += 1
                request_id = self.request_id
                self.pending = True
                self.last_status = {
                    "request_id": request_id,
                    "state": "pending",
                    "accepted": None,
                    "result_code": None,
                    "message": (
                        "Reapplying the dashboard PID override to the resolved Pi address."
                        if override_active
                        else "Applying the baseline PID snapshot to the resolved Pi address."
                    ),
                }
                status_to_publish = dict(self.last_status)
            else:
                request_id = self.request_id
            pending = self.pending
        if status_to_publish is not None:
            self.state.set_pid_status(status_to_publish)
            self._send_current(request_id)
        elif pending:
            self._send_current(request_id)

    def _send_current(self, request_id: int) -> bool:
        with self.lock:
            values = dict(self.values)
        payload = PidConfigOverridePayload(
            request_id=request_id,
            reserved=0,
            values=ConfigPidValuesPayload(**values),
        ).pack()
        sent = self.receiver.send_message(PID_CONFIG_OVERRIDE_ID, payload)
        if not sent:
            with self.lock:
                self.last_address = None
            return False
        return True


class JoystickCommandController:
    """Validate and send run-gated joystick heartbeat commands."""

    def __init__(self, state: TelemetryState, receiver: UdpReceiver) -> None:
        self.state, self.receiver = state, receiver
        self.lock = threading.Lock()
        self.closed = False

    def send(self, body: dict[str, Any]) -> dict[str, Any]:
        if body.get("release", False) is not False and body.get("release") is not True:
            raise ValueError("Joystick release must be a boolean.")
        release = body.get("release", False) is True
        forward = 0.0 if release else body.get("forward", 0.0)
        turn = 0.0 if release else body.get("turn", 0.0)
        if isinstance(forward, bool) or isinstance(turn, bool):
            raise ValueError("Joystick values must be numeric.")
        try:
            forward, turn = float(forward), float(turn)
        except (TypeError, ValueError):
            raise ValueError("Joystick values must be numeric.") from None
        if not math.isfinite(forward) or not math.isfinite(turn):
            raise ValueError("Joystick values must be finite.")
        if forward < -1.0 or forward > 1.0 or turn < -1.0 or turn > 1.0:
            raise ValueError("Joystick values must be between -1 and 1.")

        active = forward != 0.0 or turn != 0.0
        with self.state.lock:
            source_mode = self.state.source_mode
            run_active = self.state.run_active
        if active and source_mode != "live":
            raise ValueError("Joystick control requires a live Pi source.")
        if active and not run_active:
            raise ValueError("Start the balancer before using joystick control.")
        if active:
            with self.lock:
                if self.closed:
                    raise RuntimeError("Joystick control is shutting down.")

        if not active:
            sent = self._send_neutral()
            return {"ok": True, "sent": sent, "active": False, "forward": 0.0, "turn": 0.0}

        payload = JoystickCommandPayload(forward=forward, turn=turn).pack()
        sent = self.receiver.send_message(EXTERNAL_JOYSTICK_COMMAND_ID, payload)
        if not sent:
            raise RuntimeError("The Pi UDP address is not available yet.")
        return {"ok": True, "sent": True, "active": True, "forward": forward, "turn": turn}

    def release(self) -> None:
        self._send_neutral()

    def close(self) -> None:
        with self.lock:
            self.closed = True
        self._send_neutral()

    def _send_neutral(self) -> bool:
        with self.state.lock:
            live = self.state.source_mode == "live"
        if not live:
            return False
        return self.receiver.send_message(
            EXTERNAL_JOYSTICK_COMMAND_ID, JoystickCommandPayload(0.0, 0.0).pack()
        )


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
            self.receiver.update_resolved_address(address)
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
        try:
            binary_versions = {
                int(match)
                for match in re.findall(
                    rb"BALANCER_PID_CONFIG_VERSION=(\d+)\x00",
                    PI_BINARY.read_bytes(),
                )
            }
            config_versions = re.findall(
                r"^\s*config_version\s*=\s*(\d+)\s*(?:#.*)?$",
                PID_CONFIG.read_text(encoding="utf-8"),
                re.MULTILINE,
            )
        except OSError as exc:
            raise ValueError(f"Could not validate the cross-built binary and pid.conf: {exc}") from None
        if len(binary_versions) != 1:
            raise ValueError(
                "Cross-built binary has no unambiguous PID config version; "
                "run ./build_cmake OFF first."
            )
        if len(config_versions) != 1:
            raise ValueError(f"PID config must contain exactly one integer config_version: {PID_CONFIG}")
        binary_version = next(iter(binary_versions))
        config_version = int(config_versions[0])
        if binary_version != config_version:
            raise ValueError(
                f"PID config version mismatch: binary expects {binary_version}, "
                f"but {PID_CONFIG.name} is version {config_version}."
            )

    def deploy_current(self) -> dict[str, Any]:
        self._validate_current_build()
        output = self._run(["scp", str(PI_BINARY), str(PID_CONFIG), f"{self._target()}:~/"], 120)
        return {"ok": True, "message": output or "Deployed build-pi/balancer_pi and pid.conf to the Pi."}

    def start(self) -> dict[str, Any]:
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
    def __init__(
        self,
        state: TelemetryState,
        hub: SseHub,
        receiver: UdpReceiver,
        pi_port: int,
        deployer: DeploymentManager | None = None,
        pid_controller: PidSessionController | None = None,
        command_controller: JoystickCommandController | None = None,
    ) -> None:
        self.state, self.hub, self.receiver, self.pi_port = state, hub, receiver, pi_port
        self.temp_path: Path | None = None
        self.lock = threading.Lock()
        self.deployer = deployer
        self.pid_controller = pid_controller
        self.command_controller = command_controller

    def configure_live(self, target: str) -> dict[str, Any]:
        target = target.strip()
        if not target:
            raise ValueError("Enter a Pi host, SSH alias, or IP address.")
        if self.command_controller:
            self.command_controller.release()
        if self.pid_controller:
            self.pid_controller.reset_for_target()
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
        if self.command_controller:
            self.command_controller.release()
        if self.pid_controller:
            self.pid_controller.reset_for_target()
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
        if self.command_controller:
            self.command_controller.close()
        self._remove_temp()


def make_handler(
    hub: SseHub,
    state: TelemetryState,
    deployer: DeploymentManager | None = None,
    sources: SourceController | None = None,
    pid_controller: PidSessionController | None = None,
    command_controller: JoystickCommandController | None = None,
):
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
            if self.path == "/api/pid":
                if pid_controller is None:
                    self.send_error(HTTPStatus.NOT_IMPLEMENTED)
                else:
                    self._json(HTTPStatus.OK, pid_controller.snapshot())
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
            if self.path == "/api/pid":
                if pid_controller is None:
                    self.send_error(HTTPStatus.NOT_IMPLEMENTED)
                    return
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    body = json.loads(self.rfile.read(length) or b"{}")
                    if not isinstance(body, dict):
                        raise ValueError("Request body must be a JSON object.")
                    self._json(HTTPStatus.OK, pid_controller.update(body.get("values", body)))
                except (ValueError, RuntimeError, json.JSONDecodeError) as exc:
                    self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)})
                return
            if self.path == "/api/joystick":
                if command_controller is None:
                    self.send_error(HTTPStatus.NOT_IMPLEMENTED)
                    return
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    body = json.loads(self.rfile.read(length) or b"{}")
                    if not isinstance(body, dict):
                        raise ValueError("Request body must be a JSON object.")
                    self._json(HTTPStatus.OK, command_controller.send(body))
                except (ValueError, RuntimeError, json.JSONDecodeError) as exc:
                    self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)})
                return
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
                if self.path == "/api/abort" and command_controller:
                    command_controller.release()
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
                    state.begin_hardware_run()
                    state.set_run_active(True)
                    result["display_run"] = state.reset_display_run()
                    hub.begin_display_run()
                    if pid_controller:
                        pid_controller.on_start()
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
            last_slow_flush_log = 0.0
            try:
                while True:
                    version, telemetry, status = hub.wait_for_update(version)
                    write_duration = 0.0
                    if telemetry is not None and telemetry.get("sequence") != last_telemetry_sequence:
                        payload = b"event: telemetry\ndata: " + json.dumps(
                            telemetry, separators=(",", ":")
                        ).encode() + b"\n\n"
                        write_started = time.monotonic()
                        self.wfile.write(payload)
                        write_duration = max(write_duration, time.monotonic() - write_started)
                        last_telemetry_sequence = int(telemetry["sequence"])
                    now = time.monotonic()
                    status_key = (
                        status.get("run_active"), status.get("telemetry_connected"), status.get("pi_ready"),
                        status.get("connection_state"), status.get("display_run"), status.get("malformed_packets"),
                        status.get("sender_run_id"), status.get("sender_packet_seq"),
                        status.get("telemetry_gap_count"), status.get("last_gap_classification"),
                        json.dumps(status.get("pid_status"), sort_keys=True),
                        tuple(sorted(status.get("latched_flags", {}).items())),
                    )
                    if status_key != last_status_key or now - last_status_at >= 1.0 / STATUS_HZ:
                        payload = b"event: status\ndata: " + json.dumps(
                            status, separators=(",", ":")
                        ).encode() + b"\n\n"
                        write_started = time.monotonic()
                        self.wfile.write(payload)
                        write_duration = max(write_duration, time.monotonic() - write_started)
                        last_status_key, last_status_at = status_key, now
                    flush_started = time.monotonic()
                    self.wfile.flush()
                    flush_duration = time.monotonic() - flush_started
                    total_duration = max(write_duration, flush_duration)
                    hub.record_sse_flush(flush_duration)
                    if total_duration > TELEMETRY_GAP_S and time.monotonic() - last_slow_flush_log > 1.0:
                        last_slow_flush_log = time.monotonic()
                        DIAGNOSTIC_LOGGER.warning(
                            "dashboard SSE write/flush stalled for %.3f s",
                            total_duration,
                            extra={
                                "event_data": {
                                    "event": "dashboard_sse_flush_stall",
                                    "duration_s": total_duration,
                                    "flush_duration_s": flush_duration,
                                    "sender_packet_seq": status.get("sender_packet_seq"),
                                }
                            },
                        )
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
    pid_controller = PidSessionController(state, receiver)
    command_controller = JoystickCommandController(state, receiver)
    heartbeat = PiHeartbeat(state, receiver)
    deployer = DeploymentManager(initial_target)
    sources = SourceController(
        state,
        hub,
        receiver,
        args.pi_port,
        deployer,
        pid_controller,
        command_controller,
    )
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
    server = ThreadingHTTPServer(
        (host, args.port),
        make_handler(hub, state, deployer, sources, pid_controller, command_controller),
    )
    print(f"Dashboard: http://{host}:{args.port} ({source_description})")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        sources.close()
        receiver.close()
        heartbeat.close()
        hub.stop()
        receiver_thread.join(timeout=2.0)
        heartbeat_thread.join(timeout=2.0)
        hub_thread.join(timeout=2.0)
        try:
            logger.close()
        finally:
            diagnostic_listener.stop()
            server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
