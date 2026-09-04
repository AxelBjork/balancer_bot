#!/usr/bin/env python3
"""Print a compact, repeatable summary of a braced recovery timeline.

The simulator CSV intentionally contains the public simulator telemetry rather
than every internal actuator diagnostic.  This tool derives only quantities
that have an unambiguous source in that CSV and reports unavailable signals as
``N/A``.
"""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from tools.telemetry_analysis.frames import read_telemetry_csv
from tools.telemetry_analysis.stepper_geometry import METERS_PER_STEP, WHEEL_RADIUS_M


REPO_ROOT = Path(__file__).resolve().parents[1]
CONFIG_HEADER = REPO_ROOT / "src/services/main/config.h"
SIMULATOR_HEADER = REPO_ROOT / "tests/simulator/balancer_simulator.h"
SIMULATOR_SOURCE = REPO_ROOT / "tests/simulator/balancer_simulator.cpp"

_NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
_FALLBACKS = {
    "gravity": 9.81,
    "first_mass_moment_kg_m": 0.06192,
    "stepper_current_limit_a": 1.065,
    "motor_stall_torque_nm": 0.45,
    "motor_rated_phase_current_a": 1.5,
    "stepper_sqrt_two": math.sqrt(2.0),
    "bus_voltage_v": 11.1,
    "motor_slew_sps_per_s": 200000.0,
    "nominal_balance_max_sps": 16000.0,
    "fallover_shutdown_deg": 40.0,
}


def _source_number(name: str, paths: tuple[Path, ...], fallback: float) -> float:
    pattern = re.compile(rf"\b{re.escape(name)}\s*=\s*({_NUMBER})")
    for path in paths:
        try:
            text = path.read_text(encoding="utf-8")
        except OSError:
            continue
        match = pattern.search(text)
        if match:
            return float(match.group(1))
    return fallback


def _authoritative_constants() -> dict[str, float]:
    return {
        "gravity": _source_number("gravity", (SIMULATOR_HEADER,), _FALLBACKS["gravity"]),
        "first_mass_moment_kg_m": _source_number(
            "first_mass_moment_kg_m", (SIMULATOR_HEADER,), _FALLBACKS["first_mass_moment_kg_m"]
        ),
        "stepper_current_limit_a": _source_number(
            "stepper_current_limit_a", (SIMULATOR_HEADER,), _FALLBACKS["stepper_current_limit_a"]
        ),
        "motor_stall_torque_nm": _source_number(
            "motor_stall_torque_nm", (SIMULATOR_HEADER,), _FALLBACKS["motor_stall_torque_nm"]
        ),
        "motor_rated_phase_current_a": _source_number(
            "motor_rated_phase_current_a",
            (SIMULATOR_HEADER,),
            _FALLBACKS["motor_rated_phase_current_a"],
        ),
        "stepper_sqrt_two": _source_number(
            "stepper_sqrt_two", (SIMULATOR_HEADER,), _FALLBACKS["stepper_sqrt_two"]
        ),
        "bus_voltage_v": _source_number(
            "stepper_bus_voltage_v", (SIMULATOR_SOURCE,), _FALLBACKS["bus_voltage_v"]
        ),
        "motor_slew_sps_per_s": _source_number(
            "motor_slew_sps_per_s", (CONFIG_HEADER,), _FALLBACKS["motor_slew_sps_per_s"]
        ),
        "nominal_balance_max_sps": _source_number(
            "nominal_balance_max_sps", (CONFIG_HEADER,), _FALLBACKS["nominal_balance_max_sps"]
        ),
        "fallover_shutdown_deg": _source_number(
            "fallover_shutdown_deg", (CONFIG_HEADER,), _FALLBACKS["fallover_shutdown_deg"]
        ),
    }


def _number(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _column(frame: pd.DataFrame, name: str) -> np.ndarray | None:
    if name not in frame.columns:
        return None
    values = pd.to_numeric(frame[name], errors="coerce").to_numpy(dtype=float)
    return values if np.isfinite(values).any() else None


def _first_column(frame: pd.DataFrame, *names: str) -> np.ndarray | None:
    for name in names:
        values = _column(frame, name)
        if values is not None:
            return values
    return None


def _first_index(mask: np.ndarray, start: int = 0) -> int | None:
    indices = np.flatnonzero(mask[start:])
    return int(indices[0] + start) if indices.size else None


def _event(time: np.ndarray, index: int | None, values: np.ndarray | None = None) -> dict[str, float] | None:
    if index is None or index >= len(time):
        return None
    result = {"time_s": float(time[index])}
    if values is not None and index < len(values) and np.isfinite(values[index]):
        result["value"] = float(values[index])
    return result


def _event_text(event: dict[str, float] | None, unit: str = "") -> str:
    if event is None:
        return "N/A"
    if "value" not in event:
        return f"{event['time_s']:.3f}s"
    return f"{event['time_s']:.3f}s ({event['value']:.4g}{unit})"


def _fmt(value: Any, digits: int = 4, unit: str = "") -> str:
    number = _number(value)
    if number is None:
        return "N/A"
    return f"{number:.{digits}f}{unit}"


def _bool_text(value: bool | None) -> str:
    if value is None:
        return "N/A"
    return "yes" if value else "no"


def _median_dt(time: np.ndarray) -> float | None:
    if len(time) < 2:
        return None
    positive = np.diff(time)
    positive = positive[np.isfinite(positive) & (positive > 0.0)]
    return float(np.median(positive)) if positive.size else None


def _duration_fraction(time: np.ndarray, mask: np.ndarray) -> float | None:
    if len(time) < 2 or len(mask) != len(time):
        return None
    dt = np.diff(time)
    valid = np.isfinite(dt) & (dt > 0.0)
    total = float(np.sum(dt[valid]))
    return float(np.sum(dt[valid & mask[:-1]])) / total if total > 0.0 else None


def _common_sps(frame: pd.DataFrame) -> np.ndarray | None:
    left = _column(frame, "left_slewed_sps")
    right = _column(frame, "right_slewed_sps")
    if left is not None and right is not None:
        return 0.5 * (left + right)
    return _column(frame, "u_sps")


def _first_threshold(
    values: np.ndarray | None,
    threshold: float,
    time: np.ndarray,
    start: int,
    end: int,
    *,
    absolute: bool = True,
    below: bool = False,
) -> dict[str, float] | None:
    if values is None:
        return None
    selected = values[start : end + 1]
    if absolute:
        selected = np.abs(selected)
    mask = selected <= threshold if below else selected >= threshold
    relative = _first_index(mask)
    return _event(time, start + relative) if relative is not None else None


def _stable_index(
    frame: pd.DataFrame,
    time: np.ndarray,
    start: int,
    end: int,
    brace: np.ndarray | None,
    pitch: np.ndarray | None,
    rate: np.ndarray | None,
    velocity: np.ndarray | None,
    applied: np.ndarray | None,
) -> int | None:
    """Find one second of quiet, unbraced, fault-free recovery.

    This is deliberately a descriptive event definition, not a simulator
    assertion: |pitch| <= 3 deg, |pitch rate| <= 20 deg/s, |velocity| <=
    0.02 m/s, and |post-slew common command| <= 1000 SPS.
    """
    faults = _column(frame, "controller_fault_flags")
    actuator_fault = _column(frame, "actuator_fault")
    stable = np.ones(len(frame), dtype=bool)
    if brace is not None:
        stable &= brace < 0.5
    if pitch is None or rate is None or velocity is None or applied is None:
        return None
    stable &= np.abs(pitch) <= 3.0
    stable &= np.abs(rate) <= 20.0
    stable &= np.abs(velocity) <= 0.02
    stable &= np.abs(applied) <= 1000.0
    if faults is not None:
        stable &= np.abs(faults) < 0.5
    if actuator_fault is not None:
        stable &= np.abs(actuator_fault) < 0.5

    for index in range(start, end + 1):
        if not stable[index]:
            continue
        finish = int(np.searchsorted(time, time[index] + 1.0, side="left"))
        if finish > end or finish >= len(time):
            continue
        if np.all(stable[index : finish + 1]):
            return index
    return None


def _metadata_for(csv_path: Path) -> dict[str, Any]:
    path = csv_path.with_name("metadata.json")
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}


def _pid_config_for(csv_path: Path, metadata: dict[str, Any]) -> Path | None:
    candidates: list[Path] = []
    configured = metadata.get("pid_profile")
    if isinstance(configured, str) and configured:
        configured_path = Path(configured)
        candidates.extend((configured_path, csv_path.parent / configured_path.name, REPO_ROOT / configured_path))
    candidates.extend((csv_path.parent / "pid.conf", REPO_ROOT / "pid.conf"))
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    return None


def _pid_values(path: Path | None) -> dict[str, float]:
    if path is None:
        return {}
    result: dict[str, float] = {}
    pattern = re.compile(rf"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*=\s*({_NUMBER})")
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError:
        return result
    for line in lines:
        match = pattern.match(line)
        if match:
            result[match.group(1)] = float(match.group(2))
    return result


def _infer_brace_angle(frame: pd.DataFrame, metadata: dict[str, Any], override: float | None) -> float | None:
    if override is not None:
        return abs(override)
    values = _column(frame, "brace_pitch_deg")
    if values is not None:
        nonzero = np.abs(values[np.abs(values) > 1.0e-9])
        if nonzero.size:
            return float(np.median(nonzero))
    value = _number(metadata.get("brace_pitch_deg"))
    return abs(value) if value is not None else None


def _tail_stats(frame: pd.DataFrame, time: np.ndarray, end: int) -> dict[str, float | None]:
    if end < 0 or end >= len(frame):
        return {}
    start_time = time[end] - 2.0
    indices = np.flatnonzero(time[: end + 1] >= start_time)
    start = int(indices[0]) if indices.size else 0
    tail = frame.iloc[start : end + 1]
    result: dict[str, float | None] = {}
    for key, output in (
        ("plant_pitch_deg", "tail_pitch_rms_deg"),
        ("plant_pitch_rate_dps", "tail_pitch_rate_rms_dps"),
        ("plant_velocity_mps", "tail_velocity_rms_mps"),
    ):
        values = _column(tail, key)
        if values is not None:
            result[output] = float(np.sqrt(np.mean(values * values)))
    applied = _common_sps(tail)
    if applied is not None:
        result["tail_command_rms_sps"] = float(np.sqrt(np.mean(applied * applied)))
    velocity = _column(tail, "plant_velocity_mps")
    pitch = _column(tail, "plant_pitch_deg")
    applied = _common_sps(tail)
    result["final_pitch_deg"] = float(pitch[-1]) if pitch is not None else None
    result["final_velocity_mps"] = float(velocity[-1]) if velocity is not None else None
    result["late_balance_rail_fraction"] = None
    return result


def analyze_recovery(
    frame: pd.DataFrame,
    *,
    metadata: dict[str, Any] | None = None,
    brace_angle_deg: float | None = None,
    label: str | None = None,
    constants: dict[str, float] | None = None,
    pid_values: dict[str, float] | None = None,
) -> dict[str, Any]:
    """Analyze one canonical recovery frame and return a JSON-ready summary."""
    metadata = metadata or {}
    constants = constants or _authoritative_constants()
    pid_values = pid_values or {}
    data = frame.copy()
    if "t_sec" not in data.columns:
        raise ValueError("CSV must contain t_sec")
    data = data.sort_values("t_sec").reset_index(drop=True)
    time = pd.to_numeric(data["t_sec"], errors="coerce").to_numpy(dtype=float)
    valid = np.isfinite(time)
    data = data.loc[valid].reset_index(drop=True)
    time = time[valid]
    if data.empty or time.size == 0:
        raise ValueError("CSV contains no finite t_sec samples")

    brace = _column(data, "brace_contact_active")
    recovery = _column(data, "recovery_command_active")
    if recovery is None:
        user_velocity = _column(data, "user_velocity_mps")
        recovery = np.abs(user_velocity) > 1.0e-9 if user_velocity is not None else None
    enable_index = _first_index(recovery >= 0.5) if recovery is not None else None
    if enable_index is None:
        enable_index = 0

    pitch = _first_column(data, "plant_pitch_deg", "pitch_deg")
    rate = _first_column(data, "plant_pitch_rate_dps", "pitch_rate_dps")
    fused = _column(data, "fused_pitch_deg")
    velocity = _column(data, "plant_velocity_mps")
    requested = _first_column(data, "balance_unclamped_sps", "u_sps")
    balance = _column(data, "u_sps")
    applied = _common_sps(data)

    pre_start_time = time[enable_index] - 0.5
    pre_indices = np.flatnonzero(time[: enable_index + 1] >= pre_start_time)
    pre_start = int(pre_indices[0]) if pre_indices.size else 0
    settled_pitch = float(np.median(pitch[pre_start:enable_index])) if pitch is not None and enable_index > pre_start else None
    fused_at_enable = float(fused[enable_index]) if fused is not None else None
    true_at_enable = float(pitch[enable_index]) if pitch is not None else None
    estimator_error = (
        fused_at_enable - true_at_enable
        if fused_at_enable is not None and true_at_enable is not None
        else None
    )

    release_index = None
    if brace is not None and brace[enable_index] >= 0.5:
        release_index = _first_index(brace < 0.5, enable_index + 1)
    else:
        release_index = enable_index

    contact_after_release = None
    if brace is not None and release_index is not None:
        contact_after_release = _first_index(brace >= 0.5, release_index + 1)

    provisional_end = contact_after_release if contact_after_release is not None else len(data) - 1
    stable_index = _stable_index(data, time, enable_index, provisional_end, brace, pitch, rate, velocity, applied)
    end_index = stable_index if stable_index is not None else provisional_end

    angle = _infer_brace_angle(data, metadata, brace_angle_deg)
    static_angle = abs(settled_pitch) if settled_pitch is not None else angle
    gravity_torque = (
        constants["gravity"] * constants["first_mass_moment_kg_m"] * math.sin(math.radians(static_angle))
        if static_angle is not None
        else None
    )
    torque_constant = constants["motor_stall_torque_nm"] / (
        constants["stepper_sqrt_two"] * constants["motor_rated_phase_current_a"]
    )
    theoretical_per_motor = torque_constant * constants["stepper_current_limit_a"]
    theoretical_total = 2.0 * theoretical_per_motor
    static_margin = theoretical_total - gravity_torque if gravity_torque is not None else None
    static_margin_percent = (
        100.0 * static_margin / gravity_torque
        if static_margin is not None and gravity_torque not in (None, 0.0)
        else None
    )

    interval = data.iloc[enable_index : end_index + 1]
    interval_time = time[enable_index : end_index + 1]
    interval_requested = requested[enable_index : end_index + 1] if requested is not None else None
    interval_balance = balance[enable_index : end_index + 1] if balance is not None else None
    interval_applied = applied[enable_index : end_index + 1] if applied is not None else None
    interval_fcmd = _column(interval, "f_cmd")
    interval_torque = interval_fcmd * WHEEL_RADIUS_M if interval_fcmd is not None else None
    initial_sign = 1.0 if (static_angle is None or settled_pitch is None or settled_pitch >= 0.0) else -1.0
    useful_torque = initial_sign * interval_torque if interval_torque is not None else None
    inward_peak_index = int(np.argmax(useful_torque)) if useful_torque is not None else None
    outward_torque = -useful_torque if useful_torque is not None else None
    outward_peak_index = int(np.argmax(outward_torque)) if outward_torque is not None else None
    phase = _column(interval, "phase_error_steps")
    traction_limit = _column(interval, "traction_limit_n")
    contact_force = _column(interval, "f_app")
    traction_utilization = (
        np.divide(np.abs(contact_force), np.abs(traction_limit), out=np.full(len(interval), np.nan), where=np.abs(traction_limit) > 1.0e-12)
        if contact_force is not None and traction_limit is not None
        else None
    )

    def absolute_peak(values: np.ndarray | None) -> float | None:
        return float(np.nanmax(np.abs(values))) if values is not None and np.isfinite(values).any() else None

    def max_value(values: np.ndarray | None) -> float | None:
        return float(np.nanmax(values)) if values is not None and np.isfinite(values).any() else None

    def event_for_max(values: np.ndarray | None, transform: Any = None) -> dict[str, float] | None:
        if values is None or not np.isfinite(values).any():
            return None
        selected = transform(values) if transform else values
        if not np.isfinite(selected).any():
            return None
        index = int(np.nanargmax(selected))
        event = _event(interval_time, index, selected)
        if event is not None:
            event["time_s"] = float(interval_time[index])
        return event

    slew_flags = _column(interval, "actuator_saturation_flags")
    voltage_left = _column(interval, "stepper_voltage_saturated_left")
    voltage_right = _column(interval, "stepper_voltage_saturated_right")
    voltage_saturated = (
        np.maximum(voltage_left, voltage_right) >= 0.5
        if voltage_left is not None and voltage_right is not None
        else None
    )
    cap = pid_values.get("balance_max_sps", constants["nominal_balance_max_sps"])
    cap_mask = np.abs(interval_balance) >= cap * (1.0 - 1.0e-9) if interval_balance is not None else None
    fallover_limit = constants["fallover_shutdown_deg"]
    pitch_interval = pitch[enable_index : end_index + 1] if pitch is not None else None

    def crossing(threshold: float) -> dict[str, float] | None:
        if pitch_interval is None:
            return None
        mask = np.abs(pitch_interval) <= threshold
        index = _first_index(mask)
        return _event(interval_time, index) if index is not None else None

    first_slew = _event(interval_time, _first_index(slew_flags != 0.0) if slew_flags is not None else None)
    first_voltage = _event(interval_time, _first_index(voltage_saturated) if voltage_saturated is not None else None)
    first_cap = _event(interval_time, _first_index(cap_mask) if cap_mask is not None else None)
    brace_release = _event(time, release_index)
    brace_recontact = _event(time, contact_after_release)
    useful_event = _event(interval_time, inward_peak_index, useful_torque) if useful_torque is not None else None
    outward_event = _event(interval_time, outward_peak_index, outward_torque) if outward_torque is not None else None
    phase_event = event_for_max(phase, np.abs)
    traction_event = event_for_max(traction_utilization)
    release_torque = (
        useful_torque[release_index - enable_index]
        if release_index is not None
        and useful_torque is not None
        and enable_index <= release_index <= end_index
        else None
    )

    actual_wheel_velocity = _column(interval, "actual_wheel_velocity")
    emitted = _column(interval, "emitted_step_velocity_sps")
    wheel_sps = actual_wheel_velocity / METERS_PER_STEP if actual_wheel_velocity is not None else None
    fault_flags = _column(interval, "controller_fault_flags")
    traction_flag = _column(interval, "traction_saturated")
    if traction_flag is None:
        traction_flag = _column(interval, "stepper_traction_saturated")
    slip_velocity = _column(interval, "stepper_slip_velocity_mps")
    slip_distance = _column(interval, "stepper_accumulated_slip_distance_m")
    unwrapped_phase = _column(interval, "stepper_unwrapped_electrical_phase_error_left_rad")
    if unwrapped_phase is None:
        unwrapped_phase = _column(interval, "stepper_unwrapped_electrical_phase_error_rad")
    unwrapped_phase_right = _column(
        interval, "stepper_unwrapped_electrical_phase_error_right_rad"
    )
    if unwrapped_phase is not None and unwrapped_phase_right is not None:
        unwrapped_phase = np.maximum(np.abs(unwrapped_phase), np.abs(unwrapped_phase_right))
    cycle_slips = _column(interval, "stepper_accumulated_electrical_cycle_slips_left")
    cycle_slips_right = _column(
        interval, "stepper_accumulated_electrical_cycle_slips_right"
    )
    if cycle_slips is not None and cycle_slips_right is not None:
        cycle_slips = np.maximum(cycle_slips, cycle_slips_right)
    current_left = _column(interval, "stepper_current_a_left")
    current_right = _column(interval, "stepper_current_a_right")
    current_magnitude = (
        np.maximum(np.abs(current_left), np.abs(current_right))
        if current_left is not None and current_right is not None
        else None
    )

    unavailable: list[str] = []
    if voltage_saturated is None:
        unavailable.append(
            "voltage saturation: missing stepper_voltage_saturated_left/right"
        )
    if current_magnitude is None:
        unavailable.append("motor current: missing stepper_current_a_left/right")
    if unwrapped_phase is None:
        unavailable.append(
            "unwrapped phase: missing stepper_unwrapped_electrical_phase_error_left/right_rad"
        )
    if cycle_slips is None:
        unavailable.append(
            "cycle slips: missing stepper_accumulated_electrical_cycle_slips_left/right"
        )
    if slip_velocity is None:
        unavailable.append("slip velocity: missing stepper_slip_velocity_mps")
    if slip_distance is None:
        unavailable.append(
            "slip distance: missing stepper_accumulated_slip_distance_m"
        )
    if traction_flag is None:
        unavailable.append(
            "traction saturation flag: missing traction_saturated/stepper_traction_saturated"
        )

    summary: dict[str, Any] = {
        "label": label or metadata.get("scenario_name") or metadata.get("scenario_id") or "recovery",
        "csv": str(metadata.get("source_csv", "")),
        "config": {
            "brace_angle_deg": angle,
            "bus_voltage_v": constants["bus_voltage_v"],
            "current_limit_a_per_motor": constants["stepper_current_limit_a"],
            "balance_max_sps": cap,
            "sps_slew_limit": constants["motor_slew_sps_per_s"],
            "settled_true_pitch_deg": settled_pitch,
            "fused_pitch_at_recovery_deg": fused_at_enable,
            "estimator_error_fused_minus_true_deg": estimator_error,
        },
        "static_authority": {
            "angle_used_deg": static_angle,
            "gravity_torque_total_nm": gravity_torque,
            "theoretical_motor_torque_per_motor_nm": theoretical_per_motor,
            "theoretical_motor_torque_total_nm": theoretical_total,
            "static_torque_margin_total_nm": static_margin,
            "static_torque_margin_percent_of_gravity": static_margin_percent,
            "torque_source": "f_cmd [N force-equivalent] * wheel radius [m]; f_app is contact force and is not motor torque",
        },
        "events": {
            "recovery_controller_enable": _event(time, enable_index),
            "brace_release": brace_release,
            "first_slew_limiting": first_slew,
            "first_bus_voltage_saturation": first_voltage,
            "first_balance_cap_saturation": first_cap,
            "crossed_normal_fallover_angle": crossing(fallover_limit),
            "crossed_20_deg": crossing(20.0),
            "crossed_upright_1_deg": crossing(1.0),
            "maximum_useful_inward_motor_torque": useful_event,
            "maximum_outward_motor_torque": outward_event,
            "maximum_absolute_wrapped_phase_error": phase_event,
            "maximum_traction_utilization": traction_event,
            "brace_recontact": brace_recontact,
            "stable_recovery": _event(time, stable_index),
        },
        "recovery_interval": {
            "start_s": float(time[enable_index]),
            "end_s": float(time[end_index]),
            "duration_s": float(time[end_index] - time[enable_index]),
            "end_reason": "stable recovery" if stable_index is not None else "brace recontact" if contact_after_release is not None else "end of run",
        },
        "recovery_metrics": {
            "peak_requested_sps": absolute_peak(interval_requested),
            "peak_applied_post_slew_sps": absolute_peak(interval_applied),
            "peak_emitted_field_sps": absolute_peak(emitted),
            "peak_mechanical_wheel_sps": absolute_peak(wheel_sps),
            "peak_mechanical_velocity_mps": absolute_peak(_column(interval, "plant_velocity_mps")),
            "sps_at_brace_release": (
                float(interval_applied[release_index - enable_index])
                if interval_applied is not None
                and release_index is not None
                and enable_index <= release_index <= end_index
                else None
            ),
            "sps_at_upright_crossing": None,
            "slew_limited_fraction": _duration_fraction(interval_time, slew_flags != 0.0) if slew_flags is not None else None,
            "balance_cap_fraction": _duration_fraction(interval_time, cap_mask) if cap_mask is not None else None,
        },
        "electrical_phase": {
            "peak_useful_inward_torque_total_nm": max_value(useful_torque),
            "peak_outward_torque_total_nm": max_value(outward_torque),
            "useful_torque_at_brace_release_total_nm": _number(release_torque),
            "peak_current_a": max_value(current_magnitude),
            "voltage_saturation_onset_sps": None,
            "voltage_saturated_fraction": _duration_fraction(interval_time, voltage_saturated) if voltage_saturated is not None else None,
            "peak_abs_wrapped_phase_error_steps": absolute_peak(phase),
            "peak_abs_unwrapped_phase_error_rad": absolute_peak(unwrapped_phase),
            "cycle_slip_count": max_value(cycle_slips),
        },
        "traction": {
            "maximum_derived_actual_contact_utilization": max_value(traction_utilization),
            "traction_saturated": bool(np.any(traction_flag >= 0.5)) if traction_flag is not None else None,
            "peak_slip_velocity_mps": absolute_peak(slip_velocity),
            "accumulated_slip_distance_m": max_value(slip_distance),
        },
        "unavailable_inputs": unavailable,
        "outcome": {
            "left_brace": release_index is not None and release_index <= end_index,
            "crossed_fallover_angle": crossing(fallover_limit) is not None,
            "crossed_upright": crossing(1.0) is not None,
            "recontacted_brace": contact_after_release is not None,
            "stabilized": stable_index is not None,
            "faulted": bool(np.any(np.abs(fault_flags) >= 0.5)) if fault_flags is not None else None,
        },
    }
    upright_event = summary["events"]["crossed_upright_1_deg"]
    if upright_event is not None:
        upright_index = int(np.searchsorted(time, upright_event["time_s"], side="left"))
        if applied is not None and enable_index <= upright_index <= end_index:
            summary["recovery_metrics"]["sps_at_upright_crossing"] = float(applied[upright_index])

    if stable_index is not None:
        tail = _tail_stats(data, time, len(data) - 1)
        if tail:
            rail = _column(data.iloc[max(0, len(data) - int(max(2.0 / (_median_dt(time) or 0.002), 1))):], "u_sps")
            if rail is not None:
                tail["late_balance_rail_fraction"] = float(np.mean(np.abs(rail) >= cap * (1.0 - 1.0e-9)))
            summary["tail"] = tail
    else:
        summary["tail"] = None
    return summary


def _print_section(name: str, values: dict[str, Any], *, event_units: dict[str, str] | None = None) -> None:
    print(name)
    event_units = event_units or {}
    for key, value in values.items():
        display_key = key.replace("_", " ")
        if isinstance(value, dict) and "time_s" in value:
            print(f"{display_key}={_event_text(value, event_units.get(key, ''))}")
        elif isinstance(value, bool):
            print(f"{display_key}={_bool_text(value)}")
        elif value is None:
            print(f"{display_key}=N/A")
        elif isinstance(value, (int, float)):
            print(f"{display_key}={_fmt(value)}")
        else:
            print(f"{display_key}={value}")
    print()


def print_summary(summary: dict[str, Any]) -> None:
    print(f"RECOVERY RUN: {summary['label']}\n")
    _print_section("CONFIG", summary["config"], event_units={})
    _print_section("STATIC AUTHORITY", summary["static_authority"], event_units={})
    _print_section(
        "EVENTS",
        summary["events"],
        event_units={
            "maximum_useful_inward_motor_torque": " Nm",
            "maximum_outward_motor_torque": " Nm",
        },
    )
    _print_section("RECOVERY INTERVAL", summary["recovery_interval"])
    _print_section("RECOVERY METRICS", summary["recovery_metrics"])
    _print_section("ELECTRICAL / PHASE", summary["electrical_phase"])
    _print_section("TRACTION", summary["traction"])
    if summary.get("unavailable_inputs"):
        _print_section(
            "UNAVAILABLE INPUTS",
            {f"missing_{index + 1}": value for index, value in enumerate(summary["unavailable_inputs"])},
        )
    _print_section("OUTCOME", summary["outcome"])
    if summary.get("tail") is not None:
        _print_section("STABLE-TAIL (last 2 s)", summary["tail"])


def main() -> int:
    parser = argparse.ArgumentParser(description="Summarize a braced recovery timeline.csv.")
    parser.add_argument("csv", type=Path, help="Path to timeline.csv")
    parser.add_argument("--brace-angle", type=float, help="Brace angle in degrees; otherwise infer from CSV/metadata")
    parser.add_argument("--label", help="Short run label")
    parser.add_argument("--json-out", type=Path, help="Optional path for a machine-readable JSON summary")
    args = parser.parse_args()

    frame = read_telemetry_csv(args.csv)
    metadata = _metadata_for(args.csv)
    pid_path = _pid_config_for(args.csv, metadata)
    pid_values = _pid_values(pid_path)
    metadata = dict(metadata)
    metadata["source_csv"] = str(args.csv)
    summary = analyze_recovery(
        frame,
        metadata=metadata,
        brace_angle_deg=args.brace_angle,
        label=args.label,
        pid_values=pid_values,
    )
    print_summary(summary)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
