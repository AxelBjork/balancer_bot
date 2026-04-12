from __future__ import annotations

import argparse
import csv
import json
import math
import re
import shutil
from dataclasses import dataclass, field
from pathlib import Path
from statistics import median
from typing import Any


_DEFAULT_COLUMNS = [
    "sim_time_s",
    "pitch_deg",
    "pitch_rate_dps",
    "filtered_pitch_rate_dps",
    "raw_acc_pitch_deg",
    "fused_pitch_deg",
    "gyro_pitch_rate_dps",
    "pitch_sp_deg",
    "rate_sp_dps",
    "u_sps",
    "left_sps",
    "right_sps",
    "vel_error",
    "vel_i_term",
    "vel_p_term",
    "target_vel_sps",
    "measured_vel_sps",
    "filtered_vel_sps",
    "position_target_vel_sps",
    "out_norm",
    "effective_pitch_sp_deg",
    "pitch_trim_deg",
    "trim_active",
    "plant_pitch_deg",
    "plant_pitch_rate_dps",
    "plant_position",
    "plant_velocity",
    "target_wheel_velocity",
    "actual_wheel_velocity",
    "velocity_error",
    "f_cmd",
    "f_app",
    "external_force_n",
    "external_com_bias_rad",
    "x_ddot",
    "theta_ddot",
    "command_saturated",
    "force_saturated",
]

_KV_RE = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)=([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)")
_MAX_PLOT_POINTS = 2000
_PLOT_LEFT = 78
_PLOT_RIGHT = 950
_PLOT_TOP = 28
_PLOT_BOTTOM = 248
_PLOT_WIDTH = _PLOT_RIGHT - _PLOT_LEFT
_PLOT_HEIGHT = _PLOT_BOTTOM - _PLOT_TOP


def _to_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(out):
        return None
    return out


def _series(rows: list[dict[str, Any]], key: str) -> list[float]:
    values: list[float] = []
    for row in rows:
        value = _to_float(row.get(key))
        if value is not None:
            values.append(value)
    return values


def _pitch_key(rows: list[dict[str, Any]]) -> str:
    return "plant_pitch_deg" if any("plant_pitch_deg" in row for row in rows) else "pitch_deg"


def _time_key(rows: list[dict[str, Any]]) -> str | None:
    for candidate in ("sim_time_s", "t_sec", "time_s", "time"):
        if any(_to_float(row.get(candidate)) is not None for row in rows):
            return candidate
    return None


def _aligned_numeric_series(
    rows: list[dict[str, Any]],
    source_key: str,
    response_key: str,
    *,
    time_key: str | None = None,
) -> tuple[list[float], list[float], list[float]]:
    time_key = time_key or _time_key(rows)
    ts: list[float] = []
    source: list[float] = []
    response: list[float] = []
    for row in rows:
        src = _to_float(row.get(source_key))
        rsp = _to_float(row.get(response_key))
        t = _to_float(row.get(time_key)) if time_key is not None else None
        if src is None or rsp is None:
            continue
        if time_key is not None and t is None:
            continue
        ts.append(0.0 if t is None else t)
        source.append(src)
        response.append(rsp)
    return ts, source, response


def estimate_lag_scale(
    rows: list[dict[str, Any]],
    source_key: str,
    response_key: str,
    *,
    max_lag_s: float = 0.5,
    min_abs_source: float = 0.0,
    demean: bool = True,
) -> dict[str, Any]:
    ts, source, response = _aligned_numeric_series(rows, source_key, response_key)
    result: dict[str, Any] = {
        "source_key": source_key,
        "response_key": response_key,
        "sample_count": 0,
        "lag_steps": None,
        "lag_s": None,
        "scale": None,
        "correlation": None,
        "rmse": None,
    }
    if len(ts) < 3:
        return result

    deltas = [b - a for a, b in zip(ts, ts[1:]) if b > a]
    if not deltas:
        return result
    dt = median(deltas)
    if dt <= 0.0:
        return result
    max_lag_steps = max(0, int(round(max_lag_s / dt)))

    best: dict[str, Any] | None = None
    for lag_steps in range(max_lag_steps + 1):
        if lag_steps >= len(source) - 1:
            break
        src = source[: len(source) - lag_steps] if lag_steps > 0 else list(source)
        rsp = response[lag_steps:]
        if len(src) != len(rsp) or len(src) < 3:
            continue

        if min_abs_source > 0.0:
            pairs = [(sx, ry) for sx, ry in zip(src, rsp) if abs(sx) >= min_abs_source]
            if len(pairs) < 3:
                continue
            src = [sx for sx, _ in pairs]
            rsp = [ry for _, ry in pairs]

        if demean:
            src_mean = sum(src) / len(src)
            rsp_mean = sum(rsp) / len(rsp)
            src_fit = [value - src_mean for value in src]
            rsp_fit = [value - rsp_mean for value in rsp]
        else:
            src_fit = list(src)
            rsp_fit = list(rsp)

        denom = sum(value * value for value in src_fit)
        if denom <= 1e-12:
            continue
        scale = sum(sx * ry for sx, ry in zip(src_fit, rsp_fit)) / denom
        residuals = [(scale * sx) - ry for sx, ry in zip(src_fit, rsp_fit)]
        rmse = math.sqrt(sum(value * value for value in residuals) / len(residuals))
        lag_src = src_fit
        lag_rsp = rsp_fit
        if len(src_fit) >= 4:
            diff_src = [b - a for a, b in zip(src_fit, src_fit[1:])]
            diff_rsp = [b - a for a, b in zip(rsp_fit, rsp_fit[1:])]
            if sum(abs(value) for value in diff_src) > 1e-9 and sum(abs(value) for value in diff_rsp) > 1e-9:
                lag_src = diff_src
                lag_rsp = diff_rsp
        lag_denom = sum(value * value for value in lag_src)
        rsp_energy = sum(value * value for value in lag_rsp)
        if rsp_energy <= 1e-12:
            continue
        if lag_denom <= 1e-12:
            continue
        corr = sum(sx * ry for sx, ry in zip(lag_src, lag_rsp)) / math.sqrt(lag_denom * rsp_energy)

        candidate = {
            "source_key": source_key,
            "response_key": response_key,
            "sample_count": len(src),
            "lag_steps": lag_steps,
            "lag_s": lag_steps * dt,
            "scale": scale,
            "correlation": corr,
            "rmse": rmse,
            "dt_s": dt,
        }
        if best is None:
            best = candidate
            continue
        if abs(candidate["correlation"]) > abs(best["correlation"]) + 1e-9:
            best = candidate
            continue
        if abs(candidate["correlation"] - best["correlation"]) <= 1e-9 and candidate["rmse"] < best["rmse"]:
            best = candidate

    return best or result


def analyze_timeline_rows(rows: list[dict[str, Any]]) -> dict[str, Any]:
    analysis: dict[str, Any] = {
        "time_key": _time_key(rows),
    }
    analysis["estimator_pitch"] = estimate_lag_scale(
        rows,
        "raw_acc_pitch_deg",
        "fused_pitch_deg",
        max_lag_s=0.5,
        min_abs_source=0.25,
    )
    analysis["estimator_pitch_rate"] = estimate_lag_scale(
        rows,
        "gyro_pitch_rate_dps",
        "filtered_pitch_rate_dps",
        max_lag_s=0.5,
        min_abs_source=5.0,
    )
    analysis["drive_command_to_measured_velocity"] = estimate_lag_scale(
        rows,
        "u_sps",
        "measured_vel_sps",
        max_lag_s=1.0,
        min_abs_source=50.0,
    )
    analysis["drive_command_to_filtered_velocity"] = estimate_lag_scale(
        rows,
        "u_sps",
        "filtered_vel_sps",
        max_lag_s=1.0,
        min_abs_source=50.0,
    )
    if any(_to_float(row.get("plant_velocity")) is not None for row in rows):
        analysis["plant_velocity_to_measured_velocity"] = estimate_lag_scale(
            rows,
            "plant_velocity",
            "measured_vel_sps",
            max_lag_s=1.0,
            min_abs_source=0.02,
            demean=True,
        )
    return analysis


def summarize_rows(rows: list[dict[str, Any]], metadata: dict[str, Any] | None = None) -> dict[str, Any]:
    metadata = dict(metadata or {})
    summary: dict[str, Any] = {
        "run_id": metadata.get("run_id", "run"),
        "scenario": metadata,
        "sample_count": len(rows),
        "telemetry_continuous": False,
        "fell": False,
    }
    if not rows:
        summary.update(
            {
                "duration_s": 0.0,
                "final_pitch_deg": None,
                "max_abs_pitch_deg": None,
                "max_abs_u_sps": None,
            }
        )
        return summary

    pitch_key = _pitch_key(rows)
    pitch_values = _series(rows, pitch_key)
    time_values = _series(rows, "sim_time_s")
    u_values = _series(rows, "u_sps")

    summary["duration_s"] = max(time_values) - min(time_values) if len(time_values) >= 2 else 0.0
    summary["final_pitch_deg"] = pitch_values[-1] if pitch_values else None
    summary["max_abs_pitch_deg"] = max(abs(v) for v in pitch_values) if pitch_values else None
    summary["max_abs_u_sps"] = max(abs(v) for v in u_values) if u_values else None
    summary["fell"] = bool(summary["max_abs_pitch_deg"] is not None and summary["max_abs_pitch_deg"] > 75.0)

    plant_position_values = _series(rows, "plant_position")
    plant_velocity_values = _series(rows, "plant_velocity")
    summary["final_position_m"] = plant_position_values[-1] if plant_position_values else None
    summary["max_abs_position_m"] = (
        max(abs(v) for v in plant_position_values) if plant_position_values else None
    )
    target_wheel_velocity_values = _series(rows, "target_wheel_velocity")
    actual_wheel_velocity_values = _series(rows, "actual_wheel_velocity")
    summary["max_abs_target_wheel_velocity"] = (
        max(abs(v) for v in target_wheel_velocity_values) if target_wheel_velocity_values else None
    )
    summary["max_abs_actual_wheel_velocity"] = (
        max(abs(v) for v in actual_wheel_velocity_values) if actual_wheel_velocity_values else None
    )
    f_app_values = _series(rows, "f_app")
    summary["max_abs_f_app"] = max(abs(v) for v in f_app_values) if f_app_values else None
    theta_ddot_values = _series(rows, "theta_ddot")
    summary["max_abs_theta_ddot"] = (
        max(abs(v) for v in theta_ddot_values) if theta_ddot_values else None
    )

    if time_values and pitch_values:
        tail_end = time_values[-1]
        tail_start = tail_end - 2.0
        tail_rows = [row for row in rows if (_to_float(row.get("sim_time_s")) or 0.0) >= tail_start]
        tail_pitch = _series(tail_rows, pitch_key)
        if tail_pitch:
            summary["tail_rms_pitch_deg"] = math.sqrt(sum(v * v for v in tail_pitch) / len(tail_pitch))
            summary["tail_mean_abs_pitch_deg"] = sum(abs(v) for v in tail_pitch) / len(tail_pitch)
        else:
            summary["tail_rms_pitch_deg"] = None
            summary["tail_mean_abs_pitch_deg"] = None

        tail_velocity = _series(tail_rows, "plant_velocity")
        if tail_velocity:
            summary["tail_mean_abs_velocity_mps"] = sum(abs(v) for v in tail_velocity) / len(tail_velocity)
        else:
            summary["tail_mean_abs_velocity_mps"] = None

        tail_command_sat = _series(tail_rows, "command_saturated")
        if tail_command_sat:
            summary["tail_command_rail_fraction"] = (
                sum(1.0 for v in tail_command_sat if v >= 0.5) / len(tail_command_sat)
            )
        else:
            summary["tail_command_rail_fraction"] = None

        tail_force_sat = _series(tail_rows, "force_saturated")
        if tail_force_sat:
            summary["tail_rail_fraction"] = (
                sum(1.0 for v in tail_force_sat if v >= 0.5) / len(tail_force_sat)
            )
        else:
            summary["tail_rail_fraction"] = summary["tail_command_rail_fraction"]

        running_max_abs: list[float] = [0.0] * len(pitch_values)
        acc = 0.0
        for idx in range(len(pitch_values) - 1, -1, -1):
            acc = max(acc, abs(pitch_values[idx]))
            running_max_abs[idx] = acc
        settled_at_s = None
        for idx, max_abs in enumerate(running_max_abs):
            if max_abs <= 3.0:
                settled_at_s = time_values[idx]
                break
        summary["settled_at_s"] = settled_at_s

    if len(time_values) >= 2:
        deltas = [b - a for a, b in zip(time_values, time_values[1:])]
        positive = [dt for dt in deltas if dt > 0.0]
        monotonic = len(positive) == len(deltas)
        if positive:
            med = median(positive)
            max_gap = max(positive)
            summary["dt_median_s"] = med
            summary["dt_max_s"] = max_gap
            summary["telemetry_continuous"] = monotonic and max_gap <= max(5.0 * med, 1e-9)
        else:
            summary["dt_median_s"] = None
            summary["dt_max_s"] = None
            summary["telemetry_continuous"] = False
    else:
        summary["telemetry_continuous"] = True
        summary["dt_median_s"] = None
        summary["dt_max_s"] = None

    for key in ("pitch_deg", "pitch_rate_dps", "u_sps", "vel_error", "rate_sp_dps", "f_cmd", "f_app"):
        vals = _series(rows, key)
        if vals:
            summary[f"{key}_min"] = min(vals)
            summary[f"{key}_max"] = max(vals)

    return summary


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    columns = list(_DEFAULT_COLUMNS)
    for row in rows:
        for key in row:
            if key not in columns:
                columns.append(key)

    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=columns)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def _format_tick(value: float) -> str:
    if abs(value) >= 100.0:
        return f"{value:.0f}"
    if abs(value) >= 10.0:
        return f"{value:.1f}"
    return f"{value:.2f}"


def _data_range(values: list[float], *, center_zero: bool = False) -> tuple[float, float]:
    if not values:
        return (-1.0, 1.0) if center_zero else (0.0, 1.0)
    if center_zero:
        max_abs = max(abs(value) for value in values)
        if max_abs <= 1e-9:
            max_abs = 1.0
        max_abs *= 1.05
        return -max_abs, max_abs

    value_min = min(values)
    value_max = max(values)
    if value_max <= value_min:
        pad = max(abs(value_min) * 0.05, 1.0)
        return value_min - pad, value_max + pad

    pad = (value_max - value_min) * 0.05
    return value_min - pad, value_max + pad


def _scale_points(
    xs: list[float],
    ys: list[float],
    *,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
) -> list[tuple[float, float]]:
    if not xs or not ys:
        return []
    if x_max <= x_min:
        x_max = x_min + 1.0
    if y_max <= y_min:
        y_max = y_min + 1.0
    points: list[tuple[float, float]] = []
    for x, y in zip(xs, ys):
        px = _PLOT_LEFT + ((x - x_min) / (x_max - x_min)) * _PLOT_WIDTH
        py = _PLOT_TOP + (1.0 - ((y - y_min) / (y_max - y_min))) * _PLOT_HEIGHT
        points.append((px, py))
    return points


def _downsample(xs: list[float], ys: list[float], max_points: int = _MAX_PLOT_POINTS) -> tuple[list[float], list[float]]:
    if len(xs) <= max_points or len(ys) <= max_points:
        return xs, ys
    stride = max(1, len(xs) // max_points)
    xs_ds = xs[::stride]
    ys_ds = ys[::stride]
    if xs_ds[-1] != xs[-1] or ys_ds[-1] != ys[-1]:
        xs_ds.append(xs[-1])
        ys_ds.append(ys[-1])
    return xs_ds, ys_ds


def _polyline(points: list[tuple[float, float]], color: str) -> str:
    if not points:
        return ""
    pts = " ".join(f"{x:.2f},{y:.2f}" for x, y in points)
    return f'<polyline fill="none" stroke="{color}" stroke-width="2" points="{pts}" />'


def _write_svg_plot(
    path: Path,
    rows: list[dict[str, Any]],
    title: str,
    series: list[tuple[str, str, str]],
    *,
    y_label: str,
    x_label: str = "Time (s)",
    center_zero: bool = False,
) -> None:
    width = 1000
    height = 320
    xs = _series(rows, "sim_time_s")
    series_values = []
    for key, _color, _label in series:
        series_values.extend(_series(rows, key))
    if xs:
        x_min = min(xs)
        x_max = max(xs)
    else:
        x_min, x_max = 0.0, 1.0
    y_min, y_max = _data_range(series_values, center_zero=center_zero)

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">',
        '<rect width="100%" height="100%" fill="#ffffff" />',
        f'<text x="{_PLOT_LEFT}" y="18" font-family="monospace" font-size="16">{title}</text>',
    ]

    x_ticks = 6
    for idx in range(x_ticks):
        t = idx / (x_ticks - 1)
        x_value = x_min + (x_max - x_min) * t
        px = _PLOT_LEFT + t * _PLOT_WIDTH
        parts.append(
            f'<line x1="{px:.2f}" y1="{_PLOT_TOP}" x2="{px:.2f}" y2="{_PLOT_BOTTOM}" '
            'stroke="#E5E7EB" stroke-width="1"/>'
        )
        parts.append(
            f'<text x="{px:.2f}" y="{_PLOT_BOTTOM + 18}" text-anchor="middle" '
            f'font-family="monospace" font-size="11">{_format_tick(x_value)}</text>'
        )

    y_ticks = 5
    for idx in range(y_ticks):
        t = idx / (y_ticks - 1)
        y_value = y_max - (y_max - y_min) * t
        py = _PLOT_TOP + t * _PLOT_HEIGHT
        parts.append(
            f'<line x1="{_PLOT_LEFT}" y1="{py:.2f}" x2="{_PLOT_RIGHT}" y2="{py:.2f}" '
            'stroke="#E5E7EB" stroke-width="1"/>'
        )
        parts.append(
            f'<text x="{_PLOT_LEFT - 10}" y="{py + 4:.2f}" text-anchor="end" '
            f'font-family="monospace" font-size="11">{_format_tick(y_value)}</text>'
        )

    if y_min < 0.0 < y_max:
        zero_y = _PLOT_TOP + (1.0 - ((0.0 - y_min) / (y_max - y_min))) * _PLOT_HEIGHT
        parts.append(
            f'<line x1="{_PLOT_LEFT}" y1="{zero_y:.2f}" x2="{_PLOT_RIGHT}" y2="{zero_y:.2f}" '
            'stroke="#9CA3AF" stroke-width="1.5" stroke-dasharray="4 3"/>'
        )

    parts.extend(
        [
            f'<rect x="{_PLOT_LEFT}" y="{_PLOT_TOP}" width="{_PLOT_WIDTH}" height="{_PLOT_HEIGHT}" '
            'fill="none" stroke="#111827" stroke-width="1"/>',
            f'<text x="{(_PLOT_LEFT + _PLOT_RIGHT) / 2:.2f}" y="{height - 18}" text-anchor="middle" '
            f'font-family="monospace" font-size="12">{x_label}</text>',
            (
                f'<text x="18" y="{(_PLOT_TOP + _PLOT_BOTTOM) / 2:.2f}" text-anchor="middle" '
                f'font-family="monospace" font-size="12" transform="rotate(-90 18 {(_PLOT_TOP + _PLOT_BOTTOM) / 2:.2f})">'
                f"{y_label}</text>"
            ),
        ]
    )

    legend_y = 285
    legend_x = _PLOT_LEFT
    for idx, (key, color, label) in enumerate(series):
        ys = _series(rows, key)
        if len(xs) != len(ys) or not ys:
            continue
        xs_plot, ys_plot = _downsample(xs, ys)
        parts.append(
            _polyline(
                _scale_points(
                    xs_plot,
                    ys_plot,
                    x_min=x_min,
                    x_max=x_max,
                    y_min=y_min,
                    y_max=y_max,
                ),
                color,
            )
        )
        lx = legend_x + idx * 180
        parts.append(f'<line x1="{lx}" y1="{legend_y}" x2="{lx + 20}" y2="{legend_y}" stroke="{color}" stroke-width="2"/>')
        parts.append(f'<text x="{lx + 26}" y="{legend_y + 5}" font-family="monospace" font-size="12">{label}</text>')
    parts.append("</svg>")
    path.write_text("\n".join(parts), encoding="utf-8")


def write_summary_json(path: Path, summary: dict[str, Any]) -> None:
    path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_metadata_json(path: Path, metadata: dict[str, Any]) -> None:
    path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def parse_reference_file(path: str | Path) -> list[dict[str, float]]:
    path = Path(path)
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()

    for idx, line in enumerate(lines):
        stripped = line.strip()
        if not stripped or "=" in stripped or "," not in stripped:
            continue
        header = [part.strip() for part in stripped.split(",")]
        if all(header):
            rows: list[dict[str, float]] = []
            for line in lines[idx + 1 :]:
                pieces = [part.strip() for part in line.split(",")]
                if len(pieces) != len(header):
                    continue
                try:
                    rows.append({key: float(value) for key, value in zip(header, pieces)})
                except ValueError:
                    continue
            if rows:
                return rows

    rows = []
    for line in lines:
        matches = _KV_RE.findall(line)
        if not matches:
            continue
        rows.append({key: float(value) for key, value in matches})
    return rows


def summarize_reference_file(path: str | Path) -> dict[str, Any]:
    rows = parse_reference_file(path)
    summary: dict[str, Any] = {"path": str(path), "sample_count": len(rows)}
    if not rows:
        return summary
    keys = sorted({key for row in rows for key in row})
    summary["fields"] = keys
    for candidate in ("time", "time_s", "sim_time_s"):
        values = _series(rows, candidate)
        if len(values) >= 2:
            deltas = [b - a for a, b in zip(values, values[1:]) if b > a]
            summary["time_field"] = candidate
            summary["dt_min"] = min(deltas) if deltas else None
            summary["dt_max"] = max(deltas) if deltas else None
            summary["dt_mean"] = sum(deltas) / len(deltas) if deltas else None
            break
    for key in keys:
        vals = _series(rows, key)
        if vals:
            summary[f"{key}_min"] = min(vals)
            summary[f"{key}_max"] = max(vals)
    return summary


def preserve_artifacts(output_dir: Path, preserve_root: Path | None, run_id: str) -> Path | None:
    if preserve_root is None:
        return None
    dest = preserve_root / run_id
    if dest.exists():
        shutil.rmtree(dest)
    shutil.copytree(output_dir, dest)
    return dest


@dataclass
class RunRecorder:
    metadata: dict[str, Any] = field(default_factory=dict)
    rows: list[dict[str, Any]] = field(default_factory=list)

    def begin_run(self, run_metadata: dict[str, Any]) -> None:
        self.metadata = dict(run_metadata)
        self.rows = []

    def record_step(self, sample: dict[str, Any]) -> None:
        self.rows.append(dict(sample))

    def finalize(self) -> dict[str, Any]:
        return summarize_rows(self.rows, self.metadata)

    def write_csv_json_plots(self, output_dir: str | Path) -> dict[str, Any]:
        output = Path(output_dir)
        output.mkdir(parents=True, exist_ok=True)
        _write_csv(output / "timeline.csv", self.rows)
        write_metadata_json(output / "metadata.json", self.metadata)
        summary = self.finalize()
        write_summary_json(output / "summary.json", summary)
        _write_svg_plot(output / "pitch_plot.svg", self.rows, "Pitch Timeline", [
            ("pitch_deg", "#2563EB", "Telemetry pitch (deg)"),
            ("plant_pitch_deg", "#DC2626", "Plant pitch (deg)"),
            ("pitch_sp_deg", "#059669", "Pitch setpoint (deg)"),
        ], y_label="Pitch (deg)", center_zero=True)
        _write_svg_plot(output / "command_plot.svg", self.rows, "Command Timeline", [
            ("u_sps", "#7C3AED", "Pitch command (steps/s)"),
            ("left_sps", "#EA580C", "Left command (steps/s)"),
            ("right_sps", "#0891B2", "Right command (steps/s)"),
        ], y_label="Command (steps/s)")
        _write_svg_plot(output / "wheel_plot.svg", self.rows, "Wheel Velocity Timeline", [
            ("target_wheel_velocity", "#2563EB", "Target wheel velocity (m/s)"),
            ("actual_wheel_velocity", "#DC2626", "Actual wheel velocity (m/s)"),
            ("plant_velocity", "#059669", "Plant velocity (m/s)"),
        ], y_label="Velocity (m/s)", center_zero=True)
        _write_svg_plot(output / "force_plot.svg", self.rows, "Plant Force Timeline", [
            ("f_cmd", "#7C3AED", "Commanded force (N)"),
            ("f_app", "#EA580C", "Applied force (N)"),
            ("theta_ddot", "#0891B2", "Theta accel (rad/s^2)"),
        ], y_label="Force / Accel", center_zero=True)
        return summary


def load_csv_rows(path: Path) -> list[dict[str, Any]]:
    with path.open(newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        return [dict(row) for row in reader]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True)
    parser.add_argument("--metadata", required=False)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()

    metadata: dict[str, Any] = {}
    if args.metadata:
        metadata = json.loads(Path(args.metadata).read_text(encoding="utf-8"))

    recorder = RunRecorder(metadata=metadata, rows=load_csv_rows(Path(args.csv)))
    recorder.write_csv_json_plots(args.output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
