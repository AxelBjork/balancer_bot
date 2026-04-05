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
    "pitch_sp_deg",
    "rate_sp_dps",
    "u_sps",
    "left_sps",
    "right_sps",
    "vel_error",
    "vel_i_term",
    "vel_p_term",
    "out_norm",
    "plant_pitch_deg",
    "plant_pitch_rate_dps",
    "plant_position",
    "plant_velocity",
]

_KV_RE = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)=([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)")


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

    for key in ("pitch_deg", "pitch_rate_dps", "u_sps", "vel_error", "rate_sp_dps"):
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


def _scale_points(xs: list[float], ys: list[float], width: int, height: int) -> list[tuple[float, float]]:
    if not xs or not ys:
        return []
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    if x_max <= x_min:
        x_max = x_min + 1.0
    if y_max <= y_min:
        y_max = y_min + 1.0
    points: list[tuple[float, float]] = []
    for x, y in zip(xs, ys):
        px = 50 + ((x - x_min) / (x_max - x_min)) * (width - 100)
        py = 20 + (1.0 - ((y - y_min) / (y_max - y_min))) * (height - 60)
        points.append((px, py))
    return points


def _polyline(points: list[tuple[float, float]], color: str) -> str:
    if not points:
        return ""
    pts = " ".join(f"{x:.2f},{y:.2f}" for x, y in points)
    return f'<polyline fill="none" stroke="{color}" stroke-width="2" points="{pts}" />'


def _write_svg_plot(
    path: Path,
    rows: list[dict[str, Any]],
    title: str,
    series: list[tuple[str, str]],
) -> None:
    width = 1000
    height = 320
    xs = _series(rows, "sim_time_s")
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">',
        '<rect width="100%" height="100%" fill="#ffffff" />',
        '<line x1="50" y1="20" x2="50" y2="260" stroke="#111827" stroke-width="1"/>',
        '<line x1="50" y1="260" x2="950" y2="260" stroke="#111827" stroke-width="1"/>',
        f'<text x="55" y="18" font-family="monospace" font-size="16">{title}</text>',
    ]
    legend_y = 285
    legend_x = 55
    for idx, (key, color) in enumerate(series):
        ys = _series(rows, key)
        if len(xs) != len(ys) or not ys:
            continue
        parts.append(_polyline(_scale_points(xs, ys, width, height), color))
        lx = legend_x + idx * 180
        parts.append(f'<line x1="{lx}" y1="{legend_y}" x2="{lx + 20}" y2="{legend_y}" stroke="{color}" stroke-width="2"/>')
        parts.append(f'<text x="{lx + 26}" y="{legend_y + 5}" font-family="monospace" font-size="12">{key}</text>')
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
            ("pitch_deg", "#2563EB"),
            ("plant_pitch_deg", "#DC2626"),
            ("pitch_sp_deg", "#059669"),
        ])
        _write_svg_plot(output / "command_plot.svg", self.rows, "Command Timeline", [
            ("u_sps", "#7C3AED"),
            ("left_sps", "#EA580C"),
            ("right_sps", "#0891B2"),
        ])
        return summary


def _load_csv_rows(path: Path) -> list[dict[str, Any]]:
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

    recorder = RunRecorder(metadata=metadata, rows=_load_csv_rows(Path(args.csv)))
    recorder.write_csv_json_plots(args.output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
