from __future__ import annotations

from pathlib import Path
from typing import Any, Iterable, Mapping

import numpy as np
import pandas as pd


# New captures use the reflected wire names. These aliases keep older simulator
# artifacts and small hand-written fixtures readable without perpetuating two schemas.
TELEMETRY_ALIASES = {
    "t_sec": ("sim_time_s", "time_s", "time"),
    "left_target_sps": ("left_sps",),
    "right_target_sps": ("right_sps",),
    "vel_error": ("velocity_error_sps",),
    # v4 wire names retain old captures' original semantics as aliases.
    "nominal_acceleration_mps2": ("target_velocity_sps",),
    "raw_completed_velocity_sps": ("vel_error",),
    "corrected_axle_velocity_sps": ("measured_vel_sps",),
    "velocity_damping_acceleration_mps2": ("velocity_p_term_deg",),
    "com_trim_deg": ("velocity_i_term_deg",),
    "plant_position_m": ("plant_position",),
    "plant_velocity_mps": ("plant_velocity",),
    "plant_velocity_error": ("velocity_error",),
}


def canonicalize_telemetry_frame(frame: pd.DataFrame) -> pd.DataFrame:
    """Return a numeric telemetry frame using reflected wire field names."""
    normalized = frame.copy()
    for canonical, aliases in TELEMETRY_ALIASES.items():
        if canonical in normalized.columns:
            continue
        for alias in aliases:
            if alias in normalized.columns:
                normalized[canonical] = normalized[alias]
                break

    legacy_columns = {
        alias
        for canonical, aliases in TELEMETRY_ALIASES.items()
        if canonical in normalized.columns
        for alias in aliases
        if alias in normalized.columns
    }
    normalized = normalized.drop(columns=sorted(legacy_columns))
    return normalized.apply(pd.to_numeric, errors="coerce")


def telemetry_frame(rows: Iterable[Mapping[str, Any]]) -> pd.DataFrame:
    return canonicalize_telemetry_frame(pd.DataFrame.from_records(rows))


def read_telemetry_csv(path: str | Path) -> pd.DataFrame:
    try:
        frame = pd.read_csv(path)
    except pd.errors.EmptyDataError:
        frame = pd.DataFrame()
    return canonicalize_telemetry_frame(frame)


def write_telemetry_csv(path: str | Path, rows: Iterable[Mapping[str, Any]]) -> None:
    telemetry_frame(rows).to_csv(path, index=False)


def write_telemetry_frame(path: str | Path, frame: pd.DataFrame) -> None:
    """Write an already canonicalized telemetry frame without rebuilding it."""
    frame.to_csv(path, index=False)


def band_rms_equivalent(
    frame: pd.DataFrame,
    signal: str,
    low_hz: float,
    high_hz: float,
    *,
    time_column: str = "t_sec",
) -> dict[str, float | int | None]:
    """Return Hann-windowed single-sided RMS-equivalent energy for a frequency band.

    The input must be a finite, uniformly sampled contiguous timeline.  A
    ``None`` RMS reports that the selected signal cannot support a spectrum;
    callers must not substitute a plausible zero for missing telemetry.
    """
    result: dict[str, float | int | None] = {
        "rms": None,
        "sample_rate_hz": None,
        "sample_count": 0,
        "low_hz": low_hz,
        "high_hz": high_hz,
    }
    if signal not in frame or time_column not in frame or low_hz < 0.0 or high_hz <= low_hz:
        return result

    times = pd.to_numeric(frame[time_column], errors="coerce").to_numpy(dtype=float)
    values = pd.to_numeric(frame[signal], errors="coerce").to_numpy(dtype=float)
    finite = np.isfinite(times) & np.isfinite(values)
    if not finite.all() or values.size < 4:
        return result

    deltas = np.diff(times)
    if np.any(deltas <= 0.0):
        return result
    dt_s = float(np.median(deltas))
    if dt_s <= 0.0 or not np.allclose(deltas, dt_s, rtol=1e-3, atol=1e-9):
        return result

    sample_rate_hz = 1.0 / dt_s
    result["sample_rate_hz"] = sample_rate_hz
    result["sample_count"] = int(values.size)
    if high_hz > sample_rate_hz / 2.0:
        return result

    detrended = values - np.mean(values)
    window = np.hanning(values.size)
    window_energy = float(np.sum(np.square(window)))
    if window_energy == 0.0:
        return result
    spectrum = np.fft.rfft(detrended * window)
    frequencies = np.fft.rfftfreq(values.size, d=dt_s)
    in_band = (frequencies >= low_hz) & (frequencies <= high_hz)
    if not np.any(in_band):
        return result

    one_sided_weight = np.full(spectrum.size, 2.0)
    one_sided_weight[0] = 1.0
    if values.size % 2 == 0:
        one_sided_weight[-1] = 1.0
    band_power = np.sum(one_sided_weight[in_band] * np.square(np.abs(spectrum[in_band])))
    result["rms"] = float(np.sqrt(band_power / (values.size * window_energy)))
    return result
