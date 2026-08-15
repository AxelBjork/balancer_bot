from __future__ import annotations

import math
from typing import Any

import numpy as np
import pandas as pd

from .frames import canonicalize_telemetry_frame


ACTUATOR_SATURATION_LEFT_SLEW = 1 << 0
ACTUATOR_SATURATION_RIGHT_SLEW = 1 << 1


def _rms_error(frame: pd.DataFrame, source: str, response: str) -> float | None:
    if source not in frame.columns or response not in frame.columns:
        return None
    pairs = frame[[source, response]].replace([np.inf, -np.inf], np.nan).dropna()
    if pairs.empty:
        return None
    error = pairs[response].to_numpy(dtype=float) - pairs[source].to_numpy(dtype=float)
    return float(np.sqrt(np.mean(error * error)))


def _combined_rms_error(
    frame: pd.DataFrame, source_columns: tuple[str, str], response_columns: tuple[str, str]
) -> float | None:
    columns = [*source_columns, *response_columns]
    if any(column not in frame.columns for column in columns):
        return None
    pairs = frame[columns].replace([np.inf, -np.inf], np.nan).dropna()
    if pairs.empty:
        return None
    left_error = (
        pairs[response_columns[0]].to_numpy(dtype=float)
        - pairs[source_columns[0]].to_numpy(dtype=float)
    )
    right_error = (
        pairs[response_columns[1]].to_numpy(dtype=float)
        - pairs[source_columns[1]].to_numpy(dtype=float)
    )
    return float(np.sqrt(np.mean(np.concatenate((left_error, right_error)) ** 2)))


def _slope_stats(frame: pd.DataFrame, signal: str) -> dict[str, float | int | None]:
    result: dict[str, float | int | None] = {
        "sample_count": 0,
        "rms": None,
        "p95_abs": None,
        "max_abs": None,
    }
    if "t_sec" not in frame.columns or signal not in frame.columns:
        return result
    pairs = frame[["t_sec", signal]].replace([np.inf, -np.inf], np.nan).dropna()
    if len(pairs) < 2:
        return result
    time = pairs["t_sec"].to_numpy(dtype=float)
    values = pairs[signal].to_numpy(dtype=float)
    dt = np.diff(time)
    positive = dt[dt > 0.0]
    if positive.size == 0:
        return result
    median_dt = float(np.median(positive))
    valid = (dt > 0.0) & (dt <= max(5.0 * median_dt, 0.25))
    slopes = np.diff(values)[valid] / dt[valid]
    if slopes.size == 0:
        return result
    result.update(
        {
            "sample_count": int(slopes.size),
            "rms": float(np.sqrt(np.mean(slopes * slopes))),
            "p95_abs": float(np.quantile(np.abs(slopes), 0.95)),
            "max_abs": float(np.max(np.abs(slopes))),
        }
    )
    return result


def _longest_true_interval_s(time: np.ndarray, mask: np.ndarray) -> float | None:
    if time.size < 2 or mask.size != time.size:
        return None
    positive_dt = np.diff(time)
    positive_dt = positive_dt[np.isfinite(positive_dt) & (positive_dt > 0.0)]
    if positive_dt.size == 0:
        return None
    median_dt = float(np.median(positive_dt))
    # A row represents the following sample interval. Cap that interval at the
    # normal cadence so a capture gap is never counted as continuous limiting.
    dwell = np.diff(time, append=time[-1] + median_dt)
    dwell = np.where(
        np.isfinite(dwell) & (dwell > 0.0),
        np.minimum(dwell, median_dt),
        0.0,
    )
    longest = 0.0
    current = 0.0
    for selected, duration in zip(mask, dwell):
        if selected:
            current += float(duration)
            longest = max(longest, current)
        else:
            current = 0.0
    return longest


def _longest_contiguous_slice(time: np.ndarray) -> slice | None:
    if time.size < 2:
        return None
    dt = np.diff(time)
    positive = dt[np.isfinite(dt) & (dt > 0.0)]
    if positive.size == 0:
        return None
    median_dt = float(np.median(positive))
    breaks = np.flatnonzero((dt <= 0.0) | (dt > max(5.0 * median_dt, 0.25)))
    starts = np.r_[0, breaks + 1]
    stops = np.r_[breaks + 1, time.size]
    lengths = stops - starts
    best = int(np.argmax(lengths))
    return slice(int(starts[best]), int(stops[best]))


def band_frequency_response(
    frame: pd.DataFrame,
    source: str,
    response: str,
    *,
    low_hz: float = 6.0,
    high_hz: float = 12.0,
) -> dict[str, Any]:
    result: dict[str, Any] = {
        "source": source,
        "response": response,
        "band_hz": [low_hz, high_hz],
        "sample_count": 0,
        "frequency_hz": None,
        "magnitude_ratio": None,
        "phase_lag_deg": None,
        "lag_s": None,
    }
    columns = ["t_sec", source, response]
    missing = [column for column in columns if column not in frame.columns]
    if missing:
        result["unavailable_reason"] = f"missing columns: {', '.join(missing)}"
        return result

    numeric = frame[columns].replace([np.inf, -np.inf], np.nan).dropna()
    numeric = numeric.sort_values("t_sec").drop_duplicates("t_sec")
    if len(numeric) < 32:
        result["unavailable_reason"] = "fewer than 32 aligned finite samples"
        return result

    time = numeric["t_sec"].to_numpy(dtype=float)
    source_values = numeric[source].to_numpy(dtype=float)
    response_values = numeric[response].to_numpy(dtype=float)
    contiguous = _longest_contiguous_slice(time)
    if contiguous is None:
        result["unavailable_reason"] = "no positive analysis cadence"
        return result
    time = time[contiguous]
    source_values = source_values[contiguous]
    response_values = response_values[contiguous]
    if time.size < 32:
        result["unavailable_reason"] = "longest contiguous segment has fewer than 32 samples"
        return result

    median_dt = float(np.median(np.diff(time)))
    uniform_time = np.arange(time[0], time[-1] + 0.5 * median_dt, median_dt)
    if uniform_time.size < 32:
        result["unavailable_reason"] = "uniform segment has fewer than 32 samples"
        return result
    uniform_source = np.interp(uniform_time, time, source_values)
    uniform_response = np.interp(uniform_time, time, response_values)
    index = np.arange(uniform_time.size, dtype=float)
    uniform_source -= np.polyval(np.polyfit(index, uniform_source, 1), index)
    uniform_response -= np.polyval(np.polyfit(index, uniform_response, 1), index)
    window = np.hanning(uniform_time.size)
    source_fft = np.fft.rfft(uniform_source * window)
    response_fft = np.fft.rfft(uniform_response * window)
    frequencies = np.fft.rfftfreq(uniform_time.size, d=median_dt)
    band = (frequencies >= low_hz) & (frequencies <= high_hz)
    if not np.any(band):
        result["unavailable_reason"] = "selected duration has no FFT bins in the requested band"
        return result

    band_indices = np.flatnonzero(band)
    dominant_index = int(band_indices[np.argmax(np.abs(source_fft[band]))])
    source_magnitude = float(np.abs(source_fft[dominant_index]))
    if source_magnitude <= 1e-9:
        result["unavailable_reason"] = "source has no measurable energy in the requested band"
        return result
    frequency_hz = float(frequencies[dominant_index])
    transfer = response_fft[dominant_index] / source_fft[dominant_index]
    phase_lag_deg = -math.degrees(float(np.angle(transfer)))
    phase_lag_deg = (phase_lag_deg + 180.0) % 360.0 - 180.0
    result.update(
        {
            "sample_count": int(uniform_time.size),
            "sample_rate_hz": 1.0 / median_dt,
            "frequency_hz": frequency_hz,
            "magnitude_ratio": float(abs(transfer)),
            "phase_lag_deg": phase_lag_deg,
            "lag_s": phase_lag_deg / (360.0 * frequency_hz),
        }
    )
    return result


def actuator_stage_metrics(
    frame: pd.DataFrame,
    *,
    low_hz: float = 6.0,
    high_hz: float = 12.0,
) -> dict[str, Any]:
    """Summarize requested, post-slew, and pulse-applied actuator stages."""
    data = canonicalize_telemetry_frame(frame)
    result: dict[str, Any] = {
        "sample_count": int(len(data)),
        "slew_limited_fraction": None,
        "left_slew_limited_fraction": None,
        "right_slew_limited_fraction": None,
        "longest_slew_interval_s": None,
        "command_slope_sps2": {},
        "stage_error_rms_sps": {},
        "frequency_response_6_12_hz": {},
    }

    if "actuator_saturation_flags" in data.columns:
        finite_flags = data["actuator_saturation_flags"].replace([np.inf, -np.inf], np.nan).dropna()
        if not finite_flags.empty:
            flags = finite_flags.astype("uint32").to_numpy()
            result["slew_limited_fraction"] = float(np.mean(flags != 0))
            result["left_slew_limited_fraction"] = float(
                np.mean((flags & ACTUATOR_SATURATION_LEFT_SLEW) != 0)
            )
            result["right_slew_limited_fraction"] = float(
                np.mean((flags & ACTUATOR_SATURATION_RIGHT_SLEW) != 0)
            )
            if "t_sec" in data.columns:
                flag_rows = data.loc[finite_flags.index, ["t_sec"]].copy()
                flag_rows["flags"] = flags
                flag_rows = flag_rows.replace([np.inf, -np.inf], np.nan).dropna()
                result["longest_slew_interval_s"] = _longest_true_interval_s(
                    flag_rows["t_sec"].to_numpy(dtype=float),
                    flag_rows["flags"].to_numpy(dtype="uint32") != 0,
                )

    stages = {
        "requested_to_slewed": (
            ("left_target_sps", "right_target_sps"),
            ("left_slewed_sps", "right_slewed_sps"),
        ),
        "slewed_to_applied": (
            ("left_slewed_sps", "right_slewed_sps"),
            ("left_applied_sps", "right_applied_sps"),
        ),
        "requested_to_applied": (
            ("left_target_sps", "right_target_sps"),
            ("left_applied_sps", "right_applied_sps"),
        ),
    }
    for stage_name, (source_columns, response_columns) in stages.items():
        result["stage_error_rms_sps"][stage_name] = {
            "left": _rms_error(data, source_columns[0], response_columns[0]),
            "right": _rms_error(data, source_columns[1], response_columns[1]),
            "combined": _combined_rms_error(data, source_columns, response_columns),
        }

    for stage in ("target", "slewed", "applied"):
        for side in ("left", "right"):
            signal = f"{side}_{stage}_sps"
            result["command_slope_sps2"][signal] = _slope_stats(data, signal)

    for side in ("left", "right"):
        result["frequency_response_6_12_hz"][f"{side}_requested_to_applied"] = (
            band_frequency_response(
                data,
                f"{side}_target_sps",
                f"{side}_applied_sps",
                low_hz=low_hz,
                high_hz=high_hz,
            )
        )
    return result
