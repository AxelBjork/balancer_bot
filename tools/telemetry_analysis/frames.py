from __future__ import annotations

from pathlib import Path
from typing import Any, Iterable, Mapping

import pandas as pd


# New captures use the reflected wire names. These aliases keep older simulator
# artifacts and small hand-written fixtures readable without perpetuating two schemas.
TELEMETRY_ALIASES = {
    "t_sec": ("sim_time_s", "time_s", "time"),
    "left_target_sps": ("left_sps",),
    "right_target_sps": ("right_sps",),
    "vel_error": ("velocity_error_sps",),
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
