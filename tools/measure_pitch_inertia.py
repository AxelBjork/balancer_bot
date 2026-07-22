#!/usr/bin/env python3
"""Estimate wheel-axle pitch inertia from a passive-pendulum telemetry capture."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.telemetry_analysis.frames import read_telemetry_csv


def rising_zero_crossings(time_s: np.ndarray, signal: np.ndarray) -> np.ndarray:
    crossings: list[float] = []
    for index in range(len(signal) - 1):
        left, right = signal[index], signal[index + 1]
        if left <= 0.0 < right:
            fraction = -left / (right - left)
            crossings.append(float(time_s[index] + fraction * (time_s[index + 1] - time_s[index])))
    return np.asarray(crossings)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Estimate axle pitch inertia J = g H (P / 2π)^2 from passive IMU telemetry."
    )
    parser.add_argument("csv", type=Path, help="Dashboard/server telemetry CSV; input is never modified.")
    parser.add_argument("--start-s", type=float, help="Optional controller-time selection start.")
    parser.add_argument("--end-s", type=float, help="Optional controller-time selection end.")
    parser.add_argument("--session", type=int, help="Session number after splitting controller-time resets.")
    parser.add_argument("--first-mass-moment-kg-m", type=float, default=0.06192)
    parser.add_argument("--gravity-mps2", type=float, default=9.81)
    parser.add_argument("--log-decrement", type=float,
                        help="Measured natural-log amplitude decrement per period for damped correction.")
    args = parser.parse_args()

    frame = read_telemetry_csv(args.csv)
    if "pitch_deg" not in frame:
        raise SystemExit("CSV must contain pitch_deg telemetry.")
    time_column = "t_sec" if "t_sec" in frame and frame["t_sec"].notna().any() else "received_at_monotonic_s"
    if time_column not in frame:
        raise SystemExit("CSV must contain t_sec or received_at_monotonic_s for period measurement.")

    selected = frame[[time_column, "pitch_deg"]].dropna().copy()
    selected["session"] = (selected[time_column].diff().fillna(0.0) < 0.0).cumsum()
    session_count = int(selected["session"].max()) + 1
    if args.session is None and session_count > 1:
        raise SystemExit(f"CSV contains {session_count} sessions; select one with --session.")
    requested_session = 0 if args.session is None else args.session
    selected = selected[selected["session"] == requested_session].sort_values(time_column)
    selected = selected.drop_duplicates(time_column)
    if args.start_s is not None:
        selected = selected[selected[time_column] >= args.start_s]
    if args.end_s is not None:
        selected = selected[selected[time_column] <= args.end_s]
    if len(selected) < 8:
        raise SystemExit("Selection has too few valid pitch samples.")

    time_s = selected[time_column].to_numpy(dtype=float)
    # The hanging-down equilibrium may be reported near +/-180 degrees; unwrap before centering.
    pitch_rad = np.unwrap(np.deg2rad(selected["pitch_deg"].to_numpy(dtype=float)))
    centered = pitch_rad - np.mean(pitch_rad)
    crossings = rising_zero_crossings(time_s, centered)
    periods_s = np.diff(crossings)
    if len(periods_s) < 2:
        raise SystemExit("Need at least three rising zero crossings; capture more small-angle oscillations.")

    period_s = float(np.median(periods_s))
    omega_d = 2.0 * np.pi / period_s
    if args.log_decrement is not None and args.log_decrement < 0.0:
        raise SystemExit("--log-decrement must be non-negative.")
    damping_rate = 0.0 if args.log_decrement is None else args.log_decrement / period_s
    omega_n = float(np.sqrt(omega_d**2 + damping_rate**2))
    inertia = args.gravity_mps2 * args.first_mass_moment_kg_m / omega_n**2
    print(
        json.dumps(
            {
                "source": str(args.csv),
                "time_column": time_column,
                "session": requested_session,
                "sample_count": int(len(selected)),
                "rising_zero_crossings": int(len(crossings)),
                "period_count": int(len(periods_s)),
                "period_s_median": period_s,
                "period_s_stddev": float(np.std(periods_s, ddof=1)),
                "log_decrement": args.log_decrement,
                "damped_frequency_rad_s": omega_d,
                "natural_frequency_rad_s": omega_n,
                "first_mass_moment_kg_m": args.first_mass_moment_kg_m,
                "gravity_mps2": args.gravity_mps2,
                "pitch_inertia_about_axle_kg_m2": inertia,
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
