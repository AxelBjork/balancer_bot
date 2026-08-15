"""Reusable pitch-authority reconstruction and sweep measurements.

The functions in this module intentionally operate on the reflected telemetry
frame. They do not know about a particular capture path, so the same metrics
can be used for simulator artifacts and a future user-supervised hardware
authority sweep.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np
import pandas as pd

from .frames import canonicalize_telemetry_frame


# This is a deliberately conservative, predeclared gate for the first
# user-supervised hardware experiment.  The model envelope is the maximum
# observed across the nominal direct-force matrix (both signs, including the
# delay, sensor-lag, force, traction, and correlated mass/inertia cases).  The
# independent limits are the abort criteria; they are not fitted to the model.
PITCH_AUTHORITY_HARDWARE_ENVELOPES: dict[str, dict[str, Any]] = {
    "first_pm1": {
        "allowed_targets_deg": (-1.0, 1.0),
        "max_hold_s": 0.20,
        "minimum_zero_rest_s": 2.0,
        "command_duration_s": 0.20,
        "refresh_period_s": 0.020,
        "max_refresh_gap_s": 0.100,
        "model_worst_case": {
            "max_response_latency_s": 0.050,
            "max_actual_pitch_deg": 1.90,
            "max_pitch_rate_dps": 15.0,
            "max_motor_force_fraction": 0.08,
            "max_traction_fraction": 0.09,
            "max_zero_recovery_s": 1.40,
        },
        "independent_abort_limits": {
            "max_actual_pitch_deg": 10.0,
            "max_pitch_rate_dps": 50.0,
            "max_motor_force_fraction": 0.75,
            "max_traction_fraction": 0.75,
            "max_abort_to_zero_s": 0.100,
        },
    }
}


def validate_pitch_authority_hardware_envelope(
    rows: list[dict[str, Any]], *, stage: str = "first_pm1"
) -> list[dict[str, Any]]:
    """Return violations of a predeclared supervised-sweep envelope.

    This is an analysis-side release gate.  It is intentionally separate from
    the normal controller safety limits: a future hardware capture must pass
    this gate before a larger target stage is authorized.
    """

    envelope = PITCH_AUTHORITY_HARDWARE_ENVELOPES[stage]
    allowed = {float(value) for value in envelope["allowed_targets_deg"]}
    model = envelope["model_worst_case"]
    abort = envelope["independent_abort_limits"]
    violations: list[dict[str, Any]] = []

    def require(row: dict[str, Any], key: str) -> None:
        if row.get(key) is None:
            violations.append(
                {
                    "pulse_index": row.get("pulse_index"),
                    "target_deg": row.get("requested_target_deg"),
                    "metric": f"measurement_unavailable:{key}",
                    "value": None,
                    "limit": "required",
                }
            )

    def check(
        row: dict[str, Any],
        key: str,
        limit: float,
        *,
        absolute: bool = True,
        label: str | None = None,
    ) -> None:
        value = row.get(key)
        if value is None:
            return
        numeric = float(value)
        measured = abs(numeric) if absolute else numeric
        if measured > limit:
            violations.append(
                {
                    "pulse_index": row.get("pulse_index"),
                    "target_deg": row.get("requested_target_deg"),
                    "metric": label or key,
                    "value": numeric,
                    "limit": limit,
                }
            )

    for row in rows:
        target = float(row["requested_target_deg"])
        for required_key in (
            "response_latency_s",
            "peak_actual_pitch_deg",
            "peak_pitch_rate_dps",
            "motor_force_authority_fraction",
            "traction_authority_fraction",
            "zero_recovery_time_s",
            "abort_to_zero_s",
        ):
            require(row, required_key)
        if target not in allowed:
            violations.append(
                {
                    "pulse_index": row.get("pulse_index"),
                    "target_deg": target,
                    "metric": "allowed_targets_deg",
                    "value": target,
                    "limit": sorted(allowed),
                }
            )
        hold = row.get("target_hold_s")
        if hold is not None and float(hold) > float(envelope["max_hold_s"]) + 0.005:
            violations.append(
                {
                    "pulse_index": row.get("pulse_index"),
                    "target_deg": target,
                    "metric": "target_hold_s",
                    "value": float(hold),
                    "limit": float(envelope["max_hold_s"]),
                }
            )
        if row.get("response_polarity") not in (None, 0, int(math.copysign(1.0, target))):
            violations.append(
                {
                    "pulse_index": row.get("pulse_index"),
                    "target_deg": target,
                    "metric": "response_polarity",
                    "value": row.get("response_polarity"),
                    "limit": int(math.copysign(1.0, target)),
                }
            )
        check(row, "response_latency_s", float(model["max_response_latency_s"]), label="model_response_latency_s")
        check(row, "peak_actual_pitch_deg", float(model["max_actual_pitch_deg"]), label="model_peak_actual_pitch_deg")
        check(row, "peak_pitch_rate_dps", float(model["max_pitch_rate_dps"]), label="model_peak_pitch_rate_dps")
        check(row, "motor_force_authority_fraction", float(model["max_motor_force_fraction"]), label="model_motor_force_authority_fraction")
        check(row, "traction_authority_fraction", float(model["max_traction_fraction"]), label="model_traction_authority_fraction")
        check(row, "zero_recovery_time_s", float(model["max_zero_recovery_s"]), label="model_zero_recovery_time_s")
        check(row, "peak_actual_pitch_deg", float(abort["max_actual_pitch_deg"]))
        check(row, "peak_pitch_rate_dps", float(abort["max_pitch_rate_dps"]))
        check(row, "motor_force_authority_fraction", float(abort["max_motor_force_fraction"]))
        check(row, "traction_authority_fraction", float(abort["max_traction_fraction"]))
        check(row, "zero_recovery_time_s", float(model["max_zero_recovery_s"]))
        check(row, "abort_to_zero_s", float(abort["max_abort_to_zero_s"]))
    return violations


def reconstruct_final_pitch_target(
    frame: pd.DataFrame, *, gravity_mps2: float = 9.81
) -> pd.DataFrame:
    """Add production target terms and the internal pitch-error convention.

    ``pitch_error_deg`` on the wire is target minus measured pitch. The
    attitude core uses the opposite quantity, measured pitch minus target,
    because positive robot-forward motor command is the mechanical negative
    feedback direction.
    """

    data = canonicalize_telemetry_frame(frame)
    if "nominal_acceleration_mps2" in data:
        acceleration = pd.to_numeric(data["nominal_acceleration_mps2"], errors="coerce")
        data["drive_pitch_contribution_deg"] = np.degrees(
            np.arctan2(acceleration, float(gravity_mps2))
        )
    if "pitch_error_deg" in data:
        data["internal_pitch_error_deg"] = -pd.to_numeric(
            data["pitch_error_deg"], errors="coerce"
        )
    if "pitch_sp_deg" in data:
        data["final_attitude_target_deg"] = pd.to_numeric(data["pitch_sp_deg"], errors="coerce")
    if {
        "drive_pitch_contribution_deg",
        "velocity_pitch_request_limited_deg",
        "com_trim_deg",
    }.issubset(data.columns):
        data["reconstructed_pitch_target_unclamped_deg"] = (
            data["drive_pitch_contribution_deg"]
            + pd.to_numeric(data["velocity_pitch_request_limited_deg"], errors="coerce")
            + pd.to_numeric(data["com_trim_deg"], errors="coerce")
        )
    if {
        "reconstructed_pitch_target_unclamped_deg",
        "pitch_target_unclamped_deg",
    }.issubset(data.columns):
        data["pitch_target_reconstruction_error_deg"] = (
            data["reconstructed_pitch_target_unclamped_deg"]
            - pd.to_numeric(data["pitch_target_unclamped_deg"], errors="coerce")
        )
    return data


def _finite_window(
    frame: pd.DataFrame,
    columns: list[str],
    *,
    start_s: float | None = None,
    end_s: float | None = None,
) -> pd.DataFrame:
    data = canonicalize_telemetry_frame(frame)
    if "t_sec" not in data.columns:
        return pd.DataFrame(columns=["t_sec", *columns])
    selected = data.copy()
    time = pd.to_numeric(selected["t_sec"], errors="coerce")
    if start_s is not None:
        selected = selected.loc[time >= start_s]
    if end_s is not None:
        selected = selected.loc[time <= end_s]
    required = [column for column in ["t_sec", *columns] if column in selected.columns]
    if len(required) != len(columns) + 1:
        return pd.DataFrame(columns=required)
    return selected[required].replace([np.inf, -np.inf], np.nan).dropna()


def lagged_correlation(
    frame: pd.DataFrame,
    source: str,
    response: str,
    *,
    start_s: float | None = None,
    end_s: float | None = None,
    max_lag_s: float = 0.5,
) -> dict[str, Any]:
    """Find the descriptive correlation over positive and negative time lags.

    A positive lag means the response sample is evaluated later than the
    source sample. The result is an alignment diagnostic, not a causal model;
    shared state feedback can make zero-lag channels correlate strongly.
    """

    result: dict[str, Any] = {
        "source": source,
        "response": response,
        "sample_count": 0,
        "lag_s": None,
        "correlation": None,
        "scale": None,
        "rmse": None,
        "unavailable_reason": None,
    }
    selected = _finite_window(frame, [source, response], start_s=start_s, end_s=end_s)
    if len(selected) < 8:
        result["unavailable_reason"] = "fewer than eight finite aligned samples"
        return result
    selected = selected.sort_values("t_sec").drop_duplicates("t_sec")
    time = selected["t_sec"].to_numpy(dtype=float)
    source_values = selected[source].to_numpy(dtype=float)
    response_values = selected[response].to_numpy(dtype=float)
    deltas = np.diff(time)
    positive = deltas[np.isfinite(deltas) & (deltas > 0.0)]
    if positive.size == 0:
        result["unavailable_reason"] = "no positive cadence"
        return result
    dt_s = float(np.median(positive))
    breaks = np.flatnonzero((deltas <= 0.0) | (deltas > max(5.0 * dt_s, 0.25)))
    starts = np.r_[0, breaks + 1]
    stops = np.r_[breaks + 1, len(time)]
    best_segment = int(np.argmax(stops - starts))
    segment = slice(int(starts[best_segment]), int(stops[best_segment]))
    time = time[segment]
    source_values = source_values[segment]
    response_values = response_values[segment]
    if len(time) < 8:
        result["unavailable_reason"] = "longest contiguous segment is too short"
        return result
    uniform_time = np.arange(time[0], time[-1] + 0.5 * dt_s, dt_s)
    source_values = np.interp(uniform_time, time, source_values)
    response_values = np.interp(uniform_time, time, response_values)
    max_lag_steps = min(max(0, int(round(max_lag_s / dt_s))), len(source_values) - 2)

    best: dict[str, Any] | None = None
    for lag_steps in range(-max_lag_steps, max_lag_steps + 1):
        if lag_steps > 0:
            source_aligned = source_values[:-lag_steps]
            response_aligned = response_values[lag_steps:]
        elif lag_steps < 0:
            source_aligned = source_values[-lag_steps:]
            response_aligned = response_values[:lag_steps]
        else:
            source_aligned = source_values
            response_aligned = response_values
        if len(source_aligned) < 8:
            continue
        source_demeaned = source_aligned - np.mean(source_aligned)
        response_demeaned = response_aligned - np.mean(response_aligned)
        source_energy = float(np.dot(source_demeaned, source_demeaned))
        response_energy = float(np.dot(response_demeaned, response_demeaned))
        if source_energy <= 1e-12 or response_energy <= 1e-12:
            continue
        correlation = float(
            np.dot(source_demeaned, response_demeaned)
            / math.sqrt(source_energy * response_energy)
        )
        scale = float(np.dot(source_demeaned, response_demeaned) / source_energy)
        residual = scale * source_demeaned - response_demeaned
        candidate = {
            "source": source,
            "response": response,
            "sample_count": int(len(source_aligned)),
            "lag_s": lag_steps * dt_s,
            "lag_steps": lag_steps,
            "dt_s": dt_s,
            "correlation": correlation,
            "scale": scale,
            "rmse": float(np.sqrt(np.mean(residual * residual))),
            "unavailable_reason": None,
        }
        if best is None or abs(candidate["correlation"]) > abs(best["correlation"]):
            best = candidate
    return best or {**result, "unavailable_reason": "signals have no varying overlap"}


def _pulse_groups(data: pd.DataFrame) -> list[pd.DataFrame]:
    required = {"t_sec", "pitch_authority_diagnostic_active", "pitch_authority_diagnostic_target_deg"}
    if not required.issubset(data.columns):
        return []
    selected = data.loc[
        pd.to_numeric(data["pitch_authority_diagnostic_active"], errors="coerce") > 0.5
    ].copy()
    if selected.empty:
        return []
    selected["_target"] = pd.to_numeric(
        selected["pitch_authority_diagnostic_target_deg"], errors="coerce"
    )
    selected = selected.replace([np.inf, -np.inf], np.nan).dropna(subset=["t_sec", "_target"])
    selected = selected.sort_values("t_sec")
    times = selected["t_sec"].to_numpy(dtype=float)
    targets = selected["_target"].to_numpy(dtype=float)
    if len(selected) < 2:
        return []
    deltas = np.diff(times)
    positive = deltas[np.isfinite(deltas) & (deltas > 0.0)]
    cadence = float(np.median(positive)) if positive.size else 0.0
    groups: list[pd.DataFrame] = []
    start = 0
    for index in range(1, len(selected)):
        new_target = abs(targets[index] - targets[index - 1]) > 1e-6
        time_break = cadence > 0.0 and times[index] - times[index - 1] > max(5.0 * cadence, 0.25)
        if new_target or time_break:
            groups.append(selected.iloc[start:index].drop(columns=["_target"]))
            start = index
    groups.append(selected.iloc[start:].drop(columns=["_target"]))
    # Zero-target quiet gaps are not pulses. A nonzero group is one requested
    # target, including either sign.
    return [group for group in groups if abs(float(group["pitch_authority_diagnostic_target_deg"].iloc[0])) > 1e-9]


def _value(data: pd.DataFrame, column: str, default: float | None = None) -> float | None:
    if column not in data.columns or data.empty:
        return default
    values = pd.to_numeric(data[column], errors="coerce").replace([np.inf, -np.inf], np.nan).dropna()
    return float(values.iloc[0]) if not values.empty else default


def _mean_value(data: pd.DataFrame, column: str, default: float | None = None) -> float | None:
    if column not in data.columns or data.empty:
        return default
    values = pd.to_numeric(data[column], errors="coerce").replace([np.inf, -np.inf], np.nan).dropna()
    return float(values.mean()) if not values.empty else default


def _peak_abs(data: pd.DataFrame, column: str) -> float | None:
    if column not in data.columns or data.empty:
        return None
    values = pd.to_numeric(data[column], errors="coerce").replace([np.inf, -np.inf], np.nan).dropna()
    return float(np.max(np.abs(values))) if not values.empty else None


def _fraction_nonzero(data: pd.DataFrame, columns: tuple[str, ...]) -> float | None:
    present = [column for column in columns if column in data.columns]
    if not present or data.empty:
        return None
    values = data[present].apply(pd.to_numeric, errors="coerce")
    finite = values.notna().all(axis=1)
    if not finite.any():
        return None
    return float((values.loc[finite].abs().max(axis=1) > 1e-9).mean())


def _ratio_peak(data: pd.DataFrame, numerator: str, denominator: str) -> float | None:
    if numerator not in data.columns or denominator not in data.columns or data.empty:
        return None
    numerator_values = pd.to_numeric(data[numerator], errors="coerce").to_numpy(dtype=float)
    denominator_values = pd.to_numeric(data[denominator], errors="coerce").to_numpy(dtype=float)
    finite = np.isfinite(numerator_values) & np.isfinite(denominator_values)
    finite &= np.abs(denominator_values) > 1e-12
    if not np.any(finite):
        return None
    return float(np.max(np.abs(numerator_values[finite] / denominator_values[finite])))


def _slope(values_x: np.ndarray, values_y: np.ndarray) -> float | None:
    finite = np.isfinite(values_x) & np.isfinite(values_y)
    if int(np.count_nonzero(finite)) < 3:
        return None
    x = values_x[finite]
    y = values_y[finite]
    denominator = float(np.dot(x, x))
    return float(np.dot(x, y) / denominator) if denominator > 1e-12 else None


def analyze_pitch_authority_sweep(
    frame: pd.DataFrame,
    *,
    response_column: str = "fused_pitch_deg",
    meters_per_step: float = math.pi * 0.0824 / (200.0 * 16.0),
    response_threshold_deg: float = 0.10,
) -> list[dict[str, Any]]:
    """Return one row of target-realization metrics per direct-target pulse.

    Missing hardware-only signals remain ``None``. In particular, an absent
    actuator-limit field is not silently interpreted as zero limiting.
    """

    data = reconstruct_final_pitch_target(frame)
    groups = _pulse_groups(data)
    rows: list[dict[str, Any]] = []
    group_starts = [float(group["t_sec"].iloc[0]) for group in groups]
    for pulse_index, group in enumerate(groups, start=1):
        target = float(group["pitch_authority_diagnostic_target_deg"].iloc[0])
        direction = math.copysign(1.0, target)
        start_s = float(group["t_sec"].iloc[0])
        end_s = float(group["t_sec"].iloc[-1])
        # Include the first post-pulse response until the next target change.
        # This is important for short pulses: a fixed multiple of the hold
        # time can end before the body has returned to the zero-target state.
        post_end_s = end_s + max(1.5, 2.0 * (end_s - start_s))
        if pulse_index < len(group_starts):
            # ``pulse_index`` is one-based here, so it is the next group's
            # zero-based index.
            post_end_s = min(post_end_s, group_starts[pulse_index] - 1e-9)
        post = data.loc[pd.to_numeric(data["t_sec"], errors="coerce") >= start_s].copy()
        post = post.loc[pd.to_numeric(post["t_sec"], errors="coerce") <= post_end_s]
        response = pd.to_numeric(post.get(response_column, pd.Series(dtype=float)), errors="coerce")
        response = response.replace([np.inf, -np.inf], np.nan)
        finite_response = response.dropna()
        actual = finite_response.to_numpy(dtype=float)
        times = pd.to_numeric(post.loc[finite_response.index, "t_sec"], errors="coerce").to_numpy(dtype=float)
        prior = data.loc[pd.to_numeric(data["t_sec"], errors="coerce") < start_s]
        prior = prior.tail(1)
        initial_actual = _value(prior, response_column)
        if initial_actual is None:
            initial_actual = float(actual[0]) if actual.size else None
        actual_delta = actual - (initial_actual if initial_actual is not None else 0.0)
        threshold = max(float(response_threshold_deg), 0.10 * abs(target))
        response_indices = np.flatnonzero(direction * actual_delta >= threshold)
        latency_s = float(times[response_indices[0]] - start_s) if response_indices.size else None
        signed_actual = direction * actual if actual.size else np.array([], dtype=float)
        peak_index = int(np.argmax(signed_actual)) if signed_actual.size else None
        peak_actual_signed = float(actual[peak_index]) if peak_index is not None else None
        peak_actual_magnitude = (
            float(direction * peak_actual_signed) if peak_actual_signed is not None else None
        )
        peak_rate = _peak_abs(post, "filtered_pitch_rate_dps")
        if peak_rate is None:
            peak_rate = _peak_abs(post, "pitch_rate_dps")
        target_gain = (
            peak_actual_magnitude / abs(target)
            if peak_actual_magnitude is not None and abs(target) > 0.0
            else None
        )
        overshoot = (
            max(0.0, peak_actual_magnitude - abs(target))
            if peak_actual_magnitude is not None
            else None
        )

        active_times = pd.to_numeric(group["t_sec"], errors="coerce").to_numpy(dtype=float)
        active_actual = pd.to_numeric(
            group.get(response_column, pd.Series(dtype=float)), errors="coerce"
        ).to_numpy(dtype=float)
        active_finite = np.isfinite(active_times) & np.isfinite(active_actual)
        active_times = active_times[active_finite]
        active_actual = active_actual[active_finite]
        target_transition_s = start_s
        target_hold_s = max(0.0, end_s - start_s)
        rise_10_s = None
        rise_90_s = None
        if active_actual.size:
            active_delta = direction * (active_actual - (initial_actual or 0.0))
            for fraction, name in ((0.10, "rise_10_s"), (0.90, "rise_90_s")):
                crossing = np.flatnonzero(active_delta >= fraction * abs(target))
                if crossing.size:
                    value = float(active_times[crossing[0]] - start_s)
                    if name == "rise_10_s":
                        rise_10_s = value
                    else:
                        rise_90_s = value
        rise_time_s = (
            rise_90_s - rise_10_s
            if rise_10_s is not None and rise_90_s is not None
            else None
        )
        steady_start_s = start_s + 0.80 * target_hold_s
        steady = group.loc[
            pd.to_numeric(group["t_sec"], errors="coerce") >= steady_start_s
        ]
        steady_actual = _mean_value(steady, response_column)
        steady_gain = (
            direction * steady_actual / abs(target)
            if steady_actual is not None and abs(target) > 0.0
            else None
        )
        steady_error = (
            target - steady_actual if steady_actual is not None else None
        )
        settling_time_s = None
        if steady_actual is not None and active_actual.size:
            settling_tolerance = max(0.10, 0.05 * abs(steady_actual))
            distance = np.abs(active_actual - steady_actual)
            for index in range(len(active_actual)):
                if np.all(distance[index:] <= settling_tolerance):
                    settling_time_s = float(active_times[index] - start_s)
                    break
        post_hold = post.loc[pd.to_numeric(post["t_sec"], errors="coerce") > end_s]
        post_actual = pd.to_numeric(
            post_hold.get(response_column, pd.Series(dtype=float)), errors="coerce"
        ).replace([np.inf, -np.inf], np.nan).dropna()
        ringdown_peak = (
            float(np.max(np.abs(post_actual.to_numpy(dtype=float))))
            if not post_actual.empty
            else None
        )
        abort_to_zero_s = None
        if "pitch_sp_deg" in post_hold:
            target_times = pd.to_numeric(post_hold["t_sec"], errors="coerce").to_numpy(
                dtype=float
            )
            target_values = pd.to_numeric(post_hold["pitch_sp_deg"], errors="coerce").to_numpy(
                dtype=float
            )
            zero_target = np.isfinite(target_times) & np.isfinite(target_values)
            zero_target &= np.abs(target_values) <= 0.10
            zero_indices = np.flatnonzero(zero_target)
            if zero_indices.size:
                abort_to_zero_s = float(target_times[zero_indices[0]] - end_s)
        zero_recovery_time_s = None
        if not post_hold.empty and response_column in post_hold.columns:
            post_times = pd.to_numeric(post_hold["t_sec"], errors="coerce").to_numpy(dtype=float)
            post_values = pd.to_numeric(post_hold[response_column], errors="coerce").to_numpy(dtype=float)
            finite_post = np.isfinite(post_times) & np.isfinite(post_values)
            post_times = post_times[finite_post]
            post_values = post_values[finite_post]
            for index, recovery_time in enumerate(post_times):
                recovery_window = (
                    (post_times >= recovery_time)
                    & (post_times <= recovery_time + 0.25)
                )
                if (
                    np.count_nonzero(recovery_window) >= 2
                    and np.all(np.abs(post_values[recovery_window]) <= 0.10)
                ):
                    zero_recovery_time_s = float(recovery_time - end_s)
                    break

        terminal_actual = _value(group.tail(1), response_column)
        terminal_gain = (
            direction * terminal_actual / abs(target)
            if terminal_actual is not None and abs(target) > 0.0
            else None
        )
        terminal_error = target - terminal_actual if terminal_actual is not None else None
        hold_acceleration = pd.to_numeric(
            group.get("x_ddot", pd.Series(dtype=float)), errors="coerce"
        ).replace([np.inf, -np.inf], np.nan).dropna()

        velocity_column = "corrected_axle_velocity_sps"
        initial_velocity = _value(group, velocity_column)
        if initial_velocity is None:
            initial_velocity = _value(group, "plant_velocity_mps")
        final_velocity = _value(post.tail(1), velocity_column)
        if final_velocity is None:
            final_velocity = _value(post.tail(1), "plant_velocity_mps")
        velocity_change = (
            final_velocity - initial_velocity
            if initial_velocity is not None and final_velocity is not None
            else None
        )

        acceleration_column = "completed_step_acceleration_sps2"
        if acceleration_column in post.columns:
            acceleration = pd.to_numeric(post[acceleration_column], errors="coerce").to_numpy(dtype=float)
        elif velocity_column in post.columns and "t_sec" in post.columns:
            velocity = pd.to_numeric(post[velocity_column], errors="coerce").to_numpy(dtype=float)
            times_all = pd.to_numeric(post["t_sec"], errors="coerce").to_numpy(dtype=float)
            acceleration = np.gradient(velocity, times_all) if len(velocity) >= 3 else np.array([])
        else:
            acceleration = np.array([])
        actual_for_slope = pd.to_numeric(
            post.get(response_column, pd.Series(dtype=float)), errors="coerce"
        ).to_numpy(dtype=float)
        paired = np.isfinite(acceleration) & np.isfinite(actual_for_slope)
        acceleration_per_degree = (
            _slope(actual_for_slope[paired], acceleration[paired])
            if np.count_nonzero(paired) >= 3
            else None
        )
        finite_acceleration = acceleration[np.isfinite(acceleration)]
        if acceleration_per_degree is None and finite_acceleration.size:
            acceleration_per_degree = (
                float(np.mean(finite_acceleration)) / peak_actual_magnitude
                if peak_actual_magnitude
                else None
            )

        row: dict[str, Any] = {
            "pulse_index": pulse_index,
            "requested_target_deg": target,
            "actual_target_after_clamp_deg": _value(group, "pitch_sp_deg"),
            "start_s": start_s,
            "end_s": end_s,
            "initial_velocity": initial_velocity,
            "target_polarity": int(direction),
            "response_polarity": (
                int(math.copysign(1.0, peak_actual_signed))
                if peak_actual_signed is not None and abs(peak_actual_signed) > 1e-9
                else 0
            ),
            "response_latency_s": latency_s,
            "target_transition_s": target_transition_s,
            "target_hold_s": target_hold_s,
            "diagnostic_request_id": _value(
                group, "pitch_authority_diagnostic_request_id"
            ),
            "command_age_at_transition_ms": _value(
                group, "pitch_authority_diagnostic_command_age_ms"
            ),
            "command_age_peak_ms": _peak_abs(
                post, "pitch_authority_diagnostic_command_age_ms"
            ),
            "rise_10_s": rise_10_s,
            "rise_90_s": rise_90_s,
            "rise_time_10_to_90_s": rise_time_s,
            "initial_actual_pitch_deg": initial_actual,
            "peak_actual_pitch_deg": peak_actual_signed,
            "actual_target_gain": target_gain,
            "overshoot_deg": overshoot,
            "steady_actual_pitch_deg": steady_actual,
            "steady_target_gain": steady_gain,
            "steady_target_error_deg": steady_error,
            "settling_time_s": settling_time_s,
            "ringdown_peak_actual_pitch_deg": ringdown_peak,
            "terminal_actual_pitch_deg": terminal_actual,
            "terminal_target_gain": terminal_gain,
            "terminal_target_error_deg": terminal_error,
            "zero_recovery_time_s": zero_recovery_time_s,
            "abort_to_zero_s": abort_to_zero_s,
            "peak_pitch_rate_dps": peak_rate,
            "motor_command_peak_sps": _peak_abs(post, "u_sps"),
            "applied_force_peak_n": _peak_abs(post, "f_app"),
            "motor_force_authority_fraction": _ratio_peak(
                post, "f_app", "motor_force_limit_n"
            ),
            "traction_authority_fraction": _ratio_peak(
                post, "f_app", "traction_limit_n"
            ),
            "velocity_change": velocity_change,
            "mean_realized_acceleration_mps2": _mean_value(post, "x_ddot"),
            "peak_realized_acceleration_mps2": _peak_abs(post, "x_ddot"),
            "hold_mean_realized_acceleration_mps2": (
                float(hold_acceleration.mean()) if not hold_acceleration.empty else None
            ),
            "hold_peak_realized_acceleration_mps2": (
                float(np.max(np.abs(hold_acceleration.to_numpy(dtype=float))))
                if not hold_acceleration.empty
                else None
            ),
            "mean_acceleration_sps2": float(np.mean(acceleration)) if acceleration.size else None,
            "peak_acceleration_sps2": float(np.max(np.abs(acceleration))) if acceleration.size else None,
            "sps_per_s_per_degree_actual_pitch": acceleration_per_degree,
            "mps2_per_degree_actual_pitch": (
                acceleration_per_degree * meters_per_step if acceleration_per_degree is not None else None
            ),
            "actuator_limit_fraction": _fraction_nonzero(
                post, ("actuator_saturation_flags", "controller_saturation_flags", "command_saturated")
            ),
            "safety_event": _fraction_nonzero(post, ("controller_fault_flags", "actuator_fault")),
        }
        rows.append(row)
    return rows


def compare_pitch_authority_sweeps(
    simulator_frame: pd.DataFrame, hardware_frame: pd.DataFrame
) -> list[dict[str, Any]]:
    """Join simulator and hardware pulse summaries by requested target."""

    simulator = analyze_pitch_authority_sweep(simulator_frame)
    hardware = analyze_pitch_authority_sweep(hardware_frame)
    result: list[dict[str, Any]] = []
    used: set[int] = set()
    for sim_row in simulator:
        match_index = next(
            (
                index
                for index, candidate in enumerate(hardware)
                if index not in used
                and math.isclose(
                    float(candidate["requested_target_deg"]),
                    float(sim_row["requested_target_deg"]),
                    abs_tol=1e-6,
                )
            ),
            None,
        )
        merged: dict[str, Any] = {"requested_target_deg": sim_row["requested_target_deg"]}
        for prefix, row in (("simulator", sim_row), ("hardware", hardware[match_index] if match_index is not None else None)):
            if row is None:
                continue
            for key, value in row.items():
                if key not in {"requested_target_deg", "pulse_index"}:
                    merged[f"{prefix}_{key}"] = value
        if match_index is not None:
            used.add(match_index)
            for key in ("peak_actual_pitch_deg", "actual_target_gain", "response_latency_s", "peak_pitch_rate_dps"):
                sim_value = sim_row.get(key)
                hardware_value = hardware[match_index].get(key)
                if sim_value is not None and hardware_value is not None:
                    merged[f"hardware_minus_simulator_{key}"] = hardware_value - sim_value
        result.append(merged)
    return result
