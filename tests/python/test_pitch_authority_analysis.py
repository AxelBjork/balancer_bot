from __future__ import annotations

import numpy as np
import pandas as pd

from tools.telemetry_analysis import (
    analyze_pitch_authority_sweep,
    lagged_correlation,
    reconstruct_final_pitch_target,
    validate_pitch_authority_hardware_envelope,
)


def test_reconstruct_final_pitch_target_matches_production_composition() -> None:
    frame = pd.DataFrame(
        {
            "t_sec": [0.0, 0.1],
            "nominal_acceleration_mps2": [0.0, 1.25],
            "velocity_pitch_request_limited_deg": [-4.0, 2.0],
            "com_trim_deg": [0.25, -0.5],
            "pitch_target_unclamped_deg": [-3.75, np.degrees(np.arctan2(1.25, 9.81)) + 1.5],
            "pitch_sp_deg": [-3.75, 1.5],
            "pitch_error_deg": [-5.0, 4.0],
        }
    )
    reconstructed = reconstruct_final_pitch_target(frame)
    assert np.max(np.abs(reconstructed["pitch_target_reconstruction_error_deg"])) <= 1e-12
    assert np.allclose(
        reconstructed["internal_pitch_error_deg"].to_numpy(), [5.0, -4.0]
    )


def test_pitch_authority_sweep_reports_signed_bidirectional_response() -> None:
    times = np.arange(0.0, 3.0, 0.01)
    target = np.zeros_like(times)
    target[(times >= 0.5) & (times < 1.0)] = 2.0
    target[(times >= 1.5) & (times < 2.0)] = -2.0
    actual = np.where(times < 0.5, 0.0, np.where(times < 1.5, 3.0, -3.0))
    frame = pd.DataFrame(
        {
            "t_sec": times,
            "pitch_authority_diagnostic_active": np.ones_like(times),
            "pitch_authority_diagnostic_target_deg": target,
            "pitch_sp_deg": target,
            "fused_pitch_deg": actual,
            "pitch_rate_dps": np.gradient(actual, times),
            "u_sps": target * 100.0,
            "corrected_axle_velocity_sps": np.gradient(times) * 0.0,
            "completed_step_acceleration_sps2": np.zeros_like(times),
            "controller_saturation_flags": np.zeros_like(times),
            "actuator_saturation_flags": np.zeros_like(times),
            "controller_fault_flags": np.zeros_like(times),
            "actuator_fault": np.zeros_like(times),
        }
    )
    rows = analyze_pitch_authority_sweep(frame)
    assert [row["requested_target_deg"] for row in rows] == [2.0, -2.0]
    assert [row["response_polarity"] for row in rows] == [1, -1]
    assert [row["actual_target_gain"] for row in rows] == [1.5, 1.5]
    assert all(row["actuator_limit_fraction"] == 0.0 for row in rows)


def test_lagged_correlation_searches_both_time_directions() -> None:
    times = np.arange(0.0, 5.0, 0.01)
    source = np.sin(2.0 * np.pi * 0.7 * times)
    response = np.zeros_like(source)
    delay_samples = 7
    response[delay_samples:] = source[:-delay_samples]
    result = lagged_correlation(
        pd.DataFrame({"t_sec": times, "source": source, "response": response}),
        "source",
        "response",
        max_lag_s=0.2,
    )
    assert result["lag_s"] is not None
    assert abs(float(result["lag_s"]) - delay_samples * 0.01) <= 0.011
    assert float(result["correlation"]) >= 0.98


def test_first_hardware_envelope_rejects_long_or_larger_pulses() -> None:
    rows = [
        {
            "pulse_index": 1,
            "requested_target_deg": 1.0,
            "target_hold_s": 0.20,
            "response_polarity": 1,
            "response_latency_s": 0.05,
            "peak_actual_pitch_deg": 1.8,
            "peak_pitch_rate_dps": 14.0,
            "motor_force_authority_fraction": 0.07,
            "traction_authority_fraction": 0.08,
            "zero_recovery_time_s": 1.2,
            "abort_to_zero_s": 0.005,
        }
    ]
    assert validate_pitch_authority_hardware_envelope(rows) == []

    rows[0]["requested_target_deg"] = 2.0
    violations = validate_pitch_authority_hardware_envelope(rows)
    assert any(item["metric"] == "allowed_targets_deg" for item in violations)

    rows[0]["requested_target_deg"] = 1.0
    rows[0]["target_hold_s"] = 0.45
    assert any(
        item["metric"] == "target_hold_s"
        for item in validate_pitch_authority_hardware_envelope(rows)
    )
