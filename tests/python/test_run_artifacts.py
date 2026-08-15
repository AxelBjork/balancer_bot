from __future__ import annotations

import csv
import math
from pathlib import Path

import numpy as np
import pandas as pd

from tests.python.support.run_artifacts import (
    RunRecorder,
    analyze_timeline_rows,
    estimate_lag_scale,
    summarize_reference_file,
)
from tools.telemetry_analysis import actuator_stage_metrics, band_rms_equivalent


def test_band_rms_equivalent_isolates_hann_windowed_command_vibration():
    sample_rate_hz = 400.0
    sample_count = 4000
    amplitude = 80.0
    frame = pd.DataFrame(
        {
            "t_sec": [index / sample_rate_hz for index in range(sample_count)],
            "u_sps": [
                15.0 * math.sin(2.0 * math.pi * 5.0 * index / sample_rate_hz)
                + amplitude * math.sin(2.0 * math.pi * 60.0 * index / sample_rate_hz)
                for index in range(sample_count)
            ],
        }
    )

    spectrum = band_rms_equivalent(frame, "u_sps", 30.0, 100.0)

    assert abs(float(spectrum["sample_rate_hz"]) - sample_rate_hz) < 1e-9
    assert spectrum["sample_count"] == sample_count
    assert spectrum["rms"] is not None
    assert abs(float(spectrum["rms"]) - amplitude / math.sqrt(2.0)) < 0.5


def test_band_rms_equivalent_rejects_nonuniform_or_missing_signal_data():
    frame = pd.DataFrame({"t_sec": [0.0, 0.0025, 0.006, 0.0075], "u_sps": [1.0] * 4})
    assert band_rms_equivalent(frame, "u_sps", 30.0, 100.0)["rms"] is None
    assert band_rms_equivalent(frame, "missing", 30.0, 100.0)["rms"] is None


def test_run_recorder_writes_summary_and_flags_fall(tmp_path):
    recorder = RunRecorder()
    recorder.begin_run({"run_id": "unit_recorder"})
    recorder.record_step(
        {
            "sim_time_s": 0.0,
            "pitch_deg": 0.0,
            "u_sps": 0.0,
            "plant_pitch_deg": 0.0,
            "plant_position": 0.0,
            "plant_velocity": 0.0,
            "command_saturated": 0.0,
        }
    )
    recorder.record_step(
        {
            "sim_time_s": 0.01,
            "pitch_deg": 90.0,
            "u_sps": 100.0,
            "plant_pitch_deg": 90.0,
            "plant_position": 1.25,
            "plant_velocity": 0.5,
            "command_saturated": 1.0,
        }
    )

    summary = recorder.write_csv_json_plots(tmp_path)
    assert summary["fell"]
    assert summary["max_abs_pitch_deg"] == 90.0
    assert summary["tail_rms_pitch_deg"] is not None
    assert summary["tail_rail_fraction"] is not None
    assert summary["tail_command_rail_fraction"] is not None
    assert summary["final_position_m"] == 1.25
    assert summary["max_abs_position_m"] == 1.25
    assert summary["tail_mean_abs_velocity_mps"] == 0.25
    assert summary["max_abs_target_wheel_velocity"] is None
    assert summary["max_abs_actual_wheel_velocity"] is None
    assert summary["max_abs_f_app"] is None
    assert summary["max_abs_theta_ddot"] is None
    assert (tmp_path / "timeline.csv").exists()
    assert (tmp_path / "summary.json").exists()
    assert (tmp_path / "overview_plot.svg").exists()
    assert (tmp_path / "actuator_plot.svg").exists()
    assert not (tmp_path / "pitch_plot.svg").exists()
    assert not (tmp_path / "command_plot.svg").exists()
    assert not (tmp_path / "wheel_plot.svg").exists()
    assert not (tmp_path / "force_plot.svg").exists()
    with (tmp_path / "timeline.csv").open(newline="", encoding="utf-8") as stream:
        header = next(csv.reader(stream))
    assert "t_sec" in header
    assert "plant_position_m" in header
    assert "plant_velocity_mps" in header
    assert "sim_time_s" not in header
    assert "plant_position" not in header
    assert "plant_velocity" not in header


def test_reference_parser_handles_clean_csv():
    summary = summarize_reference_file(Path("tests/data") / "imu_data.csv")
    assert summary["sample_count"] > 100
    assert summary["time_field"] == "time"
    assert summary["dt_max"] is not None


def test_reference_parser_handles_mixed_logs():
    for name in ("imu_held.csv", "balance_raw.csv", "balance.csv"):
        summary = summarize_reference_file(Path("tests/data") / name)
        assert summary["sample_count"] > 100
        assert "time_field" in summary


def test_estimate_lag_scale_recovers_known_delay_and_gain():
    rows = []
    dt = 0.01
    lag_steps = 3
    for i in range(200):
        t = i * dt
        source = 0.0
        if 20 <= i < 40:
            source = 100.0
        elif 60 <= i < 75:
            source = -60.0
        elif 100 <= i < 130:
            source = 140.0
        response = 0.0
        if i >= lag_steps:
            prev = i - lag_steps
            prev_source = 0.0
            if 20 <= prev < 40:
                prev_source = 100.0
            elif 60 <= prev < 75:
                prev_source = -60.0
            elif 100 <= prev < 130:
                prev_source = 140.0
            response = 1.5 * prev_source
        rows.append(
            {
                "sim_time_s": t,
                "u_sps": source,
                "corrected_axle_velocity_sps": response,
            }
        )

    estimate = estimate_lag_scale(
        rows, "u_sps", "corrected_axle_velocity_sps", max_lag_s=0.2, min_abs_source=10.0
    )
    assert estimate["lag_steps"] == lag_steps
    assert abs(estimate["lag_s"] - (lag_steps * dt)) < 1e-9
    assert abs(estimate["scale"] - 1.5) < 0.05
    assert estimate["correlation"] > 0.99


def test_analyze_timeline_rows_reports_estimator_and_drive_sections():
    rows = []
    dt = 0.01
    for i in range(200):
        t = i * dt
        raw_pitch = 0.0
        fused_pitch = 0.0
        if 15 <= i < 35:
            raw_pitch = 5.0
        elif 70 <= i < 95:
            raw_pitch = -3.0
        elif 130 <= i < 160:
            raw_pitch = 4.0
        if i >= 2:
            prev = i - 2
            if 15 <= prev < 35:
                fused_pitch = 5.0
            elif 70 <= prev < 95:
                fused_pitch = -3.0
            elif 130 <= prev < 160:
                fused_pitch = 4.0

        gyro = 0.0
        filtered_gyro = 0.0
        if 40 <= i < 50:
            gyro = 40.0
        elif 105 <= i < 120:
            gyro = -25.0
        if i >= 1:
            prev = i - 1
            if 40 <= prev < 50:
                filtered_gyro = 40.0
            elif 105 <= prev < 120:
                filtered_gyro = -25.0

        command = 0.0
        measured = 0.0
        if 25 <= i < 45:
            command = 120.0
        elif 90 <= i < 110:
            command = -80.0
        if i >= 4:
            prev = i - 4
            if 25 <= prev < 45:
                measured = 0.8 * 120.0
            elif 90 <= prev < 110:
                measured = 0.8 * -80.0
        left_applied = 0.0
        right_applied = 0.0
        if i >= 2:
            prev = i - 2
            if 25 <= prev < 45:
                left_applied = 120.0
                right_applied = 120.0
            elif 90 <= prev < 110:
                left_applied = -80.0
                right_applied = -80.0
        rows.append(
            {
                "sim_time_s": t,
                "raw_acc_pitch_deg": raw_pitch,
                "fused_pitch_deg": fused_pitch,
                "gyro_pitch_rate_dps": gyro,
                "filtered_pitch_rate_dps": filtered_gyro,
                "u_sps": command,
                "left_applied_sps": left_applied,
                "right_applied_sps": right_applied,
                "corrected_axle_velocity_sps": measured,
                "raw_completed_velocity_sps": measured,
            }
        )

    analysis = analyze_timeline_rows(rows)
    assert analysis["time_key"] == "t_sec"
    assert analysis["estimator_pitch"]["lag_steps"] == 2
    assert analysis["estimator_pitch_rate"]["lag_steps"] == 1
    assert analysis["drive_command_to_corrected_axle_velocity"]["lag_steps"] == 4
    assert analysis["drive_command_to_left_applied"]["lag_steps"] == 2
    assert analysis["drive_command_to_right_applied"]["lag_steps"] == 2
    assert analysis["left_applied_to_corrected_axle_velocity"]["lag_steps"] == 2
    assert analysis["right_applied_to_corrected_axle_velocity"]["lag_steps"] == 2
    assert analysis["fused_pitch_to_filtered_pitch_rate"]["lag_steps"] is not None


def test_actuator_stage_metrics_reports_known_slew_errors_and_nine_hertz_phase():
    sample_count = 4000
    dt_s = 1.0 / 400.0
    frequency_hz = 9.0
    delay_steps = 2
    time = np.arange(sample_count, dtype=float) * dt_s
    requested = 1000.0 * np.sin(2.0 * np.pi * frequency_hz * time)
    left_slewed = requested + 10.0
    right_slewed = requested - 20.0
    left_applied = np.full(sample_count, np.nan)
    right_applied = np.full(sample_count, np.nan)
    left_applied[delay_steps:] = left_slewed[:-delay_steps]
    right_applied[delay_steps:] = right_slewed[:-delay_steps]
    flags = np.zeros(sample_count, dtype=np.uint32)
    flags[100:120] |= 1
    flags[200:240] |= 2
    frame = pd.DataFrame(
        {
            "t_sec": time,
            "left_target_sps": requested,
            "right_target_sps": requested,
            "left_slewed_sps": left_slewed,
            "right_slewed_sps": right_slewed,
            "left_applied_sps": left_applied,
            "right_applied_sps": right_applied,
            "actuator_saturation_flags": flags,
        }
    )

    metrics = actuator_stage_metrics(frame)
    assert abs(metrics["slew_limited_fraction"] - (60.0 / sample_count)) < 1e-12
    assert abs(metrics["left_slew_limited_fraction"] - (20.0 / sample_count)) < 1e-12
    assert abs(metrics["right_slew_limited_fraction"] - (40.0 / sample_count)) < 1e-12
    assert abs(metrics["longest_slew_interval_s"] - 0.1) < 1e-12
    errors = metrics["stage_error_rms_sps"]
    assert abs(errors["requested_to_slewed"]["left"] - 10.0) < 1e-9
    assert abs(errors["requested_to_slewed"]["right"] - 20.0) < 1e-9
    assert abs(errors["requested_to_slewed"]["combined"] - np.sqrt(250.0)) < 1e-9
    requested_slope = metrics["command_slope_sps2"]["left_target_sps"]
    assert requested_slope["sample_count"] == sample_count - 1
    assert abs(requested_slope["max_abs"] - (2.0 * np.pi * frequency_hz * 1000.0)) < 100.0
    left_response = metrics["frequency_response_6_12_hz"]["left_requested_to_applied"]
    assert abs(left_response["frequency_hz"] - frequency_hz) < 0.05
    assert abs(left_response["magnitude_ratio"] - 1.0) < 0.01
    assert abs(left_response["phase_lag_deg"] - 16.2) < 0.5
    assert abs(left_response["lag_s"] - (delay_steps * dt_s)) < 2e-4
