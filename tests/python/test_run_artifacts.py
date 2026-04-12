from __future__ import annotations

from pathlib import Path

from tests.python.support.run_artifacts import (
    RunRecorder,
    analyze_timeline_rows,
    estimate_lag_scale,
    summarize_reference_file,
)


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
    assert (tmp_path / "pitch_plot.svg").exists()
    assert (tmp_path / "command_plot.svg").exists()
    assert (tmp_path / "wheel_plot.svg").exists()
    assert (tmp_path / "force_plot.svg").exists()


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
                "measured_vel_sps": response,
            }
        )

    estimate = estimate_lag_scale(rows, "u_sps", "measured_vel_sps", max_lag_s=0.2, min_abs_source=10.0)
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
        rows.append(
            {
                "sim_time_s": t,
                "raw_acc_pitch_deg": raw_pitch,
                "fused_pitch_deg": fused_pitch,
                "gyro_pitch_rate_dps": gyro,
                "filtered_pitch_rate_dps": filtered_gyro,
                "u_sps": command,
                "measured_vel_sps": measured,
                "filtered_vel_sps": measured,
            }
        )

    analysis = analyze_timeline_rows(rows)
    assert analysis["time_key"] == "sim_time_s"
    assert analysis["estimator_pitch"]["lag_steps"] == 2
    assert analysis["estimator_pitch_rate"]["lag_steps"] == 1
    assert analysis["drive_command_to_measured_velocity"]["lag_steps"] == 4
