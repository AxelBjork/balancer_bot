from __future__ import annotations

from pathlib import Path

from tools.run_artifacts import RunRecorder, summarize_reference_file


def test_run_recorder_writes_summary_and_flags_fall(tmp_path):
    recorder = RunRecorder()
    recorder.begin_run({"run_id": "unit_recorder"})
    recorder.record_step(
        {
            "sim_time_s": 0.0,
            "pitch_deg": 0.0,
            "u_sps": 0.0,
            "plant_pitch_deg": 0.0,
        }
    )
    recorder.record_step(
        {
            "sim_time_s": 0.01,
            "pitch_deg": 90.0,
            "u_sps": 100.0,
            "plant_pitch_deg": 90.0,
        }
    )

    summary = recorder.write_csv_json_plots(tmp_path)
    assert summary["fell"]
    assert summary["max_abs_pitch_deg"] == 90.0
    assert (tmp_path / "timeline.csv").exists()
    assert (tmp_path / "summary.json").exists()
    assert (tmp_path / "pitch_plot.svg").exists()
    assert (tmp_path / "command_plot.svg").exists()


def test_reference_parser_handles_clean_csv():
    summary = summarize_reference_file(Path("data") / "imu_data.csv")
    assert summary["sample_count"] > 100
    assert summary["time_field"] == "time"
    assert summary["dt_max"] is not None


def test_reference_parser_handles_mixed_logs():
    for name in ("imu_held.csv", "balance_raw.csv", "balance.csv"):
        summary = summarize_reference_file(Path("data") / name)
        assert summary["sample_count"] > 100
        assert "time_field" in summary
