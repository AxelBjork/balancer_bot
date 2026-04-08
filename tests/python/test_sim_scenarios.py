from __future__ import annotations

import csv
import shutil
from pathlib import Path

import pytest

from tests.python.support.simulator_service import (
    DONE_COMPLETED,
    DONE_FELL,
    PHYSICS_REALISTIC,
    PHYSICS_SIMPLIFIED,
    run_scenario_live,
)


def _alternating_pulse_train(*, start_s: float, pulse_duration_s: float, gap_s: float, count: int, amplitude: float) -> list[dict]:
    pulses: list[dict] = []
    sign = 1.0
    t = start_s
    for _ in range(count):
        pulses.append(
            {"start_s": t, "duration_s": pulse_duration_s, "forward": sign * amplitude, "turn": 0.0}
        )
        sign *= -1.0
        t += pulse_duration_s + gap_s
    return pulses


SIMPLIFIED_SANITY_SCENARIOS = [
    ("simplified_neutral_hold", dict(duration_s=5.0)),
]

REALISTIC_STABILITY_SCENARIOS = [
    ("realistic_neutral_hold_40s", dict(duration_s=40.0)),
    ("realistic_pitch_bias_40s", dict(initial_pitch_deg=0.10, duration_s=40.0)),
    ("realistic_com_offset_40s", dict(com_angle_offset_rad=0.001, duration_s=40.0)),
    (
        "realistic_disturbance_pulse_40s",
        dict(
            duration_s=40.0,
            disturbances=[
                {"start_s": 1.0, "duration_s": 0.50, "forward": 1.0, "turn": 0.0},
            ],
        ),
    ),
    (
        "realistic_disturbance_train_40s",
        dict(
            duration_s=40.0,
            disturbances=_alternating_pulse_train(
                start_s=1.0,
                pulse_duration_s=0.5,
                gap_s=0.5,
                count=10,
                amplitude=1.0,
            ),
        ),
    ),
]

REALISTIC_FRONTIER_DIAGNOSTICS = [
    ("frontier_combined_0p45_0p0025_40s", dict(initial_pitch_deg=0.45, com_angle_offset_rad=0.0025, duration_s=40.0)),
    (
        "frontier_disturbance_reversal_40s",
        dict(
            duration_s=40.0,
            disturbances=[
                {"start_s": 1.0, "duration_s": 0.50, "forward": 1.0, "turn": 0.0},
                {"start_s": 2.0, "duration_s": 0.50, "forward": -1.0, "turn": 0.0},
            ],
        ),
    ),
    ("frontier_recovery_67deg_40s", dict(initial_pitch_deg=67.0, duration_s=40.0)),
]


def _artifact_dir(sim_artifact_settings, run_id: str) -> Path:
    output_dir = Path(sim_artifact_settings["temp_root"]) / run_id
    if output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir


def _read_timeline(output_dir: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with (output_dir / "timeline.csv").open("r", encoding="utf-8", newline="") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            rows.append({key: float(value) for key, value in row.items() if value not in (None, "")})
    return rows


def _assert_common_integrity(summary: dict, metadata: dict, done) -> None:
    assert summary["sample_count"] > 0
    assert summary["telemetry_continuous"]
    assert summary["max_abs_pitch_deg"] is not None
    assert summary["tail_rms_pitch_deg"] is not None
    assert summary["tail_rail_fraction"] is not None
    assert metadata["pid_profile"].endswith("pid_sim.conf")
    assert done.sample_count > 0


def _assert_pass_criteria(summary: dict, metadata: dict, done) -> None:
    _assert_common_integrity(summary, metadata, done)
    assert done.reason_code == DONE_COMPLETED
    assert not summary["fell"]
    assert summary["max_abs_pitch_deg"] <= 75.0
    assert summary["tail_rms_pitch_deg"] <= 3.0
    assert summary["tail_rail_fraction"] <= 0.05


@pytest.mark.parametrize(("run_id", "kwargs"), SIMPLIFIED_SANITY_SCENARIOS)
def test_simplified_sanity_scenarios(simulator_udp, sim_artifact_settings, run_id: str, kwargs: dict):
    summary, metadata, done = run_scenario_live(
        simulator_udp,
        run_id=1000 + SIMPLIFIED_SANITY_SCENARIOS.index((run_id, kwargs)),
        output_dir=_artifact_dir(sim_artifact_settings, run_id),
        physics_profile=PHYSICS_SIMPLIFIED,
        **kwargs,
    )
    assert metadata["physics_profile"] == "simplified"
    _assert_pass_criteria(summary, metadata, done)


@pytest.mark.parametrize(("run_id", "kwargs"), REALISTIC_STABILITY_SCENARIOS)
def test_realistic_profile_stability_scenarios(simulator_udp, sim_artifact_settings, run_id: str, kwargs: dict):
    output_dir = _artifact_dir(sim_artifact_settings, run_id)
    summary, metadata, done = run_scenario_live(
        simulator_udp,
        run_id=2000 + REALISTIC_STABILITY_SCENARIOS.index((run_id, kwargs)),
        output_dir=output_dir,
        physics_profile=PHYSICS_REALISTIC,
        **kwargs,
    )
    assert metadata["physics_profile"] == "realistic"
    _assert_pass_criteria(summary, metadata, done)
    if run_id == "realistic_com_offset_40s":
        assert summary["tail_mean_abs_pitch_deg"] <= 1.5
        assert summary["max_abs_position_m"] <= 15.0
        assert summary["tail_mean_abs_velocity_mps"] <= 2.0
    if run_id == "realistic_disturbance_pulse_40s":
        assert summary["max_abs_pitch_deg"] >= 0.10
        assert summary["tail_rms_pitch_deg"] <= 2.0
    if run_id == "realistic_disturbance_train_40s":
        assert summary["max_abs_pitch_deg"] >= 0.15
        assert summary["max_abs_f_app"] is not None and summary["max_abs_f_app"] >= 0.5
        assert summary["max_abs_actual_wheel_velocity"] is not None
        assert summary["tail_rms_pitch_deg"] <= 2.0


@pytest.mark.parametrize(("run_id", "kwargs"), REALISTIC_FRONTIER_DIAGNOSTICS)
@pytest.mark.xfail(strict=True, reason="realistic profile still has unresolved harder scenarios")
def test_realistic_profile_frontier_diagnostics(simulator_udp, sim_artifact_settings, run_id: str, kwargs: dict):
    output_dir = _artifact_dir(sim_artifact_settings, run_id)
    summary, metadata, done = run_scenario_live(
        simulator_udp,
        run_id=3000 + REALISTIC_FRONTIER_DIAGNOSTICS.index((run_id, kwargs)),
        output_dir=output_dir,
        physics_profile=PHYSICS_REALISTIC,
        **kwargs,
    )
    assert metadata["physics_profile"] == "realistic"
    _assert_common_integrity(summary, metadata, done)
    assert done.reason_code == DONE_COMPLETED
    assert not summary["fell"]
    assert summary["max_abs_pitch_deg"] <= 75.0
    if run_id == "frontier_combined_0p45_0p0025_40s":
        assert summary["tail_rms_pitch_deg"] <= 3.0
        assert summary["tail_mean_abs_velocity_mps"] <= 1.0
        assert summary["max_abs_position_m"] <= 15.0
    if run_id == "frontier_disturbance_reversal_40s":
        rows = _read_timeline(output_dir)
        first_peak = min(
            row["plant_pitch_deg"] for row in rows if 1.0 <= row["sim_time_s"] < 2.0
        )
        second_peak = max(
            row["plant_pitch_deg"] for row in rows if row["sim_time_s"] >= 2.0
        )
        assert first_peak <= -0.10
        assert second_peak > 0.0
        assert second_peak <= (abs(first_peak) + 0.05)
        assert summary["tail_rms_pitch_deg"] <= 0.20
    if run_id == "frontier_recovery_67deg_40s":
        assert summary["tail_rms_pitch_deg"] <= 3.0
        assert summary["tail_mean_abs_velocity_mps"] <= 0.5
        assert summary["max_abs_position_m"] <= 10.0


def test_realistic_fail_fast_stop_on_fall(simulator_udp, sim_artifact_settings):
    summary, _metadata, done = run_scenario_live(
        simulator_udp,
        run_id=4001,
        output_dir=_artifact_dir(sim_artifact_settings, "realistic_fail_fast_stop"),
        physics_profile=PHYSICS_REALISTIC,
        initial_pitch_deg=10.0,
        duration_s=40.0,
    )
    assert summary["sample_count"] > 0
    assert done.reason_code in (DONE_FELL, DONE_COMPLETED)


def test_realistic_large_tilt_diagnostic_self_rights_but_still_runs_away(
    simulator_udp, sim_artifact_settings
):
    summary, metadata, done = run_scenario_live(
        simulator_udp,
        run_id=4101,
        output_dir=_artifact_dir(sim_artifact_settings, "realistic_large_tilt_diagnostic_67deg"),
        physics_profile=PHYSICS_REALISTIC,
        initial_pitch_deg=67.0,
        duration_s=5.0,
    )
    assert metadata["physics_profile"] == "realistic"
    assert done.reason_code == DONE_COMPLETED
    assert summary["sample_count"] > 0
    assert summary["u_sps_min"] <= -15990.0
    assert summary["max_abs_f_app"] is not None and summary["max_abs_f_app"] >= 100.0
    assert summary["max_abs_target_wheel_velocity"] is not None
    assert summary["max_abs_actual_wheel_velocity"] is not None
    assert summary["max_abs_target_wheel_velocity"] >= 1.2
    assert summary["max_abs_actual_wheel_velocity"] >= 1.0
    assert summary["max_abs_pitch_deg"] >= 50.0
    assert summary["tail_mean_abs_velocity_mps"] >= 10.0
    assert summary["max_abs_position_m"] >= 100.0
