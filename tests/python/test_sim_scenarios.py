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


def _ramp(*, start_s: float, duration_s: float, forward: float, forward_end: float, turn: float = 0.0, turn_end: float = 0.0) -> dict:
    return {
        "kind": "ramp",
        "start_s": start_s,
        "duration_s": duration_s,
        "forward": forward,
        "turn": turn,
        "forward_end": forward_end,
        "turn_end": turn_end,
    }


def _hold_bias(*, start_s: float, forward: float, turn: float = 0.0, duration_s: float = 0.0) -> dict:
    return {
        "kind": "hold_bias",
        "start_s": start_s,
        "duration_s": duration_s,
        "forward": forward,
        "turn": turn,
    }


SIMPLIFIED_SANITY_SCENARIOS = [
    ("simplified_neutral_hold", dict(duration_s=5.0)),
]

REALISTIC_STABILITY_SCENARIOS = [
    ("realistic_neutral_hold_40s", dict(duration_s=40.0)),
    ("realistic_pitch_bias_40s", dict(initial_pitch_deg=0.10, duration_s=40.0)),
    ("realistic_com_offset_40s", dict(com_angle_offset_rad=0.001, duration_s=40.0)),
    (
        "realistic_slow_push_recover_20s",
        dict(
            duration_s=20.0,
            disturbances=[
                _ramp(start_s=1.0, duration_s=0.8, forward=0.0, forward_end=0.12),
                _ramp(start_s=1.8, duration_s=0.8, forward=0.12, forward_end=0.0),
            ],
        ),
    ),
]

REALISTIC_SECONDARY_SCENARIOS = [
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

REALISTIC_DRIFT_SCENARIOS = [
    (
        "realistic_slow_push_runaway_40s",
        dict(
            duration_s=40.0,
            velocity_feedback_scale=0.85,
            velocity_feedback_tau_s=0.10,
            imu_pitch_lag_s=0.01,
            disturbances=[
                _ramp(start_s=1.0, duration_s=8.0, forward=0.0, forward_end=0.18),
                _hold_bias(start_s=9.0, forward=0.18),
            ],
        ),
    ),
    (
        "realistic_hold_bias_long_horizon_40s",
        dict(
            duration_s=40.0,
            velocity_feedback_scale=0.85,
            velocity_feedback_tau_s=0.10,
            imu_pitch_lag_s=0.01,
            disturbances=[
                _hold_bias(start_s=1.0, forward=0.10),
            ],
        ),
    ),
]

REALISTIC_FRONTIER_DIAGNOSTICS = [
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
    if run_id == "realistic_slow_push_recover_20s":
        assert summary["max_abs_pitch_deg"] >= 0.25
        assert summary["tail_rms_pitch_deg"] <= 2.0
        assert summary["tail_mean_abs_velocity_mps"] <= 1.0
        assert summary["max_abs_position_m"] <= 1.0


@pytest.mark.parametrize(("run_id", "kwargs"), REALISTIC_SECONDARY_SCENARIOS)
def test_realistic_profile_secondary_scenarios(simulator_udp, sim_artifact_settings, run_id: str, kwargs: dict):
    output_dir = _artifact_dir(sim_artifact_settings, run_id)
    summary, metadata, done = run_scenario_live(
        simulator_udp,
        run_id=2500 + REALISTIC_SECONDARY_SCENARIOS.index((run_id, kwargs)),
        output_dir=output_dir,
        physics_profile=PHYSICS_REALISTIC,
        **kwargs,
    )
    assert metadata["physics_profile"] == "realistic"
    _assert_pass_criteria(summary, metadata, done)
    if run_id == "realistic_disturbance_train_40s":
        assert summary["max_abs_pitch_deg"] >= 0.15
        assert summary["max_abs_f_app"] is not None and summary["max_abs_f_app"] >= 0.5
        assert summary["max_abs_actual_wheel_velocity"] is not None
        assert summary["tail_rms_pitch_deg"] <= 2.0


@pytest.mark.parametrize(("run_id", "kwargs"), REALISTIC_DRIFT_SCENARIOS)
def test_realistic_profile_drift_diagnostics(simulator_udp, sim_artifact_settings, run_id: str, kwargs: dict):
    output_dir = _artifact_dir(sim_artifact_settings, run_id)
    summary, metadata, done = run_scenario_live(
        simulator_udp,
        run_id=3000 + REALISTIC_DRIFT_SCENARIOS.index((run_id, kwargs)),
        output_dir=output_dir,
        physics_profile=PHYSICS_REALISTIC,
        **kwargs,
    )
    assert metadata["physics_profile"] == "realistic"
    _assert_common_integrity(summary, metadata, done)
    assert done.reason_code == DONE_COMPLETED
    assert not summary["fell"]
    if run_id == "realistic_slow_push_runaway_40s":
        assert summary["max_abs_pitch_deg"] is not None and summary["max_abs_pitch_deg"] >= 3.0
        assert summary["max_abs_position_m"] is not None and summary["max_abs_position_m"] >= 10.0
        assert summary["tail_mean_abs_velocity_mps"] is not None and summary["tail_mean_abs_velocity_mps"] >= 1.5
        assert summary["tail_rms_pitch_deg"] is not None and summary["tail_rms_pitch_deg"] >= 2.0
        assert summary["max_abs_u_sps"] is not None and summary["max_abs_u_sps"] >= 1000.0
    if run_id == "realistic_hold_bias_long_horizon_40s":
        assert summary["max_abs_pitch_deg"] is not None and summary["max_abs_pitch_deg"] >= 3.0
        assert summary["max_abs_position_m"] is not None and summary["max_abs_position_m"] >= 10.0
        assert summary["tail_mean_abs_velocity_mps"] is not None and summary["tail_mean_abs_velocity_mps"] >= 1.5
        assert summary["tail_rms_pitch_deg"] is not None and summary["tail_rms_pitch_deg"] >= 2.0
        assert summary["max_abs_u_sps"] is not None and summary["max_abs_u_sps"] >= 1000.0


@pytest.mark.parametrize(("run_id", "kwargs"), REALISTIC_FRONTIER_DIAGNOSTICS)
@pytest.mark.xfail(strict=True, reason="realistic profile still has unresolved large-angle recovery")
def test_realistic_profile_frontier_diagnostics(simulator_udp, sim_artifact_settings, run_id: str, kwargs: dict):
    output_dir = _artifact_dir(sim_artifact_settings, run_id)
    summary, metadata, done = run_scenario_live(
        simulator_udp,
        run_id=3500 + REALISTIC_FRONTIER_DIAGNOSTICS.index((run_id, kwargs)),
        output_dir=output_dir,
        physics_profile=PHYSICS_REALISTIC,
        **kwargs,
    )
    assert metadata["physics_profile"] == "realistic"
    _assert_common_integrity(summary, metadata, done)
    assert done.reason_code == DONE_COMPLETED
    assert not summary["fell"]
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
