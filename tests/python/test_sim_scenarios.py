from __future__ import annotations

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


SIMPLIFIED_SANITY_SCENARIOS = [
    ("simplified_neutral_hold", dict(duration_s=5.0)),
]

REALISTIC_STABILITY_SCENARIOS = [
    ("realistic_neutral_hold_20s", dict(duration_s=20.0)),
    ("realistic_pitch_bias_20s", dict(initial_pitch_deg=0.10, duration_s=20.0)),
    ("realistic_com_offset_20s", dict(com_angle_offset_rad=0.001, duration_s=20.0)),
    ("realistic_disturbance_pulse_20s", dict(duration_s=20.0, disturbance={"start_s": 1.0, "duration_s": 0.20, "forward": 0.08, "turn": 0.0})),
]

REALISTIC_FRONTIER_DIAGNOSTICS = [
    ("frontier_combined_0p45_0p0025_20s", dict(initial_pitch_deg=0.45, com_angle_offset_rad=0.0025, duration_s=20.0)),
]


def _artifact_dir(sim_artifact_settings, run_id: str) -> Path:
    output_dir = Path(sim_artifact_settings["temp_root"]) / run_id
    if output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir


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
    summary, metadata, done = run_scenario_live(
        simulator_udp,
        run_id=2000 + REALISTIC_STABILITY_SCENARIOS.index((run_id, kwargs)),
        output_dir=_artifact_dir(sim_artifact_settings, run_id),
        physics_profile=PHYSICS_REALISTIC,
        **kwargs,
    )
    assert metadata["physics_profile"] == "realistic"
    _assert_pass_criteria(summary, metadata, done)
    if run_id == "realistic_com_offset_20s":
        assert summary["tail_mean_abs_pitch_deg"] <= 1.0
        assert summary["max_abs_position_m"] <= 3.0
        assert summary["tail_mean_abs_velocity_mps"] <= 0.06
    if run_id == "realistic_disturbance_pulse_20s":
        assert summary["tail_rms_pitch_deg"] <= 0.05


@pytest.mark.parametrize(("run_id", "kwargs"), REALISTIC_FRONTIER_DIAGNOSTICS)
@pytest.mark.xfail(strict=True, reason="realistic profile still has unresolved harder scenarios")
def test_realistic_profile_frontier_diagnostics(simulator_udp, sim_artifact_settings, run_id: str, kwargs: dict):
    summary, metadata, done = run_scenario_live(
        simulator_udp,
        run_id=3000 + REALISTIC_FRONTIER_DIAGNOSTICS.index((run_id, kwargs)),
        output_dir=_artifact_dir(sim_artifact_settings, run_id),
        physics_profile=PHYSICS_REALISTIC,
        **kwargs,
    )
    assert metadata["physics_profile"] == "realistic"
    _assert_pass_criteria(summary, metadata, done)


def test_realistic_fail_fast_stop_on_fall(simulator_udp, sim_artifact_settings):
    summary, _metadata, done = run_scenario_live(
        simulator_udp,
        run_id=4001,
        output_dir=_artifact_dir(sim_artifact_settings, "realistic_fail_fast_stop"),
        physics_profile=PHYSICS_REALISTIC,
        initial_pitch_deg=10.0,
        duration_s=20.0,
    )
    assert summary["sample_count"] > 0
    assert done.reason_code in (DONE_FELL, DONE_COMPLETED)
