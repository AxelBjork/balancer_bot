from __future__ import annotations

import csv
import json
import shutil
import subprocess
from pathlib import Path

import pytest

SIMPLIFIED_SANITY_SCENARIOS = [
    "neutral_hold",
    "combined_bias_pos",
]

REALISTIC_STABILITY_SCENARIOS = [
    "neutral_hold",
    "pitch_bias_pos",
    "com_offset_pos",
    "combined_bias_pos",
    "recovery_large_pitch_pos",
]

REALISTIC_STRETCH_PASSING = [
    (
        "stretch_recovery_0p5_pos",
        "neutral_hold",
        ["--initial-pitch-deg", "0.5"],
    ),
    (
        "stretch_combined_0p25_0p0015_pos",
        "neutral_hold",
        ["--initial-pitch-deg", "0.25", "--com-angle-offset-rad", "0.0015"],
    ),
]

REALISTIC_STRETCH_DIAGNOSTICS = [
    (
        "frontier_recovery_0p75_pos",
        "neutral_hold",
        ["--initial-pitch-deg", "0.75"],
    ),
    (
        "frontier_combined_0p45_0p0025_pos",
        "neutral_hold",
        ["--initial-pitch-deg", "0.45", "--com-angle-offset-rad", "0.0025"],
    ),
]

REALISTIC_LONG_HORIZON_DIAGNOSTICS = [
    (
        "long_horizon_recovery_0p5_20s",
        "neutral_hold",
        ["--initial-pitch-deg", "0.5", "--duration-s", "20"],
    ),
]


def _artifact_dir(sim_artifact_settings, run_id: str) -> Path:
    output_dir = Path(sim_artifact_settings["temp_root"]) / run_id
    if output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir


def _run_scenario(
    simulator_binary: Path,
    sim_artifact_settings,
    scenario_name: str,
    physics_profile: str,
    *,
    run_id: str | None = None,
    extra_args: list[str] | None = None,
) -> tuple[subprocess.CompletedProcess[str], dict, dict, Path]:
    run_key = run_id or f"{physics_profile}_{scenario_name}"
    output_dir = _artifact_dir(sim_artifact_settings, run_key)
    cmd = [
        str(simulator_binary),
        "--scenario",
        scenario_name,
        "--physics-profile",
        physics_profile,
        "--output-dir",
        str(output_dir),
    ]
    if extra_args:
        cmd.extend(extra_args)
    proc = subprocess.run(
        cmd,
        cwd=Path.cwd(),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )

    summary = json.loads((output_dir / "summary.json").read_text(encoding="utf-8"))
    metadata = json.loads((output_dir / "metadata.json").read_text(encoding="utf-8"))
    return proc, summary, metadata, output_dir


def _load_timeline(output_dir: Path) -> list[dict[str, str]]:
    with (output_dir / "timeline.csv").open(encoding="utf-8", newline="") as fh:
        return list(csv.DictReader(fh))


def _count_nonzero_sign_changes(values: list[float]) -> int:
    changes = 0
    previous_sign = 0
    for value in values:
        sign = 0
        if value > 1e-9:
            sign = 1
        elif value < -1e-9:
            sign = -1
        if sign == 0:
            continue
        if previous_sign != 0 and sign != previous_sign:
            changes += 1
        previous_sign = sign
    return changes


def _assert_common_integrity(summary: dict, metadata: dict) -> None:
    assert summary["sample_count"] > 0
    assert summary["telemetry_continuous"]
    assert summary["max_abs_pitch_deg"] is not None
    assert summary["max_abs_pitch_deg"] == summary["max_abs_pitch_deg"]
    assert summary["tail_rms_pitch_deg"] is not None
    assert summary["tail_rail_fraction"] is not None
    assert metadata["pid_profile"].endswith("pid_sim.conf")


def _assert_pass_criteria(proc: subprocess.CompletedProcess[str], summary: dict, metadata: dict) -> None:
    _assert_common_integrity(summary, metadata)
    assert proc.returncode == 0, proc.stderr
    assert not summary["fell"]
    assert summary["max_abs_pitch_deg"] <= 75.0
    assert summary["tail_rms_pitch_deg"] <= 3.0
    assert summary["tail_rail_fraction"] <= 0.05


@pytest.mark.parametrize("scenario_name", SIMPLIFIED_SANITY_SCENARIOS)
def test_simplified_sanity_scenarios(simulator_binary, sim_artifact_settings, scenario_name: str):
    proc, summary, metadata, _ = _run_scenario(
        simulator_binary,
        sim_artifact_settings,
        scenario_name,
        "simplified",
    )
    assert metadata["physics_profile"] == "simplified"
    _assert_pass_criteria(proc, summary, metadata)


@pytest.mark.parametrize("scenario_name", REALISTIC_STABILITY_SCENARIOS)
def test_realistic_profile_stability_scenarios(simulator_binary, sim_artifact_settings, scenario_name: str):
    proc, summary, metadata, _ = _run_scenario(
        simulator_binary,
        sim_artifact_settings,
        scenario_name,
        "realistic",
    )
    assert metadata["physics_profile"] == "realistic"
    _assert_pass_criteria(proc, summary, metadata)


def test_realistic_disturbance_pulse_is_damped(simulator_binary, sim_artifact_settings):
    proc, summary, metadata, output_dir = _run_scenario(
        simulator_binary,
        sim_artifact_settings,
        "disturbance_pulse",
        "realistic",
        run_id="realistic_disturbance_pulse",
    )
    assert metadata["physics_profile"] == "realistic"
    _assert_pass_criteria(proc, summary, metadata)
    assert summary["tail_rms_pitch_deg"] <= 0.05

    timeline = _load_timeline(output_dir)
    post_disturbance = [
        float(row["u_sps"])
        for row in timeline
        if float(row["sim_time_s"]) >= 1.2
    ]
    assert _count_nonzero_sign_changes(post_disturbance) <= 6


@pytest.mark.parametrize(("run_id", "scenario_name", "extra_args"), REALISTIC_STRETCH_PASSING)
def test_realistic_profile_stretch_passing(
    simulator_binary,
    sim_artifact_settings,
    run_id: str,
    scenario_name: str,
    extra_args: list[str],
):
    proc, summary, metadata, _ = _run_scenario(
        simulator_binary,
        sim_artifact_settings,
        scenario_name,
        "realistic",
        run_id=run_id,
        extra_args=extra_args,
    )
    assert metadata["physics_profile"] == "realistic"
    _assert_pass_criteria(proc, summary, metadata)


@pytest.mark.parametrize(("run_id", "scenario_name", "extra_args"), REALISTIC_STRETCH_DIAGNOSTICS)
@pytest.mark.xfail(strict=True, reason="realistic profile remains upright but does not yet settle the next harder tier")
def test_realistic_profile_stretch_diagnostics(
    simulator_binary,
    sim_artifact_settings,
    run_id: str,
    scenario_name: str,
    extra_args: list[str],
):
    proc, summary, metadata, _ = _run_scenario(
        simulator_binary,
        sim_artifact_settings,
        scenario_name,
        "realistic",
        run_id=run_id,
        extra_args=extra_args,
    )
    assert metadata["physics_profile"] == "realistic"
    _assert_pass_criteria(proc, summary, metadata)


@pytest.mark.parametrize(("run_id", "scenario_name", "extra_args"), REALISTIC_LONG_HORIZON_DIAGNOSTICS)
@pytest.mark.xfail(strict=True, reason="realistic profile still shows long-horizon drift on representative recovery runs")
def test_realistic_profile_long_horizon_diagnostics(
    simulator_binary,
    sim_artifact_settings,
    run_id: str,
    scenario_name: str,
    extra_args: list[str],
):
    proc, summary, metadata, _ = _run_scenario(
        simulator_binary,
        sim_artifact_settings,
        scenario_name,
        "realistic",
        run_id=run_id,
        extra_args=extra_args,
    )
    assert metadata["physics_profile"] == "realistic"
    _assert_pass_criteria(proc, summary, metadata)
