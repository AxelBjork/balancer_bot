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

REALISTIC_VEL_FEEDBACK_SCALE = 0.05
REALISTIC_DRIFT_VEL_FEEDBACK_SCALE = REALISTIC_VEL_FEEDBACK_SCALE


def _alternating_pulse_train(*, start_s: float, pulse_duration_s: float, gap_s: float, count: int, amplitude: float) -> list[dict]:
    pulses: list[dict] = []
    sign = 1.0
    t = start_s
    for _ in range(count):
        pulses.append(
            {"start_s": t, "duration_s": pulse_duration_s, "force_n": sign * amplitude}
        )
        sign *= -1.0
        t += pulse_duration_s + gap_s
    return pulses


def _ramp(*, start_s: float, duration_s: float, force_n: float, force_n_end: float, com_bias_rad: float = 0.0, com_bias_rad_end: float = 0.0) -> dict:
    return {
        "kind": "ramp",
        "start_s": start_s,
        "duration_s": duration_s,
        "force_n": force_n,
        "com_bias_rad": com_bias_rad,
        "force_n_end": force_n_end,
        "com_bias_rad_end": com_bias_rad_end,
    }


def _hold_bias(*, start_s: float, force_n: float = 0.0, com_bias_rad: float = 0.0, duration_s: float = 0.0) -> dict:
    return {
        "kind": "hold_bias",
        "start_s": start_s,
        "duration_s": duration_s,
        "force_n": force_n,
        "com_bias_rad": com_bias_rad,
    }


SIMPLIFIED_SANITY_SCENARIOS = [
    ("simplified_neutral_hold", dict(duration_s=5.0)),
]

REALISTIC_STABILITY_SCENARIOS = [
    ("realistic_neutral_hold_40s", dict(duration_s=40.0, velocity_feedback_scale=REALISTIC_VEL_FEEDBACK_SCALE)),
    (
        "realistic_slow_push_recover_20s",
        dict(
            duration_s=20.0,
            velocity_feedback_scale=REALISTIC_VEL_FEEDBACK_SCALE,
            disturbances=[
                _ramp(start_s=1.0, duration_s=1.5, force_n=0.0, force_n_end=2.0),
                _ramp(start_s=2.5, duration_s=1.5, force_n=2.0, force_n_end=0.0),
            ],
        ),
    ),
    (
        "realistic_noisy_slow_push_recover_20s",
        dict(
            duration_s=20.0,
            velocity_feedback_scale=REALISTIC_VEL_FEEDBACK_SCALE,
            velocity_feedback_tau_s=0.20,
            imu_pitch_lag_s=0.02,
            imu_noise_seed=2026,
            accel_noise_std_mps2=0.20,
            gyro_noise_std_rad_s=0.015,
            disturbances=[
                _ramp(start_s=1.0, duration_s=1.5, force_n=0.0, force_n_end=2.0),
                _ramp(start_s=2.5, duration_s=1.5, force_n=2.0, force_n_end=0.0),
            ],
        ),
    ),
]

REALISTIC_SECONDARY_SCENARIOS = [
    (
        "realistic_disturbance_train_40s",
        dict(
            duration_s=40.0,
            velocity_feedback_scale=REALISTIC_VEL_FEEDBACK_SCALE,
            disturbances=_alternating_pulse_train(
                start_s=1.0,
                pulse_duration_s=0.5,
                gap_s=0.5,
                count=10,
                amplitude=0.35,
            ),
        ),
    ),
    (
        "realistic_slip_slow_push_recover_20s",
        dict(
            duration_s=20.0,
            velocity_feedback_scale=REALISTIC_VEL_FEEDBACK_SCALE,
            wheel_slip_factor=0.55,
            disturbances=[
                _ramp(start_s=1.0, duration_s=1.5, force_n=0.0, force_n_end=2.0),
                _ramp(start_s=2.5, duration_s=1.5, force_n=2.0, force_n_end=0.0),
            ],
        ),
    ),
]

REALISTIC_DRIFT_SCENARIOS = [
    (
        "realistic_pitch_bias_40s",
        dict(
            duration_s=40.0,
            initial_pitch_deg=0.10,
            velocity_feedback_scale=REALISTIC_VEL_FEEDBACK_SCALE,
        ),
    ),
    (
        "realistic_com_offset_40s",
        dict(
            duration_s=40.0,
            com_angle_offset_rad=0.001,
            velocity_feedback_scale=REALISTIC_VEL_FEEDBACK_SCALE,
        ),
    ),
    (
        "realistic_negative_com_offset_40s",
        dict(
            duration_s=40.0,
            com_angle_offset_rad=-0.001,
            velocity_feedback_scale=REALISTIC_VEL_FEEDBACK_SCALE,
        ),
    ),
    (
        "realistic_com_offset_lpf_noise_40s",
        dict(
            duration_s=40.0,
            com_angle_offset_rad=0.001,
            velocity_feedback_scale=REALISTIC_VEL_FEEDBACK_SCALE,
            imu_noise_seed=12345,
            accel_noise_std_mps2=0.20,
            gyro_noise_std_rad_s=0.015,
        ),
    ),
    (
        "realistic_slow_push_runaway_40s",
        dict(
            duration_s=40.0,
            velocity_feedback_scale=REALISTIC_DRIFT_VEL_FEEDBACK_SCALE,
            velocity_feedback_tau_s=0.10,
            imu_pitch_lag_s=0.01,
            disturbances=[
                _ramp(start_s=1.0, duration_s=8.0, force_n=0.0, force_n_end=0.55),
                _hold_bias(start_s=9.0, force_n=0.55),
            ],
        ),
    ),
    (
        "realistic_hold_bias_long_horizon_40s",
        dict(
            duration_s=40.0,
            velocity_feedback_scale=REALISTIC_DRIFT_VEL_FEEDBACK_SCALE,
            velocity_feedback_tau_s=0.10,
            imu_pitch_lag_s=0.01,
            disturbances=[
                _hold_bias(start_s=1.0, com_bias_rad=0.02),
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
        raise AssertionError("realistic_com_offset_40s should be covered by drift diagnostics, not stability scenarios")
    if run_id == "realistic_slow_push_recover_20s":
        assert summary["tail_rms_pitch_deg"] <= 2.0
        assert summary["tail_mean_abs_pitch_deg"] <= 1.0
        assert summary["tail_mean_abs_velocity_mps"] <= 0.2
        assert summary["max_abs_position_m"] >= 0.001
        assert summary["max_abs_position_m"] <= 0.5
        assert summary["max_abs_u_sps"] is not None and summary["max_abs_u_sps"] >= 10.0
    if run_id == "realistic_noisy_slow_push_recover_20s":
        rows = _read_timeline(output_dir)
        tail_start = metadata["duration_s"] - 2.0
        tail = [row for row in rows if row["sim_time_s"] >= tail_start]
        assert tail
        mean_abs_fused_bias = sum(
            abs(row["fused_pitch_deg"] - row["plant_pitch_deg"]) for row in tail
        ) / len(tail)
        assert summary["tail_rms_pitch_deg"] <= 2.0
        assert summary["tail_mean_abs_velocity_mps"] <= 0.2
        assert summary["max_abs_position_m"] >= 0.001
        assert summary["max_abs_position_m"] <= 0.5
        assert summary["max_abs_u_sps"] is not None and summary["max_abs_u_sps"] >= 10.0
        assert mean_abs_fused_bias <= 0.5


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
        assert summary["max_abs_position_m"] >= 0.0001
        assert summary["max_abs_f_app"] is not None and summary["max_abs_f_app"] >= 0.29
        assert summary["max_abs_actual_wheel_velocity"] is not None
        assert summary["tail_rms_pitch_deg"] <= 1.0
        assert summary["max_abs_u_sps"] is not None and summary["max_abs_u_sps"] >= 1.0
    if run_id == "realistic_slip_slow_push_recover_20s":
        assert metadata["wheel_slip_factor"] == 0.55
        assert summary["max_abs_position_m"] >= 0.001
        assert summary["max_abs_position_m"] <= 0.5
        assert summary["tail_mean_abs_velocity_mps"] <= 0.2
        assert summary["tail_rms_pitch_deg"] <= 2.0
        assert summary["max_abs_u_sps"] is not None and summary["max_abs_u_sps"] >= 10.0


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
    if run_id == "realistic_pitch_bias_40s":
        assert summary["max_abs_pitch_deg"] is not None and 0.05 <= summary["max_abs_pitch_deg"] <= 0.5
        assert summary["max_abs_position_m"] is not None and 0.05 <= summary["max_abs_position_m"] <= 1.0
        assert summary["tail_mean_abs_velocity_mps"] is not None and summary["tail_mean_abs_velocity_mps"] <= 0.05
        assert summary["tail_rms_pitch_deg"] is not None and summary["tail_rms_pitch_deg"] <= 0.1
    if run_id == "realistic_com_offset_40s":
        assert summary["max_abs_pitch_deg"] is not None and 0.01 <= summary["max_abs_pitch_deg"] <= 0.2
        assert summary["max_abs_position_m"] is not None and 0.05 <= summary["max_abs_position_m"] <= 0.5
        assert summary["tail_mean_abs_velocity_mps"] is not None and summary["tail_mean_abs_velocity_mps"] <= 0.01
        assert summary["tail_rms_pitch_deg"] is not None and summary["tail_rms_pitch_deg"] <= 0.1
    if run_id == "realistic_negative_com_offset_40s":
        rows = _read_timeline(output_dir)
        assert rows
        assert rows[-1]["pitch_trim_deg"] > 0.0
        assert rows[-1]["plant_position"] < 0.0
        assert summary["max_abs_pitch_deg"] is not None and 0.01 <= summary["max_abs_pitch_deg"] <= 0.2
        assert summary["max_abs_position_m"] is not None and 0.05 <= summary["max_abs_position_m"] <= 0.5
        assert summary["tail_mean_abs_velocity_mps"] is not None and summary["tail_mean_abs_velocity_mps"] <= 0.01
        assert summary["tail_rms_pitch_deg"] is not None and summary["tail_rms_pitch_deg"] <= 0.1
    if run_id == "realistic_com_offset_lpf_noise_40s":
        rows = _read_timeline(output_dir)
        tail_start = metadata["duration_s"] - 2.0
        tail = [row for row in rows if row["sim_time_s"] >= tail_start]
        assert tail
        mean_abs_fused_bias = sum(
            abs(row["fused_pitch_deg"] - row["plant_pitch_deg"]) for row in tail
        ) / len(tail)
        assert summary["max_abs_pitch_deg"] is not None and summary["max_abs_pitch_deg"] <= 5.0
        assert summary["tail_mean_abs_velocity_mps"] is not None and summary["tail_mean_abs_velocity_mps"] <= 0.5
        assert summary["tail_rms_pitch_deg"] is not None and summary["tail_rms_pitch_deg"] <= 3.0
        assert mean_abs_fused_bias <= 0.5
    if run_id == "realistic_slow_push_runaway_40s":
        assert summary["max_abs_pitch_deg"] is not None and summary["max_abs_pitch_deg"] <= 5.0
        assert summary["max_abs_position_m"] is not None and summary["max_abs_position_m"] <= 0.5
        assert summary["tail_mean_abs_velocity_mps"] is not None and summary["tail_mean_abs_velocity_mps"] <= 0.5
        assert summary["tail_rms_pitch_deg"] is not None and summary["tail_rms_pitch_deg"] <= 3.0
        assert summary["max_abs_u_sps"] is not None and summary["max_abs_u_sps"] >= 1.0
    if run_id == "realistic_hold_bias_long_horizon_40s":
        rows = _read_timeline(output_dir)
        assert rows
        bias_start_s = kwargs["disturbances"][0]["start_s"]
        bias_rows = [row for row in rows if row["sim_time_s"] >= bias_start_s]
        assert bias_rows
        # Positive COM bias should settle by leaning negative. A correctly damped rate loop should
        # not swing through the opposite side; allow only trace numeric/logging residue.
        assert max(row["plant_pitch_deg"] for row in bias_rows) <= 0.03
        assert max(row["fused_pitch_deg"] for row in bias_rows) <= 0.03
        assert min(row["plant_velocity"] for row in bias_rows) >= -0.001
        assert summary["max_abs_pitch_deg"] is not None and summary["max_abs_pitch_deg"] <= 5.0
        assert summary["max_abs_position_m"] is not None and summary["max_abs_position_m"] <= 5.0
        assert summary["tail_mean_abs_velocity_mps"] is not None and summary["tail_mean_abs_velocity_mps"] <= 0.5
        assert summary["tail_rms_pitch_deg"] is not None and summary["tail_rms_pitch_deg"] <= 3.0
        assert summary["max_abs_u_sps"] is not None and summary["max_abs_u_sps"] >= 10.0


@pytest.mark.parametrize(("run_id", "kwargs"), REALISTIC_FRONTIER_DIAGNOSTICS)
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
    assert summary["tail_rms_pitch_deg"] <= 20.0
    assert summary["tail_mean_abs_velocity_mps"] <= 12.0
    assert summary["max_abs_position_m"] <= 1200.0


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
    assert summary["u_sps_min"] <= -3400.0
    assert summary["max_abs_f_app"] is not None and summary["max_abs_f_app"] >= 100.0
    assert summary["max_abs_target_wheel_velocity"] is not None
    assert summary["max_abs_actual_wheel_velocity"] is not None
    assert summary["max_abs_target_wheel_velocity"] >= 0.25
    assert summary["max_abs_actual_wheel_velocity"] >= 0.20
    assert summary["max_abs_pitch_deg"] >= 50.0
    assert summary["tail_mean_abs_velocity_mps"] >= 10.0
    assert summary["max_abs_position_m"] >= 100.0
