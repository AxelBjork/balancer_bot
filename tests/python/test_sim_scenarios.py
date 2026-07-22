from __future__ import annotations

import csv
import math
import shutil
from pathlib import Path

import pytest

from tests.python.support.simulator_service import (
    DONE_COMPLETED,
    PHYSICS_REALISTIC,
    run_scenario_live,
)


def test_udp_transfer_smoke_uses_downsampled_telemetry(
    simulator_udp, sim_artifact_settings: dict
):
    output = Path(sim_artifact_settings["temp_root"]) / "udp_transfer_smoke"
    summary, metadata, done = run_scenario_live(
        simulator_udp,
        run_id=1000,
        output_dir=output,
        physics_profile=PHYSICS_REALISTIC,
        duration_s=2.0,
        telemetry_stride=20,
        disturbances=[{"start_s": 0.5, "duration_s": 0.1, "force_n": 0.01}],
    )

    assert metadata["telemetry_stride"] == 20
    assert done.reason_code == DONE_COMPLETED
    assert done.sample_count == 800
    assert done.actuator_fault_count == 0
    assert done.controller_fault_flags == 0
    assert summary["max_abs_pitch_deg"] < 15.0
    assert 35 <= summary["sample_count"] <= 45


def test_udp_physics_override_reports_complete_profile(
    simulator_udp, sim_artifact_settings: dict
):
    output = Path(sim_artifact_settings["temp_root"]) / "udp_complete_physics_override"
    summary, _metadata, done = run_scenario_live(
        simulator_udp,
        run_id=1001,
        output_dir=output,
        physics_profile=PHYSICS_REALISTIC,
        duration_s=0.5,
        telemetry_stride=40,
        physics_override={
            "motor_no_load_speed_mps": 1.1,
            "motor_velocity_damping": 35.0,
            "motor_tau_s": 0.010,
            "traction_coefficient": 0.9,
            "pitch_damping": 0.03,
            "cart_damping": 0.8,
            "phase_error_limit_steps": 12.0,
            "tire_stiffness_n_per_m": 2800.0,
            "tire_damping_n_s_per_m": 31.0,
            "wheel_equivalent_mass_kg": 0.12,
        },
    )

    assert done.reason_code == DONE_COMPLETED
    for field, expected in {
        "motor_no_load_speed_mps": 1.1,
        "motor_velocity_damping": 35.0,
        "motor_tau_s": 0.010,
        "traction_coefficient": 0.9,
        "pitch_damping": 0.03,
        "cart_damping": 0.8,
        "phase_error_limit_steps": 12.0,
        "tire_stiffness_n_per_m": 2800.0,
        "tire_damping_n_s_per_m": 31.0,
        "wheel_equivalent_mass_kg": 0.12,
    }.items():
        assert math.isclose(float(summary[field]), expected, rel_tol=0.0, abs_tol=1e-5)


def _alternating_pulse_train(
    *, start_s: float, pulse_duration_s: float, count: int, amplitude: float
) -> list[dict]:
    return [
        {
            "start_s": start_s + index * pulse_duration_s,
            "duration_s": pulse_duration_s,
            "force_n": amplitude if index % 2 == 0 else -amplitude,
        }
        for index in range(count)
    ]


def _ramp(
    *, start_s: float, duration_s: float, force_n: float, force_n_end: float
) -> dict:
    return {
        "kind": "ramp",
        "start_s": start_s,
        "duration_s": duration_s,
        "force_n": force_n,
        "force_n_end": force_n_end,
    }


def _artifact_dir(sim_artifact_settings: dict, name: str) -> Path:
    output_dir = Path(sim_artifact_settings["temp_root"]) / name
    if output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir


def _read_timeline(output_dir: Path) -> list[dict[str, float | bool]]:
    def parse_value(value: str) -> float | bool:
        if value in ("True", "False"):
            return value == "True"
        return float(value)

    with (output_dir / "timeline.csv").open("r", encoding="utf-8", newline="") as stream:
        return [
            {
                key: parse_value(value)
                for key, value in row.items()
                if value not in (None, "")
            }
            for row in csv.DictReader(stream)
        ]


def _assert_common_integrity(
    summary: dict,
    metadata: dict,
    done,
    *,
    expected_physics_override: dict | None = None,
) -> None:
    assert summary["sample_count"] > 0
    assert summary["telemetry_continuous"]
    assert summary["max_abs_pitch_deg"] is not None
    assert summary["tail_rms_pitch_deg"] is not None
    assert summary["tail_rail_fraction"] is not None
    assert metadata["pid_profile"].endswith("pid.conf")
    assert metadata["physics_profile"] == "realistic"
    assert metadata["total_mass_scale"] == 1.0
    assert metadata["pitch_inertia_scale"] == 1.0
    if expected_physics_override is None:
        assert "physics_override" not in metadata
    else:
        assert metadata["physics_override"] == expected_physics_override
    assert done.sample_count > 0


def _assert_balances(
    summary: dict,
    metadata: dict,
    done,
    *,
    expected_physics_override: dict | None = None,
) -> None:
    _assert_common_integrity(
        summary,
        metadata,
        done,
        expected_physics_override=expected_physics_override,
    )
    assert done.reason_code == DONE_COMPLETED
    assert not summary["fell"]
    assert summary["max_abs_pitch_deg"] <= 15.0
    assert summary["tail_rail_fraction"] <= 0.05


def _assert_stable(
    summary: dict,
    metadata: dict,
    done,
    *,
    expected_physics_override: dict | None = None,
) -> None:
    _assert_balances(
        summary,
        metadata,
        done,
        expected_physics_override=expected_physics_override,
    )
    assert summary["tail_rms_pitch_deg"] <= 1.0


NOMINAL_SENSOR = {
    "imu_pitch_lag_s": 0.010,
    "imu_noise_seed": 2026,
    "accel_noise_std_mps2": 0.20,
    "gyro_noise_std_rad_s": 0.015,
}


SIMPLE_SCENARIOS = [
    pytest.param(2000, "realistic_neutral_hold_20s", {"duration_s": 20.0}),
    pytest.param(
        2001,
        "realistic_noisy_slow_push_recover_20s",
        {
            "duration_s": 20.0,
            **NOMINAL_SENSOR,
            "disturbances": [
                _ramp(start_s=1.0, duration_s=1.5, force_n=0.0, force_n_end=2.0),
                _ramp(start_s=2.5, duration_s=1.5, force_n=2.0, force_n_end=0.0),
            ],
        },
        marks=pytest.mark.xfail(
            reason="Controller loses stability during the noisy slow-push recovery",
            strict=True,
        ),
    ),
    pytest.param(
        2002,
        "realistic_noisy_com_offset_20s",
        {
            "duration_s": 20.0,
            "com_angle_offset_rad": 0.001,
            **NOMINAL_SENSOR,
        },
    ),
]


@pytest.mark.parametrize(("run_id", "name", "kwargs"), SIMPLE_SCENARIOS)
def test_realistic_simple_scenarios(
    simulator_udp, sim_artifact_settings, run_id: int, name: str, kwargs: dict
):
    output_dir = _artifact_dir(sim_artifact_settings, name)
    summary, metadata, done = run_scenario_live(
        simulator_udp,
        run_id=run_id,
        output_dir=output_dir,
        physics_profile=PHYSICS_REALISTIC,
        **kwargs,
    )
    _assert_stable(summary, metadata, done)

    if name == "realistic_noisy_slow_push_recover_20s":
        rows = _read_timeline(output_dir)
        tail = [row for row in rows if row["t_sec"] >= metadata["duration_s"] - 2.0]
        assert tail
        mean_abs_fused_bias = sum(
            abs(row["fused_pitch_deg"] - row["plant_pitch_deg"]) for row in tail
        ) / len(tail)
        assert summary["tail_mean_abs_velocity_mps"] <= 0.05
        assert summary["max_abs_u_sps"] is not None and summary["max_abs_u_sps"] >= 10.0
        assert mean_abs_fused_bias <= 0.5


@pytest.mark.xfail(
    reason="Pitch-only allocation has insufficient sustained authority in the conservative drive profile",
    strict=True,
)
def test_full_forward_then_stop_moves_and_settles(simulator_udp, sim_artifact_settings):
    output_dir = _artifact_dir(sim_artifact_settings, "full_forward_then_stop")
    physics_override = {"cart_damping": 40.0}
    summary, metadata, done = run_scenario_live(
        simulator_udp,
        run_id=2100,
        output_dir=output_dir,
        physics_profile=PHYSICS_REALISTIC,
        duration_s=12.0,
        telemetry_stride=20,
        physics_override=physics_override,
        joy_segments=[
            {"start_s": 1.0, "duration_s": 5.0, "forward": 1.0},
        ],
    )

    _assert_stable(
        summary,
        metadata,
        done,
        expected_physics_override=physics_override,
    )
    assert done.controller_fault_flags == 0
    assert done.actuator_fault_count == 0
    assert done.max_continuous_saturation_s < 0.5

    rows = _read_timeline(output_dir)
    lean_rows = [row for row in rows if 1.0 <= row["t_sec"] < 2.0]
    settled_rows = [row for row in rows if 4.0 <= row["t_sec"] < 6.0]
    assert lean_rows and settled_rows

    mean_target_sps = sum(row["target_velocity_sps"] for row in settled_rows) / len(
        settled_rows
    )
    mean_motor_sps = sum(row["u_sps"] for row in settled_rows) / len(settled_rows)
    mean_measured_sps = sum(row["measured_vel_sps"] for row in settled_rows) / len(
        settled_rows
    )
    mean_abs_pitch_error_deg = sum(
        abs(row["pitch_error_deg"]) for row in settled_rows
    ) / len(settled_rows)
    assert math.isclose(mean_target_sps, 1200.0, rel_tol=0.0, abs_tol=1.0)
    assert mean_motor_sps >= 0.50 * mean_target_sps
    assert mean_measured_sps >= 0.70 * mean_target_sps
    assert mean_abs_pitch_error_deg <= 1.5

    rate_output_normalized = [
        -row["u_sps"] / 3200.0
        for row in lean_rows
        if (int(row["controller_saturation_flags"]) & 4) == 0
    ]
    assert rate_output_normalized
    assert all(math.isfinite(value) for value in rate_output_normalized)

    assert summary["max_abs_position_m"] >= 0.20
    assert summary["max_abs_position_m"] <= 5.0
    assert abs(summary["final_pitch_deg"]) <= 5.0
    assert summary["tail_mean_abs_velocity_mps"] <= 0.05

# All cases below use the same nominal realistic plant. The clean wood-floor
# capture had 0.388 degree steady pitch RMS, about 0.014 m/s measured velocity
# RMS, and a roughly 0.23 m release catch. The assertions leave broad margin over
# those observations; only the explicitly named sensor-margin case varies sensors.
HARDWARE_STRESS_SCENARIOS = [
    pytest.param(
        3000,
        "hardware_like_release_3deg",
        {
            "duration_s": 20.0,
            "initial_pitch_deg": 3.0,
            "com_angle_offset_rad": 0.002,
            **NOMINAL_SENSOR,
        },
    ),
    pytest.param(
        3001,
        "sensor_margin_elevated_imu_noise",
        {
            "duration_s": 20.0,
            "initial_pitch_deg": 0.5,
            "imu_pitch_lag_s": 0.010,
            "imu_noise_seed": 20260719,
            "accel_noise_std_mps2": 0.50,
            "gyro_noise_std_rad_s": 0.030,
            "imu_timestamp_jitter_us": 1000.0,
            "imu_sample_loss_rate": 0.005,
        },
    ),
    pytest.param(
        3002,
        "hardware_like_2p3hz_rocking",
        {
            "duration_s": 20.0,
            **NOMINAL_SENSOR,
            "disturbances": _alternating_pulse_train(
                start_s=1.0,
                pulse_duration_s=0.217,
                count=10,
                amplitude=3.0,
            ),
        },
    ),
]


@pytest.mark.parametrize(("run_id", "name", "kwargs"), HARDWARE_STRESS_SCENARIOS)
def test_hardware_inspired_stress_scenarios(
    simulator_udp, sim_artifact_settings, run_id: int, name: str, kwargs: dict
):
    output_dir = _artifact_dir(sim_artifact_settings, name)
    summary, metadata, done = run_scenario_live(
        simulator_udp,
        run_id=run_id,
        output_dir=output_dir,
        physics_profile=PHYSICS_REALISTIC,
        **kwargs,
    )
    _assert_stable(summary, metadata, done)
    assert summary["tail_mean_abs_velocity_mps"] <= 0.02
    assert summary["max_abs_position_m"] <= 5.0


REALISTIC_FRONTIER_DIAGNOSTICS = [
    pytest.param(
        3500,
        "frontier_recovery_67deg",
        67.0,
        0.10,
        marks=pytest.mark.xfail(
            reason="Controller and motor ramp cannot arrest a free fall from +67 degrees",
            strict=True,
        ),
    ),
]


@pytest.mark.parametrize(
    ("run_id", "name", "initial_pitch_deg", "forward"),
    REALISTIC_FRONTIER_DIAGNOSTICS,
)
def test_realistic_profile_frontier_diagnostics(
    simulator_udp,
    sim_artifact_settings,
    run_id: int,
    name: str,
    initial_pitch_deg: float,
    forward: float,
):
    output_dir = _artifact_dir(sim_artifact_settings, name)
    summary, metadata, done = run_scenario_live(
        simulator_udp,
        run_id=run_id,
        output_dir=output_dir,
        physics_profile=PHYSICS_REALISTIC,
        initial_pitch_deg=initial_pitch_deg,
        duration_s=20.0,
        telemetry_stride=1,
        joy_segments=[
            {"start_s": 0.0, "duration_s": 3.0, "forward": forward},
        ],
        fail_fast_pitch_deg=70.0,
    )
    _assert_common_integrity(summary, metadata, done)
    assert done.reason_code == DONE_COMPLETED
    assert done.actuator_fault_count == 0
    assert done.controller_fault_flags == 0
    assert done.max_abs_pitch_deg <= 70.0
    assert not summary["fell"]
    rows = _read_timeline(output_dir)
    commanded_rows = [row for row in rows if row["t_sec"] <= 3.0]
    assert commanded_rows
    assert min(abs(row["plant_pitch_deg"]) for row in commanded_rows) <= 25.0
    assert summary["tail_rms_pitch_deg"] <= 1.0
    assert summary["tail_mean_abs_velocity_mps"] <= 0.05
    assert summary["max_abs_position_m"] <= 5.0
