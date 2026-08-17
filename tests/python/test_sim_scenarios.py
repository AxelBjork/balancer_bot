from __future__ import annotations

import csv
import json
import math
import os
import re
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import pytest

from tools.telemetry_analysis import (
    analyze_pitch_authority_sweep,
    band_rms_equivalent,
    read_telemetry_csv,
    validate_pitch_authority_hardware_envelope,
)
from tests.python.support.simulator_service import (
    DONE_COMPLETED,
    DONE_FELL,
    DONE_STOPPED_BY_CLIENT,
    PHYSICS_DIRECT_ACTUATOR,
    PHYSICS_ACTUATOR_STRESS,
    PHYSICS_REALISTIC,
    PHYSICS_STEPPER_PHASE_ELECTRICAL,
    run_scenario_live,
)
from tests.python.support.behavioral_diagnostics import (
    ScenarioDiagnostics,
    artifact_metrics,
)
from tools.telemetry_analysis.stepper_geometry import METERS_PER_STEP


@dataclass(frozen=True)
class SimulatorModel:
    key: str
    label: str
    physics_profile: int
    profile_name: str
    pid_config_path: str


_REPO_ROOT = Path(__file__).parents[2]
DIRECT_ACTUATOR_MODEL = SimulatorModel(
    key="direct_actuator",
    label="DirectActuator",
    physics_profile=PHYSICS_DIRECT_ACTUATOR,
    profile_name="direct_actuator",
    pid_config_path=str(_REPO_ROOT / "tests/data/direct_actuator_pid.conf"),
)
STEPPER_PHASE_ELECTRICAL_MODEL = SimulatorModel(
    key="stepper_phase_electrical",
    label="StepperPhaseElectrical",
    physics_profile=PHYSICS_STEPPER_PHASE_ELECTRICAL,
    profile_name="stepper_phase_electrical",
    # The environment override remains a tuning/evidence hook. The checked-in
    # default is now the current StepperPhaseElectrical profile in pid.conf.
    pid_config_path=os.environ.get(
        "STEPPER_PHASE_ELECTRICAL_PID_CONFIG",
        str(_REPO_ROOT / "pid.conf"),
    ),
)

# The electrical profile's chassis damping is an explicit plant parameter for
# all outer-loop behavioral cases.  Keep it at the profile default while the
# controller is retuned; do not silently substitute the DirectActuator
# reference's high damping.
STEPPER_PHASE_ELECTRICAL_CART_DAMPING = 1.0


def _pid_value(path: str | Path, key: str) -> float:
    pattern = re.compile(rf"^\s*{re.escape(key)}\s*=\s*([-+]?\d+(?:\.\d+)?)\s*$", re.MULTILINE)
    match = pattern.search(Path(path).read_text(encoding="utf-8"))
    assert match is not None, f"{key} is missing from {path}"
    return float(match.group(1))


def test_behavioral_models_use_explicit_pid_configurations():
    """Keep model-specific controller evidence visible at the config boundary."""
    assert DIRECT_ACTUATOR_MODEL.pid_config_path != STEPPER_PHASE_ELECTRICAL_MODEL.pid_config_path
    assert _pid_value(DIRECT_ACTUATOR_MODEL.pid_config_path, "pitch_gain") == 6000.0
    assert _pid_value(DIRECT_ACTUATOR_MODEL.pid_config_path, "pitch_rate_gain") == 350.0
    assert _pid_value(DIRECT_ACTUATOR_MODEL.pid_config_path, "pitch_accel_gain") == 0.0
    assert _pid_value(DIRECT_ACTUATOR_MODEL.pid_config_path, "balance_max_sps") == 16000.0
    if "STEPPER_PHASE_ELECTRICAL_PID_CONFIG" not in os.environ:
        assert _pid_value(STEPPER_PHASE_ELECTRICAL_MODEL.pid_config_path, "pitch_gain") == 203550.0
        assert _pid_value(STEPPER_PHASE_ELECTRICAL_MODEL.pid_config_path, "pitch_rate_gain") == 1932.0
        assert _pid_value(STEPPER_PHASE_ELECTRICAL_MODEL.pid_config_path, "pitch_accel_gain") == 0.0
    assert _pid_value(STEPPER_PHASE_ELECTRICAL_MODEL.pid_config_path, "balance_max_sps") == 16000.0


def _model_params(
    *, direct_xfail: str | None = None, stepper_xfail: str | None = None
):
    params = [
        pytest.param(
            DIRECT_ACTUATOR_MODEL,
            id=DIRECT_ACTUATOR_MODEL.key,
            marks=(
                pytest.mark.xfail(strict=True, reason=direct_xfail)
                if direct_xfail
                else []
            ),
        ),
        pytest.param(
            STEPPER_PHASE_ELECTRICAL_MODEL,
            id=STEPPER_PHASE_ELECTRICAL_MODEL.key,
            marks=(
                pytest.mark.xfail(strict=True, reason=stepper_xfail)
                if stepper_xfail
                else []
            ),
        ),
    ]
    return params


def _model_artifact_dir(sim_artifact_settings: dict, model: SimulatorModel, name: str) -> Path:
    return _artifact_dir(sim_artifact_settings, f"{model.key}/{name}")


def _run_model_scenario(
    simulator_udp, model: SimulatorModel, output_dir: Path, **kwargs
):
    pid_config_path = kwargs.pop("pid_config_path", model.pid_config_path)
    scenario_id = kwargs.pop("scenario_id", output_dir.name)
    subrun_id = kwargs.pop("subrun_id", output_dir.name)
    scenario_category = kwargs.pop("scenario_category", "")
    scenario_intent = kwargs.pop("scenario_intent", "")
    return run_scenario_live(
        simulator_udp,
        output_dir=output_dir,
        physics_profile=model.physics_profile,
        pid_config_path=pid_config_path,
        model_name=model.label,
        scenario_id=scenario_id,
        subrun_id=subrun_id,
        scenario_category=scenario_category,
        scenario_intent=scenario_intent,
        **kwargs,
    )


def _evaluate_subrun(
    diagnostics: ScenarioDiagnostics,
    output_dir: Path,
    subrun_id: str,
    summary: dict,
    done,
    checks: list[tuple[str, Callable[[], None]]],
    *,
    classification: str = "genuine_behavioral_failure",
) -> bool:
    passed = diagnostics.evaluate(
        subrun_id,
        checks,
        metrics=artifact_metrics(summary, done),
        classification=classification,
    )
    diagnostics.write(output_dir)
    return passed


def _run_outer_subrun(
    diagnostics: ScenarioDiagnostics,
    simulator_udp,
    model: SimulatorModel,
    output_dir: Path,
    subrun_id: str,
    run_kwargs: dict,
    checks_factory: Callable[[dict, dict, object, object], list[tuple[str, Callable[[], None]]]],
    *,
    scenario_id: str,
    scenario_category: str,
    scenario_intent: str,
    classification: str = "genuine_behavioral_failure",
):
    """Run one composite subrun and retain all of its gate-level evidence.

    Composite scenarios intentionally remain one pytest item.  A failed gate
    must not prevent the other signed/uncertainty cases from running, because
    the resulting matrix is part of the diagnosis rather than only a pass/fail
    convenience wrapper.
    """
    try:
        result = _run_model_scenario(
            simulator_udp,
            model,
            output_dir=output_dir,
            scenario_id=scenario_id,
            subrun_id=subrun_id,
            scenario_category=scenario_category,
            scenario_intent=scenario_intent,
            **run_kwargs,
        )
        summary, metadata, done, frame = (
            result.summary,
            result.metadata,
            result.done,
            result.frame,
        )
        checks = checks_factory(summary, metadata, done, frame)
    except Exception as exc:
        diagnostics.record_infrastructure_failure(subrun_id, repr(exc))
        diagnostics.write(output_dir)
        return None

    _evaluate_subrun(
        diagnostics,
        output_dir,
        subrun_id,
        summary,
        done,
        checks,
        classification=classification,
    )
    return summary, metadata, done, frame


def _finish_composite(diagnostics: ScenarioDiagnostics) -> None:
    if diagnostics.failures:
        pytest.fail(diagnostics.failure_message())


def assert_done_completed(done) -> None:
    assert done.reason_code == DONE_COMPLETED


def assert_not_fallen(summary: dict) -> None:
    assert not summary["fell"]


def assert_true(condition: bool, message: str) -> None:
    assert condition, message


def assert_authority_rows(rows: list[dict], sign: float) -> None:
    assert len(rows) == 3
    for row in rows:
        assert row["response_polarity"] == (1 if sign > 0 else -1)
        assert row["response_latency_s"] is not None
        assert row["peak_pitch_rate_dps"] is not None
        assert row["applied_force_peak_n"] is not None
        assert row["motor_force_authority_fraction"] is not None
        assert row["traction_authority_fraction"] is not None


def test_udp_transfer_smoke_uses_downsampled_telemetry(
    simulator_udp, sim_artifact_settings: dict
):
    output = Path(sim_artifact_settings["temp_root"]) / "udp_transfer_smoke"
    result = run_scenario_live(
        simulator_udp,
        run_id=1000,
        output_dir=output,
        physics_profile=PHYSICS_DIRECT_ACTUATOR,
        pid_config_path=DIRECT_ACTUATOR_MODEL.pid_config_path,
        duration_s=2.0,
        telemetry_stride=20,
        disturbances=[{"start_s": 0.5, "duration_s": 0.1, "force_n": 0.01}],
    )

    summary, metadata, done = result.summary, result.metadata, result.done
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
    result = run_scenario_live(
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

    summary, _metadata, done = result.summary, result.metadata, result.done
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
    model: SimulatorModel | None = None,
    expected_physics_override: dict | None = None,
    expected_physics_profile: str = "direct_actuator",
    expected_total_mass_scale: float = 1.0,
    expected_pitch_inertia_scale: float = 1.0,
) -> None:
    assert summary["sample_count"] > 0
    assert summary["telemetry_continuous"]
    assert summary["max_abs_pitch_deg"] is not None
    assert summary["tail_rms_pitch_deg"] is not None
    assert summary["tail_rail_fraction"] is not None
    assert metadata["pid_profile"].endswith("pid.conf")
    if model is not None:
        expected_physics_profile = model.profile_name
        assert metadata.get("model") == model.label
    assert metadata["physics_profile"] == expected_physics_profile
    assert metadata["total_mass_scale"] == expected_total_mass_scale
    assert metadata["pitch_inertia_scale"] == expected_pitch_inertia_scale
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
    model: SimulatorModel | None = None,
    expected_physics_override: dict | None = None,
    expected_physics_profile: str = "direct_actuator",
    expected_total_mass_scale: float = 1.0,
    expected_pitch_inertia_scale: float = 1.0,
    max_abs_pitch_deg: float = 15.0,
) -> None:
    _assert_common_integrity(
        summary,
        metadata,
        done,
        model=model,
        expected_physics_override=expected_physics_override,
        expected_physics_profile=expected_physics_profile,
        expected_total_mass_scale=expected_total_mass_scale,
        expected_pitch_inertia_scale=expected_pitch_inertia_scale,
    )
    assert done.reason_code == DONE_COMPLETED
    assert not summary["fell"]
    assert summary["max_abs_pitch_deg"] <= max_abs_pitch_deg
    assert summary["tail_rail_fraction"] <= 0.05


def _assert_stable(
    summary: dict,
    metadata: dict,
    done,
    *,
    model: SimulatorModel | None = None,
    expected_physics_override: dict | None = None,
    expected_physics_profile: str = "direct_actuator",
    expected_total_mass_scale: float = 1.0,
    expected_pitch_inertia_scale: float = 1.0,
    max_abs_pitch_deg: float = 15.0,
) -> None:
    _assert_balances(
        summary,
        metadata,
        done,
        model=model,
        expected_physics_override=expected_physics_override,
        expected_physics_profile=expected_physics_profile,
        expected_total_mass_scale=expected_total_mass_scale,
        expected_pitch_inertia_scale=expected_pitch_inertia_scale,
        max_abs_pitch_deg=max_abs_pitch_deg,
    )
    # DirectActuator is the historical controller-design reference.  The
    # electrical plant has the same bounded balance objective but a visibly
    # different, slower residual envelope with the deliberately simulator-only
    # inner-loop candidate.  Keep the reference threshold unchanged and use a
    # broad behavioral bound for the electrical model; the separate growth
    # checks below still reject divergence.
    tail_rms_limit = 1.0 if model in (None, DIRECT_ACTUATOR_MODEL) else 4.0
    assert summary["tail_rms_pitch_deg"] <= tail_rms_limit


def _assert_no_growing_oscillation(frame, *, ratio_limit: float = 1.5) -> None:
    """Reject a late RMS/envelope growth while tolerating model-specific noise.

    The early/middle/late comparison is intentionally an envelope witness,
    not a demand for a particular DirectActuator trajectory.  The existing
    split late-window check remains as a shorter-horizon guard.
    """
    assert not frame.empty
    duration_s = float(frame["t_sec"].iloc[-1])
    if duration_s < 4.0:
        return
    if duration_s >= 9.0:
        for column, absolute_floor in (
            ("plant_pitch_deg", 1.5),
            ("plant_pitch_rate_dps", 100.0),
            ("plant_velocity_mps", 0.02),
        ):
            early = frame[frame["t_sec"] < duration_s / 3.0]
            middle = frame[
                (frame["t_sec"] >= duration_s / 3.0)
                & (frame["t_sec"] < 2.0 * duration_s / 3.0)
            ]
            late = frame[frame["t_sec"] >= 2.0 * duration_s / 3.0]
            assert not early.empty and not middle.empty and not late.empty
            early_rms = math.sqrt(float((early[column] ** 2).mean()))
            middle_rms = math.sqrt(float((middle[column] ** 2).mean()))
            late_rms = math.sqrt(float((late[column] ** 2).mean()))
            # A single early transient may be larger than the late envelope;
            # classify growth only when both middle and late windows rise.
            sustained_growth = (
                middle_rms > max(absolute_floor, ratio_limit * early_rms)
                and late_rms > max(absolute_floor, ratio_limit * middle_rms)
            )
            assert not sustained_growth
    start_s = max(0.0, duration_s - 0.30 * duration_s)
    split_s = start_s + 0.5 * (duration_s - start_s)
    first = frame[(frame["t_sec"] >= start_s) & (frame["t_sec"] < split_s)]
    second = frame[frame["t_sec"] >= split_s]
    assert not first.empty and not second.empty
    for column, absolute_floor in (
        ("plant_pitch_deg", 1.5),
        ("plant_pitch_rate_dps", 100.0),
        ("plant_velocity_mps", 0.02),
    ):
        first_rms = math.sqrt(float((first[column] ** 2).mean()))
        second_rms = math.sqrt(float((second[column] ** 2).mean()))
        assert second_rms <= max(absolute_floor, ratio_limit * first_rms)


NOMINAL_SENSOR = {
    "imu_noise_seed": 2026,
    "accel_noise_std_mps2": 0.20,
    "gyro_noise_std_rad_s": 0.015,
}


SIMPLE_SCENARIOS = [
    pytest.param(2000, "neutral_hold_20s", {"duration_s": 20.0}),
    pytest.param(
        2001,
        "noisy_slow_push_recover_20s",
        {
            "duration_s": 20.0,
            **NOMINAL_SENSOR,
            "disturbances": [
                _ramp(start_s=1.0, duration_s=1.5, force_n=0.0, force_n_end=2.0),
                _ramp(start_s=2.5, duration_s=1.5, force_n=2.0, force_n_end=0.0),
            ],
        },
    ),
    pytest.param(
        2002,
        "noisy_com_offset_20s",
        {
            "duration_s": 20.0,
            "com_angle_offset_rad": 0.001,
            **NOMINAL_SENSOR,
        },
    ),
]


@pytest.mark.parametrize(("run_id", "name", "kwargs"), SIMPLE_SCENARIOS)
@pytest.mark.parametrize("model", _model_params())
def test_simple_behavioral_scenarios(
    simulator_udp,
    sim_artifact_settings,
    run_id: int,
    name: str,
    kwargs: dict,
    model: SimulatorModel,
):
    output_dir = _model_artifact_dir(sim_artifact_settings, model, name)
    result = _run_model_scenario(
        simulator_udp,
        model,
        run_id=run_id,
        output_dir=output_dir,
        **kwargs,
    )
    summary, metadata, done = result.summary, result.metadata, result.done
    # The locked attitude loop plus the new bounded, lower-gain velocity path
    # accepts a larger plant-only transient for this deliberately strong push.
    # Keep the no-fall/rail and late settling checks below; do not encode the
    # previous faster outer-loop trajectory as the default's safety boundary.
    _assert_stable(
        summary,
        metadata,
        done,
        model=model,
        max_abs_pitch_deg=30.0 if name == "noisy_slow_push_recover_20s" else 15.0,
    )
    _assert_no_growing_oscillation(result.frame)

    if name == "noisy_slow_push_recover_20s":
        rows = result.frame.to_dict(orient="records")
        tail = [row for row in rows if row["t_sec"] >= metadata["duration_s"] - 2.0]
        assert tail
        mean_abs_fused_bias = sum(
            abs(row["fused_pitch_deg"] - row["plant_pitch_deg"]) for row in tail
        ) / len(tail)
        # The selected 3 Hz velocity-control pole deliberately trades stopping
        # speed for separation from the fast attitude path.  This disturbance
        # is bounded and decaying, but it does not reproduce the old faster
        # velocity trajectory within a 20 s window.
        # The hardcoded rate-feedback notch and 3 Hz velocity-control pole
        # trade some late transient velocity for separation from the fast
        # attitude path.  Keep the check bounded and behavioral rather than
        # requiring the previous faster trajectory.
        assert summary["tail_mean_abs_velocity_mps"] <= 0.20
        assert summary["max_abs_position_m"] <= 15.0
        assert summary["max_abs_u_sps"] is not None and summary["max_abs_u_sps"] >= 10.0
        assert mean_abs_fused_bias <= 0.5


ATTITUDE_RECOVERY_SCENARIOS = [
    pytest.param(-1.0, id="minus_1deg"),
    pytest.param(1.0, id="plus_1deg"),
    pytest.param(-2.0, id="minus_2deg"),
    pytest.param(2.0, id="plus_2deg"),
    pytest.param(-4.0, id="minus_4deg"),
    pytest.param(4.0, id="plus_4deg"),
    pytest.param(-6.0, id="minus_6deg"),
    pytest.param(6.0, id="plus_6deg"),
]


@pytest.mark.parametrize("initial_pitch_deg", ATTITUDE_RECOVERY_SCENARIOS)
@pytest.mark.parametrize("model", _model_params())
def test_attitude_recovery_common_behavior(
    simulator_udp,
    sim_artifact_settings,
    initial_pitch_deg: float,
    model: SimulatorModel,
):
    """Exercise the shared known-attitude recovery frontier once per model."""
    sign = "plus" if initial_pitch_deg > 0.0 else "minus"
    output_dir = _model_artifact_dir(
        sim_artifact_settings,
        model,
        f"attitude_recovery_{sign}_{abs(initial_pitch_deg):g}deg",
    )
    result = _run_model_scenario(
        simulator_udp,
        model,
        run_id=2200 + int((initial_pitch_deg + 6.0) * 10),
        output_dir=output_dir,
        duration_s=10.0,
        telemetry_stride=1,
        initial_pitch_deg=initial_pitch_deg,
        fail_fast_pitch_deg=35.0,
    )
    summary, metadata, done, frame = (
        result.summary,
        result.metadata,
        result.done,
        result.frame,
    )
    _assert_common_integrity(summary, metadata, done, model=model)
    assert done.reason_code == DONE_COMPLETED
    assert done.controller_fault_flags == 0
    assert done.actuator_fault_count == 0
    assert not summary["fell"]
    assert summary["max_abs_pitch_deg"] <= (12.5 if model == DIRECT_ACTUATOR_MODEL else 20.0)
    assert frame["plant_pitch_rate_dps"].abs().max() <= (250.0 if model == DIRECT_ACTUATOR_MODEL else 350.0)
    assert summary["tail_rms_pitch_deg"] <= (1.5 if model == DIRECT_ACTUATOR_MODEL else 6.0)
    _assert_no_growing_oscillation(frame)


@pytest.mark.parametrize("model", _model_params())
def test_full_forward_then_stop_moves_and_settles(
    simulator_udp, sim_artifact_settings, model: SimulatorModel
):
    output_dir = _model_artifact_dir(sim_artifact_settings, model, "full_forward_then_stop")
    physics_override = _outer_physics_override(model)
    result = _run_model_scenario(
        simulator_udp,
        model,
        run_id=2100,
        output_dir=output_dir,
        duration_s=12.0,
        telemetry_stride=20,
        physics_override=physics_override,
        joy_segments=[
            {"start_s": 1.0, "duration_s": 5.0, "forward": 1.0},
        ],
    )
    summary, metadata, done = result.summary, result.metadata, result.done

    # This is an electrical-profile motion/stop scenario, not a neutral-hold
    # precision test.  The direct-force reference settles by roughly 6 s;
    # retain a wider transient margin than the neutral 1 deg threshold used by
    # _assert_stable().
    _assert_balances(
        summary,
        metadata,
        done,
        model=model,
        expected_physics_override=physics_override,
    )
    assert summary["settled_at_s"] is not None
    assert summary["settled_at_s"] <= 8.0
    assert summary["tail_rms_pitch_deg"] <= 2.0
    assert done.controller_fault_flags == 0
    assert done.actuator_fault_count == 0
    assert done.max_continuous_saturation_s < 0.5

    rows = result.frame.to_dict(orient="records")
    lean_rows = [row for row in rows if 1.0 <= row["t_sec"] < 2.0]
    drive_rows = [row for row in rows if 4.0 <= row["t_sec"] < 6.0]
    recovery_rows = [row for row in rows if 6.0 <= row["t_sec"] < 8.0]
    assert lean_rows and drive_rows and recovery_rows

    mean_reference_velocity = sum(
        row["reference_velocity_mps"] for row in drive_rows
    ) / len(drive_rows)
    peak_reference_acceleration = max(
        row["reference_acceleration_mps2"] for row in (lean_rows + drive_rows)
    )
    mean_motor_sps = sum(row["u_sps"] for row in drive_rows) / len(drive_rows)
    mean_corrected_sps = sum(row["corrected_axle_velocity_sps"] for row in drive_rows) / len(
        drive_rows
    )
    mean_abs_pitch_error_deg = sum(
        abs(row["pitch_error_deg"]) for row in recovery_rows
    ) / len(recovery_rows)
    assert mean_reference_velocity > 0.0
    assert peak_reference_acceleration > 0.0
    # The v1 SI profile intentionally caps the user target at 0.12 m/s and
    # shares a small motion-pitch budget with feedback.  Verify meaningful
    # forward motion without retaining the former high-authority trajectory
    # threshold.
    assert mean_motor_sps > 50.0
    assert mean_corrected_sps > 100.0
    assert mean_abs_pitch_error_deg <= 4

    rate_output_normalized = [
        -row["u_sps"] / 6400.0
        for row in lean_rows
        if (int(row["controller_saturation_flags"]) & 4) == 0
    ]
    assert rate_output_normalized
    assert all(math.isfinite(value) for value in rate_output_normalized)

    if model == STEPPER_PHASE_ELECTRICAL_MODEL:
        # The electrical golden surface is evaluated against the requested
        # trajectory, not the historical 40 mm witness.  Integrate over the
        # active command interval before the release phase so the stop cannot
        # improve the apparent drive score.
        active = result.frame[
            (result.frame["t_sec"] >= 1.0) & (result.frame["t_sec"] < 6.0)
        ]
        assert len(active) >= 2

        def integrate(column: str) -> float:
            values = active[column].to_numpy(dtype=float)
            times = active["t_sec"].to_numpy(dtype=float)
            return float(
                sum(
                    0.5 * (values[index] + values[index - 1])
                    * (times[index] - times[index - 1])
                    for index in range(1, len(values))
                )
            )

        reference_distance = integrate("reference_velocity_mps")
        actual_distance = integrate("plant_velocity_mps")
        hold = result.frame[
            (result.frame["t_sec"] >= 4.0) & (result.frame["t_sec"] < 6.0)
        ]
        mean_reference_velocity = float(hold["reference_velocity_mps"].mean())
        mean_actual_velocity = float(hold["plant_velocity_mps"].mean())
        assert reference_distance > 0.4
        assert actual_distance > 0.0
        assert 0.75 <= actual_distance / reference_distance <= 1.25
        assert mean_actual_velocity / mean_reference_velocity >= 0.75
        # This is a physical overspeed guard, not a displacement envelope.
        assert result.frame["plant_velocity_mps"].abs().max() <= 0.30
    else:
        assert summary["max_abs_position_m"] >= 0.04
        assert summary["max_abs_position_m"] <= 5.0
    assert abs(summary["final_pitch_deg"]) <= 5.0
    assert summary["tail_mean_abs_velocity_mps"] <= 0.05

# These cases originated as the DirectActuator controller-design reference. The clean wood-floor
# capture had 0.388 degree steady pitch RMS, about 0.014 m/s measured velocity
# RMS, and a roughly 0.23 m release catch. The DirectActuator assertions preserve
# that scope; the same behavioral cases now run against the electrical plant with
# its explicit simulator-only PID configuration.
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
        "sensor_noise_margin_elevated_imu_noise",
        {
            "duration_s": 20.0,
            "initial_pitch_deg": 0.5,
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
                amplitude=0.5,
            ),
        },
    ),
]


@pytest.mark.parametrize(("run_id", "name", "kwargs"), HARDWARE_STRESS_SCENARIOS)
@pytest.mark.parametrize("model", _model_params())
def test_hardware_inspired_stress_scenarios(
    simulator_udp,
    sim_artifact_settings,
    run_id: int,
    name: str,
    kwargs: dict,
    model: SimulatorModel,
):
    output_dir = _model_artifact_dir(sim_artifact_settings, model, name)
    result = _run_model_scenario(
        simulator_udp,
        model,
        run_id=run_id,
        output_dir=output_dir,
        telemetry_stride=1 if name == "sensor_noise_margin_elevated_imu_noise" else 80,
        **kwargs,
    )
    summary, metadata, done = result.summary, result.metadata, result.done
    _assert_stable(summary, metadata, done, model=model)
    _assert_no_growing_oscillation(result.frame)
    # The lower outer-loop gain intentionally gives up a small amount of
    # long-tail stopping speed in exchange for keeping velocity pitch demand
    # bounded.  The invariant is bounded residual motion, not the old exact
    # trajectory.
    tail_velocity_limit = 0.025 if model == DIRECT_ACTUATOR_MODEL else 0.12
    assert summary["tail_mean_abs_velocity_mps"] <= tail_velocity_limit
    assert summary["max_abs_position_m"] <= 5.0
    if name == "sensor_noise_margin_elevated_imu_noise":
        spectrum = band_rms_equivalent(
            result.frame, "u_sps", 30.0, 100.0
        )
        assert spectrum["sample_rate_hz"] is not None
        assert spectrum["rms"] is not None


# These are deliberately end-to-end cases: each one starts a fresh
# ``balancer_simulator`` process through the same UDP service used by the
# simulator protocol tests above.  Keep this catalog small and behavioral.
# The controller's detailed telemetry is asserted here instead of maintaining
# a second offline runner/report path.
OUTER_METERS_PER_STEP = METERS_PER_STEP
DIRECT_ACTUATOR_CART_DAMPING = 40.0


def _outer_physics_override(model: SimulatorModel) -> dict[str, float]:
    """Keep the DirectActuator reference profile distinct from electrical plant tuning."""
    damping = (
        STEPPER_PHASE_ELECTRICAL_CART_DAMPING
        if model == STEPPER_PHASE_ELECTRICAL_MODEL
        else DIRECT_ACTUATOR_CART_DAMPING
    )
    return {"cart_damping": damping}


def _outer_frame(output_dir: Path):
    frame = read_telemetry_csv(output_dir / "timeline.csv")
    assert not frame.empty
    assert frame["t_sec"].is_monotonic_increasing
    for column in (
        "plant_pitch_deg",
        "plant_pitch_rate_dps",
        "plant_velocity_mps",
        "plant_position_m",
        "u_sps",
        "pitch_sp_deg",
        "reference_velocity_mps",
        "reference_acceleration_mps2",
        "reference_jerk_mps3",
        "velocity_feedback_estimate_mps",
        "velocity_error_mps",
        "velocity_feedback_acceleration_mps2",
        "velocity_p_acceleration_mps2",
        "velocity_i_acceleration_mps2",
        "velocity_integral_state_mps_s",
        "acceleration_raw_mps2",
        "acceleration_cmd_mps2",
        "drive_pitch_target_deg",
        "final_pitch_target_deg",
        "fixed_com_trim_deg",
        "outer_acceleration_limited",
        "active_outer_pitch_limit_deg",
        "active_planner_max_acceleration_mps2",
        "active_planner_max_deceleration_mps2",
        "active_planner_max_jerk_mps3",
        "active_velocity_i_gain_per_s2",
        "active_velocity_i_leak_time_s",
        "active_velocity_i_acceleration_limit_mps2",
        "planner_acceleration_limited",
        "planner_jerk_limited",
        "velocity_integral_limited",
        "velocity_anti_windup_active",
        "adaptive_com_trim_enabled",
        "legacy_outer_fields_valid",
        "f_cmd",
        "traction_limit_n",
        "motor_force_limit_n",
        # Deprecated outer aliases remain present for wire compatibility, but
        # are intentionally not used as controller evidence.
        "velocity_pitch_request_unclamped_deg",
        "velocity_pitch_request_limited_deg",
        "velocity_authority_limited",
        "force_saturated",
        "com_trim_deg",
        "trim_trusted",
        "trim_learning_enabled",
        "trim_learning_allowed",
        "pitch_authority_diagnostic_active",
        "pitch_authority_diagnostic_target_deg",
        "pitch_authority_diagnostic_com_trim_deg",
        "pitch_authority_diagnostic_remaining_s",
        "pitch_authority_diagnostic_request_id",
        "pitch_authority_diagnostic_command_age_ms",
        "completed_step_acceleration_sps2",
    ):
        assert column in frame.columns
        assert frame[column].notna().all()
    return frame


def _outer_pid_variant(
    sim_artifact_settings: dict, model: SimulatorModel, damping: float, limit_deg: float
) -> str:
    source = Path(model.pid_config_path)
    output = (
        Path(sim_artifact_settings["temp_root"])
        / model.key
        / f"outer_pid_{damping:g}_{limit_deg:g}.pid.conf"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    text = source.read_text(encoding="utf-8")
    text = re.sub(
        r"(?m)^\s*velocity_gain_per_s\s*=.*$",
        f"velocity_gain_per_s = {damping:g}",
        text,
    )
    text = re.sub(
        r"(?m)^\s*outer_pitch_limit_deg\s*=.*$",
        f"outer_pitch_limit_deg = {limit_deg:g}",
        text,
    )
    output.write_text(text, encoding="utf-8")
    return str(output)


def _attitude_only_pid_variant(
    sim_artifact_settings: dict, model: SimulatorModel
) -> str:
    """Make a simulator-only fixed-target recovery profile."""
    source = Path(model.pid_config_path)
    output = (
        Path(sim_artifact_settings["temp_root"])
        / model.key
        / "outer_pid_cold_start.pid.conf"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    text = source.read_text(encoding="utf-8")
    text = re.sub(r"(?m)^\s*velocity_gain_per_s\s*=.*$", "velocity_gain_per_s = 0", text)
    text = re.sub(
        r"(?m)^\s*adaptive_com_trim_enabled\s*=.*$",
        "adaptive_com_trim_enabled = 0",
        text,
    )
    output.write_text(text, encoding="utf-8")
    return str(output)


def _adaptive_com_pid_variant(
    sim_artifact_settings: dict, model: SimulatorModel
) -> str:
    """Make an explicit opt-in profile for the legacy adaptive COM learner.

    The production/default v12 profiles keep this learner disabled.  The
    older COM acquisition and maintenance tests still provide useful coverage
    of that optional state machine, so they must opt in rather than silently
    changing the default controller semantics.
    """
    source = Path(model.pid_config_path)
    output = (
        Path(sim_artifact_settings["temp_root"])
        / model.key
        / "outer_pid_adaptive_com.pid.conf"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    text = source.read_text(encoding="utf-8")
    text = re.sub(
        r"(?m)^\s*adaptive_com_trim_enabled\s*=.*$",
        "adaptive_com_trim_enabled = 1",
        text,
    )
    text = re.sub(
        r"(?m)^\s*adaptive_com_trim_gain_deg_per_mps_s\s*=.*$",
        "adaptive_com_trim_gain_deg_per_mps_s = 4",
        text,
    )
    text = re.sub(
        r"(?m)^\s*adaptive_com_trim_limit_deg\s*=.*$",
        "adaptive_com_trim_limit_deg = 4",
        text,
    )
    output.write_text(text, encoding="utf-8")
    return str(output)


def _outer_assert_bounded(
    summary: dict,
    metadata: dict,
    done,
    frame,
    *,
    model: SimulatorModel | None = None,
    expected_physics_override: dict | None = None,
    expected_total_mass_scale: float = 1.0,
    expected_pitch_inertia_scale: float = 1.0,
    expected_velocity_pitch_limit_deg: float | None = None,
    max_pitch_deg: float = 15.0,
    check_growing_oscillation: bool = True,
) -> None:
    if expected_physics_override is None and model is not None:
        expected_physics_override = _outer_physics_override(model)
    _assert_common_integrity(
        summary,
        metadata,
        done,
        model=model,
        expected_physics_override=expected_physics_override,
        expected_total_mass_scale=expected_total_mass_scale,
        expected_pitch_inertia_scale=expected_pitch_inertia_scale,
    )
    assert done.reason_code == DONE_COMPLETED
    assert not summary["fell"]
    assert summary["max_abs_pitch_deg"] <= max_pitch_deg
    assert done.controller_fault_flags == 0
    assert done.actuator_fault_count == 0
    assert frame["plant_pitch_rate_dps"].abs().max() < 300.0
    active_outer_pitch_limit_deg = float(frame["active_outer_pitch_limit_deg"].abs().max())
    if expected_velocity_pitch_limit_deg is None:
        expected_velocity_pitch_limit_deg = active_outer_pitch_limit_deg
    assert frame["drive_pitch_target_deg"].abs().max() <= expected_velocity_pitch_limit_deg + 0.05
    assert (
        active_outer_pitch_limit_deg
        <= expected_velocity_pitch_limit_deg + 0.05
    )
    if check_growing_oscillation:
        _assert_no_growing_oscillation(frame)


def _outer_stopping_metrics(frame, event_end_s: float) -> tuple[float | None, float | None]:
    speed_sps = frame["plant_velocity_mps"].abs() / OUTER_METERS_PER_STEP
    times = frame["t_sec"]
    for index, time_s in enumerate(times):
        if time_s < event_end_s or speed_sps.iloc[index] > 50.0:
            continue
        quiet = frame[(times >= time_s) & (times < time_s + 0.5)]
        if not quiet.empty and (
            quiet["plant_velocity_mps"].abs() / OUTER_METERS_PER_STEP <= 50.0
        ).all():
            return float(time_s - event_end_s), float(abs(frame["plant_position_m"].iloc[index]))
    return None, None


def _outer_window_metrics(frame, start_s: float, end_s: float) -> dict[str, float]:
    window = frame[(frame["t_sec"] >= start_s) & (frame["t_sec"] <= end_s)]
    # A run that falls or faults before the requested late window is a genuine
    # behavioral failure. Do not turn it into an infrastructure failure by
    # asserting that a post-failure window exists; the normal numeric checks
    # below will reject the NaN metrics while the run summary retains the
    # actual termination reason.
    if window.empty:
        nan = float("nan")
        return {
            "velocity_rms_sps": nan,
            "velocity_peak_sps": nan,
            "pitch_rms_deg": nan,
            "pitch_peak_deg": nan,
            "pitch_target_peak_deg": nan,
            "acceleration_raw_peak_mps2": nan,
            "acceleration_cmd_peak_mps2": nan,
            "command_rms_sps": nan,
            "command_peak_sps": nan,
            "trim_min_deg": nan,
            "trim_max_deg": nan,
            "trim_learning_duty": nan,
            "authority_duty": nan,
            "force_saturation_duty": nan,
        }
    return {
        "velocity_rms_sps": float(
            math.sqrt(float((window["plant_velocity_mps"] / OUTER_METERS_PER_STEP).pow(2).mean()))
        ),
        "velocity_peak_sps": float(
            (window["plant_velocity_mps"].abs() / OUTER_METERS_PER_STEP).max()
        ),
        "pitch_rms_deg": float(math.sqrt(float((window["plant_pitch_deg"] ** 2).mean()))),
        "pitch_peak_deg": float(window["plant_pitch_deg"].abs().max()),
        "pitch_target_peak_deg": float(window["pitch_sp_deg"].abs().max()),
        "acceleration_raw_peak_mps2": float(
            window["acceleration_raw_mps2"].abs().max()
        ),
        "acceleration_cmd_peak_mps2": float(
            window["acceleration_cmd_mps2"].abs().max()
        ),
        "command_rms_sps": float(math.sqrt(float((window["u_sps"] ** 2).mean()))),
        "command_peak_sps": float(window["u_sps"].abs().max()),
        "trim_min_deg": float(window["com_trim_deg"].min()),
        "trim_max_deg": float(window["com_trim_deg"].max()),
        "trim_learning_duty": float(window["trim_learning_enabled"].mean()),
        "authority_duty": float(window["outer_acceleration_limited"].mean()),
        "force_saturation_duty": float(window["force_saturated"].mean()),
    }


def _outer_trust_time(frame) -> float | None:
    trusted = frame[frame["trim_trusted"] > 0.5]
    return None if trusted.empty else float(trusted["t_sec"].iloc[0])


def _outer_equilibrium_convergence(frame, end_s: float, window_s: float = 1.0) -> dict[str, float]:
    """Reconstruct the slow trim witness from controller-facing telemetry."""
    time = frame["t_sec"].to_numpy(dtype=float)
    candidate = (
        frame["pitch_deg"].to_numpy(dtype=float)
        - frame["drive_pitch_target_deg"].to_numpy(dtype=float)
        - frame["fixed_com_trim_deg"].to_numpy(dtype=float)
    )
    filtered_candidate = []
    candidate_value = float(candidate[0])
    previous_time = float(time[0])
    for current_time, current_candidate in zip(time, candidate):
        current_time = float(current_time)
        alpha = 1.0 - math.exp(-2.0 * math.pi * 0.5 * (current_time - previous_time))
        candidate_value += alpha * (float(current_candidate) - candidate_value)
        filtered_candidate.append(candidate_value)
        previous_time = current_time

    estimate = []
    estimate_value = float(filtered_candidate[0])
    previous_time = float(time[0])
    for current_time, current_candidate in zip(time, filtered_candidate):
        current_time = float(current_time)
        if estimate:
            alpha = 1.0 - math.exp(-2.0 * math.pi * 0.25 * (current_time - previous_time))
            estimate_value += alpha * (float(current_candidate) - estimate_value)
        estimate.append(estimate_value)
        previous_time = current_time

    window = frame[(frame["t_sec"] >= end_s - window_s) & (frame["t_sec"] <= end_s)]
    assert len(window) >= 2
    indices = window.index.to_numpy()
    estimate_window = [estimate[int(index)] for index in indices]
    candidate_window = [filtered_candidate[int(index)] for index in indices]
    span_s = float(window["t_sec"].iloc[-1] - window["t_sec"].iloc[0])
    return {
        "estimate_rate_deg_per_s": abs(estimate_window[-1] - estimate_window[0]) / span_s,
        "estimate_span_deg": max(estimate_window) - min(estimate_window),
        "candidate_estimate_gap_deg": max(
            abs(value - estimate_value)
            for value, estimate_value in zip(candidate_window, estimate_window)
        ),
    }


def _outer_late_slope(frame, column: str, start_s: float) -> float:
    late = frame[frame["t_sec"] >= start_s]
    assert len(late) >= 2
    return float(
        (late[column].iloc[-1] - late[column].iloc[0])
        / (late["t_sec"].iloc[-1] - late["t_sec"].iloc[0])
    )


def _outer_report_category(name: str) -> str:
    for token, category in (
        ("initial", "initial_velocity_recovery"),
        ("recovery", "external_push_recovery"),
        ("authority", "authority_and_reduced_translation"),
        ("estimator", "velocity_estimator_error"),
        ("region", "gain_authority_region"),
        ("com", "com_acquisition_and_maintenance"),
        ("reversal", "drive_and_reversal"),
        ("long", "long_horizon"),
        ("mass", "mass_inertia_uncertainty"),
    ):
        if token in name:
            return category
    return "outer_integration"


def _outer_report_row(output_dir: Path) -> dict:
    metadata = json.loads((output_dir / "metadata.json").read_text(encoding="utf-8"))
    done = json.loads((output_dir / "done.json").read_text(encoding="utf-8"))
    frame = _outer_frame(output_dir)
    duration_s = float(metadata["duration_s"])
    early_end = min(duration_s, max(20.0, duration_s * 0.2))
    middle_start = duration_s * 0.4
    middle_end = duration_s * 0.6
    late_start = max(0.0, duration_s - 25.0)
    early = _outer_window_metrics(frame, 0.0, early_end)
    middle = _outer_window_metrics(frame, middle_start, middle_end)
    late = _outer_window_metrics(frame, late_start, duration_s)
    sample_deltas = frame["t_sec"].diff().dropna()
    sample_dt = float(sample_deltas.median()) if not sample_deltas.empty else 0.0
    authority_seconds = float(
        (frame["outer_acceleration_limited"] > 0.5).sum() * max(0.0, sample_dt)
    )
    slow_band = band_rms_equivalent(frame, "plant_velocity_mps", 0.1, 0.3)
    trust_time_s = _outer_trust_time(frame)
    late_slope = _outer_late_slope(frame, "com_trim_deg", late_start)
    event_end_s = None
    if abs(float(metadata.get("initial_velocity_mps", 0.0))) > 1e-9:
        event_end_s = 0.0
    else:
        for disturbance in metadata.get("disturbances", []):
            if float(disturbance.get("force_n", 0.0)) != 0.0:
                duration = float(disturbance.get("duration_s", 0.0))
                if duration > 0.0:
                    event_end_s = float(disturbance.get("start_s", 0.0)) + duration
                    break
    stopping_time_s = None
    stopping_distance_m = None
    if event_end_s is not None:
        stopping_time_s, stopping_distance_m = _outer_stopping_metrics(frame, event_end_s)
    available_force = frame[["traction_limit_n", "motor_force_limit_n"]].min(axis=1)
    force_ratio = frame["f_cmd"].abs() / available_force.clip(lower=1e-9)
    late_half = max(1.0, (duration_s - late_start) / 2.0)
    late_first = frame[
        (frame["t_sec"] >= late_start) & (frame["t_sec"] < late_start + late_half)
    ]
    late_second = frame[frame["t_sec"] >= late_start + max(1.0, (duration_s - late_start) / 2.0)]
    late_first_peak_sps = (
        float((late_first["plant_velocity_mps"].abs() / OUTER_METERS_PER_STEP).max())
        if not late_first.empty
        else 0.0
    )
    late_second_peak_sps = (
        float((late_second["plant_velocity_mps"].abs() / OUTER_METERS_PER_STEP).max())
        if not late_second.empty
        else 0.0
    )
    late_first_velocity_rms_sps = (
        float(
            math.sqrt(
                float(
                    (
                        late_first["plant_velocity_mps"] / OUTER_METERS_PER_STEP
                    ).pow(2).mean()
                )
            )
        )
        if not late_first.empty
        else 0.0
    )
    late_second_velocity_rms_sps = (
        float(
            math.sqrt(
                float(
                    (
                        late_second["plant_velocity_mps"] / OUTER_METERS_PER_STEP
                    ).pow(2).mean()
                )
            )
        )
        if not late_second.empty
        else 0.0
    )
    return {
        "scenario": output_dir.name,
        "model": metadata.get("model", metadata.get("physics_profile", "unknown")),
        "category": _outer_report_category(output_dir.name),
        "run_id": metadata["run_id"],
        "duration_s": duration_s,
        "initial_velocity_sps": float(metadata.get("initial_velocity_mps", 0.0))
        / OUTER_METERS_PER_STEP,
        "estimator_bias_mps": metadata.get("velocity_estimator_bias_mps", 0.0),
        "estimator_bias_drift_mps_per_s": metadata.get(
            "velocity_estimator_bias_drift_mps_per_s", 0.0
        ),
        "estimator_scale": metadata.get("velocity_estimator_scale", 1.0),
        "estimator_latency_s": metadata.get("velocity_estimator_latency_s", 0.0),
        "fall": bool(done["reason_code"] != DONE_COMPLETED or done.get("fell", False)),
        "final_velocity_sps": float(frame["plant_velocity_mps"].iloc[-1])
        / OUTER_METERS_PER_STEP,
        "peak_velocity_sps": float(frame["plant_velocity_mps"].abs().max())
        / OUTER_METERS_PER_STEP,
        "peak_pitch_deg": float(frame["plant_pitch_deg"].abs().max()),
        "peak_pitch_target_deg": float(frame["pitch_sp_deg"].abs().max()),
        "peak_acceleration_raw_mps2": float(frame["acceleration_raw_mps2"].abs().max()),
        "peak_acceleration_cmd_mps2": float(frame["acceleration_cmd_mps2"].abs().max()),
        "authority_seconds": authority_seconds,
        "command_rms_sps": late["command_rms_sps"],
        "command_peak_sps": float(frame["u_sps"].abs().max()),
        "peak_pitch_rate_dps": float(frame["plant_pitch_rate_dps"].abs().max()),
        "actuator_saturation_duty": float(frame["force_saturated"].mean()),
        "max_continuous_saturation_s": done["max_continuous_saturation_s"],
        "max_force_to_available_ratio": float(force_ratio.max()),
        "stopping_time_s": stopping_time_s,
        "stopping_distance_m": stopping_distance_m,
        "trim_min_deg": float(frame["com_trim_deg"].min()),
        "trim_max_deg": float(frame["com_trim_deg"].max()),
        "final_trim_deg": float(frame["com_trim_deg"].iloc[-1]),
        "trim_trusted": bool(frame["trim_trusted"].iloc[-1] > 0.5),
        "trim_trust_time_s": trust_time_s,
        "late_trim_slope_deg_per_s": late_slope,
        "early_velocity_rms_sps": early["velocity_rms_sps"],
        "middle_velocity_rms_sps": middle["velocity_rms_sps"],
        "late_velocity_rms_sps": late["velocity_rms_sps"],
        "early_pitch_rms_deg": early["pitch_rms_deg"],
        "middle_pitch_rms_deg": middle["pitch_rms_deg"],
        "late_pitch_rms_deg": late["pitch_rms_deg"],
        "slow_velocity_band_rms_mps": slow_band["rms"],
        "late_first_velocity_rms_sps": late_first_velocity_rms_sps,
        "late_second_velocity_rms_sps": late_second_velocity_rms_sps,
        "late_first_peak_sps": late_first_peak_sps,
        "late_second_peak_sps": late_second_peak_sps,
        "trend": "growing"
        if late_second_velocity_rms_sps
        > max(10.0, 1.25 * late_first_velocity_rms_sps)
        else "bounded",
    }


@pytest.fixture(scope="module", autouse=True)
def _outer_evidence_report(sim_artifact_settings):
    root = Path(sim_artifact_settings["temp_root"])
    for output_dir in root.rglob("outer_live_*"):
        if output_dir.is_dir():
            shutil.rmtree(output_dir)
    for pid_path in root.rglob("outer_pid_*.pid.conf"):
        pid_path.unlink()
    for report_path in (
        root / "outer_acceptance_report.json",
        root / "outer_acceptance_report.csv",
    ):
        report_path.unlink(missing_ok=True)
    yield
    rows = []
    for output_dir in sorted(root.rglob("outer_live_*")):
        timeline_path = output_dir / "timeline.csv"
        done_path = output_dir / "done.json"
        if not timeline_path.exists() or not done_path.exists():
            continue
        frame = read_telemetry_csv(timeline_path)
        metadata = json.loads((output_dir / "metadata.json").read_text(encoding="utf-8"))
        if frame.empty or float(frame["t_sec"].iloc[-1]) < 0.6 * float(metadata["duration_s"]):
            continue
        rows.append(_outer_report_row(output_dir))
    report = {
        "scope": "pytest live UDP simulator outer-loop acceptance runs",
        "run_count": len(rows),
        "rows": rows,
    }
    (root / "outer_acceptance_report.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )
    if rows:
        columns = list(rows[0])
        with (root / "outer_acceptance_report.csv").open(
            "w", encoding="utf-8", newline=""
        ) as stream:
            writer = csv.DictWriter(stream, fieldnames=columns)
            writer.writeheader()
            writer.writerows(rows)


@pytest.mark.parametrize("model", _model_params())
def test_outer_velocity_recovery_envelope_is_signed_and_authority_aware(
    simulator_udp, sim_artifact_settings, model: SimulatorModel
):
    peaks_by_force: dict[float, list[float]] = {}
    aggregate_failures: list[str] = []
    diagnostics = ScenarioDiagnostics("outer_velocity_recovery", model.label)
    for index, force_n in enumerate((0.5, 1.0, 2.0)):
        signed_peaks = []
        for sign in (1.0, -1.0):
            name = f"outer_live_recovery_{int(force_n * 10)}_{'plus' if sign > 0 else 'minus'}"
            output_dir = _model_artifact_dir(sim_artifact_settings, model, name)
            subrun_id = f"force_{force_n:g}_{'plus' if sign > 0 else 'minus'}"

            def checks_factory(summary, metadata, done, frame, force_n=force_n, sign=sign):
                peak_sps = float(
                    frame["plant_velocity_mps"].abs().max() / OUTER_METERS_PER_STEP
                )
                signed_peaks.append(peak_sps)
                stop_time_s, stopping_distance_m = _outer_stopping_metrics(frame, 10.4)
                checks = [
                    (
                        "bounded_balance",
                        lambda: _outer_assert_bounded(
                            summary,
                            metadata,
                            done,
                            frame,
                            model=model,
                            max_pitch_deg=(
                                25.0
                                if model == DIRECT_ACTUATOR_MODEL and force_n == 2.0
                                else 15.0
                            ),
                        ),
                    ),
                    (
                        "stopping_time",
                        lambda: assert_true(
                            stop_time_s is not None and stop_time_s <= 3.0,
                            f"stopping time {stop_time_s!r} exceeds 3 s",
                        ),
                    ),
                        (
                            "stopping_distance",
                            lambda: assert_true(
                                stopping_distance_m is not None
                                and stopping_distance_m
                                <= (0.20 if model == DIRECT_ACTUATOR_MODEL else 0.50),
                                f"stopping distance {stopping_distance_m!r} exceeds the model envelope",
                            ),
                        ),
                    (
                        "late_velocity",
                        lambda: assert_true(
                            summary["tail_mean_abs_velocity_mps"]
                            <= (0.025 if model == DIRECT_ACTUATOR_MODEL else 0.12),
                            "late velocity remains outside the hardware-inspired bound",
                        ),
                    ),
                        *([
                            (
                                "position_bound",
                                lambda: assert_true(
                                    summary["max_abs_position_m"] <= 0.20,
                                    "position exceeds 0.20 m",
                                ),
                            )
                        ] if model == DIRECT_ACTUATOR_MODEL else []),
                    (
                        "com_trim_bound",
                        lambda: assert_true(
                            frame["com_trim_deg"].abs().max() < 0.5,
                            "COM trim exceeds 0.5 degrees",
                        ),
                    ),
                ]
                if force_n == 2.0:
                    authority = frame["outer_acceleration_limited"] > 0.5
                    if authority.any():
                        checks.extend(
                            [
                                (
                                    "authority_request",
                                    lambda: assert_true(
                                        frame.loc[authority, "acceleration_raw_mps2"].abs().max()
                                        > 9.81
                                        * math.tan(
                                            math.radians(
                                                float(frame["active_outer_pitch_limit_deg"].iloc[-1])
                                            )
                                        ),
                                        "outer authority flag lacks an over-limit acceleration request",
                                    ),
                                ),
                                (
                                    "trim_learning_blocked",
                                    lambda: assert_true(
                                        frame.loc[authority, "trim_learning_enabled"].max() == 0,
                                        "COM trim learned while velocity authority was limited",
                                    ),
                                ),
                            ]
                        )
                    else:
                        diagnostics.record_diagnostic(
                            "authority_not_exercised",
                            f"{subrun_id} stayed below the configured outer acceleration limit",
                        )
                return checks

            _run_outer_subrun(
                diagnostics,
                simulator_udp,
                model,
                output_dir,
                subrun_id,
                {
                    "run_id": 4000 + index * 10 + (1 if sign > 0 else 2),
                    "physics_override": _outer_physics_override(model),
                    "duration_s": 60.0,
                    "telemetry_stride": 40,
                    "disturbances": [
                        {"start_s": 10.0, "duration_s": 0.4, "force_n": sign * force_n}
                    ],
                },
                checks_factory,
                scenario_id="outer_velocity_recovery",
                scenario_category="velocity_recovery",
                scenario_intent="arrest signed external pushes within bounded speed and distance",
            )

        peaks_by_force[force_n] = signed_peaks
        if len(signed_peaks) == 2 and abs(signed_peaks[0] - signed_peaks[1]) > 10.0:
            aggregate_failures.append(
                f"force {force_n:g} signed peak mismatch {signed_peaks[0]:.1f} vs {signed_peaks[1]:.1f} SPS"
            )
        elif len(signed_peaks) != 2:
            aggregate_failures.append(
                f"force {force_n:g} has {len(signed_peaks)} completed signed peak observations"
            )

    if all(len(peaks_by_force[force_n]) == 2 for force_n in (0.5, 1.0, 2.0)):
        if not (
            peaks_by_force[0.5][0]
            < peaks_by_force[1.0][0]
            < peaks_by_force[2.0][0]
        ):
            aggregate_failures.append("peak response is not monotonic with disturbance force")
    else:
        aggregate_failures.append("not enough complete force-response pairs for monotonicity")
    for message in aggregate_failures:
        diagnostics.record_failure("aggregate_symmetry_or_order", message)
    _finish_composite(diagnostics)


@pytest.mark.parametrize("model", _model_params())
def test_outer_initial_velocity_recovery_envelope_is_signed(
    simulator_udp, sim_artifact_settings, model: SimulatorModel
):
    recovery = {}
    aggregate_failures: list[str] = []
    diagnostics = ScenarioDiagnostics("outer_initial_velocity_recovery", model.label)
    for index, initial_sps in enumerate(
        (100.0, 500.0, 1000.0, 1500.0, 2000.0, 2500.0, 3000.0, 3500.0)
    ):
        signed = []
        for sign in (1.0, -1.0):
            output_dir = _model_artifact_dir(
                sim_artifact_settings,
                model,
                f"outer_live_initial_{int(initial_sps)}_{'plus' if sign > 0 else 'minus'}",
            )
            subrun_id = f"{int(initial_sps)}_{'plus' if sign > 0 else 'minus'}"

            def checks_factory(summary, metadata, done, frame):
                stopping_time_s, stopping_distance_m = _outer_stopping_metrics(frame, 0.0)
                signed.append(
                    {
                        "peak_sps": float(
                            frame["plant_velocity_mps"].abs().max() / OUTER_METERS_PER_STEP
                        ),
                        "stopping_time_s": stopping_time_s,
                        "stopping_distance_m": stopping_distance_m,
                    }
                )
                return [
                    (
                        "bounded_balance",
                        lambda: _outer_assert_bounded(
                            summary,
                            metadata,
                            done,
                            frame,
                            model=model,
                            max_pitch_deg=12.0,
                        ),
                    ),
                    (
                        "stopping_time",
                        lambda: assert_true(
                            stopping_time_s is not None and stopping_time_s <= 8.0,
                            f"stopping time {stopping_time_s!r} exceeds 8 s",
                        ),
                    ),
                    (
                        "stopping_distance",
                        lambda: assert_true(
                            stopping_distance_m is not None and stopping_distance_m <= 0.75,
                            f"stopping distance {stopping_distance_m!r} exceeds 0.75 m",
                        ),
                    ),
                    (
                        "late_velocity",
                        lambda: assert_true(
                            summary["tail_mean_abs_velocity_mps"]
                            <= (0.003 if model == DIRECT_ACTUATOR_MODEL else 0.005),
                            "late velocity exceeds the model-specific bounded-recovery limit",
                        ),
                    ),
                    (
                        "com_trim_bound",
                        lambda: assert_true(
                            frame["com_trim_deg"].abs().max() < 0.6,
                            "COM trim exceeds 0.6 degrees",
                        ),
                    ),
                ]

            _run_outer_subrun(
                diagnostics,
                simulator_udp,
                model,
                output_dir,
                subrun_id,
                {
                    "run_id": 4050 + index * 10 + (1 if sign > 0 else 2),
                    "physics_override": _outer_physics_override(model),
                    "duration_s": 90.0,
                    "telemetry_stride": 40,
                    "initial_velocity_mps": sign * initial_sps * OUTER_METERS_PER_STEP,
                },
                checks_factory,
                scenario_id="outer_initial_velocity_recovery",
                scenario_category="velocity_recovery",
                scenario_intent="recover signed initial wheel velocity without trim runaway",
            )
        recovery[initial_sps] = signed
        if len(signed) != 2:
            aggregate_failures.append(
                f"initial velocity {initial_sps:g} has {len(signed)} signed observations"
            )
        else:
            if abs(signed[0]["peak_sps"] - signed[1]["peak_sps"]) > 15.0:
                aggregate_failures.append(f"initial velocity {initial_sps:g} peak asymmetry")
            if (
                signed[0]["stopping_time_s"] is None
                or signed[1]["stopping_time_s"] is None
                or abs(signed[0]["stopping_time_s"] - signed[1]["stopping_time_s"]) > 0.15
            ):
                aggregate_failures.append(f"initial velocity {initial_sps:g} stop-time asymmetry")
            if (
                signed[0]["stopping_distance_m"] is None
                or signed[1]["stopping_distance_m"] is None
                or abs(signed[0]["stopping_distance_m"] - signed[1]["stopping_distance_m"]) > 0.05
            ):
                aggregate_failures.append(
                    f"initial velocity {initial_sps:g} stop-distance asymmetry"
                )

    peaks = [recovery[sps][0]["peak_sps"] for sps in recovery if len(recovery[sps]) == 2]
    monotonic_tolerance_sps = 0.0 if model == DIRECT_ACTUATOR_MODEL else 20.0
    if any(
        later + monotonic_tolerance_sps < earlier
        for earlier, later in zip(peaks, peaks[1:])
    ):
        aggregate_failures.append("initial velocity peak response is not monotonic")
    for message in aggregate_failures:
        diagnostics.record_failure("aggregate_symmetry_or_order", message)
    _finish_composite(diagnostics)


@pytest.mark.parametrize("model", _model_params())
def test_outer_hardware_startup_recovery_is_an_authority_audit_regression(
    simulator_udp, sim_artifact_settings, model: SimulatorModel
):
    """Replay the first hardware telemetry sample as a deterministic E2E seed."""
    output_dir = _model_artifact_dir(
        sim_artifact_settings, model, "outer_live_hardware_startup_recovery"
    )
    hardware_pitch_deg = 0.5929913520812988
    hardware_pitch_rate_dps = -13.041985511779783
    hardware_velocity_sps = 144.31312561035156
    result = _run_model_scenario(
        simulator_udp,
        model,
        run_id=4685,
        output_dir=output_dir,
        physics_override=_outer_physics_override(model),
        duration_s=22.0,
        telemetry_stride=1,
        initial_pitch_deg=hardware_pitch_deg,
        initial_pitch_rate_dps=hardware_pitch_rate_dps,
        initial_velocity_mps=hardware_velocity_sps * OUTER_METERS_PER_STEP,
    )
    summary, metadata, done, frame = (
        result.summary,
        result.metadata,
        result.done,
        result.frame,
    )
    _outer_assert_bounded(summary, metadata, done, frame, model=model, max_pitch_deg=20.0)
    assert metadata["initial_pitch_deg"] == hardware_pitch_deg
    assert metadata["initial_pitch_rate_dps"] == hardware_pitch_rate_dps
    assert math.isclose(
        metadata["initial_velocity_mps"], hardware_velocity_sps * OUTER_METERS_PER_STEP
    )

    neutral = frame[(frame["t_sec"] >= 0.65) & (frame["t_sec"] <= 15.7)]
    authority = neutral[neutral["outer_acceleration_limited"] > 0.5]
    if not authority.empty:
        assert (
            authority["drive_pitch_target_deg"].abs()
            >= authority["active_outer_pitch_limit_deg"] - 0.01
        ).all()
        assert authority["trim_learning_enabled"].max() == 0
    # This is a bounded replay regression, not a claim that the nominal
    # simulator reproduces the hardware's attitude transient. The separate
    # telemetry audit records that actual pitch did not track this target.
    assert neutral["pitch_deg"].abs().max() < 20.0
    assert frame["plant_velocity_mps"].abs().iloc[-1] / OUTER_METERS_PER_STEP < 100.0


@pytest.mark.parametrize(
    "model",
    _model_params(
        stepper_xfail=(
            "StepperPhaseElectrical direct constant-lean authority diagnostics "
            "reach the known electrical phase/safety boundary; the nominal "
            "velocity-reference path is tested separately"
        )
    ),
)
def test_pitch_authority_direct_target_sweep_is_end_to_end_and_isolated(
    simulator_udp, sim_artifact_settings, model: SimulatorModel
):
    """Exercise the future hardware diagnostic target through the maintained SIL path."""
    output_dir = _model_artifact_dir(sim_artifact_settings, model, "pitch_authority_direct_sweep")
    segments = [
        {"start_s": 0.8, "duration_s": 0.45, "target_deg": target, "com_trim_deg": 0.0}
        for target in (1.0, -1.0, 2.0, -2.0, 4.0, -4.0)
    ]
    for index, segment in enumerate(segments):
        segment["start_s"] = 0.8 + index * 1.2
    result = _run_model_scenario(
        simulator_udp,
        model,
        run_id=4690,
        output_dir=output_dir,
        physics_override=_outer_physics_override(model),
        duration_s=8.5,
        telemetry_stride=1,
        pitch_authority_segments=segments,
        fail_fast_pitch_deg=35.0,
    )
    summary, metadata, done, frame = (
        result.summary,
        result.metadata,
        result.done,
        result.frame,
    )
    _assert_common_integrity(
        summary, metadata, done, model=model, expected_physics_override=_outer_physics_override(model)
    )
    assert done.reason_code == DONE_COMPLETED
    assert not summary["fell"]
    assert frame["pitch_authority_diagnostic_active"].all()
    assert frame["pitch_authority_diagnostic_remaining_s"].min() > 0.0
    assert (frame["pitch_authority_diagnostic_request_id"] > 0).all()
    assert frame["pitch_authority_diagnostic_command_age_ms"].max() <= 5.0
    assert frame["trim_learning_enabled"].max() == 0
    assert frame["trim_learning_allowed"].max() == 0
    assert frame["acceleration_raw_mps2"].abs().max() <= 1e-6
    assert frame["acceleration_cmd_mps2"].abs().max() <= 1e-6
    assert (frame["pitch_target_unclamped_deg"] - frame["pitch_sp_deg"]).abs().max() <= 1e-6

    pulse_rows = analyze_pitch_authority_sweep(frame)
    assert [row["requested_target_deg"] for row in pulse_rows] == [
        1.0,
        -1.0,
        2.0,
        -2.0,
        4.0,
        -4.0,
    ]
    for row in pulse_rows:
        assert row["response_polarity"] == row["target_polarity"]
        assert row["response_latency_s"] is not None
        assert row["peak_actual_pitch_deg"] is not None
        assert row["actual_target_gain"] is not None and row["actual_target_gain"] > 0.0
    assert metadata["pitch_authority_segments"] == segments


@pytest.mark.parametrize("model", _model_params())
def test_pitch_authority_watchdog_expires_on_refresh_dropout(
    simulator_udp, sim_artifact_settings, model: SimulatorModel
):
    output_dir = _model_artifact_dir(sim_artifact_settings, model, "pitch_authority_watchdog_dropout")
    segments = _direct_pitch_train(hold_s=1.0, rest_s=0.5, targets=(1.0,))
    result = _run_model_scenario(
        simulator_udp,
        model,
        run_id=4695,
        output_dir=output_dir,
        physics_override=_outer_physics_override(model),
        duration_s=2.75,
        telemetry_stride=1,
        pitch_authority_segments=segments,
        pitch_authority_refresh_dropout={"start_s": 0.90, "duration_s": 0.12},
        fail_fast_pitch_deg=35.0,
    )
    summary, metadata, done, frame = (
        result.summary,
        result.metadata,
        result.done,
        result.frame,
    )
    _assert_common_integrity(
        summary, metadata, done, model=model, expected_physics_override=_outer_physics_override(model)
    )
    assert done.reason_code == DONE_COMPLETED
    dropout = frame[(frame["t_sec"] >= 0.95) & (frame["t_sec"] < 1.015)]
    assert not dropout.empty
    assert dropout["pitch_authority_diagnostic_active"].max() == 0.0
    assert dropout["pitch_authority_diagnostic_remaining_s"].max() == 0.0
    assert dropout["pitch_authority_diagnostic_command_age_ms"].max() == 0.0
    assert frame["pitch_authority_diagnostic_active"].iloc[0]
    assert metadata["pitch_authority_refresh_dropout"] == {"start_s": 0.90, "duration_s": 0.12}

    last_refresh = frame[frame["t_sec"] < 0.90].iloc[-1]
    assert last_refresh["pitch_authority_diagnostic_request_id"] == 897
    assert last_refresh["pitch_authority_diagnostic_command_age_ms"] <= 5.0
    expiry = frame[
        (frame["t_sec"] >= 0.94) & (frame["pitch_authority_diagnostic_active"] < 0.5)
    ].iloc[0]
    assert expiry["t_sec"] == pytest.approx(0.945, abs=0.003)
    assert expiry["pitch_authority_diagnostic_active"] == 0.0
    assert expiry["pitch_authority_diagnostic_request_id"] == 0.0
    assert expiry["pitch_target_unclamped_deg"] == pytest.approx(0.0, abs=1e-6)
    assert expiry["pitch_sp_deg"] == pytest.approx(0.0, abs=1e-6)
    # Diagnostic expiry returns to the ordinary balance path; it is not an
    # E-stop and therefore must not be described as motor-output zeroing.
    assert abs(expiry["u_sps"]) > 0.0
    assert expiry["controller_fault_flags"] == 0.0


@pytest.mark.parametrize(
    "model",
    _model_params(
        stepper_xfail=(
            "StepperPhaseElectrical direct constant-lean authority diagnostics "
            "reach the known electrical phase/safety boundary; the nominal "
            "velocity-reference path is tested separately"
        )
    ),
)
def test_pitch_authority_first_stage_requires_zero_start_and_survives_repeated_pulses(
    simulator_udp, sim_artifact_settings, model: SimulatorModel
):
    """Exercise the exact first-stage precondition and repeated ±1° sequence."""
    diagnostics = ScenarioDiagnostics(
        "pitch_authority_first_stage", model.label
    )
    output_dir = _model_artifact_dir(
        sim_artifact_settings, model, "pitch_authority_first_pm1_repeated"
    )
    segments = _direct_pitch_train(
        hold_s=0.20, rest_s=2.0, targets=(1.0, -1.0, 1.0, -1.0)
    )
    try:
        result = _run_model_scenario(
            simulator_udp,
            model,
            run_id=4696,
            output_dir=output_dir,
            scenario_id="pitch_authority_first_stage",
            subrun_id="repeated_pm1",
            scenario_category="pitch_authority_diagnostics",
            scenario_intent="zero-start repeated signed one-degree authority witness",
            physics_override=_outer_physics_override(model),
            duration_s=10.5,
            telemetry_stride=2,
            pitch_authority_segments=segments,
            fail_fast_pitch_deg=35.0,
        )
        summary, metadata, done, frame = (
            result.summary,
            result.metadata,
            result.done,
            result.frame,
        )
        startup = frame[frame["t_sec"] < 0.75]
        rows = analyze_pitch_authority_sweep(frame)
    except Exception as exc:
        diagnostics.record_infrastructure_failure("repeated_pm1", repr(exc))
        diagnostics.write(output_dir)
        _finish_composite(diagnostics)
        return

    checks = [
        (
            "common_integrity",
            lambda: _assert_common_integrity(
                summary,
                metadata,
                done,
                model=model,
                expected_physics_override=_outer_physics_override(model),
            ),
        ),
        ("completed", lambda: assert_done_completed(done)),
        ("not_fallen", lambda: assert_not_fallen(summary)),
        (
            "zero_start_pitch",
            lambda: assert_true(
                startup["plant_pitch_deg"].abs().max() < 1e-6,
                "non-zero startup pitch",
            ),
        ),
        (
            "zero_start_rate",
            lambda: assert_true(
                startup["plant_pitch_rate_dps"].abs().max() < 1e-6,
                "non-zero startup pitch rate",
            ),
        ),
        (
            "zero_start_velocity",
            lambda: assert_true(
                startup["plant_velocity_mps"].abs().max() < 1e-9,
                "non-zero startup velocity",
            ),
        ),
        (
            "target_sequence",
            lambda: assert_true(
                [row["requested_target_deg"] for row in rows]
                == [1.0, -1.0, 1.0, -1.0],
                "repeated pulse target sequence changed",
            ),
        ),
        (
            "response_polarity",
            lambda: assert_true(
                all(row["response_polarity"] == row["target_polarity"] for row in rows),
                "authority response polarity changed",
            ),
        ),
        (
            "zero_recovery_witness",
            lambda: assert_true(
                all(row["zero_recovery_time_s"] is not None for row in rows),
                "one or more pulses lacks a zero-recovery witness",
            ),
        ),
    ]
    # The hardware-envelope validator is intentionally retained for the
    # DirectActuator reference, but it is not a shared behavioral requirement:
    # it encodes the optimistic direct-force response, not merely safe signed
    # authority.  Record it in the diagnostics for the electrical model
    # without turning a known model-shape difference into a false shared gate.
    if model == DIRECT_ACTUATOR_MODEL:
        checks.append(
            (
                "hardware_reference_envelope",
                lambda: assert_true(
                    validate_pitch_authority_hardware_envelope(rows) == [],
                    "DirectActuator hardware-envelope witness failed",
                ),
            )
        )
    else:
        # This is a diagnostic-only record, deliberately not a failure.  The
        # aggregate subrun still carries the physical metrics.
        diagnostics.record_diagnostic(
            "model_specific_hardware_envelope",
            "model_specific_diagnostic",
            metrics={
                "hardware_envelope_failures": len(
                    validate_pitch_authority_hardware_envelope(rows)
                ),
                "zero_recovery_available": all(
                    row["zero_recovery_time_s"] is not None for row in rows
                ),
            },
        )
    _evaluate_subrun(
        diagnostics,
        output_dir,
        "repeated_pm1",
        summary,
        done,
        checks,
        classification="genuine_behavioral_failure",
    )
    _finish_composite(diagnostics)


def _direct_pitch_train(
    *, hold_s: float, rest_s: float, targets: tuple[float, ...]
) -> list[dict[str, float]]:
    segments: list[dict[str, float]] = []
    start_s = 0.8
    for target in targets:
        segments.append(
            {
                "start_s": start_s,
                "duration_s": hold_s,
                "target_deg": target,
                "com_trim_deg": 0.0,
            }
        )
        start_s += hold_s + rest_s
    # Keep the diagnostic path active at zero target after the last pulse. This
    # leaves the ordinary velocity, drive, and COM paths isolated while the
    # body returns to its neutral target.
    segments.append(
        {
            "start_s": start_s - rest_s,
            "duration_s": 0.0,
            "target_deg": 0.0,
            "com_trim_deg": 0.0,
        }
    )
    return segments


@pytest.mark.parametrize(
    "model",
    _model_params(
        stepper_xfail=(
            "StepperPhaseElectrical direct constant-lean authority diagnostics "
            "reach the known electrical phase/safety boundary; the nominal "
            "velocity-reference path is tested separately"
        )
    ),
)
def test_pitch_authority_long_holds_and_reversals_have_event_metrics(
    simulator_udp, sim_artifact_settings, model: SimulatorModel
):
    """Keep a longer direct-target train as a shared realization reference."""
    output_dir = _model_artifact_dir(sim_artifact_settings, model, "pitch_authority_long_train")
    segments = _direct_pitch_train(
        hold_s=4.0, rest_s=2.0, targets=(1.0, -1.0, 2.0, -2.0, 4.0, -4.0)
    )
    result = _run_model_scenario(
        simulator_udp,
        model,
        run_id=4691,
        output_dir=output_dir,
        physics_override=_outer_physics_override(model),
        duration_s=39.0,
        telemetry_stride=10,
        pitch_authority_segments=segments,
        fail_fast_pitch_deg=45.0,
        done_timeout=120.0,
    )
    summary, metadata, done, frame = (
        result.summary,
        result.metadata,
        result.done,
        result.frame,
    )
    _assert_common_integrity(
        summary, metadata, done, model=model, expected_physics_override=_outer_physics_override(model)
    )
    assert done.reason_code == DONE_COMPLETED
    assert not summary["fell"]
    assert frame["pitch_authority_diagnostic_active"].all()
    assert frame["trim_learning_allowed"].max() == 0
    rows = analyze_pitch_authority_sweep(frame)
    assert [row["requested_target_deg"] for row in rows] == [1.0, -1.0, 2.0, -2.0, 4.0, -4.0]
    for row in rows:
        assert row["target_hold_s"] >= 3.95
        assert row["response_polarity"] == row["target_polarity"]
        assert row["rise_10_s"] is not None
        assert row["steady_actual_pitch_deg"] is not None
        assert row["steady_target_error_deg"] is not None
        assert row["motor_force_authority_fraction"] is not None
        assert row["traction_authority_fraction"] is not None
    # The 1 s zero-target tail must return the final target through the same
    # production path; it is a watchdog/neutral recovery check, not a tuning
    # assertion about the outer loop.
    tail = frame[frame["t_sec"] >= 36.0]
    assert tail["pitch_sp_deg"].abs().max() <= 1e-6
    assert tail["pitch_deg"].abs().max() < 15.0


@pytest.mark.parametrize("model", _model_params())
def test_pitch_authority_nominal_uncertainty_matrix_stays_within_reference_envelope(
    simulator_udp, sim_artifact_settings, model: SimulatorModel
):
    """Bounded no-slip sensitivity matrix for the shared controller reference.

    This is a model envelope, not a hardware fit. The retained phase/tire
    profiles are covered separately because they are not calibrated.
    """
    cases = (
        ("nominal", {}, 1.0, 1.0, 0.0),
        ("delay", {"motor_tau_s": 0.005}, 1.0, 1.0, 0.0),
        ("sensor_lag", {"motor_tau_s": 0.005}, 1.0, 1.0, 0.020),
        ("low_traction", {"motor_tau_s": 0.005, "traction_coefficient": 0.80}, 1.0, 1.0, 0.0),
        ("low_force", {
            "motor_tau_s": 0.005,
            "traction_coefficient": 0.80,
            "motor_max_force_n": 8.0,
        }, 1.0, 1.0, 0.0),
        ("mass_inertia_low", {"motor_tau_s": 0.005, "traction_coefficient": 0.80}, 0.90, 0.90, 0.0),
        ("mass_inertia_high", {"motor_tau_s": 0.005, "traction_coefficient": 0.80}, 1.10, 1.10, 0.0),
    )
    diagnostics = ScenarioDiagnostics(
        "pitch_authority_nominal_uncertainty", model.label
    )
    observed = []
    last_output_dir: Path | None = None
    for index, (name, override, mass_scale, inertia_scale, imu_lag_s) in enumerate(cases):
        for sign in (1.0, -1.0):
            subrun_id = f"{name}_{'plus' if sign > 0 else 'minus'}"
            output_dir = _model_artifact_dir(
                sim_artifact_settings,
                model,
                f"pitch_authority_uncertainty_{name}_{'p' if sign > 0 else 'n'}",
            )
            last_output_dir = output_dir
            physics_override = {**_outer_physics_override(model), **override}
            segments = _direct_pitch_train(
                hold_s=0.20, rest_s=2.0, targets=(1.0 * sign, 2.0 * sign, 4.0 * sign)
            )
            try:
                result = _run_model_scenario(
                    simulator_udp,
                    model,
                    run_id=4700 + index * 2 + (0 if sign > 0 else 1),
                    output_dir=output_dir,
                    scenario_id="pitch_authority_nominal_uncertainty",
                    subrun_id=subrun_id,
                    scenario_category="pitch_authority_diagnostics",
                    scenario_intent="bounded signed authority under plant and sensor uncertainty",
                    physics_override=physics_override,
                    duration_s=8.0,
                    telemetry_stride=2,
                    total_mass_scale=mass_scale,
                    pitch_inertia_scale=inertia_scale,
                    imu_pitch_lag_s=imu_lag_s,
                    pitch_authority_segments=segments,
                    fail_fast_pitch_deg=40.0,
                )
                summary, metadata, done, frame = (
                    result.summary,
                    result.metadata,
                    result.done,
                    result.frame,
                )
            except Exception as exc:
                diagnostics.record_infrastructure_failure(subrun_id, repr(exc))
                diagnostics.write(output_dir)
                continue

            rows = analyze_pitch_authority_sweep(frame)
            for row in rows:
                observed.append((name, sign, row))
            checks = [
                (
                    "common_integrity",
                    lambda: _assert_common_integrity(
                        summary,
                        metadata,
                        done,
                        model=model,
                        expected_physics_override=physics_override,
                        expected_total_mass_scale=mass_scale,
                        expected_pitch_inertia_scale=inertia_scale,
                    ),
                ),
                ("completed", lambda: assert_done_completed(done)),
                ("not_fallen", lambda: assert_not_fallen(summary)),
                (
                    "three_pulse_rows",
                    lambda: assert_true(
                        len(rows) == 3, f"expected 3 pulse rows, got {len(rows)}"
                    ),
                ),
                (
                    "response_metrics",
                    lambda: assert_authority_rows(rows, sign),
                ),
                (
                    "force_not_saturated",
                    lambda: assert_true(
                        frame["force_saturated"].max() == 0.0,
                        "direct actuator force saturated",
                    ),
                ),
            ]
            _evaluate_subrun(
                diagnostics,
                output_dir,
                subrun_id,
                summary,
                done,
                checks,
            )

    if last_output_dir is not None:
        if len(observed) != 42:
            diagnostics.record_failure(
                "aggregate_observation_count",
                f"expected 42 authority observations, got {len(observed)}",
            )
        for name in {case[0] for case in cases}:
            for target in (1.0, 2.0, 4.0):
                pair = [
                    row
                    for case_name, _sign, row in observed
                    if case_name == name and abs(row["requested_target_deg"]) == target
                ]
                if len(pair) != 2:
                    diagnostics.record_failure(
                        "signed_pair_complete",
                        f"{name} target {target:g} has {len(pair)} signed observations",
                    )
                elif not math.isclose(
                    pair[0]["response_latency_s"],
                    pair[1]["response_latency_s"],
                    abs_tol=0.025,
                ):
                    diagnostics.record_failure(
                        "signed_latency_symmetry",
                        f"{name} target {target:g} response latency is not symmetric",
                    )
        diagnostics.write(last_output_dir)
    _finish_composite(diagnostics)


@pytest.mark.parametrize(
    "model",
    _model_params(
        stepper_xfail=(
            "StepperPhaseElectrical direct constant-lean authority diagnostics "
            "reach the known electrical phase/safety boundary; the nominal "
            "velocity-reference path is tested separately"
        )
    ),
)
def test_pitch_authority_direct_targets_cover_initial_condition_variation(
    simulator_udp, sim_artifact_settings, model: SimulatorModel
):
    cases = (
        ("pitch_pos", 1.0, 0.0, 0.0, 2.0),
        ("pitch_neg", -1.0, 0.0, 0.0, -2.0),
        ("rate_pos", 0.0, 20.0, 0.0, 2.0),
        ("rate_neg", 0.0, -20.0, 0.0, -2.0),
        ("velocity_pos", 0.0, 0.0, 0.02, 2.0),
        ("velocity_neg", 0.0, 0.0, -0.02, -2.0),
    )
    diagnostics = ScenarioDiagnostics(
        "pitch_authority_initial_condition_variation", model.label
    )
    observed = []
    for index, (name, initial_pitch, initial_rate, initial_velocity, target) in enumerate(cases):
        output_dir = _model_artifact_dir(
            sim_artifact_settings, model, f"pitch_authority_initial_{name}"
        )
        segments = _direct_pitch_train(hold_s=0.75, rest_s=0.75, targets=(target,))
        subrun_id = name

        def checks_factory(summary, metadata, done, frame, target=target):
            rows = analyze_pitch_authority_sweep(frame)
            observed.append((name, rows))
            return [
                (
                    "common_integrity",
                    lambda: _assert_common_integrity(
                        summary,
                        metadata,
                        done,
                        model=model,
                        expected_physics_override=_outer_physics_override(model),
                    ),
                ),
                ("completed", lambda: assert_done_completed(done)),
                ("not_fallen", lambda: assert_not_fallen(summary)),
                (
                    "one_pulse_row",
                    lambda: assert_true(len(rows) == 1, f"expected one pulse row, got {len(rows)}"),
                ),
                (
                    "response_polarity",
                    lambda: assert_true(
                        bool(rows) and rows[0]["response_polarity"] == (1 if target > 0.0 else -1),
                        "initial-condition authority response polarity changed",
                    ),
                ),
                (
                    "response_latency",
                    lambda: assert_true(
                        bool(rows) and rows[0]["response_latency_s"] is not None,
                        "initial-condition authority response has no latency witness",
                    ),
                ),
            ]

        _run_outer_subrun(
            diagnostics,
            simulator_udp,
            model,
            output_dir,
            subrun_id,
            {
                "run_id": 4720 + index,
                "physics_override": _outer_physics_override(model),
                "duration_s": 3.0,
                "telemetry_stride": 2,
                "initial_pitch_deg": initial_pitch,
                "initial_pitch_rate_dps": initial_rate,
                "initial_velocity_mps": initial_velocity,
                "pitch_authority_segments": segments,
                "fail_fast_pitch_deg": 35.0,
            },
            checks_factory,
            scenario_id="pitch_authority_initial_condition_variation",
            scenario_category="pitch_authority_diagnostics",
            scenario_intent="authority response from pitch, rate, and velocity initial conditions",
        )
    if len(observed) != len(cases):
        diagnostics.record_failure(
            "aggregate_observation_count",
            f"expected {len(cases)} initial-condition observations, got {len(observed)}",
        )
    _finish_composite(diagnostics)


@pytest.mark.parametrize("physics_profile", [PHYSICS_REALISTIC, PHYSICS_ACTUATOR_STRESS])
def test_pitch_authority_sweep_reports_nonideal_profile_sensitivity(
    simulator_udp, sim_artifact_settings, physics_profile: int
):
    """Record the retained non-ideal profile's safety boundary.

    These profiles contain the uncalibrated phase/tire actuator model and are
    deliberately diagnostic-only. The direct target can drive that model into
    its normal fall protection before the full nominal train completes. That
    outcome is evidence to report, not a reason to use the profile for tuning.
    """
    profile_name = "realistic" if physics_profile == PHYSICS_REALISTIC else "actuator_stress"
    output_dir = _artifact_dir(sim_artifact_settings, f"pitch_authority_{profile_name}")
    segments = _direct_pitch_train(hold_s=0.5, rest_s=0.75, targets=(1.0, -1.0, 2.0, -2.0, 4.0, -4.0))
    result = run_scenario_live(
        simulator_udp,
        run_id=4692 + physics_profile,
        output_dir=output_dir,
        physics_profile=physics_profile,
        duration_s=12.0,
        telemetry_stride=1,
        pitch_authority_segments=segments,
        fail_fast_pitch_deg=100.0,
    )
    summary, metadata, done, frame = (
        result.summary,
        result.metadata,
        result.done,
        result.frame,
    )
    _assert_common_integrity(
        summary,
        metadata,
        done,
        expected_physics_profile=("realistic" if physics_profile == PHYSICS_REALISTIC else "actuator_stress"),
    )
    assert done.reason_code in (DONE_COMPLETED, DONE_STOPPED_BY_CLIENT, DONE_FELL)
    rows = analyze_pitch_authority_sweep(frame)
    assert rows
    assert rows[0]["response_polarity"] == rows[0]["target_polarity"]
    assert rows[0]["response_latency_s"] is not None
    assert rows[0]["applied_force_peak_n"] is not None
    assert rows[0]["motor_force_authority_fraction"] is not None
    if done.reason_code != DONE_COMPLETED:
        # The retained non-ideal model must fail safe, rather than silently
        # producing an unbounded direct-target response.
        assert done.controller_fault_flags & (1 << 3)
        assert not summary["fell"] or done.reason_code == 2


@pytest.mark.parametrize("model", _model_params())
def test_outer_startup_combines_pitch_velocity_and_com_errors(
    simulator_udp, sim_artifact_settings, model: SimulatorModel
):
    diagnostics = ScenarioDiagnostics("outer_combined_startup", model.label)
    for index, sign in enumerate((1.0, -1.0)):
        output_dir = _model_artifact_dir(
            sim_artifact_settings,
            model,
            f"outer_live_startup_combined_{'plus' if sign > 0 else 'minus'}",
        )
        subrun_id = "plus" if sign > 0 else "minus"

        def checks_factory(summary, metadata, done, frame, sign=sign):
            return [
                (
                    "bounded_balance",
                    lambda: _outer_assert_bounded(
                        summary, metadata, done, frame, model=model, max_pitch_deg=8.0
                    ),
                ),
                (
                    "observer_valid_after_startup",
                    lambda: assert_true(
                        bool(frame["velocity_feedback_valid"].iloc[-1]),
                        "velocity feedback did not become valid after startup",
                    ),
                ),
                (
                    "adaptive_com_disabled",
                    lambda: assert_true(
                        frame["adaptive_com_trim_enabled"].max() == 0
                        and frame["trim_trusted"].max() == 0
                        and frame["com_trim_deg"].abs().max() <= 1e-6,
                        "default startup unexpectedly used adaptive COM trim",
                    ),
                ),
                (
                    "late_velocity",
                    lambda: assert_true(
                        summary["tail_mean_abs_velocity_mps"] <= 0.02,
                        "combined startup retains excessive late velocity",
                    ),
                ),
                (
                    "position_bound",
                    lambda: assert_true(
                        summary["max_abs_position_m"] <= 1.5,
                        "combined startup translates without a bounded recovery",
                    ),
                ),
            ]

        _run_outer_subrun(
            diagnostics,
            simulator_udp,
            model,
            output_dir,
            subrun_id,
            {
                "run_id": 4600 + index,
                    "physics_override": _outer_physics_override(model),
                "duration_s": 100.0,
                "telemetry_stride": 40,
                "initial_pitch_deg": sign * 3.0,
                "initial_velocity_mps": sign * 1500.0 * OUTER_METERS_PER_STEP,
                "com_angle_offset_rad": sign * 0.008,
            },
            checks_factory,
            scenario_id="outer_combined_startup",
            scenario_category="startup_recovery",
            scenario_intent="recover simultaneous pitch, wheel velocity, and COM error",
        )
    _finish_composite(diagnostics)


@pytest.mark.parametrize("model", _model_params())
def test_outer_velocity_estimator_bias_scale_and_latency_remain_bounded(
    simulator_udp, sim_artifact_settings, model: SimulatorModel
):
    cases = (
        ("bias_plus", {"velocity_estimator_bias_mps": 0.002}),
        ("bias_minus", {"velocity_estimator_bias_mps": -0.002}),
        ("scale_low", {"velocity_estimator_scale": 0.95}),
        ("scale_high", {"velocity_estimator_scale": 1.05}),
        ("bias_drift", {"velocity_estimator_bias_drift_mps_per_s": 0.00001}),
    )
    metrics = {}
    diagnostics = ScenarioDiagnostics("outer_velocity_estimator_perturbations", model.label)
    for index, (name, estimator_kwargs) in enumerate(cases):
        output_dir = _model_artifact_dir(
            sim_artifact_settings, model, f"outer_live_estimator_{name}"
        )
        def checks_factory(summary, metadata, done, frame, name=name):
            late = _outer_window_metrics(frame, 90.0, 120.0)
            metrics[name] = late
            return [
                (
                    "bounded_balance",
                    lambda: _outer_assert_bounded(
                        summary, metadata, done, frame, model=model, max_pitch_deg=6.0
                    ),
                ),
                (
                    "late_velocity_rms",
                    lambda: assert_true(
                        late["velocity_rms_sps"]
                            <= (180.0 if model == DIRECT_ACTUATOR_MODEL else 250.0),
                        f"late velocity RMS {late['velocity_rms_sps']:.1f} SPS exceeds the model limit",
                    ),
                ),
                (
                    "late_pitch_rms",
                    lambda: assert_true(
                        late["pitch_rms_deg"] <= 1.0,
                        f"late pitch RMS {late['pitch_rms_deg']:.3f} degrees exceeds 1",
                    ),
                ),
                (
                    "trim_span",
                    lambda: assert_true(
                        late["trim_max_deg"] - late["trim_min_deg"] <= 0.6,
                        "estimator perturbation causes excessive trim span",
                    ),
                ),
                (
                    "trim_slope",
                    lambda: assert_true(
                        abs(_outer_late_slope(frame, "com_trim_deg", 90.0)) <= 0.02,
                        "estimator perturbation causes growing COM trim",
                    ),
                ),
            ]

        _run_outer_subrun(
            diagnostics,
            simulator_udp,
            model,
            output_dir,
            name,
            {
                "run_id": 4650 + index,
                "physics_override": _outer_physics_override(model),
                "duration_s": 120.0,
                "telemetry_stride": 40,
                "com_angle_offset_rad": 0.004,
                **estimator_kwargs,
            },
            checks_factory,
            scenario_id="outer_velocity_estimator_perturbations",
            scenario_category="velocity_estimator_error",
            scenario_intent="remain bounded under signed bias, scale, latency, and drift errors",
        )
    if "bias_plus" in metrics and "bias_minus" in metrics:
        if abs(
            metrics["bias_plus"]["velocity_rms_sps"]
            - metrics["bias_minus"]["velocity_rms_sps"]
            ) > (20.0 if model == DIRECT_ACTUATOR_MODEL else 75.0):
            diagnostics.record_failure("aggregate_bias_symmetry", "signed estimator bias is asymmetric")
    else:
        diagnostics.record_failure("aggregate_bias_symmetry", "missing signed estimator bias pair")
    if "scale_low" in metrics and "scale_high" in metrics:
        if abs(
            metrics["scale_low"]["velocity_rms_sps"]
            - metrics["scale_high"]["velocity_rms_sps"]
        ) > 20.0:
            diagnostics.record_failure("aggregate_scale_symmetry", "estimator scale response is asymmetric")
    else:
        diagnostics.record_failure("aggregate_scale_symmetry", "missing estimator scale pair")
    _finish_composite(diagnostics)


@pytest.mark.xfail(
    strict=True,
    reason=(
        "StepperPhaseElectrical 100 ms controller-velocity latency drives the "
        "outer loop into a sustained high-pitch electrical recovery boundary"
    ),
)
def test_outer_velocity_estimator_latency_is_known_electrical_boundary(
    simulator_udp, sim_artifact_settings
):
    """Retain the latency boundary without weakening nominal feedback tests."""
    model = STEPPER_PHASE_ELECTRICAL_MODEL
    output_dir = _model_artifact_dir(
        sim_artifact_settings, model, "outer_live_estimator_latency_boundary"
    )
    result = _run_model_scenario(
        simulator_udp,
        model,
        output_dir=output_dir,
        run_id=4654,
        physics_override=_outer_physics_override(model),
        duration_s=120.0,
        telemetry_stride=40,
        com_angle_offset_rad=0.004,
        velocity_estimator_latency_s=0.10,
    )
    _assert_common_integrity(
        result.summary,
        result.metadata,
        result.done,
        model=model,
        expected_physics_override=_outer_physics_override(model),
    )
    assert result.done.reason_code == DONE_COMPLETED
    assert result.summary["max_abs_pitch_deg"] <= 6.0


@pytest.mark.parametrize("model", _model_params())
def test_outer_transient_authority_saturation_recovers_without_trim_growth(
    simulator_udp, sim_artifact_settings, model: SimulatorModel
):
    results = []
    diagnostics = ScenarioDiagnostics("outer_transient_authority", model.label)
    authority_pid_path = _outer_pid_variant(
        sim_artifact_settings, model, damping=0.5, limit_deg=0.25
    )
    for index, sign in enumerate((1.0, -1.0)):
        output_dir = _model_artifact_dir(
            sim_artifact_settings,
            model,
            f"outer_live_sustained_authority_{'plus' if sign > 0 else 'minus'}",
        )
        subrun_id = "plus" if sign > 0 else "minus"

        def checks_factory(summary, metadata, done, frame):
            authority = frame["outer_acceleration_limited"] > 0.5
            after = frame[frame["t_sec"] >= 10.0]
            results.append(frame)
            return [
                (
                    "bounded_balance",
                    lambda: _outer_assert_bounded(
                        summary,
                        metadata,
                        done,
                        frame,
                        model=model,
                        expected_velocity_pitch_limit_deg=0.25,
                        max_pitch_deg=12.0,
                    ),
                ),
                (
                    "authority_witness",
                    lambda: assert_true(authority.sum() >= 5, "authority limit was not exercised"),
                ),
                (
                    "trim_learning_blocked",
                    lambda: assert_true(
                        frame.loc[authority, "trim_learning_enabled"].max() == 0,
                        "COM trim learned during authority limitation",
                    ),
                ),
                (
                    "authority_recovers",
                    lambda: assert_true(
                        not (after["outer_acceleration_limited"] > 0.5).any(),
                        "authority remains limited after 10 s",
                    ),
                ),
                (
                    "post_transient_speed",
                    lambda: assert_true(
                        after["plant_velocity_mps"].abs().max() / OUTER_METERS_PER_STEP <= 160.0,
                        "post-transient speed exceeds 160 SPS",
                    ),
                ),
                (
                    "com_trim_bound",
                    lambda: assert_true(
                        frame["com_trim_deg"].abs().max() < 0.6,
                        "COM trim exceeds 0.6 degrees",
                    ),
                ),
            ]

        _run_outer_subrun(
            diagnostics,
            simulator_udp,
            model,
            output_dir,
            subrun_id,
            {
                "run_id": 4680 + index,
                "pid_config_path": authority_pid_path,
                "physics_override": _outer_physics_override(model),
                "duration_s": 20.0,
                "telemetry_stride": 1,
                "com_angle_offset_rad": 0.004,
                # Preserve the old physical disturbance after 1/32 doubles the
                # SPS representation of the same wheel speed.
                "initial_velocity_mps": sign * 7000.0 * OUTER_METERS_PER_STEP,
            },
            checks_factory,
            scenario_id="outer_transient_authority",
            scenario_category="authority_and_saturation",
            scenario_intent="recover from wheel-speed authority saturation without trim growth",
        )

    if len(results) == 2:
        asymmetry = abs(
            float(results[0]["plant_velocity_mps"].abs().max())
            - float(results[1]["plant_velocity_mps"].abs().max())
        ) / OUTER_METERS_PER_STEP
        if asymmetry > 80.0:
            diagnostics.record_failure(
                "aggregate_signed_speed_symmetry",
                f"signed transient peak speed differs by {asymmetry:.1f} SPS",
            )
    else:
        diagnostics.record_failure(
            "aggregate_signed_speed_symmetry",
            f"expected two signed transient runs, got {len(results)}",
        )
    _finish_composite(diagnostics)


@pytest.mark.parametrize("model", _model_params())
def test_outer_gain_authority_region_is_broad_and_symmetric(
    simulator_udp, sim_artifact_settings, model: SimulatorModel
):
    candidates = (
        (6.0, 3.0),
        (6.0, 4.0),
        (8.0, 3.0),
        (8.0, 4.0),
        (8.0, 5.0),
        (8.0, 6.0),
        (10.0, 4.0),
        (10.0, 5.0),
    )
    results = []
    diagnostics = ScenarioDiagnostics("outer_gain_authority_region", model.label)
    for index, (damping, limit_deg) in enumerate(candidates):
        pid_path = _outer_pid_variant(sim_artifact_settings, model, damping, limit_deg)
        output_dir = _model_artifact_dir(
            sim_artifact_settings,
            model,
            f"outer_live_region_{damping:g}_{limit_deg:g}",
        )
        subrun_id = f"damping_{damping:g}_limit_{limit_deg:g}"

        def checks_factory(summary, metadata, done, frame, damping=damping, limit_deg=limit_deg):
            late = _outer_window_metrics(frame, 50.0, 70.0)
            results.append((damping, limit_deg, late))
            return [
                (
                    "bounded_balance",
                    lambda: _outer_assert_bounded(
                        summary,
                        metadata,
                        done,
                        frame,
                        model=model,
                        expected_velocity_pitch_limit_deg=limit_deg,
                        max_pitch_deg=15.0,
                    ),
                ),
                (
                    "late_velocity_rms",
                    lambda: assert_true(
                        late["velocity_rms_sps"] <= 80.0,
                        f"late velocity RMS {late['velocity_rms_sps']:.1f} SPS exceeds 80",
                    ),
                ),
                (
                    "late_pitch_rms",
                    lambda: assert_true(
                        late["pitch_rms_deg"] <= 1.0,
                        f"late pitch RMS {late['pitch_rms_deg']:.3f} degrees exceeds 1",
                    ),
                ),
                (
                    "command_authority",
                    lambda: assert_true(
                        late["command_peak_sps"] <= 16000.0,
                        f"late command peak {late['command_peak_sps']:.1f} exceeds 16 kSPS",
                    ),
                ),
                (
                    "trim_span",
                    lambda: assert_true(
                        late["trim_max_deg"] - late["trim_min_deg"] <= 0.6,
                        "gain candidate causes excessive COM trim span",
                    ),
                ),
            ]

        _run_outer_subrun(
            diagnostics,
            simulator_udp,
            model,
            output_dir,
            subrun_id,
            {
                "run_id": 4700 + index,
                "physics_override": _outer_physics_override(model),
                "duration_s": 70.0,
                "telemetry_stride": 40,
                "pid_config_path": pid_path,
                "disturbances": [
                    {"start_s": 8.0, "duration_s": 0.4, "force_n": 1.5},
                    {"start_s": 28.0, "duration_s": 0.4, "force_n": -1.5},
                ],
            },
            checks_factory,
            scenario_id="outer_gain_authority_region",
            scenario_category="outer_gain_region",
            scenario_intent="identify a broad bounded outer-loop damping and pitch-limit region",
        )

    selected = next((item for item in results if item[:2] == (8.0, 4.0)), None)
    if selected is None:
        diagnostics.record_failure("aggregate_reference_candidate", "8/4 candidate produced no report")
    else:
        selected_rms = selected[2]["velocity_rms_sps"]
        if any(
            abs(item[2]["velocity_rms_sps"] - selected_rms) > 40.0 for item in results
        ):
            diagnostics.record_failure(
                "aggregate_gain_region_spread",
                "gain candidates do not remain within the broad reference RMS envelope",
            )
    _finish_composite(diagnostics)


@pytest.mark.xfail(
    strict=True,
    reason=(
        "StepperPhaseElectrical P=8/s with 2.5 degree motion authority is a "
        "known high-demand electrical fall/rail boundary"
    ),
)
def test_outer_gain_authority_8_per_s_2p5deg_is_known_boundary(
    simulator_udp, sim_artifact_settings
):
    model = STEPPER_PHASE_ELECTRICAL_MODEL
    pid_path = _outer_pid_variant(sim_artifact_settings, model, 8.0, 2.5)
    output_dir = _model_artifact_dir(
        sim_artifact_settings, model, "outer_live_region_8_2.5_boundary"
    )
    result = _run_model_scenario(
        simulator_udp,
        model,
        output_dir=output_dir,
        run_id=4702,
        physics_override=_outer_physics_override(model),
        duration_s=70.0,
        telemetry_stride=40,
        pid_config_path=pid_path,
        disturbances=[
            {"start_s": 8.0, "duration_s": 0.4, "force_n": 1.5},
            {"start_s": 28.0, "duration_s": 0.4, "force_n": -1.5},
        ],
    )
    _assert_common_integrity(
        result.summary,
        result.metadata,
        result.done,
        model=model,
        expected_physics_override=_outer_physics_override(model),
    )
    assert result.done.reason_code == DONE_COMPLETED


@pytest.mark.parametrize("model", _model_params())
def test_outer_drive_stop_and_reversal_are_symmetric(
    simulator_udp, sim_artifact_settings, model: SimulatorModel
):
    results = []
    diagnostics = ScenarioDiagnostics("outer_drive_stop_reversal", model.label)
    for index, sign in enumerate((1.0, -1.0)):
        output_dir = _artifact_dir(
            sim_artifact_settings,
            f"{model.key}/outer_live_reversal_{'forward' if sign > 0 else 'reverse'}",
        )
        subrun_id = "forward" if sign > 0 else "reverse"

        def checks_factory(summary, metadata, done, frame, sign=sign):
            # Preserve the physical speed envelope after the verified 1/32
            # migration: the same m/s now corresponds to twice the SPS.
            peak_sps = frame["plant_velocity_mps"].abs().max() / OUTER_METERS_PER_STEP
            requested_peak_sps = (
                float(frame["user_velocity_mps"].abs().max()) / OUTER_METERS_PER_STEP
            )
            drive_window = _outer_window_metrics(frame, 2.0, 14.0)
            reversal_window = _outer_window_metrics(frame, 25.0, 27.0)
            results.append((summary, frame))
            checks = [
                (
                    "bounded_balance",
                    lambda: _outer_assert_bounded(
                        summary, metadata, done, frame, model=model, max_pitch_deg=12.0
                    ),
                ),
                (
                    "drive_direction",
                    lambda: assert_true(
                        sign
                        * float(
                            frame.loc[
                                (frame["t_sec"] >= 2.0) & (frame["t_sec"] < 14.0),
                                "plant_velocity_mps",
                            ].mean()
                        )
                        / OUTER_METERS_PER_STEP
                        >= 20.0,
                        "drive did not move in the commanded direction",
                    ),
                ),
                (
                    "reversal_direction",
                    lambda: assert_true(
                        sign
                        * float(
                            frame.loc[
                                (frame["t_sec"] >= 25.0) & (frame["t_sec"] < 27.0),
                                "plant_velocity_mps",
                            ].mean()
                        )
                        / OUTER_METERS_PER_STEP
                        <= -20.0,
                        "reversal did not move in the opposite direction",
                    ),
                ),
                (
                    "drive_command_bound",
                    lambda: assert_true(
                        drive_window["command_rms_sps"] <= 16000.0,
                        "drive command RMS exceeds 16 kSPS",
                    ),
                ),
                (
                    "reversal_command_bound",
                    lambda: assert_true(
                        reversal_window["command_peak_sps"] <= 16000.0,
                        "reversal command exceeds 16 kSPS",
                    ),
                ),
                (
                    "late_velocity",
                    lambda: assert_true(
                        summary["tail_mean_abs_velocity_mps"]
                        <= (0.003 if model == DIRECT_ACTUATOR_MODEL else 0.005),
                        "drive/reversal retains excessive late velocity",
                    ),
                ),
                (
                    "outer_pitch_bound",
                    lambda: assert_true(
                        frame["drive_pitch_target_deg"].abs().max()
                        <= frame["active_outer_pitch_limit_deg"].abs().max() + 0.05,
                        "drive/reversal exceeded the shared outer pitch authority",
                    ),
                ),
            ]
            if model == STEPPER_PHASE_ELECTRICAL_MODEL:
                drive_reference = abs(float(
                    frame.loc[
                        (frame["t_sec"] >= 2.0) & (frame["t_sec"] < 14.0),
                        "reference_velocity_mps",
                    ].mean()
                ))
                drive_actual = sign * float(
                    frame.loc[
                        (frame["t_sec"] >= 2.0) & (frame["t_sec"] < 14.0),
                        "plant_velocity_mps",
                    ].mean()
                )
                reversal_reference = abs(float(
                    frame.loc[
                        (frame["t_sec"] >= 25.0) & (frame["t_sec"] < 27.0),
                        "reference_velocity_mps",
                    ].mean()
                ))
                reversal_actual = -sign * float(
                    frame.loc[
                        (frame["t_sec"] >= 25.0) & (frame["t_sec"] < 27.0),
                        "plant_velocity_mps",
                    ].mean()
                )
                checks.extend(
                    [
                        (
                            "drive_tracking",
                            lambda: assert_true(
                                drive_reference > 0.0
                                and drive_actual / drive_reference >= 0.75,
                                "drive did not track the requested velocity",
                            ),
                        ),
                        (
                            "reversal_tracking",
                            lambda: assert_true(
                                reversal_reference > 0.0
                                and reversal_actual / reversal_reference >= 0.70,
                                "reversal did not track the opposite requested velocity",
                            ),
                        ),
                        (
                            "physical_overspeed_bound",
                            lambda: assert_true(
                                peak_sps <= max(0.30 / OUTER_METERS_PER_STEP, 5.0 * requested_peak_sps),
                                "drive exceeded the physical overspeed guard",
                            ),
                        ),
                    ]
                )
            else:
                checks.extend(
                    [
                        (
                            "speed_envelope",
                            lambda: assert_true(
                                200.0 <= peak_sps <= requested_peak_sps * 1.50,
                                f"drive speed {peak_sps:.1f} SPS is outside the commanded envelope ending near {requested_peak_sps:.1f} SPS",
                            ),
                        ),
                        (
                            "position_bound",
                            lambda: assert_true(
                                summary["max_abs_position_m"] <= 0.60,
                                "DirectActuator drive/reversal position exceeds 0.60 m",
                            ),
                        ),
                    ]
                )
            return checks

        _run_outer_subrun(
            diagnostics,
            simulator_udp,
            model,
            output_dir,
            subrun_id,
            {
                "run_id": 4100 + index,
                "physics_override": _outer_physics_override(model),
                "duration_s": 70.0,
                "telemetry_stride": 40,
                "joy_segments": [
                    {"start_s": 2.0, "duration_s": 12.0, "forward": sign * 0.45},
                    {"start_s": 25.0, "duration_s": 2.0, "forward": -sign * 0.55},
                ],
            },
            checks_factory,
            scenario_id="outer_drive_stop_reversal",
            scenario_category="drive_stop",
            scenario_intent="drive, stop, and reverse in both robot-forward directions",
        )

    if len(results) == 2:
        positive, negative = results
        if abs(positive[0]["max_abs_pitch_deg"] - negative[0]["max_abs_pitch_deg"]) > 0.25:
            diagnostics.record_failure("aggregate_pitch_symmetry", "drive pitch envelope is asymmetric")
        if abs(positive[0]["max_abs_position_m"] - negative[0]["max_abs_position_m"]) > 0.05:
            diagnostics.record_failure("aggregate_position_symmetry", "drive position envelope is asymmetric")
    else:
        diagnostics.record_failure("aggregate_signed_pair", f"expected two drive directions, got {len(results)}")
    _finish_composite(diagnostics)


@pytest.mark.parametrize("model", _model_params())
def test_outer_com_acquisition_is_symmetric_over_useful_bias_range(
    simulator_udp, sim_artifact_settings, model: SimulatorModel
):
    pytest.skip(
        "adaptive COM acquisition/trust/trim is intentionally deferred from the golden controller surface"
    )
    final_trims = {}
    aggregate_failures: list[str] = []
    diagnostics = ScenarioDiagnostics("outer_com_acquisition", model.label)
    # The optional learner is exercised over the bounded, convergent region;
    # larger static biases remain a separate controller/authority experiment.
    for index, offset_rad in enumerate((0.002, 0.004, 0.008)):
        signed_trims = []
        for sign in (1.0, -1.0):
            output_dir = _artifact_dir(
                sim_artifact_settings,
                f"{model.key}/outer_live_com_{int(offset_rad * 1000)}_{'plus' if sign > 0 else 'minus'}",
            )
            subrun_id = f"offset_{offset_rad:g}_{'plus' if sign > 0 else 'minus'}"

            def checks_factory(summary, metadata, done, frame, offset_rad=offset_rad, sign=sign):
                trust_time_s = _outer_trust_time(frame)
                equilibrium = (
                    _outer_equilibrium_convergence(frame, trust_time_s)
                    if trust_time_s is not None
                    else None
                )
                expected_trim_deg = -sign * offset_rad * 180.0 / math.pi
                final_trim_deg = float(frame["com_trim_deg"].iloc[-1])
                signed_trims.append(final_trim_deg)
                return [
                    (
                        "bounded_balance",
                        lambda: _outer_assert_bounded(
                            summary, metadata, done, frame, model=model, max_pitch_deg=5.0
                        ),
                    ),
                    (
                        "com_trust_time",
                        lambda: assert_true(
                            trust_time_s is not None and trust_time_s <= 45.0,
                            f"COM trust time {trust_time_s!r} exceeds 45 s",
                        ),
                    ),
                    (
                        "equilibrium_rate",
                        lambda: assert_true(
                            equilibrium is not None
                            and equilibrium["estimate_rate_deg_per_s"] <= 0.05,
                            "COM equilibrium estimate rate is not bounded",
                        ),
                    ),
                    (
                        "equilibrium_span",
                        lambda: assert_true(
                            equilibrium is not None and equilibrium["estimate_span_deg"] <= 0.15,
                            "COM equilibrium estimate span is too large",
                        ),
                    ),
                    (
                        "equilibrium_gap",
                        lambda: assert_true(
                            equilibrium is not None
                            and equilibrium["candidate_estimate_gap_deg"] <= 0.08,
                            "COM candidate and estimate do not converge",
                        ),
                    ),
                    (
                        "trim_trusted",
                        lambda: assert_true(
                            bool(frame["trim_trusted"].iloc[-1]),
                            "COM trim is not trusted at the end of acquisition",
                        ),
                    ),
                    (
                        "trim_value",
                        lambda: assert_true(
                            abs(final_trim_deg - expected_trim_deg) <= 0.08,
                            f"final COM trim {final_trim_deg:.3f} differs from {expected_trim_deg:.3f}",
                        ),
                    ),
                    (
                        "position_bound",
                        lambda: assert_true(
                                summary["max_abs_position_m"] <= 0.15,
                                "COM acquisition position exceeds 0.15 m",
                        ),
                    ),
                    (
                        "late_velocity",
                        lambda: assert_true(
                            summary["tail_mean_abs_velocity_mps"] <= 0.003,
                            "COM acquisition retains excessive late velocity",
                        ),
                    ),
                ]

            _run_outer_subrun(
                diagnostics,
                simulator_udp,
                model,
                output_dir,
                subrun_id,
            {
                "run_id": 4200 + index * 10 + (1 if sign > 0 else 2),
                "pid_config_path": _adaptive_com_pid_variant(sim_artifact_settings, model),
                "physics_override": _outer_physics_override(model),
                "duration_s": 60.0,
                "telemetry_stride": 40,
                "com_angle_offset_rad": sign * offset_rad,
            },
                checks_factory,
                scenario_id="outer_com_acquisition",
                scenario_category="com_acquisition_and_maintenance",
                scenario_intent="acquire signed physical COM bias and settle without motion runaway",
            )
        final_trims[offset_rad] = signed_trims
        if len(signed_trims) == 2:
            if abs(signed_trims[0] + signed_trims[1]) > 0.02:
                aggregate_failures.append(f"COM trim asymmetry at offset {offset_rad:g} rad")
        else:
            aggregate_failures.append(
                f"COM offset {offset_rad:g} has {len(signed_trims)} signed trim observations"
            )
    for message in aggregate_failures:
        diagnostics.record_failure("aggregate_trim_symmetry", message)
    _finish_composite(diagnostics)


@pytest.mark.parametrize("model", _model_params())
def test_outer_com_acquisition_pauses_through_motion_and_maintenance_reacquires(
    simulator_udp, sim_artifact_settings, model: SimulatorModel
):
    pytest.skip(
        "adaptive COM acquisition/trust/trim is intentionally deferred from the golden controller surface"
    )
    diagnostics = ScenarioDiagnostics("outer_com_interruptions_maintenance", model.label)
    interrupted_dir = _artifact_dir(
        sim_artifact_settings, f"{model.key}/outer_live_com_interruptions"
    )
    interrupted_result = None

    def interrupted_checks(summary, metadata, done, frame):
        moving = frame[(frame["t_sec"] >= 0.5) & (frame["t_sec"] < 6.5)]
        authority = frame["outer_acceleration_limited"] > 0.5
        trust_time_s = _outer_trust_time(frame)
        return [
            (
                "bounded_balance",
                lambda: _outer_assert_bounded(
                    summary,
                    metadata,
                    done,
                    frame,
                    model=model,
                    max_pitch_deg=(15.0 if model == DIRECT_ACTUATOR_MODEL else 8.0),
                ),
            ),
            (
                "motion_window",
                lambda: assert_true(not moving.empty, "motion window is empty"),
            ),
            (
                "learning_paused_in_motion",
                lambda: assert_true(
                    not moving.empty and moving["trim_learning_enabled"].max() == 0,
                    "COM trim learned during commanded motion",
                ),
            ),
            (
                "learning_paused_in_authority",
                lambda: assert_true(
                    not authority.any()
                    or frame.loc[authority, "trim_learning_enabled"].max() == 0,
                    "COM trim learned during velocity authority limiting",
                ),
            ),
            (
                "trust_after_motion",
                lambda: assert_true(
                    trust_time_s is not None and trust_time_s > 6.5,
                    f"COM trust time {trust_time_s!r} did not remain after motion",
                ),
            ),
            (
                "learning_resumes",
                lambda: assert_true(
                    frame["trim_learning_enabled"].iloc[-1] > 0.5,
                    "COM trim learning did not resume",
                ),
            ),
            (
                "late_velocity",
                lambda: assert_true(
                    summary["tail_mean_abs_velocity_mps"]
                    <= (0.01 if model == DIRECT_ACTUATOR_MODEL else 0.003),
                    "interrupted COM acquisition retains excessive late velocity",
                ),
            ),
        ]

    interrupted_result = _run_outer_subrun(
        diagnostics,
        simulator_udp,
        model,
        interrupted_dir,
        "interruptions",
        {
            "run_id": 4301,
            "pid_config_path": _adaptive_com_pid_variant(sim_artifact_settings, model),
            "physics_override": _outer_physics_override(model),
            "duration_s": 100.0,
            "telemetry_stride": 40,
            "com_angle_offset_rad": 0.004,
            "joy_segments": [{"start_s": 0.5, "duration_s": 6.0, "forward": 0.35}],
            "disturbances": [
                {"start_s": 14.0, "duration_s": 0.4, "force_n": 1.0},
                {"start_s": 28.0, "duration_s": 0.4, "force_n": -1.0},
            ],
        },
        interrupted_checks,
        scenario_id="outer_com_interruptions_maintenance",
        scenario_category="com_acquisition_and_maintenance",
        scenario_intent="pause COM learning during motion and resume after motion settles",
    )

    maintenance_dir = _artifact_dir(
        sim_artifact_settings, f"{model.key}/outer_live_com_maintenance"
    )
    def maintenance_checks(summary, metadata, done, frame):
        expected_trim_deg = -(0.004 + 0.003) * 180.0 / math.pi
        final_trim_deg = float(frame["com_trim_deg"].iloc[-1])
        return [
            (
                "bounded_balance",
                lambda: _outer_assert_bounded(
                    summary, metadata, done, frame, model=model, max_pitch_deg=3.0
                ),
            ),
            (
                "trim_value",
                lambda: assert_true(
                    abs(final_trim_deg - expected_trim_deg) <= 0.10,
                    f"maintenance trim {final_trim_deg:.3f} differs from {expected_trim_deg:.3f}",
                ),
            ),
            (
                "trim_trusted",
                lambda: assert_true(
                    bool(frame["trim_trusted"].iloc[-1]),
                    "maintenance run did not retain COM trust",
                ),
            ),
            (
                "late_velocity",
                lambda: assert_true(
                    summary["tail_mean_abs_velocity_mps"] <= 0.003,
                    "maintenance run retains excessive late velocity",
                ),
            ),
            (
                "trim_slope",
                lambda: assert_true(
                    abs(_outer_late_slope(frame, "com_trim_deg", 90.0)) <= 0.01,
                    "maintenance COM trim continues to drift",
                ),
            ),
        ]

    _run_outer_subrun(
        diagnostics,
        simulator_udp,
        model,
        maintenance_dir,
        "maintenance",
        {
            "run_id": 4302,
            "pid_config_path": _adaptive_com_pid_variant(sim_artifact_settings, model),
            "physics_override": _outer_physics_override(model),
            "duration_s": 140.0,
            "telemetry_stride": 40,
            "com_angle_offset_rad": 0.004,
            "disturbances": [
                {
                    "kind": "hold_bias",
                    "start_s": 30.0,
                    "duration_s": 0.0,
                    "com_bias_rad": 0.003,
                }
            ],
        },
        maintenance_checks,
        scenario_id="outer_com_interruptions_maintenance",
        scenario_category="com_acquisition_and_maintenance",
        scenario_intent="retain COM trust and reacquire a changed physical bias during maintenance",
    )
    _finish_composite(diagnostics)


@pytest.mark.parametrize("model", _model_params())
def test_outer_reduced_translation_authority_degrades_without_trim_runaway(
    simulator_udp, sim_artifact_settings, model: SimulatorModel
):
    if model == STEPPER_PHASE_ELECTRICAL_MODEL:
        pytest.skip(
            "generic traction/force override is not a calibrated StepperPhaseElectrical authority contract"
        )
    diagnostics = ScenarioDiagnostics("outer_reduced_translation_authority", model.label)
    for index, fraction in enumerate((1.0, 0.8, 0.6, 0.4)):
        output_dir = _artifact_dir(
            sim_artifact_settings,
            f"{model.key}/outer_live_authority_{int(fraction * 100)}",
        )
        override = {
            **_outer_physics_override(model),
            "traction_coefficient": fraction,
            "motor_max_force_n": 22.5 * fraction,
        }
        subrun_id = f"authority_{fraction:g}"

        def checks_factory(summary, metadata, done, frame, fraction=fraction, override=override):
            expected_traction_limit_n = 1.032 * 9.81 * fraction
            return [
                (
                    "bounded_balance",
                    lambda: _outer_assert_bounded(
                        summary,
                        metadata,
                        done,
                        frame,
                        model=model,
                        expected_physics_override=override,
                        max_pitch_deg=15.0,
                    ),
                ),
                (
                    "traction_limit",
                    lambda: assert_true(
                        abs(frame["traction_limit_n"].max() - expected_traction_limit_n) <= 0.05,
                        "reported traction limit does not match the authority override",
                    ),
                ),
                (
                    "com_trim_bound",
                    lambda: assert_true(
                        frame["com_trim_deg"].abs().max() < 0.6,
                        "reduced authority causes COM trim runaway",
                    ),
                ),
                (
                    "late_velocity",
                    lambda: assert_true(
                        summary["tail_mean_abs_velocity_mps"]
                        <= (0.01 if model == DIRECT_ACTUATOR_MODEL else 0.003),
                        "reduced authority retains excessive late velocity",
                    ),
                ),
            ]

        _run_outer_subrun(
            diagnostics,
            simulator_udp,
            model,
            output_dir,
            subrun_id,
            {
                "run_id": 4400 + index,
                "physics_override": override,
                "duration_s": 60.0,
                "telemetry_stride": 40,
                "com_angle_offset_rad": 0.004,
                "joy_segments": [{"start_s": 2.0, "duration_s": 5.0, "forward": 0.5}],
            },
            checks_factory,
            scenario_id="outer_reduced_translation_authority",
            scenario_category="authority_and_saturation",
            scenario_intent="degrade gracefully as translation authority is reduced without trim runaway",
        )
    _finish_composite(diagnostics)


@pytest.mark.parametrize("model", _model_params())
def test_outer_noise_and_correlated_mass_uncertainty_remain_bounded(
    simulator_udp, sim_artifact_settings, model: SimulatorModel
):
    pytest.skip(
        "this legacy uncertainty matrix enables adaptive COM and is deferred with COM behavior"
    )
    diagnostics = ScenarioDiagnostics("outer_noise_mass_uncertainty", model.label)
    for index, scale in enumerate((0.85, 1.15)):
        output_dir = _artifact_dir(
            sim_artifact_settings, f"{model.key}/outer_live_mass_{int(scale * 100)}"
        )
        subrun_id = f"mass_scale_{scale:g}"

        def checks_factory(summary, metadata, done, frame, scale=scale):
            return [
                (
                    "bounded_balance",
                    lambda: _outer_assert_bounded(
                        summary,
                        metadata,
                        done,
                        frame,
                        model=model,
                        expected_total_mass_scale=scale,
                        expected_pitch_inertia_scale=scale,
                        max_pitch_deg=(15.0 if model == DIRECT_ACTUATOR_MODEL else 10.0),
                    ),
                ),
                (
                    "trim_trusted",
                    lambda: assert_true(
                        bool(frame["trim_trusted"].iloc[-1]),
                        "mass uncertainty run did not retain COM trust",
                    ),
                ),
                (
                    "late_velocity",
                    lambda: assert_true(
                        summary["tail_mean_abs_velocity_mps"]
                        <= (0.01 if model == DIRECT_ACTUATOR_MODEL else 0.003),
                        "mass uncertainty retains excessive late velocity",
                    ),
                ),
            ]

        _run_outer_subrun(
            diagnostics,
            simulator_udp,
            model,
            output_dir,
            subrun_id,
            {
                "run_id": 4500 + index,
                "pid_config_path": _adaptive_com_pid_variant(sim_artifact_settings, model),
                "physics_override": _outer_physics_override(model),
                "duration_s": 90.0,
                "telemetry_stride": 40,
                "com_angle_offset_rad": 0.004,
                "total_mass_scale": scale,
                "pitch_inertia_scale": scale,
                "disturbances": [{"start_s": 30.0, "duration_s": 0.4, "force_n": 1.0}],
            },
            checks_factory,
            scenario_id="outer_noise_mass_uncertainty",
            scenario_category="uncertainty",
            scenario_intent="remain bounded under correlated body mass and pitch-inertia uncertainty",
        )

    output_dir = _artifact_dir(
        sim_artifact_settings, f"{model.key}/outer_live_noise_long"
    )
    def noise_checks(summary, metadata, done, frame):
        return [
            (
                "bounded_balance",
                lambda: _outer_assert_bounded(
                    summary, metadata, done, frame, model=model, max_pitch_deg=5.0
                ),
            ),
            (
                "trim_trusted",
                lambda: assert_true(
                    bool(frame["trim_trusted"].iloc[-1]),
                    "long noise run did not retain COM trust",
                ),
            ),
            (
                "com_trim_bound",
                lambda: assert_true(
                    frame["com_trim_deg"].abs().max() < 0.6,
                    "long noise run causes COM trim runaway",
                ),
            ),
            (
                "late_velocity",
                lambda: assert_true(
                    summary["tail_mean_abs_velocity_mps"] <= 0.003,
                    "long noise run retains excessive late velocity",
                ),
            ),
        ]

    _run_outer_subrun(
        diagnostics,
        simulator_udp,
        model,
        output_dir,
        "long_noise",
        {
            "run_id": 4520,
            "pid_config_path": _adaptive_com_pid_variant(sim_artifact_settings, model),
            "physics_override": _outer_physics_override(model),
            "duration_s": 120.0,
            "telemetry_stride": 40,
            "com_angle_offset_rad": 0.004,
            "imu_noise_seed": 20260808,
            "accel_noise_std_mps2": 0.12,
            "gyro_noise_std_rad_s": 0.006,
            "imu_timestamp_jitter_us": 200.0,
            "imu_sample_loss_rate": 0.001,
        },
        noise_checks,
        scenario_id="outer_noise_mass_uncertainty",
        scenario_category="uncertainty",
        scenario_intent="remain bounded during a long noisy estimator run",
    )
    _finish_composite(diagnostics)


@pytest.mark.parametrize("model", _model_params())
def test_outer_ten_minute_event_run_has_no_growing_late_envelope(
    simulator_udp, sim_artifact_settings, model: SimulatorModel
):
    output_dir = _artifact_dir(
        sim_artifact_settings, f"{model.key}/outer_live_long_events_600s"
    )
    result = _run_model_scenario(
        simulator_udp,
        model,
        run_id=4600,
        output_dir=output_dir,
        physics_override=_outer_physics_override(model),
        duration_s=600.0,
        telemetry_stride=80,
        # Keep the long-horizon qualification run on the nominal fixed-trim
        # surface. Adaptive COM acquisition and maintenance are explicit SKIP
        # scenarios while COM is deferred.
        com_angle_offset_rad=0.0,
        joy_segments=[
            {"start_s": 2.0, "duration_s": 12.0, "forward": 0.35},
            {"start_s": 60.0, "duration_s": 12.0, "forward": -0.35},
            {"start_s": 150.0, "duration_s": 15.0, "forward": 0.45},
            {"start_s": 220.0, "duration_s": 15.0, "forward": -0.45},
        ],
        disturbances=[
            {"start_s": 35.0, "duration_s": 0.4, "force_n": 1.5},
            {"start_s": 100.0, "duration_s": 0.4, "force_n": -1.5},
            {"start_s": 260.0, "duration_s": 0.4, "force_n": 1.5},
            {"start_s": 400.0, "duration_s": 0.4, "force_n": -1.5},
            {"start_s": 520.0, "duration_s": 0.4, "force_n": 1.5},
        ],
    )
    summary, metadata, done, frame = (
        result.summary,
        result.metadata,
        result.done,
        result.frame,
    )
    _outer_assert_bounded(
        summary,
        metadata,
        done,
        frame,
        model=model,
        max_pitch_deg=(20.0 if model == DIRECT_ACTUATOR_MODEL else 15.0),
        # The run contains disturbances at 400 s and 520 s. A whole-run
        # early/middle/late split treats the final disturbance as growth;
        # the explicit 575 s tail checks below are the correct post-event
        # stability witness for this scenario.
        check_growing_oscillation=False,
    )
    late = frame[frame["t_sec"] >= 575.0]
    assert len(late) >= 100
    late_velocity_rms = math.sqrt(float((late["plant_velocity_mps"] ** 2).mean()))
    late_pitch_rms = math.sqrt(float((late["plant_pitch_deg"] ** 2).mean()))
    assert late_velocity_rms <= (0.01 if model == DIRECT_ACTUATOR_MODEL else 0.003)
    assert late_pitch_rms <= (0.75 if model == DIRECT_ACTUATOR_MODEL else 0.5)
    assert abs(_outer_late_slope(frame, "com_trim_deg", 575.0)) <= 0.01
    assert frame["drive_pitch_target_deg"].abs().max() <= frame["active_outer_pitch_limit_deg"].abs().max() + 0.05
    # The full run intentionally contains commanded translation and five
    # disturbances. Evaluate the low-frequency residual on the post-event
    # qualification tail rather than folding commanded motion into a neutral
    # balance metric.
    slow_velocity = band_rms_equivalent(late, "plant_velocity_mps", 0.1, 0.3)
    assert slow_velocity["sample_rate_hz"] is not None
    assert slow_velocity["rms"] is not None and slow_velocity["rms"] <= (
        0.006 if model == DIRECT_ACTUATOR_MODEL else 0.005
    )


COLD_START_DIAGNOSTICS = [
    pytest.param(
        3500,
        "cold_start_50deg_estimator_limited",
        50.0,
    ),
]


@pytest.mark.parametrize(
    ("run_id", "name", "initial_pitch_deg"),
    COLD_START_DIAGNOSTICS,
)
@pytest.mark.parametrize("model", _model_params())
def test_cold_start_50deg_estimator_limited_is_tracked_against_70_degree_boundary(
    simulator_udp,
    sim_artifact_settings,
    run_id: int,
    name: str,
    initial_pitch_deg: float,
    model: SimulatorModel,
):
    output_dir = _model_artifact_dir(sim_artifact_settings, model, name)
    pid_path = _attitude_only_pid_variant(sim_artifact_settings, model)
    result = _run_model_scenario(
        simulator_udp,
        model,
        run_id=run_id,
        output_dir=output_dir,
        initial_pitch_deg=initial_pitch_deg,
        duration_s=20.0,
        telemetry_stride=1,
        pid_config_path=pid_path,
        fail_fast_pitch_deg=70.0,
    )
    summary, metadata, done = result.summary, result.metadata, result.done
    _assert_common_integrity(summary, metadata, done, model=model)
    assert done.reason_code == DONE_COMPLETED
    assert done.actuator_fault_count == 0
    assert done.controller_fault_flags == 0
    assert done.max_abs_pitch_deg <= 70.0
    assert not summary["fell"]
    assert summary["tail_rms_pitch_deg"] <= 1.0
