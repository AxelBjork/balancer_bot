from __future__ import annotations

import math
from pathlib import Path

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
        # This is a UDP/downsampling smoke test, not the disabled legacy
        # closed-loop transfer acceptance gate. Keep the perturbation small.
        disturbances=[{"start_s": 0.5, "duration_s": 0.1, "force_n": 0.01}],
    )

    assert metadata["telemetry_stride"] == 20
    assert done.reason_code == DONE_COMPLETED
    assert done.sample_count == 800
    assert done.actuator_fault_count == 0
    assert done.controller_fault_flags == 0
    assert summary["max_abs_pitch_deg"] < 15.0
    # 800 controller ticks produce only 40 streamed rows at stride 20.
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
            "motor_max_force_n": 10.5,
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
        "motor_max_force_n": 10.5,
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
        assert math.isclose(float(summary[field]), expected, rel_tol=0.0, abs_tol=1e-12)
