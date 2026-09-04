from __future__ import annotations

import math

import numpy as np
import pandas as pd

from tools.analyze_recovery import analyze_recovery


def test_recovery_analyzer_derives_torque_and_recovery_events() -> None:
    time = np.arange(0.0, 4.1, 0.1)
    pitch = np.where(time < 1.0, 40.0, np.where(time < 1.8, 40.0 - 48.0 * (time - 1.0), 1.0))
    contact = (time < 1.1).astype(float)
    rate = np.where(time < 1.8, -10.0, 0.0)
    velocity = np.where(time < 1.8, 0.05, 0.0)
    applied = np.where(time < 2.0, 5000.0, 0.0)
    frame = pd.DataFrame(
        {
            "t_sec": time,
            "plant_pitch_deg": pitch,
            "plant_pitch_rate_dps": rate,
            "fused_pitch_deg": pitch,
            "plant_velocity_mps": velocity,
            "f_cmd": np.full_like(time, 5.0),
            "f_app": np.full_like(time, 2.0),
            "traction_limit_n": np.full_like(time, 4.0),
            "phase_error_steps": np.full_like(time, 3.0),
            "brace_contact_active": contact,
            "recovery_command_active": (time >= 1.0).astype(float),
            "balance_unclamped_sps": applied,
            "u_sps": applied,
            "left_slewed_sps": applied,
            "right_slewed_sps": applied,
            "emitted_step_velocity_sps": applied,
            "actual_wheel_velocity": velocity,
            "controller_fault_flags": np.zeros_like(time),
            "actuator_fault": np.zeros_like(time),
            "brace_pitch_deg": np.full_like(time, 40.0),
        }
    )
    constants = {
        "gravity": 9.81,
        "first_mass_moment_kg_m": 0.06192,
        "stepper_current_limit_a": 1.065,
        "motor_stall_torque_nm": 0.45,
        "motor_rated_phase_current_a": 1.5,
        "stepper_sqrt_two": math.sqrt(2.0),
        "bus_voltage_v": 11.1,
        "motor_slew_sps_per_s": 200000.0,
        "nominal_balance_max_sps": 32000.0,
        "fallover_shutdown_deg": 40.0,
    }
    summary = analyze_recovery(
        frame,
        brace_angle_deg=None,
        constants=constants,
        pid_values={"balance_max_sps": 32000.0},
    )

    assert summary["config"]["brace_angle_deg"] == 40.0
    assert math.isclose(
        summary["static_authority"]["gravity_torque_total_nm"],
        9.81 * 0.06192 * math.sin(math.radians(40.0)),
    )
    assert summary["events"]["recovery_controller_enable"]["time_s"] == 1.0
    assert summary["events"]["brace_release"]["time_s"] == 1.1
    assert summary["events"]["crossed_upright_1_deg"]["time_s"] == 1.8
    assert math.isclose(
        summary["electrical_phase"]["peak_useful_inward_torque_total_nm"],
        5.0 * 0.0412,
    )
    assert summary["outcome"]["stabilized"]
