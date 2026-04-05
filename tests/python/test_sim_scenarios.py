from __future__ import annotations

import math
from pathlib import Path

import pytest

from generated_balancer import (
    BalancerMsgId,
    ImuSamplePayload,
    JoystickCommandPayload,
    MotorTargetsPayload,
    PhysicsTickPayload,
    SystemTelemetryPayload,
)
from tools.run_artifacts import RunRecorder, preserve_artifacts


class SimpleBalancerPlant:
    gravity = 9.81
    wheel_radius = 0.080 / 2.0
    robot_mass = 1.032
    wheel_mass = 0.050
    cart_mass = 2.0 * wheel_mass
    body_mass = robot_mass - cart_mass
    center_of_mass_height = 0.06
    i_com = 0.0034
    steps_per_rev = 200.0 * 16.0

    def __init__(self, initial_pitch_deg: float, com_angle_offset_rad: float):
        self.pitch = math.radians(initial_pitch_deg)
        self.pitch_rate = 0.0
        self.position = 0.0
        self.velocity = 0.0
        self.com_angle_offset_rad = com_angle_offset_rad
        self.last_x_ddot = 0.0
        self.left_target_sps = 0.0
        self.right_target_sps = 0.0

    def set_targets(self, left_sps: float, right_sps: float) -> None:
        self.left_target_sps = left_sps
        self.right_target_sps = right_sps

    def step(self, dt_s: float) -> None:
        avg_steps_per_sec = 0.5 * (self.left_target_sps + self.right_target_sps)
        target_wheel_velocity = (
            (avg_steps_per_sec / self.steps_per_rev) * 2.0 * math.pi * self.wheel_radius
        )

        v_err = target_wheel_velocity - self.velocity
        f_cmd = 500.0 * v_err
        f_app = max(-20.0, min(20.0, f_cmd))

        q = self.pitch + self.com_angle_offset_rad
        q_dot = self.pitch_rate
        sq = math.sin(q)
        cq = math.cos(q)

        d11 = self.cart_mass + self.body_mass
        d12 = self.body_mass * self.center_of_mass_height * cq
        d21 = d12
        d22 = self.i_com + self.body_mass * self.center_of_mass_height**2

        rhs1 = f_app + self.body_mass * self.center_of_mass_height * q_dot * q_dot * sq - 2.0 * self.velocity
        rhs2 = self.body_mass * self.gravity * self.center_of_mass_height * sq - 0.05 * self.pitch_rate

        det = d11 * d22 - d12 * d21
        x_ddot = (d22 * rhs1 - d12 * rhs2) / det
        theta_ddot = (d11 * rhs2 - d21 * rhs1) / det

        self.velocity += x_ddot * dt_s
        self.position += self.velocity * dt_s
        self.pitch_rate += theta_ddot * dt_s
        self.pitch += self.pitch_rate * dt_s
        self.last_x_ddot = x_ddot

    def imu_payload(self, sim_time_us: int) -> ImuSamplePayload:
        q = self.pitch + self.com_angle_offset_rad
        ax_mps2 = self.last_x_ddot * math.cos(q) + self.gravity * math.sin(q)
        az_mps2 = -self.last_x_ddot * math.sin(q) + self.gravity * math.cos(q)
        return ImuSamplePayload(
            pitch_rad=self.pitch,
            acc=[-ax_mps2, 0.0, -az_mps2],
            gyr=[0.0, self.pitch_rate, 0.0],
            timestamp_us=sim_time_us,
        )


def _drain_messages(udp, plant: SimpleBalancerPlant) -> SystemTelemetryPayload | None:
    telemetry = None
    while True:
        try:
            msg_id, payload_bytes = udp.recv(timeout=0.001 if telemetry is None else 0.0)
        except (TimeoutError, BlockingIOError):
            return telemetry

        if msg_id == int(BalancerMsgId.MotorTargets):
            targets = MotorTargetsPayload.unpack(payload_bytes)
            plant.set_targets(targets.left_sps, targets.right_sps)
        elif msg_id == int(BalancerMsgId.SystemTelemetry):
            telemetry = SystemTelemetryPayload.unpack(payload_bytes)


def _artifact_dir(sim_artifact_settings, run_id: str) -> Path:
    output_dir = Path(sim_artifact_settings["temp_root"]) / run_id
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir


@pytest.mark.parametrize(
    ("run_id", "initial_pitch_deg", "com_angle_offset_rad", "expect_upright"),
    [
        ("neutral_hold", 0.0, 0.0, True),
        ("small_pitch_bias", 0.05, 0.0, False),
        ("small_com_offset", 0.0, 0.001, False),
    ],
)
def test_tick_driven_simulation_scenarios(
    fresh_udp,
    sim_artifact_settings,
    run_id: str,
    initial_pitch_deg: float,
    com_angle_offset_rad: float,
    expect_upright: bool,
):
    plant = SimpleBalancerPlant(initial_pitch_deg, com_angle_offset_rad)
    recorder = RunRecorder()
    recorder.begin_run(
        {
            "run_id": run_id,
            "initial_pitch_deg": initial_pitch_deg,
            "com_angle_offset_rad": com_angle_offset_rad,
            "source": "pytest_sim",
        }
    )

    fresh_udp.send(BalancerMsgId.JoystickCommand, JoystickCommandPayload(forward=0.0, turn=0.0).pack())

    dt_s = 1.0 / 400.0
    sim_time_us = 0
    for _ in range(500):
        sim_time_us += int(dt_s * 1e6)
        fresh_udp.send(BalancerMsgId.ImuData, plant.imu_payload(sim_time_us).pack())
        fresh_udp.send(BalancerMsgId.PhysicsTick, PhysicsTickPayload(dt_s=dt_s, sim_time_us=sim_time_us).pack())
        telemetry = _drain_messages(fresh_udp, plant)
        plant.step(dt_s)

        row = {
            "sim_time_s": sim_time_us / 1e6,
            "left_sps": plant.left_target_sps,
            "right_sps": plant.right_target_sps,
            "plant_pitch_deg": math.degrees(plant.pitch),
            "plant_pitch_rate_dps": math.degrees(plant.pitch_rate),
            "plant_position": plant.position,
            "plant_velocity": plant.velocity,
        }
        if telemetry is not None:
            row.update(
                {
                    "pitch_deg": telemetry.pitch_deg,
                    "pitch_rate_dps": telemetry.pitch_rate_dps,
                    "pitch_sp_deg": telemetry.pitch_sp_deg,
                    "rate_sp_dps": telemetry.rate_sp_dps,
                    "u_sps": telemetry.u_sps,
                    "vel_error": telemetry.vel_error,
                    "vel_i_term": telemetry.vel_i_term,
                    "vel_p_term": telemetry.vel_p_term,
                    "out_norm": telemetry.out_norm,
                }
            )
        recorder.record_step(row)

    output_dir = _artifact_dir(sim_artifact_settings, run_id)
    summary = recorder.write_csv_json_plots(output_dir)
    preserve_artifacts(output_dir, sim_artifact_settings["preserve_root"], run_id)

    assert summary["sample_count"] > 0
    assert summary["telemetry_continuous"]
    assert summary["max_abs_pitch_deg"] is not None
    assert summary["max_abs_pitch_deg"] == summary["max_abs_pitch_deg"]
    assert (summary["fell"] is False) == expect_upright
    if expect_upright:
        assert summary["max_abs_pitch_deg"] <= 75.0
    else:
        assert summary["fell"]
