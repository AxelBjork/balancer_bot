from __future__ import annotations

import json
import time
from pathlib import Path

from generated_balancer import (
    BalancerMsgId,
    SimRunDonePayload,
    SimStartAckPayload,
    SimStartRunPayload,
    SimStopRunPayload,
    SystemTelemetryPayload,
)

from tests.python.support.run_artifacts import RunRecorder

PHYSICS_SIMPLIFIED = 0
PHYSICS_REALISTIC = 1

ACK_ACCEPTED = 0
ACK_BUSY = 1
ACK_INVALID = 2

DONE_COMPLETED = 0
DONE_STOPPED_BY_CLIENT = 1
DONE_FELL = 2
DONE_INTERNAL_ERROR = 3

DISTURBANCE_STEP = 0
DISTURBANCE_RAMP = 1
DISTURBANCE_HOLD_BIAS = 2


def _fixed_bytes(value: str, size: int) -> bytes:
    raw = value.encode("utf-8")
    if len(raw) >= size:
        raw = raw[: size - 1]
    return raw + (b"\x00" * (size - len(raw)))


def _disturbance_kind(value: str | int | None) -> int:
    if value is None:
        return DISTURBANCE_STEP
    if isinstance(value, int):
        return value
    mapping = {
        "step": DISTURBANCE_STEP,
        "ramp": DISTURBANCE_RAMP,
        "hold_bias": DISTURBANCE_HOLD_BIAS,
    }
    try:
        return mapping[value]
    except KeyError as exc:
        raise ValueError(f"Unsupported disturbance kind: {value}") from exc


def make_start_payload(
    *,
    run_id: int,
    physics_profile: int,
    duration_s: float,
    initial_pitch_deg: float = 0.0,
    com_angle_offset_rad: float = 0.0,
    wheel_slip_factor: float = 1.0,
    velocity_feedback_scale: float = 1.0,
    velocity_feedback_tau_s: float = 0.0,
    imu_pitch_lag_s: float = 0.0,
    disturbances: list[dict] | None = None,
    pid_config_path: str = "",
) -> SimStartRunPayload:
    disturbances = list(disturbances or [])
    if len(disturbances) > 10:
        raise ValueError("SimStartRunPayload supports at most 10 disturbance segments")

    wire_disturbances = [
        {
            "kind": DISTURBANCE_STEP,
            "reserved0": 0,
            "reserved1": 0,
            "start_s": 0.0,
            "duration_s": 0.0,
            "force_n": 0.0,
            "com_bias_rad": 0.0,
            "force_n_end": 0.0,
            "com_bias_rad_end": 0.0,
        }
        for _ in range(10)
    ]
    for idx, disturbance in enumerate(disturbances):
        force_n = float(disturbance.get("force_n", 0.0))
        com_bias_rad = float(disturbance.get("com_bias_rad", 0.0))
        wire_disturbances[idx] = {
            "kind": _disturbance_kind(disturbance.get("kind")),
            "reserved0": 0,
            "reserved1": 0,
            "start_s": float(disturbance.get("start_s", 0.0)),
            "duration_s": float(disturbance.get("duration_s", 0.0)),
            "force_n": force_n,
            "com_bias_rad": com_bias_rad,
            "force_n_end": float(disturbance.get("force_n_end", force_n)),
            "com_bias_rad_end": float(disturbance.get("com_bias_rad_end", com_bias_rad)),
        }

    return SimStartRunPayload(
        run_id=run_id,
        physics_profile=physics_profile,
        reserved0=0,
        reserved1=0,
        duration_s=duration_s,
        initial_pitch_deg=initial_pitch_deg,
        com_angle_offset_rad=com_angle_offset_rad,
        wheel_slip_factor=wheel_slip_factor,
        velocity_feedback_scale=velocity_feedback_scale,
        velocity_feedback_tau_s=velocity_feedback_tau_s,
        imu_pitch_lag_s=imu_pitch_lag_s,
        disturbances=wire_disturbances,
        pid_config_path=_fixed_bytes(pid_config_path, 128),
    )


def wait_for_ack(udp, run_id: int, timeout: float = 1.0) -> SimStartAckPayload:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            msg_id, payload = udp.recv(timeout=min(0.1, max(0.01, deadline - time.monotonic())))
        except TimeoutError:
            continue
        if msg_id != int(BalancerMsgId.SimStartAck):
            continue
        ack = SimStartAckPayload.unpack(payload)
        if ack.run_id == run_id:
            return ack
    raise AssertionError(f"Timed out waiting for SimStartAck for run_id={run_id}")


def wait_for_done(udp, run_id: int, timeout: float = 2.0) -> SimRunDonePayload:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            msg_id, payload = udp.recv(timeout=min(0.1, max(0.01, deadline - time.monotonic())))
        except TimeoutError:
            continue
        if msg_id != int(BalancerMsgId.SimRunDone):
            continue
        done = SimRunDonePayload.unpack(payload)
        if done.run_id == run_id:
            return done
    raise AssertionError(f"Timed out waiting for SimRunDone for run_id={run_id}")


def run_scenario_live(
    udp,
    *,
    run_id: int,
    output_dir: Path,
    physics_profile: int,
    duration_s: float,
    initial_pitch_deg: float = 0.0,
    com_angle_offset_rad: float = 0.0,
    wheel_slip_factor: float = 1.0,
    velocity_feedback_scale: float = 1.0,
    velocity_feedback_tau_s: float = 0.0,
    imu_pitch_lag_s: float = 0.0,
    disturbances: list[dict] | None = None,
    pid_config_path: str = "",
    fail_fast_pitch_deg: float = 75.0,
    done_timeout: float = 15.0,
) -> tuple[dict, dict, SimRunDonePayload]:
    output_dir.mkdir(parents=True, exist_ok=True)
    recorder = RunRecorder()

    metadata = {
        "run_id": str(run_id),
        "scenario_name": output_dir.name,
        "physics_profile": "realistic" if physics_profile == PHYSICS_REALISTIC else "simplified",
        "pid_profile": pid_config_path or "pid_sim.conf",
        "duration_s": duration_s,
        "initial_pitch_deg": initial_pitch_deg,
        "com_angle_offset_rad": com_angle_offset_rad,
        "wheel_slip_factor": wheel_slip_factor,
        "velocity_feedback_scale": velocity_feedback_scale,
        "velocity_feedback_tau_s": velocity_feedback_tau_s,
        "imu_pitch_lag_s": imu_pitch_lag_s,
    }
    if disturbances:
        metadata["disturbances"] = disturbances
    recorder.begin_run(metadata)

    udp.drain()
    start = make_start_payload(
        run_id=run_id,
        physics_profile=physics_profile,
        duration_s=duration_s,
        initial_pitch_deg=initial_pitch_deg,
        com_angle_offset_rad=com_angle_offset_rad,
        wheel_slip_factor=wheel_slip_factor,
        velocity_feedback_scale=velocity_feedback_scale,
        velocity_feedback_tau_s=velocity_feedback_tau_s,
        imu_pitch_lag_s=imu_pitch_lag_s,
        disturbances=disturbances,
        pid_config_path=pid_config_path,
    )
    udp.send(BalancerMsgId.SimStartRun, start.pack())
    ack = wait_for_ack(udp, run_id)
    if not ack.accepted:
        raise AssertionError(f"Simulator rejected run_id={run_id} with status={ack.status_code}")

    done = None
    deadline = time.monotonic() + done_timeout
    while time.monotonic() < deadline:
        try:
            msg_id, payload = udp.recv(timeout=min(0.1, max(0.01, deadline - time.monotonic())))
        except TimeoutError:
            continue
        if msg_id == int(BalancerMsgId.SystemTelemetry):
            telemetry = SystemTelemetryPayload.unpack(payload)
            if telemetry.run_id != run_id:
                continue
            row = {
                "sim_time_s": telemetry.sim_time_s,
                "pitch_deg": telemetry.pitch_deg,
                "pitch_rate_dps": telemetry.pitch_rate_dps,
                "filtered_pitch_rate_dps": telemetry.filtered_pitch_rate_dps,
                "raw_acc_pitch_deg": telemetry.raw_acc_pitch_deg,
                "fused_pitch_deg": telemetry.fused_pitch_deg,
                "gyro_pitch_rate_dps": telemetry.gyro_pitch_rate_dps,
                "pitch_sp_deg": telemetry.pitch_sp_deg,
                "rate_sp_dps": telemetry.rate_sp_dps,
                "u_sps": telemetry.u_sps,
                "left_sps": telemetry.u_sps,
                "right_sps": telemetry.u_sps,
                "vel_error": telemetry.vel_error,
                "vel_i_term": telemetry.vel_i_term,
                "vel_p_term": telemetry.vel_p_term,
                "target_vel_sps": telemetry.target_vel_sps,
                "measured_vel_sps": telemetry.measured_vel_sps,
                "filtered_vel_sps": telemetry.filtered_vel_sps,
                "position_target_vel_sps": telemetry.position_target_vel_sps,
                "pitch_ref_from_vel_deg": telemetry.pitch_ref_from_vel_deg,
                "pitch_ref_from_pos_deg": telemetry.pitch_ref_from_pos_deg,
                "pitch_error_deg": telemetry.pitch_error_deg,
                "rate_error_dps": telemetry.rate_error_dps,
                "out_norm": telemetry.out_norm,
                "pitch_sp_deg": telemetry.pitch_sp_deg,
                "effective_pitch_sp_deg": telemetry.effective_pitch_sp_deg,
                "pitch_trim_deg": telemetry.pitch_trim_deg,
                "trim_active": telemetry.trim_active,
                "left_applied_sps": telemetry.left_applied_sps,
                "right_applied_sps": telemetry.right_applied_sps,
                "left_actual_steps": telemetry.left_actual_steps,
                "right_actual_steps": telemetry.right_actual_steps,
                "plant_pitch_deg": telemetry.plant_pitch_deg,
                "plant_pitch_rate_dps": telemetry.plant_pitch_rate_dps,
                "plant_position": telemetry.plant_position_m,
                "plant_velocity": telemetry.plant_velocity_mps,
                "target_wheel_velocity": telemetry.target_wheel_velocity,
                "actual_wheel_velocity": telemetry.actual_wheel_velocity,
                "velocity_error": telemetry.plant_velocity_error,
                "f_cmd": telemetry.f_cmd,
                "f_app": telemetry.f_app,
                "external_force_n": telemetry.external_force_n,
                "external_com_bias_rad": telemetry.external_com_bias_rad,
                "x_ddot": telemetry.x_ddot,
                "theta_ddot": telemetry.theta_ddot,
                "command_saturated": telemetry.command_saturated,
                "force_saturated": telemetry.force_saturated,
            }
            recorder.record_step(row)
            if abs(telemetry.plant_pitch_deg) > fail_fast_pitch_deg:
                udp.send(BalancerMsgId.SimStopRun, SimStopRunPayload(run_id=run_id).pack())
        elif msg_id == int(BalancerMsgId.SimRunDone):
            done = SimRunDonePayload.unpack(payload)
            if done.run_id == run_id:
                break

    if done is None:
        raise AssertionError(f"Timed out waiting for SimRunDone for run_id={run_id}")

    summary = recorder.write_csv_json_plots(output_dir)
    (output_dir / "done.json").write_text(
        json.dumps(
            {
                "run_id": done.run_id,
                "reason_code": done.reason_code,
                "sample_count": done.sample_count,
                "elapsed_s": done.elapsed_s,
                "final_pitch_deg": done.final_pitch_deg,
                "max_abs_pitch_deg": done.max_abs_pitch_deg,
                "tail_rms_pitch_deg": done.tail_rms_pitch_deg,
                "tail_rail_fraction": done.tail_rail_fraction,
                "tail_mean_abs_pitch_deg": done.tail_mean_abs_pitch_deg,
                "max_abs_position_m": done.max_abs_position_m,
                "tail_mean_abs_velocity_mps": done.tail_mean_abs_velocity_mps,
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    return summary, metadata, done
