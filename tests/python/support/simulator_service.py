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
from tests.python.support.telemetry_rows import system_telemetry_to_row

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
    imu_noise_seed: int = 0,
    accel_noise_std_mps2: float = 0.0,
    gyro_noise_std_rad_s: float = 0.0,
    accel_bias_mps2: list[float] | None = None,
    gyro_bias_rad_s: list[float] | None = None,
    disturbances: list[dict] | None = None,
    pid_config_path: str = "",
) -> SimStartRunPayload:
    disturbances = list(disturbances or [])
    if len(disturbances) > 10:
        raise ValueError("SimStartRunPayload supports at most 10 disturbance segments")
    accel_bias_mps2 = list(accel_bias_mps2 or [0.0, 0.0, 0.0])
    gyro_bias_rad_s = list(gyro_bias_rad_s or [0.0, 0.0, 0.0])
    if len(accel_bias_mps2) != 3 or len(gyro_bias_rad_s) != 3:
        raise ValueError("IMU bias vectors must have exactly three entries")

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
        imu_noise_seed=imu_noise_seed,
        accel_noise_std_mps2=accel_noise_std_mps2,
        gyro_noise_std_rad_s=gyro_noise_std_rad_s,
        accel_bias_mps2=accel_bias_mps2,
        gyro_bias_rad_s=gyro_bias_rad_s,
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
    imu_noise_seed: int = 0,
    accel_noise_std_mps2: float = 0.0,
    gyro_noise_std_rad_s: float = 0.0,
    accel_bias_mps2: list[float] | None = None,
    gyro_bias_rad_s: list[float] | None = None,
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
        "imu_noise_seed": imu_noise_seed,
        "accel_noise_std_mps2": accel_noise_std_mps2,
        "gyro_noise_std_rad_s": gyro_noise_std_rad_s,
        "accel_bias_mps2": accel_bias_mps2 or [0.0, 0.0, 0.0],
        "gyro_bias_rad_s": gyro_bias_rad_s or [0.0, 0.0, 0.0],
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
        imu_noise_seed=imu_noise_seed,
        accel_noise_std_mps2=accel_noise_std_mps2,
        gyro_noise_std_rad_s=gyro_noise_std_rad_s,
        accel_bias_mps2=accel_bias_mps2,
        gyro_bias_rad_s=gyro_bias_rad_s,
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
            recorder.record_step(system_telemetry_to_row(telemetry))
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
