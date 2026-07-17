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
DONE_ACCEPTANCE_FAILED = 4

DISTURBANCE_STEP = 0
DISTURBANCE_RAMP = 1
DISTURBANCE_HOLD_BIAS = 2

_PHYSICS_DEFAULTS = {
    PHYSICS_SIMPLIFIED: {
        "motor_max_force_n": 8.0,
        "motor_no_load_speed_mps": 1.6,
        "traction_coefficient": 1.2,
        "motor_velocity_damping": 30.0,
        "cart_damping": 0.4,
        "pitch_damping": 0.04,
        "motor_tau_s": 0.004,
        "phase_error_limit_steps": 16.0,
        "tire_stiffness_n_per_m": 2500.0,
        "tire_damping_n_s_per_m": 30.0,
        "wheel_equivalent_mass_kg": 0.10,
    },
    PHYSICS_REALISTIC: {
        "motor_max_force_n": 12.0,
        "motor_no_load_speed_mps": 1.2,
        "traction_coefficient": 1.0,
        "motor_velocity_damping": 40.0,
        "cart_damping": 1.0,
        "pitch_damping": 0.02,
        "motor_tau_s": 0.008,
        "phase_error_limit_steps": 16.0,
        "tire_stiffness_n_per_m": 3000.0,
        "tire_damping_n_s_per_m": 35.0,
        "wheel_equivalent_mass_kg": 0.10,
    },
}


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
    telemetry_stride: int = 80,
    transfer_scenario_index: int = 0xFFFF,
    initial_pitch_deg: float = 0.0,
    com_angle_offset_rad: float = 0.0,
    mass_scale: float = 1.0,
    com_height_scale: float = 1.0,
    inertia_scale: float = 1.0,
    physics_override: dict | None = None,
    imu_pitch_lag_s: float = 0.0,
    imu_noise_seed: int = 0,
    accel_noise_std_mps2: float = 0.0,
    gyro_noise_std_rad_s: float = 0.0,
    imu_timestamp_jitter_us: float = 0.0,
    imu_sample_loss_rate: float = 0.0,
    accel_bias_mps2: list[float] | None = None,
    gyro_bias_rad_s: list[float] | None = None,
    disturbances: list[dict] | None = None,
    joy_segments: list[dict] | None = None,
    pid_config_path: str = "",
) -> SimStartRunPayload:
    disturbances = list(disturbances or [])
    joy_segments = list(joy_segments or [])
    if len(disturbances) > 10:
        raise ValueError("SimStartRunPayload supports at most 10 disturbance segments")
    if len(joy_segments) > 4:
        raise ValueError("SimStartRunPayload supports at most 4 joystick segments")
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

    wire_joy = [
        {"start_s": 0.0, "duration_s": 0.0, "forward": 0.0, "turn": 0.0,
         "forward_end": 0.0, "turn_end": 0.0}
        for _ in range(4)
    ]
    for idx, segment in enumerate(joy_segments):
        wire_joy[idx] = {
            "start_s": float(segment.get("start_s", 0.0)),
            "duration_s": float(segment.get("duration_s", 0.0)),
            "forward": float(segment.get("forward", 0.0)),
            "turn": float(segment.get("turn", 0.0)),
            "forward_end": float(segment.get("forward_end", segment.get("forward", 0.0))),
            "turn_end": float(segment.get("turn_end", segment.get("turn", 0.0))),
        }
    override = dict(_PHYSICS_DEFAULTS[physics_profile])
    override.update(physics_override or {})

    return SimStartRunPayload(
        run_id=run_id,
        physics_profile=physics_profile,
        has_physics_override=1 if physics_override is not None else 0,
        telemetry_stride=telemetry_stride,
        transfer_scenario_index=transfer_scenario_index,
        reserved1=0,
        duration_s=duration_s,
        initial_pitch_deg=initial_pitch_deg,
        com_angle_offset_rad=com_angle_offset_rad,
        mass_scale=mass_scale,
        com_height_scale=com_height_scale,
        inertia_scale=inertia_scale,
        motor_max_force_n=float(override["motor_max_force_n"]),
        motor_no_load_speed_mps=float(override["motor_no_load_speed_mps"]),
        motor_velocity_damping=float(override["motor_velocity_damping"]),
        motor_tau_s=float(override["motor_tau_s"]),
        traction_coefficient=float(override["traction_coefficient"]),
        pitch_damping=float(override["pitch_damping"]),
        cart_damping=float(override["cart_damping"]),
        phase_error_limit_steps=float(override["phase_error_limit_steps"]),
        tire_stiffness_n_per_m=float(override["tire_stiffness_n_per_m"]),
        tire_damping_n_s_per_m=float(override["tire_damping_n_s_per_m"]),
        wheel_equivalent_mass_kg=float(override["wheel_equivalent_mass_kg"]),
        imu_pitch_lag_s=imu_pitch_lag_s,
        imu_noise_seed=imu_noise_seed,
        accel_noise_std_mps2=accel_noise_std_mps2,
        gyro_noise_std_rad_s=gyro_noise_std_rad_s,
        imu_timestamp_jitter_us=imu_timestamp_jitter_us,
        imu_sample_loss_rate=imu_sample_loss_rate,
        accel_bias_mps2=accel_bias_mps2,
        gyro_bias_rad_s=gyro_bias_rad_s,
        disturbances=wire_disturbances,
        joy_segments=wire_joy,
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
    telemetry_stride: int = 80,
    transfer_scenario_index: int = 0xFFFF,
    initial_pitch_deg: float = 0.0,
    com_angle_offset_rad: float = 0.0,
    mass_scale: float = 1.0,
    com_height_scale: float = 1.0,
    inertia_scale: float = 1.0,
    physics_override: dict | None = None,
    imu_pitch_lag_s: float = 0.0,
    imu_noise_seed: int = 0,
    accel_noise_std_mps2: float = 0.0,
    gyro_noise_std_rad_s: float = 0.0,
    imu_timestamp_jitter_us: float = 0.0,
    imu_sample_loss_rate: float = 0.0,
    accel_bias_mps2: list[float] | None = None,
    gyro_bias_rad_s: list[float] | None = None,
    disturbances: list[dict] | None = None,
    joy_segments: list[dict] | None = None,
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
        "telemetry_stride": telemetry_stride,
        "transfer_scenario_index": transfer_scenario_index,
        "initial_pitch_deg": initial_pitch_deg,
        "com_angle_offset_rad": com_angle_offset_rad,
        "mass_scale": mass_scale,
        "com_height_scale": com_height_scale,
        "inertia_scale": inertia_scale,
        "imu_pitch_lag_s": imu_pitch_lag_s,
        "imu_noise_seed": imu_noise_seed,
        "accel_noise_std_mps2": accel_noise_std_mps2,
        "gyro_noise_std_rad_s": gyro_noise_std_rad_s,
        "imu_timestamp_jitter_us": imu_timestamp_jitter_us,
        "imu_sample_loss_rate": imu_sample_loss_rate,
        "accel_bias_mps2": accel_bias_mps2 or [0.0, 0.0, 0.0],
        "gyro_bias_rad_s": gyro_bias_rad_s or [0.0, 0.0, 0.0],
    }
    if disturbances:
        metadata["disturbances"] = disturbances
    if joy_segments:
        metadata["joy_segments"] = joy_segments
    if physics_override is not None:
        metadata["physics_override"] = physics_override
    recorder.begin_run(metadata)

    udp.drain()
    start = make_start_payload(
        run_id=run_id,
        physics_profile=physics_profile,
        duration_s=duration_s,
        telemetry_stride=telemetry_stride,
        transfer_scenario_index=transfer_scenario_index,
        initial_pitch_deg=initial_pitch_deg,
        com_angle_offset_rad=com_angle_offset_rad,
        mass_scale=mass_scale,
        com_height_scale=com_height_scale,
        inertia_scale=inertia_scale,
        physics_override=physics_override,
        imu_pitch_lag_s=imu_pitch_lag_s,
        imu_noise_seed=imu_noise_seed,
        accel_noise_std_mps2=accel_noise_std_mps2,
        gyro_noise_std_rad_s=gyro_noise_std_rad_s,
        imu_timestamp_jitter_us=imu_timestamp_jitter_us,
        imu_sample_loss_rate=imu_sample_loss_rate,
        accel_bias_mps2=accel_bias_mps2,
        gyro_bias_rad_s=gyro_bias_rad_s,
        disturbances=disturbances,
        joy_segments=joy_segments,
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
                "sim_time_s": telemetry.t_sec,
                "controller_fault_flags": telemetry.controller_fault_flags,
                "controller_saturation_flags": telemetry.controller_saturation_flags,
                "pitch_deg": telemetry.pitch_deg,
                "pitch_rate_dps": telemetry.pitch_rate_dps,
                "filtered_pitch_rate_dps": telemetry.filtered_pitch_rate_dps,
                "raw_acc_pitch_deg": telemetry.raw_acc_pitch_deg,
                "fused_pitch_deg": telemetry.fused_pitch_deg,
                "gyro_pitch_rate_dps": telemetry.gyro_pitch_rate_dps,
                "pitch_sp_deg": telemetry.pitch_sp_deg,
                "rate_setpoint_dps": telemetry.rate_setpoint_dps,
                "u_sps": telemetry.u_sps,
                "turn_sps": telemetry.turn_sps,
                "left_sps": telemetry.left_target_sps,
                "right_sps": telemetry.right_target_sps,
                "velocity_error_sps": telemetry.vel_error,
                "velocity_i_term_deg": telemetry.velocity_i_term_deg,
                "velocity_p_term_deg": telemetry.velocity_p_term_deg,
                "target_velocity_sps": telemetry.target_velocity_sps,
                "measured_vel_sps": telemetry.measured_vel_sps,
                "pitch_error_deg": telemetry.pitch_error_deg,
                "rate_error_dps": telemetry.rate_error_dps,
                "actuator_fault": telemetry.actuator_fault,
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
                "phase_error_steps": telemetry.phase_error_steps,
                "missed_steps": telemetry.missed_steps,
                "traction_limit_n": telemetry.traction_limit_n,
                "motor_force_limit_n": telemetry.motor_force_limit_n,
                "imu_timestamp_us": telemetry.imu_timestamp_us,
                "motor_update_dt_ms": telemetry.motor_update_dt_ms,
                "motor_feedback_age_ms": telemetry.motor_feedback_age_ms,
                "seed": telemetry.seed,
                "mass_scale": telemetry.mass_scale,
                "com_height_scale": telemetry.com_height_scale,
                "inertia_scale": telemetry.inertia_scale,
                "motor_max_force_n": telemetry.motor_max_force_n,
                "motor_no_load_speed_mps": telemetry.motor_no_load_speed_mps,
                "motor_velocity_damping": telemetry.motor_velocity_damping,
                "motor_tau_s": telemetry.motor_tau_s,
                "traction_coefficient": telemetry.traction_coefficient,
                "pitch_damping": telemetry.pitch_damping,
                "cart_damping": telemetry.cart_damping,
                "phase_error_limit_steps": telemetry.phase_error_limit_steps,
                "tire_stiffness_n_per_m": telemetry.tire_stiffness_n_per_m,
                "tire_damping_n_s_per_m": telemetry.tire_damping_n_s_per_m,
                "wheel_equivalent_mass_kg": telemetry.wheel_equivalent_mass_kg,
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
                "max_continuous_saturation_s": done.max_continuous_saturation_s,
                "actuator_fault_count": done.actuator_fault_count,
                "controller_fault_flags": done.controller_fault_flags,
                "timeline_hash": done.timeline_hash,
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    return summary, metadata, done
