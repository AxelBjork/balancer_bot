from __future__ import annotations

import json
import struct
import time
from dataclasses import dataclass, fields
from functools import lru_cache
from pathlib import Path

from generated_balancer import (
    BalancerMsgId,
    SimRunDonePayload,
    SimPitchAuthoritySegmentPayload,
    SimStartAckPayload,
    SimStartRunPayload,
    SimStopRunPayload,
    SystemTelemetryPayload,
    SimulatorTelemetryPayload,
)

from tests.python.support.run_artifacts import RunRecorder

PHYSICS_SIMPLIFIED = 0
PHYSICS_REALISTIC = 1
PHYSICS_ACTUATOR_STRESS = 2
# Offline-only direct-force references. They bypass the phase-position
# actuator and are controller-validation profiles.  The old constant remains
# as a source-compatible alias for existing Python callers.
PHYSICS_DIRECT_ACTUATOR = 3
PHYSICS_IDEAL_FORCE = PHYSICS_DIRECT_ACTUATOR
# First-stage 1/32 STEP-to-magnetic-field/rotor-phase actuator profile.
PHYSICS_STEPPER_PHASE = 6
# First averaged electrical-driver stage: same phase mechanics with
# voltage-limited R/L/back-EMF current evolution.
PHYSICS_STEPPER_PHASE_ELECTRICAL = 7

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

_TELEMETRY_RUN_ID_OFFSET = 0
_TELEMETRY_PLANT_PITCH_OFFSET = SystemTelemetryPayload.WIRE_SIZE + struct.calcsize("<I")


def _inspect_telemetry_header(payload: bytes) -> tuple[int, float]:
    """Read only the fields needed while the UDP socket is being drained."""
    if len(payload) < SimulatorTelemetryPayload.WIRE_SIZE:
        raise AssertionError(
            "Simulator telemetry payload is truncated: "
            f"received={len(payload)} expected={SimulatorTelemetryPayload.WIRE_SIZE}"
        )
    (run_id,) = struct.unpack_from("<I", payload, _TELEMETRY_RUN_ID_OFFSET)
    (plant_pitch_deg,) = struct.unpack_from(
        "<f", payload, _TELEMETRY_PLANT_PITCH_OFFSET
    )
    return run_id, plant_pitch_deg


@dataclass(frozen=True)
class RunScenarioResult:
    """One live run's terminal data and the canonical received telemetry frame."""

    summary: dict
    metadata: dict
    done: SimRunDonePayload
    frame: object

    def __iter__(self):
        """Keep the old three-value unpacking usable for external test helpers."""
        yield self.summary
        yield self.metadata
        yield self.done


@lru_cache(maxsize=None)
def _dataclass_fields(value_type):
    """Cache reflection metadata for the high-rate simulator telemetry path."""
    return fields(value_type)

_PHYSICS_DEFAULTS = {
    PHYSICS_SIMPLIFIED: {
        "motor_max_force_n": 22.5,
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
        "motor_max_force_n": 22.5,
        "motor_no_load_speed_mps": 1.2,
        "traction_coefficient": 1.0,
        "motor_velocity_damping": 40.0,
        "cart_damping": 1.0,
        "pitch_damping": 0.02,
        "motor_tau_s": 0.002,
        "phase_error_limit_steps": 16.0,
        "tire_stiffness_n_per_m": 3000.0,
        "tire_damping_n_s_per_m": 35.0,
        "wheel_equivalent_mass_kg": 0.10,
    },
    PHYSICS_ACTUATOR_STRESS: {
        "motor_max_force_n": 22.5,
        "motor_no_load_speed_mps": 1.2,
        "traction_coefficient": 1.0,
        "motor_velocity_damping": 40.0,
        "cart_damping": 1.0,
        "pitch_damping": 0.02,
        "motor_tau_s": 0.020,
        "phase_error_limit_steps": 16.0,
        "tire_stiffness_n_per_m": 3000.0,
        "tire_damping_n_s_per_m": 35.0,
        "wheel_equivalent_mass_kg": 0.10,
    },
    PHYSICS_DIRECT_ACTUATOR: {
        "motor_max_force_n": 22.5,
        "motor_no_load_speed_mps": 1.2,
        "traction_coefficient": 1.0,
        "motor_velocity_damping": 0.0,
        "cart_damping": 1.0,
        "pitch_damping": 0.02,
        "motor_tau_s": 0.0,
        "phase_error_limit_steps": 16.0,
        "tire_stiffness_n_per_m": 3000.0,
        "tire_damping_n_s_per_m": 35.0,
        "wheel_equivalent_mass_kg": 0.10,
    },
    PHYSICS_STEPPER_PHASE: {
        # These scalar fields are retained only for common start-payload
        # compatibility; the StepperPhase actuator uses its own torque/phase
        # topology and does not interpret SPS as force.
        "motor_max_force_n": 2 * 0.45 / (0.0824 / 2),
        "motor_no_load_speed_mps": 0.0,
        "traction_coefficient": 1.0,
        "motor_velocity_damping": 0.0,
        "cart_damping": 1.0,
        "pitch_damping": 0.02,
        "motor_tau_s": 0.0,
        "phase_error_limit_steps": 0.0,
        "tire_stiffness_n_per_m": 0.0,
        "tire_damping_n_s_per_m": 0.0,
        "wheel_equivalent_mass_kg": 0.0,
    },
    PHYSICS_STEPPER_PHASE_ELECTRICAL: {
        # The electrical actuator owns its R/L/current/back-EMF parameters;
        # these common scalar fields keep the start payload compatible.
        "motor_max_force_n": 2 * 0.45 / (0.0824 / 2),
        "motor_no_load_speed_mps": 0.0,
        "traction_coefficient": 1.0,
        "motor_velocity_damping": 0.0,
        "cart_damping": 1.0,
        "pitch_damping": 0.02,
        "motor_tau_s": 0.0,
        "phase_error_limit_steps": 0.0,
        "tire_stiffness_n_per_m": 0.0,
        "tire_damping_n_s_per_m": 0.0,
        "wheel_equivalent_mass_kg": 0.0,
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
    initial_pitch_rate_dps: float = 0.0,
    initial_velocity_mps: float = 0.0,
    com_angle_offset_rad: float = 0.0,
    total_mass_scale: float = 1.0,
    pitch_inertia_scale: float = 1.0,
    physics_override: dict | None = None,
    imu_pitch_lag_s: float = 0.0,
    imu_noise_seed: int = 0,
    accel_noise_std_mps2: float = 0.0,
    gyro_noise_std_rad_s: float = 0.0,
    imu_timestamp_jitter_us: float = 0.0,
    imu_sample_loss_rate: float = 0.0,
    accel_bias_mps2: list[float] | None = None,
    gyro_bias_rad_s: list[float] | None = None,
    velocity_estimator_bias_mps: float = 0.0,
    velocity_estimator_bias_drift_mps_per_s: float = 0.0,
    velocity_estimator_scale: float = 1.0,
    velocity_estimator_latency_s: float = 0.0,
    disturbances: list[dict] | None = None,
    joy_segments: list[dict] | None = None,
    pitch_authority_segments: list[dict] | None = None,
    pitch_authority_refresh_dropout: dict | None = None,
    pid_config_path: str = "",
) -> SimStartRunPayload:
    disturbances = list(disturbances or [])
    joy_segments = list(joy_segments or [])
    pitch_authority_segments = list(pitch_authority_segments or [])
    pitch_authority_refresh_dropout = dict(pitch_authority_refresh_dropout or {})
    if len(disturbances) > 10:
        raise ValueError("SimStartRunPayload supports at most 10 disturbance segments")
    if len(joy_segments) > 4:
        raise ValueError("SimStartRunPayload supports at most 4 joystick segments")
    if len(pitch_authority_segments) > 12:
        raise ValueError("SimStartRunPayload supports at most 12 pitch-authority segments")
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
    wire_pitch_authority = [
        {"start_s": 0.0, "duration_s": 0.0, "target_deg": 0.0, "com_trim_deg": 0.0}
        for _ in range(12)
    ]
    for idx, segment in enumerate(pitch_authority_segments):
        wire_pitch_authority[idx] = {
            "start_s": float(segment.get("start_s", 0.0)),
            "duration_s": float(segment.get("duration_s", 0.0)),
            "target_deg": float(segment.get("target_deg", 0.0)),
            "com_trim_deg": float(segment.get("com_trim_deg", 0.0)),
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
        initial_velocity_mps=initial_velocity_mps,
        com_angle_offset_rad=com_angle_offset_rad,
        total_mass_scale=total_mass_scale,
        pitch_inertia_scale=pitch_inertia_scale,
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
        velocity_estimator_bias_mps=velocity_estimator_bias_mps,
        velocity_estimator_bias_drift_mps_per_s=velocity_estimator_bias_drift_mps_per_s,
        velocity_estimator_scale=velocity_estimator_scale,
        velocity_estimator_latency_s=velocity_estimator_latency_s,
        initial_pitch_rate_dps=initial_pitch_rate_dps,
        disturbances=wire_disturbances,
        joy_segments=wire_joy,
        pitch_authority_segments=[SimPitchAuthoritySegmentPayload(**item) for item in wire_pitch_authority],
        pitch_authority_refresh_dropout_start_s=float(
            pitch_authority_refresh_dropout.get("start_s", 0.0)
        ),
        pitch_authority_refresh_dropout_duration_s=float(
            pitch_authority_refresh_dropout.get("duration_s", 0.0)
        ),
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


def validate_telemetry_packets(
    telemetry: list[SimulatorTelemetryPayload],
    done: SimRunDonePayload,
    telemetry_stride: int,
) -> None:
    """Require ordered packet sequence numbers and the exact completed count."""
    sequences = [int(sample.system.packet_seq) for sample in telemetry]
    expected_sequence = list(range(1, len(sequences) + 1))
    if sequences != expected_sequence:
        raise AssertionError(
            "Simulator telemetry packet sequence is not contiguous: "
            f"received={sequences[:5]}...{sequences[-5:] if sequences else []}"
        )

    if done.reason_code == DONE_INTERNAL_ERROR:
        raise AssertionError("Simulator reported an internal transport failure")

    if done.reason_code in (DONE_COMPLETED, DONE_ACCEPTANCE_FAILED):
        expected_count = (
            0
            if telemetry_stride <= 0
            else (int(done.sample_count) + telemetry_stride - 1) // telemetry_stride
        )
        if len(telemetry) != expected_count:
            raise AssertionError(
                "Simulator telemetry packet count mismatch: "
                f"received={len(telemetry)} expected={expected_count} "
                f"sample_count={done.sample_count} stride={telemetry_stride}"
            )


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
    initial_pitch_rate_dps: float = 0.0,
    initial_velocity_mps: float = 0.0,
    com_angle_offset_rad: float = 0.0,
    total_mass_scale: float = 1.0,
    pitch_inertia_scale: float = 1.0,
    physics_override: dict | None = None,
    imu_pitch_lag_s: float = 0.0,
    imu_noise_seed: int = 0,
    accel_noise_std_mps2: float = 0.0,
    gyro_noise_std_rad_s: float = 0.0,
    imu_timestamp_jitter_us: float = 0.0,
    imu_sample_loss_rate: float = 0.0,
    accel_bias_mps2: list[float] | None = None,
    gyro_bias_rad_s: list[float] | None = None,
    velocity_estimator_bias_mps: float = 0.0,
    velocity_estimator_bias_drift_mps_per_s: float = 0.0,
    velocity_estimator_scale: float = 1.0,
    velocity_estimator_latency_s: float = 0.0,
    disturbances: list[dict] | None = None,
    joy_segments: list[dict] | None = None,
    pitch_authority_segments: list[dict] | None = None,
    pitch_authority_refresh_dropout: dict | None = None,
    pid_config_path: str = "",
    model_name: str = "",
    scenario_id: str = "",
    subrun_id: str = "",
    scenario_category: str = "",
    scenario_intent: str = "",
    write_plots: bool = False,
    fail_fast_pitch_deg: float = 75.0,
    done_timeout: float = 15.0,
) -> RunScenarioResult:
    output_dir.mkdir(parents=True, exist_ok=True)
    recorder = RunRecorder()

    metadata = {
        "run_id": str(run_id),
        "scenario_name": output_dir.name,
        "physics_profile": {
            PHYSICS_SIMPLIFIED: "simplified",
            PHYSICS_REALISTIC: "realistic",
            PHYSICS_ACTUATOR_STRESS: "actuator_stress",
            PHYSICS_DIRECT_ACTUATOR: "direct_actuator",
            PHYSICS_STEPPER_PHASE: "stepper_phase",
            PHYSICS_STEPPER_PHASE_ELECTRICAL: "stepper_phase_electrical",
        }.get(physics_profile, f"unknown:{physics_profile}"),
        "pid_profile": pid_config_path or "pid.conf",
        "duration_s": duration_s,
        "telemetry_stride": telemetry_stride,
        "transfer_scenario_index": transfer_scenario_index,
        "initial_pitch_deg": initial_pitch_deg,
        "initial_pitch_rate_dps": initial_pitch_rate_dps,
        "initial_velocity_mps": initial_velocity_mps,
        "com_angle_offset_rad": com_angle_offset_rad,
        "total_mass_scale": total_mass_scale,
        "pitch_inertia_scale": pitch_inertia_scale,
        "imu_pitch_lag_s": imu_pitch_lag_s,
        "imu_noise_seed": imu_noise_seed,
        "accel_noise_std_mps2": accel_noise_std_mps2,
        "gyro_noise_std_rad_s": gyro_noise_std_rad_s,
        "imu_timestamp_jitter_us": imu_timestamp_jitter_us,
        "imu_sample_loss_rate": imu_sample_loss_rate,
        "accel_bias_mps2": accel_bias_mps2 or [0.0, 0.0, 0.0],
        "gyro_bias_rad_s": gyro_bias_rad_s or [0.0, 0.0, 0.0],
        "velocity_estimator_bias_mps": velocity_estimator_bias_mps,
        "velocity_estimator_bias_drift_mps_per_s": velocity_estimator_bias_drift_mps_per_s,
        "velocity_estimator_scale": velocity_estimator_scale,
        "velocity_estimator_latency_s": velocity_estimator_latency_s,
    }
    if model_name:
        metadata["model"] = model_name
    if scenario_id:
        metadata["scenario_id"] = scenario_id
    if subrun_id:
        metadata["subrun_id"] = subrun_id
    if scenario_category:
        metadata["scenario_category"] = scenario_category
    if scenario_intent:
        metadata["scenario_intent"] = scenario_intent
    if disturbances:
        metadata["disturbances"] = disturbances
    if joy_segments:
        metadata["joy_segments"] = joy_segments
    if pitch_authority_segments:
        metadata["pitch_authority_segments"] = pitch_authority_segments
    if pitch_authority_refresh_dropout:
        metadata["pitch_authority_refresh_dropout"] = pitch_authority_refresh_dropout
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
        initial_pitch_rate_dps=initial_pitch_rate_dps,
        initial_velocity_mps=initial_velocity_mps,
        com_angle_offset_rad=com_angle_offset_rad,
        total_mass_scale=total_mass_scale,
        pitch_inertia_scale=pitch_inertia_scale,
        physics_override=physics_override,
        imu_pitch_lag_s=imu_pitch_lag_s,
        imu_noise_seed=imu_noise_seed,
        accel_noise_std_mps2=accel_noise_std_mps2,
        gyro_noise_std_rad_s=gyro_noise_std_rad_s,
        imu_timestamp_jitter_us=imu_timestamp_jitter_us,
        imu_sample_loss_rate=imu_sample_loss_rate,
        accel_bias_mps2=accel_bias_mps2,
        gyro_bias_rad_s=gyro_bias_rad_s,
        velocity_estimator_bias_mps=velocity_estimator_bias_mps,
        velocity_estimator_bias_drift_mps_per_s=velocity_estimator_bias_drift_mps_per_s,
        velocity_estimator_scale=velocity_estimator_scale,
        velocity_estimator_latency_s=velocity_estimator_latency_s,
        disturbances=disturbances,
        joy_segments=joy_segments,
        pitch_authority_segments=pitch_authority_segments,
        pitch_authority_refresh_dropout=pitch_authority_refresh_dropout,
        pid_config_path=pid_config_path,
    )
    udp.send(BalancerMsgId.SimStartRun, start.pack())
    ack = wait_for_ack(udp, run_id)
    if not ack.accepted:
        raise AssertionError(f"Simulator rejected run_id={run_id} with status={ack.status_code}")

    done = None
    # Drain the UDP stream in raw bursts.  Full dataclass expansion and row
    # reflection happen only after SimRunDone; otherwise the receiver can be
    # slower than a deterministic full-rate simulator burst.  The run ID and
    # plant pitch are inspected from each datagram during reception so the
    # existing live fail-fast behavior remains active.
    received_payloads: list[bytes] = []
    stop_sent = False
    deadline = time.monotonic() + done_timeout
    while time.monotonic() < deadline:
        try:
            packets = udp.recv_batch(
                timeout=min(0.1, max(0.01, deadline - time.monotonic()))
            )
        except TimeoutError:
            continue
        for msg_id, payload in packets:
            if msg_id == int(BalancerMsgId.SimulatorTelemetry):
                telemetry_run_id, plant_pitch_deg = _inspect_telemetry_header(payload)
                if telemetry_run_id != run_id:
                    continue
                received_payloads.append(payload)
                if not stop_sent and abs(plant_pitch_deg) > fail_fast_pitch_deg:
                    udp.send(BalancerMsgId.SimStopRun, SimStopRunPayload(run_id=run_id).pack())
                    stop_sent = True
            elif msg_id == int(BalancerMsgId.SimRunDone):
                candidate = SimRunDonePayload.unpack(payload)
                if candidate.run_id == run_id:
                    done = candidate
                    break
        if done is not None:
            break

    if done is None:
        raise AssertionError(f"Timed out waiting for SimRunDone for run_id={run_id}")

    received_telemetry = [
        SimulatorTelemetryPayload.unpack(payload) for payload in received_payloads
    ]
    validate_telemetry_packets(received_telemetry, done, telemetry_stride)

    for telemetry in received_telemetry:
        # Keep simulator artifacts flat while the wire API remains a single,
        # nested simulator frame.
        row = {
            field.name: getattr(telemetry.system, field.name)
            for field in _dataclass_fields(type(telemetry.system))
        }
        row.update(
            {
                field.name: getattr(telemetry, field.name)
                for field in _dataclass_fields(type(telemetry))
                if field.name != "system"
            }
        )
        recorder.record_step(row)

    # Live pytest scenarios consume the CSV/JSON artifacts and summary metrics;
    # rendering Matplotlib SVGs for every run is intentionally opt-in because
    # it dominates the test wall time.  Report-generation tools can request
    # the full plots explicitly.
    summary = (
        recorder.write_csv_json_plots(output_dir)
        if write_plots
        else recorder.write_csv_json(output_dir)
    )
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
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    return RunScenarioResult(
        summary=summary,
        metadata=metadata,
        done=done,
        frame=recorder.materialize_frame(),
    )
