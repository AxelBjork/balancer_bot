from __future__ import annotations

import argparse
import shutil
import struct
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tests.python.generated_balancer import (
    ImuRawPayload,
    JoystickCommandPayload,
    MsgId,
    PhysicsTickPayload,
)

from tests.fuzz.registry import DEFAULT_BUILD_DIR, corpus_root

_FIXED_TIMESTAMP_US = 1_000_000


def _udp_frame(msg_id: int, payload: bytes) -> bytes:
    return int(msg_id).to_bytes(2, "little") + payload


def _write(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(payload)


def _reset_dir(path: Path) -> None:
    shutil.rmtree(path, ignore_errors=True)
    path.mkdir(parents=True, exist_ok=True)


def _encode_disturbance(
    *,
    kind: int = 0,
    start_s: float = 0.0,
    duration_s: float = 0.0,
    force_n: float = 0.0,
    com_bias_rad: float = 0.0,
    force_n_end: float | None = None,
    com_bias_rad_end: float | None = None,
) -> bytes:
    if force_n_end is None:
        force_n_end = force_n
    if com_bias_rad_end is None:
        com_bias_rad_end = com_bias_rad
    return struct.pack(
        "<BBHffffff",
        kind,
        0,
        0,
        start_s,
        duration_s,
        force_n,
        com_bias_rad,
        force_n_end,
        com_bias_rad_end,
    )


def _encode_scenario(
    *,
    physics_profile: int,
    duration_s: float,
    initial_pitch_deg: float = 0.0,
    com_angle_offset_rad: float = 0.0,
    wheel_slip_factor: float = 1.0,
    velocity_feedback_scale: float = 1.0,
    velocity_feedback_tau_s: float = 0.0,
    imu_pitch_lag_s: float = 0.0,
    disturbances: list[bytes] | None = None,
) -> bytes:
    raw_disturbances = list(disturbances or [])
    if len(raw_disturbances) > 4:
        raise ValueError("simulator corpus seeds support at most four disturbances")
    disturbance_count = len(raw_disturbances)
    raw_disturbances += [_encode_disturbance()] * (4 - len(raw_disturbances))
    header = struct.pack(
        "<BBBB7f",
        1,
        physics_profile,
        disturbance_count,
        0,
        duration_s,
        initial_pitch_deg,
        com_angle_offset_rad,
        wheel_slip_factor,
        velocity_feedback_scale,
        velocity_feedback_tau_s,
        imu_pitch_lag_s,
    )
    return header + b"".join(raw_disturbances[:4])


def _encode_joy_segment(
    *,
    start_s: float = 0.0,
    duration_s: float = 0.0,
    forward: float = 0.0,
    turn: float = 0.0,
    forward_end: float | None = None,
    turn_end: float | None = None,
) -> bytes:
    if forward_end is None:
        forward_end = forward
    if turn_end is None:
        turn_end = turn
    return struct.pack("<ffffff", start_s, duration_s, forward, turn, forward_end, turn_end)


def _encode_stability_scenario(
    *,
    physics_profile: int,
    duration_s: float,
    initial_pitch_deg: float = 0.0,
    com_angle_offset_rad: float = 0.0,
    wheel_slip_factor: float = 1.0,
    velocity_feedback_scale: float = 1.0,
    velocity_feedback_tau_s: float = 0.0,
    imu_pitch_lag_s: float = 0.0,
    disturbances: list[bytes] | None = None,
    joy_segments: list[bytes] | None = None,
) -> bytes:
    base = bytearray(
        _encode_scenario(
            physics_profile=physics_profile,
            duration_s=duration_s,
            initial_pitch_deg=initial_pitch_deg,
            com_angle_offset_rad=com_angle_offset_rad,
            wheel_slip_factor=wheel_slip_factor,
            velocity_feedback_scale=velocity_feedback_scale,
            velocity_feedback_tau_s=velocity_feedback_tau_s,
            imu_pitch_lag_s=imu_pitch_lag_s,
            disturbances=disturbances,
        )
    )
    base[0] = 2
    raw_joy = list(joy_segments or [])
    if len(raw_joy) > 4:
        raise ValueError("simulator stability corpus seeds support at most four joystick segments")
    joy_count = len(raw_joy)
    raw_joy += [_encode_joy_segment()] * (4 - len(raw_joy))
    return bytes(base) + struct.pack("<BBH", joy_count, 0, 0) + b"".join(raw_joy[:4])


def write_udp_corpus(output_root: Path) -> None:
    udp_dir = output_root / "udp_sequence"
    _reset_dir(udp_dir)
    timestamp_us = _FIXED_TIMESTAMP_US

    imu_raw = ImuRawPayload(
        acc=[0.0, 0.0, 9.81],
        gyr=[0.0, 0.25, 0.0],
        timestamp_us=timestamp_us,
    )
    tick = PhysicsTickPayload(dt_s=0.0025, sim_time_us=2500)
    joystick = JoystickCommandPayload(forward=0.2, turn=-0.1)

    _write(udp_dir / "imu_raw_only.bin", _udp_frame(MsgId.ImuRawData, imu_raw.pack()))
    _write(udp_dir / "tick_only.bin", _udp_frame(MsgId.PhysicsTick, tick.pack()))
    _write(udp_dir / "joystick_only.bin", _udp_frame(MsgId.JoystickCommand, joystick.pack()))

    sequence = b"".join(
        [
            _udp_frame(MsgId.JoystickCommand, joystick.pack()),
            _udp_frame(MsgId.ImuRawData, imu_raw.pack()),
            _udp_frame(MsgId.PhysicsTick, tick.pack()),
            _udp_frame(
                MsgId.ImuRawData,
                ImuRawPayload(
                    acc=[0.0, 0.0, 9.81],
                    gyr=[0.0, 0.20, 0.0],
                    timestamp_us=timestamp_us + 2500,
                ).pack(),
            ),
            _udp_frame(
                MsgId.PhysicsTick,
                PhysicsTickPayload(dt_s=0.0025, sim_time_us=5000).pack(),
            ),
        ]
    )
    _write(udp_dir / "control_sequence.bin", sequence)


def write_simulator_corpus(output_root: Path) -> None:
    sim_dir = output_root / "simulator_scenario"
    _reset_dir(sim_dir)

    _write(
        sim_dir / "simplified_neutral_hold.bin",
        _encode_scenario(physics_profile=0, duration_s=1.0),
    )
    _write(
        sim_dir / "simplified_pitch_bias.bin",
        _encode_scenario(physics_profile=0, duration_s=1.0, initial_pitch_deg=0.10),
    )
    _write(
        sim_dir / "realistic_slow_push_recover.bin",
        _encode_scenario(
            physics_profile=1,
            duration_s=2.0,
            velocity_feedback_scale=0.05,
            disturbances=[
                _encode_disturbance(kind=1, start_s=0.2, duration_s=0.7, force_n=0.0, force_n_end=2.0),
                _encode_disturbance(kind=1, start_s=0.9, duration_s=0.7, force_n=2.0, force_n_end=0.0),
            ],
        ),
    )
    _write(
        sim_dir / "realistic_hold_bias.bin",
        _encode_scenario(
            physics_profile=1,
            duration_s=2.0,
            velocity_feedback_scale=0.08,
            velocity_feedback_tau_s=0.10,
            imu_pitch_lag_s=0.01,
            disturbances=[
                _encode_disturbance(kind=2, start_s=0.3, duration_s=0.0, com_bias_rad=0.02),
            ],
        ),
    )


def write_simulator_stability_corpus(output_root: Path) -> None:
    sim_dir = output_root / "simulator_stability"
    _reset_dir(sim_dir)

    _write(
        sim_dir / "realistic_neutral_recoverable.bin",
        _encode_stability_scenario(physics_profile=1, duration_s=1.0, velocity_feedback_scale=0.05),
    )
    _write(
        sim_dir / "realistic_recoverable_pitch.bin",
        _encode_stability_scenario(
            physics_profile=1,
            duration_s=1.5,
            initial_pitch_deg=12.0,
            velocity_feedback_scale=0.05,
        ),
    )
    _write(
        sim_dir / "realistic_short_push.bin",
        _encode_stability_scenario(
            physics_profile=1,
            duration_s=2.0,
            velocity_feedback_scale=0.05,
            disturbances=[
                _encode_disturbance(kind=1, start_s=0.2, duration_s=0.6, force_n=0.0, force_n_end=1.5),
                _encode_disturbance(kind=1, start_s=0.8, duration_s=0.6, force_n=1.5, force_n_end=0.0),
            ],
        ),
    )
    _write(
        sim_dir / "realistic_com_bias.bin",
        _encode_stability_scenario(
            physics_profile=1,
            duration_s=2.0,
            velocity_feedback_scale=0.05,
            velocity_feedback_tau_s=0.10,
            imu_pitch_lag_s=0.01,
            disturbances=[
                _encode_disturbance(kind=2, start_s=0.3, duration_s=0.0, com_bias_rad=0.01),
            ],
        ),
    )
    _write(
        sim_dir / "realistic_forward_authority.bin",
        _encode_stability_scenario(
            physics_profile=1,
            duration_s=2.0,
            velocity_feedback_scale=0.05,
            joy_segments=[
                _encode_joy_segment(start_s=0.15, duration_s=0.6, forward=0.0, turn=0.0, forward_end=0.8),
                _encode_joy_segment(start_s=0.75, duration_s=0.6, forward=0.8, turn=0.0, forward_end=0.0),
            ],
        ),
    )
    _write(
        sim_dir / "realistic_xbox_mixed_authority.bin",
        _encode_stability_scenario(
            physics_profile=1,
            duration_s=2.5,
            velocity_feedback_scale=0.05,
            joy_segments=[
                _encode_joy_segment(start_s=0.2, duration_s=0.5, forward=0.0, turn=0.0, forward_end=0.6, turn_end=0.7),
                _encode_joy_segment(start_s=0.7, duration_s=0.5, forward=0.6, turn=0.7, forward_end=-0.4, turn_end=-0.7),
                _encode_joy_segment(start_s=1.2, duration_s=0.5, forward=-0.4, turn=-0.7, forward_end=0.0, turn_end=0.0),
            ],
        ),
    )


def write_corpora(output_root: Path) -> None:
    output_root.mkdir(parents=True, exist_ok=True)
    write_udp_corpus(output_root)
    write_simulator_corpus(output_root)
    write_simulator_stability_corpus(output_root)


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate deterministic AFL seed corpora.")
    parser.add_argument(
        "--output-root",
        default=str(corpus_root(DEFAULT_BUILD_DIR)),
        help="Directory where corpus subdirectories are written",
    )
    args = parser.parse_args()

    write_corpora(Path(args.output_root))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
