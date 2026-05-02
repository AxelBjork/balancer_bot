#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import itertools
import json
import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
TEST_PYTHON = REPO_ROOT / "tests" / "python"
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
if str(TEST_PYTHON) not in sys.path:
    sys.path.insert(0, str(TEST_PYTHON))

from tests.python.support.simulator_service import (  # noqa: E402
    DONE_COMPLETED,
    PHYSICS_REALISTIC,
    run_scenario_live,
)
from udp_client import UdpClient  # noqa: E402

PID_KEYS = (
    "rate_P",
    "rate_I",
    "rate_D",
    "rate_I_lim",
    "rate_FF",
    "vel_P",
    "lean_trim_I",
    "lean_trim_max_deg",
    "pitch_P",
    "pitch_D",
)


def _ramp(
    *,
    start_s: float,
    duration_s: float,
    force_n: float,
    force_n_end: float,
    com_bias_rad: float = 0.0,
    com_bias_rad_end: float = 0.0,
) -> dict:
    return {
        "kind": "ramp",
        "start_s": start_s,
        "duration_s": duration_s,
        "force_n": force_n,
        "force_n_end": force_n_end,
        "com_bias_rad": com_bias_rad,
        "com_bias_rad_end": com_bias_rad_end,
    }


def _hold_bias(*, start_s: float, force_n: float = 0.0, com_bias_rad: float = 0.0, duration_s: float = 0.0) -> dict:
    return {
        "kind": "hold_bias",
        "start_s": start_s,
        "duration_s": duration_s,
        "force_n": force_n,
        "com_bias_rad": com_bias_rad,
    }


def _joy(*,
         start_s: float,
         duration_s: float,
         forward: float = 0.0,
         turn: float = 0.0,
         forward_end: float | None = None,
         turn_end: float | None = None) -> dict:
    if forward_end is None:
        forward_end = forward
    if turn_end is None:
        turn_end = turn
    return {
        "start_s": start_s,
        "duration_s": duration_s,
        "forward": forward,
        "turn": turn,
        "forward_end": forward_end,
        "turn_end": turn_end,
    }


def _alternating_pulse_train(*, start_s: float, pulse_duration_s: float, gap_s: float, count: int, amplitude: float) -> list[dict]:
    pulses: list[dict] = []
    sign = 1.0
    t = start_s
    for _ in range(count):
        pulses.append({"start_s": t, "duration_s": pulse_duration_s, "force_n": sign * amplitude})
        sign *= -1.0
        t += pulse_duration_s + gap_s
    return pulses


SCENARIOS: dict[str, dict] = {
    "realistic_disturbance_train_40s": {
        "duration_s": 40.0,
        "velocity_feedback_scale": 0.05,
        "disturbances": _alternating_pulse_train(
            start_s=1.0,
            pulse_duration_s=0.5,
            gap_s=0.5,
            count=10,
            amplitude=0.35,
        ),
    },
    "realistic_slow_push_recover_20s": {
        "duration_s": 20.0,
        "velocity_feedback_scale": 0.05,
        "disturbances": [
            _ramp(start_s=1.0, duration_s=1.5, force_n=0.0, force_n_end=2.0),
            _ramp(start_s=2.5, duration_s=1.5, force_n=2.0, force_n_end=0.0),
        ],
    },
    "realistic_noisy_slow_push_recover_20s": {
        "duration_s": 20.0,
        "velocity_feedback_scale": 0.05,
        "velocity_feedback_tau_s": 0.20,
        "imu_pitch_lag_s": 0.02,
        "imu_noise_seed": 2026,
        "accel_noise_std_mps2": 0.20,
        "gyro_noise_std_rad_s": 0.015,
        "disturbances": [
            _ramp(start_s=1.0, duration_s=1.5, force_n=0.0, force_n_end=2.0),
            _ramp(start_s=2.5, duration_s=1.5, force_n=2.0, force_n_end=0.0),
        ],
    },
    "realistic_com_offset_40s": {
        "duration_s": 40.0,
        "com_angle_offset_rad": 0.001,
        "velocity_feedback_scale": 0.05,
    },
    "realistic_negative_com_offset_40s": {
        "duration_s": 40.0,
        "com_angle_offset_rad": -0.001,
        "velocity_feedback_scale": 0.05,
    },
    "realistic_hold_bias_long_horizon_40s": {
        "duration_s": 40.0,
        "velocity_feedback_scale": 0.05,
        "velocity_feedback_tau_s": 0.10,
        "imu_pitch_lag_s": 0.01,
        "disturbances": [_hold_bias(start_s=1.0, com_bias_rad=0.02)],
    },
    "realistic_forward_authority_20s": {
        "duration_s": 20.0,
        "velocity_feedback_scale": 0.05,
        "joy_segments": [
            _joy(start_s=1.0, duration_s=2.0, forward=0.0, forward_end=0.65),
            _joy(start_s=3.0, duration_s=3.0, forward=0.65, forward_end=0.65),
            _joy(start_s=6.0, duration_s=2.0, forward=0.65, forward_end=0.0),
            _joy(start_s=10.0, duration_s=2.0, forward=0.0, forward_end=-0.45),
            _joy(start_s=12.0, duration_s=2.0, forward=-0.45, forward_end=0.0),
        ],
    },
    "realistic_xbox_mixed_authority_20s": {
        "duration_s": 20.0,
        "velocity_feedback_scale": 0.05,
        "joy_segments": [
            _joy(start_s=1.0, duration_s=1.5, forward=0.0, turn=0.0, forward_end=0.55, turn_end=0.7),
            _joy(start_s=2.5, duration_s=2.0, forward=0.55, turn=0.7),
            _joy(start_s=4.5, duration_s=1.5, forward=0.55, turn=0.7, forward_end=-0.35, turn_end=-0.7),
            _joy(start_s=6.0, duration_s=2.0, forward=-0.35, turn=-0.7),
            _joy(start_s=8.0, duration_s=1.5, forward=-0.35, turn=-0.7, forward_end=0.0, turn_end=0.0),
        ],
    },
    "realistic_authority_smoke_6s": {
        "duration_s": 6.0,
        "velocity_feedback_scale": 0.05,
        "joy_segments": [
            _joy(start_s=0.5, duration_s=0.8, forward=0.0, turn=0.0, forward_end=0.65, turn_end=0.6),
            _joy(start_s=1.3, duration_s=1.0, forward=0.65, turn=0.6),
            _joy(start_s=2.3, duration_s=0.8, forward=0.65, turn=0.6, forward_end=-0.45, turn_end=-0.6),
            _joy(start_s=3.1, duration_s=1.0, forward=-0.45, turn=-0.6),
            _joy(start_s=4.1, duration_s=0.8, forward=-0.45, turn=-0.6, forward_end=0.0, turn_end=0.0),
        ],
    },
    "realistic_slow_push_runaway_40s": {
        "duration_s": 40.0,
        "velocity_feedback_scale": 0.05,
        "velocity_feedback_tau_s": 0.10,
        "imu_pitch_lag_s": 0.01,
        "disturbances": [
            _ramp(start_s=1.0, duration_s=8.0, force_n=0.0, force_n_end=0.55),
            _hold_bias(start_s=9.0, force_n=0.55),
        ],
    },
}

SCENARIO_GROUPS: dict[str, list[str]] = {
    "smooth-first": ["realistic_disturbance_train_40s"],
    "validation": [
        "realistic_disturbance_train_40s",
        "realistic_slow_push_recover_20s",
        "realistic_noisy_slow_push_recover_20s",
        "realistic_com_offset_40s",
        "realistic_hold_bias_long_horizon_40s",
        "realistic_forward_authority_20s",
        "realistic_xbox_mixed_authority_20s",
    ],
    "authority": [
        "realistic_forward_authority_20s",
        "realistic_xbox_mixed_authority_20s",
    ],
    "authority-smoke": ["realistic_authority_smoke_6s"],
    "all-realistic": list(SCENARIOS),
}


def _float_list(value: str) -> list[float]:
    return [float(part.strip()) for part in value.split(",") if part.strip()]


def _allocate_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def _read_pid(path: Path) -> dict[str, float]:
    values: dict[str, float] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.split("#", 1)[0].strip()
        if not stripped or "=" not in stripped:
            continue
        key, raw_value = (part.strip() for part in stripped.split("=", 1))
        if key in PID_KEYS:
            values[key] = float(raw_value)
    missing = [key for key in PID_KEYS if key not in values]
    if missing:
        raise ValueError(f"{path} is missing PID keys: {', '.join(missing)}")
    return values


def _write_pid(path: Path, values: dict[str, float]) -> None:
    lines = [
        "# Balancer Bot PID Configuration",
        "# Generated by tools/sweep_pid.py",
        "",
        "# --- Rate Controller (Inner Loop) ---",
        f"rate_P               = {values['rate_P']:.12g}",
        f"rate_I               = {values['rate_I']:.12g}",
        f"rate_D               = {values['rate_D']:.12g}",
        f"rate_I_lim           = {values['rate_I_lim']:.12g}",
        f"rate_FF              = {values['rate_FF']:.12g}",
        "",
        "# --- Outer Loop ---",
        f"vel_P                = {values['vel_P']:.12g}",
        f"lean_trim_I          = {values['lean_trim_I']:.12g}",
        f"lean_trim_max_deg    = {values['lean_trim_max_deg']:.12g}",
        f"pitch_P              = {values['pitch_P']:.12g}",
        f"pitch_D              = {values['pitch_D']:.12g}",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def _start_simulator(binary: Path, port: int) -> subprocess.Popen:
    proc = subprocess.Popen(
        [str(binary), "--port", str(port)],
        cwd=REPO_ROOT,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )
    deadline = time.monotonic() + 0.25
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            raise RuntimeError(f"balancer_simulator exited during startup (rc={proc.returncode})")
        time.sleep(0.005)
    return proc


def _stop_process(proc: subprocess.Popen) -> None:
    if proc.poll() is not None:
        return
    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        proc.wait(timeout=5)


def _metric(summary: dict, key: str) -> float:
    value = summary.get(key)
    if value is None:
        return 0.0
    return abs(float(value))


def _score_components(summaries: list[dict], done_reasons: list[int], *, max_tail_rail: float, max_hold_bias_velocity: float) -> tuple[float, float, float, float, str]:
    reject_reasons: list[str] = []
    for index, (summary, done_reason) in enumerate(zip(summaries, done_reasons)):
        scenario_name = str(summary.get("scenario", {}).get("scenario_name", f"scenario_{index}"))
        if done_reason != DONE_COMPLETED:
            reject_reasons.append(f"{scenario_name}:done={done_reason}")
        if summary.get("fell"):
            reject_reasons.append(f"{scenario_name}:fell")
        rail_fraction = float(summary.get("tail_rail_fraction") or 0.0)
        command_rail_fraction = float(summary.get("tail_command_rail_fraction") or 0.0)
        if rail_fraction > max_tail_rail or command_rail_fraction > max_tail_rail:
            reject_reasons.append(f"{scenario_name}:rail")
        if "hold_bias" in scenario_name:
            tail_velocity = float(summary.get("tail_mean_abs_velocity_mps") or 0.0)
            if tail_velocity > max_hold_bias_velocity:
                reject_reasons.append(f"{scenario_name}:velocity")

    rms_pitch = sum(_metric(summary, "tail_rms_pitch_deg") for summary in summaries)
    velocity = sum(_metric(summary, "tail_mean_abs_velocity_mps") for summary in summaries)
    effort = sum(_metric(summary, "max_abs_u_sps") for summary in summaries)
    authority = 0.0
    for summary in summaries:
        scenario_name = str(summary.get("scenario", {}).get("scenario_name", ""))
        if "authority" in scenario_name:
            authority += _metric(summary, "tail_rms_pitch_deg") * 0.5
            authority += _metric(summary, "tail_command_rail_fraction") * 25.0
            authority += _metric(summary, "tail_mean_abs_velocity_mps") * 2.0
    reject_reason = ";".join(reject_reasons)
    if reject_reason:
        return (1.0e9, velocity, effort, authority, reject_reason)
    return (rms_pitch + authority, velocity, effort, authority, "")


def _candidate_grid(base: dict[str, float], args: argparse.Namespace) -> list[dict[str, float]]:
    axes = {
        "rate_P": _float_list(args.rate_p),
        "rate_I": _float_list(args.rate_i),
        "rate_D": _float_list(args.rate_d),
        "vel_P": _float_list(args.vel_p),
        "lean_trim_I": _float_list(args.lean_trim_i),
        "lean_trim_max_deg": _float_list(args.lean_trim_max_deg),
        "pitch_P": _float_list(args.pitch_p),
        "pitch_D": _float_list(args.pitch_d),
    }
    candidates = []
    for values in itertools.product(*(axes[key] for key in axes)):
        candidate = dict(base)
        candidate.update(dict(zip(axes.keys(), values)))
        candidates.append(candidate)
    return candidates


def _row_for(
    index: int,
    pid_path: Path,
    values: dict[str, float],
    scenario_results: list[tuple[str, dict, int]],
    *,
    max_tail_rail: float,
    max_hold_bias_velocity: float,
) -> dict:
    summaries = [summary for _name, summary, _done_reason in scenario_results]
    done_reasons = [done_reason for _name, _summary, done_reason in scenario_results]
    score_rms, score_velocity, score_effort, score_authority, reject_reason = _score_components(
        summaries,
        done_reasons,
        max_tail_rail=max_tail_rail,
        max_hold_bias_velocity=max_hold_bias_velocity,
    )
    row = {key: values[key] for key in PID_KEYS}
    row.update(
        {
            "rank": 0,
            "index": index,
            "pid_config": str(pid_path),
            "score": score_rms,
            "score_velocity": score_velocity,
            "score_effort": score_effort,
            "score_authority": score_authority,
            "reject_reason": reject_reason,
        }
    )
    for scenario_name, summary, done_reason in scenario_results:
        prefix = scenario_name.replace("-", "_")
        row.update(
            {
                f"{prefix}_done_reason": done_reason,
                f"{prefix}_fell": bool(summary.get("fell")),
                f"{prefix}_final_position_m": summary.get("final_position_m"),
                f"{prefix}_max_abs_position_m": summary.get("max_abs_position_m"),
                f"{prefix}_tail_mean_abs_velocity_mps": summary.get("tail_mean_abs_velocity_mps"),
                f"{prefix}_tail_mean_abs_pitch_deg": summary.get("tail_mean_abs_pitch_deg"),
                f"{prefix}_tail_rms_pitch_deg": summary.get("tail_rms_pitch_deg"),
                f"{prefix}_max_abs_pitch_deg": summary.get("max_abs_pitch_deg"),
                f"{prefix}_max_abs_u_sps": summary.get("max_abs_u_sps"),
                f"{prefix}_tail_rail_fraction": summary.get("tail_rail_fraction"),
                f"{prefix}_tail_command_rail_fraction": summary.get("tail_command_rail_fraction"),
            }
        )
    return row


def _resolve_scenarios(names: list[str]) -> list[str]:
    resolved: list[str] = []
    for name in names:
        if name in SCENARIO_GROUPS:
            for scenario_name in SCENARIO_GROUPS[name]:
                if scenario_name not in resolved:
                    resolved.append(scenario_name)
        elif name in SCENARIOS:
            if name not in resolved:
                resolved.append(name)
        else:
            choices = ", ".join(sorted([*SCENARIOS, *SCENARIO_GROUPS]))
            raise ValueError(f"Unknown scenario or group {name!r}. Expected one of: {choices}")
    return resolved


def main() -> int:
    parser = argparse.ArgumentParser(description="Sweep simulator PID values and rank smooth realistic scenarios.")
    parser.add_argument("--sim-bin", type=Path, default=REPO_ROOT / "build" / "balancer_simulator")
    parser.add_argument("--base-pid", type=Path, default=REPO_ROOT / "pid_sim.conf")
    parser.add_argument("--output-dir", type=Path, default=REPO_ROOT / "build" / "sim_sweeps" / time.strftime("%Y%m%d_%H%M%S"))
    parser.add_argument("--limit", type=int, default=0, help="Optional cap on candidates, useful for smoke runs.")
    parser.add_argument("--run-id-base", type=int, default=7000)
    parser.add_argument(
        "--scenario",
        action="append",
        default=[],
        help="Scenario name or group to run. May be repeated. Groups: smooth-first, validation, all-realistic.",
    )
    parser.add_argument("--top-k", type=int, default=50, help="Number of ranked candidates to save to top JSON.")
    parser.add_argument("--max-tail-rail", type=float, default=0.05)
    parser.add_argument("--max-hold-bias-velocity", type=float, default=0.5)
    parser.add_argument("--done-timeout", type=float, default=20.0)
    parser.add_argument("--rate-p", default="0.10,0.15,0.20,0.25,0.30,0.35")
    parser.add_argument("--rate-i", default="0.0")
    parser.add_argument("--rate-d", default="0.00,0.01,0.02,0.03,0.04,0.05,0.06,0.08")
    parser.add_argument("--vel-p", default="0.00005,0.00010,0.00020,0.00035,0.00050,0.00070,0.00090")
    parser.add_argument("--lean-trim-i", default="0.05,0.15,0.30,0.60")
    parser.add_argument("--lean-trim-max-deg", default="2.0,4.0,6.0")
    parser.add_argument("--pitch-p", default="4.0,5.0,6.0,8.0,10.0,12.0")
    parser.add_argument("--pitch-d", default="0.20,0.35,0.50,0.65,0.80")
    args = parser.parse_args()

    if not args.sim_bin.exists():
        raise SystemExit(f"Simulator binary not found at {args.sim_bin}; run pytest --build first.")

    scenario_names = _resolve_scenarios(args.scenario or ["smooth-first"])
    base = _read_pid(args.base_pid)
    candidates = _candidate_grid(base, args)
    if args.limit > 0:
        candidates = candidates[: args.limit]

    args.output_dir.mkdir(parents=True, exist_ok=True)
    pid_dir = args.output_dir / "pid"
    run_dir = args.output_dir / "runs"
    pid_dir.mkdir(parents=True, exist_ok=True)
    run_dir.mkdir(parents=True, exist_ok=True)

    port = _allocate_udp_port()
    proc = _start_simulator(args.sim_bin, port)
    rows: list[dict] = []
    try:
        with UdpClient(bridge_port=port) as udp:
            udp.register()
            udp.drain()
            for index, values in enumerate(candidates):
                pid_path = pid_dir / f"candidate_{index:04d}.conf"
                _write_pid(pid_path, values)
                scenario_results: list[tuple[str, dict, int]] = []
                for scenario_offset, scenario_name in enumerate(scenario_names):
                    output_dir = run_dir / f"candidate_{index:04d}" / scenario_name
                    summary, _metadata, done = run_scenario_live(
                        udp,
                        run_id=args.run_id_base + (index * max(1, len(scenario_names))) + scenario_offset,
                        output_dir=output_dir,
                        physics_profile=PHYSICS_REALISTIC,
                        pid_config_path=str(pid_path),
                        done_timeout=args.done_timeout,
                        **SCENARIOS[scenario_name],
                    )
                    scenario_results.append((scenario_name, summary, done.reason_code))
                row = _row_for(
                    index,
                    pid_path,
                    values,
                    scenario_results,
                    max_tail_rail=args.max_tail_rail,
                    max_hold_bias_velocity=args.max_hold_bias_velocity,
                )
                rows.append(row)
                print(
                    f"{index + 1:4d}/{len(candidates)} "
                    f"score={row['score']:.4g} vel={row['score_velocity']:.4g} "
                    f"u={row['score_effort']:.4g} auth={row['score_authority']:.4g} "
                    f"reject={row['reject_reason'] or '-'}",
                    flush=True,
                )
    finally:
        _stop_process(proc)

    rows.sort(key=lambda row: (row["score"], row["score_velocity"], row["score_effort"]))
    for rank, row in enumerate(rows, start=1):
        row["rank"] = rank

    fieldnames = list(rows[0].keys()) if rows else []
    with (args.output_dir / "results.csv").open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    (args.output_dir / f"top{args.top_k}.json").write_text(
        json.dumps(rows[: args.top_k], indent=2),
        encoding="utf-8",
    )
    (args.output_dir / "scenarios.json").write_text(
        json.dumps({name: SCENARIOS[name] for name in scenario_names}, indent=2),
        encoding="utf-8",
    )

    print(f"\nWrote {args.output_dir / 'results.csv'}")
    print(json.dumps(rows[: min(5, len(rows))], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
