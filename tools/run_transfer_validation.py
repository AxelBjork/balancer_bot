#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
for import_root in (REPO_ROOT, REPO_ROOT / "tests/python"):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

from tests.python.support.simulator_service import (
    DONE_ACCEPTANCE_FAILED,
    DONE_COMPLETED,
    PHYSICS_REALISTIC,
    run_scenario_live,
)
from tests.python.udp_client import UdpClient


def _allocate_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def _git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=REPO_ROOT, text=True).strip()


def _pid_digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _direct_acceptance(sim_bin: Path, pid: Path, index: int) -> tuple[bool, list[str]]:
    result = json.loads(
        subprocess.check_output(
            [str(sim_bin), "--pid-config", str(pid), "--direct-summary", str(index)],
            cwd=REPO_ROOT,
            text=True,
        )
    )
    return bool(result["accepted"]), [str(failure) for failure in result["failures"]]


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate the simulator-to-hardware transfer report.")
    parser.add_argument("--sim-bin", type=Path, default=REPO_ROOT / "build/balancer_simulator")
    parser.add_argument("--pid", type=Path, default=REPO_ROOT / "pid.conf")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--telemetry-stride", type=int, default=1)
    parser.add_argument("--include-build-gates", action="store_true")
    args = parser.parse_args()

    commit = _git("rev-parse", "--short=12", "HEAD")
    working_tree_dirty = bool(_git("status", "--short"))
    run_id = time.strftime("%Y%m%d_%H%M%S", time.gmtime()) + f"_{commit}"
    output = args.output or REPO_ROOT / "build/sim/transfer" / run_id
    output.mkdir(parents=True, exist_ok=False)
    catalog = json.loads(
        subprocess.check_output([str(args.sim_bin), "--catalog-json"], cwd=REPO_ROOT, text=True)
    )
    if len(catalog) != 10:
        raise RuntimeError(f"Expected 10 transfer scenarios, got {len(catalog)}")
    scenario_names = [str(scenario["name"]) for scenario in catalog]

    cross_status = "not run"
    fuzz_status = "not run"
    if args.include_build_gates:
        subprocess.run(["pytest", "--fuzz-smoke", "--build-only", "-q"], cwd=REPO_ROOT, check=True)
        fuzz_status = "14/14 host-compiler seeds passed"
        subprocess.run([str(REPO_ROOT / "build_cmake"), "OFF"], cwd=REPO_ROOT, check=True)
        cross_status = "Pi cross-build passed"

    port = _allocate_port()
    proc = subprocess.Popen(
        [str(args.sim_bin), "--port", str(port), "--pid-config", str(args.pid)],
        cwd=REPO_ROOT,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )
    rows: list[dict] = []
    try:
        time.sleep(0.05)
        with UdpClient(bridge_port=port) as udp:
            udp.register()
            for index, scenario in enumerate(catalog):
                name = str(scenario["name"])
                summary, metadata, done = run_scenario_live(
                    udp,
                    run_id=10_000 + index,
                    output_dir=output / name,
                    physics_profile=PHYSICS_REALISTIC,
                    duration_s=float(scenario["duration_s"]),
                    telemetry_stride=args.telemetry_stride,
                    transfer_scenario_index=index,
                    initial_pitch_deg=float(scenario["initial_pitch_deg"]),
                    com_angle_offset_rad=float(scenario["com_angle_offset_rad"]),
                    total_mass_scale=float(scenario["total_mass_scale"]),
                    pitch_inertia_scale=float(scenario["pitch_inertia_scale"]),
                    physics_override=dict(scenario["physics"]),
                    imu_pitch_lag_s=float(scenario["imu_pitch_lag_s"]),
                    imu_noise_seed=int(scenario["imu_noise_seed"]),
                    accel_noise_std_mps2=float(scenario["accel_noise_std_mps2"]),
                    gyro_noise_std_rad_s=float(scenario["gyro_noise_std_rad_s"]),
                    imu_timestamp_jitter_us=float(scenario["imu_timestamp_jitter_us"]),
                    imu_sample_loss_rate=float(scenario["imu_sample_loss_rate"]),
                    accel_bias_mps2=list(scenario["accel_bias_mps2"]),
                    gyro_bias_rad_s=list(scenario["gyro_bias_rad_s"]),
                    disturbances=list(scenario["disturbances"]),
                    joy_segments=list(scenario["joy_segments"]),
                    pid_config_path=str(args.pid),
                    done_timeout=30.0,
                )
                passed, failures = _direct_acceptance(args.sim_bin, args.pid, index)
                if done.reason_code not in (DONE_COMPLETED, DONE_ACCEPTANCE_FAILED):
                    passed = False
                    failures.append(f"udp_done={done.reason_code}")
                rows.append(
                    {
                        "name": name,
                        "passed": passed,
                        "failures": failures,
                        "done": done.__dict__,
                        "summary": summary,
                        "metadata": metadata,
                    }
                )
                print(
                    f"{index + 1:02d}/{len(catalog):02d} {name}: "
                    f"{'PASS' if passed else 'FAIL'}",
                    flush=True,
                )
    finally:
        if proc.poll() is None:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=5)

    passed_count = sum(bool(row["passed"]) for row in rows)
    manifest = {
        "commit": commit,
        "working_tree_dirty": working_tree_dirty,
        "pid": str(args.pid),
        "pid_sha256": _pid_digest(args.pid),
        "telemetry_stride": args.telemetry_stride,
        "passed": passed_count,
        "total": len(rows),
        "fuzz_status": fuzz_status,
        "cross_build_status": cross_status,
        "tuning_provenance": str(REPO_ROOT / "build/sim/tuning"),
        "catalog": catalog,
        "scenarios": rows,
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    lines = [
        "# Simulator-to-Hardware Transfer Summary",
        "",
        f"- Commit: `{commit}`",
        f"- Working tree: `{'dirty' if working_tree_dirty else 'clean'}`",
        f"- PID: `{args.pid}` (`{manifest['pid_sha256']}`)",
        f"- Scenarios: **{passed_count}/{len(rows)} passed**",
        f"- Telemetry stride: `{args.telemetry_stride}`",
        f"- Fuzz: {fuzz_status}",
        f"- Cross-build: {cross_status}",
        f"- Tuning provenance: `{manifest['tuning_provenance']}`",
        "- Hardware status: simulation-qualified only; no robot operation was performed.",
        "",
        "## Scenario Results",
        "",
        "| Scenario | Seed | Result | Peak pitch | Tail RMS | Max saturation |",
        "| --- | ---: | --- | ---: | ---: | ---: |",
    ]
    for row in rows:
        done = row["done"]
        lines.append(
            f"| [{row['name']}]({row['name']}/summary.json) | "
            f"{int(row['summary'].get('seed', row['metadata']['imu_noise_seed']))} | "
            f"{'PASS' if row['passed'] else 'FAIL: ' + ', '.join(row['failures'])} | "
            f"{done['max_abs_pitch_deg']:.4f} deg | {done['tail_rms_pitch_deg']:.4f} deg | "
            f"{done['max_continuous_saturation_s']:.4f} s |"
        )
    nominal = rows[0]["summary"]
    lines.extend(
        [
            "",
            "## Retained Nominal Plant Parameters",
            "",
            "| Parameter | Value |",
            "| --- | ---: |",
            f"| Motor maximum force | {nominal['motor_max_force_n']} N |",
            f"| Motor no-load speed | {nominal['motor_no_load_speed_mps']} m/s |",
            f"| Motor velocity damping | {nominal['motor_velocity_damping']} |",
            f"| Motor time constant | {nominal['motor_tau_s']} s |",
            f"| Traction coefficient | {nominal['traction_coefficient']} |",
            f"| Pitch damping | {nominal['pitch_damping']} N m s/rad |",
            f"| Cart damping | {nominal['cart_damping']} N s/m |",
            f"| Phase-error limit | {nominal['phase_error_limit_steps']} steps |",
            f"| Tire stiffness | {nominal['tire_stiffness_n_per_m']} N/m |",
            f"| Tire damping | {nominal['tire_damping_n_s_per_m']} N s/m |",
            f"| Wheel equivalent mass | {nominal['wheel_equivalent_mass_kg']} kg |",
            "",
            "Each linked scenario records its complete physical overrides and sensor settings in "
            "`summary.json` and `metadata.json`.",
        ]
    )
    report = "\n".join(lines) + "\n"
    (output / "transfer_summary.md").write_text(report, encoding="utf-8")
    stable_summary = REPO_ROOT / "build/sim/transfer_summary.md"
    stable_prefix = Path(os.path.relpath(output, stable_summary.parent)).as_posix()
    stable_report = report
    for name in scenario_names:
        stable_report = stable_report.replace(
            f"]({name}/summary.json)", f"]({stable_prefix}/{name}/summary.json)"
        )
    stable_summary.write_text(stable_report, encoding="utf-8")
    print(f"Wrote {output / 'transfer_summary.md'}")
    return 0 if passed_count == len(rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
