from __future__ import annotations

import os
import json
import shutil
import select
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

_REPO_ROOT = Path(__file__).parents[2]
_BUILD_DIR = _REPO_ROOT / "build"
_DEFAULT_BIN = _BUILD_DIR / "sil_app"
_DEFAULT_SIM_BIN = _BUILD_DIR / "balancer_simulator"
_DEFAULT_SIM_PORT = 9001

if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import pytest

from udp_client import UdpClient
from tests.python.support.behavioral_diagnostics import artifact_metrics


# These are model-specific behavioral gaps, not a blanket exemption for the
# electrical plant.  Keep the reasons next to the collection hook so strict
# xfails remain visible in the pytest output and become failures when the
# corresponding behavior is recovered.
_BEHAVIORAL_XFAILS = (
    (
        "test_cold_start_50deg_estimator_limited_is_tracked_against_70_degree_boundary[direct_actuator-3500-",
        "DirectActuator retained 50 degree estimator-limited cold start crosses the 70 degree fail-fast boundary before the estimate catches up",
    ),
    (
        "test_cold_start_50deg_estimator_limited_is_tracked_against_70_degree_boundary[stepper_phase_electrical-3500-",
        "StepperPhaseElectrical retained 50 degree estimator-limited cold start crosses the 70 degree fail-fast boundary before the estimate catches up",
    ),
    (
        "test_simple_behavioral_scenarios[stepper_phase_electrical-2001-",
        "StepperPhaseElectrical noisy push reaches the current electrical authority boundary before recovery",
    ),
    (
        "test_pitch_authority_nominal_uncertainty_matrix_stays_within_reference_envelope[stepper_phase_electrical]",
        "StepperPhaseElectrical direct authority under plant uncertainty reaches the current phase/safety boundary",
    ),
    (
        "test_outer_transient_authority_saturation_recovers_without_trim_growth[stepper_phase_electrical]",
        "StepperPhaseElectrical 7000-SPS initial-velocity recovery remains outside the sustained authority envelope",
    ),
)


def pytest_collection_modifyitems(config, items):
    del config
    for item in items:
        for node_fragment, reason in _BEHAVIORAL_XFAILS:
            if node_fragment in item.nodeid:
                item.add_marker(pytest.mark.xfail(strict=True, reason=reason))
                break


_BEHAVIORAL_REPORTS = {}


def pytest_sessionstart(session):
    """Start each scenario run with only its own generated model artifacts."""
    if hasattr(session.config, "workerinput"):
        return
    root = _BUILD_DIR / "sim"
    candidates = [root / "direct_actuator", root / "stepper_phase_electrical"]
    candidates.extend(
        worker_root / model
        for worker_root in root.glob("gw*")
        for model in ("direct_actuator", "stepper_phase_electrical")
    )
    for candidate in candidates:
        if candidate.is_dir():
            shutil.rmtree(candidate)
    for name in ("simulator_behavioral_matrix.json", "simulator_behavioral_matrix.md"):
        (root / name).unlink(missing_ok=True)


def pytest_runtest_logreport(report):
    """Collect model-parameterized scenario outcomes for the result matrix."""
    if report.when != "call" or "tests/python/test_sim_scenarios.py::" not in report.nodeid:
        return
    if "[direct_actuator" in report.nodeid:
        model = "DirectActuator"
    elif "[stepper_phase_electrical" in report.nodeid:
        model = "StepperPhaseElectrical"
    else:
        return
    was_xfail = getattr(report, "wasxfail", None)
    if was_xfail:
        status = "xfail" if report.outcome == "skipped" else "unexpected_pass"
    elif report.outcome == "passed":
        status = "pass"
    elif report.outcome == "skipped":
        status = "skip"
    else:
        status = "unexpected_failure"
    _BEHAVIORAL_REPORTS[report.nodeid] = {
        "nodeid": report.nodeid,
        "test": report.nodeid.split("::")[-1].split("[", 1)[0],
        "model": model,
        "status": status,
        "xfail_reason": was_xfail or None,
    }


def _matrix_category(test_name: str) -> str:
    for token, category in (
        ("attitude_recovery", "attitude_recovery"),
        ("simple_behavioral", "quiet_balance_and_disturbance"),
        ("hardware_inspired", "hardware_stress"),
        ("full_forward", "drive_stop"),
        ("pitch_authority", "pitch_authority_diagnostics"),
        ("com_acquisition", "com_acquisition_and_maintenance"),
        ("velocity_recovery", "velocity_recovery"),
        ("initial_velocity", "velocity_recovery"),
        ("startup", "startup_recovery"),
        ("estimator", "velocity_estimator_error"),
        ("transient_authority", "authority_and_saturation"),
        ("gain_authority", "outer_gain_region"),
        ("drive_stop", "drive_stop"),
        ("reduced_translation", "authority_and_saturation"),
        ("noise_and_correlated", "uncertainty"),
        ("ten_minute", "long_horizon"),
        ("cold_start", "high_angle_boundary"),
    ):
        if token in test_name:
            return category
    return "behavioral"


def _matrix_artifact_prefixes(test_name: str, scenario_name: str) -> tuple[str, ...]:
    if test_name == "test_simple_behavioral_scenarios":
        return (scenario_name,)
    if test_name == "test_attitude_recovery_common_behavior":
        return (scenario_name.removeprefix("attitude_recovery_"),)
    if test_name == "test_hardware_inspired_stress_scenarios":
        return (scenario_name,)
    if test_name == "test_full_forward_then_stop_moves_and_settles":
        return ("full_forward_then_stop",)
    prefix_map = {
        "test_outer_velocity_recovery_envelope_is_signed_and_authority_aware": ("outer_live_recovery_",),
        "test_outer_initial_velocity_recovery_envelope_is_signed": ("outer_live_initial_",),
        "test_outer_hardware_startup_recovery_is_an_authority_audit_regression": ("outer_live_hardware_startup_recovery",),
        "test_outer_startup_combines_pitch_velocity_and_com_errors": ("outer_live_startup_combined_",),
        "test_outer_velocity_estimator_bias_scale_and_latency_remain_bounded": ("outer_live_estimator_",),
        "test_outer_transient_authority_saturation_recovers_without_trim_growth": ("outer_live_sustained_authority",),
        "test_outer_gain_authority_region_is_broad_and_symmetric": ("outer_live_region_",),
        "test_outer_drive_stop_and_reversal_are_symmetric": ("outer_live_reversal_",),
        "test_outer_com_acquisition_is_symmetric_over_useful_bias_range": (
            "outer_live_com_2_",
            "outer_live_com_8_",
            "outer_live_com_20_",
        ),
        "test_outer_com_acquisition_pauses_through_motion_and_maintenance_reacquires": ("outer_live_com_interruptions", "outer_live_com_maintenance"),
        "test_outer_reduced_translation_authority_degrades_without_trim_runaway": ("outer_live_authority_",),
        "test_outer_noise_and_correlated_mass_uncertainty_remain_bounded": ("outer_live_mass_", "outer_live_noise_long"),
        "test_outer_ten_minute_event_run_has_no_growing_late_envelope": ("outer_live_long_events_600s",),
        "test_pitch_authority_direct_target_sweep_is_end_to_end_and_isolated": ("pitch_authority_direct_sweep",),
        "test_pitch_authority_watchdog_expires_on_refresh_dropout": ("pitch_authority_watchdog_dropout",),
        "test_pitch_authority_first_stage_requires_zero_start_and_survives_repeated_pulses": ("pitch_authority_first_pm1_repeated",),
        "test_pitch_authority_long_holds_and_reversals_have_event_metrics": ("pitch_authority_long_train",),
        "test_pitch_authority_nominal_uncertainty_matrix_stays_within_reference_envelope": ("pitch_authority_uncertainty_",),
        "test_pitch_authority_direct_targets_cover_initial_condition_variation": ("pitch_authority_initial_",),
        "test_cold_start_50deg_estimator_limited_is_tracked_against_70_degree_boundary": ("cold_start_50deg_estimator_limited",),
    }
    return prefix_map.get(test_name, ())


def _matrix_xfail_group(row: dict) -> str:
    """Normalize test-level reasons into useful behavioral cause groups."""
    category = str(row.get("category", "behavioral"))
    reason = str(row.get("xfail_reason", "")).lower()
    if "cold start" in reason or category == "high_angle_boundary":
        return "high_angle_boundary"
    if category == "pitch_authority_diagnostics" or "authority/uncertainty" in reason:
        return "attitude_recovery_frontier"
    if category == "long_horizon" or "long-horizon" in reason:
        return "long_horizon_instability"
    if category == "uncertainty" or "noise" in reason:
        return "uncertainty_and_noise"
    if category == "com_acquisition_and_maintenance" or (
        category == "startup_recovery" and "com " in reason
    ):
        return "com_behavior"
    if category in {
        "velocity_recovery",
        "startup_recovery",
        "velocity_estimator_error",
        "outer_gain_region",
        "drive_stop",
    } or any(token in reason for token in ("outer-loop", "drive/stop", "drive/reversal", "velocity-loop")):
        return "outer_loop_tuning"
    if category == "authority_and_saturation" or "authority" in reason:
        return "command_authority"
    if any(
        token in reason
        for token in ("com ", "com acquisition", "com interruption", "com trust")
    ):
        return "com_behavior"
    return category


def _matrix_write_artifacts() -> None:
    root = _BUILD_DIR / "sim"
    artifacts = []
    for metadata_path in sorted(root.rglob("metadata.json")):
        output_dir = metadata_path.parent
        summary_path = output_dir / "summary.json"
        done_path = output_dir / "done.json"
        if not summary_path.exists() or not done_path.exists():
            continue
        try:
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            if metadata.get("model") not in ("DirectActuator", "StepperPhaseElectrical"):
                continue
            diagnostic_path = output_dir / "behavioral_diagnostics.json"
            diagnostics = {}
            if diagnostic_path.exists():
                diagnostics = json.loads(diagnostic_path.read_text(encoding="utf-8"))
            artifacts.append(
                {
                    "path": output_dir,
                    "metadata": metadata,
                    "summary": json.loads(summary_path.read_text(encoding="utf-8")),
                    "done": json.loads(done_path.read_text(encoding="utf-8")),
                    "diagnostics": diagnostics,
                }
            )
        except (OSError, json.JSONDecodeError):
            continue

    rows = []
    used_reports = set()
    row_keys = set()

    def append_artifact_row(report, artifact, diagnostic=None, *, subrun=None):
        metadata = artifact["metadata"]
        summary = artifact["summary"]
        done = artifact["done"]
        pid_config = metadata.get("pid_profile")
        if pid_config:
            try:
                pid_config = str(Path(pid_config).resolve().relative_to(_REPO_ROOT))
            except ValueError:
                pid_config = str(pid_config)
        scenario = metadata.get("scenario_id", metadata.get("scenario_name"))
        subrun_value = (
            subrun
            if subrun is not None
            else metadata.get("subrun_id", metadata.get("scenario_name"))
        )
        key = (report["nodeid"], scenario, subrun_value, diagnostic and diagnostic.get("classification"))
        if key in row_keys:
            return
        row_keys.add(key)
        diagnostic = diagnostic or {}
        rows.append(
            {
                "scenario": scenario,
                "subrun": subrun_value,
                "intent": metadata.get("scenario_intent", ""),
                "category": metadata.get(
                    "scenario_category", _matrix_category(report["test"])
                ),
                "model": report["model"],
                "pid_config": pid_config,
                "status": report["status"],
                "subrun_status": diagnostic.get("status", "not_recorded"),
                "classification": diagnostic.get("classification", "unclassified"),
                "failed_gates": diagnostic.get("failures", []),
                "xfail_reason": report["xfail_reason"],
                "pytest_nodeid": report["nodeid"],
                "metrics": artifact_metrics(summary, done),
                "artifact": str(artifact["path"].relative_to(_REPO_ROOT)),
            }
        )

    for report in _BEHAVIORAL_REPORTS.values():
        test_name = report["test"]
        model = report["model"]
        matching = []
        for artifact in artifacts:
            if artifact["metadata"].get("model") != model:
                continue
            scenario_name = str(artifact["metadata"].get("scenario_name", ""))
            prefixes = _matrix_artifact_prefixes(test_name, scenario_name)
            if not prefixes:
                continue
            if test_name in {
                "test_simple_behavioral_scenarios",
                "test_attitude_recovery_common_behavior",
                "test_hardware_inspired_stress_scenarios",
            }:
                if not any(prefix in report["nodeid"] for prefix in prefixes):
                    continue
            elif not any(scenario_name.startswith(prefix) for prefix in prefixes):
                continue
            matching.append(artifact)

        if not matching:
            rows.append(
                {
                    "scenario": report["nodeid"],
                    "subrun": None,
                    "intent": "",
                    "category": _matrix_category(test_name),
                    "model": model,
                    "pid_config": "tests/data/direct_actuator_pid.conf"
                    if model == "DirectActuator"
                    else "pid.conf",
                    "status": report["status"],
                    "subrun_status": "not_recorded",
                    "classification": "infrastructure_failure",
                    "failed_gates": [],
                    "xfail_reason": report["xfail_reason"],
                    "metrics": {},
                }
            )
            continue

        used_reports.add(report["nodeid"])
        for artifact in matching:
            diagnostic_records = artifact["diagnostics"].get("subruns", [])
            artifact_subrun = artifact["metadata"].get(
                "subrun_id", artifact["metadata"].get("scenario_name")
            )
            matching_diagnostic = next(
                (
                    record
                    for record in reversed(diagnostic_records)
                    if record.get("subrun_id") == artifact_subrun
                ),
                None,
            )
            append_artifact_row(
                report, artifact, matching_diagnostic, subrun=artifact_subrun
            )

        # Composite diagnostics are written incrementally so every concrete
        # artifact is self-describing.  Aggregate/model-specific records are
        # attached once, using the artifact with the most complete diagnostic
        # history, rather than multiplying partial snapshots in the matrix.
        complete_artifact = max(
            matching,
            key=lambda artifact: len(artifact["diagnostics"].get("subruns", [])),
        )
        extra_records = complete_artifact["diagnostics"].get("subruns", [])
        for diagnostic in extra_records:
            subrun_id = diagnostic.get("subrun_id")
            if subrun_id in {
                complete_artifact["metadata"].get(
                    "subrun_id", complete_artifact["metadata"].get("scenario_name")
                )
            }:
                continue
            if subrun_id not in ("aggregate", "model_specific_hardware_envelope"):
                continue
            append_artifact_row(report, complete_artifact, diagnostic, subrun=subrun_id)

    # Keep report-only rows for parameterized tests whose simulator run failed
    # before it produced a complete artifact.
    for report in _BEHAVIORAL_REPORTS.values():
        if report["nodeid"] not in used_reports:
            rows.append(
                {
                    "scenario": report["nodeid"],
                    "subrun": None,
                    "intent": "",
                    "category": _matrix_category(report["test"]),
                    "model": report["model"],
                    "pid_config": "tests/data/direct_actuator_pid.conf"
                    if report["model"] == "DirectActuator"
                    else "pid.conf",
                    "status": report["status"],
                    "subrun_status": "not_recorded",
                    "classification": "infrastructure_failure",
                    "failed_gates": [],
                    "xfail_reason": report["xfail_reason"],
                    "metrics": {},
                }
            )

    for row in rows:
        row["xfail_group"] = (
            _matrix_xfail_group(row) if row["status"] == "xfail" else None
        )
    rows.sort(key=lambda row: (row["model"], row["category"], row["scenario"]))
    execution_counts = {}
    paired_statuses = {}
    for report in _BEHAVIORAL_REPORTS.values():
        execution_counts.setdefault(report["model"], {})
        status_counts = execution_counts[report["model"]]
        status_counts[report["status"]] = status_counts.get(report["status"], 0) + 1
        normalized = report["nodeid"].replace("[direct_actuator", "[").replace(
            "[stepper_phase_electrical", "["
        )
        paired_statuses.setdefault(normalized, {})[report["model"]] = report["status"]
    shared_success = sum(
        statuses.get("DirectActuator") == "pass"
        and statuses.get("StepperPhaseElectrical") == "pass"
        for statuses in paired_statuses.values()
    )
    shared_success_scenarios = sorted(
        normalized
        for normalized, statuses in paired_statuses.items()
        if statuses.get("DirectActuator") == "pass"
        and statuses.get("StepperPhaseElectrical") == "pass"
    )
    subrun_counts = {}
    xfail_groups = {}
    for row in rows:
        model_counts = subrun_counts.setdefault(row["model"], {})
        subrun_status = row.get("subrun_status", "not_recorded")
        model_counts[subrun_status] = model_counts.get(subrun_status, 0) + 1
        if (
            row["status"] == "xfail"
            and row.get("subrun") != "aggregate"
            and row.get("subrun_status") not in (
                "pass",
                "diagnostic",
            )
        ):
            group = row.get("xfail_group") or "unclassified"
            groups = xfail_groups.setdefault(group, [])
            identity = f"{row['model']}:{row['scenario']}:{row.get('subrun')}"
            if identity not in groups:
                groups.append(identity)
    matrix = {
        "source": "tests/python/test_sim_scenarios.py",
        "schema_version": 2,
        "unique_scenario_definitions": len(
            {
                report["nodeid"].replace("[direct_actuator", "[").replace(
                    "[stepper_phase_electrical", "["
                )
                for report in _BEHAVIORAL_REPORTS.values()
            }
        ),
        "model_scenario_executions": len(_BEHAVIORAL_REPORTS),
        "concrete_simulation_runs": len(rows),
        "execution_counts": execution_counts,
        "subrun_counts": subrun_counts,
        "shared_success_executions": shared_success,
        "shared_success_scenarios": shared_success_scenarios,
        "xfail_groups": xfail_groups,
        "rows": rows,
    }
    json_path = root / "simulator_behavioral_matrix.json"
    json_path.write_text(json.dumps(matrix, indent=2), encoding="utf-8")
    lines = [
        "# Python simulator behavioral matrix",
        "",
        "Source: `tests/python/test_sim_scenarios.py`.",
        "",
        f"Unique scenario definitions: {matrix['unique_scenario_definitions']}",
        f"Model/scenario pytest executions: {matrix['model_scenario_executions']}",
        f"Concrete simulator runs represented: {matrix['concrete_simulation_runs']}",
        "",
        "| Model | Pass | Strict xfail | Unexpected failure | Unexpected pass |",
        "|---|---:|---:|---:|---:|",
    ]
    for model in ("DirectActuator", "StepperPhaseElectrical"):
        model_counts = execution_counts.get(model, {})
        lines.append(
            f"| {model} | {model_counts.get('pass', 0)} | "
            f"{model_counts.get('xfail', 0)} | "
            f"{model_counts.get('unexpected_failure', 0)} | "
            f"{model_counts.get('unexpected_pass', 0)} |"
        )
    lines += [
        "",
        f"Shared successful model/scenario executions: {shared_success}",
        "",
        "Shared successful scenarios:",
    ]
    lines += [f"- `{scenario}`" for scenario in shared_success_scenarios]
    lines += [
        "",
        "Subrun status counts:",
    ]
    for model in ("DirectActuator", "StepperPhaseElectrical"):
        counts = subrun_counts.get(model, {})
        lines.append(f"- `{model}`: {counts}")
    lines += [
        "",
        "Strict-xfail diagnostic groups:",
    ]
    for group, identities in sorted(xfail_groups.items()):
        lines.append(f"- `{group}`: {len(identities)} subruns")
    lines += [
        "",
        "| Scenario | Subrun | Category | Model | Aggregate | Subrun | Classification | Xfail group | Peak pitch | Peak rate | Late pitch RMS | Late rate RMS | Peak applied SPS |",
        "|---|---|---|---|---|---|---|---|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        metrics = row.get("metrics", {})
        lines.append(
            "| {scenario} | {subrun} | {category} | {model} | {status} | {subrun_status} | {classification} | {xfail_group} | {pitch} | {rate} | {tail} | {tail_rate} | {sps} |".format(
                scenario=row["scenario"],
                subrun=row.get("subrun", ""),
                category=row["category"],
                model=row["model"],
                status=row["status"],
                subrun_status=row.get("subrun_status", ""),
                classification=row.get("classification", ""),
                xfail_group=row.get("xfail_group", ""),
                pitch=metrics.get("max_abs_pitch_deg", ""),
                rate=metrics.get("peak_pitch_rate_dps", ""),
                tail=metrics.get("tail_rms_pitch_deg", ""),
                tail_rate=metrics.get("tail_rms_pitch_rate_dps", ""),
                sps=metrics.get("peak_applied_sps", ""),
            )
        )
    (root / "simulator_behavioral_matrix.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def pytest_sessionfinish(session, exitstatus):
    del exitstatus
    if hasattr(session.config, "workerinput"):
        return
    _matrix_write_artifacts()


def _sil_binary() -> Path:
    env_override = os.environ.get("SIL_APP")
    path = Path(env_override) if env_override else _DEFAULT_BIN
    if not path.exists():
        pytest.fail(
            f"sil_app binary not found at {path}.\n"
            "Run: pytest --build or cmake -S . -B build && cmake --build build"
        )
    return path


def _sim_binary() -> Path:
    env_override = os.environ.get("BALANCER_SIM_BIN")
    path = Path(env_override) if env_override else _DEFAULT_SIM_BIN
    if not path.exists():
        pytest.fail(
            f"balancer_simulator binary not found at {path}.\n"
            "Run: pytest --build or cmake -S . -B build && cmake --build build"
        )
    return path


def _sim_port() -> int:
    return int(os.environ.get("BALANCER_SIM_PORT", str(_DEFAULT_SIM_PORT)))


def _allocate_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def _worker_udp_port() -> int:
    """Give each xdist worker an isolated SIL port; production remains 9000."""
    worker_id = os.environ.get("PYTEST_XDIST_WORKER")
    if not worker_id:
        return 9000
    try:
        worker_index = int(worker_id.removeprefix("gw"))
    except ValueError:
        worker_index = 0
    return 9000 + worker_index


@pytest.fixture(scope="session")
def sil_port():
    return _worker_udp_port()


@pytest.fixture(scope="session")
def sil_process(sil_port):
    proc = _start_sil_process(sil_port)
    yield proc
    _stop_sil_process(proc)


@pytest.fixture(scope="function")
def udp(sil_process, sil_port):
    if sil_process.poll() is not None:
        pytest.fail(f"sil_app is not running (rc={sil_process.returncode})")

    with UdpClient(bridge_port=sil_port) as client:
        client.register()
        client.drain()
        yield client


def _start_sil_process(port: int = 9000):
    proc = subprocess.Popen(
        [str(_sil_binary()), "--port", str(port)],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        bufsize=0,
        preexec_fn=os.setsid,
    )

    deadline = time.monotonic() + 5.0
    startup_output = bytearray()
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            output = startup_output.decode(errors="replace")
            pytest.fail(f"sil_app exited during startup (rc={proc.returncode})\n{output}")

        ready, _, _ = select.select([proc.stdout], [], [], 0.1)
        if not ready:
            continue

        chunk = os.read(proc.stdout.fileno(), 4096)
        startup_output.extend(chunk)
        if b"UDP Bridge listening" in startup_output:
            return proc

    _stop_sil_process(proc)
    output = startup_output.decode(errors="replace")
    pytest.fail(f"sil_app did not become ready within 5 seconds\n{output}")


def _stop_sil_process(proc):
    if proc.poll() is None:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            proc.wait(timeout=5)


@pytest.fixture(scope="function")
def fresh_udp(sil_port):
    proc = _start_sil_process(sil_port)
    try:
        with UdpClient(bridge_port=sil_port) as client:
            client.register()
            client.drain()
            yield client
    finally:
        _stop_sil_process(proc)


@pytest.fixture(scope="session")
def simulator_binary() -> Path:
    return _sim_binary()


@pytest.fixture(scope="function")
def simulator_port():
    env_override = os.environ.get("BALANCER_SIM_PORT")
    if env_override is not None:
        return int(env_override)
    return _allocate_udp_port()


@pytest.fixture(scope="function")
def simulator_process(simulator_port):
    proc = subprocess.Popen(
        [str(_sim_binary()), "--port", str(simulator_port)],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        bufsize=0,
        preexec_fn=os.setsid,
    )

    deadline = time.monotonic() + 5.0
    startup_output = bytearray()
    marker = b"Starting balancer_simulator service on UDP port"
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            output = startup_output.decode(errors="replace")
            pytest.fail(
                f"balancer_simulator exited during startup (rc={proc.returncode})\n{output}"
            )

        ready, _, _ = select.select([proc.stdout], [], [], 0.1)
        if not ready:
            continue
        chunk = os.read(proc.stdout.fileno(), 4096)
        startup_output.extend(chunk)
        if marker in startup_output:
            break
    else:
        _stop_sil_process(proc)
        output = startup_output.decode(errors="replace")
        pytest.fail(f"balancer_simulator did not become ready within 5 seconds\n{output}")

    yield proc
    _stop_sil_process(proc)


@pytest.fixture(scope="function")
def simulator_udp(simulator_process, simulator_port):
    if simulator_process.poll() is not None:
        pytest.fail(f"balancer_simulator is not running (rc={simulator_process.returncode})")

    with UdpClient(bridge_port=simulator_port) as client:
        client.register()
        client.drain()
        yield client


@pytest.fixture(scope="session")
def sim_artifact_settings():
    worker_id = os.environ.get("PYTEST_XDIST_WORKER")
    output_root = _BUILD_DIR / "sim" / worker_id if worker_id else _BUILD_DIR / "sim"
    output_root.mkdir(parents=True, exist_ok=True)
    return {
        "temp_root": output_root,
    }
