"""Small, file-backed diagnostics for composite simulator scenarios.

The Python behavioral suite intentionally keeps some scenarios composite: a
single test may exercise both signs, several disturbance magnitudes, or an
uncertainty matrix.  This module records the result of each simulator run
without changing the aggregate pytest item semantics.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Callable


@dataclass
class SubrunDiagnostic:
    scenario_id: str
    subrun_id: str
    model: str
    status: str = "not_evaluated"
    classification: str = "unclassified"
    failures: list[dict[str, str]] = field(default_factory=list)
    metrics: dict[str, Any] = field(default_factory=dict)


class ScenarioDiagnostics:
    """Collect named checks while allowing later composite subruns to run."""

    def __init__(self, scenario_id: str, model: str):
        self.scenario_id = scenario_id
        self.model = model
        self.records: list[SubrunDiagnostic] = []

    def evaluate(
        self,
        subrun_id: str,
        checks: list[tuple[str, Callable[[], None]]],
        *,
        metrics: dict[str, Any] | None = None,
        classification: str = "unclassified",
    ) -> bool:
        failures: list[dict[str, str]] = []
        for gate, check in checks:
            try:
                check()
            except AssertionError as exc:
                failures.append(
                    {
                        "gate": gate,
                        "message": str(exc) or "assertion failed",
                    }
                )

        self.records.append(
            SubrunDiagnostic(
                scenario_id=self.scenario_id,
                subrun_id=subrun_id,
                model=self.model,
                status="pass" if not failures else "fail",
                classification=(classification if failures else "pass"),
                failures=failures,
                metrics=dict(metrics or {}),
            )
        )
        return not failures

    def record_infrastructure_failure(
        self, subrun_id: str, message: str, *, metrics: dict[str, Any] | None = None
    ) -> None:
        self.records.append(
            SubrunDiagnostic(
                scenario_id=self.scenario_id,
                subrun_id=subrun_id,
                model=self.model,
                status="infrastructure_failure",
                classification="infrastructure_failure",
                failures=[{"gate": "execution", "message": message}],
                metrics=dict(metrics or {}),
            )
        )

    def record_failure(self, gate: str, message: str) -> None:
        self.records.append(
            SubrunDiagnostic(
                scenario_id=self.scenario_id,
                subrun_id="aggregate",
                model=self.model,
                status="fail",
                classification="aggregate_behavioral_failure",
                failures=[{"gate": gate, "message": message}],
            )
        )

    def record_diagnostic(
        self,
        subrun_id: str,
        classification: str,
        *,
        metrics: dict[str, Any] | None = None,
    ) -> None:
        """Record an observed model-specific fact without failing the gate."""
        self.records.append(
            SubrunDiagnostic(
                scenario_id=self.scenario_id,
                subrun_id=subrun_id,
                model=self.model,
                status="diagnostic",
                classification=classification,
                failures=[],
                metrics=dict(metrics or {}),
            )
        )

    @property
    def failures(self) -> list[SubrunDiagnostic]:
        return [
            record
            for record in self.records
            if record.status not in ("pass", "diagnostic")
        ]

    def write(self, output_dir: Path) -> None:
        payload = {
            "scenario_id": self.scenario_id,
            "model": self.model,
            "subruns": [asdict(record) for record in self.records],
        }
        (output_dir / "behavioral_diagnostics.json").write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )

    def failure_message(self) -> str:
        lines = []
        for record in self.failures:
            gates = "; ".join(
                f"{failure['gate']}: {failure['message']}"
                for failure in record.failures
            )
            lines.append(f"{record.subrun_id} [{record.classification}] {gates}")
        return "Composite simulator subruns failed:\n" + "\n".join(lines)


def artifact_metrics(summary: dict[str, Any], done: Any) -> dict[str, Any]:
    """Return stable high-level metrics for matrix rows."""

    def done_value(name: str) -> Any:
        if isinstance(done, dict):
            return done.get(name)
        return getattr(done, name, None)

    return {
        "fell": summary.get("fell"),
        "max_abs_pitch_deg": summary.get("max_abs_pitch_deg"),
        "peak_pitch_rate_dps": summary.get("peak_pitch_rate_dps"),
        "tail_rms_pitch_deg": summary.get("tail_rms_pitch_deg"),
        "tail_rms_pitch_rate_dps": summary.get("tail_rms_pitch_rate_dps"),
        "tail_rms_velocity_mps": summary.get("tail_rms_velocity_mps"),
        "tail_mean_abs_velocity_mps": summary.get("tail_mean_abs_velocity_mps"),
        "peak_velocity_mps": summary.get("peak_velocity_mps"),
        "peak_velocity_sps": summary.get("peak_velocity_sps"),
        "peak_requested_sps": summary.get("peak_requested_sps"),
        "peak_applied_sps": summary.get("peak_applied_sps"),
        "p95_applied_sps": summary.get("p95_applied_sps"),
        "p99_applied_sps": summary.get("p99_applied_sps"),
        "saturation_duration_s": summary.get("saturation_duration_s"),
        "saturation_duty": summary.get("saturation_duty"),
        "settled_at_s": summary.get("settled_at_s"),
        "max_abs_position_m": summary.get("max_abs_position_m"),
        "reason_code": done_value("reason_code"),
        "controller_fault_flags": done_value("controller_fault_flags"),
        "actuator_fault_count": done_value("actuator_fault_count"),
        "max_continuous_saturation_s": done_value("max_continuous_saturation_s"),
        "oscillation_trend": summary.get("oscillation_trend"),
        "oscillation_windows": summary.get("oscillation_windows", {}),
    }
