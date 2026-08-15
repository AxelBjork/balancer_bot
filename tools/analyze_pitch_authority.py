#!/usr/bin/env python3
"""Analyze a direct pitch-target authority sweep from telemetry."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

# When invoked as ``python3 tools/analyze_pitch_authority.py``, Python puts the
# tools directory—not the repository root—on sys.path. Make the documented
# standalone invocation use the same package imports as pytest.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tools.telemetry_analysis import (
    PITCH_AUTHORITY_HARDWARE_ENVELOPES,
    analyze_pitch_authority_sweep,
    compare_pitch_authority_sweeps,
    read_telemetry_csv,
    validate_pitch_authority_hardware_envelope,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="telemetry CSV to analyze")
    parser.add_argument(
        "--simulator-csv",
        type=Path,
        help="optional simulator sweep CSV to compare with the primary capture",
    )
    parser.add_argument(
        "--envelope",
        choices=sorted(PITCH_AUTHORITY_HARDWARE_ENVELOPES),
        help="apply a predeclared supervised-hardware release gate",
    )
    parser.add_argument("--output", type=Path, help="write JSON here instead of stdout")
    args = parser.parse_args()

    primary = read_telemetry_csv(args.csv)
    primary_rows = analyze_pitch_authority_sweep(primary)
    if args.simulator_csv is None:
        report = {
            "source": str(args.csv),
            "pulse_count": len(primary_rows),
            "pulses": primary_rows,
        }
    else:
        simulator = read_telemetry_csv(args.simulator_csv)
        report = {
            "hardware_source": str(args.csv),
            "simulator_source": str(args.simulator_csv),
            "pulses": compare_pitch_authority_sweeps(simulator, primary),
        }
    violations = []
    if args.envelope is not None:
        violations = validate_pitch_authority_hardware_envelope(
            primary_rows, stage=args.envelope
        )
        report["envelope"] = {
            "stage": args.envelope,
            "passed": not violations,
            "violations": violations,
        }
    encoded = json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n"
    if args.output is None:
        print(encoded, end="")
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded, encoding="utf-8")
    return 2 if violations else 0


if __name__ == "__main__":
    raise SystemExit(main())
