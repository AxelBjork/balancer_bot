from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tests.python.support.run_artifacts import analyze_timeline_rows, load_csv_rows, summarize_rows


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze balancer timeline.csv artifacts.")
    parser.add_argument("csv", help="Path to timeline.csv")
    parser.add_argument(
        "--summary-json",
        action="store_true",
        help="Include the standard summary fields in the output",
    )
    parser.add_argument(
        "--output",
        help="Optional path to write JSON output. Defaults to stdout.",
    )
    args = parser.parse_args()

    rows = load_csv_rows(Path(args.csv))
    payload: dict[str, object] = {
        "path": str(Path(args.csv)),
        "analysis": analyze_timeline_rows(rows),
    }
    if args.summary_json:
        payload["summary"] = summarize_rows(rows)

    encoded = json.dumps(payload, indent=2, sort_keys=True) + "\n"
    if args.output:
        Path(args.output).write_text(encoded, encoding="utf-8")
    else:
        print(encoded, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
