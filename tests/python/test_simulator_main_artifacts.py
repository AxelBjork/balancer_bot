from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path


def test_balancer_simulator_writes_artifacts(tmp_path):
    output_dir = tmp_path / "simulator_run"
    env = dict(os.environ)
    env["BALANCER_RUN_DIR"] = str(output_dir)

    proc = subprocess.run(
        [str(Path("build") / "balancer_simulator")],
        cwd=Path.cwd(),
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )

    assert (output_dir / "timeline.csv").exists()
    assert (output_dir / "metadata.json").exists()
    assert (output_dir / "summary.json").exists()
    assert (output_dir / "pitch_plot.svg").exists()
    assert (output_dir / "command_plot.svg").exists()

    summary = json.loads((output_dir / "summary.json").read_text(encoding="utf-8"))
    metadata = json.loads((output_dir / "metadata.json").read_text(encoding="utf-8"))
    assert summary["sample_count"] > 0
    assert summary["fell"] == (proc.returncode != 0)
    assert metadata["physics_profile"] == "simplified"
    assert metadata["pid_profile"].endswith("pid_sim.conf")
