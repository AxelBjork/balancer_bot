#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import signal
import shutil
import subprocess
import sys
from pathlib import Path


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


REPO_ROOT = _repo_root()
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tests.fuzz.generate_corpora import write_corpora  # noqa: E402
from tests.fuzz.registry import DEFAULT_BUILD_DIR, FUZZ_TARGETS, corpus_root, get_fuzz_target  # noqa: E402


def _find_afl_fuzz() -> str | None:
    afl_fuzz = shutil.which("afl-fuzz")
    if afl_fuzz:
        return afl_fuzz
    repo_local = Path("/usr/local/bin/afl-fuzz")
    if repo_local.exists():
        return str(repo_local)
    return None


def _default_asan_options() -> str:
    return "abort_on_error=1:detect_leaks=0:symbolize=0"


def main() -> int:
    parser = argparse.ArgumentParser(description="Run afl-fuzz for a registered balancer fuzz target.")
    parser.add_argument("target", nargs="?", choices=sorted(FUZZ_TARGETS), help="Registered fuzz target name")
    parser.add_argument("--list", action="store_true", help="List registered fuzz targets and exit")
    parser.add_argument("--build-dir", default=str(DEFAULT_BUILD_DIR), help="AFL build directory")
    parser.add_argument("--output-dir", help="AFL output directory")
    parser.add_argument("--input-dir", help="Override input corpus directory")
    parser.add_argument("--timeout-ms", type=int, help="Per-exec timeout passed to afl-fuzz")
    args, extra_afl_args = parser.parse_known_args()
    if extra_afl_args and extra_afl_args[0] == "--":
        extra_afl_args = extra_afl_args[1:]

    if args.list:
        for name, target in sorted(FUZZ_TARGETS.items()):
            print(f"{name}: {target.binary_name} <- {target.corpus_dir()}")
        return 0

    if not args.target:
        parser.error("target is required unless --list is used")

    afl_fuzz = _find_afl_fuzz()
    if not afl_fuzz:
        print("afl-fuzz not found on PATH or at ./AFLplusplus/afl-fuzz", file=sys.stderr)
        return 1

    target = get_fuzz_target(args.target)
    build_dir = Path(args.build_dir)
    binary = target.binary_path(build_dir)
    if not binary.exists():
        print(f"fuzz target binary not found: {binary}", file=sys.stderr)
        return 1

    if args.input_dir:
        input_dir = Path(args.input_dir)
    else:
        write_corpora(corpus_root(build_dir))
        input_dir = target.corpus_dir(build_dir)
    if not input_dir.exists():
        print(f"input corpus not found: {input_dir}", file=sys.stderr)
        return 1

    output_dir = Path(args.output_dir) if args.output_dir else build_dir / "afl-out" / target.name
    timeout_ms = args.timeout_ms or target.timeout_ms

    env = os.environ.copy()
    env.setdefault("ASAN_OPTIONS", _default_asan_options())
    env.setdefault("AFL_I_DONT_CARE_ABOUT_MISSING_CRASHES", "1")
    env.setdefault("AFL_SKIP_CPUFREQ", "1")

    cmd = [
        afl_fuzz,
        "-i",
        str(input_dir),
        "-o",
        str(output_dir),
        "-t",
        f"{timeout_ms}+",
        *extra_afl_args,
        "--",
        *target.command(build_dir, "@@"),
    ]

    with subprocess.Popen(cmd, cwd=REPO_ROOT, env=env) as proc:
        try:
            return proc.wait()
        except KeyboardInterrupt:
            try:
                proc.wait(timeout=5)
                return 130
            except subprocess.TimeoutExpired:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=5)
                    return 130
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
                    return 130


if __name__ == "__main__":
    raise SystemExit(main())
