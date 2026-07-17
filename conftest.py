from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
from pathlib import Path

import pytest

from tests.fuzz.generate_corpora import write_corpora
from tests.fuzz.registry import corpus_root, iter_fuzz_targets

_REPO_ROOT = Path(__file__).parent
_BUILD_DIR = _REPO_ROOT / "build"
_AFL_BUILD_DIR = _REPO_ROOT / "build-afl"
_FUZZ_SMOKE_BUILD_DIR = _REPO_ROOT / "build-fuzz-smoke"


def pytest_addoption(parser):
    parser.addoption(
        "--build",
        action="store_true",
        default=False,
        help="Run CMake configure, build, and CTest before the pytest suite.",
    )
    parser.addoption(
        "--build-only",
        action="store_true",
        default=False,
        help="Run CMake configure, build, and CTest, then exit without running Python tests.",
    )
    parser.addoption(
        "--fuzz",
        action="store_true",
        default=False,
        help="Run an AFL++ build in build-afl/ and validate the registered fuzz harness corpora.",
    )
    parser.addoption(
        "--fuzz-smoke",
        action="store_true",
        default=False,
        help="Build fuzz harnesses with the host compiler and run every registered seed.",
    )


def pytest_sessionstart(session):
    config = session.config
    wants_build = (
        config.getoption("--build")
        or config.getoption("--build-only")
        or config.getoption("--fuzz")
        or config.getoption("--fuzz-smoke")
    )
    if not wants_build:
        return

    try:
        _run_standard_build()
        if config.getoption("--fuzz"):
            _run_afl_validation()
        if config.getoption("--fuzz-smoke"):
            _run_host_fuzz_smoke()
    except subprocess.CalledProcessError as exc:
        pytest.exit(f"Build, CTest, or AFL validation failed (rc={exc.returncode})", returncode=1)
    except RuntimeError as exc:
        pytest.exit(str(exc), returncode=1)

    if config.getoption("--build-only"):
        if config.getoption("--fuzz") or config.getoption("--fuzz-smoke"):
            pytest.exit("C++ build/CTest and fuzz validation successful. Exiting.", returncode=0)
        pytest.exit("C++ build/CTest successful. Exiting.", returncode=0)


def _run_standard_build() -> None:
    configure_cmd = [
        "cmake",
        "-S",
        str(_REPO_ROOT),
        "-B",
        str(_BUILD_DIR),
        "-DBUILD_TESTS=ON",
        "-DBUILD_REFLECTION_ARTIFACTS=ON",
    ]
    build_cmd = [
        "cmake",
        "--build",
        str(_BUILD_DIR),
        "--target",
        "balancer_tests",
        "balancer_simulator",
        "sil_app",
        "balancer_reflection",
        "-j8",
    ]
    ctest_cmd = ["ctest", "--test-dir", str(_BUILD_DIR), "--output-on-failure", "-j8"]

    cache_path = _BUILD_DIR / "CMakeCache.txt"
    cache = cache_path.read_text(encoding="utf-8", errors="replace") if cache_path.exists() else ""
    configured_for_gate = (
        "BUILD_TESTS:BOOL=ON" in cache and "BUILD_REFLECTION_ARTIFACTS:BOOL=ON" in cache
    )
    # The build tool re-runs CMake automatically when CMake inputs change. Avoiding an
    # unconditional no-op configure saves roughly ten seconds in this environment.
    if not configured_for_gate:
        subprocess.run(configure_cmd, check=True, cwd=_REPO_ROOT)
    subprocess.run(build_cmd, check=True, cwd=_REPO_ROOT)
    subprocess.run(ctest_cmd, check=True, cwd=_REPO_ROOT)


def _afl_runtime_env() -> dict[str, str]:
    env = os.environ.copy()
    env.setdefault("ASAN_OPTIONS", "abort_on_error=1:detect_leaks=0:symbolize=0")
    return env


def _run_host_fuzz_smoke() -> None:
    configure_cmd = [
        "cmake",
        "-S",
        str(_REPO_ROOT),
        "-B",
        str(_FUZZ_SMOKE_BUILD_DIR),
        "-DBUILD_TESTS=ON",
        "-DBUILD_AFL_TARGETS=ON",
        "-DBUILD_REFLECTION_ARTIFACTS=OFF",
        "-DCMAKE_BUILD_TYPE=RelWithDebInfo",
    ]
    build_cmd = [
        "cmake",
        "--build",
        str(_FUZZ_SMOKE_BUILD_DIR),
        "--target",
        "balancer_fuzz_targets",
        "-j8",
    ]
    subprocess.run(configure_cmd, check=True, cwd=_REPO_ROOT)
    subprocess.run(build_cmd, check=True, cwd=_REPO_ROOT)
    write_corpora(corpus_root(_FUZZ_SMOKE_BUILD_DIR))
    for target in iter_fuzz_targets():
        for seed in target.seed_files(_FUZZ_SMOKE_BUILD_DIR):
            subprocess.run(
                target.command(_FUZZ_SMOKE_BUILD_DIR, seed),
                check=True,
                cwd=_REPO_ROOT,
                timeout=max(1.0, target.timeout_ms / 1000.0),
            )


def _run_afl_validation() -> None:
    afl_cc = shutil.which("afl-clang-fast")
    afl_cxx = shutil.which("afl-clang-fast++")
    afl_showmap = shutil.which("afl-showmap")
    if not afl_cc or not afl_cxx or not afl_showmap:
        raise RuntimeError(
            "AFL++ tools not found on PATH; need afl-clang-fast, "
            "afl-clang-fast++, and afl-showmap"
        )

    configure_cmd = [
        "cmake",
        "-S",
        str(_REPO_ROOT),
        "-B",
        str(_AFL_BUILD_DIR),
        f"-DCMAKE_C_COMPILER={afl_cc}",
        f"-DCMAKE_CXX_COMPILER={afl_cxx}",
        "-DBUILD_TESTS=ON",
        "-DBUILD_AFL_TARGETS=ON",
        "-DBUILD_REFLECTION_ARTIFACTS=OFF",
        "-DCMAKE_BUILD_TYPE=RelWithDebInfo",
    ]
    build_cmd = [
        "cmake",
        "--build",
        str(_AFL_BUILD_DIR),
        "--target",
        "balancer_fuzz_targets",
        "-j8",
    ]

    build_env = os.environ.copy()
    build_env.setdefault("AFL_USE_ASAN", "1")
    subprocess.run(configure_cmd, check=True, cwd=_REPO_ROOT, env=build_env)
    subprocess.run(build_cmd, check=True, cwd=_REPO_ROOT, env=build_env)
    write_corpora(corpus_root(_AFL_BUILD_DIR))

    _validate_afl_targets(afl_showmap)


def _validate_afl_targets(afl_showmap: str) -> None:
    with tempfile.TemporaryDirectory(dir=_AFL_BUILD_DIR) as temp_dir:
        temp_root = Path(temp_dir)
        for target in iter_fuzz_targets():
            binary = target.binary_path(_AFL_BUILD_DIR)
            if not binary.exists():
                raise RuntimeError(f"Expected fuzz target binary at {binary}")

            seeds = target.seed_files(_AFL_BUILD_DIR)
            if not seeds:
                raise RuntimeError(f"No seed files found for fuzz target {target.name}")

            for seed in seeds:
                trace_path = temp_root / f"{target.name}_{seed.name}.trace"
                cmd = [
                    afl_showmap,
                    "-q",
                    "-o",
                    str(trace_path),
                    "-t",
                    str(target.timeout_ms),
                    "--",
                    *target.command(_AFL_BUILD_DIR, seed),
                ]
                subprocess.run(
                    cmd,
                    check=True,
                    cwd=_REPO_ROOT,
                    env=_afl_runtime_env(),
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                if not trace_path.exists() or trace_path.stat().st_size == 0:
                    raise RuntimeError(
                        f"afl-showmap produced no coverage for {target.name} seed {seed.name}"
                    )
