from __future__ import annotations

import os
import signal
import shutil
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

_REPO_ROOT = Path(__file__).parents[2]
_BUILD_DIR = _REPO_ROOT / "build"
_AFL_BUILD_DIR = _REPO_ROOT / "build-afl"
_DEFAULT_BIN = _BUILD_DIR / "sil_app"
_DEFAULT_SIM_BIN = _BUILD_DIR / "balancer_simulator"
_DEFAULT_SIM_PORT = 9001

if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import pytest

from tests.fuzz.generate_corpora import write_corpora
from tests.fuzz.registry import corpus_root, iter_fuzz_targets
from udp_client import UdpClient


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


def pytest_sessionstart(session):
    config = session.config
    wants_build = (
        config.getoption("--build")
        or config.getoption("--build-only")
        or config.getoption("--fuzz")
    )
    if not wants_build:
        return

    try:
        _run_standard_build()
        if config.getoption("--fuzz"):
            _run_afl_validation()
    except subprocess.CalledProcessError as exc:
        pytest.exit(f"Build, CTest, or AFL validation failed (rc={exc.returncode})", returncode=1)
    except RuntimeError as exc:
        pytest.exit(str(exc), returncode=1)

    if config.getoption("--build-only"):
        if config.getoption("--fuzz"):
            pytest.exit("C++ build/CTest and AFL validation successful. Exiting.", returncode=0)
        pytest.exit("C++ build/CTest successful. Exiting.", returncode=0)


def _run_standard_build() -> None:
    configure_cmd = [
        "cmake",
        "-S",
        str(_REPO_ROOT),
        "-B",
        str(_BUILD_DIR),
        "-DBUILD_TESTS=ON",
    ]
    build_cmd = ["cmake", "--build", str(_BUILD_DIR), "-j8"]
    bindings_cmd = ["cmake", "--build", str(_BUILD_DIR), "--target", "balancer_bindings"]
    ctest_cmd = ["ctest", "--test-dir", str(_BUILD_DIR), "--output-on-failure", "-j8"]

    subprocess.run(configure_cmd, check=True, cwd=_REPO_ROOT)
    subprocess.run(build_cmd, check=True, cwd=_REPO_ROOT)
    subprocess.run(bindings_cmd, check=True, cwd=_REPO_ROOT)
    subprocess.run(ctest_cmd, check=True, cwd=_REPO_ROOT)


def _afl_runtime_env() -> dict[str, str]:
    env = os.environ.copy()
    env.setdefault("ASAN_OPTIONS", "abort_on_error=1:detect_leaks=0:symbolize=0")
    return env


def _run_afl_validation() -> None:
    afl_cc = shutil.which("afl-clang-fast")
    afl_cxx = shutil.which("afl-clang-fast++")
    afl_showmap = shutil.which("afl-showmap")
    if not afl_cc or not afl_cxx or not afl_showmap:
        raise RuntimeError("AFL++ tools not found on PATH; need afl-clang-fast, afl-clang-fast++, and afl-showmap")

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


@pytest.fixture(scope="session")
def sil_process():
    proc = _start_sil_process()
    yield proc
    _stop_sil_process(proc)


@pytest.fixture(scope="function")
def udp(sil_process):
    if sil_process.poll() is not None:
        pytest.fail(f"sil_app is not running (rc={sil_process.returncode})")

    with UdpClient() as client:
        client.register()
        client.drain()
        yield client


def _start_sil_process():
    proc = subprocess.Popen(
        [str(_sil_binary())],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )

    deadline = time.monotonic() + 0.01
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            pytest.fail(f"sil_app exited during startup (rc={proc.returncode})")
        time.sleep(0.001)
    return proc


def _stop_sil_process(proc):
    if proc.poll() is None:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            proc.wait(timeout=5)


@pytest.fixture(scope="function")
def fresh_udp():
    proc = _start_sil_process()
    try:
        with UdpClient() as client:
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
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )

    deadline = time.monotonic() + 0.05
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            pytest.fail(f"balancer_simulator exited during startup (rc={proc.returncode})")
        time.sleep(0.001)

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
    output_root = _BUILD_DIR / "sim"
    output_root.mkdir(parents=True, exist_ok=True)
    return {
        "temp_root": output_root,
    }
