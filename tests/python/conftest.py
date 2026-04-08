from __future__ import annotations

import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest

from udp_client import UdpClient

_REPO_ROOT = Path(__file__).parents[2]
_BUILD_DIR = _REPO_ROOT / "build"
_DEFAULT_BIN = _BUILD_DIR / "sil_app"
_DEFAULT_SIM_BIN = _BUILD_DIR / "balancer_simulator"
_DEFAULT_SIM_PORT = 9001

if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))


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


def pytest_sessionstart(session):
    config = session.config
    if not config.getoption("--build") and not config.getoption("--build-only"):
        return

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

    try:
        subprocess.run(configure_cmd, check=True, cwd=_REPO_ROOT)
        subprocess.run(build_cmd, check=True, cwd=_REPO_ROOT)
        subprocess.run(bindings_cmd, check=True, cwd=_REPO_ROOT)
        subprocess.run(ctest_cmd, check=True, cwd=_REPO_ROOT)
    except subprocess.CalledProcessError as exc:
        pytest.exit(f"C++ build or CTest failed (rc={exc.returncode})", returncode=1)

    if config.getoption("--build-only"):
        pytest.exit("C++ build/CTest successful. Exiting.", returncode=0)


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
