from __future__ import annotations

import os
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
