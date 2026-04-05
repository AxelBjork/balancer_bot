from __future__ import annotations

import os
import signal
import subprocess
import time
from pathlib import Path

import pytest

from udp_client import UdpClient

_REPO_ROOT = Path(__file__).parents[2]
_BUILD_DIR = _REPO_ROOT / "build"
_DEFAULT_BIN = _BUILD_DIR / "sil_app"


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
    ctest_cmd = ["ctest", "--test-dir", str(_BUILD_DIR), "--output-on-failure", "-j8"]

    try:
        subprocess.run(configure_cmd, check=True, cwd=_REPO_ROOT)
        subprocess.run(build_cmd, check=True, cwd=_REPO_ROOT)
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


@pytest.fixture(scope="session")
def sil_process():
    proc = subprocess.Popen(
        [str(_sil_binary())],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )

    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            pytest.fail(f"sil_app exited during startup (rc={proc.returncode})")
        time.sleep(0.02)
        if time.monotonic() + 0.1 >= deadline:
            break

    yield proc

    if proc.poll() is None:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            proc.wait(timeout=5)


@pytest.fixture(scope="function")
def udp(sil_process):
    if sil_process.poll() is not None:
        pytest.fail(f"sil_app is not running (rc={sil_process.returncode})")

    with UdpClient() as client:
        client.register()
        client.drain()
        yield client
