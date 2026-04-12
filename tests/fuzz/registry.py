from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_BUILD_DIR = REPO_ROOT / "build-afl"
CORPUS_DIRNAME = "fuzz-corpus"


@dataclass(frozen=True)
class FuzzTarget:
    name: str
    binary_name: str
    corpus_name: str
    argv: tuple[str, ...] = ("@@",)
    timeout_ms: int = 2000

    def binary_path(self, build_dir: str | Path = DEFAULT_BUILD_DIR) -> Path:
        return Path(build_dir) / self.binary_name

    def corpus_dir(self, build_dir: str | Path = DEFAULT_BUILD_DIR) -> Path:
        return Path(build_dir) / CORPUS_DIRNAME / self.corpus_name

    def seed_files(self, build_dir: str | Path = DEFAULT_BUILD_DIR) -> list[Path]:
        corpus_dir = self.corpus_dir(build_dir)
        if not corpus_dir.exists():
            return []
        return sorted(path for path in corpus_dir.iterdir() if path.is_file())

    def command(self, build_dir: str | Path = DEFAULT_BUILD_DIR, input_path: str | Path | None = None) -> list[str]:
        argv = [str(self.binary_path(build_dir))]
        for arg in self.argv:
            if arg == "@@" and input_path is not None:
                argv.append(str(input_path))
            else:
                argv.append(arg)
        return argv


FUZZ_TARGETS: dict[str, FuzzTarget] = {
    "udp_sequence": FuzzTarget(
        name="udp_sequence",
        binary_name="balancer_fuzz_udp_sequence",
        corpus_name="udp_sequence",
        timeout_ms=1000,
    ),
    "simulator_scenario": FuzzTarget(
        name="simulator_scenario",
        binary_name="balancer_fuzz_simulator_scenario",
        corpus_name="simulator_scenario",
        timeout_ms=2000,
    ),
}


def iter_fuzz_targets() -> list[FuzzTarget]:
    return [FUZZ_TARGETS[name] for name in sorted(FUZZ_TARGETS)]


def get_fuzz_target(name: str) -> FuzzTarget:
    try:
        return FUZZ_TARGETS[name]
    except KeyError as exc:
        choices = ", ".join(sorted(FUZZ_TARGETS))
        raise KeyError(f"Unknown fuzz target '{name}'. Expected one of: {choices}") from exc


def corpus_root(build_dir: str | Path = DEFAULT_BUILD_DIR) -> Path:
    return Path(build_dir) / CORPUS_DIRNAME
