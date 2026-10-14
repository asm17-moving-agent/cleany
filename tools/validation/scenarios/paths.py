"""Shared external data paths for scenario runners."""

from __future__ import annotations
import os
from pathlib import Path
import subprocess


def data_root(root: Path, explicit: Path | None = None) -> Path:
    """Resolve data storage from the explicit path, environment, then checkout default."""
    return (
        Path(
            explicit
            or os.environ.get("CLEANY_DATA_ROOT")
            or root.resolve().parent / "cleany-data"
        )
        .expanduser()
        .resolve()
    )


def runtime_overlay(root: Path, data: Path) -> Path:
    build = Path(
        os.environ.get("CLEANY_BUILD_ROOT") or data / "builds" / root.resolve().name
    )
    return build.expanduser().resolve() / "install-scenario-runtime/setup.bash"


def batch_directory(output_root: Path, name: str) -> Path:
    """Reject empty or nested names that could escape the output directory."""
    if not name or name in (".", "..") or Path(name).name != name:
        raise ValueError("batch name must be a single directory name")
    return output_root / name


def source_identity(root: Path) -> dict:

    def git(*args: str) -> str:
        return subprocess.check_output(
            ["git", "-C", str(root), *args], text=True
        ).strip()

    return {
        "base_commit": git("rev-parse", "HEAD"),
        "branch": git("branch", "--show-current"),
        "source_status": git("status", "--porcelain"),
    }
