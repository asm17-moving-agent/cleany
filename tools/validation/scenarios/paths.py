"""Shared external data paths for scenario runners."""

from __future__ import annotations
import os
from pathlib import Path
import subprocess


def data_root(root: Path, explicit: Path | None = None) -> Path:
    """명시 옵션, 환경변수, 저장소 옆 기본 경로 순서로 데이터 위치를 선택한다."""
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
    """이 checkout에 대응하는 시나리오용 ROS 설치 환경을 찾는다."""
    build = Path(
        os.environ.get("CLEANY_BUILD_ROOT") or data / "builds" / root.resolve().name
    )
    return build.expanduser().resolve() / "install-scenario-runtime/setup.bash"


def batch_directory(output_root: Path, name: str) -> Path:
    """실행 이름이 출력 루트 밖으로 벗어나지 않도록 검사한다."""
    if not name or name in (".", "..") or Path(name).name != name:
        raise ValueError("batch name must be a single directory name")
    return output_root / name


def source_identity(root: Path) -> dict:
    """실행 근거에 남길 commit, branch, 미커밋 변경을 수집한다."""

    def git(*args: str) -> str:
        return subprocess.check_output(
            ["git", "-C", str(root), *args], text=True
        ).strip()

    return {
        "base_commit": git("rev-parse", "HEAD"),
        "branch": git("branch", "--show-current"),
        "source_status": git("status", "--porcelain"),
    }
