"""Signal only currently live processes that belong to a recorded test output."""

from __future__ import annotations
import os
from pathlib import Path
import signal


def signal_owned(pid: int, output: Path, marker: bytes, *, group: bool = False) -> bool:
    """현재 PID의 실행 경로를 대조한 뒤 이 실험 소유 프로세스만 종료한다."""
    try:
        command = Path(f"/proc/{pid}/cmdline").read_bytes()
        if str(output).encode() not in command or marker not in command:
            return False
        if group:
            if os.getpgid(pid) != pid:
                return False
            os.killpg(pid, signal.SIGINT)
        else:
            os.kill(pid, signal.SIGINT)
    except (FileNotFoundError, ProcessLookupError):
        return False
    return True
