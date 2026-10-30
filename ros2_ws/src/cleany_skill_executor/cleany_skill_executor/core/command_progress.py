"""Local operator-command observations before a manipulation Action exists."""

from __future__ import annotations

import json
import os
from pathlib import Path
import tempfile
import time


def process_identity(pid: int) -> str:
    """Return Linux process start ticks, distinguishing reused PIDs."""
    try:
        return Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[19]
    except (OSError, IndexError):
        return ''


class CommandProgress:
    def __init__(self, path: str, execution_id: str, owner_pid: int) -> None:
        self.path = Path(path).expanduser()
        self.execution_id, self.owner_pid = execution_id, owner_pid
        self.owner_start = process_identity(owner_pid)

    def update(self, phase: str, message: str = '', *, state: str = 'ACTIVE') -> None:
        """Best-effort telemetry must not interrupt robot startup or cancellation."""
        temporary = None
        message = message[:2048]
        try:
            try:
                previous = json.loads(self.path.read_text())
            except (OSError, ValueError):
                previous = {}
            if not isinstance(previous, dict):
                previous = {}
            if previous.get('execution_id') != self.execution_id:
                previous = {}
            # The parent shell's final exit must retain a more specific client error.
            if previous.get('state') == 'FAILED' and phase == 'COMMAND_FAILED':
                return
            now = time.time_ns()
            history = previous.get('history', [])
            if not isinstance(history, list):
                history = []
            if not history or history[-1]['phase'] != phase:
                history = [*history, dict(phase=phase, at_ns=now, message=message[:512])][-16:]
            value = dict(schema=1, execution_id=self.execution_id, phase=phase, state=state,
                         message=message[:2048], started_at_ns=previous.get('started_at_ns', now),
                         updated_at_ns=now, owner_pid=self.owner_pid, owner_start=self.owner_start,
                         history=history)
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile(mode='w', dir=self.path.parent, delete=False) as handle:
                temporary = Path(handle.name)
                json.dump(value, handle)
            temporary.replace(self.path)
        except (OSError, ValueError, TypeError, KeyError):
            pass
        finally:
            if temporary is not None:
                try:
                    temporary.unlink(missing_ok=True)
                except OSError:
                    pass

    @classmethod
    def from_environment(cls, execution_id: str) -> CommandProgress | None:
        path = os.environ.get('CLEANY_MANIPULATION_PROGRESS_FILE')
        if not path:
            return None
        try:
            value = json.loads(Path(path).expanduser().read_text())
            if isinstance(value, dict) and value.get('execution_id') == execution_id:
                return cls(path, execution_id, int(value['owner_pid']))
        except (OSError, ValueError, TypeError, KeyError):
            pass
        return None
