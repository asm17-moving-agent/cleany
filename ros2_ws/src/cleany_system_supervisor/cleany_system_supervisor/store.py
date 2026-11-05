"""Atomic JSON persistence for state that must survive a Supervisor restart."""
import json
import os
from pathlib import Path

FORMAT_VERSION = 1


class LatchStore:
    def __init__(self, path: str | Path):
        self.path = Path(path).expanduser()

    def load(self) -> dict | None:
        """None means no previous state. Unreadable state raises: callers must fail closed."""
        if not self.path.exists():
            return None
        data = json.loads(self.path.read_text(encoding='utf-8'))
        if data.get('format_version') != FORMAT_VERSION:
            raise ValueError(f'unsupported supervisor state format {data.get("format_version")!r}')
        for key in ('faults', 'leases'):
            if not isinstance(data.get(key), list):
                raise ValueError(f'supervisor state lacks {key}')
        return data

    def save(self, state: dict) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + '.tmp')
        with open(temporary, 'w', encoding='utf-8') as stream:
            json.dump({'format_version': FORMAT_VERSION, **state}, stream, sort_keys=True)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.path)
