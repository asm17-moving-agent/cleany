"""Supervisor 재시작 후에도 유지해야 할 상태의 원자적 JSON 저장입니다."""
import json
import os
from pathlib import Path

FORMAT_VERSION = 1


class LatchStore:
    def __init__(self, path: str | Path):
        self.path = Path(path).expanduser()

    def load(self) -> dict | None:
        """None은 이전 상태가 없다는 뜻입니다. 읽을 수 없으면 예외를 내며 호출자는 거부해야 합니다."""
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
