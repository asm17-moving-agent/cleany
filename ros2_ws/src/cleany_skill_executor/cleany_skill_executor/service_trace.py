"""선택적으로 출력만 하는 ROS 서비스 요청 및 완료 시간 기록입니다."""
from __future__ import annotations

import json
from pathlib import Path
import time
from typing import Any, Callable
from uuid import uuid4

from rclpy.serialization import serialize_message


class ServiceTrace:
    def __init__(self, directory: str, warn: Callable[[str], None]) -> None:
        root = Path(directory)
        if not directory or not root.is_absolute():
            raise ValueError('service trace directory must be absolute')
        self.directory = root / f'services-{uuid4().hex}'
        self.directory.mkdir(parents=True)
        self._sequence = 0
        self._warn = warn

    def __call__(self, service: str, request: Any, future: Any) -> None:
        self._sequence += 1
        stem = self.directory / f'{self._sequence:06d}'
        started = time.monotonic()
        try:
            stem.with_suffix('.request.cdr').write_bytes(serialize_message(request))
        except Exception as error:
            self._warn(f'Service request trace failed: {error}')
            return

        def completed(result: Any) -> None:
            metadata = {'service': service, 'elapsed_sec': time.monotonic() - started,
                        'cancelled': result.cancelled(),
                        'request_type': type(request).__qualname__}
            try:
                if not result.cancelled():
                    response = result.result()
                    if response is not None:
                        stem.with_suffix('.response.cdr').write_bytes(
                            serialize_message(response))
                stem.with_suffix('.json').write_text(json.dumps(metadata), encoding='utf-8')
            except Exception as error:
                self._warn(f'Service response trace failed: {error}')

        future.add_done_callback(completed)
