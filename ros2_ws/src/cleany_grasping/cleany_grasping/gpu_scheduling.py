"""Cross-process GPU admission using shared, read-only lock-file inodes."""
from __future__ import annotations

from contextlib import ExitStack, contextmanager
import fcntl
from pathlib import Path
from threading import RLock
import time
from typing import Callable


class GpuLease:
    """Serialize GPU work and let background work yield to pending requests.

    Both containers must bind the same host files, including the .demand file.
    Files are opened read-only; locks are released by the OS on process exit.
    """

    def __init__(self, path: str = '') -> None:
        self._path = Path(path) if path else None
        if self._path is not None and not all(p.is_file() for p in (
                self._path, Path(str(self._path) + '.demand'))):
            raise ValueError('GPU scheduling requires the shared lock and demand files')
        self._lock = RLock()
        self._depth = 0

    @contextmanager
    def acquire(self, *, foreground: bool = True, blocking: bool = True,
                continue_work: Callable[[], bool] | None = None):
        with self._lock:
            if self._path is None or self._depth:
                self._depth += 1
                try:
                    yield True
                finally:
                    self._depth -= 1
                return
            with ExitStack() as stack:
                demand = stack.enter_context(Path(str(self._path) + '.demand').open('rb'))
                try:
                    fcntl.flock(demand, fcntl.LOCK_SH if foreground else fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    yield False
                    return
                work = stack.enter_context(self._path.open('rb'))
                while True:
                    if continue_work is not None and not continue_work():
                        raise InterruptedError('GPU admission canceled or timed out')
                    try:
                        fcntl.flock(work, fcntl.LOCK_EX | fcntl.LOCK_NB)
                        break
                    except BlockingIOError:
                        if not blocking:
                            yield False
                            return
                        time.sleep(.01)
                if not foreground:
                    fcntl.flock(demand, fcntl.LOCK_UN)
                self._depth = 1
                try:
                    yield True
                finally:
                    self._depth = 0
                    fcntl.flock(work, fcntl.LOCK_UN)

    def foreground_pending(self) -> bool:
        if self._path is None:
            return False
        with Path(str(self._path) + '.demand').open('rb') as demand:
            try:
                fcntl.flock(demand, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                return True
            fcntl.flock(demand, fcntl.LOCK_UN)
            return False
