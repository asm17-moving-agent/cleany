"""One physical trajectory owner per user; process exit releases the lease."""
import fcntl
import os
from pathlib import Path
import tempfile


class ExecutionOwnership:
    def __init__(self, path: str | Path | None = None):
        self.path = Path(path or Path(tempfile.gettempdir()) / f'cleany-physical-motion-{os.getuid()}.lock')
        self._file = self.path.open('a+')
        try:
            fcntl.flock(self._file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            self._file.close()
            raise RuntimeError('Another physical manipulation process owns trajectory execution') from error

    def close(self):
        self._file.close()
