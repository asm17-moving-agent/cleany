"""Bounded asynchronous control-event journal; disk I/O never runs in control callbacks."""

from datetime import datetime, timezone
import json
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import queue
import sys
import threading
import uuid


class _CheckedHandler(RotatingFileHandler):
    def handleError(self, record):
        raise sys.exc_info()[1]


class EventJournal:
    def __init__(
        self,
        path: Path,
        *,
        max_bytes: int = 2_000_000,
        backups: int = 3,
        capacity: int = 256,
    ):
        if max_bytes <= 0 or backups < 1 or capacity < 1:
            raise ValueError("journal bounds must be positive")
        self.path = path.expanduser().resolve()
        self.max_bytes, self.backups = max_bytes, backups
        self.queue = queue.Queue(maxsize=capacity)
        self.lock = threading.Lock()
        self.dropped = self.written = self.sequence = 0
        self.error: str | None = None
        self.closed = False
        self.run_id = uuid.uuid4().hex
        self.thread = threading.Thread(
            target=self._work, name="teleop-events", daemon=True
        )
        self.thread.start()

    def emit(self, event: dict) -> None:
        with self.lock:
            self.sequence += 1
            record = dict(
                event,
                utc=datetime.now(timezone.utc).isoformat(),
                run_id=self.run_id,
                event_id=self.sequence,
                pid=os.getpid(),
                component="teleop",
            )
            if self.closed:
                self.dropped += 1
                return
            try:
                self.queue.put_nowait(record)
            except queue.Full:
                self.dropped += 1

    def status(self) -> dict:
        with self.lock:
            return dict(
                path=str(self.path),
                run_id=self.run_id,
                written=self.written,
                dropped=self.dropped,
                error=self.error,
            )

    def _work(self) -> None:
        handler = None
        while True:
            event = self.queue.get()
            try:
                if event is None:
                    return
                if handler is None:
                    self.path.parent.mkdir(parents=True, exist_ok=True)
                    handler = _CheckedHandler(
                        self.path,
                        maxBytes=self.max_bytes,
                        backupCount=self.backups,
                        encoding="utf-8",
                        delay=True,
                    )
                handler.emit(
                    logging.LogRecord(
                        "teleop.events",
                        logging.INFO,
                        "",
                        0,
                        json.dumps(event, ensure_ascii=False),
                        (),
                        None,
                    )
                )
                with self.lock:
                    self.written += 1
            except Exception as error:
                with self.lock:
                    changed = self.error != str(error)
                    self.error = str(error)
                    self.dropped += 1
                if changed:
                    print(
                        f"Teleop event journal failed ({self.path}): {error}",
                        file=sys.stderr,
                        flush=True,
                    )
                if handler is not None:
                    handler.close()
                    handler = None
            finally:
                self.queue.task_done()
                if event is None and handler is not None:
                    handler.close()

    def close(self, timeout: float = 2.0) -> None:
        with self.lock:
            if self.closed:
                return
            self.closed = True
        try:
            self.queue.put(None, timeout=timeout)
        except queue.Full:
            with self.lock:
                self.error = (
                    "Event journal shutdown timed out; pending records may be lost"
                )
            return
        self.thread.join(timeout)
        if self.thread.is_alive():
            with self.lock:
                self.error = (
                    "Event journal shutdown timed out; pending records may be lost"
                )
