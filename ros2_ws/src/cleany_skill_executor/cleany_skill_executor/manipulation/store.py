"""Atomic SQLite snapshots and an append-only progress journal, with process locking."""

from __future__ import annotations

import fcntl
import json
import logging
import os
from pathlib import Path
import sqlite3
from dataclasses import replace

from .models import ObjectState, Record, RecordState, Status


LOGGER = logging.getLogger(__name__)


class StoreError(RuntimeError):
    pass


class DuplicateExecution(StoreError):
    pass


def default_database_path() -> str:
    state = Path(os.environ.get('XDG_STATE_HOME') or Path.home() / '.local/state')
    return str(state / 'cleany/manipulation_mock/executions.sqlite3')


class ExecutionStore:
    def __init__(self, path: str) -> None:
        self.path = Path(path).expanduser().resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock_file = self.path.with_suffix(self.path.suffix + '.lock').open('a')
        self._db: sqlite3.Connection | None = None
        try:
            fcntl.flock(self._lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            self._lock_file.close()
            raise StoreError(f'database already owned: {self.path}') from exc
        try:
            self._db = sqlite3.connect(str(self.path), timeout=0.1, check_same_thread=False)
            self._db.execute('PRAGMA journal_mode=WAL')
            self._db.execute('PRAGMA synchronous=FULL')
            with self._db:
                self._db.execute('CREATE TABLE IF NOT EXISTS executions ('
                                 'execution_id TEXT PRIMARY KEY, payload TEXT NOT NULL)')
                self._db.execute('CREATE TABLE IF NOT EXISTS execution_events ('
                                 'execution_id TEXT NOT NULL, revision INTEGER NOT NULL, '
                                 'payload TEXT NOT NULL, PRIMARY KEY(execution_id, revision))')
        except sqlite3.Error as exc:
            self.close()
            raise StoreError(str(exc)) from exc

    def close(self) -> None:
        if self._db is not None:
            self._db.close()
            self._db = None
        if not self._lock_file.closed:
            fcntl.flock(self._lock_file, fcntl.LOCK_UN)
            self._lock_file.close()

    def save(self, record: Record, *, create: bool = False) -> None:
        payload = json.dumps(record.to_dict(), sort_keys=True)
        try:
            with self._db:
                if create:
                    self._db.execute('INSERT INTO executions VALUES (?, ?)',
                                     (record.goal.execution_id, payload))
                else:
                    cursor = self._db.execute(
                        'UPDATE executions SET payload=? WHERE execution_id=?',
                        (payload, record.goal.execution_id))
                    if cursor.rowcount != 1:
                        raise StoreError('execution disappeared from the database')
                self._db.execute('INSERT INTO execution_events VALUES (?, ?, ?)',
                                 (record.goal.execution_id, record.revision, payload))
        except sqlite3.IntegrityError as exc:
            if create:
                raise DuplicateExecution(record.goal.execution_id) from exc
            raise StoreError(str(exc)) from exc
        except sqlite3.Error as exc:
            raise StoreError(str(exc)) from exc

    def get(self, execution_id: str) -> Record | None:
        try:
            row = self._db.execute('SELECT payload FROM executions WHERE execution_id=?',
                                   (execution_id,)).fetchone()
            return Record.from_dict(json.loads(row[0])) if row else None
        except (sqlite3.Error, ValueError, KeyError, TypeError) as exc:
            raise StoreError(str(exc)) from exc

    def all_records(self) -> list[Record]:
        try:
            return [Record.from_dict(json.loads(row[0])) for row in self._db.execute(
                'SELECT payload FROM executions ORDER BY rowid')]
        except (sqlite3.Error, ValueError, KeyError, TypeError) as exc:
            raise StoreError(str(exc)) from exc

    def recover(self, now_ns: int) -> list[Record]:
        recovered = []
        for record in self.all_records():
            if record.record_state == RecordState.ACTIVE:
                record = replace(record, record_state=RecordState.INTERRUPTED,
                                 human_confirmation_required=True, stop_confirmed=False,
                                 result=None, revision=record.revision + 1,
                                 updated_at_ns=now_ns,
                                 message='Process interrupted; physical state needs human confirmation')
                self.save(record)
                recovered.append(record)
        return recovered


class ExecutionJournal:
    """Keep runtime state in memory; persistence errors are diagnostic only.

    Loading existing history remains mandatory at startup so an interrupted
    physical execution cannot be silently forgotten.
    """

    def __init__(self, store: ExecutionStore) -> None:
        self._store = store
        self._records = {record.goal.execution_id: record for record in store.all_records()}
        self._persisted = set(self._records)
        self.write_errors: dict[str, str] = {}

    def get(self, execution_id: str) -> Record | None:
        return self._records.get(execution_id)

    def all_records(self) -> list[Record]:
        return list(self._records.values())

    def _write_failed(self, kind: str, error: Exception) -> None:
        message = str(error)
        if self.write_errors.get(kind) != message:
            LOGGER.warning('%s recording failed; execution state is retained in memory: %s',
                           kind, error)
        self.write_errors[kind] = message

    def _write_succeeded(self, kind: str) -> None:
        if self.write_errors.pop(kind, None) is not None:
            LOGGER.info('%s recording resumed', kind)

    def save(self, record: Record, *, create: bool = False) -> None:
        execution_id = record.goal.execution_id
        if create and execution_id in self._records:
            raise DuplicateExecution(execution_id)
        self._records[execution_id] = record
        try:
            self._store.save(record, create=execution_id not in self._persisted)
        except DuplicateExecution as error:
            if create:
                # A conflicting durable ID is an invalid request, not a write outage.
                del self._records[execution_id]
                raise
            self._write_failed('execution', error)
        except Exception as error:
            # Serialization and closed-connection errors must not become robot faults.
            self._write_failed('execution', error)
        else:
            self._persisted.add(execution_id)
            self._write_succeeded('execution')

    def recover(self, now_ns: int) -> list[Record]:
        recovered = []
        for record in self.all_records():
            if record.record_state == RecordState.ACTIVE:
                record = replace(record, record_state=RecordState.INTERRUPTED,
                                 human_confirmation_required=True, stop_confirmed=False,
                                 result=None, revision=record.revision + 1,
                                 updated_at_ns=now_ns,
                                 message='Process interrupted; physical state needs human confirmation')
                self.save(record)
                recovered.append(record)
        return recovered


def inhibits_execution(record: Record) -> bool:
    return (record.human_confirmation_required
            or record.record_state in (RecordState.INTERRUPTED, RecordState.RECORDING_FAILED)
            or record.object_state in (ObjectState.HELD, ObjectState.UNKNOWN)
            or (record.result is not None and record.result.status == Status.FATAL))
