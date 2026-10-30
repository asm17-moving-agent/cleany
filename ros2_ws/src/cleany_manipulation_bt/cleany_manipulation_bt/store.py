"""MuJoCo journal extension for actual native node transitions."""
import sqlite3
import time
import hashlib
import json
from cleany_skill_executor.manipulation.store import ExecutionJournal, ExecutionStore, StoreError
from cleany_skill_executor.manipulation.models import Record


class BTExecutionStore(ExecutionStore):
    def __init__(self, path: str) -> None:
        super().__init__(path)
        try:
            with self._db:
                self._db.execute('CREATE TABLE IF NOT EXISTS bt_transitions ('
                    'sequence INTEGER PRIMARY KEY, execution_id TEXT NOT NULL, '
                    'uid INTEGER NOT NULL, node_id TEXT NOT NULL, previous TEXT NOT NULL, '
                    'status TEXT NOT NULL, observed_at_ns INTEGER NOT NULL)')
                self._db.execute('CREATE TABLE IF NOT EXISTS recovery_confirmations ('
                    'execution_id TEXT NOT NULL, revision INTEGER NOT NULL, '
                    'payload_sha256 TEXT NOT NULL, confirmed_at_ns INTEGER NOT NULL, '
                    'operator_note TEXT NOT NULL, PRIMARY KEY(execution_id, revision))')
        except sqlite3.Error as error:
            self.close()
            raise StoreError(str(error)) from error

    def recovery_confirmations(self) -> dict[tuple[str, int], str]:
        try:
            return {(execution_id, revision): digest for execution_id, revision, digest
                    in self._db.execute('SELECT execution_id, revision, payload_sha256 '
                                        'FROM recovery_confirmations')}
        except sqlite3.Error as error:
            raise StoreError(str(error)) from error

    def confirm_recovery(self, record: Record, operator_note: str) -> None:
        """Durable, revision-bound acknowledgement; retain the original failure."""
        try:
            with self._db:
                # Reserve the write transaction before checking the durable revision.
                self._db.execute('BEGIN IMMEDIATE')
                current = self.get(record.goal.execution_id)
                if current is None or record_digest(current) != record_digest(record):
                    raise StoreError('Execution changed or is not durably recorded; query it again')
                self._db.execute('INSERT OR IGNORE INTO recovery_confirmations VALUES (?, ?, ?, ?, ?)',
                    (record.goal.execution_id, record.revision, record_digest(record),
                     time.time_ns(), operator_note))
                digest = self._db.execute('SELECT payload_sha256 FROM recovery_confirmations '
                    'WHERE execution_id=? AND revision=?', (record.goal.execution_id, record.revision)).fetchone()[0]
                if digest != record_digest(record):
                    raise StoreError('Recovery confirmation conflicts with the recorded execution')
        except sqlite3.Error as error:
            raise StoreError(str(error)) from error

    def save_transitions(self, execution_id: str, transitions: list[dict]) -> None:
        try:
            with self._db:
                self._db.executemany('INSERT INTO bt_transitions '
                    '(execution_id, uid, node_id, previous, status, observed_at_ns) VALUES (?, ?, ?, ?, ?, ?)',
                    [(execution_id, t['uid'], t['node_id'], t['previous'], t['status'], time.time_ns())
                     for t in transitions])
        except sqlite3.Error as error:
            raise StoreError(str(error)) from error


class BTExecutionJournal(ExecutionJournal):
    def __init__(self, store: BTExecutionStore) -> None:
        super().__init__(store)
        self._transition_store = store
        self._confirmed_recoveries = store.recovery_confirmations()

    def recovery_confirmed(self, record: Record) -> bool:
        return self._confirmed_recoveries.get((record.goal.execution_id, record.revision)) == record_digest(record)

    def confirm_recovery(self, record: Record, operator_note: str) -> None:
        # Unlike progress logging, failed acknowledgement persistence must block recovery.
        self._transition_store.confirm_recovery(record, operator_note)
        self._confirmed_recoveries[(record.goal.execution_id, record.revision)] = record_digest(record)

    def save_transitions(self, execution_id: str, transitions: list[dict]) -> None:
        try:
            self._transition_store.save_transitions(execution_id, transitions)
        except Exception as error:
            self._write_failed('BT transition', error)
        else:
            self._write_succeeded('BT transition')


def record_digest(record: Record) -> str:
    return hashlib.sha256(json.dumps(record.to_dict(), sort_keys=True).encode()).hexdigest()
