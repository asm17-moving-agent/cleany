"""MuJoCo journal extension for actual native node transitions."""
import sqlite3
import time
from cleany_skill_executor.manipulation.store import ExecutionStore, StoreError


class BTExecutionStore(ExecutionStore):
    def __init__(self, path: str) -> None:
        super().__init__(path)
        try:
            with self._db:
                self._db.execute('CREATE TABLE IF NOT EXISTS bt_transitions ('
                    'sequence INTEGER PRIMARY KEY, execution_id TEXT NOT NULL, '
                    'uid INTEGER NOT NULL, node_id TEXT NOT NULL, previous TEXT NOT NULL, '
                    'status TEXT NOT NULL, observed_at_ns INTEGER NOT NULL)')
        except sqlite3.Error as error:
            self.close()
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
