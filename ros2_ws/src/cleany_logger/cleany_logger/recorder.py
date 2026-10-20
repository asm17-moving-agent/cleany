"""Durable observation sessions, storage quotas and historical replay.

The embedding application owns the worker/thread and supplies snapshots. This
module does not import ROS, HTTP, browser or mission execution components.
"""

import hashlib
import json
import os
import sqlite3
import zlib
from pathlib import Path
from uuid import uuid4

from .recording import allowed, category, validate
from .rosbag import RosbagWriter
from .sources import resolved, validate_sources
from .archive import ArchiveReader, decode_record


class Recorder(ArchiveReader):
    """Single-worker storage owner. Never touches MissionJournal or deletes evidence."""

    def __init__(self, directory, session_limit=100 * 1024**2, total_limit=1024**3, controls=None, bag_factory=RosbagWriter, *, open_session=True):
        self.controls = controls
        self.directory = Path(directory).expanduser()
        self.session_limit, self.total_limit = session_limit, total_limit
        self.session_id = str(uuid4())
        self.path = self.directory / f"{self.session_id}.sqlite3"
        self.bag_path = self.directory / 'bags' / self.session_id
        self.bag_factory = bag_factory
        self.bag = None
        self.db = None
        self.reason = ""
        self.last = 0
        self.bytes = 0
        self.hashes = {}
        self.sampled_at = {}
        self.recording_revision = None
        self.source_revision = 0
        self.raw_losses = {}
        self.raw_loss_seen = {}
        self.raw_loss_revision = None
        if not open_session:
            return
        try:
            self.directory.mkdir(parents=True, exist_ok=True)
            if self.disk_bytes() >= total_limit:
                self.reason = "Total recording limit reached; new recording stopped."
                return
            self.db = sqlite3.connect(self.path)
            self.db.execute(
                "CREATE TABLE records(seq INTEGER PRIMARY KEY, received REAL, kind TEXT, key TEXT, payload BLOB)"
            )
            self.db.execute("CREATE INDEX records_time ON records(received)")
            self.db.execute("CREATE TABLE metadata(key TEXT PRIMARY KEY, value TEXT)")
            self.db.execute("CREATE TABLE raw_index(id INTEGER PRIMARY KEY, category TEXT, topic TEXT, type TEXT, received REAL, ros_time REAL, timestamp_ns INTEGER)")
            self.db.execute("CREATE INDEX raw_time ON raw_index(received)")
            self.db.execute("CREATE INDEX raw_locator ON raw_index(topic,timestamp_ns,id)")
            self.db.execute(
                "INSERT INTO metadata VALUES (?, ?)", ("schema_version", "4")
            )
            self.db.executemany('INSERT INTO metadata VALUES (?, ?)', [
                ('raw_storage', 'rosbag2/sqlite3'), ('processed_storage', 'sqlite3'),
                ('bag_uri', f'bags/{self.session_id}'), ('bag_state', 'not_started'),
            ])
            self.db.commit()
        except (OSError, sqlite3.Error) as exc:
            self.reason = f"Recording unavailable: {exc}"
            if self.db:
                self.db.close()
                self.db = None

    def restore_settings(self, config: dict) -> None:
        """Restore valid saved policy; keep defaults when absent or unreadable."""
        if self.controls is None:
            return
        self.controls.source_config = dict(config)
        path = self.directory / "recording-settings.json"
        try:
            loaded = validate(json.loads(path.read_text()))
            validate_sources({**resolved(config), **loaded["sources"]})
            self.controls.configure(loaded)
            self.session_limit = self.controls.settings["limit_mib"] * 1024**2
        except (ValueError, OSError, TypeError):
            pass

    def persist_settings(self, settings: dict) -> None:
        """Atomically persist a policy validated against the producer's sources."""
        path = self.directory / "recording-settings.json"
        temporary = path.with_suffix(".tmp")
        with temporary.open("w") as handle:
            json.dump(settings, handle)
            handle.flush()
            os.fsync(handle.fileno())
        temporary.replace(path)

    def start_session(self) -> None:
        """Finalize the previous session and open a fresh UUID with current limits.

        The producer must seal callback tickets and drain its previous snapshot
        before calling this method, then provide the new session's initial state.
        """
        directory, session_limit, total_limit = self.directory, self.session_limit, self.total_limit
        controls, bag_factory = self.controls, self.bag_factory
        self.close()
        self.__init__(directory, session_limit, total_limit, controls=controls, bag_factory=bag_factory)
        if controls is not None:
            controls.writer_error = self.reason

    def disk_bytes(self):
        return (sum(p.stat().st_size for p in self.directory.glob("*.sqlite3*"))
                + sum(p.stat().st_size for p in (self.directory / 'bags').rglob('*') if p.is_file()))

    def session_bytes(self):
        return ((self.path.stat().st_size if self.path.exists() else 0)
                + sum(p.stat().st_size for p in self.bag_path.rglob('*') if p.is_file()))

    def limit_reached(self, size):
        return (max(self.session_bytes(), self.bytes) + size + 65536 > self.session_limit
                or self.disk_bytes() + size + 65536 > self.total_limit)

    def stop_recording(self, reason):
        self.reason = reason
        if self.controls:
            self.controls.writer_error = reason
        if self.db:
            self.db.execute('INSERT OR REPLACE INTO metadata VALUES (?, ?)', ('stopped_reason', reason))
        self.close_bag()

    def close_bag(self):
        bag, self.bag = self.bag, None
        if bag is not None:
            failed = False
            try:
                bag.close()
            except (OSError, RuntimeError) as exc:
                failed = True
                self.reason = f'rosbag2 finalization failed: {exc}'
                if self.controls:
                    self.controls.writer_error = self.reason
                if self.db:
                    self.db.execute('INSERT OR REPLACE INTO metadata VALUES (?, ?)', ('stopped_reason', self.reason))
            if self.db:
                state = 'finalized' if not failed and (self.bag_path / 'metadata.yaml').is_file() else 'incomplete'
                self.db.execute('INSERT OR REPLACE INTO metadata VALUES (?, ?)', ('bag_state', state))

    def append(self, snapshot: dict) -> None:
        if self.reason or self.db is None:
            return
        rows = [r for r in snapshot["rows"] if r["seq"] > self.last]
        if snapshot.get("gap"):
            self.db.execute(
                "INSERT OR REPLACE INTO metadata VALUES (?, ?)",
                ("gap", "Event ring overflow; recording is incomplete"),
            )
        settings = self.controls.view()["settings"] if self.controls else None
        if self.controls and self.recording_revision != self.controls.revision:
            self.sampled_at.clear()
            self.recording_revision = self.controls.revision
        revision = next((r['data'].get('topic_revision', 0) for r in snapshot['rows'] if r['kind'] == 'config'), self.source_revision)
        if revision != self.source_revision:
            self.source_revision = revision
            self.hashes.clear()
            self.sampled_at.clear()
        for row in rows:
            if settings and not allowed(row, settings):
                continue
            key = (row["kind"], row["key"])
            sample_time = row.get('received_monotonic', row['received_at'])
            identity = row.get('source_id')
            rate = (settings['source_hz'].get(identity) if identity else settings['hz'].get(category(row))) if settings else None
            context = row['kind'] == 'slam_texture' or (row['kind'] == 'slam' and row['key'] == 'submaps') or row['kind'] in ('config', 'graph', 'recording_policy', 'recording_status') or (row['kind'] == 'event' and row['data'].get('kind') == 'navigation_decision')
            limited = rate is not None and not context and (bool(identity) or (row['kind'] not in ('event', 'log', 'result') and category(row) is not None))
            previous = self.sampled_at.get(key)
            if limited and previous is not None and 0 <= sample_time-previous < 1/rate:
                continue
            payload = zlib.compress(
                json.dumps(row, ensure_ascii=False, allow_nan=False).encode(), level=3
            )
            digest = hashlib.sha256(
                json.dumps(row["data"], sort_keys=True).encode()
            ).hexdigest()
            if row["kind"] == "layer" and self.hashes.get(key) == digest:
                continue
            size = len(payload)
            if self.limit_reached(size):
                self.stop_recording('Recording size limit reached; evidence retained.')
                break
            self.db.execute(
                "INSERT OR IGNORE INTO records VALUES (?, ?, ?, ?, ?)",
                (row["seq"], row["received_at"], row["kind"], row["key"], payload),
            )
            self.bytes += size
            if limited:
                self.sampled_at[key] = sample_time
            if self.controls and category(row):
                self.controls.saved(category(row))
            self.hashes[key] = digest
        self.db.commit()
        self.last = snapshot["sequence"]

    def append_raw(self):
        if not self.controls:
            return
        batch = self.controls.drain()
        if self.reason or self.db is None:
            for message in batch:
                self.controls.dropped(message.category)
            self.save_raw_gaps()
            return
        for index, message in enumerate(batch):
            try:
                if self.limit_reached(len(message.payload)):
                    self.stop_recording('Recording size limit reached; evidence retained.')
                else:
                    if self.bag is None:
                        self.bag_path.parent.mkdir(parents=True, exist_ok=True)
                        self.bag = self.bag_factory(self.bag_path)
                        self.db.execute('INSERT OR REPLACE INTO metadata VALUES (?, ?)', ('bag_state', 'recording'))
                    self.bag.write(message)
                    self.db.execute('INSERT INTO raw_index(category,topic,type,received,ros_time,timestamp_ns) VALUES (?,?,?,?,?,?)',
                                    (message.category, message.topic, message.message_type, message.received, message.ros_time, message.timestamp_ns))
                    self.bytes += len(message.payload)
                    self.controls.saved(message.category)
            except (ImportError, OSError, RuntimeError, ValueError, sqlite3.Error) as exc:
                self.stop_recording(f'rosbag2 recording unavailable: {exc}')
            if self.reason:
                for pending in batch[index:]:
                    self.controls.dropped(pending.category)
                break
        self.save_raw_gaps()
        self.db.commit()

    def save_raw_gaps(self):
        if self.db:
            view = self.controls.view()
            if self.raw_loss_revision != view['stats_generation']:
                self.raw_loss_seen.clear()
                self.raw_loss_revision = view['stats_generation']
            changed = False
            for key, stat in view['status'].items():
                dropped = stat.get('dropped', 0)
                delta = dropped - self.raw_loss_seen.get(key, 0)
                if delta > 0:
                    previous = self.raw_losses.get(key, {}).get('dropped', 0)
                    self.raw_losses[key] = {**stat, 'dropped': previous + delta}
                    changed = True
                self.raw_loss_seen[key] = dropped
            if changed:
                self.db.execute('INSERT OR REPLACE INTO metadata VALUES (?,?)', ('raw_gap', json.dumps(self.raw_losses)))
                self.db.commit()

    def close(self):
        self.close_bag()
        if self.db:
            self.db.commit()
            self.db.close()
            self.db = None
