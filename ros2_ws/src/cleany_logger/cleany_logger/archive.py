"""Read-only archive access. Construction never creates or changes a session."""
import json
import sqlite3
import zipfile
import zlib
from pathlib import Path
from uuid import UUID
from .recording import allowed
from .rosbag import read_cdr


class ArchiveReader:
    def __init__(self, directory):
        self.directory = Path(directory).expanduser()
        self.session_id = ""
        self.bag = None
        self.reason = ""

    def sessions(self) -> list[dict]:
        result = []
        for path in sorted(
            self.directory.glob("*.sqlite3"),
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        ):
            with sqlite3.connect(f"file:{path}?mode=ro", uri=True) as db:
                count, start, end = db.execute(
                    "SELECT count(*),min(received),max(received) FROM records"
                ).fetchone()
                runtime_count, runtime_start = db.execute(
                    "SELECT count(*),min(received) FROM records WHERE kind='runtime'"
                ).fetchone()
                meta = dict(db.execute("SELECT key,value FROM metadata"))
                policies = [decode_record(r[0]) for r in db.execute(
                    "SELECT payload FROM records WHERE kind='recording_policy' ORDER BY received,seq")]
                raw_table = self.raw_table(db)
                segments = []
                def add_segment(left, right):
                    bounds = []
                    for table in (['records', raw_table] if raw_table else ['records']):
                        where = "received>=? AND received<?"
                        if table == 'records':
                            where += " AND kind NOT IN ('config','graph','recording_policy','recording_status')"
                        # Indexed first/last reads avoid scanning each active period every poll.
                        first = db.execute(f"SELECT received FROM {table} WHERE {where} ORDER BY received ASC LIMIT 1", (left, right)).fetchone()
                        last = db.execute(f"SELECT received FROM {table} WHERE {where} ORDER BY received DESC LIMIT 1", (left, right)).fetchone()
                        if first and last:
                            bounds.append((first[0], last[0]))
                    if bounds:
                        segments.append(dict(start=min(a for a, _ in bounds), end=max(b for _, b in bounds)))
                if policies:
                    opened = None
                    for policy in policies:
                        if policy['data'].get('active', True):
                            if opened is None:
                                opened = policy['received_at']
                        elif opened is not None:
                            add_segment(opened, policy['received_at'])
                            opened = None
                    if opened is not None:
                        add_segment(opened, float('inf'))
                else:
                    add_segment(float('-inf'), float('inf'))
            result.append(
                {
                    "id": path.stem,
                    "count": count,
                    "runtime_count": runtime_count,
                    "runtime_start": runtime_start,
                    "start": start,
                    "end": end,
                    "bytes": path.stat().st_size + sum(p.stat().st_size for p in (self.directory / 'bags' / path.stem).rglob('*') if p.is_file()),
                    "bag_available": self.bag_available(path.stem, meta),
                    "bag_recording": path.stem == self.session_id and self.bag is not None,
                    "metadata": meta,
                    "segments": segments,
                }
            )
        return result

    def resolve(self, session: str) -> Path:
        # UUID names only; no arbitrary filesystem paths from browser.
        if str(UUID(session)) != session:
            raise ValueError("Invalid session ID")
        path = self.directory / f"{session}.sqlite3"
        if not path.is_file():
            raise FileNotFoundError(session)
        return path

    @staticmethod
    def raw_table(db):
        tables = {row[0] for row in db.execute("SELECT name FROM sqlite_master WHERE type='table'")}
        return 'raw_index' if 'raw_index' in tables else 'raw_messages' if 'raw_messages' in tables else None

    def resolve_bag(self, session):
        self.resolve(session)
        return self.directory / 'bags' / session

    def bag_available(self, session, metadata=None):
        if metadata is None:
            with sqlite3.connect(f'file:{self.resolve(session)}?mode=ro', uri=True) as db:
                metadata = dict(db.execute('SELECT key,value FROM metadata'))
        return (metadata.get('bag_state') == 'finalized'
                and (self.directory / 'bags' / session / 'metadata.yaml').is_file())

    def export_sqlite(self, session: str, target: Path) -> None:
        """Back up committed data consistently, including an active session."""
        source = self.resolve(session)
        with (
            sqlite3.connect(f"file:{source}?mode=ro", uri=True) as src,
            sqlite3.connect(target) as dst,
        ):
            src.backup(dst)

    def export_bag(self, session: str, target: Path) -> None:
        """Archive finalized native segments only; never copy an active writer."""
        directory = self.resolve_bag(session)
        if session == self.session_id and self.bag is not None:
            raise RuntimeError("Stop recording before exporting rosbag2")
        if not self.bag_available(session):
            raise FileNotFoundError("No finalized rosbag2 recording")
        with zipfile.ZipFile(target, "w", compression=zipfile.ZIP_STORED) as archive:
            for path in sorted(directory.iterdir()):
                if path.is_file() and (path.name == "metadata.yaml" or path.suffix == ".db3"):
                    archive.write(path, f"{session}/{path.name}")

    def raw_message(self, session, key, at):
        with sqlite3.connect(f'file:{self.resolve(session)}?mode=ro', uri=True) as db:
            table = self.raw_table(db)
            if table is None:
                raise FileNotFoundError('원본 녹화 없음')
            column = 'timestamp_ns' if table == 'raw_index' else 'payload'
            row = db.execute(f'SELECT {column},type,topic,id FROM {table} WHERE category=? AND received<=? ORDER BY received DESC,id DESC LIMIT 1', (key, at)).fetchone()
            occurrence = (db.execute('SELECT count(*) FROM raw_index WHERE topic=? AND timestamp_ns=? AND id<?',
                                     (row[2], row[0], row[3])).fetchone()[0]
                          if row is not None and table == 'raw_index' else 0)
        if row is None:
            raise FileNotFoundError('이 시점 이전에 원본 녹화가 없습니다.')
        data = read_cdr(self.resolve_bag(session), row[2], row[0], occurrence) if table == 'raw_index' else zlib.decompress(row[0])
        return data, row[1], row[2]

    def replay(self, session: str, at: float) -> dict:
        with sqlite3.connect(f"file:{self.resolve(session)}?mode=ro", uri=True) as db:
            latest = db.execute(
                """SELECT payload FROM records WHERE seq IN (
                SELECT max(seq) FROM records WHERE received <= ? AND kind NOT IN ('event','log','result') GROUP BY kind,key)""",
                (at,),
            )
            rows = [decode_record(r[0]) for r in latest]
            rows += [
                decode_record(r[0])
                for r in db.execute(
                    """SELECT payload FROM records
                WHERE received <= ? AND kind IN ('event','log','result') ORDER BY seq DESC LIMIT 500""",
                    (at,),
                )
            ]
            meta = dict(db.execute("SELECT key,value FROM metadata"))
            table = self.raw_table(db)
            if table:
                raw = [dict(category=k, count=count, last_received=last, topic=topic, message_type=typ)
                       for k,count,last,topic,typ in db.execute(f"SELECT category,count(*),max(received),topic,type FROM {table} WHERE received<=? GROUP BY category,topic,type", (at,))]
                if raw:
                    rows.append(dict(seq=0, kind='recording_raw', key='archive',
                                     data={'items':raw, 'format':'rosbag2' if table == 'raw_index' else 'legacy_sqlite',
                                           'bag_available': self.bag_available(session, meta),
                                           'bag_state': meta.get('bag_state'),
                                           'bag_recording': session == self.session_id and self.bag is not None},
                                     received_at=at, ros_time=None))
        revision = next((r['data'].get('topic_revision', 0) for r in rows if r['kind'] == 'config'), 0)
        rows = [r for r in rows if r['kind'] in ('config','resources','recording_policy','recording_status','recording_raw','event','log','result') or r.get('topic_revision',0) == revision]
        policy = next((r['data'] for r in rows if r['kind'] == 'recording_policy'), None)
        if policy:
            rows = [r for r in rows if r['kind'] in ('recording_raw', 'recording_policy') or allowed(r, policy)]
        return {
            "rows": sorted(rows, key=lambda r: r["seq"]),
            "replay": True,
            "metadata": meta,
        }

    def navigation_runs(self) -> list[dict]:
        """Bounded summaries only; never scan/deserialize every recorded sample."""
        runs = []
        for session in self.sessions()[:50]:
            with sqlite3.connect(f"file:{self.resolve(session['id'])}?mode=ro", uri=True) as db:
                row = db.execute("SELECT payload FROM records WHERE kind='navigation' AND key='summary' ORDER BY seq DESC LIMIT 1").fetchone()
                config = db.execute("SELECT payload FROM records WHERE kind='config' ORDER BY seq DESC LIMIT 1").fetchone()
            if not row:
                continue
            data = decode_record(row[0])["data"]
            settings = decode_record(config[0])["data"] if config else {}
            context = {key: settings.get(key) for key in (
                "run_label", "source_revision", "requested_rtf", "stationary_speed_mps",
                "creep_speed_mps", "navigation_freshness_s", "map_frame", "velocity_topics")}
            for run in data.get("runs", []):
                runs.append({**run, "session_id": session["id"], "configuration": context,
                             "recording_gap": session["metadata"].get("gap")})
        return runs


def decode_record(payload: bytes | str) -> dict:
    """Version 1 was plain JSON; version 2 stores zlib-compressed UTF-8 JSON."""
    return json.loads(
        zlib.decompress(payload) if isinstance(payload, bytes) else payload
    )
