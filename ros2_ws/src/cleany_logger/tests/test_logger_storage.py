"""Standalone storage compatibility, quotas and retained evidence."""

import sqlite3
import zlib
from cleany_logger import Recorder


def test_active_sqlite_export_is_a_consistent_independent_backup(tmp_path):
    recorder = Recorder(tmp_path)
    row = dict(seq=1, kind="event", key="mission", received_at=12.0,
               ros_time=2.0, data={"result": "FAILED"})
    recorder.append(dict(sequence=1, rows=[row]))
    target = tmp_path / "export.db"
    recorder.export_sqlite(recorder.session_id, target)
    with sqlite3.connect(target) as copy:
        assert copy.execute("SELECT count(*) FROM records").fetchone()[0] == 1
        copy.execute("DELETE FROM records")
    assert recorder.replay(recorder.session_id, 12.0)["rows"] == [row]
    recorder.close()


def test_legacy_sqlite_payload_stays_readable(tmp_path):
    recorder = Recorder(tmp_path)
    recorder.db.execute("DROP TABLE raw_index")
    recorder.db.execute(
        "CREATE TABLE raw_messages(id INTEGER PRIMARY KEY, category TEXT, topic TEXT, type TEXT, received REAL, ros_time REAL, payload BLOB)"
    )
    recorder.db.execute(
        "INSERT INTO raw_messages VALUES (1,?,?,?,?,?,?)",
        (
            "camera",
            "/old/image",
            "sensor_msgs/msg/Image",
            12.0,
            2.0,
            zlib.compress(b"old cdr"),
        ),
    )
    recorder.db.commit()
    assert recorder.raw_message(recorder.session_id, "camera", 12.0) == (
        b"old cdr",
        "sensor_msgs/msg/Image",
        "/old/image",
    )
    assert (
        recorder.replay(recorder.session_id, 12.0)["rows"][0]["data"]["format"]
        == "legacy_sqlite"
    )
    recorder.close()
