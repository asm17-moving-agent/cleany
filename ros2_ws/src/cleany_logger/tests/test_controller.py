"""Session ownership and recovery contracts without ROS or a web application."""
import sqlite3

import pytest

from cleany_logger import ArchiveReader
from cleany_logger.controller import ControlConflict, RecordingWorker
from cleany_logger.observation import ObservationBuffer
from cleany_logger.recording import defaults


def config(path):
    return dict(state_directory=str(path), session_limit_bytes=104857600, total_limit_bytes=1073741824)


def test_reader_does_not_create_storage(tmp_path):
    directory = tmp_path / 'absent'
    reader = ArchiveReader(directory)
    assert reader.sessions() == []
    assert not directory.exists()


def test_worker_lifecycle_conflicts_and_directory_lock(tmp_path):
    hub = ObservationBuffer()
    worker = RecordingWorker(hub, config(tmp_path), record_on_start=True)
    worker.start()
    try:
        session = worker.snapshot()['session_id']
        assert worker.submit('start').result(2)['session_id'] == session
        with pytest.raises(ControlConflict, match='Stop recording'):
            worker.submit('configure', settings=defaults()).result(2)
        with pytest.raises(ControlConflict, match='Session changed'):
            worker.submit('stop', session_id='another-session').result(2)
        duplicate = RecordingWorker(ObservationBuffer(), config(tmp_path), record_on_start=True)
        count = len(list(tmp_path.glob('*.sqlite3')))
        with pytest.raises(RuntimeError, match='Another logger'):
            duplicate.start()
        assert len(list(tmp_path.glob('*.sqlite3'))) == count
        hub.put('event', 'test', {'sequence': 1}, critical=True)
        stopped = worker.submit('stop', session_id=session).result(2)
        assert stopped['state'] == 'IDLE'
        with sqlite3.connect(tmp_path / f'{session}.sqlite3') as db:
            assert db.execute("SELECT count(*) FROM records WHERE kind='event'").fetchone()[0] == 1
            assert db.execute("SELECT count(*) FROM records WHERE kind='recording_status'").fetchone()[0] > 0
        worker.submit('configure', settings=dict(defaults(), active=False)).result(2)
        assert worker.submit('start').result(2)['session_id'] != session
    finally:
        worker.close()


def test_recovery_marks_abandoned_bags_incomplete_and_uses_new_session(tmp_path):
    old = tmp_path / 'abandoned.sqlite3'
    with sqlite3.connect(old) as db:
        db.execute('CREATE TABLE metadata(key TEXT PRIMARY KEY, value TEXT)')
        db.execute("INSERT INTO metadata VALUES ('bag_state','recording')")
    worker = RecordingWorker(ObservationBuffer(), config(tmp_path), record_on_start=False)
    worker.start()
    try:
        assert worker.snapshot()['state'] == 'IDLE'
        with sqlite3.connect(old) as db:
            assert db.execute("SELECT value FROM metadata WHERE key='bag_state'").fetchone()[0] == 'incomplete'
        worker.submit('start').result(2)
        assert worker.snapshot()['session_id'] != 'abandoned'
    finally:
        worker.close()
