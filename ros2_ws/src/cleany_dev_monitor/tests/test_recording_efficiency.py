"""Optimizations must preserve quota, remapping, and live catalog semantics."""

import sqlite3
import zlib

import pytest

from cleany_dev_monitor.core import Recorder, directory_bytes
from cleany_dev_monitor.recording import RecordingControls


def test_quota_reads_fresh_sizes_including_other_bags_and_journals(tmp_path):
    recorder = Recorder(tmp_path, session_limit=200_000, total_limit=200_000)
    try:
        (tmp_path / 'previous.sqlite3').write_bytes(b'db' * 31)
        (tmp_path / 'previous.sqlite3-wal').write_bytes(b'journal' * 20)
        previous = tmp_path / 'bags/previous/nested'
        previous.mkdir(parents=True)
        evidence = previous / 'segment.db3'
        evidence.write_bytes(b'x' * 100)
        recorder.bag_path.mkdir()
        own_bag = recorder.bag_path / 'segment.db3'
        own_bag.write_bytes(b'y' * 13)
        # File links count like files; directory links must not recurse.
        (previous / 'file-link').symlink_to(own_bag)
        (previous / 'directory-loop').symlink_to(tmp_path / 'bags', target_is_directory=True)
        expected = sum(p.stat().st_size for p in tmp_path.glob('*.sqlite3*')) + 100 + 13 + 13
        assert recorder.disk_bytes() == expected
        assert recorder.session_bytes() == recorder.path.stat().st_size + 13
        assert not recorder.limit_reached(1)
        evidence.write_bytes(b'x' * 200_000)
        assert recorder.limit_reached(1)  # Another session grew after the last check.
        evidence.write_bytes(b'x' * 100)
        assert not recorder.limit_reached(1)
        own_bag.write_bytes(b'y' * 200_000)
        assert recorder.limit_reached(1)
        assert evidence.read_bytes() == b'x' * 100
    finally:
        recorder.close()


def test_missing_bag_is_empty_but_unreadable_bag_is_not_assumed_empty(tmp_path, monkeypatch):
    assert directory_bytes(tmp_path / 'absent') == 0

    def denied(path):
        raise PermissionError('cannot inspect recording size')

    monkeypatch.setattr('cleany_dev_monitor.core.os.scandir', denied)
    with pytest.raises(PermissionError):
        directory_bytes(tmp_path)


def test_cached_context_changes_atomically_and_rejects_old_callback_ticket():
    controls = RecordingControls()
    supplied = {'map_topic': '/first/map'}
    controls.source_config = supplied
    supplied['map_topic'] = '/unapplied/map'
    exported = controls.source_config
    exported['map_topic'] = '/also_unapplied/map'
    assert '/first/map' in controls.topics()
    with pytest.raises(TypeError):
        controls.context_topics()['/injected'] = ('costmaps', 'invalid', None)
    old_ticket = controls.reserve('/first/map')
    assert old_ticket is not None
    controls.source_config = {'map_topic': '/second/map'}
    controls.offer(old_ticket, '/first/map', 'nav_msgs/msg/OccupancyGrid', b'old', 1.)
    assert controls.drain() == []
    assert controls.reserve('/first/map') is None
    controls.offer(controls.reserve('/second/map'), '/second/map',
                   'nav_msgs/msg/OccupancyGrid', b'current', 2.)
    assert [message.payload for message in controls.drain()] == [b'current']
    settings = controls.policy()
    settings['sources']['map_topic'] = '/overridden/map'
    settings['source_enabled']['path_topic'] = False
    controls.configure(settings)
    assert '/overridden/map' in controls.topics() and '/second/map' not in controls.topics()
    assert '/plan' not in controls.topics()


def test_cached_context_tracks_disable_camera_remapping_and_session_boundary():
    controls = RecordingControls()
    settings = controls.policy()
    settings['enabled'] = dict.fromkeys(settings['enabled'], False)
    controls.configure(settings)
    assert controls.topics() == set()
    assert controls.reserve('/tf') is None
    settings['enabled']['camera'] = True
    settings['topics']['camera'] = '/wrist/preview/compressed'
    controls.configure(settings)
    assert controls.camera_info_topic() == '/wrist/camera_info'
    assert {'/tf', '/tf_static', '/clock', '/wrist/camera_info'} <= controls.topics()
    old_ticket = controls.reserve('/tf_static')
    with controls.transition():
        assert controls.reserve('/tf_static') is None
        controls.offer(old_ticket, '/tf_static', 'tf2_msgs/msg/TFMessage', b'old', 1.)
        assert controls.drain() == []
    controls.offer(controls.reserve('/tf_static'), '/tf_static',
                   'tf2_msgs/msg/TFMessage', b'fresh-session', 2.)
    assert [message.payload for message in controls.drain()] == [b'fresh-session']


@pytest.mark.parametrize('legacy', [False, True])
def test_catalog_aggregates_match_records_during_growth_updates_and_deletion(tmp_path, legacy):
    recorder = Recorder(tmp_path)
    try:
        if legacy:
            recorder.db.execute('DROP INDEX records_kind_time')

        def check():
            recorder.db.commit()
            with sqlite3.connect(recorder.path) as db:
                count, start, end = db.execute('SELECT count(*),min(received),max(received) FROM records').fetchone()
                runtime_count, runtime_start = db.execute("SELECT count(*),min(received) FROM records WHERE kind='runtime'").fetchone()
            summary = recorder.sessions()[0]
            assert [summary[key] for key in ('count', 'start', 'end', 'runtime_count', 'runtime_start')] == [count, start, end, runtime_count, runtime_start]

        check()  # Empty session retains null time bounds.
        recorder.db.executemany('INSERT INTO records VALUES (?,?,?,?,?)', [
            (1, 10., 'runtime', 'state', zlib.compress(b'{}')),
            (2, 3., 'sample', 'sensor', zlib.compress(b'{}')),
            (3, 30., 'runtime', 'state', zlib.compress(b'{}')),
            (4, None, 'runtime', 'state', zlib.compress(b'{}')),
        ])
        check()
        recorder.db.execute('UPDATE records SET received=2 WHERE seq=3')
        check()
        recorder.db.execute('DELETE FROM records WHERE seq=1')
        check()
        recorder.db.execute('DELETE FROM records')
        check()
    finally:
        recorder.close()
