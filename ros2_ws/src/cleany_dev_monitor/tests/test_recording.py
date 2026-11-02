"""Protect source-independent subscriptions, lossless bytes and policy boundaries."""
import asyncio
import json
import sqlite3
import zlib

import pytest
from aiohttp import ClientSession
from aiohttp.test_utils import TestServer
from cleany_logger import Recorder
from cleany_dev_monitor.core import Hub
from cleany_logger.recording import RecordingControls, defaults, validate, rate_profiles
from logger_fixture import create_app


def enabled(key='point_cloud'):
    settings = defaults()
    settings['enabled'][key] = True
    settings['preset'] = 'standard'
    return settings


def test_raw_is_independent_of_ui_bounded_and_lossless(tmp_path):
    pytest.importorskip('rosbag2_py')
    hub = Hub()
    controls = hub.recording
    controls.configure(enabled())
    assert {'/camera/head/depth/points','/tf','/tf_static'} <= controls.topics() and not hub.selections()
    recorder = Recorder(tmp_path, controls=controls)
    data = b'full CDR bytes with all points\x00\xff'
    ticket = controls.reserve('/camera/head/depth/points')
    assert ticket
    assert controls.reserve('/camera/head/depth/points') is None
    controls.offer(ticket, '/camera/head/depth/points', 'sensor_msgs/msg/PointCloud2', data, 10.)
    recorder.append_raw()
    assert recorder.db.execute("SELECT name FROM sqlite_master WHERE name='raw_messages'").fetchone() is None
    saved = recorder.db.execute('SELECT type,ros_time,received FROM raw_index').fetchone()
    assert saved[:2] == ('sensor_msgs/msg/PointCloud2', 10.)
    assert recorder.raw_message(recorder.session_id, 'point_cloud', saved[2])[0] == data
    assert controls.view()['status']['point_cloud']['count'] == 1
    controls.configure(defaults())
    controls.offer(ticket, '/camera/head/depth/points', 'sensor_msgs/msg/PointCloud2', data, 11.)
    assert not controls.drain() and '/camera/head/depth/points' not in controls.topics()
    recorder.close()




def test_policy_boundary_masks_disabled_stale_rows_and_respects_time(tmp_path):
    hub = Hub()
    recorder = Recorder(tmp_path, controls=hub.recording)
    hub.put('recording_policy', 'settings', defaults(), critical=True)
    first = hub.put('layer', 'local_costmap', {'cells':[100]})
    recorder.append(hub.snapshot())
    settings = defaults()
    settings['enabled']['costmaps'] = False
    hub.recording.configure(settings)
    second = hub.put('recording_policy', 'settings', settings, critical=True)
    hub.put('layer', 'local_costmap', {'cells':[0]})
    recorder.append(hub.snapshot(recorder.last))
    assert any(r['key']=='local_costmap' for r in recorder.replay(recorder.session_id, first['received_at'])['rows'])
    assert not any(r['key']=='local_costmap' for r in recorder.replay(recorder.session_id, second['received_at'])['rows'])
    recorder.close()




def test_settings_http_persistence_origin_and_raw_download(tmp_path):
    pytest.importorskip('rosbag2_py')
    async def scenario():
        hub = Hub()
        recorder = Recorder(tmp_path)
        async with TestServer(create_app(hub, {}, recorder, tmp_path)) as server, ClientSession() as client:
            await client.put(server.make_url("/api/recording/settings"), json=dict(defaults(), active=False))
            settings = enabled()
            response = await client.put(server.make_url('/api/recording/settings'), json=settings)
            assert response.status == 200
            assert json.loads((tmp_path/'recording-settings.json').read_text()) == dict(settings, active=False)
            assert (await client.put(server.make_url('/api/recording/settings'), json=settings, headers={'Origin':'http://other.invalid'})).status == 403
            topic=settings['topics']['point_cloud']
            hub.recording.offer(hub.recording.reserve(topic), topic, 'sensor_msgs/msg/PointCloud2', b'lossless', 1.)
            recorder.append_raw()
            at=recorder.db.execute('SELECT received FROM raw_index').fetchone()[0]
            assert (await client.get(server.make_url(f'/api/recordings/{recorder.session_id}/bag'))).status == 409
            response=await client.get(server.make_url(f'/api/recordings/{recorder.session_id}/raw'), params={'category':'point_cloud','at':str(at)})
            assert response.status == 200 and await response.read() == b'lossless'
            response=await client.get(server.make_url(f'/api/recordings/{recorder.session_id}/raw'), params={'category':'point_cloud','at':str(at-1)})
            assert response.status == 404
            assert recorder.replay(recorder.session_id, at)['rows'][0]['data']['items'][0]['count'] == 1
            previous_id = recorder.session_id
            response = await client.post(server.make_url('/api/recording/session'), json=settings)
            assert response.status == 200 and (await response.json())['session_id'] == previous_id
            assert (tmp_path/f'{previous_id}.sqlite3').exists()
            with sqlite3.connect(tmp_path/f'{previous_id}.sqlite3') as previous:
                assert previous.execute('SELECT count(*) FROM raw_index').fetchone()[0] == 1
        recorder.close()
        newhub = Hub()
        newrecorder = Recorder(tmp_path)
        create_app(newhub, {}, newrecorder, tmp_path)
        assert newhub.recording.view()['settings'] == settings
        newrecorder.close()
    asyncio.run(scenario())


def test_basic_recording_hz_is_per_stream_and_preserves_events(tmp_path):
    hub = Hub()
    settings = defaults()
    settings.pop('source_hz')
    settings['hz']['motion'] = 1.
    settings['hz']['resources'] = .5
    hub.recording.configure(settings)
    recorder = Recorder(tmp_path, controls=hub.recording)
    streams = [('pose', 'robot'), ('sample', '/wheel/odom'), ('resources', 'host'), ('event', 'transition'), ('result', 'mission'), ('log', '/rosout')]
    for index in range(9):
        for kind, key in streams:
            row = hub.put(kind, key, {'index': index}, critical=kind in ('event', 'result', 'log'))
            row['received_monotonic'] = 1000 + index * .25
            recorder.append(hub.snapshot(recorder.last))
    def indices(kind, key):
        return [json.loads(zlib.decompress(payload))['data']['index'] for (payload,) in recorder.db.execute('SELECT payload FROM records WHERE kind=? AND key=? ORDER BY seq', (kind,key))]
    assert indices('pose','robot') == [0,4,8]
    assert indices('sample','/wheel/odom') == [0,4,8]
    assert indices('resources','host') == [0,8]
    for kind,key in streams[3:]:
        assert indices(kind,key) == list(range(9))
    assert hub.latest_data('pose','robot')['index'] == 8
    settings['hz']['motion'] = .1
    hub.recording.configure(settings)
    row = hub.put('pose', 'robot', {'index':99})
    row['received_monotonic'] = 1002.1
    recorder.append(hub.snapshot(recorder.last))
    assert len(indices('pose','robot')) == 4
    recorder.close()






def test_individual_topic_rates_control_saved_samples_and_events(tmp_path):
    from cleany_dev_monitor.sources import resolved
    hub = Hub()
    hub.reset_sources(resolved({}),1)
    settings = defaults()
    settings['source_hz'].update(odometry_topic=1., ground_truth_topic=2., runtime_events_topic=1., runtime_result_topic=2., rosout_topic=.5, map_topic=1.)
    hub.recording.configure(settings)
    recorder = Recorder(tmp_path, controls=hub.recording)
    streams = [('sample','/wheel/odom'), ('sample','/ground_truth/odom'), ('event','/mission/runtime_events'), ('result','/mission/result'), ('log','/rosout'), ('layer','map')]
    for index in range(9):
        for kind,key in streams:
            row = hub.put(kind,key,{'index':index,'topic':'/map' if key=='map' else key},critical=kind in ('event','result','log'))
            row['received_monotonic'] = 1000 + index*.25
            recorder.append(hub.snapshot(recorder.last))
    def indices(kind,key):
        return [json.loads(zlib.decompress(payload))['data']['index'] for (payload,) in recorder.db.execute('SELECT payload FROM records WHERE kind=? AND key=? ORDER BY seq',(kind,key))]
    assert indices('sample','/wheel/odom') == [0,4,8]
    assert indices('sample','/ground_truth/odom') == [0,2,4,6,8]
    assert indices('event','/mission/runtime_events') == [0,4,8]
    assert indices('result','/mission/result') == [0,2,4,6,8]
    assert indices('log','/rosout') == [0,8]
    assert indices('layer','map') == [0,4,8]
    assert hub.latest_data('sample','/wheel/odom')['index'] == 8
    settings['source_hz']['odometry_topic'] = .1
    hub.recording.configure(settings)
    row = hub.put('sample','/wheel/odom',{'index':99})
    row['received_monotonic'] = 1002.1
    recorder.append(hub.snapshot(recorder.last))
    assert indices('sample','/wheel/odom') == [0,4,8,99]
    recorder.close()




def test_event_defaults_preserve_bursts_and_user_can_limit_them(tmp_path, monkeypatch):
    from cleany_dev_monitor.sources import resolved
    hub = Hub()
    hub.reset_sources(resolved({}),1)
    settings = defaults()
    hub.recording.configure(settings)
    recorder = Recorder(tmp_path,controls=hub.recording)
    topics = [('event','/mission/runtime_events'),('result','/mission/result'),('log','/rosout'),('event','/behavior_tree_log')]
    for index in range(9):
        for kind,topic in topics:
            row = hub.put(kind,topic,{'index':index},critical=True)
            row['received_monotonic'] = 1000+index*.01
        recorder.append(hub.snapshot(recorder.last))
    for _,topic in topics:
        assert recorder.db.execute('SELECT count(*) FROM records WHERE key=?',(topic,)).fetchone()[0] == 9
    settings['source_hz']['rosout_topic'] = 1.
    hub.recording.configure(settings)
    for index in range(5):
        row = hub.put('log','/rosout',{'index':index+9},critical=True)
        row['received_monotonic'] = 1001+index*.25
        recorder.append(hub.snapshot(recorder.last))
    assert recorder.db.execute("SELECT count(*) FROM records WHERE key='/rosout'").fetchone()[0] == 11
    recorder.close()


@pytest.mark.parametrize('profile',['low','standard','precise'])
def test_rate_profiles_preserve_topic_selection_and_control_saved_frequency(tmp_path,profile):
    from cleany_dev_monitor.sources import resolved
    hub = Hub()
    hub.reset_sources(resolved({}),1)
    settings = defaults()
    settings['enabled']['camera'] = True
    settings['source_enabled']['path_topic'] = False
    settings['sources']['path_topic'] = '/custom/plan'
    rates = rate_profiles()[profile]
    settings.update(preset=profile,source_hz=rates['source_hz'],hz={**settings['hz'],**rates['hz']})
    hub.recording.configure(settings)
    saved = hub.recording.view()['settings']
    assert saved['enabled']['camera']
    assert not saved['source_enabled']['path_topic']
    assert saved['sources']['path_topic'] == '/custom/plan'
    recorder = Recorder(tmp_path,controls=hub.recording)
    for index in range(21):
        row = hub.put('sample','/wheel/odom',{'index':index})
        row['received_monotonic'] = 1000+index*.125
        event = hub.put('event','/mission/runtime_events',{'index':index},critical=True)
        event['received_monotonic'] = row['received_monotonic']
        recorder.append(hub.snapshot(recorder.last))
    assert recorder.db.execute("SELECT count(*) FROM records WHERE key='/wheel/odom'").fetchone()[0] == {'low': 6, 'standard': 11, 'precise': 21}[profile]
    assert recorder.db.execute("SELECT count(*) FROM records WHERE key='/mission/runtime_events'").fetchone()[0] == 21
    recorder.close()




def test_rates_above_old_ui_cap_are_saved_and_raw_sampler_uses_them(tmp_path, monkeypatch):
    hub = Hub()
    settings = defaults()
    settings['source_hz']['odometry_topic'] = 20.
    settings['enabled']['camera'] = True
    settings['hz']['camera'] = 15.
    hub.recording.configure(settings)
    recorder = Recorder(tmp_path, controls=hub.recording)
    for index in range(9):
        row = hub.put('sample', '/wheel/odom', {'index': index})
        row['source_id'] = 'odometry_topic'
        row['received_monotonic'] = 1000 + index * .03125
        recorder.append(hub.snapshot(recorder.last))
    assert recorder.db.execute("SELECT count(*) FROM records WHERE key='/wheel/odom'").fetchone()[0] == 5
    topic = settings['topics']['camera']
    monkeypatch.setattr('cleany_logger.recording.time.monotonic', lambda: 1000.)
    assert hub.recording.reserve(topic)
    monkeypatch.setattr('cleany_logger.recording.time.monotonic', lambda: 1000.03125)
    assert hub.recording.reserve(topic) is None
    monkeypatch.setattr('cleany_logger.recording.time.monotonic', lambda: 1000.09375)
    assert hub.recording.reserve(topic)
    assert hub.recording.view()['settings']['hz']['camera'] == 15.
    with pytest.raises(ValueError):
        validate({**settings, 'source_hz': {'odometry_topic': 1001.}})
    recorder.close()






def test_low_profile_keeps_short_safety_decision_transitions(tmp_path):
    from cleany_dev_monitor.sources import resolved
    hub = Hub()
    hub.reset_sources(resolved({}), 1)
    settings = defaults()
    settings['preset'] = 'low'
    settings['source_hz'] = rate_profiles()['low']['source_hz']
    hub.recording.configure(settings)
    recorder = Recorder(tmp_path, controls=hub.recording)
    for index, decision in enumerate(('STOP_UNAVAILABLE', 'CLEAR', 'STOP_OBSTACLE', 'CLEAR')):
        row = hub.put('event', '/body_guard_3d/state', {'kind': 'navigation_decision', 'data': {'decision': decision}}, critical=True)
        row['received_monotonic'] = 100 + index * .01
        recorder.append(hub.snapshot(recorder.last))
    rows = recorder.db.execute("SELECT payload FROM records WHERE kind='event' AND key='/body_guard_3d/state' ORDER BY seq").fetchall()
    assert [json.loads(zlib.decompress(r[0]))['data']['data']['decision'] for r in rows] == ['STOP_UNAVAILABLE', 'CLEAR', 'STOP_OBSTACLE', 'CLEAR']
    recorder.close()




def test_new_session_ignores_pre_session_eviction_but_detects_new_loss(tmp_path):
    async def run():
        hub = Hub(event_limit=2)
        recorder = Recorder(tmp_path, controls=hub.recording)
        hub.put('config', 'monitor', {'read_only': True})
        app = create_app(hub, {}, recorder, tmp_path)
        server = TestServer(app)
        await server.start_server()
        try:
            async with ClientSession() as client:
                await client.put(server.make_url('/api/recording/settings'), json=dict(defaults(), active=False))
            for index in range(5):
                hub.put('event', '/mission/runtime_events', {'index': index}, critical=True)
            async with ClientSession() as client:
                response = await client.post(server.make_url('/api/recording/session'), json=defaults())
                assert response.status == 200
            assert recorder.db.execute("SELECT value FROM metadata WHERE key='gap'").fetchone() is None
            assert recorder.db.execute("SELECT count(*) FROM records WHERE kind='config'").fetchone()[0] == 1
            assert recorder.db.execute("SELECT count(*) FROM records WHERE kind='event'").fetchone()[0] == 0
            for index in range(5):
                hub.put('event', '/mission/runtime_events', {'index': index+5}, critical=True)
            recorder.append(hub.snapshot(recorder.last))
            assert recorder.db.execute("SELECT value FROM metadata WHERE key='gap'").fetchone() is not None
        finally:
            await server.close()
            recorder.close()
    asyncio.run(run())


def test_timeline_segments_exclude_pause_and_stale_initial_state_and_include_raw(tmp_path):
    hub = Hub()
    recorder = Recorder(tmp_path)
    def save(kind, key, data, at):
        row = hub.put(kind, key, data, critical=kind == 'recording_policy')
        row['received_at'] = at
        row['received_monotonic'] = at
        recorder.append(hub.snapshot(recorder.last))
    save('layer', 'map', {'stale': True}, 90)
    save('recording_policy', 'settings', {'active': True}, 100)
    save('sample', '/odom', {}, 101)
    save('recording_policy', 'settings', {'active': True, 'hz': 2}, 103)
    save('sample', '/odom', {}, 104)
    save('recording_policy', 'settings', {'active': False}, 105)
    save('recording_status', 'writer', {}, 108)
    save('recording_policy', 'settings', {'active': True}, 110)
    recorder.db.execute('DROP TABLE raw_index')
    recorder.db.execute('CREATE TABLE raw_messages(id INTEGER PRIMARY KEY, category TEXT, topic TEXT, type TEXT, received REAL, ros_time REAL, payload BLOB)')
    recorder.db.execute("INSERT INTO raw_messages(category,topic,type,received,payload) VALUES ('camera','/camera','sensor_msgs/msg/Image',111,?)", (b'raw',))
    recorder.db.commit()
    assert recorder.sessions()[0]['segments'] == [{'start': 101, 'end': 104}, {'start': 111, 'end': 111}]
    recorder.close()


def test_stop_closes_file_and_each_start_creates_a_separate_recording(tmp_path):
    async def scenario():
        hub = Hub()
        recorder = Recorder(tmp_path)
        async with TestServer(create_app(hub, {}, recorder, tmp_path)) as server, ClientSession() as client:
            settings = (await (await client.get(server.make_url('/api/recording/settings'))).json())['settings']
            hub.put('runtime', 'runtime', {'state': 'IDLE'})
            recorder.append(hub.snapshot(recorder.last))
            first = recorder.session_id
            settings['active'] = False
            response = await client.put(server.make_url('/api/recording/settings'), json=settings)
            assert response.status == 200
            assert recorder.db is None
            first_path = tmp_path / f'{first}.sqlite3'
            stopped = first_path.read_bytes()
            hub.put('runtime', 'runtime', {'state': 'WORKING'})
            recorder.append(hub.snapshot(recorder.last))
            settings['limit_mib'] = 200
            response = await client.put(server.make_url('/api/recording/settings'), json=settings)
            assert response.status == 200
            assert first_path.read_bytes() == stopped
            response = await client.post(server.make_url('/api/recording/session'), json=settings)
            assert response.status == 200
            second = (await response.json())['session_id']
            assert second != first and recorder.db is not None
            hub.put('runtime', 'runtime', {'state': 'NAVIGATE_TO_TARGET'})
            recorder.append(hub.snapshot(recorder.last))
            assert recorder.db.execute("SELECT count(*) FROM records WHERE kind='runtime'").fetchone()[0] > 0
            assert first_path.read_bytes() == stopped
            response = await client.put(server.make_url('/api/recording/settings'), json=dict(settings, active=False))
            assert response.status == 200 and recorder.db is None
            # Older clients setting active=True also start a new file.
            response = await client.put(server.make_url('/api/recording/settings'), json=dict(settings, active=True))
            assert response.status == 200
            third = (await response.json())['session_id']
            assert third not in (first, second)
            assert (tmp_path / f'{second}.sqlite3').exists()
            assert (tmp_path / f'{third}.sqlite3').exists()
        recorder.close()
    asyncio.run(scenario())
