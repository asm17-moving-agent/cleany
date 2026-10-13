"""Protect source-independent subscriptions, lossless bytes and policy boundaries."""
import asyncio
import json
import sqlite3
import zlib

import pytest
from aiohttp import ClientSession
from aiohttp.test_utils import TestServer
from cleany_dev_monitor.core import Hub, Recorder
from cleany_dev_monitor.recording import RecordingControls, defaults, validate
from cleany_dev_monitor.server import create_app


def enabled(key='point_cloud'):
    settings = defaults()
    settings['enabled'][key] = True
    settings['preset'] = 'custom'
    return settings


def test_raw_is_independent_of_ui_bounded_and_lossless(tmp_path):
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
    saved = recorder.db.execute('SELECT type,ros_time,payload FROM raw_messages').fetchone()
    assert saved[:2] == ('sensor_msgs/msg/PointCloud2', 10.) and zlib.decompress(saved[2]) == data
    assert controls.view()['status']['point_cloud']['count'] == 1
    controls.configure(defaults())
    controls.offer(ticket, '/camera/head/depth/points', 'sensor_msgs/msg/PointCloud2', data, 11.)
    assert not controls.drain() and '/camera/head/depth/points' not in controls.topics()
    recorder.close()


def test_queue_loss_and_size_limit_are_explicit(tmp_path):
    controls = RecordingControls(queue_limit=4)
    controls.configure(enabled())
    ticket = controls.reserve('/camera/head/depth/points')
    controls.offer(ticket, '/camera/head/depth/points', 'sensor_msgs/msg/PointCloud2', b'12345', 1.)
    assert controls.view()['status']['point_cloud']['dropped'] == 1
    recorder = Recorder(tmp_path, controls=controls, session_limit=1024)
    controls.offer(ticket, '/camera/head/depth/points', 'sensor_msgs/msg/PointCloud2', b'12', 1.)
    recorder.append_raw()
    assert recorder.reason and recorder.path.exists()
    assert recorder.db.execute('SELECT count(*) FROM raw_messages').fetchone()[0] == 0
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


@pytest.mark.parametrize('change', [{'hz':None}, {'limit_mib':0}, {'active':'yes'}, {'hz':{'point_cloud':float('nan')}}])
def test_invalid_configuration_is_rejected(change):
    with pytest.raises(ValueError):
        validate(dict(defaults(), **change))


def test_settings_http_persistence_origin_and_raw_download(tmp_path):
    async def scenario():
        hub = Hub()
        recorder = Recorder(tmp_path)
        async with TestServer(create_app(hub, {}, recorder, tmp_path)) as server, ClientSession() as client:
            settings = enabled()
            response = await client.put(server.make_url('/api/recording/settings'), json=settings)
            assert response.status == 200
            assert json.loads((tmp_path/'recording-settings.json').read_text()) == settings
            assert (await client.put(server.make_url('/api/recording/settings'), json=settings, headers={'Origin':'http://other.invalid'})).status == 403
            topic=settings['topics']['point_cloud']
            hub.recording.offer(hub.recording.reserve(topic), topic, 'sensor_msgs/msg/PointCloud2', b'lossless', 1.)
            recorder.append_raw()
            at=recorder.db.execute('SELECT received FROM raw_messages').fetchone()[0]
            response=await client.get(server.make_url(f'/api/recordings/{recorder.session_id}/raw'), params={'category':'point_cloud','at':str(at)})
            assert response.status == 200 and await response.read() == b'lossless'
            response=await client.get(server.make_url(f'/api/recordings/{recorder.session_id}/raw'), params={'category':'point_cloud','at':str(at-1)})
            assert response.status == 404
            assert recorder.replay(recorder.session_id, at)['rows'][0]['data']['items'][0]['count'] == 1
            previous_id = recorder.session_id
            response = await client.post(server.make_url('/api/recording/session'), json=settings)
            assert response.status == 200 and (await response.json())['session_id'] != previous_id
            assert (tmp_path/f'{previous_id}.sqlite3').exists()
            with sqlite3.connect(tmp_path/f'{previous_id}.sqlite3') as previous:
                assert previous.execute('SELECT count(*) FROM raw_messages').fetchone()[0] == 1
        recorder.close()
        newhub = Hub()
        newrecorder = Recorder(tmp_path)
        create_app(newhub, {}, newrecorder, tmp_path)
        assert newhub.recording.view()['settings'] == settings
        newrecorder.close()
    asyncio.run(scenario())
