import asyncio

import pytest
from aiohttp import ClientSession
from aiohttp.test_utils import TestServer
from cleany_logger import Recorder
from cleany_dev_monitor.core import Hub
from cleany_logger.recording import defaults, allowed
from cleany_dev_monitor.sources import resolved, routes, validate_sources
from logger_fixture import create_app


def test_namespace_and_custom_roles_rebuild_actual_routes():
    config = resolved({'runtime_namespace': '/robot', 'manipulation_events_topic': 'arm/events'})
    assert config['runtime_debug_topic'] == '/robot/mission/debug_snapshot'
    assert config['manipulation_events_topic'] == '/robot/arm/events'
    overrides = dict(path_topic='/custom/plan', runtime_debug_topic='/custom/state', rosout_topic='/custom/log', velocity_topics=['/custom/command'], reactive_guard_topic='/custom/guard')
    c, fixed, layers, navigation = routes({**config, **overrides})
    assert '/custom/state' in fixed and '/custom/log' in fixed
    assert '/robot/mission/debug_snapshot' not in fixed
    assert '/plan' not in layers and layers['/custom/plan'] == 'path'
    assert '/reactive_guard/state' not in navigation and navigation['/custom/guard'] == 'reactive_guard'
    assert c['velocity_topics'] == ['/custom/command']


@pytest.mark.parametrize('value', [{'path_topic': 'relative'}, {'path_topic': '/wrong//path'}, {'path_topic': '/bad;cmd'}, {'unknown': '/a'}, {'velocity_topics': []}, {'path_topic': '/same', 'map_topic': '/same'}])
def test_invalid_or_ambiguous_topics_rejected(value):
    with pytest.raises(ValueError):
        validate_sources(value)


def test_route_change_clears_live_and_replayed_stale_state(tmp_path):
    hub = Hub()
    recorder = Recorder(tmp_path, controls=hub.recording)
    hub.put('config', 'monitor', resolved({}))
    old = hub.put('layer', 'path', {'points': ['old']})
    recorder.append(hub.snapshot())
    hub.reset_sources({**resolved({}), 'path_topic': '/new/plan'}, 1)
    after = hub.latest[('config', 'monitor')]['received_at']
    assert hub.latest_data('layer', 'path') == {}
    recorder.append(hub.snapshot(recorder.last))
    assert any(r['key'] == 'path' for r in recorder.replay(recorder.session_id, old['received_at'])['rows'])
    assert not any(r['key'] == 'path' for r in recorder.replay(recorder.session_id, after)['rows'])
    row = hub.put('sample', '/reactive_guard/state', {'decision': 'STOP'})
    assert row['category'] == 'safety'
    summary = hub.put('navigation', 'scan', {'topic': '/scan'})
    assert summary['category'] == 'safety'
    assert allowed(summary, defaults())  # Raw LiDAR remains OFF; summary is independent.
    settings = defaults()
    settings['enabled']['costmaps'] = False
    assert not allowed(hub.put('layer', 'map', {'topic': '/map'}), settings)
    fresh = hub.put('layer', 'path', {'points': ['old']})
    recorder.append(hub.snapshot(recorder.last))
    assert any(r['key'] == 'path' for r in recorder.replay(recorder.session_id, fresh['received_at'])['rows'])
    recorder.close()


def test_topic_settings_persist_validate_and_expose_applied_pending(tmp_path):
    async def scenario():
        hub = Hub()
        config = {**resolved({}), 'read_only': True}
        hub.put('config', 'monitor', config)
        recorder = Recorder(tmp_path)
        server = TestServer(create_app(hub, config, recorder, tmp_path))
        await server.start_server()
        async with ClientSession() as client:
            response = await client.get(server.make_url('/api/recording/settings'))
            initial = await response.json()
            assert initial['source_fields']['path_topic']['group'] == 'paths'
            settings = initial['settings']
            settings['active'] = False
            await client.put(server.make_url('/api/recording/settings'), json=settings)
            settings['sources']['path_topic'] = '/custom/plan'
            settings['source_hz']['path_topic'] = .5
            settings['source_hz']['local_path_topic'] = 3.
            response = await client.put(server.make_url('/api/recording/settings'), json=settings)
            value = await response.json()
            assert response.status == 200 and not value['source_pending']
            assert value['source_applied']['path_topic'] == '/custom/plan'
            hub.reset_sources({**config, **settings['sources']}, 1)
            value = await (await client.get(server.make_url('/api/recording/settings'))).json()
            assert not value['source_pending']
            assert value['source_applied']['path_topic'] == '/custom/plan'
            invalid = {**settings, 'sources': {'path_topic': '/map'}}
            assert (await client.put(server.make_url('/api/recording/settings'), json=invalid)).status == 400
            assert hub.recording.view()['settings']['sources']['path_topic'] == '/custom/plan'
        await server.close()
        recorder.close()
        second = Hub()
        second_recorder = Recorder(tmp_path)
        create_app(second, config, second_recorder, tmp_path)
        assert second.recording.view()['settings']['sources']['path_topic'] == '/custom/plan'
        assert second.recording.view()['settings']['source_hz']['path_topic'] == .5
        assert second.recording.view()['settings']['source_hz']['local_path_topic'] == 3.
        second_recorder.close()
    asyncio.run(scenario())


def test_per_source_recording_selection_does_not_hide_live_observation(tmp_path):
    hub = Hub()
    hub.reset_sources(resolved({}), 1)
    settings = defaults()
    settings['source_enabled'] = {'path_topic': False, 'runtime_events_topic': False, 'velocity_topics:0': False}
    hub.recording.configure(settings)
    recorder = Recorder(tmp_path, controls=hub.recording)
    hub.put('recording_policy', 'settings', settings, critical=True)
    path = hub.put('layer', 'path', {'topic': '/plan', 'points': [1]})
    local = hub.put('layer', 'local_path', {'topic': '/local_plan', 'points': [2]})
    event = hub.put('event', '/mission/runtime_events', {'kind': 'transition'}, critical=True)
    result = hub.put('result', '/mission/result', {'status': 'SUCCESS'}, critical=True)
    cmd = hub.put('sample', '/nav2/cmd_vel', {'speed': 1})
    enabled_cmd = hub.put('sample', '/safety_2d/cmd_vel', {'speed': 1})
    recorder.append(hub.snapshot())
    rows = recorder.replay(recorder.session_id, result['received_at']+1)['rows']
    sequences = {r['seq'] for r in rows}
    assert not {path['seq'],event['seq'],cmd['seq']} & sequences
    assert {local['seq'],result['seq'],enabled_cmd['seq']} <= sequences
    assert hub.latest_data('layer','path')['points'] == [1]
    recorder.close()
