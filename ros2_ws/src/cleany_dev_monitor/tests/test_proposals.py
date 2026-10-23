"""A reviewed target is immutable, single-use and revalidated before dispatch."""

import asyncio
from copy import deepcopy
import json
from pathlib import Path
import time
from types import SimpleNamespace as NS
from uuid import uuid4

import pytest

from cleany_dev_monitor.operator_jobs import approved_command, atomic_json
from cleany_dev_monitor.proposals import DEFAULTS, ProposalStore, fingerprint, scene_context, validate_fresh_target
from test_operator import environment


def scene():
    item = NS(object_id=7, track_id='track-a', tracking_state='TRACKED', label='cup',
        sorting_category='trash',  confidence=.9,
        distance_valid=True, distance_m=.4, position_valid=True,
        representative_position=NS(x=.4, y=0., z=.2), x_min=10., y_min=10., x_max=70., y_max=90.,
        median_depth_m=.4, estimated_width_m=.05, estimated_height_m=.07)
    return NS(snapshot_id='new-scene', tracking_session_id='session', tracking_epoch='epoch',
              representative_frame='base_link', detections=[item])


def proposal():
    observed = scene()
    return dict(id=str(uuid4()), state='READY', created_at=time.time()-1, expires_at=time.time()+60,
        snapshot_id='approved-scene', tracking_session_id='session', tracking_epoch='epoch',
        representative_frame='base_link', scene_context=scene_context(observed), scene_object_count=1,
        skill_name='collect_trash', destination_id='return_to_pick',
        config_fingerprint=fingerprint({'label_destinations': {'*': 'return_to_pick'}}),
        target=dict(object_id=3, track_id='track-a', label='cup', category='trash',
                    bbox=[10., 10., 70., 90.], position=[.4, 0., .2]))


def test_revalidation_accepts_same_track_with_new_local_number_but_never_changes_approved_goal():
    config = {'label_destinations': {'*': 'return_to_pick'}}
    approved = proposal()
    before = deepcopy(approved)
    validate_fresh_target(approved, scene(), config, DEFAULTS, time.time())
    assert approved == before
    command = approved_command(Path('/repo'), dict(id=str(uuid4()), proposal=approved, proposal_policy=DEFAULTS))
    assert '--auto-select' not in command
    assert command[command.index('--snapshot-id')+1] == 'approved-scene'
    assert command[command.index('--object-id')+1] == '3'


@pytest.mark.parametrize('change', ['moved', 'missing', 'ambiguous', 'category', 'label', 'epoch',
                                   'camera', 'bbox', 'nan', 'depth', 'duplicate_target',
                                   'replacement_target', 'config', 'expired'])
def test_revalidation_blocks_changed_or_uncertain_observations(change):
    approved, fresh = proposal(), scene()
    config = {'label_destinations': {'*': 'return_to_pick'}}
    item = fresh.detections[0]
    if change == 'moved': item.representative_position.x += .05
    if change == 'missing': fresh.detections.clear()
    if change == 'ambiguous': item.tracking_state = 'AMBIGUOUS'
    if change == 'category': item.sorting_category = 'lost_item'
    if change == 'label': item.label = 'bottle'
    if change == 'epoch': fresh.tracking_epoch = 'new-epoch'
    if change == 'camera': fresh.representative_frame = 'camera'
    if change == 'bbox': item.x_min += 30
    if change == 'nan': item.representative_position.x = float('nan')
    if change == 'depth': item.distance_valid = False
    if change == 'duplicate_target': fresh.detections.append(deepcopy(item))
    if change == 'replacement_target': item.track_id = 'different-track'
    if change == 'config': config['operation_mode'] = 'changed'
    if change == 'expired': approved['expires_at'] = time.time()-1
    with pytest.raises(ValueError):
        validate_fresh_target(approved, fresh, config, DEFAULTS, time.time())


@pytest.mark.parametrize('change', ['added', 'missing', 'moved', 'label', 'category',
                                   'ambiguous', 'invalid_position'])
def test_revalidation_accepts_unchanged_target_when_other_detections_change(change):
    approved, fresh = proposal(), scene()
    neighbor = deepcopy(fresh.detections[0])
    neighbor.object_id, neighbor.track_id, neighbor.label = 8, 'neighbor', 'book'
    neighbor.representative_position.x = .7
    fresh.detections.append(neighbor)
    approved.update(scene_context=scene_context(fresh), scene_object_count=2)
    if change == 'added':
        approved.update(scene_context=scene_context(scene()), scene_object_count=1)
        # A newly visible, closer candidate must not replace the approved target.
        neighbor.distance_m = .1
    if change == 'missing': fresh.detections.remove(neighbor)
    if change == 'moved': neighbor.representative_position.x += .1
    if change == 'label': neighbor.label = 'cell phone'
    if change == 'category': neighbor.sorting_category = 'review'
    if change == 'ambiguous': neighbor.tracking_state = 'AMBIGUOUS'
    if change == 'invalid_position': neighbor.position_valid = False
    before = deepcopy(approved)
    validate_fresh_target(approved, fresh, {'label_destinations': {'*': 'return_to_pick'}},
                          DEFAULTS, time.time())
    assert approved == before


def test_consumed_proposal_stays_consumed_after_restart_and_beyond_recent_job_list(tmp_path):
    store = ProposalStore(tmp_path)
    approved = proposal()
    store.save(approved)
    assert store.claimable(approved['id']) == approved
    jobs = tmp_path / 'jobs'
    jobs.mkdir()
    request_id = str(uuid4())
    atomic_json(jobs / f'{request_id}.json', dict(id=request_id, action='approve',
        args={'proposal_id': approved['id']}, state='FAILED'))
    for _ in range(25):
        atomic_json(jobs / f'{uuid4()}.json', dict(action='inspect', args={}, state='SUCCEEDED'))
    restarted = ProposalStore(tmp_path)
    assert restarted.current()['state'] == 'CONSUMED'
    with pytest.raises(ValueError): restarted.claimable(approved['id'])
    new = proposal()
    restarted.save(new)
    with pytest.raises(ValueError): restarted.claimable(approved['id'])
    assert restarted.claimable(new['id']) == new


def test_approval_http_is_authenticated_single_use_and_server_bound(tmp_path, monkeypatch):
    async def scenario():
        async with environment(tmp_path, monkeypatch) as (control, port, jobs, client, url, origin, login):
            approved = proposal()
            control.proposals.save(approved, b'image-fixture')
            request = dict(request_id=str(uuid4()), action='approve', args={'proposal_id': approved['id']})
            assert (await client.post(url('jobs'), json=request, headers=origin)).status == 401
            assert (await client.get(url(f"proposals/{approved['id']}/image"))).status == 401
            await login()
            assert await (await client.get(url(f"proposals/{approved['id']}/image"))).read() == b'image-fixture'
            invalid = deepcopy(request)
            invalid['args']['object_id'] = 99
            assert (await client.post(url('jobs'), json=invalid, headers=origin)).status == 400
            responses = await asyncio.gather(*[client.post(url('jobs'), json=request, headers=origin) for _ in range(2)])
            assert sorted(r.status for r in responses) == [200, 202]
            assert len(jobs) == 1 and jobs[0]['proposal'] == approved
            path = control.jobs / f"{request['request_id']}.json"
            atomic_json(path, {**json.loads(path.read_text()), 'state': 'FAILED'})
            response = await client.post(url('jobs'), json={**request, 'request_id': str(uuid4())}, headers=origin)
            assert response.status == 400
            assert not port.calls
    asyncio.run(scenario())


def test_expired_proposal_and_new_observation_invalidate_approval(tmp_path, monkeypatch):
    async def scenario():
        async with environment(tmp_path, monkeypatch) as (control, port, jobs, client, url, origin, login):
            await login()
            expired = proposal()
            expired['expires_at'] = time.time()-1
            control.proposals.save(expired)
            request = dict(request_id=str(uuid4()), action='approve', args={'proposal_id': expired['id']})
            assert (await client.post(url('jobs'), json=request, headers=origin)).status == 400
            approved = proposal()
            control.proposals.save(approved)
            response = await client.post(url('jobs'), json=dict(request_id=str(uuid4()), action='propose', args={}), headers=origin)
            assert response.status == 202
            assert control.proposals.current() is None
            assert jobs[-1]['proposal']['id'] == approved['id']
    asyncio.run(scenario())


@pytest.mark.parametrize('changed', [False, True])
def test_worker_checks_before_dispatch_and_next_proposal_never_executes(tmp_path, monkeypatch, changed):
    from cleany_dev_monitor.operator_jobs import execute_job
    import cleany_dev_monitor.proposal_ros as adapter
    from test_operator import Port
    calls = []
    approved = proposal()
    store = ProposalStore(tmp_path)
    store.save(approved)
    (tmp_path / 'enabled').touch()
    (tmp_path / 'jobs').mkdir()
    request_id = str(uuid4())
    path = tmp_path / 'jobs' / f'{request_id}.json'
    atomic_json(path, dict(id=request_id, action='approve', args={'proposal_id': approved['id']},
        proposal=approved, proposal_policy=DEFAULTS, state='PENDING', created_at=time.time()))

    class Adapter(Port):
        def __init__(self, *_): super().__init__()
        def revalidate(self, value, cancelled):
            calls.append('revalidate')
            assert value == approved
            if changed: raise ValueError('scene changed')
        def propose(self, cancelled, **kwargs):
            calls.append('propose')
            assert kwargs['last_result']['execution_id'] == request_id
            value = proposal()
            store.save(value)
            return value
        def close(self): pass

    class Process:
        pid = 1234
        returncode = 0
        def __init__(self, command, **kwargs):
            calls.append(command)
            path.with_suffix('.log').write_text(json.dumps(dict(found=True, record=dict(
                has_result=True, execution_id=request_id, status='SUCCESS', stop_confirmed=True,
                arm_recovered=True, placement_state='ASSUMED'))))
        def poll(self): return 0

    monkeypatch.setattr(adapter, 'ProposalROS', Adapter)
    monkeypatch.setattr('cleany_dev_monitor.operator_jobs.runtime_info', lambda: dict(managed=True, pipeline_busy=False))
    monkeypatch.setattr('cleany_dev_monitor.operator_jobs.subprocess.Popen', Process)
    execute_job(path, tmp_path)
    result = json.loads(path.read_text())
    if changed:
        assert result['state'] == 'FAILED' and calls == ['revalidate']
    else:
        assert result['state'] == 'SUCCEEDED'
        assert len([value for value in calls if isinstance(value, list)]) == 1
        assert calls[0] == 'revalidate' and calls[-1] == 'propose'
        assert store.current()['id'] != approved['id'] and store.current()['state'] == 'READY'
