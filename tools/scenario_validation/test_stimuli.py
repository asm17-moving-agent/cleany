import math
import json
import subprocess
import xml.etree.ElementTree as ET
import pytest
import stimuli
from stimuli import obstacle_ahead,scene_model_names


def test_omni_lateral_motion_places_obstacle_on_travel_path_not_body_heading():
    current={'x':2.,'y':3.,'yaw':math.pi/2}
    position=obstacle_ahead(current,{'x':1.,'y':3.},1.)
    assert position==[3.,3.,0.]


def test_unknown_motion_direction_is_not_used_for_obstacle_trial():
    with pytest.raises(ValueError):
        obstacle_ahead({'x':1.,'y':2.},{'x':1.,'y':2.},1.)


def test_scene_names_ignore_nested_links_models_and_quoted_braces():
    scene='model { name: "chair_01" link { name: "chair_link" visual { name: "{visual}" } } model { name: "nested" } } model { name: "robot" }'
    assert scene_model_names(scene)=={'chair_01','robot'}


@pytest.mark.parametrize('text',['','model { name: "chair"','model { name: "chair" } model { name: "chair" }','model { id: 1 }'])
def test_incomplete_or_ambiguous_scene_fails_closed(text):
    with pytest.raises(ValueError):scene_model_names(text)


class GazeboTransport:
    """Separate mutation effects from replies, including loss after application."""
    def __init__(self, plans=None):
        self.names={'robot','wall','chair_b','chair_c'}
        self.plans=plans or {}
        self.calls=[]
        self.mutations=[]

    def __call__(self, command, **kwargs):
        service=command[command.index('-s')+1].split('/')[-1]
        self.calls.append(service)
        plans=self.plans.get(service,[])
        behavior=plans.pop(0) if plans else 'ok'
        if service=='info':
            if behavior=='timeout':
                return subprocess.CompletedProcess(command,0,'','Service call timed out\n')
            if behavior=='malformed':
                return subprocess.CompletedProcess(command,0,'model { name: "broken"','')
            return subprocess.CompletedProcess(command,0,
                ''.join('model { name: '+json.dumps(name)+' }' for name in sorted(self.names)),'')
        self.mutations.append(service)
        if behavior=='unexpected':self.names.remove('wall')
        if behavior not in ('unapplied_timeout','unapplied_ack','unexpected'):
            if service=='create':self.names.add('chair_a')
            else:self.names.remove('chair_b')
        if behavior=='process_timeout_applied':raise subprocess.TimeoutExpired(command,6)
        if behavior.endswith('timeout'):
            return subprocess.CompletedProcess(command,0,'','Service call timed out\n')
        return subprocess.CompletedProcess(command,0,'data: true\n','')


def run_transition(monkeypatch,tmp_path,transport):
    monkeypatch.setattr(stimuli.subprocess,'run',transport)
    monkeypatch.setattr(stimuli.time,'sleep',lambda _:None)
    return stimuli.switch_chair('test',ET.fromstring('<model name="chair_a"/>'),
        'chair_b',{'chair_a','chair_b','chair_c'},tmp_path/'transition')


@pytest.mark.parametrize('lost_service,behavior',[
    ('create','applied_timeout'),('remove','applied_timeout'),
    ('create','process_timeout_applied')])
def test_lost_reply_does_not_repeat_applied_mutation(monkeypatch,tmp_path,lost_service,behavior):
    transport=GazeboTransport({lost_service:[behavior]})
    requests=run_transition(monkeypatch,tmp_path,transport)
    assert transport.mutations==['create','remove']
    assert transport.names=={'robot','wall','chair_a','chair_c'}
    assert all(r['scene_confirmed'] for r in requests)
    assert any(not r['acknowledged'] for r in requests)
    evidence=json.loads((tmp_path/'transition/transition.json').read_text())
    assert evidence['verified'] and len(evidence['scenes'])==3


def test_unapplied_request_is_retried_after_scene_readback(monkeypatch,tmp_path):
    transport=GazeboTransport({'create':['unapplied_timeout','ok']})
    requests=run_transition(monkeypatch,tmp_path,transport)
    assert transport.calls==['info','create','info','create','info','remove','info']
    assert len(requests)==3 and not requests[0].get('scene_confirmed')
    assert transport.names=={'robot','wall','chair_a','chair_c'}


def test_failed_readback_is_retried_without_reissuing_mutation(monkeypatch,tmp_path):
    transport=GazeboTransport({'create':['applied_timeout'],'info':['ok','timeout','ok']})
    run_transition(monkeypatch,tmp_path,transport)
    assert transport.mutations==['create','remove']
    records=list((tmp_path/'transition').glob('create-1.pbtxt.attempt-*.json'))
    assert len(records)==2
    assert any('timed out' in json.loads(p.read_text())['stderr'] for p in records)


@pytest.mark.parametrize('behavior',['unapplied_timeout','unapplied_ack'])
def test_missing_effect_fails_closed_even_with_positive_ack(monkeypatch,tmp_path,behavior):
    transport=GazeboTransport({'create':[behavior]*3})
    with pytest.raises(RuntimeError,match='not applied'):
        run_transition(monkeypatch,tmp_path,transport)
    assert transport.mutations==['create']*3
    assert 'chair_b' in transport.names and 'chair_a' not in transport.names
    evidence=json.loads((tmp_path/'transition/transition.json').read_text())
    assert 'error' in evidence and not evidence.get('verified')


def test_unreadable_scene_prevents_any_mutation(monkeypatch,tmp_path):
    transport=GazeboTransport({'info':['timeout']*3})
    with pytest.raises(RuntimeError,match='scene query failed'):
        run_transition(monkeypatch,tmp_path,transport)
    assert transport.mutations==[]


def test_unexpected_scene_change_prevents_following_mutation(monkeypatch,tmp_path):
    transport=GazeboTransport({'create':['unexpected']})
    with pytest.raises(RuntimeError,match='unexpected scene'):
        run_transition(monkeypatch,tmp_path,transport)
    assert transport.mutations==['create']
    assert 'chair_b' in transport.names


def test_wrong_initial_chair_inventory_prevents_mutation(monkeypatch,tmp_path):
    transport=GazeboTransport();transport.names.remove('chair_c')
    with pytest.raises(RuntimeError,match='precondition'):
        run_transition(monkeypatch,tmp_path,transport)
    assert transport.mutations==[]


def test_malformed_scene_is_not_treated_as_an_empty_scene(monkeypatch,tmp_path):
    transport=GazeboTransport({'info':['malformed']})
    with pytest.raises(ValueError):run_transition(monkeypatch,tmp_path,transport)
    assert transport.calls==['info'] and transport.mutations==[]
