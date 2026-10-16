from pathlib import Path
from xml.etree import ElementTree as ET
from cleany_navigation.bt_trace import describe_tree, TreeTrace

XML = Path(__file__).parents[1] / 'config/behavior_trees/wait_before_detour.xml'


def make():
    return describe_tree(XML.read_text(), '/runtime/tree.xml')


def event(node, status, previous='IDLE'):
    return dict(node=node, status=status, previous=previous, timestamp=1.)


def test_runtime_names_are_unique_and_ports_topology_preserved():
    xml, manifest = make()
    before = list(ET.fromstring(XML.read_text()).find('BehaviorTree').iter())[1:]
    after = list(ET.fromstring(xml).find('BehaviorTree').iter())[1:]
    assert len(set(n['id'] for n in manifest['nodes'])) == len(before)
    assert len([n for n in manifest['nodes'] if n['type'] == 'ComputePathToPose']) == 3
    for a, b in zip(before, after):
        assert a.tag == b.tag and len(a) == len(b)
        assert {k:v for k,v in a.attrib.items() if k != 'name'} == {k:v for k,v in b.attrib.items() if k != 'name'}
    assert [n['id'] for n in manifest['nodes']] == [n.get('name') for n in after]


def test_duplicate_types_keep_independent_states_and_every_failure():
    _, manifest = make()
    trace = TreeTrace(manifest)
    nodes = [n['id'] for n in manifest['nodes'] if n['type'] == 'ComputePathToPose']
    trace.apply([event(nodes[0], 'RUNNING'), event(nodes[1], 'FAILURE')], 1, 1)
    trace.apply([event(nodes[1], 'FAILURE', 'FAILURE')], 2, 1.1)
    assert trace.states[nodes[0]] == 'RUNNING'
    assert trace.states[nodes[2]] == 'UNKNOWN'
    assert len(trace.history) == 3


def test_goal_lifecycle_and_clock_rewind_do_not_retain_old_states():
    _, manifest = make()
    trace = TreeTrace(manifest)
    node = manifest['nodes'][0]['id']
    trace.lifecycle(True)
    trace.goal('first', 2)
    trace.apply([event(node, 'RUNNING')], 10)
    trace.goal('second', 2)
    assert trace.states[node] == 'UNKNOWN' and not trace.history
    trace.apply([event(node, 'RUNNING')], 11)
    trace.lifecycle(False)
    assert trace.goal_id is None and trace.states[node] == 'UNKNOWN'
    trace.apply([event(node, 'SUCCESS')], 1)
    assert trace.goal_id is None and trace.epoch >= 4


def test_foreign_tree_and_missing_transition_are_explicit():
    _, manifest = make()
    trace = TreeTrace(manifest)
    node = manifest['nodes'][0]['id']
    trace.apply([event(node, 'RUNNING')], 1)
    trace.apply([event(node, 'SUCCESS', 'IDLE')], 2)
    assert trace.incomplete
    trace.apply([event('foreign', 'RUNNING')], 3)
    assert trace.incomplete and not trace.observed
    assert set(trace.states.values()) == {'UNKNOWN'}


def test_history_is_bounded_and_snapshot_is_detached():
    _, manifest = make()
    trace = TreeTrace(manifest)
    node = manifest['nodes'][0]['id']
    for i in range(200):
        trace.apply([event(node, 'FAILURE', 'FAILURE' if i else 'IDLE')], i)
    snapshot = trace.snapshot()
    assert len(snapshot['history']) == 128
    snapshot['states'][node] = 'SUCCESS'
    assert trace.states[node] == 'FAILURE'
