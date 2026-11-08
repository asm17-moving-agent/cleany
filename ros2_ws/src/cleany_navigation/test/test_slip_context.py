import json
import pytest
from cleany_navigation.slip_context import CommandHistory, json_safe


def test_context_uses_past_command_not_current_or_future():
    h = CommandHistory(.3)
    h.add(1., (.1, .2, .3))
    h.add(1.2, (.4, .5, .6))
    assert h.at(1.1)['velocity'] == [.1, .2, .3]
    assert h.at(.9)['reason'] == 'no_prior_sample'
    assert h.at(2.)['velocity'] is None
    assert h.at(2.)['reason'] == 'stale'


def test_history_is_bounded_and_clock_reversal_discards_old_epoch():
    h = CommandHistory()
    for i in range(2000):
        h.add(i*.001, (0., 0., 0.))
    assert len(h.samples) == 512
    h.add(.1, (1., 0., 0.))
    assert len(h.samples) == 1
    h.add(10., (0., 0., 0.))
    assert len(h.samples) == 1
    h.add(11., (float('nan'), 0., 0.))
    assert len(h.samples) == 1


def test_nested_diagnostics_are_strict_json_and_invalid_age_rejected():
    assert json.loads(json.dumps(json_safe({'pose': [1., float('nan')],
        'commands': {'velocity': (float('inf'),)}}), allow_nan=False)) == {
        'pose': [1., None], 'commands': {'velocity': [None]}}
    for age in (0., -1., float('nan'), float('inf')):
        with pytest.raises(ValueError):
            CommandHistory(age)


def test_adapter_looks_up_observation_end_and_does_not_fall_back_to_latest():
    from types import SimpleNamespace as NS
    from cleany_navigation.slip_monitor_node import SlipMonitor
    calls = []
    class Buffer:
        def lookup_transform(self, target, source, when):
            calls.append((target, source, when.nanoseconds))
            return NS(transform=NS(translation=NS(x=1., y=2.),
                rotation=NS(x=0., y=0., z=0., w=1.)))
    history = CommandHistory()
    history.add(9.9, (.1, .2, 0.))
    history.add(10.1, (.5, .6, 0.))
    node = NS(map_frame='map', base='base_link', buffer=Buffer(), map_revision='revision',
              commands={'requested': history}, command_topics={'requested': 'nav2/cmd_vel'})
    result = SlipMonitor.observation_context(node, {'t0': 9., 't1': 10.})
    assert calls == [('map', 'base_link', 10000000000)]
    assert result['pose'] == [1., 2., 0.]
    assert result['commands']['requested']['velocity'] == [.1, .2, 0.]
    def unavailable(*args):
        raise RuntimeError('no historical TF')
    node.buffer.lookup_transform = unavailable
    result = SlipMonitor.observation_context(node, {'t0': 9., 't1': 10.})
    assert result['pose'] is None
    assert result['pose_reason'] == 'historical_transform_unavailable'


def test_map_identity_changes_with_cells_but_not_publication_time():
    from types import SimpleNamespace
    from nav_msgs.msg import OccupancyGrid
    from cleany_navigation.slip_monitor_node import SlipMonitor
    node = SimpleNamespace(map_revision=None)
    m = OccupancyGrid()
    m.header.frame_id = 'map'; m.info.width = 2; m.info.height = 1
    m.info.resolution = .05; m.info.origin.orientation.w = 1.
    m.data = [0, -1]
    SlipMonitor.on_map(node, m)
    first = node.map_revision
    m.header.stamp.sec = 10
    SlipMonitor.on_map(node, m)
    assert node.map_revision == first
    m.data[1] = 100
    SlipMonitor.on_map(node, m)
    assert node.map_revision != first


def test_ros_adapter_constructs_with_its_own_isolated_context():
    import rclpy
    from rclpy.context import Context
    from cleany_navigation.slip_monitor_node import SlipMonitor
    context = Context()
    rclpy.init(context=context, domain_id=193)
    node = None
    try:
        node = SlipMonitor(context=context)
        assert node.context is context
        assert node.recording_context is None
        node.publish()  # Strict JSON heartbeat remains valid during warm-up.
    finally:
        if node is not None:
            node.destroy_node()
        context.shutdown()
