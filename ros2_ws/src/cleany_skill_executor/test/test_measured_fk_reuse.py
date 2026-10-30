"""Exercise measured FK reuse through the production pose method without hardware."""
from copy import deepcopy
import math
from types import SimpleNamespace as NS

import pytest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from cleany_skill_executor.collection_pipeline import CollectionPipeline
from cleany_skill_executor.core.measured_fk_cache import MeasuredFkCache


DESCRIPTION = '''<robot name="reuse_test">
<joint name="shoulder" type="revolute"><parent link="base_link"/><child link="left_grasp_tcp"/></joint>
<joint name="jaw" type="prismatic"><parent link="left_grasp_tcp"/><child link="finger"/></joint>
</robot>'''


@pytest.fixture
def measured_fk():
    calls, monitors = [], []
    state = JointState(name=['shoulder', 'jaw'], position=[.1, .2])
    stamped = PoseStamped()
    stamped.header.frame_id = 'base_link'
    stamped.pose.position.x = .45
    stamped.pose.orientation.w = 1.
    response = NS(error_code=NS(val=1), pose_stamped=[stamped], fk_link_names=['left_grasp_tcp'])
    node = NS(description=DESCRIPTION, _reuse_measured_fk=True,
              _measured_fk_cache=MeasuredFkCache(), fk=object(),
              real=NS(communication_recovery_count=0,
                      monitor=lambda: (monitors.append(1) or deepcopy(state))),
              pose_message=CollectionPipeline.pose_message,
              cartesian_pose=CollectionPipeline.cartesian_pose)
    node.call = lambda client, request: (calls.append(deepcopy(request)) or deepcopy(response))
    node.pose = lambda link=None: CollectionPipeline.pose(node, 'left', link)
    return node, state, response, calls, monitors


def test_reuse_skips_only_rpc_and_returns_independent_messages(measured_fk):
    node, state, _, calls, monitors = measured_fk
    node.pose().position.x = 123.
    state.header.stamp.nanosec = 20_000_000
    state.name, state.position = ['jaw', 'shoulder'], [.3, .1]
    node.pose().position.x = 456.
    assert node.pose().position.x == .45
    assert len(monitors) == 3 and len(calls) == 1
    assert node._measured_fk_cache_hits == 2
    assert node._measured_fk_cache_misses == 1


def test_cached_pose_still_requires_live_monitor_success(measured_fk):
    node, _, _, calls, _ = measured_fk
    node.pose()

    def fail():
        raise RuntimeError('hardware monitor rejected execution')

    node.real.monitor = fail
    with pytest.raises(RuntimeError, match='hardware monitor rejected execution'):
        node.pose()
    assert len(calls) == 1


def test_changed_joint_model_recovery_and_disabled_reuse_refresh_fk(measured_fk):
    node, state, _, calls, _ = measured_fk
    node.pose()
    state.position[0] = math.nextafter(.1, math.inf)
    node.pose()
    assert len(calls) == 2
    node.description = DESCRIPTION.replace('reuse_test', 'updated')
    node.pose()
    assert len(calls) == 3
    node.real.communication_recovery_count += 1
    node.pose()
    assert len(calls) == 4
    node._reuse_measured_fk = False
    node.pose()
    node.pose()
    assert len(calls) == 6


def test_changed_link_uses_its_own_joint_dependencies(measured_fk):
    node, state, response, calls, _ = measured_fk
    node.pose()
    response.fk_link_names = ['finger']
    node.pose('finger')
    node.pose('finger')
    assert len(calls) == 2
    state.position[1] = .3
    node.pose('finger')
    assert len(calls) == 3


def test_failed_or_mismatched_response_is_not_cached(measured_fk):
    node, _, response, calls, _ = measured_fk
    response.error_code.val = -1
    with pytest.raises(RuntimeError, match='Measured-state FK failed'):
        node.pose()
    response.error_code.val = 1
    response.pose_stamped[0].header.frame_id = 'other_frame'
    node.pose()
    node.pose()
    assert len(calls) == 3
    response.pose_stamped[0].header.frame_id = 'base_link'
    response.fk_link_names = ['other_link']
    node.pose()
    node.pose()
    assert len(calls) == 5


def test_model_change_during_rpc_discards_the_result(measured_fk):
    node, _, _, calls, _ = measured_fk
    original = node.call

    def changed(client, request):
        response = original(client, request)
        node.description = DESCRIPTION.replace('reuse_test', 'updated')
        return response

    node.call = changed
    node.pose()
    node.call = original
    node.pose()
    assert len(calls) == 2


@pytest.mark.parametrize('unusable', ['mimic_chain', 'missing_joint'])
def test_unusable_chain_state_bypasses_reuse(measured_fk, unusable):
    node, state, _, calls, _ = measured_fk
    node.pose()
    if unusable == 'mimic_chain':
        node.description = DESCRIPTION.replace(
            '<child link="left_grasp_tcp"/>', '<child link="left_grasp_tcp"/><mimic joint="jaw"/>')
    else:
        state.name, state.position = ['jaw'], [.2]
    node.pose()
    node.pose()
    assert len(calls) == 3
