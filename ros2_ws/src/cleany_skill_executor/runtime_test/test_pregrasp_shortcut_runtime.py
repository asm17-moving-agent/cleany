"""Native MoveIt comparison; mock controllers only and no trajectory submission."""
from copy import deepcopy
from dataclasses import replace
import json
import os
import time

import pytest
from test_curobo_gpu import ros_node
from test_hybrid_gpu import native
from test_hybrid_target_matrix_gpu import apply_obstacle

pytestmark = pytest.mark.skipif(os.environ.get('CLEANY_TEST_PIPELINE_RUNTIME') != '1',
                              reason='Requires mock MoveIt/controllers and CLEANY_TEST_PIPELINE_RUNTIME=1')


@pytest.mark.parametrize('arm', ['left', 'right'])
def test_shortcut_uses_full_moveit_checks_and_rejects_a_blocked_scene(native, arm, tmp_path):
    from moveit_msgs.srv import ApplyPlanningScene
    from cleany_skill_executor.core.physical_grasp_selection import ARM_JOINT_NAMES, JointSolution
    from cleany_skill_executor.physical_moveit_adapter import MoveItGraspAdapter

    node, adapter, measured, _ = native
    state = deepcopy(measured[-1])
    names = ARM_JOINT_NAMES[arm]
    start = (0., .65, .8, -.4, 0.)
    for name, value in zip(names, start): state.position[state.name.index(name)] = value
    adapter.set_current_state(state)
    adapter.set_gripper_position(arm, .4)
    adapter.set_deadline(time.monotonic()+120.)
    assert adapter.state_is_valid(arm, JointSolution(names, start))
    goal = JointSolution(names, (.05, .7, .85, -.4, .1))
    assert adapter.state_is_valid(arm, goal)
    original = adapter._config
    rows = []
    apply = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
    try:
        for shortcut in (False, True, False, True):
            adapter._config = replace(original, pregrasp_joint_shortcut_enabled=shortcut,
                                      planning_attempts=1)
            began = time.monotonic()
            result = MoveItGraspAdapter.plan(adapter, arm, goal, None)
            rows.append({'shortcut': shortcut, 'route': adapter.last_plan_route,
                         'wall_sec': time.monotonic()-began})
            assert result is not None
            assert adapter.last_plan_route == ('direct_joint' if shortcut else 'rrt')
            assert result.points[-1] == pytest.approx(goal.positions, abs=1e-4)
        target = adapter.forward_kinematics(arm, goal)
        apply_obstacle(node, apply, target.position, (.3, .3, .3))
        adapter._config = replace(adapter._config, planning_timeout_sec=.5)
        assert MoveItGraspAdapter.plan(adapter, arm, goal, None) is None
        assert adapter.last_plan_route == 'rrt'  # A failed direct check takes the normal fallback.
        (tmp_path/'planning-times.json').write_text(json.dumps(rows, indent=2))
    finally:
        apply_obstacle(node, apply)
        node.destroy_client(apply)
        adapter._config = original
