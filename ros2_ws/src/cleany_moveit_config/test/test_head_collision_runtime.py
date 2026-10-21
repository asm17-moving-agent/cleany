"""Check CAD voids without disabling nonadjacent or environment collisions."""
import json
import os
import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get('CLEANY_TEST_OCTOMAP_RUNTIME') != '1',
    reason='Enable with make test-manipulation-fixes (patched shutdown required)',
)

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from shape_msgs.msg import SolidPrimitive
from runtime_support import call, moveit_runtime


def state(pan=0., tilt=0.):
    # Surveyed collision-free reference for the current CAD mast/head geometry.
    result = RobotState()
    result.joint_state.name = [f'{side}_{suffix}_joint' for side in ('left', 'right')
        for suffix in ('shoulder_yaw','shoulder_pitch','elbow_pitch','wrist_pitch','wrist_roll','gripper')]
    result.joint_state.position = [-1.6, 2.87, 3.13, -.4, 1.55, -.2, 1.87, 2.44, 3.08, -1.42, 1.55, -.26]
    result.joint_state.name += ['head_pan_joint', 'head_tilt_joint']
    result.joint_state.position = list(result.joint_state.position) + [pan, tilt]
    return result


def test_head_cad_collision_and_obstacle_detection(tmp_path):
    with moveit_runtime(tmp_path, head=True) as (node, process):
        client = node.create_client(GetStateValidity, '/check_state_validity')
        request = GetStateValidity.Request()
        request.robot_state = state()
        result = call(node, process, client, request)
        pairs = {(c.contact_body_1, c.contact_body_2) for c in result.contacts}
        assert not pairs, pairs
        assert result.valid
        # Keep the nonadjacent pair enabled, even where its geometry does not touch.
        from moveit_msgs.srv import GetPlanningScene
        from moveit_msgs.msg import PlanningSceneComponents
        scene_client = node.create_client(GetPlanningScene, '/get_planning_scene')
        query = GetPlanningScene.Request()
        query.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        matrix = call(node, process, scene_client, query).scene.allowed_collision_matrix
        i, j = matrix.entry_names.index('top_base_link'), matrix.entry_names.index('head_tilt_link')
        assert not matrix.entry_values[i].enabled[j]
        # Survey the full nominal head range, recording genuine geometric contacts.
        # Range endpoints are not automatically declared collision-free.
        sweep = []
        for pan in (-3.2, -1.6, 0., 1.6, 3.2):
            for tilt in (-.76, -.4, 0., .4, .8, 1.2, 1.45):
                request.robot_state = state(pan, tilt)
                result = call(node, process, client, request)
                sweep.append({'pan': pan, 'tilt': tilt, 'valid': result.valid,
                    'contacts': [(c.contact_body_1, c.contact_body_2) for c in result.contacts]})
        assert any(set(pair) == {'top_base_link', 'head_tilt_link'}
                   for sample in sweep for pair in sample['contacts'])
        (tmp_path / 'head_sweep.json').write_text(json.dumps(sweep, indent=2))
        request.robot_state = state()
        # An actual obstacle in the head must still be rejected.
        obstacle = CollisionObject()
        obstacle.header.frame_id = 'base_link'
        obstacle.id = 'head_collision_test_obstacle'
        obstacle.operation = CollisionObject.ADD
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.12,.12,.12])
        pose = Pose(); pose.position.z = .78; pose.orientation.w = 1.
        obstacle.primitives = [box]; obstacle.primitive_poses = [pose]
        apply = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        update = ApplyPlanningScene.Request()
        update.scene.is_diff = True
        update.scene.world.collision_objects = [obstacle]
        assert call(node, process, apply, update).success
        result = call(node, process, client, request)
        assert not result.valid
        assert any('head_collision_test_obstacle' in (c.contact_body_1,c.contact_body_2) for c in result.contacts)
