"""Compare original/reduced arm meshes in real MoveIt/FCL, without drivers."""
from copy import deepcopy
import json
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

if os.environ.get('CLEANY_TEST_ARM_MESH_RUNTIME') != '1':
    pytest.skip('Enable with make test-arm-collision-runtime', allow_module_level=True)

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, Constraints, JointConstraint, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetPositionFK, GetStateValidity
from shape_msgs.msg import SolidPrimitive
from runtime_support import call, moveit_runtime
from test_head_collision_runtime import state


PACKAGE = Path(__file__).resolve().parents[2] / 'cleany_description'
# Keep probe coverage independent of the optimization list.
PROBE_MESHES = {
    'Upper_Arm_5cm.stl', 'Lower_Arm_5cm.stl', 'Base.stl', 'Rotation_Pitch.stl',
    'Wrist_Pitch_Roll.stl', 'Base_Motor.stl', 'Rotation_Pitch_Motor.stl',
    'Upper_Arm_Motor.stl', 'Lower_Arm_Motor.stl', 'Wrist_Pitch_Roll_Motor.stl',
    'Fixed_Jaw_Motor.stl',
    'Fixed_Jaw_part1.ply.convex.stl', 'Fixed_Jaw_part2.ply.convex.stl',
    'Moving_Jaw_part1.ply.convex.stl', 'Moving_Jaw_part2.ply.convex.stl', 'Moving_Jaw_part3.ply.convex.stl',
    'cad_frame/topbase1.stl', 'topbase2.stl', 'tophead1.stl', 'tophead4.stl', 'tophead5.stl', 'tophead6.stl',
}


def source_resource(collision):
    return collision.find('geometry/mesh').get('filename').split('/meshes/')[1].removeprefix('collision/')


def test_reduced_arm_geometry_preserves_collision_detection_and_planning(tmp_path):
    outcomes = {}
    candidate_trajectory = None
    for mode in ('true', 'false'):
        directory = tmp_path / mode
        directory.mkdir()
        model = directory / 'robot.urdf'
        model.write_bytes(subprocess.check_output([
            'xacro', str(PACKAGE / 'urdf/cleany.urdf.xacro'), 'include_wheel_joints:=false',
            'head_collision_meshes:=true', f'optimized_arm_collision_meshes:={mode}']))
        tree = ET.parse(model).getroot()
        collision_resources = [(link.get('name'), collision)
            for link in tree.findall('link') for collision in link.findall('collision')
            if collision.find('geometry/mesh') is not None
            and source_resource(collision) in PROBE_MESHES]
        links = list(dict.fromkeys(name for name, _ in collision_resources))
        assert len(collision_resources) == 38
        assert {source_resource(collision) for _, collision in collision_resources} == PROBE_MESHES
        report = {'random_valid': [], 'surface_contacts': [], 'surface_contact_details': [],
                  'planning_success': False}
        with moveit_runtime(directory, head=True, robot_model_file=model) as (node, process):
            validity = node.create_client(GetStateValidity, '/check_state_validity')
            fk = node.create_client(GetPositionFK, '/compute_fk')
            apply = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
            planner = node.create_client(GetMotionPlan, '/plan_kinematic_path')
            current = state()

            def check(sample):
                return call(node, process, validity, GetStateValidity.Request(robot_state=sample))

            initial = check(current)
            assert initial.valid, [(c.contact_body_1, c.contact_body_2) for c in initial.contacts]
            rng = np.random.default_rng(937)
            for _ in range(128):
                sample = deepcopy(current)
                sample.joint_state.position = [float(rng.uniform(
                    float(tree.find(f"joint[@name='{name}']/limit").get('lower')),
                    float(tree.find(f"joint[@name='{name}']/limit").get('upper'))))
                    for name in sample.joint_state.name]
                report['random_valid'].append(check(sample).valid)

            request = GetPositionFK.Request(robot_state=current, fk_link_names=links)
            request.header.frame_id = 'base_link'
            response = call(node, process, fk, request)
            assert response.error_code.val == response.error_code.SUCCESS
            poses = dict(zip(response.fk_link_names, response.pose_stamped))
            for link, collision in collision_resources:
                posed = poses[link]
                pose = posed.pose
                rotation = Rotation.from_quat([pose.orientation.x, pose.orientation.y,
                    pose.orientation.z, pose.orientation.w]).as_matrix()
                translation = np.array([pose.position.x, pose.position.y, pose.position.z])
                origin = collision.find('origin')
                placement = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy'), sep=' ')).as_matrix()
                offset = np.fromstring(origin.get('xyz'), sep=' ')
                # Always use the original CAD as the independent obstacle fixture.
                filename = source_resource(collision)
                raw = (PACKAGE / 'meshes' / filename).read_bytes()
                dtype = np.dtype([('n', '<f4', (3,)), ('v', '<f4', (3, 3)), ('a', '<u2')])
                vertices = np.frombuffer(raw, dtype=dtype, offset=84)['v'].reshape(-1, 3).astype(float)
                vertices *= np.fromstring(collision.find('geometry/mesh').get('scale', '1 1 1'), sep=' ')
                for direction in np.vstack((np.eye(3), -np.eye(3))):
                    point = vertices[np.argmax(vertices @ direction)]
                    for gap in (-.002, .002, .02):
                        position = rotation @ (placement @ (point + gap * direction) + offset) + translation
                        obstacle = CollisionObject(id='arm_mesh_surface_probe', operation=CollisionObject.ADD)
                        obstacle.header.frame_id = 'base_link'
                        obstacle.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[.003])]
                        p = Pose(); p.orientation.w = 1.
                        p.position.x, p.position.y, p.position.z = map(float, position)
                        obstacle.primitive_poses = [p]
                        update = ApplyPlanningScene.Request()
                        update.scene.is_diff = True
                        update.scene.world.collision_objects = [obstacle]
                        assert call(node, process, apply, update).success
                        contacts = check(current).contacts
                        report['surface_contacts'].append(any('arm_mesh_surface_probe' in
                            (contact.contact_body_1, contact.contact_body_2) for contact in contacts))
                        report['surface_contact_details'].append({
                            'source_link': link, 'source_mesh': filename,
                            'direction': direction.tolist(), 'gap_m': gap,
                            'obstacle_position': position.tolist(),
                            'pairs': [[contact.contact_body_1, contact.contact_body_2] for contact in contacts],
                        })
            update = ApplyPlanningScene.Request()
            update.scene.is_diff = True
            update.scene.world.collision_objects = [CollisionObject(id='arm_mesh_surface_probe', operation=CollisionObject.REMOVE)]
            assert call(node, process, apply, update).success
            goal = deepcopy(current)
            goal.joint_state.position[4] += .05
            assert check(goal).valid
            motion = GetMotionPlan.Request()
            motion.motion_plan_request.group_name = 'left_arm'
            motion.motion_plan_request.start_state = current
            motion.motion_plan_request.allowed_planning_time = 2.
            motion.motion_plan_request.num_planning_attempts = 1
            motion.motion_plan_request.goal_constraints = [Constraints(joint_constraints=[
                JointConstraint(joint_name=name, position=value, tolerance_above=.001,
                    tolerance_below=.001, weight=1.)
                for name, value in zip(goal.joint_state.name[:5], goal.joint_state.position[:5])])]
            began = time.monotonic()
            planned = call(node, process, planner, motion).motion_plan_response
            report['planning_rpc_sec'] = time.monotonic() - began
            report['planning_success'] = planned.error_code.val == planned.error_code.SUCCESS
            assert report['planning_success'], planned.error_code.val
            if mode == 'true':
                candidate_trajectory = planned.trajectory.joint_trajectory
            else:
                # Recheck every state of the reduced-model path in original CAD.
                for point in candidate_trajectory.points:
                    sample = deepcopy(current)
                    for name, value in zip(candidate_trajectory.joint_names, point.positions):
                        sample.joint_state.position[sample.joint_state.name.index(name)] = value
                    assert check(sample).valid
                report['candidate_path_states_checked_in_original'] = len(candidate_trajectory.points)
        outcomes[mode] = report
    (tmp_path / 'comparison.json').write_text(json.dumps(outcomes, indent=2) + '\n')
    original, reduced = outcomes['false'], outcomes['true']
    assert not any(not old and new for old, new in zip(original['random_valid'], reduced['random_valid']))
    assert any(original['surface_contacts']) and not all(original['surface_contacts'])
    assert not any(old and not new for old, new in zip(original['surface_contacts'], reduced['surface_contacts']))
    print({'random_states': 128, 'surface_probes': len(original['surface_contacts']),
        'additional_rejections': sum(old and not new for old, new in zip(original['random_valid'], reduced['random_valid'])),
        'additional_surface_contacts': sum(not old and new for old, new in zip(original['surface_contacts'], reduced['surface_contacts'])),
        'candidate_path_states_checked_in_original': original['candidate_path_states_checked_in_original']})
