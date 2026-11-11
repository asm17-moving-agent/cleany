"""Project edge depth into the real updater and check collision/path rejection."""
import os
import json
import time
from copy import deepcopy

import numpy as np
import pytest

if os.environ.get('CLEANY_TEST_OCTOMAP_RUNTIME') != '1':
    pytest.skip('Run with make test-full-view (isolated MoveIt, no hardware)', allow_module_level=True)

from geometry_msgs.msg import TransformStamped
from moveit_msgs.msg import Constraints, JointConstraint, PlanningSceneComponents, RobotState
from moveit_msgs.srv import GetMotionPlan, GetPlanningScene, GetPositionFK, GetStateValidity
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
from tf2_ros import StaticTransformBroadcaster
from runtime_support import call, moveit_runtime
from cleany_perception.depth_scene_cloud import depth_scene_cloud


def folded_state():
    state = RobotState()
    state.joint_state.name = [f'{side}_{name}_joint' for side in ('left', 'right')
        for name in ('shoulder_yaw', 'shoulder_pitch', 'elbow_pitch', 'wrist_pitch', 'wrist_roll', 'gripper')]
    state.joint_state.position = [-1.6, 2.87, 3.13, -.4, 1.55, -.2,
                                  1.87, 2.44, 3.08, -1.42, 1.55, -.26]
    state.joint_state.name += ['head_pan_joint', 'head_tilt_joint']
    state.joint_state.position = list(state.joint_state.position) + [
        .023009711818284616, .4049709280018093]
    return state


def test_edge_obstacle_is_mapped_and_blocks_arm_goal(tmp_path):
    with moveit_runtime(tmp_path, head=True, octomap=True) as (node, process):
        current = folded_state()
        joint_pub = node.create_publisher(JointState, '/joint_states', 10)
        cloud_pub = node.create_publisher(PointCloud2, '/perception/scene_cloud', 10)
        def publish_joints():
            current.joint_state.header.stamp = node.get_clock().now().to_msg()
            joint_pub.publish(current.joint_state)
        joint_timer = node.create_timer(.05, publish_joints)
        validity = node.create_client(GetStateValidity, '/check_state_validity')
        fk = node.create_client(GetPositionFK, '/compute_fk')
        query = node.create_client(GetPlanningScene, '/get_planning_scene')
        planner = node.create_client(GetMotionPlan, '/plan_kinematic_path')
        # Registering the full CAD self-filter can exceed the service helper's
        # 60 s default on Jetson. Keep publishing synthetic joints during startup.
        startup_deadline = time.monotonic() + 180.
        while not validity.service_is_ready():
            assert process.poll() is None, 'MoveIt exited during startup'
            assert time.monotonic() < startup_deadline, 'MoveIt startup exceeded 180 s'
            node.executor.spin_once(timeout_sec=.05)

        def valid(state):
            return call(node, process, validity, GetStateValidity.Request(robot_state=state))
        def pose(state):
            request = GetPositionFK.Request(robot_state=state, fk_link_names=['left_lower_arm_link'])
            request.header.frame_id = 'base_link'
            response = call(node, process, fk, request)
            assert response.error_code.val == response.error_code.SUCCESS
            point = response.pose_stamped[0].pose.position
            return np.array([point.x, point.y, point.z])

        baseline = valid(current)
        initial_contacts = [(c.contact_body_1, c.contact_body_2) for c in baseline.contacts]
        if not baseline.valid:
            # Test fixtures must use a geometrically valid state; do not loosen
            # the collision matrix to compensate for an invalid head posture.
            print({'initial_fixture_contacts': initial_contacts})
            for tilt in (0., -.4, .6, .8, 1.):
                trial = deepcopy(current)
                trial.joint_state.position[-1] = tilt
                baseline = valid(trial)
                if baseline.valid:
                    current = trial
                    break
        assert baseline.valid, [(c.contact_body_1, c.contact_body_2) for c in baseline.contacts]
        current_point = pose(current)
        candidate = None
        for positions in [(-1.6, 1.4, 2.5, -.4, 1.55), (-.8, 1.4, 2.2, -.4, 1.55),
                          (-1.6, 2., 2.4, -.4, 1.55), (-1., 1.6, 1.8, -.4, 1.55)]:
            trial = deepcopy(current)
            trial.joint_state.position = list(positions) + list(current.joint_state.position[5:])
            if valid(trial).valid and np.linalg.norm(pose(trial)-current_point) > .20:
                candidate = trial
                break
        assert candidate is not None, 'No separate collision-free arm probe state'
        obstacle = pose(candidate)

        # A synthetic optical frame places the edge patch through the future
        # forearm, while the measured robot remains folded for self-filtering.
        depth_m, focal, cx, cy = .6, 600., 320., 240.
        patch_center = np.array([(40.-cx)*depth_m/focal, 0., depth_m])
        origin = obstacle - patch_center
        broadcaster = StaticTransformBroadcaster(node)
        transform = TransformStamped()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'full_view_test_optical'
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.transform.rotation.w = 1.
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = map(float, origin)
        broadcaster.sendTransform(transform)
        values = np.zeros((480, 640), dtype='<f4')
        values[200:280, 0:80] = depth_m
        image = Image(height=480, width=640, encoding='32FC1', step=640*4, data=values.tobytes())
        image.header.frame_id = transform.child_frame_id
        image.header.stamp = node.get_clock().now().to_msg()
        info = CameraInfo(height=480, width=640, k=[focal,0.,cx,0.,focal,cy,0.,0.,1.])
        info.header = image.header
        cropped = depth_scene_cloud(image, info, .1, 2., 4,
                                     workspace_roi_pixels=[140.,90.,500.,430.])
        full = depth_scene_cloud(image, info, .1, 2., 4)
        assert cropped.width == 0
        assert full.width == 400
        def publish_cloud():
            full.header.stamp = node.get_clock().now().to_msg()
            cloud_pub.publish(full)
        cloud_timer = node.create_timer(.1, publish_cloud)
        started = time.monotonic()
        deadline = started + 25.
        scene_request = GetPlanningScene.Request()
        scene_request.components.components = PlanningSceneComponents.OCTOMAP
        while True:
            scene = call(node, process, query, scene_request).scene
            collision = valid(candidate)
            if scene.world.octomap.octomap.data and any(
                    '<octomap>' in (c.contact_body_1, c.contact_body_2) for c in collision.contacts):
                break
            assert time.monotonic() < deadline, 'Edge patch not reflected in OctoMap collision check'
        assert not collision.valid
        map_collision_latency = time.monotonic()-started
        assert valid(current).valid, 'The edge patch must not collide with the measured folded robot'
        motion = GetMotionPlan.Request()
        motion.motion_plan_request.group_name = 'left_arm'
        motion.motion_plan_request.start_state = current
        motion.motion_plan_request.allowed_planning_time = 1.
        motion.motion_plan_request.num_planning_attempts = 1
        motion.motion_plan_request.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=name, position=value,
                            tolerance_above=.001, tolerance_below=.001, weight=1.)
            for name, value in zip(candidate.joint_state.name[:5], candidate.joint_state.position[:5])])]
        planning_started = time.monotonic()
        planned = call(node, process, planner, motion).motion_plan_response
        assert planned.error_code.val != planned.error_code.SUCCESS
        metrics = {'full_points': full.width, 'cropped_points': cropped.width,
               'map_bytes': len(scene.world.octomap.octomap.data),
               'map_collision_latency_sec': map_collision_latency,
               'planning_latency_sec': time.monotonic()-planning_started,
               'planning_error_code': planned.error_code.val,
               'initial_fixture_contacts': initial_contacts,
               'synthetic_head_pan_tilt_rad': list(current.joint_state.position[-2:])}
        (tmp_path/'map_metrics.json').write_text(json.dumps(metrics, indent=2)+'\n')
        print(metrics)
        node.destroy_timer(cloud_timer)
        node.destroy_timer(joint_timer)
