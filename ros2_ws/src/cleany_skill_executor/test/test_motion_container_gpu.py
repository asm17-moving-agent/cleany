"""Actual Jetson selector container -> host MoveIt, using mock hardware only."""
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time
from uuid import uuid4

import pytest
import yaml

from cleany_skill_executor.core.physical_grasp_selection import ARM_JOINT_NAMES, JointSolution
from cleany_skill_executor.core.alignment import tcp_axes
from test_curobo_gpu import ros_node
from test_hybrid_gpu import execute, native, wait
from test_hybrid_target_matrix_gpu import apply_obstacle
from hybrid_target_matrix import joint_samples

pytestmark = pytest.mark.skipif(os.environ.get('CLEANY_RUN_MOTION_CONTAINER') != '1',
    reason='Enable with make test-motion-container-gpu on Jetson')


def test_container_action_uses_gpu_and_host_planning_and_cleans_up(native, tmp_path):
    from cleany_interfaces.action import SelectReachableGrasp
    from cleany_interfaces.msg import GraspCandidate
    from moveit_msgs.srv import ApplyPlanningScene
    from rcl_interfaces.srv import GetParameters
    from rclpy.action import ActionClient
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    node, reference, measured, actions = native
    root = Path(__file__).resolve().parents[4]
    owner = 'motion-container-test-' + uuid4().hex[:12]
    environment = dict(os.environ, ROS_DOMAIN_ID=str(node.context.get_domain_id()), CLEANY_MOTION_OWNER=owner)
    # Synthetic target poses keep geometry translation out of this transport test.
    config = tmp_path / 'selector test parameters.yaml'
    config.write_text(yaml.safe_dump({'grasp_selection_server': {'ros__parameters': {
        'require_sensor_scene': False, 'require_pregrasp_visibility': False,
        'asymmetric_gripper_enabled': False, 'pregrasp_offset_m': 0.02,
        'grasp_ik_first': True,
        'pregrasp_joint_shortcut_enabled': True,
        'curobo_ik_cpu_fallback': False, 'curobo_ik_adaptive_batch_size': False,
        'planning_timeout_sec': 10.0, 'action_timeout_sec': 600.0}}}))
    log = tmp_path / 'container-selector.log'
    start = JointSolution(ARM_JOINT_NAMES['left'], (0., .65, .8, -.4, 0.))
    current = dict(zip(measured[-1].name, measured[-1].position))
    trajectory = JointTrajectory(joint_names=list(start.names), points=[
        JointTrajectoryPoint(positions=[current[n] for n in start.names], velocities=[0.]*5),
        JointTrajectoryPoint(positions=list(start.positions), velocities=[0.]*5)])
    trajectory.points[-1].time_from_start.sec = 2
    execute(node, actions['left'], trajectory, measured)
    reference.set_current_state(deepcopy(measured[-1]))
    reference.set_deadline(time.monotonic() + 600.)
    target = reference.forward_kinematics('left', JointSolution(start.names, (.1, .7, .85, -.4, .1)))
    # Reproduce the existing target matrix against the current runtime CAD model;
    # an ignored benchmark artifact must not be required on a fresh checkout.
    reference.set_gripper_position('left', .4)
    samples = joint_samples([reference._joint_limits[name][:2] for name in start.names], seed=20261002)
    positive = []
    for values in samples['interior']:
        witness = JointSolution(start.names, values)
        if reference.state_is_valid('left', witness):
            positive.append(reference.forward_kinematics('left', witness))
        if len(positive) == 8:
            break
    assert len(positive) == 8
    radius = sum(math.sqrt(sum(v*v for v in joint.translation))
                 for joint in reference._local_fk['left'].joints) + .5
    vectors = ((1,0,0), (-1,0,0), (0,1,0), (0,-1,0), (0,0,1), (0,0,-1), (1,1,1), (-1,-1,1))
    outside = [(tuple(radius*v/math.sqrt(sum(x*x for x in vector)) for v in vector),
                target.orientation_xyzw) for vector in vectors]

    def candidates(poses, *, mixed_openings=False):
        result = []
        for index, (position, orientation) in enumerate(poses):
            candidate = GraspCandidate(snapshot_id='container-synthetic', object_id=1,
                                       required_opening_m=.04+(.002*index if mixed_openings else 0.), score=.9)
            candidate.header.frame_id = 'base_link'
            candidate.tcp_pose.position.x, candidate.tcp_pose.position.y, candidate.tcp_pose.position.z = position
            (candidate.tcp_pose.orientation.x, candidate.tcp_pose.orientation.y,
             candidate.tcp_pose.orientation.z, candidate.tcp_pose.orientation.w) = orientation
            approach, _ = tcp_axes(tuple(orientation))
            candidate.approach_direction.x, candidate.approach_direction.y, candidate.approach_direction.z = approach
            candidate.target_object.object_id = 1
            candidate.target_object.label = 'synthetic'
            candidate.target_object.obb_pose.position.x = 3.0
            candidate.target_object.obb_pose.orientation.w = 1.0
            candidate.target_object.obb_size.x = candidate.target_object.obb_size.y = candidate.target_object.obb_size.z = .02
            result.append(candidate)
        return result

    client = ActionClient(node, SelectReachableGrasp, '/grasp/select_reachable')
    apply = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
    parameters = node.create_client(GetParameters, '/grasp_selection_server/get_parameters')
    rows = []
    process = None
    obstacle = False
    try:
        with log.open('w') as output:
            command = ['ros2', 'launch', 'cleany_skill_executor', 'grasp_selection.launch.py',
                f'repository_root:={root}', 'motion_backend:=curobo_ik_moveit', 'motion_runtime:=container',
                f'selector_config_file:={config}']
            process = subprocess.Popen(command, env=environment, stdout=output, stderr=subprocess.STDOUT,
                start_new_session=True)
            deadline = time.monotonic() + 90
            while not client.wait_for_server(timeout_sec=.2):
                assert process.poll() is None and time.monotonic() < deadline, log.read_text()[-6000:]
            assert parameters.wait_for_service(timeout_sec=10)
            values = wait(node, parameters.call_async(GetParameters.Request(names=[
                'motion_backend', 'maximum_candidates', 'curobo_ik_primary_seeds',
                'curobo_ik_fallback_seeds', 'curobo_ik_cpu_fallback',
                'curobo_ik_group_gripper_openings', 'curobo_ik_adaptive_batch_size',
                'curobo_ik_progressive_pregrasp', 'curobo_ik_pregrasp_retry_passes',
                'stop_after_first_feasible', 'curobo_spheres_per_link',
                'grasp_ik_first', 'curobo_sphere_budget_per_link']))).values
            assert values[0].string_value == 'curobo_ik_moveit'
            assert [v.integer_value for v in values[1:4]] == [32, 16, 32]
            assert not values[4].bool_value
            assert values[5].bool_value and not values[6].bool_value and values[7].bool_value
            assert values[8].integer_value == 0
            assert values[9].bool_value and values[10].integer_value == 8
            assert values[11].bool_value
            assert values[12].integer_value == 8

            def request(stage, batch, success):
                began = time.monotonic()
                startup_feedback_retries = 0
                feedback = []
                stages = []
                while True:
                    handle = wait(node, client.send_goal_async(SelectReachableGrasp.Goal(
                        candidates=batch, required_arm='left'),
                        feedback_callback=lambda message: (
                            feedback.append(message.feedback.message), stages.append(message.feedback.stage))))
                    assert handle.accepted
                    response = wait(node, handle.get_result_async(), timeout=650.).result
                    # The action server can be discovered before its first
                    # cross-container JointState arrives. Permit only this
                    # bounded startup condition; incomplete vectors, stale
                    # feedback and planning failures must still fail the test.
                    if (stage == 'first_initialization'
                            and response.error_code == response.ERROR_JOINT_STATE_INCOMPLETE
                            and response.message == 'joint state is incomplete'
                            and time.monotonic() - began < 30.):
                        startup_feedback_retries += 1
                        time.sleep(.1)
                        continue
                    break
                rows.append(dict(stage=stage, seconds=time.monotonic()-began, success=response.success,
                                 error_code=response.error_code, planner_id=response.planner_id,
                                 pregrasp_points=len(response.pregrasp_trajectory.points),
                                 grasp_points=len(response.grasp_trajectory.points), message=response.message,
                                 phases=sorted(set(re.findall(r'phase=(\w+)', ' '.join(feedback)))),
                                 startup_feedback_retries=startup_feedback_retries))
                print('CONTAINER_ACTION ' + json.dumps(rows[-1]), flush=True)
                assert response.success is success, (response.message, log.read_text()[-6000:])
                assert not any('retrying deferred IK' in message for message in feedback)
                if not success:
                    assert response.error_code == response.ERROR_NO_REACHABLE_GRASP, response.message
                if success:
                    assert response.pregrasp_trajectory.points and response.grasp_trajectory.points
                    assert not any('preparing batched pregrasp IK' in message for message in feedback)
                    assert stages.index(SelectReachableGrasp.Feedback.STAGE_GRASP_IK) < stages.index(
                        SelectReachableGrasp.Feedback.STAGE_PREGRASP_IK)

            batch = candidates([(pose.position, pose.orientation_xyzw) for pose in positive],
                               mixed_openings=True)
            request('first_initialization', batch, True)
            request('repeat', batch, True)
            request('outside_reach', candidates(outside), False)
            apply_obstacle(node, apply, target.position, (.3, .3, .3))
            obstacle = True
            request('collision', candidates([(target.position, target.orientation_xyzw)]), False)
            apply_obstacle(node, apply)
            obstacle = False
            statistics = [json.loads(line.split('candidate_ik_statistics=', 1)[1])
                          for line in log.read_text().splitlines() if 'candidate_ik_statistics=' in line]
            assert statistics and statistics[0]['primary_batches'] >= 1
            assert any(call['route'] == 'direct_joint'
                       for item in statistics for call in item['planning_calls'])
            assert statistics[0]['batch_calls'][0]['padded_targets'] == 8
            assert statistics[0]['batch_calls'][0]['stage'] == 'grasp'
            assert len(set(statistics[0]['batch_calls'][0]['actual_gripper_positions'])) == 8
            assert 'GPU_IK' in rows[0]['phases'] and 'MOVEIT_COLLISION' in rows[0]['phases']
            assert all(s['fallback_batches'] == 0 and s['cpu_ik_fallback_calls'] == []
                       for s in statistics)
            assert all(call['seeds'] == 16 for s in statistics for call in s['batch_calls'])
            assert len(statistics[2]['batch_calls']) == 1
            assert len(statistics[3]['batch_calls']) == 1
            # A container crash must end its parent launch as well.
            name = re.search(r'MOTION_CONTAINER name=(\S+)', log.read_text()).group(1)
            subprocess.run(['docker', 'kill', name], check=True, capture_output=True)
            process.wait(timeout=30)
            assert 'Motion selector exited' in log.read_text()
            # Also exercise ordinary ROS launch cancellation on a fresh container.
            process = subprocess.Popen(command, env=environment, stdout=output, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            deadline = time.monotonic() + 90
            while log.read_text().count('Motion backend:') < 2:
                assert process.poll() is None and time.monotonic() < deadline, log.read_text()[-6000:]
                time.sleep(.1)
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=30)
            (tmp_path / 'receipt.json').write_text(json.dumps(dict(rows=rows, statistics=statistics,
                physical_motors_accessed=False, host_moveit=True, container_crash_propagated=True,
                launch_sigint_cleanup=True, rmw=os.environ.get('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')),
                indent=2))
    finally:
        if obstacle:
            apply_obstacle(node, apply)
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=30)
        subprocess.run([str(root / 'tools/motion-container'), 'stop', '--owner', owner], check=True)
        remaining = subprocess.check_output(['docker', 'ps', '-a', '--filter', f'label=cleany.owner={owner}',
                                             '--format', '{{.Names}}'], text=True)
        assert not remaining.strip()
        assert not reference._gpus, 'Host reference must not initialize CUDA'
        client.destroy()
        node.destroy_client(apply)
        node.destroy_client(parameters)
