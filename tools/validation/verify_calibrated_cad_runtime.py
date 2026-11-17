#!/usr/bin/env python3
"""Exercise CAD FK, collision and planning on an isolated, driver-free MoveIt."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--diagnostic-cam-opening-deg', type=float)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '214' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise ValueError('This check requires isolated localhost ROS domain 214')
    args.output.mkdir(parents=True, exist_ok=False)
    from cleany_description.cad_coordinates import CadCoordinates
    from cleany_skill_executor.core.urdf_fk import UrdfChain
    import rclpy
    from moveit_msgs.msg import CollisionObject, Constraints, JointConstraint, RobotState
    from moveit_msgs.srv import ApplyPlanningScene, GetPositionFK, GetStateValidity, GetMotionPlan
    from geometry_msgs.msg import Pose
    from shape_msgs.msg import SolidPrimitive

    xml = args.urdf.read_text()
    cad = CadCoordinates.from_urdf(xml)
    if cad is None:
        raise ValueError('A calibrated CAD model is required')
    limits = {j.get('name'): tuple(float(j.find('limit').get(k)) for k in ('lower', 'upper'))
              for j in ET.fromstring(xml).findall('joint')
              if j.get('type') == 'revolute' and j.find('mimic') is None}
    log = (args.output/'move-group.log').open('w')
    process = subprocess.Popen(['ros2', 'launch', 'cleany_moveit_config', 'physical_move_group.launch.py',
        'robot_description_file:='+str(args.urdf), 'use_rviz:=false',
        'allow_trajectory_execution:=false', 'include_head_camera:=true',
        'enable_depth_octomap:=false'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    result = {'domain_id': 214, 'motor_driver_started': False, 'trajectory_goals_sent': 0}
    rclpy.init()
    node = rclpy.create_node('calibrated_cad_no_motion_check')

    def call(client, request):
        deadline = time.monotonic()+90.
        while not client.wait_for_service(timeout_sec=.2):
            if process.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('Driver-free MoveIt failed to become ready; inspect move-group.log')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=30.)
        if not future.done() or future.exception() is not None:
            raise RuntimeError('Driver-free MoveIt query failed')
        return future.result()

    validity = node.create_client(GetStateValidity, '/check_state_validity')
    fk = node.create_client(GetPositionFK, '/compute_fk')
    scene = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
    plan = node.create_client(GetMotionPlan, '/plan_kinematic_path')

    def robot_state(q):
        expanded = cad.expanded(q)
        state = RobotState()
        state.joint_state.name = list(expanded)
        state.joint_state.position = list(expanded.values())
        state.is_diff = False
        return state

    def valid(q):
        response = call(validity, GetStateValidity.Request(robot_state=robot_state(q)))
        return response.valid, [[p.contact_body_1, p.contact_body_2] for p in response.contacts[:8]]

    try:
        rng = np.random.default_rng(20261118)
        samples = [{name: (lo+hi)/2 for name, (lo,hi) in limits.items()}]
        samples.extend({name: float(rng.uniform(lo+.01, hi-.01)) for name,(lo,hi) in limits.items()}
                       for _ in range(96))
        for q in samples:
            for side in ('left','right'):
                q[side+'_gripper_joint'] = limits[side+'_gripper_joint'][1]
            if args.diagnostic_cam_opening_deg is not None:
                for group in cad.config['grippers']:
                    q[group['source_joint']] = (np.deg2rad(-args.diagnostic_cam_opening_deg)-group['offset_rad'])/group['scale']
        result['diagnostic_cam_opening_deg'] = args.diagnostic_cam_opening_deg
        sweep = []
        physical_degrees = sorted(set([float(v) for v in range(161)]+
                                     [v/10 for v in range(1,11)]))
        for degree in physical_degrees:
            q = dict(samples[0])
            for group in cad.config['grippers']:
                lower, upper = limits[group['source_joint']]
                q[group['source_joint']] = upper+(lower-upper)*degree/160.
            response = call(validity, GetStateValidity.Request(robot_state=robot_state(q)))
            sweep.append({'closing_from_open_deg':degree,'valid':bool(response.valid),
                          'contacts':[{'links':[c.contact_body_1,c.contact_body_2],
                                       'depth_m':float(c.depth),
                                       'position_m':[getattr(c.position,k) for k in 'xyz']}
                                      for c in response.contacts]})
        result['physical_closing_sweep'] = sweep
        if not all(row['valid'] for row in sweep):
            raise RuntimeError('CAD collision rejected part of the unchanged 0-to-160-degree motor stroke')
        checked = []
        accepted = None
        for q in samples:
            ok, contacts = valid(q)
            checked.append({'valid': bool(ok), 'contacts': contacts})
            if ok:
                accepted = q
                break
        result['sampled_states'] = checked
        largest = 0.
        for q in samples[:5]:
            request = GetPositionFK.Request(robot_state=robot_state(q))
            request.header.frame_id = 'base_link'
            request.fk_link_names = ['left_grasp_tcp','right_grasp_tcp',
                                     'cad_left_gripper_internal_L','cad_right_gripper_distal_R']
            # Some releases retain different passive-body names; use actual child links.
            bodies = {j.get('name'):j.find('child').get('link') for j in ET.fromstring(xml).findall('joint')}
            request.fk_link_names[2:] = [bodies[name] for name in cad.dependent_names]
            response = call(fk, request)
            if response.error_code.val != 1:
                raise RuntimeError('Native FK rejected the calibrated CAD model')
            for name, stamped in zip(response.fk_link_names, response.pose_stamped):
                expected, rotation = UrdfChain(xml,'base_link',name).pose(cad.expanded(q))
                actual = np.array([getattr(stamped.pose.position,k) for k in 'xyz'])
                largest = max(largest,float(np.linalg.norm(expected-actual)))
        if largest > 1e-8:
            raise RuntimeError('Native FK differs from the calibrated CAD coordinate chain')
        result['maximum_native_fk_disagreement_m'] = largest
        if accepted is None:
            raise RuntimeError('No collision-free CAD state found; refusing runtime activation')
        result['collision_free_state'] = accepted
        # Occupy the entire workspace, then remove it: the same state must flip both ways.
        obj = CollisionObject(id='cad_no_motion_collision_probe', operation=CollisionObject.ADD)
        obj.header.frame_id = 'base_link'
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[4.,4.,4.])]
        obj.primitive_poses = [Pose()]
        obj.primitive_poses[0].orientation.w = 1.
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.world.collision_objects = [obj]
        if not call(scene, request).success:
            raise RuntimeError('CAD collision probe could not be inserted')
        collided, contacts = valid(accepted)
        if collided:
            raise RuntimeError('CAD collision detector accepted a fully obstructed workspace')
        result['obstacle_rejected'] = True
        result['obstacle_contacts'] = contacts
        obj.operation = CollisionObject.REMOVE
        request.scene.world.collision_objects = [obj]
        if not call(scene, request).success or not valid(accepted)[0]:
            raise RuntimeError('CAD collision state did not recover after probe removal')
        goal = dict(accepted)
        goal['left_shoulder_yaw_joint'] += .002
        if not valid(goal)[0]:
            goal = dict(accepted)
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = 'left_grasp_arm'
        motion.pipeline_id = 'ompl'
        motion.allowed_planning_time = 5.
        motion.num_planning_attempts = 1
        motion.start_state = robot_state(accepted)
        motion.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=name,position=goal[name],tolerance_above=1e-4,
                            tolerance_below=1e-4,weight=1.)
            for name in limits if name.startswith('left_') and name != 'left_gripper_joint'])]
        response = call(plan, request)
        result['plan_only_error_code'] = response.motion_plan_response.error_code.val
        result['planned_points'] = len(response.motion_plan_response.trajectory.joint_trajectory.points)
        if result['plan_only_error_code'] != 1 or not result['planned_points']:
            raise RuntimeError('Native CAD plan-only request failed')
        log.flush()
        if 'Verified collision detector: cleany/CalibratedCADFCL' not in (args.output/'move-group.log').read_text():
            raise RuntimeError('The requested CAD collision detector was not verified')
        result['collision_detector'] = 'cleany/CalibratedCADFCL'
        result['success'] = True
    finally:
        (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        node.destroy_node()
        rclpy.try_shutdown()
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:
                process.wait(timeout=15.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM)
                process.wait(timeout=5.)
        log.close()
    print(json.dumps(result),flush=True)


if __name__ == '__main__':
    main()
