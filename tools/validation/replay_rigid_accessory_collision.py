#!/usr/bin/env python3
"""Replay a saved collision with synthetic joint/cloud input and no hardware driver."""
import argparse
from copy import deepcopy
import importlib.util
import inspect
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity
from sensor_msgs.msg import JointState, PointCloud2, PointField
from rclpy.qos import qos_profile_sensor_data
from shape_msgs.msg import SolidPrimitive

ROOT = Path(__file__).resolve().parents[2]
runtime_path = ROOT/'ros2_ws/src/cleany_moveit_config/runtime_test/runtime_support.py'
if not runtime_path.exists():
    runtime_path = ROOT/'ros2_ws/src/cleany_moveit_config/test/runtime_support.py'
spec = importlib.util.spec_from_file_location('isolated_moveit_runtime', runtime_path)
runtime = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runtime)


def run(model, output, recorded, joint_limits):
    output.mkdir(parents=True, exist_ok=True)
    sample = recorded['recorded_collision']
    current = dict(zip(sample['full_joint_names'], sample['full_joint_rad']))
    arrival = recorded['measured_joint_rad']
    current.update(zip(sample['sample_joint_names'], arrival))
    model_root = ET.parse(model).getroot()
    defaults = {joint.get('name'): (float(joint.find('limit').get('lower')) +
                float(joint.find('limit').get('upper')))/2
                for joint in model_root.findall('joint')
                if joint.get('type') in ('revolute', 'prismatic')}
    current = {**defaults, **current}
    contact = np.asarray(sample['contacts'][0]['position_m'])
    self_point = (np.floor(contact/.01)+.5)*.01
    foreign_point = self_point + np.array([0., -.075, 0.])
    points = np.asarray([self_point, foreign_point], dtype='<f4')
    options = dict(head=True, octomap=True, robot_model_file=model)
    if 'joint_limits_file' in inspect.signature(runtime.moveit_runtime).parameters:
        options['joint_limits_file'] = joint_limits
    with runtime.moveit_runtime(output, **options) as (node, process):
        joints = node.create_publisher(JointState, '/joint_states', 10)
        clouds = node.create_publisher(PointCloud2, '/perception/scene_cloud', 10)
        snapshot = []
        def observed(message):
            values = np.ndarray((message.width, 3), dtype='<f4', buffer=bytes(message.data),
                                strides=(message.point_step, 4)).copy()
            snapshot.append(values)
        node.create_subscription(PointCloud2, '/perception/scene_cloud_filtered', observed, qos_profile_sensor_data)
        state = JointState(name=list(current), position=list(current.values()))
        cloud = PointCloud2(height=1, width=2, is_dense=True, point_step=12, row_step=24,
            fields=[PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                    for i,n in enumerate(('x','y','z'))], data=points.tobytes())
        cloud.header.frame_id = 'base_link'
        def publish():
            state.header.stamp = node.get_clock().now().to_msg()
            cloud.header.stamp = state.header.stamp
            joints.publish(state)
            clouds.publish(cloud)
        node.create_timer(.05, publish)
        validity = node.create_client(GetStateValidity, '/check_state_validity')
        until = time.monotonic()+120.
        while not validity.service_is_ready() or len(snapshot) < 4:
            if process.poll() is not None or time.monotonic()>until:
                raise RuntimeError('Isolated mapping did not become ready')
            node.executor.spin_once(timeout_sec=.05)
        trial = RobotState()
        commanded = {**defaults, **dict(zip(sample['full_joint_names'], sample['full_joint_rad']))}
        trial.joint_state.name = list(commanded)
        trial.joint_state.position = list(commanded.values())
        response = runtime.call(node, process, validity, GetStateValidity.Request(
            robot_state=trial, group_name='left_grasp_arm'))
        frames=snapshot[-3:]
        has=lambda point: all(any(np.linalg.norm(row-point)<1e-5 for row in frame) for frame in frames)
        result={'model':str(model), 'self_cell_center_m':self_point.tolist(),
                'filtered_self_present':has(self_point), 'foreign_point_preserved':has(foreign_point),
                'sample_valid':bool(response.valid),
                'contacts':[{'body_1':c.contact_body_1,'body_2':c.contact_body_2,'depth':c.depth}
                            for c in response.contacts]}
        result['recorded_state_outside_current_bounds'] = [
            joint.get('name') for joint in model_root.findall('joint')
            if joint.get('name') in commanded and joint.find('limit') is not None
            and not float(joint.find('limit').get('lower')) <= commanded[joint.get('name')]
            <= float(joint.find('limit').get('upper'))]
        query=node.create_client(GetPlanningScene,'/get_planning_scene')
        scene=runtime.call(node,process,query,GetPlanningScene.Request(components=PlanningSceneComponents(
            components=PlanningSceneComponents.OCTOMAP)))
        from rclpy.serialization import serialize_message
        (output/'scene.cdr').write_bytes(serialize_message(scene.scene))
        # An explicit external object still blocks exactly the same commanded pose.
        obj=CollisionObject(id='external_probe',operation=CollisionObject.ADD)
        obj.header.frame_id='base_link'
        obj.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.02,.02,.02])]
        pose=Pose();pose.orientation.w=1.
        pose.position.x,pose.position.y,pose.position.z=map(float,contact)
        obj.primitive_poses=[pose]
        change=PlanningScene(is_diff=True);change.world.collision_objects=[obj]
        apply=node.create_client(ApplyPlanningScene,'/apply_planning_scene')
        assert runtime.call(node,process,apply,ApplyPlanningScene.Request(scene=change)).success
        external=runtime.call(node,process,validity,GetStateValidity.Request(
            robot_state=trial,group_name='left_grasp_arm'))
        result['external_obstacle_blocked']=not external.valid and any(
            'external_probe' in (c.contact_body_1,c.contact_body_2) for c in external.contacts)
        (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result),flush=True)
        return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--before',type=Path,required=True)
    parser.add_argument('--after',type=Path,required=True)
    parser.add_argument('--recorded',type=Path,required=True)
    parser.add_argument('--joint-limits',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    recorded=json.loads(args.recorded.read_text())
    before=run(args.before,args.output/'before',recorded,args.joint_limits)
    after=run(args.after,args.output/'after',recorded,args.joint_limits)
    success=(before['filtered_self_present'] and not after['filtered_self_present']
             and before['foreign_point_preserved'] and after['foreign_point_preserved']
             and after['external_obstacle_blocked'])
    # Later joint calibration can make the historical pose inadmissible. This
    # replay verifies point filtering and external collision detection only.
    if not after['recorded_state_outside_current_bounds']:
        success = success and not before['sample_valid'] and after['sample_valid']
    (args.output/'summary.json').write_text(json.dumps({'success':success,'before':before,'after':after},indent=2)+'\n')
    return int(not success)


if __name__=='__main__':
    raise SystemExit(main())
