#!/usr/bin/env python3
"""Verify the production selector container against isolated, plan-only MoveIt."""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from uuid import uuid4
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[2]


def verify(model: Path, parameters: dict, output: Path, timeout: float,
           collection_config: Path | None = None, state_profile: Path | None = None) -> dict:
    """Publish synthetic feedback; never launch a controller or hardware driver."""
    import rclpy
    from rcl_interfaces.srv import GetParameters
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    support = ROOT/'ros2_ws/src/cleany_moveit_config/runtime_test/runtime_support.py'
    if not support.exists():
        support = ROOT/'ros2_ws/src/cleany_moveit_config/test/runtime_support.py'
    spec = importlib.util.spec_from_file_location('isolated_container_moveit', support)
    runtime = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runtime)
    if parameters['motion_backend'] != 'curobo_ik_moveit' or parameters['curobo_ik_cpu_fallback']:
        raise ValueError('Production GPU IK with CPU fallback disabled is required')
    config = output/'selector.yaml'
    config.write_text(yaml.safe_dump({'grasp_selection_server': {'ros__parameters': parameters}}))
    q = {}
    for joint in ET.parse(model).getroot().findall('joint'):
        if joint.get('type') == 'revolute' and joint.find('mimic') is None:
            limit = joint.find('limit')
            q[joint.get('name')] = (float(limit.get('lower'))+float(limit.get('upper')))/2
    for side in ('left', 'right'):
        q[side+'_gripper_joint'] = parameters['gripper_open_joint_rad']
    from cleany_description.cad_coordinates import CadCoordinates
    cad = CadCoordinates.from_urdf(model.read_text())
    if cad is not None:
        q = cad.expanded(q)
    process = None
    owner = 'no-motion-validation-'+uuid4().hex[:12]
    began = time.monotonic()
    with runtime.moveit_runtime(output, head=True, robot_model_file=model,
                                localhost_only=False) as (node, moveit):
        states = node.create_publisher(JointState, '/joint_states', 10)
        descriptions = node.create_publisher(String, '/robot_description',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        descriptions.publish(String(data=model.read_text()))

        def publish():
            state = JointState(name=list(q), position=list(q.values()))
            state.header.stamp = node.get_clock().now().to_msg()
            states.publish(state)

        node.create_timer(.05, publish)
        environment = dict(os.environ, ROS_DOMAIN_ID=str(node.context.get_domain_id()),
            ROS_LOCALHOST_ONLY='0', CLEANY_MOTION_OWNER=owner)
        ready = node.create_client(Trigger, '/grasp/warmup_status')
        settings = node.create_client(GetParameters, '/grasp_selection_server/get_parameters')
        message = 'selector startup'
        try:
            with (output/'selector.log').open('w') as log:
                command = [str(ROOT/'tools/launch-motion-container'),
                    'physical_grasp_selection_server', '--ros-args', '--params-file', str(config)]
                if collection_config is not None:
                    command = ['ros2', 'launch', 'cleany_skill_executor',
                        'pregrasp_hardware_support.launch.py', f'repository_root:={ROOT}',
                        f'profile_file:={state_profile or ROOT/"configs/robot/camera_preview.json"}',
                        f'collection_config_file:={collection_config}',
                        f'output_root:={output}/launch-profile',
                        'motion_backend:=curobo_ik_moveit', 'motion_runtime:=container',
                        'grasp_predictor:=geometric', 'start_head_camera:=false']
                process = subprocess.Popen(command,
                    env=environment, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                deadline = time.monotonic()+timeout
                while True:
                    if process.poll() is not None:
                        raise RuntimeError('Selector exited: '+(output/'selector.log').read_text()[-6000:])
                    if time.monotonic() > deadline:
                        raise TimeoutError('Container warmup deadline: '+message)
                    node.executor.spin_once(timeout_sec=.02)
                    if ready.wait_for_service(timeout_sec=.05):
                        response = runtime.call(node, moveit, ready, Trigger.Request(), timeout=10.)
                        message = response.message
                        if response.success:
                            break
                names = ['motion_backend', 'curobo_ik_cpu_fallback', 'pregrasp_z_offset_m',
                         'gripper_open_joint_rad', 'asymmetric_gripper_enabled',
                         'curobo_ik_endpoint_margin_rad', 'curobo_sphere_model']
                values = runtime.call(node, moveit, settings, GetParameters.Request(names=names)).values
                observed = {names[0]: values[0].string_value, names[1]: values[1].bool_value,
                    names[2]: values[2].double_value, names[3]: values[3].double_value,
                    names[4]: values[4].bool_value, names[5]: values[5].double_value,
                    names[6]: values[6].string_value}
                assert all(observed[name] == parameters[name] for name in names), observed
                forbidden = [name for name, _ in node.get_service_names_and_types()
                    if name.startswith('/controller_manager/') or name.startswith('/real_control/')]
                assert not forbidden, forbidden
                sensor_nodes = sorted(set(node.get_node_names()) &
                    {'camera', 'calibrated_rgbd', 'rgbd_frame_bridge'})
                if collection_config is not None:
                    assert not sensor_nodes, sensor_nodes
                result = dict(success=True, scope='container_selector_warmup_with_host_moveit',
                    parameters=observed, warmup_message=message,
                    wall_sec=time.monotonic()-began, synthetic_joint_count=len(q),
                    ros_domain_id=node.context.get_domain_id(), hardware_drivers_started=0,
                    rmw_implementation=os.environ['RMW_IMPLEMENTATION'],
                    dds_profile=os.environ.get('CYCLONEDDS_URI', ''),
                    trajectory_goals_sent=0, torque_commands_sent=0, owner=owner)
                result.update(shared_support_launch=collection_config is not None,
                              duplicate_head_camera_nodes=sensor_nodes)
                (output/'result.json').write_text(json.dumps(result, indent=2)+'\n')
                print(json.dumps(result), flush=True)
                return result
        finally:
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=10)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--parameters', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=300.)
    parser.add_argument('--collection-config', type=Path,
                        help='Verify the full shared-camera support launch with this collection profile')
    parser.add_argument('--state-profile', type=Path,
                        help='Materialize this model/state profile in the shared support launch')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    parameters = json.loads(args.parameters.read_text())
    os.environ.update(ROS_LOCALHOST_ONLY='0', OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1')
    os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    if os.environ['RMW_IMPLEMENTATION'] == 'rmw_cyclonedds_cpp':
        os.environ.setdefault('CYCLONEDDS_URI',
            'file://'+str(ROOT/'containers/vision/cyclonedds/host.xml'))
    verify(args.model, parameters.get('effective', parameters), args.output, args.timeout,
           args.collection_config, args.state_profile)


if __name__ == '__main__':
    main()
