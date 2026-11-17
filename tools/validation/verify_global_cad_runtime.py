"""Check default launch models on an isolated ROS graph without motor drivers."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET


def model_hash(description: str) -> str:
    return hashlib.sha256(ET.canonicalize(description, strip_text=True).encode()).hexdigest()


def main() -> None:
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from rcl_interfaces.srv import GetParameters
    from sensor_msgs.msg import JointState

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '213' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise ValueError('Use isolated localhost domain 213')
    args.output.mkdir(parents=True, exist_ok=False)
    model = Path(get_package_share_directory('cleany_description'))/'urdf/cleany.urdf'
    expected = model_hash(model.read_text())
    results = []
    for package, launch, node_name, extra in [
        ('cleany_moveit_config', 'physical_move_group.launch.py', '/move_group', []),
        ('cleany_moveit_config', 'move_group.launch.py', '/move_group', []),
        ('cleany_description', 'description.launch.py', '/global_cad_validation/runtime_robot_state_publisher',
         ['namespace:=global_cad_validation', 'use_rviz:=false'])]:
        log = (args.output/(launch+'.log')).open('w')
        process = subprocess.Popen(['ros2','launch',package,launch,*extra], stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = rclpy.create_node('verify_global_cad_runtime')
        samples = []
        subscription = node.create_subscription(JointState, '/global_cad_validation/joint_states', samples.append, 10)
        client = node.create_client(GetParameters, node_name+'/get_parameters')
        try:
            deadline = time.monotonic()+90
            while not client.wait_for_service(timeout_sec=.2):
                if process.poll() is not None or time.monotonic()>deadline:
                    raise RuntimeError('Default launch failed: '+launch)
            names = ['robot_description'] if package == 'cleany_description' else [
                'robot_description','allow_trajectory_execution','collision_detector']
            future = client.call_async(GetParameters.Request(names=names))
            rclpy.spin_until_future_complete(node, future, timeout_sec=10)
            values = future.result().values
            actual = model_hash(values[0].string_value)
            assert actual == expected, launch
            result = {'launch':launch,'canonical_model_hash':actual,'matches':True}
            if package != 'cleany_description':
                assert not values[1].bool_value
                assert values[2].string_value == 'cleany/CalibratedCADFCL'
                result.update(execution_enabled=False, collision_detector=values[2].string_value)
            else:
                deadline = time.monotonic()+5
                while not samples and time.monotonic()<deadline:
                    rclpy.spin_once(node, timeout_sec=.1)
                assert samples and len(samples[-1].name)==30
                result['display_joints'] = len(samples[-1].name)
            forbidden = [n for n,_ in node.get_service_names_and_types()
                         if n.startswith(('/controller_manager/', '/real_control/'))]
            assert not forbidden, forbidden
            results.append(result)
        finally:
            node.destroy_node()
            rclpy.try_shutdown()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=10)
            log.close()
    report = {'success':True,'launches':results,'hardware_drivers_started':0,'motor_commands_sent':0}
    (args.output/'result.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report))


if __name__ == '__main__':
    main()
