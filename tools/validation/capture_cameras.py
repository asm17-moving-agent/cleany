#!/usr/bin/env python3
"""Capture real simulator camera topics and fail on missing or malformed data."""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import os
from pathlib import Path
import platform
import signal
import subprocess
import sys
import time
from uuid import uuid4

import numpy as np
from PIL import Image


@dataclass(frozen=True)
class Camera:
    topic: str
    frame: str
    width: int
    height: int
    depth: bool = False


def camera_contracts(backend: str) -> dict[str, Camera]:
    if backend == 'mujoco':
        return {
            'head': Camera('/camera/color/image_raw', 'head_camera_rgb_optical_frame', 640, 480),
            **{side: Camera(f'/{side}_wrist_camera/image_raw',
                            f'{side}_wrist_rgb_optical_frame', 640, 480)
               for side in ('left', 'right')},
        }
    return {
        'head': Camera('/camera/head/color/image_raw', 'head_camera_rgb_optical_frame', 640, 480),
        'depth': Camera('/camera/head/depth/image_raw', 'head_camera_depth_optical_frame',
                        640, 480, depth=True),
        **{side: Camera(f'/camera/{side}_wrist/color/image_raw',
                        f'{side}_wrist_rgb_optical_frame', 640, 360)
           for side in ('left', 'right')},
    }


def image_pixels(message, expected: Camera) -> np.ndarray:
    """Validate the wire contract and remove row padding before decoding."""
    if (message.width, message.height, message.header.frame_id) != (
            expected.width, expected.height, expected.frame):
        raise ValueError(f'Unexpected dimensions/frame on {expected.topic}')
    encodings = ('32FC1',) if expected.depth else ('rgb8', 'bgr8')
    if message.encoding not in encodings:
        raise ValueError(f'Unexpected encoding on {expected.topic}: {message.encoding}')
    pixel_bytes = 4 if expected.depth else 3
    row_bytes = message.width * pixel_bytes
    if message.step < row_bytes or len(message.data) != message.height * message.step:
        raise ValueError(f'Malformed image buffer on {expected.topic}')
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    if expected.depth:
        dtype = '>f4' if message.is_bigendian else '<f4'
        return np.frombuffer(rows[:, :row_bytes].tobytes(), dtype=dtype).reshape(
            message.height, message.width)
    pixels = rows[:, :row_bytes].reshape(message.height, message.width, 3)
    return pixels[:, :, ::-1] if message.encoding == 'bgr8' else pixels


def gazebo_world(output: Path, model_name: str) -> Path:
    from xml.etree import ElementTree as ET
    from ament_index_python.packages import get_package_share_directory
    import yaml
    from cleany_gazebo_sim.world.cad_frame import adapt_cad_frame
    from cleany_gazebo_sim.world.generator import materialize_mecanum_wheel_world

    package = Path(get_package_share_directory('cleany_gazebo_sim'))
    world = materialize_mecanum_wheel_world(
        package / 'worlds/cleany_mecanum_fortress.sdf', target_path=output / 'world.sdf')
    if model_name == 'cad-with-arms':
        profile = yaml.safe_load((package / 'config/cad_frame.yaml').read_text())
        profile['simulation_excluded_link_roots'] = []
        config = output / 'cad-with-arms.yaml'
        config.write_text(yaml.safe_dump(profile))
        tree = ET.parse(world)
        parent = tree.find('world')
        old = parent.find("model[@name='cleany_mecanum']")
        model, metadata = adapt_cad_frame(
            old, Path(get_package_share_directory('cleany_description')), config)
        parent.remove(old)
        parent.append(model)
        tree.write(world, encoding='unicode')
        (output / 'model.json').write_text(json.dumps(metadata, indent=2))
    return world


def stop_process(process: subprocess.Popen) -> None:
    """Stop only the process group created for this capture, including children."""
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=5)


def capture(args: argparse.Namespace, report: dict) -> None:
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rcl_interfaces.srv import SetParameters
    from sensor_msgs.msg import Image as RosImage, JointState
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock

    os.environ['ROS_DOMAIN_ID'] = str(args.ros_domain_id)
    os.environ['IGN_PARTITION'] = f'cleany_camera_capture_{uuid4().hex}'
    if args.backend == 'mujoco':
        command = ['ros2', 'launch', 'cleany_mujoco_sim', 'study_cafe_backend.launch.py',
                   'headless:=true', 'scheduled_cameras:=true', 'enable_gripper_controllers:=true']
    else:
        world = gazebo_world(args.output, args.gazebo_model)
        command = ['ros2', 'launch', 'cleany_gazebo_sim', 'gazebo_fortress.launch.py',
                   'headless:=true', 'sensor_profile:=all_cameras', f'world:={world}']
    report.update(command=command, images={}, clock_messages=0, state_messages=0)
    contracts = camera_contracts(args.backend)

    def on_image(message, name):
        if name in report['images']:
            return
        pixels = image_pixels(message, contracts[name])
        info = dict(width=message.width, height=message.height, encoding=message.encoding,
                    frame_id=message.header.frame_id)
        if contracts[name].depth:
            valid = np.isfinite(pixels) & (pixels > 0)
            if not np.any(valid):
                raise ValueError('Head depth image has no positive finite samples')
            info['finite_fraction'] = float(np.mean(valid))
        else:
            Image.fromarray(pixels).save(args.output / f'{name}.png')
        report['images'][name] = info

    def on_clock(message):
        seconds = message.clock.sec + message.clock.nanosec * 1e-9
        report.setdefault('first_sim_time', seconds)
        report['last_sim_time'] = seconds
        report['clock_messages'] += 1

    def on_state(message):
        if args.backend == 'mujoco':
            expected = {f'{side}_{joint}_joint' for side in ('left', 'right')
                        for joint in ('shoulder_yaw', 'shoulder_pitch', 'elbow_pitch',
                                      'wrist_pitch', 'wrist_roll', 'gripper')}
            if not expected <= set(message.name):
                return
            values = message.position
            if len(values) != len(message.name):
                raise ValueError('Joint feedback names/positions have different lengths')
        else:
            pose = message.pose.pose
            values = [pose.position.x, pose.position.y, pose.position.z,
                      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        if not np.isfinite(values).all():
            raise ValueError('Non-finite simulator state')
        report['state_messages'] += 1

    process = None
    node = None
    with (args.output / 'launch.log').open('w') as log:
        try:
            rclpy.init()
            node = rclpy.create_node('camera_capture')
            for name, camera in contracts.items():
                node.create_subscription(RosImage, camera.topic,
                    lambda msg, key=name: on_image(msg, key), qos_profile_sensor_data)
            node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)
            node.create_subscription(JointState if args.backend == 'mujoco' else Odometry,
                '/joint_states' if args.backend == 'mujoco' else '/odom',
                on_state, qos_profile_sensor_data)
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            deadline = time.monotonic() + args.timeout

            def wait_for(predicate):
                while not predicate():
                    if process.poll() is not None:
                        raise RuntimeError(f'Simulator exited with {process.returncode}; see launch.log')
                    if time.monotonic() >= deadline:
                        raise TimeoutError('Timed out waiting for camera/clock/state; see report.json')
                    rclpy.spin_once(node, timeout_sec=0.1)

            if args.backend == 'mujoco':
                client = node.create_client(SetParameters, '/sorting_cameras/set_parameters')
                wait_for(client.service_is_ready)
                for name in contracts:
                    future = client.call_async(SetParameters.Request(parameters=[
                        Parameter('active_camera', value=name).to_parameter_msg()]))
                    wait_for(future.done)
                    result = future.result()
                    if result is None or not result.results or not all(r.successful for r in result.results):
                        raise RuntimeError(f'Camera selection failed: {name}')
                    wait_for(lambda: name in report['images'])
            wait_for(lambda: len(report['images']) == len(contracts)
                     and report['state_messages'] > 0
                     and report.get('last_sim_time', 0) > report.get('first_sim_time', 0))
            if process.poll() is not None:
                raise RuntimeError('Simulator exited before capture completed')
        finally:
            if process is not None:
                stop_process(process)
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=('mujoco', 'gazebo'), required=True)
    parser.add_argument('--gazebo-model', choices=('legacy', 'cad-with-arms'), default='cad-with-arms')
    parser.add_argument('--output', type=Path, required=True, help='New directory for PNG, JSON and launch log')
    parser.add_argument('--timeout', type=float, default=60.0)
    parser.add_argument('--ros-domain-id', type=int, default=170 + os.getpid() % 30)
    args = parser.parse_args()
    if not np.isfinite(args.timeout) or args.timeout <= 0 or not 0 <= args.ros_domain_id <= 232:
        parser.error('timeout must be positive and ROS domain must be between 0 and 232')
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    repo = Path(__file__).resolve().parents[2]
    report = dict(backend=args.backend, gazebo_model=args.gazebo_model,
                  ros_domain_id=args.ros_domain_id, python=platform.python_version(),
                  architecture=platform.machine(), status='failed')
    try:
        report['commit'] = subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip()
        report['dirty'] = bool(subprocess.check_output(
            ['git', 'status', '--porcelain'], cwd=repo, text=True).strip())
        capture(args, report)
        report['status'] = 'passed'
    except (Exception, KeyboardInterrupt) as error:
        report['error'] = f'{type(error).__name__}: {error}'
        print(report['error'], file=sys.stderr)
    finally:
        (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(f"{args.backend}: {report['status']} ({args.output})")
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
