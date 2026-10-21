"""Isolated MoveIt service tests: synthetic states/clouds, no device drivers."""
import contextlib
import itertools
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor


_domains = itertools.count(os.getpid())


@contextlib.contextmanager
def moveit_runtime(tmp_path, *, head=False, octomap=False, robot_model_file=None):
    domain = str(120 + next(_domains) % 60)
    previous_localhost = os.environ.get('ROS_LOCALHOST_ONLY')
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    env = {**os.environ, 'ROS_DOMAIN_ID': domain}
    context = Context()
    rclpy.init(args=[], context=context, domain_id=int(domain))
    node = rclpy.create_node('isolated_moveit_probe', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    log_path = Path(tmp_path) / 'moveit.log'
    with log_path.open('w') as log:
        rsp = None
        if octomap:
            # Match move_group's fixed-wheel model so every self-filter shape
            # has a TF even though the synthetic state only publishes arm/head.
            model = (Path(robot_model_file).read_text() if robot_model_file else
                subprocess.check_output(['xacro', str(Path(get_package_share_directory('cleany_description'))/'urdf/cleany.urdf.xacro'), f'include_head_camera:={str(head).lower()}', 'include_wheel_joints:=false'], text=True))
            params = Path(tmp_path)/'robot-state.yaml'
            params.write_text(yaml.safe_dump({'/**': {'ros__parameters': {'robot_description': model}}}))
            rsp = subprocess.Popen(['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher', '--ros-args', '--params-file', str(params)], env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        process = subprocess.Popen([
            'ros2', 'launch', 'cleany_moveit_config', 'move_group.launch.py',
            'use_rviz:=false', 'allow_trajectory_execution:=false',
            f'include_head_camera:={str(head).lower()}',
            f'enable_depth_octomap:={str(octomap).lower()}',
            *([f'robot_description_file:={robot_model_file}'] if robot_model_file else []),
        ], env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            yield node, process
        finally:
            if process.poll() is None:
                os.kill(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=10)
            if rsp is not None and rsp.poll() is None:
                os.killpg(rsp.pid, signal.SIGINT)
                try:
                    rsp.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(rsp.pid, signal.SIGTERM)
                    rsp.wait(timeout=10)
            executor.shutdown()
            node.destroy_node()
            context.shutdown()
            if previous_localhost is None:
                os.environ.pop('ROS_LOCALHOST_ONLY', None)
            else:
                os.environ['ROS_LOCALHOST_ONLY'] = previous_localhost
            log.flush()
            output = log_path.read_text()
            assert 'Segmentation fault' not in output, 'MoveIt crashed during shutdown; see moveit.log'
            assert 'process has died' not in output, 'MoveIt child exited abnormally; see moveit.log'


def call(node, process, client, request, timeout=60):
    deadline = time.monotonic() + timeout
    while not client.wait_for_service(timeout_sec=.2):
        assert process.poll() is None, 'MoveIt launch exited'
        assert time.monotonic() < deadline, f'Service unavailable: {client.srv_name}'
    future = client.call_async(request)
    while not future.done():
        assert process.poll() is None, 'MoveIt launch exited'
        assert time.monotonic() < deadline, f'Service timed out: {client.srv_name}'
        node.executor.spin_once(timeout_sec=.02)
    result = future.result()
    assert result is not None
    return result
