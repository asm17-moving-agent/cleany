"""Exercise the installed MoveIt process, including controller plugin teardown."""
from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import time

import pytest

if not os.environ.get('ROS_DISTRO'):
    pytest.skip('ROS 2 environment is not active', allow_module_level=True)

import rclpy
from moveit_msgs.srv import GetStateValidity
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from cleany_moveit_config.controller_plugin import controller_plugin_environment


def test_controller_preload_preserves_existing_environment(monkeypatch):
    monkeypatch.setenv('LD_PRELOAD', '/existing/instrumentation.so')
    environment = controller_plugin_environment(
        'moveit_simple_controller_manager/MoveItSimpleControllerManager', keep_loaded=True)
    assert environment['LD_PRELOAD'].startswith('/existing/instrumentation.so:')
    assert Path(environment['LD_PRELOAD'].split(':')[-1]).is_file()
    assert os.environ['LD_PRELOAD'] == '/existing/instrumentation.so'
    assert controller_plugin_environment('other/Controller', keep_loaded=True) == {}
    assert controller_plugin_environment(
        'moveit_simple_controller_manager/MoveItSimpleControllerManager', keep_loaded=False) == {}


def test_move_group_exits_cleanly_after_sigint(tmp_path):
    domain = 30 + os.getpid() % 180
    context = Context()
    rclpy.init(args=[], context=context, domain_id=domain)
    executor = SingleThreadedExecutor(context=context)
    node = Node('move_group_shutdown_test', context=context)
    client = node.create_client(GetStateValidity, '/check_state_validity')
    log = tmp_path / 'move_group.log'
    process = None
    try:
        with log.open('w') as stream:
            process = subprocess.Popen(
                ['ros2', 'launch', 'cleany_moveit_config', 'move_group.launch.py',
                 'use_rviz:=false', 'backend_log_level:=warn'],
                env={**os.environ, 'ROS_DOMAIN_ID': str(domain),
                     'RCUTILS_LOGGING_USE_STDOUT': '1'},
                stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 20
            while not client.service_is_ready() and time.monotonic() < deadline:
                assert process.poll() is None, log.read_text()
                rclpy.spin_once(node, executor=executor, timeout_sec=.1)
            assert client.service_is_ready(), log.read_text()
            # Exercise the capability before tearing down its callback groups.
            request = GetStateValidity.Request()
            request.robot_state.is_diff = True
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, executor=executor, timeout_sec=5)
            assert future.done() and future.result() is not None, log.read_text()
            process.send_signal(signal.SIGINT)
            process.wait(timeout=12)
        output = log.read_text()
        shutdown_output = output.split('user interrupted with ctrl-c (SIGINT)', 1)[-1]
        assert process.returncode == 0, output
        assert '[WARN]' in output, output
        assert '[ERROR]' not in shutdown_output, output
        assert 'process has finished cleanly' in output, output
        assert "sending signal 'SIGTERM'" not in output, output
    finally:
        if process is not None:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
