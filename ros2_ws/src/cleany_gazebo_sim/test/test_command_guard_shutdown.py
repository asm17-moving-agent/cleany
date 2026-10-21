"""Exercise command delivery and SIGINT cleanup in a real ROS child process."""
from __future__ import annotations

import signal
import subprocess
import sys
import time
import uuid

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


def test_sigint_exits_cleanly_while_receiving_commands(tmp_path):
    namespace = '/guard_shutdown_' + uuid.uuid4().hex
    context = Context()
    rclpy.init(context=context)
    node = Node('guard_shutdown_observer', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    output = []
    publisher = node.create_publisher(Twist, namespace + '/cmd_vel', 10)
    node.create_subscription(Twist, namespace + '/gazebo_cmd_vel', output.append, 10)
    command = Twist()
    command.linear.x = .5
    log_path = tmp_path/'command_guard.log'
    with log_path.open('w') as log:
        child = subprocess.Popen([
            sys.executable, '-c',
            'from cleany_gazebo_sim.command_guard import main; main()',
            '--ros-args', '-r', '__ns:=' + namespace,
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 10
            while not output and child.poll() is None and time.monotonic() < deadline:
                publisher.publish(command)
                executor.spin_once(timeout_sec=.005)
            assert output, log_path.read_text()
            assert output[-1].linear.x == pytest.approx(.3)
            # Keep incoming callbacks ready across the shutdown request.
            for _ in range(50):
                publisher.publish(command)
            child.send_signal(signal.SIGINT)
            deadline = time.monotonic() + 5
            while child.poll() is None and time.monotonic() < deadline:
                publisher.publish(command)
                executor.spin_once(timeout_sec=.005)
            assert child.poll() == 0, log_path.read_text()
            assert 'Traceback' not in log_path.read_text()
        finally:
            if child.poll() is None:
                child.kill()
            child.wait(timeout=5)
            executor.shutdown()
            node.destroy_node()
            context.shutdown()
