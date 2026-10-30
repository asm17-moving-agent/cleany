"""Signal the real server lifecycle during a synthetic gripper opening."""
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid
import pytest
import rclpy
from cleany_interfaces.action import ReturnArm
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from test_arm_return import HOME_FILE
from test_jetson_lifecycle import free_port


@pytest.mark.parametrize('stop_signal,confirmed', [
    (signal.SIGINT, True), (signal.SIGTERM, True), (signal.SIGTERM, False),
])
def test_server_signal_drains_manual_return_before_ros_shutdown(tmp_path, stop_signal, confirmed):
    namespace = '/return_shutdown_' + uuid.uuid4().hex
    script = f'''
import sys
sys.path.insert(0, {str(Path(__file__).parent)!r})
from test_arm_return import FakeOperations
from cleany_manipulation_bt import jetson_node as node
from cleany_manipulation_bt.backend import Observation, WorkerBackend
from rclpy.node import Node
from rclpy.parameter import Parameter

class Operations(FakeOperations, Node):
    def __init__(self):
        Node.__init__(self, 'synthetic_return_operations', namespace={namespace!r})
        FakeOperations.__init__(self)
        self.block = True
        self.stop_confirmed = {confirmed!r}
    def destroy_node(self):
        return Node.destroy_node(self)

server = node.JetsonManipulationNode
node.JetsonManipulationNode = lambda: server(operations=Operations(),
    backend=WorkerBackend(lambda *_: Observation()), namespace={namespace!r},
    parameter_overrides=[Parameter('database_path', value={str(tmp_path / 'journal.sqlite3')!r}),
                        Parameter('monitor_port', value={free_port()}),
                        Parameter('arm_return_pose_file', value={str(HOME_FILE)!r})])
node.main()
'''
    process = subprocess.Popen([sys.executable, '-u', '-c', script],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    context = Context()
    rclpy.init(context=context)
    observer = Node('return_shutdown_observer', context=context, namespace=namespace)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(observer)
    client = ActionClient(observer, ReturnArm, 'manipulation/return_arm')
    feedback = []
    try:
        assert client.wait_for_server(timeout_sec=10.)
        sent = client.send_goal_async(ReturnArm.Goal(arm='right'),
            feedback_callback=lambda msg: feedback.append(msg.feedback.stage))
        executor.spin_until_future_complete(sent, timeout_sec=5.)
        assert sent.done() and sent.result().accepted
        result = sent.result().get_result_async()
        deadline = time.monotonic() + 5.
        while 'opening_right' not in feedback and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        assert 'opening_right' in feedback
        process.send_signal(stop_signal)
        executor.spin_until_future_complete(result, timeout_sec=10.)
        assert result.done()
        assert not result.result().result.success
        assert result.result().result.stop_confirmed == confirmed
        output, _ = process.communicate(timeout=10.)
        assert process.returncode == 0, output
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate(timeout=5.)
        client.destroy()
        executor.shutdown()
        observer.destroy_node()
        context.shutdown()
