"""Signal a real server process with synthetic operations; never command hardware."""
import os
import signal
import socket
import subprocess
import sys
import time
import uuid
from pathlib import Path
import pytest
import rclpy
from action_msgs.msg import GoalStatus
from cleany_interfaces.action import ExecuteManipulationSkill
from cleany_manipulation_bt.store import BTExecutionStore
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from cleany_manipulation_bt import jetson_node as server_module
from cleany_skill_executor.core.execution_ownership import ExecutionOwnership
from test_jetson_lifecycle import free_port, overrides


@pytest.mark.parametrize('stop_signal,confirmed,launch_process,stop_delay', [
    (signal.SIGINT, True, False, 0.), (signal.SIGTERM, True, False, 0.),
    (signal.SIGTERM, False, False, 0.), (signal.SIGINT, True, True, 11.),
])
def test_signal_shutdown_delivers_cancel_and_honest_stop_result(
        tmp_path, stop_signal, confirmed, launch_process, stop_delay):
    namespace = '/shutdown_' + uuid.uuid4().hex
    database = str(tmp_path / 'executions.sqlite3')
    with socket.socket() as available:
        available.bind(('127.0.0.1', 0))
        port = available.getsockname()[1]
    script = f'''
import time
from cleany_manipulation_bt import jetson_node as node
from cleany_manipulation_bt.backend import Observation, WorkerBackend
from cleany_skill_executor.manipulation.models import Error
from rclpy.parameter import Parameter

def operation(stage, context):
    if stage == 'ValidateGoal':
        if not context.abort.wait(20.):
            raise RuntimeError('Shutdown did not interrupt the running operation')
        return Observation(False, Error.CANCELED, 'Synthetic operation interrupted')
    if stage in ('StopAndAssess', 'StopAfterRecovery'):
        time.sleep({stop_delay!r})
    return Observation(stop_confirmed={confirmed!r})

server = node.JetsonManipulationNode
node.JetsonManipulationNode = lambda: server(
    backend=WorkerBackend(operation), namespace={namespace!r},
    parameter_overrides=[Parameter('database_path', value={database!r}),
                        Parameter('monitor_port', value={port}),
                        Parameter('bt_stop_timeout_sec', value=15.)])
node.main()
'''
    if launch_process:
        launch_path = Path(__file__).parents[1] / 'launch/manipulation_jetson.launch.py'
        root = Path(__file__).resolve().parents[4]
        # Use production launch timing, replacing only the motor-facing process
        # with this synthetic server. Real launch escalation remains enabled.
        script = f'''
import importlib.util
import sys
from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, SetLaunchConfiguration

spec = importlib.util.spec_from_file_location('jetson_launch', {str(launch_path)!r})
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
def synthetic_process(**kwargs):
    return ExecuteProcess(cmd=[sys.executable, '-u', '-c', {script!r}], output='screen',
        sigterm_timeout=kwargs.get('sigterm_timeout', '5'),
        sigkill_timeout=kwargs.get('sigkill_timeout', '5'))
module.Node = synthetic_process
service = LaunchService()
service.include_launch_description(LaunchDescription([
    SetLaunchConfiguration('repository_root', {str(root)!r}),
    SetLaunchConfiguration('collection_config', {str(root / 'ros2_ws/src/cleany_skill_executor/test/data/supervised_collection_fixture.json')!r}),
    SetLaunchConfiguration('hardware_monitor_config', {str(root / 'ros2_ws/src/cleany_skill_executor/test/data/hardware_motion_monitor_fixture.yaml')!r}),
    *module.generate_launch_description().entities]))
raise SystemExit(service.run())
'''
    context = Context()
    rclpy.init(context=context)
    client_node = Node('shutdown_client', context=context, namespace=namespace)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(client_node)
    client = ActionClient(client_node, ExecuteManipulationSkill, 'manipulation/execute_skill')
    log = tmp_path / 'server.log'
    with log.open('w') as output:
        process = subprocess.Popen([sys.executable, '-u', '-c', script],
                                   stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            assert client.wait_for_server(timeout_sec=5.), log.read_text()
            goal = ExecuteManipulationSkill.Goal(mission_id='m', task_id='t', execution_id='signal',
                skill_name='collect_trash', snapshot_id='s', object_id=1, destination_id='d')
            stages = []
            accepted = client.send_goal_async(goal,
                feedback_callback=lambda message: stages.append(message.feedback.substage))
            executor.spin_until_future_complete(accepted, timeout_sec=5.)
            assert accepted.done() and accepted.result().accepted, log.read_text()
            deadline = time.monotonic() + 5.
            while 'ValidateGoal' not in stages and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.05)
            assert 'ValidateGoal' in stages, log.read_text()
            result = accepted.result().get_result_async()
            # Repeated launch/terminal signals must not interrupt the drain.
            if launch_process:
                # collection-stop and terminal Ctrl+C signal the whole group.
                os.killpg(process.pid, stop_signal)
            else:
                process.send_signal(stop_signal)
                process.send_signal(stop_signal)
            executor.spin_until_future_complete(result, timeout_sec=20.)
            assert result.done(), log.read_text()
            wrapped = result.result()
            assert wrapped.result.stop_confirmed == confirmed, log.read_text()
            assert wrapped.result.cancel_mode == 'IMMEDIATE'
            assert wrapped.status == (GoalStatus.STATUS_CANCELED if confirmed else GoalStatus.STATUS_ABORTED)
            assert wrapped.result.status == ('CANCELED' if confirmed else 'FATAL')
            assert process.wait(timeout=8.) == 0, log.read_text()
            store = BTExecutionStore(database)
            try:
                assert store.get('signal').result.stop_confirmed == confirmed
            finally:
                store.close()
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5.)
            executor.shutdown()
            client_node.destroy_node()
            context.shutdown()


def test_shutdown_failure_still_releases_ros_context_and_physical_ownership(tmp_path, monkeypatch):
    server_type = server_module.JetsonManipulationNode
    executor_type = server_module.MultiThreadedExecutor

    def interrupted_executor(**kwargs):
        executor = executor_type(**kwargs)
        def interrupt(**kwargs):
            raise KeyboardInterrupt
        executor.spin_once = interrupt
        return executor

    def failed_drain(*args):
        raise RuntimeError('Injected shutdown transport failure')

    monkeypatch.setattr(server_module, 'JetsonManipulationNode',
        lambda: server_type(parameter_overrides=overrides(tmp_path, free_port())))
    monkeypatch.setattr(server_module, 'MultiThreadedExecutor', interrupted_executor)
    monkeypatch.setattr(server_module, 'drain_shutdown', failed_drain)
    with pytest.raises(RuntimeError, match='Injected shutdown transport failure'):
        server_module.main(args=[])
    assert not rclpy.ok()
    ExecutionOwnership().close()
