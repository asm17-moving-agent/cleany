"""Actual DDS ActionClient traffic, retained events and process crash recovery."""

from pathlib import Path
import subprocess
import sys
import threading
import time
import uuid

from action_msgs.msg import GoalStatus
from cleany_interfaces.action import ExecuteManipulationSkill
from cleany_interfaces.msg import ManipulationExecutionRecord
from cleany_interfaces.srv import CancelManipulation, GetManipulationExecution
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import yaml

from cleany_skill_executor.manipulation.core import NORMAL_STAGES
from cleany_skill_executor.manipulation.models import Stage
from cleany_skill_executor.manipulation.store import StoreError
from cleany_skill_executor.manipulation_node import ManipulationNode


CONFIG_PATH = Path(__file__).parents[1] / 'config/manipulation_mock.yaml'


def eventually(predicate, timeout=10.0, *, description='ROS condition'):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError(f'Timed out waiting for {description}')


def response(future, timeout=10.0):
    eventually(future.done, timeout)
    return future.result()


def goal(execution_id=None, **changes):
    values = dict(mission_id='mission', task_id='task',
                  execution_id=execution_id or str(uuid.uuid4()), skill_name='collect_trash',
                  snapshot_id='mock-snapshot-001', object_id=1, destination_id='mock_trash_bin')
    values.update(changes)
    return ExecuteManipulationSkill.Goal(**values)


class RosHarness:
    def __init__(self, tmp_path, *, scenario='success', start=True, slow=False):
        self.namespace = '/manipulation_test_' + uuid.uuid4().hex
        self.context = Context()
        rclpy.init(context=self.context)
        self.executor = MultiThreadedExecutor(num_threads=4, context=self.context)
        self.node = Node('client', namespace=self.namespace, context=self.context)
        self.executor.add_node(self.node)
        self.config = tmp_path / 'mock.yaml'
        self.database = tmp_path / 'executions.sqlite3'
        config = yaml.safe_load(CONFIG_PATH.read_text())
        config.update(stage_duration_sec=0.25 if slow else 0.08, stop_duration_sec=0.03,
                      preparation_timeout_sec=5.0, verification_timeout_sec=5.0,
                      motion_timeout_sec=10.0, stop_timeout_sec=1.0)
        self.config.write_text(yaml.safe_dump(config))
        self.feedback = []
        self.events = []
        self.qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscription = self.node.create_subscription(
            ManipulationExecutionRecord, 'manipulation/execution_events', self.events.append, self.qos)
        self.client = ActionClient(self.node, ExecuteManipulationSkill, 'manipulation/execute_skill')
        self.service = self.node.create_client(GetManipulationExecution, 'manipulation/get_execution')
        self.server = None
        if start:
            self.server = ManipulationNode(context=self.context, namespace=self.namespace,
                parameter_overrides=[Parameter('database_path', value=str(self.database)),
                                     Parameter('mock_config', value=str(self.config)),
                                     Parameter('scenario', value=scenario)])
            self.executor.add_node(self.server)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        if start:
            assert self.client.wait_for_server(timeout_sec=10.0)

    def send(self, message):
        return response(self.client.send_goal_async(
            message, feedback_callback=lambda item: self.feedback.append(item.feedback)))

    def query(self, execution_id):
        assert self.service.wait_for_service(timeout_sec=10.0)
        return response(self.service.call_async(GetManipulationExecution.Request(execution_id=execution_id)))

    def close(self):
        self.executor.shutdown(timeout_sec=5.0)
        self.thread.join(timeout=5.0)
        self.client.destroy()
        if self.server is not None:
            self.server.destroy_node()
        self.node.destroy_node()
        self.context.shutdown()


@pytest.fixture
def ros(tmp_path):
    harnesses = []

    def create(**kwargs):
        directory = tmp_path / str(len(harnesses))
        directory.mkdir()
        harness = RosHarness(directory, **kwargs)
        harnesses.append(harness)
        return harness

    yield create
    for harness in harnesses:
        harness.close()


def test_real_ros_success_feedback_lookup_and_retained_event(ros):
    harness = ros()
    message = goal()
    handle = harness.send(message)
    assert handle.accepted
    result = response(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_SUCCEEDED
    assert result.result.status == 'SUCCESS' and result.result.execution_profile == 'mock'
    assert result.result.stop_confirmed and result.result.arm_recovered
    eventually(lambda: any(item.stage == 'FINALIZING' for item in harness.feedback),
               description='FINALIZING Action feedback')
    stages = list(dict.fromkeys(item.stage for item in harness.feedback))
    assert stages == [stage.value for stage in NORMAL_STAGES] + ['FINALIZING']
    assert {'GraspObject', 'ConfirmGrasp'} <= {item.substage for item in harness.feedback}
    record = harness.query(message.execution_id)
    assert record.found and record.record.has_result and record.record.status == 'SUCCESS'
    assert record.record.last_completed_stage == 'VERIFYING_PLACEMENT'
    assert record.record.object_state == 'LEFT_GRIPPER' and record.record.placement_state == 'CONFIRMED'
    assert {'GraspObject', 'ConfirmGrasp', 'ConfirmRelease'} <= set(record.record.completed_substages)
    assert not harness.query('missing').found
    # A fresh ROS context represents a monitor joining after the Action ended.
    observer = ros(start=False)
    late_events = []
    subscriber = observer.node.create_subscription(ManipulationExecutionRecord,
        harness.namespace + '/manipulation/execution_events', late_events.append, observer.qos)
    eventually(lambda: any(item.has_result for item in late_events),
               description='retained result in the late monitor')
    assert any(item.execution_id == message.execution_id and item.status == 'SUCCESS' for item in late_events)
    observer.node.destroy_subscription(subscriber)
    assert not response(handle.cancel_goal_async()).goals_canceling


@pytest.mark.parametrize('changes,error', [
    ({'snapshot_id': 'absent'}, 'TARGET_UNAVAILABLE'),
    ({'object_id': 99}, 'TARGET_UNAVAILABLE'),
    ({'snapshot_id': 'mock-stale-snapshot'}, 'STALE_TARGET'),
    ({'destination_id': 'absent'}, 'DESTINATION_UNAVAILABLE'),
])
def test_real_ros_preparation_returns_blocked_without_motion(ros, changes, error):
    harness = ros()
    message = goal(**changes)
    handle = harness.send(message)
    assert handle.accepted
    result = response(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_SUCCEEDED
    assert result.result.status == 'BLOCKED' and result.result.error_code == error
    record = harness.query(message.execution_id).record
    assert record.object_state == 'NOT_TOUCHED' and record.placement_state == 'NOT_CHECKED'
    assert not any(item.stage == 'APPROACHING' for item in harness.events)


@pytest.mark.parametrize('scenario,error,object_state,placement_state', [
    ('grasp_failure', 'GRASP_FAILED', 'UNKNOWN', 'NOT_CHECKED'),
    ('grasp_lost', 'GRASP_LOST', 'UNKNOWN', 'NOT_CHECKED'),
    ('placement_failure', 'PLACEMENT_NOT_CONFIRMED', 'LEFT_GRIPPER', 'NOT_CONFIRMED'),
    ('verification_timeout', 'VERIFICATION_TIMEOUT', 'LEFT_GRIPPER', 'UNKNOWN'),
    ('return_failure', 'MOTION_FAILED', 'LEFT_GRIPPER', 'CONFIRMED'),
])
def test_real_ros_failure_keeps_evidence_in_result_and_query(ros, scenario, error, object_state, placement_state):
    harness = ros(scenario=scenario)
    message = goal()
    handle = harness.send(message)
    result = response(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_ABORTED
    assert result.result.status == 'FAILED' and result.result.error_code == error
    assert result.result.object_state == object_state and result.result.placement_state == placement_state
    assert result.result.stop_confirmed
    record = harness.query(message.execution_id).record
    assert record.has_result and record.error_code == error
    assert record.object_state == object_state and record.placement_state == placement_state
    if object_state == 'UNKNOWN':
        assert not harness.send(goal()).accepted


@pytest.mark.parametrize('stage', NORMAL_STAGES)
def test_real_ros_stage_cancellation_stops_before_next_stage(ros, stage):
    harness = ros(slow=True)
    message = goal()
    handle = harness.send(message)
    eventually(lambda: any(item.stage == stage.value and item.message.startswith('Starting')
                           for item in harness.feedback))
    canceled = response(handle.cancel_goal_async())
    assert canceled.goals_canceling
    result = response(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_CANCELED
    assert result.result.status == 'CANCELED' and result.result.stop_confirmed
    assert result.result.failed_stage == stage.value
    assert not result.result.retryable
    record = harness.query(message.execution_id).record
    assert record.has_result and record.status == 'CANCELED'
    forbidden = [item.value for item in NORMAL_STAGES[NORMAL_STAGES.index(stage) + 1:]]
    assert not any(item.stage in forbidden for item in harness.feedback)


@pytest.mark.parametrize('mode', ['IMMEDIATE', 'CHECKPOINT', 'RETURN_ARM'])
def test_real_ros_cancel_mode_service_keeps_action_state_and_recovery_evidence(ros, mode):
    harness = ros(slow=True)
    message = goal()
    handle = harness.send(message)
    eventually(lambda: any(item.stage == 'TRANSPORTING' for item in harness.feedback))
    service = harness.node.create_client(CancelManipulation, 'manipulation/cancel')
    assert service.wait_for_service(timeout_sec=5.)
    canceled = response(service.call_async(CancelManipulation.Request(execution_id=message.execution_id, mode=mode)))
    assert canceled.accepted, canceled.message
    result = response(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_CANCELED
    assert result.result.status == 'CANCELED' and result.result.cancel_mode == mode
    assert result.result.stop_confirmed
    assert result.result.arm_recovered == (mode == 'RETURN_ARM')
    assert result.result.object_state == ('LEFT_GRIPPER' if mode == 'RETURN_ARM' else 'HELD')
    assert result.result.placement_state == 'NOT_CHECKED'
    assert harness.query(message.execution_id).record.cancel_mode == mode


def test_real_ros_immediate_escalation_interrupts_requested_arm_return(ros):
    harness = ros(slow=True)
    message = goal()
    handle = harness.send(message)
    eventually(lambda: any(item.stage == 'TRANSPORTING' for item in harness.feedback))
    service = harness.node.create_client(CancelManipulation, 'manipulation/cancel')
    assert service.wait_for_service(timeout_sec=5.)
    assert response(service.call_async(CancelManipulation.Request(
        execution_id=message.execution_id, mode='RETURN_ARM'))).accepted
    eventually(lambda: any(item.stage == 'RECOVERING_ARM' for item in harness.feedback))
    assert response(service.call_async(CancelManipulation.Request(
        execution_id=message.execution_id, mode='IMMEDIATE'))).accepted
    result = response(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_CANCELED
    assert result.result.cancel_mode == 'IMMEDIATE' and not result.result.arm_recovered


@pytest.mark.parametrize('wrong_id,mode', [(False, 'invalid'), (True, 'RETURN_ARM')])
def test_real_ros_invalid_mode_request_does_not_cancel_execution(ros, wrong_id, mode):
    harness = ros(slow=True)
    message = goal()
    handle = harness.send(message)
    service = harness.node.create_client(CancelManipulation, 'manipulation/cancel')
    assert service.wait_for_service(timeout_sec=5.)
    rejected = response(service.call_async(CancelManipulation.Request(
        execution_id='wrong' if wrong_id else message.execution_id, mode=mode)))
    assert not rejected.accepted
    assert response(handle.get_result_async()).status == GoalStatus.STATUS_SUCCEEDED


def test_cli_return_cancel_uses_mode_service_and_prints_recovery_result(ros):
    harness = ros(slow=True)
    completed = subprocess.run(
        ['ros2', 'run', 'cleany_skill_executor', 'manipulation_test_client',
         '--namespace', harness.namespace, '--cancel-stage', 'TRANSPORTING',
         '--cancel-mode', 'RETURN_ARM'], capture_output=True, text=True, timeout=15.)
    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert '"cancel_mode": "RETURN_ARM"' in completed.stdout
    assert '"arm_recovered": true' in completed.stdout
    assert '"status": "CANCELED"' in completed.stdout


@pytest.mark.parametrize('scenario,error,cancel', [
    ('stop_failure', 'STOP_UNCONFIRMED', True),
    ('stop_timeout', 'STOP_UNCONFIRMED', True),
    ('hardware_fault', 'HARDWARE_ERROR', False),
    ('e_stop', 'E_STOP', False),
])
def test_real_ros_fatal_rejects_followup_goal(ros, scenario, error, cancel):
    harness = ros(scenario=scenario)
    message = goal()
    handle = harness.send(message)
    if cancel:
        eventually(lambda: any(item.stage == 'APPROACHING' for item in harness.feedback))
        assert response(handle.cancel_goal_async()).goals_canceling
    result = response(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_ABORTED
    assert result.result.status == 'FATAL' and result.result.error_code == error
    assert not harness.send(goal()).accepted
    assert harness.query(message.execution_id).record.human_confirmation_required


def test_real_ros_concurrent_goal_and_duplicate_id_run_once(ros):
    harness = ros()
    messages = [goal(), goal()]
    futures = [harness.client.send_goal_async(message) for message in messages]
    handles = [response(future) for future in futures]
    assert sum(handle.accepted for handle in handles) == 1
    index = next(index for index, handle in enumerate(handles) if handle.accepted)
    assert response(handles[index].get_result_async()).result.status == 'SUCCESS'
    assert not harness.send(messages[index]).accepted
    assert not harness.send(goal(messages[index].execution_id, object_id=2)).accepted
    assert not harness.query(messages[1 - index].execution_id).found


def test_real_ros_record_failure_preserves_result_query_and_followup(ros, monkeypatch):
    harness = ros()
    save = harness.server.store.save

    def fail_final(record, **kwargs):
        if record.result is not None:
            raise StoreError('injected final transaction failure')
        save(record, **kwargs)

    monkeypatch.setattr(harness.server.store, 'save', fail_final)
    message = goal()
    handle = harness.send(message)
    result = response(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_SUCCEEDED
    assert result.result.status == 'SUCCESS' and result.result.error_code == 'NONE'
    record = harness.query(message.execution_id).record
    assert record.record_state == 'FINISHED' and not record.human_confirmation_required
    assert record.has_result and record.status == 'SUCCESS'
    assert harness.server.store.get(message.execution_id).result is None
    followup = harness.send(goal())
    assert followup.accepted
    assert response(followup.get_result_async()).result.status == 'SUCCESS'
    assert harness.query(message.execution_id).record.status == 'SUCCESS'


def test_process_kill_restart_reports_interruption_without_action_result(ros, tmp_path):
    harness = ros(start=False, slow=True)
    command = [sys.executable, '-c',
               'from cleany_skill_executor.manipulation_node import main; main()',
               '--ros-args', '-r', '__ns:=' + harness.namespace,
               '-p', 'database_path:=' + str(harness.database),
               '-p', 'mock_config:=' + str(harness.config)]
    log_path = tmp_path / 'server.log'
    process = None
    with log_path.open('w') as log:
        try:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            assert harness.client.wait_for_server(timeout_sec=10.0)
            message = goal()
            handle = harness.send(message)
            assert handle.accepted
            eventually(lambda: any(item.stage == 'LIFTING' for item in harness.feedback))
            process.kill()
            process.wait(timeout=5.0)
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            eventually(lambda: 'inhibited=True' in log_path.read_text())
            record = harness.query(message.execution_id).record
            assert record.record_state == 'INTERRUPTED' and record.human_confirmation_required
            assert not record.has_result and record.status == ''
            assert record.object_state == 'HELD' and record.last_completed_stage == 'GRASPING'
            assert not record.stop_confirmed
            eventually(lambda: any(item.record_state == 'INTERRUPTED' for item in harness.events))
            assert not harness.send(goal()).accepted
            assert not harness.send(message).accepted
        finally:
            if process is not None and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5.0)
