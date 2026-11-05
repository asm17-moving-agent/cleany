"""Real DDS recovery traffic over fake stop evidence, with no hardware access."""
import socket
import threading
import time
from types import SimpleNamespace
import uuid
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from cleany_interfaces.srv import ConfirmManipulationRecovery, GetManipulationExecution
from cleany_manipulation_bt.backend import Observation, WorkerBackend
from cleany_manipulation_bt.jetson_node import JetsonManipulationNode
from cleany_manipulation_bt.store import BTExecutionStore
from cleany_skill_executor.manipulation.models import (
    Error, Goal, ObjectState, Placement, Record, RecordState, Result, Status,
)


@pytest.mark.parametrize('stop_verified', [True, False])
def test_recovery_service_preserves_failure_and_requires_stop_evidence(tmp_path, stop_verified):
    database = str(tmp_path / 'runtime.sqlite3')
    store = BTExecutionStore(database)
    goal = Goal('mission', 'task', 'previous', 'collect_trash', 'old', 1, 'return_to_pick')
    result = Result('previous', 'jetson', Status.FATAL, Error.STOP_UNCONFIRMED,
        'STOPPING', 'VALIDATING', ObjectState.NOT_TOUCHED, Placement.NOT_CHECKED,
        '', False, False, False, 'Physical stop unconfirmed')
    record = Record(goal, execution_profile='jetson', record_state=RecordState.FINISHED,
                    human_confirmation_required=True, result=result, revision=17)
    store.save(record, create=True)
    store.close()
    context = Context()
    rclpy.init(context=context)
    namespace = '/recovery_test_' + uuid.uuid4().hex
    observed = []
    backend = WorkerBackend(lambda *args: observed.append(args) or Observation())
    operations = SimpleNamespace(config={}, destroy_node=lambda: None,
                                  confirm_recovery_stationary=lambda: stop_verified)
    with socket.socket() as available:
        available.bind(('127.0.0.1', 0))
        port = available.getsockname()[1]
    server = JetsonManipulationNode(context=context, namespace=namespace,
        backend=backend, operations=operations,
        parameter_overrides=[Parameter('database_path', value=database), Parameter('monitor_port', value=port)])
    client = Node('recovery_client', context=context, namespace=namespace)
    service = client.create_client(ConfirmManipulationRecovery, 'manipulation/confirm_recovery')
    get_record = client.create_client(GetManipulationExecution, 'manipulation/get_execution')
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(server)
    executor.add_node(client)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def call(service, request):
        future = service.call_async(request)
        deadline = time.monotonic() + 5.
        while not future.done() and time.monotonic() < deadline:
            time.sleep(.01)
        assert future.done()
        return future.result()

    try:
        assert service.wait_for_service(timeout_sec=5.)
        status = call(service, ConfirmManipulationRecovery.Request())
        assert status.success and status.inhibited
        assert status.blocked_execution_ids == ['previous']
        assert list(status.blocked_revisions) == [17]
        missing_confirmation = call(service, ConfirmManipulationRecovery.Request(
            execution_id='previous', expected_revision=17, robot_stopped_confirmed=True))
        assert not missing_confirmation.success and missing_confirmation.inhibited
        acknowledged = call(service, ConfirmManipulationRecovery.Request(
            execution_id='previous', expected_revision=17,
            robot_stopped_confirmed=True, gripper_empty_confirmed=True, operator_note='Checked by operator'))
        assert acknowledged.success == stop_verified
        assert acknowledged.inhibited != stop_verified
        original = call(get_record, GetManipulationExecution.Request(execution_id='previous'))
        assert original.found and original.record.status == 'FATAL'
        assert original.record.error_code == 'STOP_UNCONFIRMED'
        assert original.record.human_confirmation_required and not original.record.stop_confirmed
        assert server.core.record is None and observed == []
    finally:
        executor.shutdown(timeout_sec=5.)
        thread.join(timeout=5.)
        server.destroy_node()
        client.destroy_node()
        context.shutdown()
