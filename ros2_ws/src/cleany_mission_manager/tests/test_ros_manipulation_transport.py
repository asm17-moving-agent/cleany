"""ROS diagnostics must not alter the existing outcome or accept stale feedback."""

from dataclasses import asdict
from types import SimpleNamespace
from uuid import uuid4

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("cleany_interfaces.action")

from cleany_interfaces.action import ExecuteManipulationSkill
from cleany_mission_manager.adapters.goals import goal_uuid
from cleany_mission_manager.adapters.manipulation import ROSManipulationTransport, result_response
from cleany_mission_manager.core.runtime_models import ManipulationOutcome
from test_manipulation import outcome, resolved


def feedback(execution_id, substage="GraspObject"):
    return SimpleNamespace(feedback=ExecuteManipulationSkill.Feedback(
        execution_id=execution_id, stage="GRASPING", substage=substage, selected_arm="left"))


def test_latest_feedback_ignores_old_wrong_and_finished_executions():
    transport = object.__new__(ROSManipulationTransport)
    transport.active_execution_id, transport.feedback = "first", {}
    transport._feedback("first", feedback("first"))
    transport._feedback("first", feedback("wrong"))
    assert transport.feedback["execution_id"] == "first"
    transport._feedback("first", feedback("first", "ConfirmGrasp"))
    assert transport.feedback["substage"] == "ConfirmGrasp"
    transport._finished("first")
    transport._feedback("first", feedback("first", "late"))
    assert transport.feedback["substage"] == "ConfirmGrasp"
    callbacks, uuids = [], []
    def send_goal(request, **kwargs):
        callbacks.append(kwargs["feedback_callback"])
        uuids.append(kwargs["goal_uuid"])
        return resolved(SimpleNamespace(accepted=True))
    transport.client = SimpleNamespace(send_goal_async=send_goal)
    execution_id = str(uuid4())
    transport.send(dict(execution_id=execution_id))
    assert transport.feedback == {}
    assert uuids[0] == goal_uuid(execution_id)
    transport._feedback("first", feedback("first"))
    transport._finished("first")
    assert transport.active_execution_id == execution_id and transport.feedback == {}
    callbacks[0](feedback(execution_id))
    assert transport.feedback["execution_id"] == execution_id
    callbacks[0](SimpleNamespace())
    assert transport.feedback["execution_id"] == execution_id


def test_extended_action_result_keeps_legacy_dto_fields_only():
    payload = ExecuteManipulationSkill.Result(**asdict(outcome("id")),
        failed_substage="", cancel_mode="CHECKPOINT")
    translated = result_response(SimpleNamespace(status=4, result=payload))
    assert set(translated["result"]) == set(ManipulationOutcome.__dataclass_fields__)
    assert ManipulationOutcome(**translated["result"]) == outcome("id")
    assert translated["details"] == {"failed_substage": "", "cancel_mode": "CHECKPOINT"}
