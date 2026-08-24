from types import SimpleNamespace

import pytest
from moveit_msgs.msg import AllowedCollisionEntry, AllowedCollisionMatrix

from cleany_skill_executor.core.grasp_selection import InfrastructureError
from cleany_skill_executor.planning_scene import TargetSceneTransaction


class _Future:
    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class _ApplyClient:
    def __init__(self, responses):
        self._responses = iter(responses)

    def wait_for_service(self, timeout_sec):
        return True

    def call_async(self, request):
        return _Future(next(self._responses))


def test_only_evaluated_gripper_links_are_allowed_to_touch_target():
    original = AllowedCollisionMatrix()
    original.entry_names = ['base_link', 'right_gripper_frame']
    for _ in original.entry_names:
        entry = AllowedCollisionEntry()
        entry.enabled = [False, False]
        original.entry_values.append(entry)

    updated = TargetSceneTransaction._with_target_permissions(
        original, 'target', 'left'
    )
    matrix = {
        row_name: dict(zip(updated.entry_names, row.enabled))
        for row_name, row in zip(updated.entry_names, updated.entry_values)
    }
    assert matrix['target']['left_gripper_frame'] is True
    assert matrix['target']['left_moving_jaw_link'] is True
    assert matrix['target']['base_link'] is False
    assert matrix['target']['right_gripper_frame'] is False
    assert original.entry_names == ['base_link', 'right_gripper_frame']


def test_failed_restore_keeps_transaction_state_for_retry():
    client = _ApplyClient(
        [SimpleNamespace(success=False), SimpleNamespace(success=True)]
    )
    transaction = TargetSceneTransaction(
        object(),
        apply_client=client,
        get_client=object(),
    )
    transaction._object_id = 'target'
    transaction._saved_acm = AllowedCollisionMatrix()

    with pytest.raises(InfrastructureError, match='restore planning scene'):
        transaction.restore()

    assert transaction._object_id == 'target'
    assert transaction._saved_acm is not None
    transaction.restore()
    assert transaction._object_id == ''
    assert transaction._saved_acm is None
