from moveit_msgs.msg import AllowedCollisionEntry, AllowedCollisionMatrix

from cleany_skill_executor.planning_scene import TargetSceneTransaction


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
