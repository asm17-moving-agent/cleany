from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]


def test_demo_launch_composes_real_backend_moveit_selector_and_executor() -> None:
    source = (
        PACKAGE_ROOT / 'launch' / 'grasp_execution_demo.launch.py'
    ).read_text(encoding='utf-8')

    assert "'cleany_mujoco_sim', 'handeye_backend.launch.py'" in source
    assert "'cleany_moveit_config', 'move_group.launch.py'" in source
    assert "executable='grasp_selection_server'" in source
    assert "executable='grasp_execution_demo'" in source
    assert "'allow_trajectory_execution': 'true'" in source
    assert source.count("'use_sim_time': True") == 2


def test_demo_executes_both_selected_endpoints_and_verifies_feedback() -> None:
    source = (
        PACKAGE_ROOT
        / 'cleany_skill_executor'
        / 'grasp_execution_demo.py'
    ).read_text(encoding='utf-8')

    pregrasp = "self._move_to(result.selected_arm, result.pregrasp_joint_state"
    grasp = "self._move_to(result.selected_arm, result.grasp_joint_state"
    assert source.index(pregrasp) < source.index(grasp)
    assert "goal.planning_options.plan_only = False" in source
    assert "self._verify_feedback(result.grasp_joint_state)" in source
    assert "'DEMO COMPLETE:" in source


def test_demo_entry_point_and_launch_are_installed() -> None:
    setup = (PACKAGE_ROOT / 'setup.py').read_text(encoding='utf-8')

    assert "glob('launch/*.launch.py')" in setup
    assert (
        'grasp_execution_demo = '
        'cleany_skill_executor.grasp_execution_demo:main'
    ) in setup

