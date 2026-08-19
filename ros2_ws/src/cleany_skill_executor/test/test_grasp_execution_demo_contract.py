from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]


def test_demo_launch_composes_backend_moveit_and_executor() -> None:
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


def test_can_demo_composes_rgbd_grasping_selection_and_gui() -> None:
    source = (
        PACKAGE_ROOT / 'launch' / 'can_grasp_execution_demo.launch.py'
    ).read_text(encoding='utf-8')
    setup = (PACKAGE_ROOT / 'setup.py').read_text(encoding='utf-8')

    assert "'can_grasp_execution_demo.xml.in'" in source
    assert "'camera_name': 'pick_demo_rgbd'" in source
    assert "package='cleany_grasping'" in source
    assert "executable='grasp_selection_server'" in source
    assert "executable='can_grasp_execution_demo'" in source
    assert "package='rqt_image_view'" in source
    assert "arguments=['/grasp/can_grasp_image']" in source
    assert "'allow_trajectory_execution': 'true'" in source
    assert "'enable_gripper_controllers': 'true'" in source
    assert "'can_grasp_execution_demo = '" in setup
    assert "'cleany_skill_executor.can_grasp_execution_demo:main'" in setup


def test_can_demo_opens_and_stops_at_collision_checked_pregrasp() -> None:
    source = (
        PACKAGE_ROOT
        / 'cleany_skill_executor'
        / 'can_grasp_execution_demo.py'
    ).read_text(encoding='utf-8')

    assert 'self._register_execution_collision(target_object)' in source
    assert 'self._open_gripper(result.selected_arm)' in source
    assert 'aimed_pregrasp = self._solve_aimed_pregrasp(' in source
    assert "ik.group_name = f'{arm}_pregrasp_aim_arm'" in source
    assert "ik.ik_link_name = f'{arm}_pregrasp_aim_tip'" in source
    assert 'ik.avoid_collisions = True' in source
    assert "'collision-checked aimed pre-grasp'" in source
    assert 'self._verify_pregrasp_facing(' in source
    assert 'selected_approach=aimed_pregrasp.approach_direction' in source
    assert 'selected_pregrasp=aimed_pregrasp.tcp_position' in source
    grasp_execution = (
        "self._move_to(result.selected_arm, result.grasp_joint_state"
    )
    assert grasp_execution not in source
