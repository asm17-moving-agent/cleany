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
    assert "'grasp_demo_ros2_controllers.yaml'" in source
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
    assert "self.declare_parameter('planning_attempts', 3)" in source
    assert "self.declare_parameter('replan_attempts', 2)" in source
    assert "self.declare_parameter('replan_delay_sec', 0.25)" in source
    assert "code == MoveItErrorCodes.CONTROL_FAILED" in source
    assert 'replanning once from current state' in source
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
    assert "'allowed_execution_duration_scaling': '2.0'" in source
    assert "'allowed_goal_duration_margin': '1.0'" in source
    assert "'enable_gripper_controllers': 'true'" in source
    assert "'grasp_demo_ros2_controllers.yaml'" in source
    assert "'planning_attempts': 3" in source
    assert "'pregrasp_closing_tolerance_deg': 15.0" in source
    assert "'grasp_closing_tolerance_deg': ParameterValue(" in source
    assert "'grasp_closing_sign_invariant': False" in source
    assert "'grasp_approach_offset_m': ParameterValue(" in source
    assert "'grasp_lateral_offset_m': ParameterValue(" in source
    assert "'grasp_lateral_execution_offset_m': ParameterValue(" in source
    assert "'grasp_approach_execution_offset_m': ParameterValue(" in source
    assert "'gripper_close_position_rad': ParameterValue(" in source
    assert "'gripper_close_position_rad', default_value='0.30'" in source
    assert "'gripper_force_full_close': ParameterValue(" in source
    assert "'gripper_force_full_close', default_value='false'" in source
    assert "') / 2.0 - 0.008'" in source
    assert "target_depth_m," in source
    assert "'target_label': target_label" in source
    assert "'grasp_closing_tolerance_deg', default_value='5.0'" in source
    assert "'left_shoulder_yaw_initial': '-1.53'" in source
    assert "'right_shoulder_yaw_initial': '1.58'" in source
    assert source.count("_pitch_initial': '3.35'") == 2
    assert source.count("_elbow_pitch_initial': '3.12'") == 2
    assert source.count("_wrist_pitch_initial': '-1.63'") == 2
    assert source.count("_wrist_roll_initial': '1.58'") == 2
    assert source.count("_gripper_initial': '-0.35'") == 2
    assert "'can_grasp_execution_demo = '" in setup
    assert "'cleany_skill_executor.can_grasp_execution_demo:main'" in setup


def test_can_demo_executes_selected_grasp_closes_and_lifts() -> None:
    source = (
        PACKAGE_ROOT
        / 'cleany_skill_executor'
        / 'can_grasp_execution_demo.py'
    ).read_text(encoding='utf-8')

    registration = (
        'self._register_execution_collision(target_object, result.selected_arm)'
    )
    opening = (
        "self._command_gripper(\n                result.selected_arm,\n"
        "                float(self.get_parameter('gripper_open_position_rad').value)"
    )
    grasp_execution = "result.grasp_joint_state,\n                'contact-enabled grasp'"
    closing = (
        "self._command_gripper(\n                result.selected_arm,\n"
        '                self._candidate_close_position(result.selected_candidate)'
    )
    attachment = (
        'self._attach_execution_collision(target_object, result.selected_arm)'
    )
    lift = "result.pregrasp_joint_state,\n                'attached-can lift retreat'"
    verification = 'self._verify_can_lifted('

    assert registration in source
    assert 'aimed_pregrasp = self._solve_aimed_pregrasp(' in source
    assert 'joint_state=seed' in source
    assert "GetPositionIK" not in source
    assert "f'{arm}_pregrasp_aim_tip'" in source
    assert "'collision-checked aimed pre-grasp'" in source
    assert 'self._verify_pregrasp_facing(' in source
    assert 'selected_approach=aimed_pregrasp.approach_direction' in source
    assert 'selected_pregrasp=aimed_pregrasp.tcp_position' in source
    move = "self._move_to(\n                result.selected_arm,"
    feedback = 'self._verify_feedback(aimed_pregrasp.joint_state)'
    assert source.index(move) < source.index(feedback) < source.index(opening)
    assert (
        source.index(opening)
        < source.index(grasp_execution)
        < source.index(closing)
        < source.index(attachment)
        < source.index(lift)
        < source.index(verification)
    )
    assert 'pre-grasp reached; opening gripper' in source
    assert 'accept_control_failure=lambda:' in source
    assert 'self._guarded_contact_stop_is_valid(' in source
    assert 'allow_contact_stall=True' in source
    assert 'goal.path_tolerance = [tolerance]' in source
    assert 'goal.goal_tolerance = [tolerance]' in source
    assert "JointTolerance(name=joint, position=-1.0)" in source
    assert "self._hold('lift_hold_sec')" in source
    assert source.count('self._verify_can_lifted(') == 2
    assert "self.declare_parameter('gripper_close_position_rad', 0.30)" in source
    assert 'MoveIt attached collision remains active' in source
    assert 'self._restore_execution_collision()' in source
