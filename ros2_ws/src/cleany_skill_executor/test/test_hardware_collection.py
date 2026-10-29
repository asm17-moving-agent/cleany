"""Physical connection contracts; these tests never connect to motor ports."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as NS
import json
import math
import pytest

from cleany_skill_executor.core.collection import configuration_issues, load_arm, check_evidence
from cleany_skill_executor.core.hardware_execution import (
    ArmHold, PhysicalJointLimit, bounded_path, bridge_joint_path,
    physical_limits, remaining_joint_path,
)
from cleany_skill_executor.core.physical_grasp_selection import ARM_JOINT_NAMES
from cleany_skill_executor.hardware_execution import (
    collision_contact_details,
    transient_octomap_arm_collision,
)


def config():
    return {'frame':'base_link','commissioning_verified':True,'label_destinations':{'bottle':'recycling'},
            'lift_height_m':.05,'cartesian_step_m':.005,'cartesian_tolerance_m':.005,
            'evidence_max_age_sec':.5,'empty_confirmations':2,'maximum_cycles':3,
            'gripper_close_step_rad':.005,'pregrasp_opening_margin_m':.005,
            'arms':{'left':{'gripper':{'measured':True,
            'closed_joint_rad':-.2,'open_joint_rad':1.,'maximum_width_m':.08},
            'home_joint_rad':[0.]*5,'bins':{'recycling':[.1]*5}}}}


def test_repository_return_hold_profile_needs_no_destination_or_bin():
    root = Path(__file__).resolve().parents[4]
    issues = configuration_issues(json.loads((root/'ros2_ws/src/cleany_skill_executor/test/data/supervised_collection_fixture.json').read_text()))
    assert issues == []
    assert not configuration_issues(config())


@pytest.mark.parametrize('key,value', [
    ('offset', -.01), ('offset', float('nan')), ('offset', float('inf')),
    ('offset', True), ('offset', '0.02'), ('offset', None),
    ('tolerance', 0.), ('tolerance', -.01), ('tolerance', float('nan')),
    ('tolerance', .02), ('tolerance', .03), ('tolerance', True),
])
def test_raised_pickup_return_rejects_invalid_or_nonpositive_clearance(key, value):
    settings = {**config(), 'return_to_pick_height_offset_m': .02,
                'return_to_pick_height_tolerance_m': .01,
                f'return_to_pick_height_{key}_m': value}
    assert any('return_to_pick_height_' in issue for issue in configuration_issues(settings))


@pytest.mark.parametrize('key', ['return_to_pick_reverse_enabled', 'return_to_pick_collision_check_enabled'])
@pytest.mark.parametrize('value', ['false', 0, None])
def test_return_replay_switches_require_booleans(key, value):
    assert any(key in issue for issue in configuration_issues({**config(), key: value}))


def test_replay_collision_exception_is_scoped_to_temporary_raised_return():
    root = Path(__file__).resolve().parents[4]
    settings = json.loads((root/'ros2_ws/src/cleany_skill_executor/test/data/supervised_collection_fixture.json').read_text())
    assert settings['return_to_pick_reverse_enabled'] is True
    assert settings['return_to_pick_collision_check_enabled'] is False
    assert not configuration_issues(settings)
    for changes in ({'operation_mode': 'full_collection'}, {'return_to_pick_height_offset_m': 0.},
                    {'return_to_pick_reverse_enabled': False}):
        assert any('return_to_pick_' in issue for issue in configuration_issues({**settings, **changes}))


@pytest.mark.parametrize('key,value', [
    ('max_attempts', -1), ('max_attempts', 5), ('max_attempts', True), ('max_attempts', 1.5),
    ('max_step_m', 0.), ('max_step_m', .011), ('max_step_m', float('nan')), ('max_step_m', True),
])
def test_return_height_corrections_have_finite_bounded_settings(key, value):
    assert any('return_to_pick_height_correction_' in issue for issue in configuration_issues(
        {**config(), 'return_to_pick_height_correction_'+key: value}))


@pytest.mark.parametrize('value', ['false', 0, None])
def test_pregrasp_motion_policy_requires_a_boolean(value):
    assert 'pregrasp_motion_enabled must be boolean' in configuration_issues(
        {**config(), 'pregrasp_motion_enabled': value})


@pytest.mark.parametrize('key', ['visual_servo_enabled', 'wrist_observation_enabled'])
def test_direct_grasp_rejects_pregrasp_observation_modes(key):
    settings = {**config(), 'pregrasp_motion_enabled': False,
                'visual_servo_enabled': False, 'wrist_observation_enabled': False}
    assert not configuration_issues(settings)
    settings[key] = True
    assert 'pregrasp_motion_enabled=false requires '+key+'=false' in configuration_issues(settings)


@pytest.mark.parametrize('arm', ['left', 'right'])
def test_disabled_pregrasp_opens_jaw_without_an_arm_trajectory(arm):
    from cleany_skill_executor.collection_pipeline import CollectionPipeline

    calls = []
    raw = config()['arms']['left']
    arm_config = load_arm(raw)
    node = NS(config={'pregrasp_motion_enabled': False, 'pregrasp_opening_margin_m': .005},
        arm_config=arm_config, scene_monitor=NS(wait=lambda: None),
        jaw=lambda *args, **options: calls.append(('jaw', args)),
        motion=NS(set_gripper_position=lambda *args: calls.append(('model', args))),
        event=lambda *args, **kwargs: calls.append(('event', args)),
        real=NS(monitor=lambda: NS(header=NS(stamp=NS(sec=2, nanosec=3)))))
    selected = NS(selected_arm=arm, selected_candidate=NS(required_opening_m=.04))
    assert CollectionPipeline.execute_pregrasp(node, selected) == 2_000_000_003
    opening = arm_config.opening(.045)
    assert calls == [('jaw', (arm, opening, 'open_before_direct_grasp')),
                     ('model', (arm, opening)), ('event', ('pregrasp_motion_skipped',))]


@pytest.mark.parametrize('arm', ['left', 'right'])
@pytest.mark.parametrize('error_m,accepted', [(0., True), (.029, True), (.031, False)])
@pytest.mark.parametrize('pose_checks', [True, False])
def test_direct_grasp_uses_selected_joints_and_checks_tcp_before_closing(arm, error_m, accepted, pose_checks):
    from geometry_msgs.msg import Pose
    from cleany_skill_executor.collection_pipeline import CollectionPipeline

    positions = (.1, .2, .3, .4, .5)
    target, actual = Pose(), Pose()
    target.orientation.w = actual.orientation.w = 1.
    actual.position.x = error_m
    commands = []
    selected = NS(selected_arm=arm, grasp_joint_state=NS(name=ARM_JOINT_NAMES[arm], position=positions),
                  fk_grasp_pose=target)
    node = NS(config={'direct_grasp_shortcut_enabled': False,
                      'direct_grasp_pose_checks_enabled': pose_checks}, event=lambda *args, **kwargs: None,
              move_joint_pose=lambda *args, **kwargs: commands.append((args, kwargs)), pose=lambda _: actual,
              cartesian_pose=CollectionPipeline.cartesian_pose,
              get_logger=lambda: NS(warning=lambda _: None),
              real=NS(value=lambda _: .030, monitor=lambda: 'measured state', last_execution_metrics={}))
    report = {}
    if accepted or not pose_checks:
        assert CollectionPipeline.move_direct_to_grasp(node, selected, report) == 'measured state'
        assert report['pregrasp_executed'] is False and report['direct_grasp_executed'] is True
        assert report['direct_grasp_tcp_position_error_m'] == pytest.approx(error_m)
        assert report['direct_grasp_pose_checks_enabled'] is pose_checks
    else:
        with pytest.raises(RuntimeError, match='direct_grasp: measured TCP target not reached'):
            CollectionPipeline.move_direct_to_grasp(node, selected, report)
        assert 'direct_grasp_executed' not in report
    assert commands == [((arm, positions, 'direct_grasp'),
                         {} if pose_checks else {'allow_endpoint_error': True})]


@pytest.mark.parametrize('arm', ['left', 'right'])
@pytest.mark.parametrize('clear', [False, True])
def test_direct_grasp_uses_checked_joint_connection_before_rrt(arm, clear):
    from geometry_msgs.msg import Pose
    from sensor_msgs.msg import JointState
    from cleany_skill_executor.collection_pipeline import CollectionPipeline
    from cleany_skill_executor.core.physical_grasp_selection import PlannedPath

    names = ARM_JOINT_NAMES[arm]
    measured = JointState(name=list(names), position=[.1]*5)
    positions = (.2, .3, .4, .5, .6)
    target = Pose()
    target.orientation.w = 1.
    calls = []
    checked = PlannedPath(names, ((.1,)*5, positions), (0., 1.), ((0.,)*5, (0.,)*5))
    def validate(selected_arm, path):
        assert selected_arm == arm and path.points == ((.1,)*5, positions)
        calls.append('collision_check')
        return checked if clear else None
    def execute(selected_arm, trajectory, controller):
        assert selected_arm == arm and controller == 'arm'
        assert len(trajectory.points) == 2
        assert tuple(trajectory.points[-1].positions) == positions
        calls.append('execute_direct')
    node = NS(config={'direct_grasp_shortcut_enabled': True},
              scene_monitor=NS(wait=lambda: calls.append('fresh_scene')),
              real=NS(monitor=lambda: measured, value=lambda _: .03, last_execution_metrics={}),
              motion=NS(set_current_state=lambda state: calls.append('initialize'),
                        _validated_cubic_path=validate),
              event=lambda *args, **kwargs: None, execute_trajectory=execute,
              move_joint_pose=lambda *args: calls.append('plan_rrt'),
              pose=lambda _: target, cartesian_pose=CollectionPipeline.cartesian_pose)
    selected = NS(selected_arm=arm, grasp_joint_state=NS(name=names, position=positions),
                  fk_grasp_pose=target)
    report = {}
    CollectionPipeline.move_direct_to_grasp(node, selected, report)
    assert calls == ['fresh_scene', 'initialize', 'collision_check',
                     'execute_direct' if clear else 'plan_rrt']
    assert report['direct_grasp_route'] == ('joint_direct' if clear else 'rrt')


@pytest.mark.parametrize('value', [0, None, 'true'])
def test_direct_grasp_shortcut_policy_requires_boolean(value):
    assert 'direct_grasp_shortcut_enabled must be boolean' in configuration_issues(
        {**config(), 'direct_grasp_shortcut_enabled': value})


@pytest.mark.parametrize('value', [0, None, 'false'])
def test_direct_grasp_pose_policy_requires_boolean(value):
    assert 'direct_grasp_pose_checks_enabled must be boolean' in configuration_issues(
        {**config(), 'direct_grasp_pose_checks_enabled': value})


@pytest.mark.parametrize('positions', [(0.,)*4, (0., 0., 0., 0., float('nan'))])
def test_direct_grasp_rejects_invalid_targets_before_planning(positions):
    from cleany_skill_executor.collection_pipeline import CollectionPipeline
    selected = NS(selected_arm='left', grasp_joint_state=NS(name=ARM_JOINT_NAMES['left'],
                                                           position=positions))
    node = NS(move_joint_pose=lambda *_: pytest.fail('Invalid target was dispatched'))
    with pytest.raises(RuntimeError, match='complete finite selected joint target'):
        CollectionPipeline.move_direct_to_grasp(node, selected, {})


@pytest.mark.parametrize('invalid', [float('nan'), float('inf')])
def test_relaxed_grasp_still_rejects_nonfinite_tcp_feedback(invalid):
    from geometry_msgs.msg import Pose
    from cleany_skill_executor.collection_pipeline import CollectionPipeline
    target, actual = Pose(), Pose()
    target.orientation.w = actual.orientation.w = 1.
    actual.position.x = invalid
    selected = NS(selected_arm='left', grasp_joint_state=NS(name=ARM_JOINT_NAMES['left'],
                   position=[0.]*5), fk_grasp_pose=target)
    node = NS(config={'direct_grasp_pose_checks_enabled': False},
              move_joint_pose=lambda *args, **kwargs: None, event=lambda *args, **kwargs: None,
              pose=lambda _: actual, cartesian_pose=CollectionPipeline.cartesian_pose)
    with pytest.raises(ValueError, match='three finite coordinates'):
        CollectionPipeline.move_direct_to_grasp(node, selected, {})


@pytest.mark.parametrize('arm', ['left', 'right'])
def test_joint_pose_initializes_execution_adapter_before_planning_and_dispatch(arm):
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory
    from cleany_skill_executor.collection_pipeline import CollectionPipeline

    measured = JointState(name=[*ARM_JOINT_NAMES['left'], *ARM_JOINT_NAMES['right'],
                                'left_gripper_joint', 'right_gripper_joint'], position=[.1]*12)
    calls = []
    motion = NS(current=None)
    planner = object()
    def initialize(state):
        motion.current = state
        calls.append('initialize')
    def plan(client, request, timeout):
        assert client is planner
        assert motion.current == measured == request.motion_plan_request.start_state.joint_state
        assert request.motion_plan_request.group_name == f'{arm}_grasp_arm'
        calls.append('plan')
        return NS(motion_plan_response=NS(error_code=NS(val=1),
                                         trajectory=NS(joint_trajectory=JointTrajectory())))
    def execute(selected_arm, trajectory, controller):
        assert motion.current == measured
        assert selected_arm == arm and controller == 'arm'
        calls.append('execute')
    motion.set_current_state = initialize
    node = NS(real=NS(monitor=lambda: measured), motion=motion, joint_plan=planner,
              call=plan, event=lambda stage: calls.append(stage), execute_trajectory=execute)
    CollectionPipeline.move_joint_pose(node, arm, (.2,)*5, 'direct_grasp')
    assert calls == ['initialize', 'plan', 'direct_grasp', 'execute']


@pytest.mark.parametrize('value', ['false', 0, None])
def test_approach_collision_policy_requires_a_boolean(value):
    issues = configuration_issues({**config(), 'approach_collision_check_enabled': value})
    assert 'approach_collision_check_enabled must be boolean' in issues


@pytest.mark.parametrize('key', ['grasp_collision_check_enabled',
                                 'gripper_opening_collision_check_enabled'])
@pytest.mark.parametrize('value', ['false', 0, None])
def test_gripper_collision_policies_require_booleans(key, value):
    issues = configuration_issues({**config(), key: value})
    assert key+' must be boolean' in issues


@pytest.mark.parametrize('value', ['false', 0, None])
def test_lift_collision_policy_requires_a_boolean(value):
    issues = configuration_issues({**config(), 'lift_collision_check_enabled': value})
    assert 'lift_collision_check_enabled must be boolean' in issues


@pytest.mark.parametrize('value', ['false', 0, None])
def test_release_retreat_collision_policy_requires_a_boolean(value):
    issues = configuration_issues({**config(), 'retreat_after_release_collision_check_enabled': value})
    assert 'retreat_after_release_collision_check_enabled must be boolean' in issues


def test_repository_disables_only_release_retreat_collision_policy_by_explicit_setting():
    root = Path(__file__).resolve().parents[4]
    settings = json.loads((root/'ros2_ws/src/cleany_skill_executor/test/data/supervised_collection_fixture.json').read_text())
    assert settings['retreat_after_release_collision_check_enabled'] is False
    assert not configuration_issues(settings)


@pytest.mark.parametrize('key', ['approach_collision_replanning_enabled',
                               'fixed_jaw_clearance_enabled', 'visual_servo_enabled'])
def test_unchecked_straight_approach_rejects_conflicting_modes(key):
    issues = configuration_issues({**config(), 'approach_collision_check_enabled': False,
                                   key: True})
    assert 'approach_collision_check_enabled=false requires '+key+'=false' in issues


@pytest.mark.parametrize('invalid', [
    {'assume_stationary_target': 'true'},
    {'operation_mode': 'full_collection'},
    {'visual_servo_enabled': True},
    {'wrist_observation_enabled': True},
])
def test_stationary_target_assumption_requires_an_explicit_temporary_profile(invalid):
    settings = {**config(), 'operation_mode': 'pick_return_release',
                'temporary_contact_verification': 'raw_feedback',
                'temporary_placement_verification': 'assume_success',
                'label_destinations': {'*': 'return_to_pick'},
                'assume_stationary_target': True, 'visual_servo_enabled': False,
                'wrist_observation_enabled': False}
    assert configuration_issues(settings) == []
    settings.update(invalid)
    assert any('assume_stationary_target' in issue for issue in configuration_issues(settings))


@pytest.mark.parametrize('field,value', [('home_joint_rad',None),('home_joint_rad',[0.]*4),('bins',{}),('bins',{'recycling':[float('nan')]*5})])
def test_missing_or_invalid_destination_never_passes(field,value):
    c=config();c['arms']['left'][field]=value
    assert configuration_issues(c)


def test_gripper_mapping_is_measured_bounded_and_supports_reversed_direction():
    raw=config()['arms']['left'];arm=load_arm(raw)
    assert arm.opening(.04)==pytest.approx(.4)
    with pytest.raises(ValueError): arm.opening(.09)
    raw['gripper']['open_joint_rad']=-1.
    assert load_arm(raw).opening(.04)==pytest.approx(-.6)
    raw['gripper']['measured']=False
    with pytest.raises(ValueError):load_arm(raw)
    raw['gripper']['approved_existing_mapping']=True
    assert load_arm(raw).opening(.04)==pytest.approx(-.6)


def test_startup_home_is_explicit_and_not_confused_with_a_missing_pose():
    raw=config()['arms']['left']
    raw['home_joint_rad']='startup'
    assert load_arm(raw).home is None
    raw['home_joint_rad']=None
    with pytest.raises(ValueError, match='home/bin'):
        load_arm(raw)


def test_single_destination_can_use_a_wildcard_label():
    from cleany_skill_executor.collection_pipeline import CollectionPipeline
    selected=NS(selected_arm='left',selected_object=NS(label='earbud charging case'),
                selected_candidate=NS(required_opening_m=.04),gripper_joint_position=.4)
    arm=load_arm(config()['arms']['left'])
    limits={name:NS(check=lambda _:None) for name in (*ARM_JOINT_NAMES['left'],'left_gripper_joint')}
    node=NS(config={**config(),'label_destinations':{'*':'recycling'}},arm_config=None,
            destination='',real=NS(verify=lambda _:limits, closed_contact_position=lambda *_:0.),
            evidence=NS(wait_for_service=lambda **_:True),
            event=lambda *_,**__:None)
    CollectionPipeline.before_execution(node,selected)
    assert node.destination=='recycling'


@pytest.mark.parametrize('arm,zero,closed', [('left',1345,1358),('right',1359,1372)])
def test_contact_close_uses_each_motor_raw_goal_through_physical_calibration(arm, zero, closed):
    from cleany_skill_executor.hardware_execution import HardwareExecution
    from cleany_skill_executor.core.gripper_contact import load_contact_criteria
    root = Path(__file__).resolve().parents[4]
    limit = PhysicalJointLimit(-.374532977628,1.74532919957,1.,2.,
                               zero,1.,-.374532977628,zero,zero+1394)
    node = NS(contact_criteria=load_contact_criteria(root/'configs/robot/gripper_contact_decision.yaml'))
    target = HardwareExecution.closed_contact_position(node, arm, {f'{arm}_gripper_joint':limit})
    assert round(limit.zero+(target-limit.offset)*4096/math.tau) == closed


@pytest.mark.parametrize('state', ['CONTACT','EMPTY','UNKNOWN'])
def test_live_gate_accepts_only_contact_and_bounds_wait_for_unknown(state):
    from cleany_skill_executor.hardware_execution import HardwareExecution
    from cleany_skill_executor.core.gripper_contact import ContactDecision, ContactState
    decision = ContactDecision(ContactState(state), 'synthetic')
    node = NS(value=lambda _: .001, monitor=lambda:None,
              contact_decision=lambda *_,**__:decision)
    if state == 'CONTACT':
        assert HardwareExecution.wait_for_contact(node, 'right', 1) is decision
    else:
        with pytest.raises(RuntimeError, match='empty' if state == 'EMPTY' else 'unknown'):
            HardwareExecution.wait_for_contact(node, 'right', 1)


def test_payload_monitor_rejects_empty_and_stale_unknown_in_hold_modes(monkeypatch):
    from cleany_skill_executor.collection_pipeline import CollectionPipeline
    from cleany_skill_executor.core.gripper_contact import ContactDecision, ContactState
    clock = [10.]
    monkeypatch.setattr('cleany_skill_executor.collection_pipeline.time.monotonic',lambda:clock[0])
    state = [ContactState.CONTACT]
    node = NS(carrying={'arm':'left'},releasing=False,
              payload_selected=NS(selected_arm='left'),payload_evidence_time=0.,
              config={'operation_mode':'grasp_lift_hold','evidence_max_age_sec':.5},
              real=NS(contact_decision=lambda _:ContactDecision(state[0], 'synthetic')))
    CollectionPipeline.monitor_payload(node)
    assert node.payload_evidence_time == 10.
    state[0] = ContactState.UNKNOWN
    clock[0] = 10.2
    CollectionPipeline.monitor_payload(node)
    clock[0] = 10.6
    with pytest.raises(RuntimeError, match='unknown or stale'):
        CollectionPipeline.monitor_payload(node)
    state[0] = ContactState.EMPTY
    with pytest.raises(RuntimeError, match='contact was lost'):
        CollectionPipeline.monitor_payload(node)


def evidence():
    request=NS(arm='left',reference_id='ref',source_snapshot_id='capture',source_object_id=1,
               after_stamp_ns=9_000_000_000,stage=1,GRASP=1,LIFT=2,RELEASE=3)
    response=NS(success=True,arm='left',reference_id='ref',source_snapshot_id='capture',source_object_id=1,
                header=NS(stamp=NS(sec=10,nanosec=0)),contact_confirmed=True,object_tracked=True,release_confirmed=False)
    return request,response


@pytest.mark.parametrize('field,value',[('contact_confirmed',False),('object_tracked',False),('arm','right'),('reference_id','other'),('source_object_id',2),('success',False)])
def test_motor_arrival_or_wrong_object_cannot_confirm_grasp(field,value):
    request,response=evidence();setattr(response,field,value)
    with pytest.raises(RuntimeError):check_evidence(response,request,10_100_000_000,.5)


def test_evidence_requires_new_capture_and_release_requires_no_contact():
    request,response=evidence();check_evidence(response,request,10_100_000_000,.5)
    for now in (9_900_000_000,11_000_000_000):
        with pytest.raises(RuntimeError):check_evidence(response,request,now,.5)
    request.after_stamp_ns=10_000_000_000
    with pytest.raises(RuntimeError):check_evidence(response,request,10_100_000_000,.5)
    request.after_stamp_ns=9_000_000_000;request.stage=3;response.release_confirmed=True
    with pytest.raises(RuntimeError):check_evidence(response,request,10_100_000_000,.5)
    response.contact_confirmed=False;check_evidence(response,request,10_100_000_000,.5)


def test_hardware_retimer_obeys_driver_limits_and_encoder_bounds():
    limits={'a':PhysicalJointLimit(-2,2,.04,.15,2048,1,0,0,4095)}
    samples,duration=bounded_path(('a',),[(0.,),(1.,)],limits,velocity=.1,acceleration=1.)
    assert duration>40
    assert max(abs(p.derivatives[0]/duration) for p in samples)<=.032
    assert samples[0].derivatives==(0.,) and samples[-1].derivatives==(0.,)
    for points in ([(-3.,),(0.,)],[(0.,),(float('nan'),)]):
        with pytest.raises(ValueError):bounded_path(('a',),points,limits,velocity=.02,acceleration=.05)
    with pytest.raises(ValueError):bounded_path(('right',),[(0.,),(1.,)],limits,velocity=.02,acceleration=.05)


def test_small_start_drift_is_bridged_and_retimed_from_measured_position():
    points = bridge_joint_path([(0.,), (.2,), (.4,)], (.012,), .03)
    limits = {'a': PhysicalJointLimit(-2., 2., .1, .3, 2048., 1., 0., 0., 4095.)}
    samples, duration = bounded_path(('a',), points, limits, velocity=.1, acceleration=.3)
    assert samples[0].positions == (.012,)
    assert samples[-1].positions == pytest.approx((.4,))
    assert samples[0].derivatives == (0.,)
    assert samples[-1].derivatives == pytest.approx((0.,))
    assert max(abs(p.derivatives[0]/duration) for p in samples) <= .08+1e-9
    with pytest.raises(ValueError, match='too far'):
        bridge_joint_path([(0.,), (.2,)], (.031,), .03)


def test_communication_resume_keeps_remaining_path_and_rejects_large_departure():
    assert remaining_joint_path([(0., 0.), (.2, 0.), (.4, .1)], (.15, .005)) == [
        (.15, .005), (.2, 0.), (.4, .1)]
    with pytest.raises(ValueError, match='departed too far'):
        remaining_joint_path([(0., 0.), (.2, 0.)], (.15, .1))


@pytest.mark.parametrize('confirmed', [False, True])
def test_controller_resume_requires_cancel_and_rebuilds_from_current_position(confirmed):
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from cleany_skill_executor.core.hardware_execution import CommunicationRecovered
    from cleany_skill_executor.hardware_execution import HardwareExecution
    names = ARM_JOINT_NAMES['left']
    state = JointState(name=list(names), position=[.15, 0., 0., 0., 0.])
    sent, stops = [], []

    def execute(arm, trajectory, controller, **options):
        sent.append((trajectory, options))
        if len(sent) == 1:
            error = CommunicationRecovered(1)
            error.cancellation_confirmed = confirmed
            raise error
        assert trajectory.points[0].positions == state.position
        assert list(trajectory.points[-1].positions) == [.4, 0., 0., 0., 0.]
        assert options['held_command'] is None
        return state

    fake = NS(_execute=execute, positive_integer=lambda _: 1,
              value=lambda _: .03, wait_for_settle=lambda _: state,
              arm_holds={'left': ArmHold((0.,)*5, (.01,)*5)}, last_execution_metrics={},
              fail_stop=lambda: stops.append(True),
              node=NS(get_logger=lambda: NS(warning=lambda _: None)))
    trajectory = JointTrajectory(joint_names=list(names), points=[
        JointTrajectoryPoint(positions=[0., 0., 0., 0., 0.]),
        JointTrajectoryPoint(positions=[.2, 0., 0., 0., 0.]),
        JointTrajectoryPoint(positions=[.4, 0., 0., 0., 0.])])
    if confirmed:
        assert HardwareExecution._execute_with_recovery(fake, 'left', trajectory, 'arm') is state
        assert len(sent) == 2 and not stops
        assert fake.last_execution_metrics['communication_resumes'][0]['generation'] == 1
        assert 'left' not in fake.arm_holds
    else:
        with pytest.raises(RuntimeError, match='could not confirm'):
            HardwareExecution._execute_with_recovery(fake, 'left', trajectory, 'arm')
        assert len(sent) == 1 and stops == [True]


def test_physical_limit_clamps_to_exact_encoder_boundary():
    limit = PhysicalJointLimit(
        -1.5247769031583274, 2.9513790358919736, 1., 2.,
        2513., -1., 0., 589., 3507.,
    )
    lower, upper = limit.position_bounds()
    assert lower == pytest.approx(-1.5247769031583274)
    assert upper == pytest.approx(2.9513790358919736)
    assert limit.clamp(upper + .02) == pytest.approx(upper)
    assert limit.clamp(lower - .02) == pytest.approx(lower)
    limit.check(limit.clamp(upper + .02))


@pytest.mark.parametrize('outside', [False, True])
def test_release_at_upper_limit_preserves_goal_without_relaxing_range_checks(outside):
    limit = PhysicalJointLimit(-.374532977628, 1.74532919957, 1., 2.,
                               1345., 1., -.37453297762778587, 1323., 2842.)
    goal = math.nextafter(limit.upper, math.inf) if outside else limit.upper
    args = (('left_gripper_joint',), [(-.07694070477797149,), (goal,)],
            {'left_gripper_joint': limit})
    if outside:
        with pytest.raises(ValueError, match='physical joint/calibration range'):
            bounded_path(*args, velocity=1., acceleration=2.)
    else:
        samples, _ = bounded_path(*args, velocity=1., acceleration=2.)
        assert len(samples) == 275
        assert samples[-1].positions == (goal,)
        assert round(limit.zero+(goal-limit.offset)*4096/math.tau) == 2727


@pytest.mark.parametrize('arm', ['left', 'right'])
@pytest.mark.parametrize('direction', [-1, 1])
@pytest.mark.parametrize('enabled,contact_close,turnaround,exempt', [
    (False, False, False, True), (True, False, False, False),
    (False, True, False, False), (False, False, True, False),
])
def test_only_monotone_gripper_opening_can_skip_position_range(
        arm, direction, enabled, contact_close, turnaround, exempt):
    from cleany_skill_executor.hardware_execution import HardwareExecution
    joint = f'{arm}_gripper_joint'
    limit = PhysicalJointLimit(-1., 1., 1., 2., 2048., direction, 0., 0., 4095.)
    # Raw opening direction is +1; calibrated ROS direction can be either sign.
    points = [(direction*.8,), (direction*1.1,)]
    if turnaround: points.append((direction*.9,))
    fake = NS(gripper_opening_position_range_check_enabled=enabled,
              contact_criteria={arm: NS(opening_direction=1)})
    exemptions = HardwareExecution.opening_position_range_exemptions(
        fake, arm, (joint,), points, {joint: limit}, contact_close=contact_close)
    assert exemptions == ((joint,) if exempt else ())
    if exempt:
        samples, _ = bounded_path((joint,), points, {joint: limit}, velocity=1., acceleration=2.,
                                  position_range_exemptions=exemptions)
        assert samples[-1].positions == points[-1]
    else:
        with pytest.raises(ValueError, match='physical joint/calibration range'):
            bounded_path((joint,), points, {joint: limit}, velocity=1., acceleration=2.,
                         position_range_exemptions=exemptions)


def test_opening_position_range_exemption_keeps_arm_closing_and_encoder_limits():
    from cleany_skill_executor.hardware_execution import HardwareExecution
    limit = PhysicalJointLimit(-1., 1., 1., 2., 2048., 1., 0., 0., 4095.)
    fake = NS(gripper_opening_position_range_check_enabled=False,
              contact_criteria={'left': NS(opening_direction=1)})
    joint = 'left_gripper_joint'
    assert HardwareExecution.opening_position_range_exemptions(
        fake, 'left', (joint,), [(.8,), (-1.1,)], {joint: limit}) == ()
    assert HardwareExecution.opening_position_range_exemptions(
        fake, 'left', ('left_elbow_pitch_joint',), [(.8,), (1.1,)],
        {'left_elbow_pitch_joint': limit}) == ()
    with pytest.raises(ValueError, match='physical joint/calibration range'):
        bounded_path((joint,), [(.8,), (4.,)], {joint: limit}, velocity=1., acceleration=2.,
                     position_range_exemptions=(joint,))


def test_operational_profile_disables_only_gripper_opening_position_range_check():
    import yaml
    root = Path(__file__).resolve().parents[4]
    parameters = yaml.safe_load((root/'ros2_ws/src/cleany_skill_executor/test/data/hardware_motion_monitor_fixture.yaml').read_text())
    assert parameters['pregrasp_pipeline']['ros__parameters'][
        'hardware_gripper_opening_position_range_check_enabled'] is False


@pytest.mark.parametrize('enabled', [False, True])
@pytest.mark.parametrize('arm', ['left', 'right'])
def test_opening_range_policy_is_applied_before_sending_the_gripper_goal(monkeypatch, enabled, arm):
    import cleany_skill_executor.hardware_execution as module
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    joint = f'{arm}_gripper_joint'
    state = JointState(name=[joint, 'head_pan_joint', 'head_tilt_joint'], position=[.1, 0., 0.])
    limit = PhysicalJointLimit(-1., 1., 1., 2., 2048., 1., 0., 0., 4095.)
    values = {'hardware_settle_time_sec': .01, 'hardware_start_blend_max_rad': .03,
              'hardware_start_tolerance_rad': .005, 'hardware_arrival_tolerance_rad': .1,
              'hardware_uncommanded_tolerance_rad': .03, 'hardware_velocity_rad_s': .8,
              'hardware_acceleration_rad_s2': 1.2, 'hardware_path_step_rad': .01,
              'hardware_path_simplification_tolerance_rad': .002}
    sent, events = [], []
    def run(_client, goal, _timeout, **_kwargs):
        sent.append(goal)
        state.position[0] = goal.trajectory.points[-1].positions[0]
        return NS(error_code=0, SUCCESSFUL=0)
    node = NS(run_action=run, event=lambda stage, **fields: events.append(stage),
              get_logger=lambda: NS(info=lambda _: None))
    fake = NS(node=node, verify=lambda _: {joint: limit}, monitor=lambda: deepcopy(state),
              value=values.__getitem__, arm_holds={}, uncommanded=None, last_execution_metrics={},
              gripper_opening_position_range_check_enabled=enabled,
              contact_criteria={arm: NS(opening_direction=1)},
              fail_stop=lambda: pytest.fail('range validation precedes motion submission'))
    monkeypatch.setattr(module, 'ActionClient', lambda *_: NS(destroy=lambda: None))
    monkeypatch.setattr(module.time, 'sleep', lambda _: None)
    trajectory = JointTrajectory(joint_names=[joint], points=[
        JointTrajectoryPoint(positions=[.1]), JointTrajectoryPoint(positions=[1.1])])
    if enabled:
        with pytest.raises(ValueError, match='physical joint/calibration range'):
            module.HardwareExecution._execute(fake, arm, trajectory, 'gripper', check_collisions=False)
        assert not sent
    else:
        module.HardwareExecution._execute(fake, arm, trajectory, 'gripper', check_collisions=False)
        assert len(sent) == 1 and tuple(sent[0].trajectory.points[-1].positions) == (1.1,)
        assert events == ['gripper_opening_position_range_check_disabled']


@pytest.mark.parametrize('arm', ['left', 'right'])
@pytest.mark.parametrize('position,angle,check_orientation,accepted', [
    (.097, 0., True, True),
    (.1, .01, True, True),
    (.08, 0., True, False),
    (.1, .2, True, False),
    (.1, .2, False, True),
    (.08, .2, False, False),
], ids=['position-within-limit', 'orientation-within-limit', 'position-outside-limit',
        'orientation-outside-limit', 'orientation-not-required', 'position-still-required'])
@pytest.mark.parametrize('allow_endpoint_error', [False, True])
def test_endpoint_verification_never_sends_post_arrival_corrections(
    arm, position, angle, check_orientation, accepted, allow_endpoint_error,
):
    from sensor_msgs.msg import JointState
    import cleany_skill_executor.hardware_execution as module

    names = ARM_JOINT_NAMES[arm]
    state = JointState(name=list(names), position=[position, angle, 0., 0., 0.])
    target = (.1, 0., 0., 0., 0.)
    limits = {name: PhysicalJointLimit(-2., 2., 1., 2., 2048., 1., 0., 0., 4095.)
              for name in names}
    values = {
        'hardware_tcp_position_tolerance_m': .005,
        'hardware_tcp_orientation_tolerance_deg': 2.,
    }
    settlements = []
    messages = []

    def settle(joints):
        settlements.append(tuple(joints))
        return deepcopy(state)

    def forward_kinematics(_arm, solution):
        half_angle = solution.positions[1] / 2
        return NS(position=(solution.positions[0], 0., 0.),
                  orientation_xyzw=(0., 0., math.sin(half_angle), math.cos(half_angle)))

    fake = NS(
        arm_holds={},
        node=NS(motion=NS(forward_kinematics=forward_kinematics),
                get_logger=lambda: NS(info=messages.append, warning=messages.append)),
        value=values.__getitem__,
        wait_for_settle=settle,
        execute=lambda *_args, **_kwargs: pytest.fail(
            'endpoint verification must not send another motion goal'),
    )
    fake.endpoint_metrics = lambda *args, **kwargs: module.HardwareExecution.endpoint_metrics(
        fake, *args, **kwargs)

    if accepted or allow_endpoint_error:
        actual, metrics = module.HardwareExecution.verify_arm_endpoint(
            fake, arm, target, limits, check_orientation=check_orientation,
            allow_endpoint_error=allow_endpoint_error)
        assert metrics['endpoint_accuracy_checks_enabled'] is not allow_endpoint_error
        assert metrics['endpoint_within_tolerance'] is accepted
        assert actual.position == pytest.approx(state.position)
        assert metrics['endpoint_corrections'] == 0
        assert fake.arm_holds[arm] == ArmHold(target, target)
    else:
        with pytest.raises(RuntimeError, match='Physical TCP endpoint outside tolerance'):
            module.HardwareExecution.verify_arm_endpoint(
                fake, arm, target, limits, check_orientation=check_orientation)
        assert fake.arm_holds == {}
    assert settlements == [names]
    assert fake.last_execution_metrics['endpoint_corrections'] == 0
    assert len(fake.last_execution_metrics['endpoint_history']) == 1
    assert len(messages) == (2 if allow_endpoint_error and not accepted else 1)
    assert json.loads(messages[0])['stage'] == (
        'hardware_endpoint_verified' if accepted else
        'hardware_endpoint_error_ignored' if allow_endpoint_error else 'hardware_endpoint_validation_failed')


def test_confirmed_fail_stop_is_idempotent():
    import cleany_skill_executor.hardware_execution as module

    report = {
        'requested': True,
        'controllers_stopped': True,
        'physical_stop_verified': False,
    }
    fake = NS(stop_report=report)

    assert module.HardwareExecution.fail_stop(fake) == report


def test_broken_bt_stop_handoff_still_requests_independent_stop():
    from cleany_skill_executor.hardware_execution import HardwareExecution
    from cleany_skill_executor.core.physical_grasp_selection import InfrastructureError
    stops, errors = [], []
    def broken(_):
        raise RuntimeError('stop handoff failed')
    fake = NS(node=NS(defer_stop_assessment=broken,
                      get_logger=lambda: NS(error=errors.append)),
              fail_stop=lambda: stops.append(True))
    HardwareExecution.stop_after_execution_error(fake, InfrastructureError('scene is stale'))
    assert stops == [True]
    assert 'stop handoff failed' in errors[0]


def test_grasp_endpoint_verification_can_ignore_orientation_error():
    from sensor_msgs.msg import JointState
    import cleany_skill_executor.hardware_execution as module

    names = ARM_JOINT_NAMES['left']
    state = JointState(name=list(names), position=[.1, .2, 0., 0., 0.])
    target = (.1, 0., 0., 0., 0.)
    limits = {
        name: PhysicalJointLimit(-2., 2., 1., 2., 2048., 1., 0., 0., 4095.)
        for name in names
    }
    values = {
        'hardware_tcp_position_tolerance_m': .008,
        'hardware_tcp_orientation_tolerance_deg': 6.,
    }

    def forward_kinematics(_arm, solution):
        half_angle = solution.positions[1] / 2
        return NS(
            position=(solution.positions[0], 0., 0.),
            orientation_xyzw=(0., 0., math.sin(half_angle), math.cos(half_angle)),
        )

    fake = NS(
        arm_holds={},
        node=NS(
            motion=NS(forward_kinematics=forward_kinematics),
            get_logger=lambda: NS(info=lambda _: None),
        ),
        value=values.__getitem__,
        positive_integer=lambda _: 3,
        wait_for_settle=lambda _: deepcopy(state),
        execute=lambda *_args, **_kwargs: pytest.fail(
            'orientation-only error must not trigger a correction'),
    )
    fake.endpoint_metrics = lambda *args, **kwargs: module.HardwareExecution.endpoint_metrics(
        fake, *args, **kwargs)

    actual, metrics = module.HardwareExecution.verify_arm_endpoint(
        fake, 'left', target, limits, check_orientation=False)

    assert actual.position == pytest.approx(state.position)
    assert metrics['endpoint_corrections'] == 0
    assert metrics['final_tcp_position_error_m'] == 0.
    assert metrics['final_tcp_orientation_error_deg'] > 6.


def contact(body_1='<octomap>', body_2='left_gripper_frame', depth=.004):
    return NS(
        contact_body_1=body_1,
        contact_body_2=body_2,
        depth=depth,
        position=NS(x=.37, y=.09, z=.42),
    )


def test_only_octomap_contacts_with_commanded_arm_are_transient():
    response = NS(valid=False, contacts=[contact()])
    assert transient_octomap_arm_collision(response, 'left')
    assert collision_contact_details(response) == [{
        'body_1': '<octomap>',
        'body_2': 'left_gripper_frame',
        'depth_m': .004,
        'position_m': [.37, .09, .42],
    }]
    assert not transient_octomap_arm_collision(
        NS(valid=False, contacts=[contact(body_1='table')]), 'left')
    assert not transient_octomap_arm_collision(
        NS(valid=False, contacts=[contact(body_2='right_gripper_frame')]),
        'left',
    )
    assert not transient_octomap_arm_collision(
        NS(valid=False, contacts=[]), 'left')


def test_frozen_sensor_scene_is_nested_and_always_resumed():
    import cleany_skill_executor.hardware_execution as module

    waits = []
    updates = []
    fake = object.__new__(module.HardwareExecution)
    fake._sensor_scene_freeze_depth = 0
    fake.wait_for_fresh_sensor_scene = waits.append
    fake.set_sensor_scene_updates = updates.append
    fake.node = NS(get_logger=lambda: NS(error=lambda _: None),
                   get_parameter=lambda _: NS(value=False))

    with pytest.raises(RuntimeError, match='test failure'):
        with fake.frozen_sensor_scene():
            assert fake.sensor_scene_frozen
            with fake.frozen_sensor_scene():
                assert fake._sensor_scene_freeze_depth == 2
            raise RuntimeError('test failure')

    assert waits == ['pre_motion_octomap']
    assert updates == [False, True]
    assert not fake.sensor_scene_frozen


@pytest.mark.parametrize('obstacle', [
    contact(),
    contact(body_1='table'),
])
def test_frozen_collision_validation_rejects_any_contact_without_refresh(obstacle):
    import cleany_skill_executor.hardware_execution as module
    from sensor_msgs.msg import JointState

    names = ARM_JOINT_NAMES['left']
    state = JointState(name=list(names), position=[0.] * len(names))
    fake = NS(
        node=NS(
            call=lambda *_: NS(
                valid=False,
                contacts=[obstacle],
            ),
            get_logger=lambda: NS(info=lambda _: None, warning=lambda _: None),
        ),
        validity=object(),
        monitor=lambda: deepcopy(state),
        sensor_scene_frozen=True,
    )

    with pytest.raises(RuntimeError, match='collision validation'):
        module.HardwareExecution.validate_collision_samples(
            fake,
            'left',
            names,
            [NS(positions=(0.,) * len(names))],
            dict(zip(names, state.position)),
            state,
        )


def test_endpoint_settle_requires_a_full_stable_measurement_window(monkeypatch):
    from sensor_msgs.msg import JointState
    import cleany_skill_executor.hardware_execution as module

    names = ARM_JOINT_NAMES['left']
    state = JointState(name=list(names), position=[.1, .2, .3, .4, .5])
    clock = iter((0., 0., .1, .2, .31))
    monkeypatch.setattr(module.time, 'monotonic', lambda: next(clock))
    monkeypatch.setattr(module.time, 'sleep', lambda _: None)
    values = {
        'hardware_endpoint_settle_duration_sec': .3,
        'hardware_endpoint_settle_timeout_sec': 2.,
        'hardware_endpoint_settle_velocity_rad_s': .02,
    }
    fake = NS(value=values.__getitem__, monitor=lambda: deepcopy(state))

    settled = module.HardwareExecution.wait_for_settle(fake, names)

    assert settled.position == state.position


@pytest.mark.parametrize('index,bridge_count,phase', [
    (0, 0, 'command_path'), (2, 0, 'command_path'),
    (0, 4, 'hold_bridge'), (2, 4, 'hold_bridge'),
    (3, 4, 'hold_bridge'), (4, 4, 'command_path'),
])
def test_collision_diagnostics_separate_mandatory_entry_from_later_path(index, bridge_count, phase):
    import cleany_skill_executor.hardware_execution as module
    from cleany_skill_executor.core.hardware_execution import CollisionValidationError
    from sensor_msgs.msg import JointState
    names, checked, logged = ARM_JOINT_NAMES['right'], [], []
    state = JointState(name=list(names), position=[0.] * len(names))
    def call(_, request):
        checked.append(request.robot_state.joint_state.position)
        return NS(valid=len(checked)-1 != index, contacts=[contact(body_2='right_gripper_frame')])
    fake = NS(node=NS(call=call, get_logger=lambda: NS(warning=logged.append)),
              validity=object(), monitor=lambda: deepcopy(state), sensor_scene_frozen=False)
    with pytest.raises(CollisionValidationError) as rejected:
        module.HardwareExecution.validate_collision_samples(
            fake, 'right', names, [NS(positions=(0.,)*5) for _ in range(6)],
            dict(zip(names, state.position)), state, hold_bridge_samples=bridge_count)
    error, details = rejected.value, json.loads(logged[0])
    assert error.phase == details['phase'] == phase
    assert error.sample_index == details['sample_index'] == index
    assert error.entry_blocked is (phase == 'hold_bridge' or index == 0)
    assert error.contacts and error.octomap_arm_contact and not error.motion_started
    assert len(checked) == index+1  # Stop at the first rejected sample.


def test_collision_on_a_communication_resume_is_marked_as_already_started():
    from cleany_skill_executor.hardware_execution import HardwareExecution
    from cleany_skill_executor.core.hardware_execution import CollisionValidationError, CommunicationRecovered
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    names, calls = ARM_JOINT_NAMES['left'], []
    state = JointState(name=list(names), position=[.05, 0., 0., 0., 0.])
    error = CollisionValidationError('collision after resume', phase='command_path', sample_index=2)
    def execute(*args, **kwargs):
        calls.append(True)
        if len(calls) == 1:
            interrupted = CommunicationRecovered(1)
            interrupted.cancellation_confirmed = True
            raise interrupted
        raise error
    fake = NS(_execute=execute, positive_integer=lambda _: 1, value=lambda _: .03,
              wait_for_settle=lambda _: state, arm_holds={},
              node=NS(get_logger=lambda: NS(warning=lambda _: None)))
    trajectory = JointTrajectory(joint_names=list(names), points=[
        JointTrajectoryPoint(positions=[0., 0., 0., 0., 0.]),
        JointTrajectoryPoint(positions=[.1, 0., 0., 0., 0.])])
    with pytest.raises(CollisionValidationError):
        HardwareExecution._execute_with_recovery(fake, 'left', trajectory, 'arm')
    assert error.motion_started and len(calls) == 2


def test_synchronized_collision_after_communication_resume_is_not_a_new_plan_rejection():
    from contextlib import nullcontext
    from cleany_skill_executor.hardware_execution import HardwareExecution
    from cleany_skill_executor.core.hardware_execution import CollisionValidationError, CommunicationRecovered
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    names, calls = ARM_JOINT_NAMES['left'], []
    state = JointState(name=[*names, 'left_gripper_joint'], position=[.05, 0., 0., 0., 0., .2])
    error = CollisionValidationError('collision after resume', phase='command_path', sample_index=2)
    def execute(*args, **kwargs):
        calls.append(True)
        if len(calls) == 1:
            interrupted = CommunicationRecovered(1)
            interrupted.cancellation_confirmed = True
            raise interrupted
        raise error
    fake = NS(_execute_pregrasp_parallel=execute, positive_integer=lambda _: 1, value=lambda _: .03,
              wait_for_settle=lambda _: state, arm_holds={}, frozen_sensor_scene=nullcontext,
              node=NS(get_logger=lambda: NS(warning=lambda _: None)))
    trajectory = JointTrajectory(joint_names=list(names), points=[
        JointTrajectoryPoint(positions=[0., 0., 0., 0., 0.]),
        JointTrajectoryPoint(positions=[.1, 0., 0., 0., 0.])])
    jaw = JointTrajectory(joint_names=['left_gripper_joint'], points=[
        JointTrajectoryPoint(positions=[0.]), JointTrajectoryPoint(positions=[.4])])
    with pytest.raises(CollisionValidationError):
        HardwareExecution.execute_pregrasp_parallel(fake, 'left', trajectory, jaw, .3, 1)
    assert error.motion_started and len(calls) == 2


def test_physical_driver_gate_rejects_mock_readonly_and_uncommandable_arm():
    params={'lower':-2,'upper':2,'max_velocity':.04,'max_acceleration':.15,'zero_tick':2048,
            'direction':1,'offset_rad':0,'range_min':0,'range_max':4095}
    joints=''.join(f'<joint name="{name}"><command_interface name="position"/>'+''.join(f'<param name="{k}">{v}</param>' for k,v in params.items())+'</joint>' for name in (*ARM_JOINT_NAMES['left'],'left_gripper_joint'))
    xml='<robot><ros2_control><hardware><plugin>cleany_feetech_hardware/FeetechSystem</plugin><param name="read_only">false</param></hardware>'+joints+'</ros2_control></robot>'
    assert len(physical_limits(xml,'left'))==6
    for invalid in (xml.replace('false','true'),xml.replace('cleany_feetech_hardware/FeetechSystem','mock/Driver'),xml.replace('<command_interface name="position"/>','')):
        with pytest.raises(ValueError):physical_limits(invalid,'left')
    with pytest.raises(ValueError):physical_limits(xml,'right')


def test_collection_sequence_stops_before_transport_without_lift_evidence():
    from cleany_skill_executor.collection_pipeline import CollectionPipeline
    from cleany_skill_executor.pregrasp_pipeline import PregraspPipeline
    from geometry_msgs.msg import Pose
    events=[];current=Pose();current.orientation.w=1.
    node=NS(config=config(),arm_config=load_arm(config()['arms']['left']),destination='recycling',carrying=None,
            _grasp_scene_freeze_held=True,
            real=NS(end_sensor_scene_freeze=lambda:events.append('octomap-resume')),
            scene=NS(begin=lambda *_:events.append('scene'),allow_contacts_for=lambda *_:None,
                     restore=lambda:events.append('restore')),pose=lambda *_:deepcopy(current),
            get_clock=lambda:NS(now=lambda:NS(nanoseconds=10)),releasing=False)
    def linear(arm,target,stage,*_,**__):
        events.append(stage);current.position=deepcopy(target.position)
    def attach(*_):node.carrying={'release_confirmed':False};events.append('attach')
    def detach():node.carrying=None;events.append('detach')
    node.linear=linear;node.close_until_contact=lambda *_:events.append('contact');node.attach=attach;node.detach=detach
    node.approach_target=lambda selected, report:CollectionPipeline.approach_target(node,selected,report)
    node.record_approach_commands=lambda arm:CollectionPipeline.record_approach_commands(node,arm)
    node.record_pickup_departure=lambda arm:CollectionPipeline.record_pickup_departure(node,arm)
    node.retreat_to_pregrasp=lambda *args,**kwargs:CollectionPipeline.retreat_to_pregrasp(node,*args,**kwargs)
    node.selected_grasp_roll=CollectionPipeline.selected_grasp_roll
    node.stop_wrist_camera=lambda:events.append('camera-off')
    node.wrist_observation_required=lambda:False
    node.jaw=lambda arm,q,stage,**options:events.append(stage)
    node.move_joint_pose=lambda arm,q,stage:events.append(stage)
    def fail_lift(*args):raise RuntimeError('contact lost')
    node.verify_evidence=fail_lift
    selected=NS(selected_arm='left',selected_candidate=NS(),fk_grasp_pose=Pose(),
                grasp_joint_state=NS(name=['left_wrist_roll_joint'], position=[0.]))
    report={}
    with pytest.raises(RuntimeError,match='contact lost'):
        CollectionPipeline.after_pregrasp(node,selected,NS(reference_id='wrist'),report)
    assert events==['scene','approach','contact','attach','retreat','lift']
    assert node._grasp_scene_freeze_held
    assert report['payload_preserved_on_failure'] and node.carrying is not None
    events.clear();node.carrying=None;node.verify_evidence=lambda *_:events.append('evidence')
    node._grasp_scene_freeze_held=True
    CollectionPipeline.after_pregrasp(node,selected,NS(reference_id='wrist'),report)
    assert events==['scene','approach','contact','attach','retreat','lift','evidence','camera-off',
                    'transport','release','evidence','detach','restore','return','restore']
    assert node._grasp_scene_freeze_held
    assert report['returned'] and report['released']
    PregraspPipeline.release_grasp_scene_freeze(node)
    assert events[-1]=='octomap-resume'
    assert not node._grasp_scene_freeze_held


def test_return_hold_mode_requires_contact_and_lift_evidence_and_keeps_payload():
    from cleany_skill_executor.collection_pipeline import CollectionPipeline
    from geometry_msgs.msg import Pose
    events=[];current=Pose();current.orientation.w=1.
    settings={**config(),'operation_mode':'grasp_lift_return_hold','label_destinations':{}}
    settings['arms']['left']['bins']={}
    node=NS(config=settings,arm_config=load_arm(settings['arms']['left'],require_bins=False),
            destination='',carrying=None,
            _grasp_scene_freeze_held=True,
            real=NS(end_sensor_scene_freeze=lambda:events.append('octomap-resume')),
            scene=NS(begin=lambda *_:events.append('scene'),allow_contacts_for=lambda *_:None,
                     restore=lambda:events.append('restore')),pose=lambda *_:deepcopy(current),
            get_clock=lambda:NS(now=lambda:NS(nanoseconds=10)),releasing=False)
    def linear(arm,target,stage,*_,**__):
        events.append(stage);current.position=deepcopy(target.position)
    def attach(*_):node.carrying={'release_confirmed':False};events.append('attach')
    node.linear=linear;node.attach=attach
    node.approach_target=lambda selected, report:CollectionPipeline.approach_target(node,selected,report)
    node.record_approach_commands=lambda arm:CollectionPipeline.record_approach_commands(node,arm)
    node.record_pickup_departure=lambda arm:CollectionPipeline.record_pickup_departure(node,arm)
    node.retreat_to_pregrasp=lambda *args,**kwargs:CollectionPipeline.retreat_to_pregrasp(node,*args,**kwargs)
    node.selected_grasp_roll=CollectionPipeline.selected_grasp_roll
    node.stop_wrist_camera=lambda:events.append('camera-off')
    node.wrist_observation_required=lambda:False
    node.jaw=lambda arm,q,stage,**options:events.append(stage)
    node.move_joint_pose=lambda arm,q,stage:events.append(stage)
    node.event=lambda stage,**_:events.append(stage)
    def contact(_, report):
        events.append('contact')
        report.update(grasp_contact_confirmed=True, grasp_assumed=False)
    node.close_until_contact=contact
    node.verify_evidence=lambda *_:events.append('evidence')
    selected=NS(selected_arm='left',selected_candidate=NS(),fk_grasp_pose=Pose(),
                grasp_joint_state=NS(name=['left_wrist_roll_joint'], position=[0.]))
    report={}
    CollectionPipeline.after_pregrasp(node,selected,None,report)
    assert events==['scene','approach','contact',
                    'attach','retreat','lift','evidence','camera-off','return_holding_payload']
    assert node._grasp_scene_freeze_held
    assert not report['grasp_assumed'] and report['grasp_verified']
    assert report['payload_held'] and report['payload_preserved_on_success']
    assert node.carrying is not None


def test_payload_attachment_does_not_remove_world_object_twice():
    from geometry_msgs.msg import Pose, Vector3
    from moveit_msgs.msg import PlanningScene
    from std_msgs.msg import Header
    from sensor_msgs.msg import JointState
    from cleany_skill_executor.collection_pipeline import CollectionPipeline

    applied = []
    events = []
    selected = NS(
        selected_arm='left',
        selected_candidate=NS(header=Header(frame_id='base_link')),
        selected_object=NS(
            obb_size=Vector3(x=.1, y=.05, z=.03),
            obb_pose=Pose(),
        ),
    )
    node = NS(
        apply_scene=object(),
        get_scene=object(),
        config={},
        carrying=None,
        payload_selected=None,
        payload_evidence_time=0.,
        get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: Header().stamp)),
        pose=lambda _arm: Pose(),
        measured=lambda: JointState(
            name=['left_wrist_roll_joint'], position=[.25]),
        event=lambda stage, **fields: events.append((stage, fields)),
    )

    def apply(scene: PlanningScene) -> None:
        applied.append(scene)

    def call(_client, _request):
        return NS(scene=NS(robot_state=NS(
            attached_collision_objects=list(
                applied[-1].robot_state.attached_collision_objects))))

    node.apply = apply
    node.call = call

    CollectionPipeline.attach(node, selected, 'payload')

    assert len(applied) == 1
    assert applied[0].world.collision_objects == []
    attached = applied[0].robot_state.attached_collision_objects
    assert len(attached) == 1
    assert attached[0].object.id == 'payload'
    assert node.carrying['object_id'] == 'payload'
    assert events[0][0] == 'attach'


def test_service_failure_without_message_reports_service_name():
    from cleany_skill_executor.pregrasp_pipeline import PregraspPipeline

    client = NS(
        srv_name='/apply_planning_scene',
        wait_for_service=lambda **_: True,
        call_async=lambda _request: object(),
    )
    node = NS(wait=lambda _future, _timeout, **_: NS(success=False))

    with pytest.raises(RuntimeError, match='Service call failed: /apply_planning_scene'):
        PregraspPipeline.call(node, client, object())


@pytest.mark.parametrize('segment,expected', [(None, [.005, .01, .012]), (.02, [.012])])
def test_visual_approach_reobserves_after_each_bounded_cartesian_segment(segment, expected):
    from cleany_skill_executor.collection_pipeline import CollectionPipeline
    from geometry_msgs.msg import Pose
    from sensor_msgs.msg import JointState

    def observation(stamp):
        return NS(
            header=NS(stamp=NS(sec=stamp, nanosec=0), frame_id='base_link'),
            observed_center=NS(x=.3, y=.1, z=.2),
            observed_extent=NS(x=.05, y=.05, z=.1),
            mask_pixels=1000, valid_depth_points=990,
            valid_depth_fraction=.99,
        )

    current = Pose()
    current.orientation.w = 1.
    target = Pose()
    target.position.x = .012
    target.orientation.w = 1.
    moves = []
    orientation_checks = []
    captures = iter((observation(2), observation(3), observation(4)))
    measured = JointState(
        name=['left_wrist_roll_joint'], position=[0.])

    def linear(
        _arm, waypoint, _stage, wrist_roll_target=None, *, check_orientation=True,
    ):
        current.position = deepcopy(waypoint.position)
        current.orientation = deepcopy(waypoint.orientation)
        moves.append((current.position.x, wrist_roll_target))
        orientation_checks.append(check_orientation)
        measured.position[0] = wrist_roll_target
        state = JointState()
        state.header.stamp.sec = len(moves)
        return state

    settings = {
        **config(), 'visual_servo_enabled': True,
        'visual_approach_step_m': .005,
        'visual_approach_max_steps': 6,
        'visual_servo_maximum_correction_m': .03,
        'visual_grasp_capture_depth_m': 0.,
        'visual_grasp_capture_lateral_tolerance_m': 0.,
        'cartesian_tolerance_m': .001,
    }
    if segment is not None:
        settings['visual_approach_segment_m'] = segment
    node = NS(
        config=settings,
        cartesian_pose=CollectionPipeline.cartesian_pose,
        pose_message=CollectionPipeline.pose_message,
        pose=lambda _arm: deepcopy(current),
        linear=linear,
        observe=lambda _reference, _after: next(captures),
        real=NS(value=lambda _: 2., monitor=lambda: deepcopy(measured)),
        event=lambda *_args, **_kwargs: None,
    )
    selected = NS(selected_arm='left', reference_id='reference',
                  fk_grasp_pose=target,
                  grasp_joint_state=JointState(
                      name=['left_wrist_roll_joint'], position=[.12]))
    report = {}

    CollectionPipeline.visual_approach(
        node, selected, observation(1), observation(1), report)

    assert [position for position, _ in moves] == pytest.approx(expected)
    assert [roll for _, roll in moves] == pytest.approx([x*10 for x in expected])
    assert orientation_checks == [False] * len(expected)
    assert report['visual_approach']['steps'] == len(expected)
    assert len(report['visual_approach']['observations']) == len(expected)+1


def test_visual_grasp_approach_completes_on_position_without_orientation_gate():
    from cleany_skill_executor.collection_pipeline import CollectionPipeline
    from geometry_msgs.msg import Pose
    from sensor_msgs.msg import JointState

    current = Pose()
    current.position.x = .3
    current.position.y = .1
    current.position.z = .4
    current.orientation.w = 1.
    target = deepcopy(current)
    target.orientation.w = 0.
    target.orientation.x = 1.
    observation = NS(
        header=NS(stamp=NS(sec=1, nanosec=0), frame_id='base_link'),
        observed_center=NS(x=.2, y=.1, z=.3),
        observed_extent=NS(x=.05, y=.04, z=.03),
        mask_pixels=100,
        valid_depth_points=100,
        valid_depth_fraction=1.,
    )
    node = NS(
        config={**config(), 'visual_approach_max_steps': 3,
                'visual_approach_step_m': .005,
                'visual_servo_maximum_correction_m': .03,
                'cartesian_tolerance_m': .001},
        cartesian_pose=CollectionPipeline.cartesian_pose,
        pose=lambda _arm: deepcopy(current),
        linear=lambda *_args, **_kwargs: pytest.fail(
            'position already reached; no trajectory is required'),
        event=lambda *_args, **_kwargs: None,
    )
    selected = NS(
        selected_arm='left', reference_id='reference', fk_grasp_pose=target,
        grasp_joint_state=JointState(
            name=['left_wrist_roll_joint'], position=[0.]),
    )
    report = {}

    CollectionPipeline.visual_approach(
        node, selected, observation, observation, report)

    assert report['visual_approach']['steps'] == 0
    assert report['visual_approach']['observations'][0][
        'tcp_orientation_remaining_deg'] == pytest.approx(180.)


def test_visual_approach_closes_without_more_motion_inside_capture_corridor():
    from cleany_skill_executor.collection_pipeline import CollectionPipeline
    from geometry_msgs.msg import Pose
    from sensor_msgs.msg import JointState

    current = Pose()
    current.orientation.w = 1.
    target = Pose()
    target.position.y = -.04
    target.orientation.w = 1.
    observation = NS(
        header=NS(stamp=NS(sec=1, nanosec=0), frame_id='base_link'),
        observed_center=NS(x=.2, y=.1, z=.3),
        observed_extent=NS(x=.05, y=.04, z=.03),
        mask_pixels=100, valid_depth_points=100,
        valid_depth_fraction=1.,
    )
    events = []
    node = NS(
        config={
            **config(), 'visual_approach_max_steps': 3,
            'visual_approach_step_m': .005,
            'visual_servo_maximum_correction_m': .03,
            'visual_grasp_capture_depth_m': .045,
            'visual_grasp_capture_lateral_tolerance_m': .012,
            'cartesian_tolerance_m': .001,
        },
        cartesian_pose=CollectionPipeline.cartesian_pose,
        pose=lambda _arm: deepcopy(current),
        linear=lambda *_args, **_kwargs: pytest.fail(
            'capture-ready pose must proceed to close without another move'),
        real=NS(monitor=lambda: pytest.fail(
            'capture-ready pose must not prepare another move')),
        event=lambda stage, **kwargs: events.append((stage, kwargs)),
    )
    selected = NS(
        selected_arm='left', reference_id='reference', fk_grasp_pose=target,
        grasp_joint_state=JointState(
            name=['left_wrist_roll_joint'], position=[0.]),
    )
    report = {}

    CollectionPipeline.visual_approach(
        node, selected, observation, observation, report)

    assert report['visual_approach']['steps'] == 0
    assert report['visual_approach'][
        'finalized_from_grasp_capture_corridor'] is True
    assert report['visual_approach']['observations'][0][
        'inside_grasp_capture_corridor'] is True
    assert events[-1][0] == 'visual_capture_finalized'
    assert events[-1][1]['final_motion_skipped'] is True


def test_visual_approach_discards_one_centroid_jump_after_confirmation():
    from cleany_skill_executor.collection_pipeline import CollectionPipeline
    from geometry_msgs.msg import Pose
    from sensor_msgs.msg import JointState

    def observation(stamp, x=0.):
        return NS(
            header=NS(stamp=NS(sec=stamp, nanosec=0), frame_id='base_link'),
            observed_center=NS(x=x, y=0., z=0.),
            observed_extent=NS(x=.05, y=.04, z=.06),
            mask_pixels=1000, valid_depth_points=990,
            valid_depth_fraction=.99,
        )

    current = Pose()
    current.orientation.w = 1.
    target = Pose()
    target.position.y = -.04
    target.orientation.w = 1.
    moves = []
    captures = iter([
        observation(2, .04),
        observation(3),
        observation(4),
        observation(5),
    ])
    measured = JointState(name=['left_wrist_roll_joint'], position=[0.])

    def linear(_arm, waypoint, stage, wrist_roll_target=None, **_kwargs):
        current.position = deepcopy(waypoint.position)
        moves.append((stage, current.position.y))
        measured.position[0] = wrist_roll_target
        state = JointState()
        state.header.stamp.sec = len(moves)
        return state

    settings = {
        **config(), 'visual_servo_enabled': True,
        'visual_approach_step_m': .005,
        'visual_approach_segment_m': .02,
        'visual_approach_max_steps': 5,
        'visual_servo_maximum_correction_m': .03,
        'visual_servo_hard_correction_limit_m': .06,
        'visual_observation_confirmation_frames': 3,
        'visual_observation_consistency_m': .005,
        'visual_observation_increment_limit_m': .01,
        'visual_grasp_capture_depth_m': .005,
        'visual_grasp_capture_lateral_tolerance_m': .005,
        'cartesian_tolerance_m': .001,
    }
    node = NS(
        config=settings,
        cartesian_pose=CollectionPipeline.cartesian_pose,
        pose_message=CollectionPipeline.pose_message,
        pose=lambda _arm: deepcopy(current),
        linear=linear,
        observe=lambda _reference, _after: next(captures),
        real=NS(monitor=lambda: deepcopy(measured)),
        event=lambda *_args, **_kwargs: None,
    )
    selected = NS(
        selected_arm='left', reference_id='reference', fk_grasp_pose=target,
        grasp_joint_state=JointState(
            name=['left_wrist_roll_joint'], position=[0.]),
    )
    report = {}

    CollectionPipeline.visual_approach(
        node, selected, observation(1), observation(1), report)

    assert [stage for stage, _ in moves] == [
        'visual_approach_step', 'visual_approach_step']
    assert [position for _, position in moves] == pytest.approx([-.02, -.04])
    assert report['visual_approach'][
        'finalized_from_grasp_capture_corridor'] is False
    assert report['visual_approach']['observations'][1][
        'confirmation_frames'] == 3
    assert report['visual_approach']['observations'][1][
        'target_surface_correction_m'] == 0.


def test_confirmed_centroid_jump_near_jaws_finishes_from_last_stable_target():
    from cleany_skill_executor.collection_pipeline import CollectionPipeline
    from geometry_msgs.msg import Pose
    from sensor_msgs.msg import JointState

    def observation(stamp, x=0.):
        return NS(
            header=NS(stamp=NS(sec=stamp, nanosec=0), frame_id='base_link'),
            observed_center=NS(x=x, y=0., z=0.),
            observed_extent=NS(x=.05, y=.04, z=.06),
            mask_pixels=1000, valid_depth_points=990,
            valid_depth_fraction=.99,
        )

    current = Pose()
    current.orientation.w = 1.
    target = Pose()
    target.position.y = -.04
    target.orientation.w = 1.
    moves = []
    captures = iter([
        observation(2, .04),
        observation(3, .04),
        observation(4, .04),
    ])
    measured = JointState(name=['left_wrist_roll_joint'], position=[0.])

    def linear(_arm, waypoint, stage, wrist_roll_target=None, **_kwargs):
        current.position = deepcopy(waypoint.position)
        moves.append((stage, current.position.y))
        measured.position[0] = wrist_roll_target
        state = JointState()
        state.header.stamp.sec = len(moves)
        return state

    settings = {
        **config(), 'visual_servo_enabled': True,
        'visual_approach_step_m': .005,
        'visual_approach_segment_m': .02,
        'visual_approach_max_steps': 5,
        'visual_servo_maximum_correction_m': .03,
        'visual_servo_hard_correction_limit_m': .06,
        'visual_observation_confirmation_frames': 3,
        'visual_observation_consistency_m': .005,
        'visual_observation_increment_limit_m': .01,
        'visual_grasp_capture_depth_m': .025,
        'visual_grasp_capture_lateral_tolerance_m': .005,
        'cartesian_tolerance_m': .001,
    }
    node = NS(
        config=settings,
        cartesian_pose=CollectionPipeline.cartesian_pose,
        pose_message=CollectionPipeline.pose_message,
        pose=lambda _arm: deepcopy(current),
        linear=linear,
        observe=lambda _reference, _after: next(captures),
        real=NS(monitor=lambda: deepcopy(measured)),
        event=lambda *_args, **_kwargs: None,
    )
    selected = NS(
        selected_arm='left', reference_id='reference', fk_grasp_pose=target,
        grasp_joint_state=JointState(
            name=['left_wrist_roll_joint'], position=[0.]),
    )
    report = {}

    CollectionPipeline.visual_approach(
        node, selected, observation(1), observation(1), report)

    assert [stage for stage, _ in moves] == ['visual_approach_step']
    assert [position for _, position in moves] == pytest.approx([-.02])
    assert report['visual_approach'][
        'finalized_from_grasp_capture_corridor'] is True
    assert report['visual_approach']['observations'][-1][
        'inside_grasp_capture_corridor'] is True


def test_confirmed_centroid_jump_far_from_jaws_still_stops():
    from cleany_skill_executor.collection_pipeline import CollectionPipeline
    from geometry_msgs.msg import Pose
    from sensor_msgs.msg import JointState

    def observation(stamp, x):
        return NS(
            header=NS(stamp=NS(sec=stamp, nanosec=0), frame_id='base_link'),
            observed_center=NS(x=x, y=0., z=0.),
            observed_extent=NS(x=.05, y=.04, z=.06),
            mask_pixels=1000, valid_depth_points=990,
            valid_depth_fraction=.99,
        )

    current = Pose()
    current.orientation.w = 1.
    target = Pose()
    target.position.y = -.08
    target.orientation.w = 1.
    outliers = iter([observation(2, .04), observation(3, .04)])
    settings = {
        **config(), 'visual_servo_enabled': True,
        'visual_approach_step_m': .005,
        'visual_approach_segment_m': .02,
        'visual_approach_max_steps': 5,
        'visual_servo_maximum_correction_m': .03,
        'visual_servo_hard_correction_limit_m': .06,
        'visual_observation_confirmation_frames': 3,
        'visual_observation_consistency_m': .005,
        'visual_observation_increment_limit_m': .01,
        'visual_grasp_capture_depth_m': .045,
        'visual_grasp_capture_lateral_tolerance_m': .005,
        'cartesian_tolerance_m': .001,
    }
    node = NS(
        config=settings,
        cartesian_pose=CollectionPipeline.cartesian_pose,
        pose=lambda _arm: deepcopy(current),
        observe=lambda _reference, _after: next(outliers),
        real=NS(monitor=lambda: pytest.fail('motion must not start')),
        event=lambda *_args, **_kwargs: None,
    )
    selected = NS(
        selected_arm='left', reference_id='reference', fk_grasp_pose=target,
        grasp_joint_state=JointState(
            name=['left_wrist_roll_joint'], position=[0.]),
    )

    with pytest.raises(RuntimeError, match='outside the visual correction'):
        CollectionPipeline.visual_approach(
            node, selected, observation(1, 0.), observation(1, .04), {})


def test_wrist_confirmation_bridges_head_occlusion_to_last_safe_3d_target():
    from cleany_skill_executor.collection_pipeline import CollectionPipeline
    from geometry_msgs.msg import Pose
    from sensor_msgs.msg import JointState

    def observation(stamp, x):
        return NS(
            header=NS(stamp=NS(sec=stamp, nanosec=0), frame_id='base_link'),
            observed_center=NS(x=x, y=0., z=0.),
            observed_extent=NS(x=.05, y=.04, z=.06),
            mask_pixels=1000, valid_depth_points=990,
            valid_depth_fraction=.99,
        )

    current = Pose()
    current.orientation.w = 1.
    target = Pose()
    target.position.y = -.04
    target.orientation.w = 1.
    captures = iter([
        observation(2, .04), observation(3, .04),
        observation(4, 0.), observation(5, 0.),
    ])
    measured = JointState(name=['left_wrist_roll_joint'], position=[0.])
    moves = []
    wrist_checks = []

    def linear(_arm, waypoint, stage, wrist_roll_target=None, **_kwargs):
        current.position = deepcopy(waypoint.position)
        measured.position[0] = wrist_roll_target
        moves.append((stage, current.position.y))
        state = JointState()
        state.header.stamp.sec = len(moves)
        return state

    settings = {
        **config(), 'visual_servo_enabled': True,
        'visual_approach_step_m': .005,
        'visual_approach_segment_m': .02,
        'visual_approach_max_steps': 5,
        'visual_servo_maximum_correction_m': .03,
        'visual_servo_hard_correction_limit_m': .06,
        'visual_observation_confirmation_frames': 3,
        'visual_observation_consistency_m': .005,
        'visual_observation_increment_limit_m': .01,
        'visual_grasp_capture_depth_m': .005,
        'visual_grasp_capture_lateral_tolerance_m': .005,
        'visual_wrist_occlusion_fallback_enabled': True,
        'visual_wrist_occlusion_max_steps': 1,
        'cartesian_tolerance_m': .001,
    }
    node = NS(
        config=settings,
        cartesian_pose=CollectionPipeline.cartesian_pose,
        pose_message=CollectionPipeline.pose_message,
        pose=lambda _arm: deepcopy(current),
        linear=linear,
        observe=lambda _reference, _after: next(captures),
        real=NS(monitor=lambda: deepcopy(measured)),
        confirm_wrist_target=lambda report, step: (
            wrist_checks.append(step) or {'stamp_ns': 5_000_000_000}),
        event=lambda *_args, **_kwargs: None,
    )
    selected = NS(
        selected_arm='left', reference_id='reference', fk_grasp_pose=target,
        grasp_joint_state=JointState(
            name=['left_wrist_roll_joint'], position=[0.]),
    )
    report = {}

    CollectionPipeline.visual_approach(
        node, selected, observation(1, 0.), observation(1, .04), report)

    assert wrist_checks == [0]
    assert [position for _, position in moves] == pytest.approx([-.02, -.04])
    assert report['visual_approach']['wrist_occlusion_fallbacks'] == 1
    rejected = report['visual_approach']['observations'][0]
    assert rejected['wrist_occlusion_fallback'] is True
    assert rejected['wrist_confirmation']['stamp_ns'] == 5_000_000_000


def test_collection_wrist_handoff_follows_explicit_profile_switch():
    from cleany_skill_executor.collection_pipeline import CollectionPipeline

    node = NS(config={
        'operation_mode': 'grasp_lift_return_hold',
        'wrist_observation_enabled': False,
    })

    assert not CollectionPipeline.wrist_observation_required(node)
    node.config['wrist_observation_enabled'] = True
    assert CollectionPipeline.wrist_observation_required(node)


@pytest.mark.parametrize('mode, wrist, required', [
    ('pick_return_release', False, False),
    ('pick_return_release', True, True),
    ('full_collection', False, True),
    ('grasp_lift_return_hold', False, True),
])
def test_only_temporary_return_can_keep_motor_contact_without_wrist_identity(mode, wrist, required):
    from cleany_skill_executor.core.collection import (
        contact_verification_required, grasp_identity_verification_required,
    )
    settings = dict(operation_mode=mode, temporary_contact_verification='raw_feedback',
                    wrist_observation_enabled=wrist)
    assert contact_verification_required(settings)
    assert grasp_identity_verification_required(settings) is required


@pytest.mark.parametrize('wrist', [False, True])
def test_raw_contact_close_only_requests_wrist_evidence_when_enabled(wrist):
    from cleany_skill_executor.collection_pipeline import CollectionPipeline

    calls = []
    decision = NS(minimum_residual_ticks=25, median_current_raw=10, minimum_load_raw=10,
                  position_span_ticks=1, window_sec=.4)
    node = NS(config=dict(operation_mode='pick_return_release',
                          temporary_contact_verification='raw_feedback',
                          wrist_observation_enabled=wrist),
              get_clock=lambda: NS(now=lambda: NS(nanoseconds=100)),
              real=NS(verify=lambda _: {}, closed_contact_position=lambda *_: -.3,
                      wait_for_contact=lambda *_: calls.append('contact') or decision),
              jaw=lambda *_, **__: calls.append('close'),
              confirm_wrist_target=lambda *_: calls.append('wrist'),
              verify_evidence=lambda *_: calls.append('identity'), event=lambda *_, **__: None)
    report = {}
    CollectionPipeline.close_until_contact(node, NS(selected_arm='right'), report)
    assert calls == ['close', 'contact'] + (['wrist', 'identity'] if wrist else [])
    assert report['grasp_contact_confirmed'] and not report['grasp_assumed']
    assert report['grasp_identity_verified'] is wrist


@pytest.mark.parametrize('arm', ['left', 'right'])
@pytest.mark.parametrize('collision_checks', [None, True, False])
@pytest.mark.parametrize('opening_collision_checks', [None, True, False])
def test_gripper_collision_policies_independently_control_close_and_release(
        monkeypatch, arm, collision_checks, opening_collision_checks):
    from sensor_msgs.msg import JointState
    from cleany_skill_executor.collection_pipeline import CollectionPipeline
    from cleany_skill_executor.core.hardware_execution import CollisionValidationError
    import cleany_skill_executor.hardware_execution as transport

    joint = f'{arm}_gripper_joint'
    state = JointState(name=[joint], position=[.2])
    limit = PhysicalJointLimit(-2., 2., 1., 2., 2048., 1., 0., 0., 4095.)
    values = {
        'hardware_velocity_rad_s': .8, 'hardware_acceleration_rad_s2': 1.2,
        'hardware_path_step_rad': .01, 'hardware_path_simplification_tolerance_rad': .002,
        'hardware_start_tolerance_rad': .005, 'hardware_arrival_tolerance_rad': .07,
        'hardware_start_blend_max_rad': .03, 'hardware_uncommanded_tolerance_rad': .03,
        'hardware_settle_time_sec': .5,
    }
    checks, sent, contacts, logs = [], [], [], []
    monkeypatch.setattr(transport.time, 'sleep', lambda _: None)
    monkeypatch.setattr(transport, 'ActionClient', lambda *_: NS(destroy=lambda: None))

    def validity(_client, request):
        checks.append(request.robot_state.joint_state)
        return NS(valid=False, contacts=[])

    def run_action(_client, goal, _timeout, monitor):
        assert monitor
        sent.append(goal.trajectory)
        state.position = list(goal.trajectory.points[-1].positions)
        return NS(error_code=0, SUCCESSFUL=0)

    settings = dict(operation_mode='pick_return_release',
                    temporary_contact_verification='raw_feedback', wrist_observation_enabled=False)
    if collision_checks is not None:
        settings['grasp_collision_check_enabled'] = collision_checks
    if opening_collision_checks is not None:
        settings['gripper_opening_collision_check_enabled'] = opening_collision_checks
    node = NS(config=settings, call=validity, run_action=run_action,
              arm_config=NS(opened=.2),
              get_clock=lambda: NS(now=lambda: NS(nanoseconds=100)),
              get_logger=lambda: NS(info=logs.append, warning=logs.append, error=logs.append),
              event=lambda *_, **__: None)
    real = object.__new__(transport.HardwareExecution)
    real.node, node.real = node, real
    real.arm_holds, real.last_execution_metrics, real.uncommanded = {}, {}, None
    real.validity = object()
    real._sensor_scene_freeze_depth = 0
    real.verify = lambda _: {joint: limit}
    real.monitor = lambda: deepcopy(state)
    real.value = values.__getitem__
    real.positive_integer = lambda _: 1
    real.closed_contact_position = lambda *_: -.3
    decision = NS(minimum_residual_ticks=25, median_current_raw=10, minimum_load_raw=10,
                  position_span_ticks=1, window_sec=.4)
    real.wait_for_contact = lambda *_: contacts.append(True) or decision
    node.jaw = lambda *args, **options: CollectionPipeline.jaw(node, *args, **options)
    node.execute_trajectory = real.execute
    report = {}
    if collision_checks is not False:
        with pytest.raises(CollisionValidationError):
            CollectionPipeline.close_until_contact(node, NS(selected_arm=arm), report)
        assert checks and not sent and not contacts
        return

    CollectionPipeline.close_until_contact(node, NS(selected_arm=arm), report)
    assert not checks and len(sent) == 1 and contacts == [True]
    assert sent[0].joint_names == [joint] and tuple(sent[0].points[-1].positions) == (-.3,)
    assert report['grasp_contact_confirmed'] and not report['grasp_assumed']
    assert real.last_execution_metrics['collision_checks_enabled'] is False
    assert any(json.loads(log).get('stage') == 'hardware_collision_validation_skipped'
               and json.loads(log).get('controller') == 'gripper' for log in logs)
    if opening_collision_checks is not False:
        with pytest.raises(CollisionValidationError):
            node.jaw(arm, .2, 'release', check_collisions=True)
        assert checks and len(sent) == 1
    else:
        node.jaw(arm, .2, 'release', check_collisions=False)
        assert not checks and len(sent) == 2
        assert sent[-1].joint_names == [joint]
        assert tuple(sent[-1].points[-1].positions) == (.2,)
        assert real.last_execution_metrics['collision_checks_enabled'] is False


def test_temporary_motor_contact_preflight_needs_no_wrist_evidence_provider():
    from cleany_skill_executor.collection_pipeline import CollectionPipeline

    settings = config()
    settings.update(operation_mode='pick_return_release', temporary_contact_verification='raw_feedback',
                    temporary_placement_verification='assume_success',
                    label_destinations={'*': 'return_to_pick'}, wrist_observation_enabled=False)
    limits = {name: NS(check=lambda _: None)
              for name in (*ARM_JOINT_NAMES['left'], 'left_gripper_joint')}
    contact_setup = []
    node = NS(config=settings, arm_config=None, destination='',
              real=NS(verify=lambda _: limits, closed_contact_position=lambda *_: contact_setup.append(True)),
              evidence=NS(wait_for_service=lambda **_: pytest.fail('Unexpected wrist evidence dependency')),
              event=lambda *_, **__: None)
    selected = NS(selected_arm='left', selected_candidate=NS(required_opening_m=.04),
                  gripper_joint_position=.4)
    CollectionPipeline.before_execution(node, selected)
    assert contact_setup == [True] and node.destination == 'return_to_pick'
    node.config['operation_mode'] = 'full_collection'
    with pytest.raises(RuntimeError, match='requires wrist'):
        CollectionPipeline.before_execution(node, selected)


def test_disabled_wrist_cannot_reuse_head_prior_through_occlusion_fallback():
    from cleany_interfaces.srv import ObserveObjectReference
    from cleany_skill_executor.collection_pipeline import CollectionPipeline
    from cleany_skill_executor.pregrasp_pipeline import ReferenceObservationFailure

    response = ObserveObjectReference.Response(error_code=ObserveObjectReference.Response.ERROR_MASK,
                                               reference_id='head-reference', message='target mask lost')
    def observe(*_):
        raise ReferenceObservationFailure(response)
    node = NS(config={'wrist_observation_enabled': False,
                      'visual_wrist_occlusion_fallback_enabled': True},
              observe=observe, event=lambda *_, **__: pytest.fail('Unexpected wrist fallback'))
    with pytest.raises(ReferenceObservationFailure, match='target mask lost'):
        CollectionPipeline.observe_for_approach(node, 'head-reference', 100)


def test_cartesian_retarget_preserves_measured_roll_and_reaches_validated_roll():
    from cleany_skill_executor.collection_pipeline import CollectionPipeline
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    names = list(ARM_JOINT_NAMES['left'])
    trajectory = JointTrajectory(joint_names=names, points=[
        JointTrajectoryPoint(positions=[0., 0., 0., 0., -1.7]),
        JointTrajectoryPoint(positions=[.1, .2, .3, .4, -1.7]),
        JointTrajectoryPoint(positions=[.2, .4, .6, .8, -1.7]),
    ])
    measured = JointState(name=names, position=[0., 0., 0., 0., 2.7])

    CollectionPipeline.retarget_wrist_roll(
        trajectory, measured, 'left', 2.8)

    assert [point.positions[-1] for point in trajectory.points] == pytest.approx(
        [2.7, 2.75, 2.8])


@pytest.mark.parametrize('second_stamp,success',[(20,True),(10,False)])
def test_empty_completion_requires_two_distinct_captures(second_stamp,success):
    from cleany_skill_executor.collection_pipeline import CollectionPipeline
    cycles=iter([{'success':False,'error_code':2,'detection_count':0,'capture_stamp_ns':stamp} for stamp in (10,second_stamp)])
    node=NS(config=config(),run=lambda _:next(cycles),events=[],carrying=None,
            real=NS(monitor=lambda:None,fail_stop=lambda:{'requested':True}))
    report=CollectionPipeline.run_collection(node,'')
    assert report['success'] is success


def test_return_hold_mode_stops_after_one_successful_cycle():
    from cleany_skill_executor.collection_pipeline import CollectionPipeline
    node=NS(config={**config(),'operation_mode':'grasp_lift_return_hold'},
            run=lambda _:{'success':True,'payload_held':True},events=[],carrying={'object_id':'held'},
            real=NS(fail_stop=lambda:pytest.fail('stop must not be requested')))
    report=CollectionPipeline.run_collection(node,'')
    assert report['success'] and len(report['cycles'])==1


def test_unresolved_detection_is_never_empty_or_retried_as_success():
    from cleany_skill_executor.collection_pipeline import CollectionPipeline
    node=NS(config=config(),run=lambda _:{'success':False,'error_code':2,'detection_count':1,'message':'unresolved'},
            events=[],carrying=None,real=NS(fail_stop=lambda:{'requested':True}))
    report=CollectionPipeline.run_collection(node,'')
    assert not report['success'] and report['empty_confirmations']==0


def test_guard_fault_latches_even_after_ready_and_rejects_stale_status(monkeypatch):
    import cleany_skill_executor.hardware_execution as module
    fake=NS(guard=None,guard_time=0.,fault='',uncommanded=None,
            value=lambda _: .25,node=NS(measured=lambda:NS(name=['head_pan_joint','head_tilt_joint'],position=[0.,0.])))
    monkeypatch.setattr(module.time,'monotonic',lambda:10.)
    module.HardwareExecution.on_guard(fake,NS(data='{"state":"fault","reason":"motor fault"}'))
    module.HardwareExecution.on_guard(fake,NS(data='{"state":"ready"}'))
    with pytest.raises(RuntimeError,match='motor fault'):module.HardwareExecution.monitor(fake)
    fake.fault='';fake.guard_time=9.
    with pytest.raises(RuntimeError,match='stale'):module.HardwareExecution.monitor(fake)


@pytest.mark.parametrize('enabled', [None, True, False])
def test_uncommanded_motion_switch_is_loaded_from_ros_parameters(enabled):
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from cleany_skill_executor.hardware_execution import HardwareExecution
    context = Context()
    rclpy.init(context=context)
    node = Node('uncommanded_motion_test', context=context, parameter_overrides=(
        [] if enabled is None else [Parameter('hardware_uncommanded_motion_check_enabled', value=enabled)]))
    try:
        execution = HardwareExecution(node)
        assert execution.uncommanded_motion_check_enabled is (True if enabled is None else enabled)
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)


@pytest.mark.parametrize('enabled', [True, False])
@pytest.mark.parametrize('delta', [.02, .03, .04])
def test_uncommanded_motion_switch_controls_abort_and_keeps_diagnostics(enabled, delta):
    import time
    from cleany_skill_executor.hardware_execution import HardwareExecution
    state = NS(name=['head_pan_joint', 'head_tilt_joint', 'left_wrist_pitch_joint'],
               position=[.02, 0., delta])
    values = {'hardware_guard_max_age_sec': .25, 'hardware_uncommanded_tolerance_rad': .03}
    fake = NS(fault='', guard={'state': 'ready'}, guard_time=time.monotonic(),
              uncommanded_motion_check_enabled=enabled,
              uncommanded={'head_pan_joint': 0., 'left_wrist_pitch_joint': 0.},
              value=values.__getitem__, node=NS(measured=lambda: state))
    if enabled and delta > .03:
        with pytest.raises(RuntimeError, match=r'left_wrist_pitch_joint delta=\+0.040000rad; tolerance=0.030000rad'):
            HardwareExecution.monitor(fake)
    else:
        assert HardwareExecution.monitor(fake) is state


@pytest.mark.parametrize('failure', ['fault', 'stale_guard', 'stale_feedback', 'motion', 'payload'])
def test_disabling_uncommanded_motion_check_keeps_other_runtime_failures(failure):
    import time
    from cleany_skill_executor.hardware_execution import HardwareExecution
    state = NS(name=['head_pan_joint', 'head_tilt_joint', 'left_gripper_joint'],
               position=[0., 0., .021234])
    node = NS(measured=lambda: state, get_clock=lambda: NS(
        now=lambda: NS(nanoseconds=10_000_000_000)))
    fake = NS(fault='', guard={'state': 'ready'}, guard_time=time.monotonic(),
              complete_feedback_stamp_ns=10_000_000_000,
              uncommanded_motion_check_enabled=False,
              uncommanded={'left_gripper_joint': .054982}, value=lambda _: .25, node=node)
    def fail():
        raise RuntimeError(failure)
    expected = failure
    if failure == 'fault':
        fake.fault = failure
    elif failure == 'stale_guard':
        fake.guard_time -= 1.
        expected = 'Physical guard unavailable/stale/not ready'
    elif failure == 'stale_feedback':
        fake.complete_feedback_stamp_ns -= 1_000_000_000
        expected = 'Physical complete-feedback acquisition stale'
    elif failure == 'motion':
        fake._motion_monitors = [NS(check=lambda _: fail())]
    else:
        node.monitor_payload = fail
    with pytest.raises(RuntimeError, match=expected):
        HardwareExecution.monitor(fake)


def test_operational_profile_allows_recorded_held_gripper_closure():
    import time
    import yaml
    from cleany_skill_executor.hardware_execution import HardwareExecution
    root = Path(__file__).resolve().parents[4]
    parameters = yaml.safe_load((root/'ros2_ws/src/cleany_skill_executor/test/data/hardware_motion_monitor_fixture.yaml').read_text())[
        'pregrasp_pipeline']['ros__parameters']
    state = NS(name=['head_pan_joint', 'head_tilt_joint', 'left_gripper_joint'],
               position=[0., 0., .021234])
    fake = NS(fault='', guard={'state': 'ready'}, guard_time=time.monotonic(),
              uncommanded_motion_check_enabled=parameters['hardware_uncommanded_motion_check_enabled'],
              uncommanded={'left_gripper_joint': .054982}, value=lambda _: .25,
              node=NS(measured=lambda: state))
    assert HardwareExecution.monitor(fake) is state


def test_broadcaster_replay_cannot_refresh_complete_hardware_feedback():
    import time
    from cleany_skill_executor.hardware_execution import HardwareExecution
    fake = NS(fault='', guard={'state': 'ready'}, guard_time=time.monotonic(),
              complete_feedback_stamp_ns=10_000_000_000, value=lambda _: .25,
              node=NS(measured=lambda: object(), get_clock=lambda: NS(
                  now=lambda: NS(nanoseconds=10_300_000_000))))
    with pytest.raises(RuntimeError, match='acquisition stale'):
        HardwareExecution.monitor(fake)


def test_path_validation_drift_uses_separate_commanded_and_other_tolerances():
    from cleany_skill_executor.hardware_execution import path_validation_drift
    state=NS(name=['arm','gripper','head'],position=[.006,.02,.031])
    drift=path_validation_drift({'arm':0.,'gripper':0.,'head':0.},state,('arm',),.005,.03)
    assert drift == {'arm':.006,'head':.031}


@pytest.mark.parametrize('action_error',[False, True, 'scene_deferred', 'scene_legacy'])
@pytest.mark.parametrize('start_drift', [0., .012])
@pytest.mark.parametrize('arrival_error', [0., .025])
@pytest.mark.parametrize('allow_endpoint_error', [False, True])
def test_physical_transport_sends_bounded_goal_and_stops_on_failure(
        monkeypatch, action_error, start_drift, arrival_error, allow_endpoint_error):
    import cleany_skill_executor.hardware_execution as module
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
    names=ARM_JOINT_NAMES['left']
    state=JointState(name=list(names)+['left_gripper_joint','head_pan_joint','head_tilt_joint'],position=[0.]*8)
    state.position[0] = start_drift
    limits={n:PhysicalJointLimit(-2.,2.,.04,.15,2048.,1.,0.,0.,4095.) for n in names}
    sent=[];stops=[];destroyed=[]
    monkeypatch.setattr(module,'ActionClient',lambda *args:NS(destroy=lambda:destroyed.append(True)))
    monkeypatch.setattr(module.time,'sleep',lambda _:None)
    values={'hardware_velocity_rad_s':.025,'hardware_acceleration_rad_s2':.08,
            'hardware_path_step_rad':.01,
            'hardware_path_simplification_tolerance_rad':.002,
            'hardware_start_delay_sec':.8,
            'hardware_start_tolerance_rad':.005,'hardware_start_blend_max_rad':.03,
            'hardware_arrival_tolerance_rad':.015,
            'hardware_uncommanded_tolerance_rad':.03,'hardware_settle_time_sec':.5}
    def run_action(client,goal,timeout,monitor):
        sent.append(goal)
        assert monitor and timeout>10
        if action_error in ('scene_deferred', 'scene_legacy'):
            from cleany_skill_executor.core.physical_grasp_selection import InfrastructureError
            raise InfrastructureError('Sensor planning scene unavailable')
        if action_error:raise RuntimeError('controller aborted')
        for i,n in enumerate(goal.trajectory.joint_names):
            state.position[state.name.index(n)]=goal.trajectory.points[-1].positions[i]
        state.position[0] -= arrival_error
        return NS(error_code=0,SUCCESSFUL=0)
    node=NS(call=lambda *_:NS(valid=True),run_action=run_action,
            get_clock=lambda:NS(now=lambda:NS(nanoseconds=1_000_000_000)),
            get_logger=lambda:NS(info=lambda _:None, warning=lambda _:None))
    if action_error == 'scene_deferred':
        node.defer_stop_assessment = lambda _: True
    endpoint = {'final_tcp_position_error_m': 0.,
                'final_tcp_orientation_error_deg': 0.,
                'final_arm_joint_error_rad': [0.]*5}
    fake=NS(node=node,verify=lambda _:limits,monitor=lambda:deepcopy(state),value=values.__getitem__,
            validity=object(),fail_stop=lambda:stops.append(True),uncommanded=None,
            last_execution_metrics={}, arm_holds={},
            verify_arm_endpoint=lambda _arm, _target, _limits, actual,
            **_kwargs: (actual, endpoint))
    trajectory=JointTrajectory(joint_names=list(names),points=[
        JointTrajectoryPoint(positions=[0.]*5),JointTrajectoryPoint(positions=[.1,0.,0.,0.,0.])])
    if action_error or (arrival_error > .015 and not allow_endpoint_error):
        message = ('Sensor planning scene unavailable' if isinstance(action_error, str) else
                   'controller aborted' if action_error else 'Physical arrival not confirmed')
        with pytest.raises(RuntimeError,match=message):
            module.HardwareExecution._execute(fake,'left',trajectory,'arm',
                                              allow_endpoint_error=allow_endpoint_error)
        assert stops == ([] if action_error == 'scene_deferred' else [True])
    else:
        module.HardwareExecution._execute(fake,'left',trajectory,'arm',
                                          allow_endpoint_error=allow_endpoint_error)
        assert stops==[]
        assert fake.last_execution_metrics['joint_arrival_within_tolerance'] is (arrival_error <= .015)
    assert len(sent)==1 and destroyed==[True] and fake.uncommanded is None
    output=sent[0].trajectory
    assert tuple(output.joint_names)==names
    assert output.points[0].positions[0] == start_drift
    assert all(not p.accelerations for p in output.points)
    assert max(abs(v) for p in output.points for v in p.velocities)<=.025
    assert [(tol.name, tol.position) for tol in sent[0].goal_tolerance] == (
        [(name, -1.) for name in names] if allow_endpoint_error else [])
    assert output.header.stamp.sec == output.header.stamp.nanosec == 0
    assert output.points[0].time_from_start.sec == output.points[0].time_from_start.nanosec == 0


def test_endpoint_transport_keeps_held_setpoint_and_validates_measured_bridge(monkeypatch):
    import cleany_skill_executor.hardware_execution as module
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    names = ARM_JOINT_NAMES['left']
    state = JointState(
        name=list(names)+['left_gripper_joint', 'head_pan_joint', 'head_tilt_joint'],
        position=[.08, 0., 0., 0., 0., 0., 0., 0.],
    )
    limits = {
        name: PhysicalJointLimit(-2., 2., 1., 2., 2048., 1., 0., 0., 4095.)
        for name in names
    }
    held = (.1, 0., 0., 0., 0.)
    corrected = (.114, 0., 0., 0., 0.)
    sent = []
    validated = []
    monkeypatch.setattr(
        module, 'ActionClient',
        lambda *_: NS(destroy=lambda: None),
    )
    monkeypatch.setattr(module.time, 'sleep', lambda _: None)
    values = {
        'hardware_velocity_rad_s': .8,
        'hardware_acceleration_rad_s2': 1.2,
        'hardware_path_step_rad': .01,
        'hardware_path_simplification_tolerance_rad': .002,
        'hardware_start_delay_sec': .8,
        'hardware_start_tolerance_rad': .005,
        'hardware_arrival_tolerance_rad': .03,
        'hardware_uncommanded_tolerance_rad': .03,
        'hardware_settle_time_sec': .5,
    }

    def call(_client, request):
        positions = dict(zip(
            request.robot_state.joint_state.name,
            request.robot_state.joint_state.position,
        ))
        validated.append(tuple(positions[name] for name in names))
        return NS(valid=True)

    def run_action(_client, goal, _timeout, monitor):
        assert monitor
        sent.append(goal.trajectory)
        for index, name in enumerate(goal.trajectory.joint_names):
            state.position[state.name.index(name)] = goal.trajectory.points[-1].positions[index]
        return NS(error_code=0, SUCCESSFUL=0)

    node = NS(
        call=call,
        run_action=run_action,
        get_clock=lambda: NS(now=lambda: NS(nanoseconds=1_000_000_000)),
        get_logger=lambda: NS(info=lambda _: None),
    )
    fake = NS(
        node=node,
        verify=lambda _: limits,
        monitor=lambda: deepcopy(state),
        value=values.__getitem__,
        validity=object(),
        fail_stop=lambda: pytest.fail('unexpected stop'),
        uncommanded=None,
        last_execution_metrics={},
        arm_holds={},
    )
    trajectory = JointTrajectory(joint_names=list(names), points=[
        JointTrajectoryPoint(positions=list(held)),
        JointTrajectoryPoint(positions=list(corrected)),
    ])

    module.HardwareExecution._execute(
        fake, 'left', trajectory, 'arm', terminal_check=False,
        held_command=held,
    )

    assert len(sent) == 1
    assert sent[0].header.stamp.sec == sent[0].header.stamp.nanosec == 0
    assert tuple(sent[0].points[0].positions) == pytest.approx(held)
    assert tuple(sent[0].points[0].positions) != pytest.approx((.08, 0., 0., 0., 0.))
    assert tuple(sent[0].points[-1].positions) == pytest.approx(corrected)
    assert validated[0] == pytest.approx((.08, 0., 0., 0., 0.))
    assert any(row == pytest.approx(held) for row in validated)


@pytest.mark.parametrize('arm_side', ['left', 'right'])
@pytest.mark.parametrize('collision', [False, True, 'interior', 'bridge', 'limits'])
@pytest.mark.parametrize('check_collisions', [True, False])
@pytest.mark.parametrize('entry_bend', [False, True])
@pytest.mark.parametrize('closing,stuck', [(False, False), (True, False), (True, True), (True, 'late')])
@pytest.mark.parametrize('sparse', [False, True])
def test_physical_pregrasp_dispatches_arm_and_gripper_before_waiting_for_results(
        monkeypatch, collision, arm_side, closing, stuck, entry_bend, check_collisions, sparse):
    import cleany_skill_executor.hardware_execution as module
    from action_msgs.msg import GoalStatus
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    arm_names = ARM_JOINT_NAMES[arm_side]
    gripper_name = f'{arm_side}_gripper_joint'
    names = [*arm_names, gripper_name, 'head_pan_joint', 'head_tilt_joint']
    state = JointState(name=names, position=[0.] * len(names))
    if closing:
        state.position[state.name.index(gripper_name)] = .4
    limits = {name: PhysicalJointLimit(-2., 2., 1., 2., 2048., 1., 0., 0., 4095.)
              for name in (*arm_names, gripper_name)}
    sent = []
    results_requested = []
    destroyed = []
    stopped = []

    class Immediate:
        def __init__(self, value): self._value = value
        def done(self): return True
        def result(self): return self._value

    class Handle:
        accepted = True
        def __init__(self, controller, goal):
            self.controller, self.goal = controller, goal
        def get_result_async(self):
            assert len(sent) == 2
            results_requested.append(self.controller)
            for index, name in enumerate(self.goal.trajectory.joint_names):
                if not (stuck is True and self.controller == 'gripper'):
                    state.position[state.name.index(name)] = self.goal.trajectory.points[-1].positions[index]
            result = NS(error_code=0, SUCCESSFUL=0, error_string='')
            return Immediate(NS(status=GoalStatus.STATUS_SUCCEEDED, result=result))
        def cancel_goal_async(self): return Immediate(NS())

    class Client:
        def __init__(self, _node, _action, path): self.controller = 'gripper' if 'gripper' in path else 'arm'
        def wait_for_server(self, **_): return True
        def send_goal_async(self, goal, feedback_callback=None):
            sent.append((self.controller, goal))
            return Immediate(Handle(self.controller, goal))
        def destroy(self): destroyed.append(self.controller)

    monkeypatch.setattr(module, 'ActionClient', Client)
    monkeypatch.setattr(module.time, 'sleep', lambda _: None)
    prepared_paths = []
    prepare = module.prepare_bounded_path
    def capture_path(*args, **options):
        path = prepare(*args, **options)
        prepared_paths.append(path)
        return path
    monkeypatch.setattr(module, 'prepare_bounded_path', capture_path)
    values = {'hardware_velocity_rad_s': .8, 'hardware_acceleration_rad_s2': 1.2,
              'hardware_path_step_rad': .01,
              'hardware_path_simplification_tolerance_rad': .002,
              'hardware_start_delay_sec': .8, 'hardware_return_start_delay_sec': .3,
              'hardware_start_tolerance_rad': .005, 'hardware_start_blend_max_rad': .03,
              'hardware_arrival_tolerance_rad': .03,
              'hardware_command_continuity_max_offset_rad': .06,
              'hardware_tcp_position_tolerance_m': .1,
              'hardware_tcp_orientation_tolerance_deg': 20.,
              'hardware_uncommanded_tolerance_rad': .03, 'hardware_settle_time_sec': .5}
    checked, diagnostics = [], []
    def validate(_client, request):
        q = tuple(request.robot_state.joint_state.position)
        checked.append(q)
        blocked = (collision is True or (collision == 'interior' and .02 < abs(q[0]) < .05)
                   or (collision == 'bridge' and q[0] < .01))
        return NS(valid=not blocked, contacts=[contact(body_2=f'{arm_side}_gripper_frame')]
                  if blocked else [])
    node = NS(call=validate, wait=lambda future, *_: future.result(),
              get_clock=lambda: NS(now=lambda: NS(nanoseconds=2_000_000_000 + (400_000_000 if stuck == 'late' and sent else 0))),
              get_logger=lambda: NS(error=lambda _: None, info=lambda _: None, warning=diagnostics.append),
              motion=NS(forward_kinematics=lambda _arm, solution: NS(
                  position=solution.positions[:3], orientation_xyzw=(0., 0., 0., 1.))))
    fake = NS(node=node, verify=lambda _: limits, monitor=lambda: deepcopy(state),
              value=values.__getitem__, validity=object(), fail_stop=lambda: stopped.append(True),
              uncommanded=None, last_execution_metrics={}, arm_holds={}, sensor_scene_frozen=False,
              sparse_cubic_commands=sparse)
    if collision == 'bridge':
        command = (.02, 0., 0., 0., 0.)
        fake.arm_holds[arm_side] = ArmHold(command, command)
    fake.endpoint_metrics = lambda arm_name, target, actual, **kwargs: (
        module.HardwareExecution.endpoint_metrics(
            fake, arm_name, target, actual, **kwargs))
    fake.wait_for_settle = lambda _: deepcopy(state)
    fake.verify_arm_endpoint = lambda *args: module.HardwareExecution.verify_arm_endpoint(fake, *args)
    endpoint = (-1.5, -.3, -1.3, .1, -1.) if entry_bend else (.1, .2, .1, .15, 0.)
    knots = [(0.,)*5]
    if entry_bend:
        knots.extend([(.0009, -.0082, -.0057, .0064, -.0021), (-1., -.6, -.9, .3, -.7)])
    knots.append(endpoint)
    arm = JointTrajectory(joint_names=list(arm_names), points=[
        JointTrajectoryPoint(positions=list(q)) for q in knots])
    jaw = JointTrajectory(joint_names=[gripper_name], points=[
        JointTrajectoryPoint(positions=[.4 if closing else 0.]),
        JointTrajectoryPoint(positions=[0. if closing else .4])])
    options = {'gripper_goal_required': True, 'stage_prefix': 'synchronized_return'} if closing else {}

    if not check_collisions:
        options['check_collisions'] = False
    if collision == 'limits':
        from cleany_skill_executor.core.hardware_execution import JointRangeValidationError
        arm.points[-1].positions[0] = 3.
        with pytest.raises(JointRangeValidationError):
            module.HardwareExecution._execute_pregrasp_parallel(
                fake, arm_side, arm, jaw, 0. if closing else .3, 1, **options)
        assert not sent
        return

    if collision and check_collisions:
        from cleany_skill_executor.core.hardware_execution import CollisionValidationError
        with pytest.raises(CollisionValidationError, match='collision validation') as rejected:
            module.HardwareExecution._execute_pregrasp_parallel(
                fake, arm_side, arm, jaw, 0. if closing else .3, 1, **options)
        assert checked and not sent
        assert not stopped
        error, details = rejected.value, json.loads(diagnostics[0])
        assert not error.motion_started and error.contacts and error.octomap_arm_contact
        assert error.phase == ('hold_bridge' if collision == 'bridge' else 'command_path')
        assert error.sample_index == len(checked)-1
        assert error.entry_blocked is (collision is True or collision == 'bridge')
        assert details['trajectory_stage'] == ('synchronized_return' if closing else 'synchronized_pregrasp')
        assert details['sample_joint_names'] == [*arm_names, gripper_name]
        assert details['full_joint_names'] == state.name
        assert details['full_joint_rad'] == list(checked[-1])
        assert details['contacts'] == list(error.contacts)
        return
    if stuck == 'late':
        with pytest.raises(RuntimeError, match='accepted before their start time'):
            module.HardwareExecution._execute_pregrasp_parallel(fake, arm_side, arm, jaw, 0., 1, **options)
        assert stopped and len(sent) == 2 and sorted(destroyed) == ['arm', 'gripper']
        return
    if stuck:
        with pytest.raises(RuntimeError, match='gripper goal not confirmed'):
            module.HardwareExecution._execute_pregrasp_parallel(fake, arm_side, arm, jaw, 0., 1, **options)
        assert stopped and len(sent) == 2
        return
    module.HardwareExecution._execute_pregrasp_parallel(
        fake, arm_side, arm, jaw, 0. if closing else .3, 1, **options)

    assert bool(checked) is check_collisions
    assert fake.last_execution_metrics['collision_checks_enabled'] is check_collisions
    assert [name for name, _ in sent] == ['arm', 'gripper']
    assert results_requested == ['arm', 'gripper']
    assert sorted(destroyed) == ['arm', 'gripper']
    arm_goal, gripper_goal = (goal.trajectory for _, goal in sent)
    assert len(arm_goal.points) == len(gripper_goal.points)
    assert arm_goal.joint_names == list(arm_names)
    assert sent[0][1].goal_tolerance == []
    assert sent[1][1].goal_tolerance[0].name == gripper_name
    if closing:
        assert sent[1][1].goal_tolerance[0].position == .03
    else:
        assert sent[1][1].goal_tolerance[0].position > .1
    assert [(p.time_from_start.sec, p.time_from_start.nanosec) for p in arm_goal.points] == [
        (p.time_from_start.sec, p.time_from_start.nanosec) for p in gripper_goal.points]
    assert arm_goal.header.stamp == gripper_goal.header.stamp
    assert arm_goal.header.stamp.sec == 2 and arm_goal.header.stamp.nanosec == 0
    assert arm_goal.points[0].positions == arm_goal.points[1].positions
    assert arm_goal.points[0].time_from_start.sec == 0
    assert arm_goal.points[0].time_from_start.nanosec == 0
    assert arm_goal.points[1].time_from_start.nanosec == (300_000_000 if closing else 800_000_000)
    assert tuple(arm_goal.points[-1].positions) == pytest.approx(endpoint)
    assert tuple(gripper_goal.points[-1].positions) == pytest.approx((0. if closing else .4,))
    assert fake.last_execution_metrics['gripper_goal_required'] == closing
    assert not stopped
    if check_collisions:
        assert any(q[:5] == pytest.approx(endpoint) for q in checked)
    assert fake.last_execution_metrics['maximum_final_arm_joint_error_rad'] == pytest.approx(0.)
    assert fake.last_execution_metrics['final_tcp_position_error_m'] == pytest.approx(0.)
    # Check the exact combined curve sent to both controllers, after local
    # retiming. No check against only the original planner knots can authorize it.
    combined = [tuple(a.positions+g.positions) for a, g in zip(
        arm_goal.points[1:], gripper_goal.points[1:], strict=True)]
    assert [q[:6] for q in checked] == ([sample.positions for sample in prepared_paths[0].samples] if check_collisions else [])
    if sparse:
        from cleany_skill_executor.core.recorded_trajectory import CommandPoint, RecordedTrajectory
        assert len(combined) < len(prepared_paths[0].samples)
        encoded = RecordedTrajectory(tuple(names[:6]), tuple(CommandPoint(
            a.time_from_start.sec*10**9+a.time_from_start.nanosec,
            tuple(a.positions+g.positions), tuple(a.velocities+g.velocities))
            for a, g in zip(arm_goal.points, gripper_goal.points, strict=True)))
        for sample in prepared_paths[0].samples:
            ns = (300_000_000 if closing else 800_000_000)+round(
                sample.progress*prepared_paths[0].duration*1e9)
            assert encoded.sample_at(ns).positions == pytest.approx(sample.positions, abs=1e-8)
    else:
        assert [q[:6] for q in checked] == (combined if check_collisions else [])
    if entry_bend:
        end = arm_goal.points[-1].time_from_start
        assert end.sec+end.nanosec/1e9-.8 < 8.
        for knot in (knots[1:] if collision == 'bridge' else knots):
            assert any(tuple(point.positions) == pytest.approx(knot) for point in arm_goal.points)
    assert fake.last_execution_metrics['target_arm_joint_rad'] == pytest.approx(endpoint)
    assert fake.last_execution_metrics['commanded_arm_joint_rad'] == pytest.approx(endpoint)
    assert fake.arm_holds[arm_side].nominal == endpoint
    assert fake.arm_holds[arm_side].command == tuple(arm_goal.points[-1].positions)
    assert set(fake.last_execution_metrics['uncommanded_joint_change_rad']) == {
        'head_pan_joint', 'head_tilt_joint'}
    assert fake.uncommanded is None


def test_abort_result_is_only_allowed_for_explicit_validation_call():
    from cleany_skill_executor.pregrasp_pipeline import PregraspPipeline
    wrapped=NS(status=6,result=NS(message='no eligible object'))
    handle=NS(accepted=True,get_result_async=lambda:wrapped,cancel_goal_async=lambda:NS())
    fake=NS(wait=lambda f,*a,**kw:f,
            get_logger=lambda:NS(error=lambda _:None))
    client=NS(wait_for_server=lambda **_:True,send_goal_async=lambda _:handle)
    assert PregraspPipeline.run_action(fake,client,object(),5.,allow_aborted=True) is wrapped.result
    with pytest.raises(RuntimeError,match='Action failed'):
        PregraspPipeline.run_action(fake,client,object(),5.)


@pytest.mark.parametrize('resume', [False, True])
def test_return_collision_bypass_reaches_dispatch_and_survives_communication_resume(resume):
    from cleany_skill_executor.hardware_execution import HardwareExecution
    from cleany_skill_executor.core.hardware_execution import CommunicationRecovered
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    names, calls = ARM_JOINT_NAMES['left'], []
    state = JointState(name=[*names, 'left_gripper_joint'], position=[.05, 0., 0., 0., 0., .2])
    def execute(*args, **kwargs):
        calls.append(kwargs)
        assert kwargs == dict(gripper_goal_required=True, stage_prefix='synchronized_return',
                              check_collisions=False)
        if resume and len(calls) == 1:
            interrupted = CommunicationRecovered(1)
            interrupted.cancellation_confirmed = True
            raise interrupted
        return state
    fake = NS(_execute_pregrasp_parallel=execute, positive_integer=lambda _: 1, value=lambda _: .03,
              wait_for_settle=lambda _: state, arm_holds={}, last_execution_metrics={},
              frozen_sensor_scene=lambda **_: pytest.fail('Collision-disabled return waited for scene'),
              node=NS(get_logger=lambda: NS(warning=lambda _: None)))
    fake.execute_pregrasp_parallel = lambda *args, **kwargs: HardwareExecution.execute_pregrasp_parallel(
        fake, *args, **kwargs)
    trajectory = JointTrajectory(joint_names=list(names), points=[
        JointTrajectoryPoint(positions=[0.] * 5),
        JointTrajectoryPoint(positions=[.1, 0., 0., 0., 0.])])
    jaw = JointTrajectory(joint_names=['left_gripper_joint'], points=[
        JointTrajectoryPoint(positions=[.4]), JointTrajectoryPoint(positions=[0.])])
    assert HardwareExecution.execute_arm_gripper(
        fake, 'left', trajectory, jaw, check_collisions=False) is state
    assert len(calls) == 1 + int(resume)


@pytest.mark.parametrize('check_collisions', [0, 1, None, 'false'])
def test_synchronized_execution_rejects_nonboolean_collision_policy(check_collisions):
    from cleany_skill_executor.hardware_execution import HardwareExecution

    with pytest.raises(ValueError, match='check_collisions must be boolean'):
        HardwareExecution.execute_pregrasp_parallel(
            NS(), 'left', None, None, 0., 1, check_collisions=check_collisions)
