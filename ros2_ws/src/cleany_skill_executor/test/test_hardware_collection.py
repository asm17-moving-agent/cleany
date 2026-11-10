from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as NS
import json
import pytest
from cleany_skill_executor.core.collection import configuration_issues, load_arm, check_evidence
from cleany_skill_executor.core.hardware_execution import PhysicalJointLimit
from cleany_skill_executor.core.physical_grasp_selection import ARM_JOINT_NAMES

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
    (key, value)
    for key, invalid_string in (
        ('pregrasp_motion_enabled', 'false'),
        ('direct_grasp_shortcut_enabled', 'true'),
        ('direct_grasp_pose_checks_enabled', 'false'),
        ('approach_collision_check_enabled', 'false'),
        ('grasp_collision_check_enabled', 'false'),
        ('gripper_opening_collision_check_enabled', 'false'),
        ('lift_collision_check_enabled', 'false'),
        ('retreat_after_release_collision_check_enabled', 'false'),
    )
    for value in (invalid_string, 0, None)
])
def test_motion_and_collision_policies_require_booleans(key, value):
    assert key+' must be boolean' in configuration_issues({**config(), key: value})


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
    limits={name:NS(check=lambda _:None) for name in (*ARM_JOINT_NAMES['left'],'left_gripper_joint')}
    node=NS(config={**config(),'label_destinations':{'*':'recycling'}},arm_config=None,
            destination='',real=NS(verify=lambda _:limits, closed_contact_position=lambda *_:0.),
            evidence=NS(wait_for_service=lambda **_:True),
            event=lambda *_,**__:None)
    CollectionPipeline.before_execution(node,selected)
    assert node.destination=='recycling'


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


def contact(body_1='<octomap>', body_2='left_gripper_frame', depth=.004):
    return NS(
        contact_body_1=body_1,
        contact_body_2=body_2,
        depth=depth,
        position=NS(x=.37, y=.09, z=.42),
    )


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


@pytest.mark.parametrize('segment,expected', [(None, [.005, .01, .012]), (.02, [.012])])
def test_visual_approach_reobserves_after_each_bounded_cartesian_segment(segment, expected):
    from cleany_skill_executor.collection_pipeline import CollectionPipeline

    node, selected, moves = visual_approach_case(
        [visual_observation(stamp) for stamp in (2, 3, 4)], target_y=-.012, capture_depth=0.)
    node.config['visual_grasp_capture_lateral_tolerance_m'] = 0.
    node.config.pop('visual_approach_segment_m')
    if segment is not None:
        node.config['visual_approach_segment_m'] = segment
    selected.grasp_joint_state.position[0] = .12
    node.real.value = lambda _: 2.
    controls, linear = [], node.linear
    def record(arm, waypoint, stage, wrist_roll_target=None, *, check_orientation=True):
        controls.append((wrist_roll_target, check_orientation))
        return linear(arm, waypoint, stage, wrist_roll_target)
    node.linear = record
    report = {}
    CollectionPipeline.visual_approach(
        node, selected, visual_observation(1), visual_observation(1), report)

    assert [-position for _, position in moves] == pytest.approx(expected)
    assert [roll for roll, _ in controls] == pytest.approx([x*10 for x in expected])
    assert [check for _, check in controls] == [False]*len(expected)
    assert report['visual_approach']['steps'] == len(expected)
    assert len(report['visual_approach']['observations']) == len(expected)+1


@pytest.mark.parametrize('capture_corridor', [False, True])
def test_visual_approach_finishes_without_motion_when_capture_ready(capture_corridor):
    from cleany_skill_executor.collection_pipeline import CollectionPipeline

    node, selected, moves = visual_approach_case(
        [], target_y=-.04 if capture_corridor else 0., capture_depth=.045)
    if not capture_corridor:
        selected.fk_grasp_pose.orientation.w = 0.
        selected.fk_grasp_pose.orientation.x = 1.
    node.real.monitor = lambda: pytest.fail('Capture-ready pose must not prepare a move')
    events = []
    node.event = lambda stage, **fields: events.append((stage, fields))
    report = {}
    CollectionPipeline.visual_approach(
        node, selected, visual_observation(1), visual_observation(1), report)

    summary = report['visual_approach']
    assert not moves and summary['steps'] == 0
    if capture_corridor:
        assert summary['finalized_from_grasp_capture_corridor']
        assert summary['observations'][0]['inside_grasp_capture_corridor']
        assert events[-1][0] == 'visual_capture_finalized'
        assert events[-1][1]['final_motion_skipped']
    else:
        assert summary['observations'][0]['tcp_orientation_remaining_deg'] == pytest.approx(180.)


def visual_observation(stamp, x=0.):
    return NS(header=NS(stamp=NS(sec=stamp, nanosec=0), frame_id='base_link'),
              observed_center=NS(x=x, y=0., z=0.),
              observed_extent=NS(x=.05, y=.04, z=.06),
              mask_pixels=1000, valid_depth_points=990, valid_depth_fraction=.99)


def visual_approach_case(captures, *, target_y=-.04, capture_depth=.005):
    from cleany_skill_executor.collection_pipeline import CollectionPipeline
    from geometry_msgs.msg import Pose
    from sensor_msgs.msg import JointState

    current, target = Pose(), Pose()
    current.orientation.w = target.orientation.w = 1.
    target.position.y = target_y
    measured = JointState(name=['left_wrist_roll_joint'], position=[0.])
    captures, moves = iter(captures), []

    def linear(_arm, waypoint, stage, wrist_roll_target=None, **_kwargs):
        current.position = deepcopy(waypoint.position)
        measured.position[0] = wrist_roll_target
        moves.append((stage, current.position.y))
        state = JointState()
        state.header.stamp.sec = len(moves)
        return state

    node = NS(config={
        **config(), 'visual_servo_enabled': True, 'visual_approach_step_m': .005,
        'visual_approach_segment_m': .02, 'visual_approach_max_steps': 5,
        'visual_servo_maximum_correction_m': .03, 'visual_servo_hard_correction_limit_m': .06,
        'visual_observation_confirmation_frames': 3, 'visual_observation_consistency_m': .005,
        'visual_observation_increment_limit_m': .01, 'visual_grasp_capture_depth_m': capture_depth,
        'visual_grasp_capture_lateral_tolerance_m': .005, 'cartesian_tolerance_m': .001,
    }, cartesian_pose=CollectionPipeline.cartesian_pose,
        pose_message=CollectionPipeline.pose_message, pose=lambda _arm: deepcopy(current),
        linear=linear, observe=lambda _reference, _after: next(captures),
        real=NS(monitor=lambda: deepcopy(measured)), event=lambda *_args, **_kwargs: None)
    selected = NS(selected_arm='left', reference_id='reference', fk_grasp_pose=target,
                  grasp_joint_state=JointState(name=['left_wrist_roll_joint'], position=[0.]))
    return node, selected, moves


def test_visual_approach_discards_one_centroid_jump_after_confirmation():
    from cleany_skill_executor.collection_pipeline import CollectionPipeline

    node, selected, moves = visual_approach_case([
        visual_observation(2, .04), visual_observation(3),
        visual_observation(4), visual_observation(5)])
    report = {}
    CollectionPipeline.visual_approach(
        node, selected, visual_observation(1), visual_observation(1), report)

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

    node, selected, moves = visual_approach_case(
        [visual_observation(stamp, .04) for stamp in (2, 3, 4)], capture_depth=.025)
    report = {}
    CollectionPipeline.visual_approach(
        node, selected, visual_observation(1), visual_observation(1), report)

    assert [stage for stage, _ in moves] == ['visual_approach_step']
    assert [position for _, position in moves] == pytest.approx([-.02])
    assert report['visual_approach'][
        'finalized_from_grasp_capture_corridor'] is True
    assert report['visual_approach']['observations'][-1][
        'inside_grasp_capture_corridor'] is True


def test_confirmed_centroid_jump_far_from_jaws_still_stops():
    from cleany_skill_executor.collection_pipeline import CollectionPipeline

    node, selected, _ = visual_approach_case(
        [visual_observation(2, .04), visual_observation(3, .04)],
        target_y=-.08, capture_depth=.045)
    node.real.monitor = lambda: pytest.fail('motion must not start')
    with pytest.raises(RuntimeError, match='outside the visual correction'):
        CollectionPipeline.visual_approach(
            node, selected, visual_observation(1), visual_observation(1, .04), {})


def test_wrist_confirmation_bridges_head_occlusion_to_last_safe_3d_target():
    from cleany_skill_executor.collection_pipeline import CollectionPipeline

    node, selected, moves = visual_approach_case([
        visual_observation(2, .04), visual_observation(3, .04),
        visual_observation(4), visual_observation(5)])
    node.config.update(visual_wrist_occlusion_fallback_enabled=True,
                       visual_wrist_occlusion_max_steps=1)
    wrist_checks = []
    node.confirm_wrist_target = lambda report, step: (
        wrist_checks.append(step) or {'stamp_ns': 5_000_000_000})
    report = {}
    CollectionPipeline.visual_approach(
        node, selected, visual_observation(1), visual_observation(1, .04), report)

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
@pytest.mark.parametrize('collision_checks,opening_collision_checks', [
    (None, None), (True, False), (False, None), (False, True), (False, False)])
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
