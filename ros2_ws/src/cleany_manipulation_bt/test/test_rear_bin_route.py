"""Exercise the web profile and real BT route adapter without commanding a robot."""
from contextlib import nullcontext
from hashlib import sha256
import json
from pathlib import Path
from types import SimpleNamespace as NS

from geometry_msgs.msg import Pose, Vector3
import pytest
import yaml

from cleany_manipulation_bt.backend import OperationError
from cleany_manipulation_bt.jetson_operations import JetsonOperations
from cleany_robot_interface.core import mappings, motor_calibration_directory
from cleany_skill_executor.collection_pipeline import CollectionPipeline
from cleany_skill_executor.core.collection import configuration_issues, load_arm_home
from cleany_skill_executor.core.physical_grasp_selection import ARM_JOINT_NAMES
from cleany_skill_executor.core.hardware_execution import ArmHold, CollisionValidationError
from cleany_skill_executor.core.rear_bin import RearBinRoute, RearBinTrajectories, TRAJECTORY_STAGES
from cleany_skill_executor.core.recorded_trajectory import CommandPoint, RecordedTrajectory


ROOT = Path(__file__).resolve().parents[4]


@pytest.fixture(params=['left', 'right'])
def trial(request):
    arm = request.param
    doc = json.loads((ROOT / 'ros2_ws/src/cleany_skill_executor/test/data/supervised_collection_fixture.json').read_text())
    doc.update(operation_mode='pick_rear_bin_release', label_destinations={'*': 'rear_bin_a'},
               return_to_pick_reverse_enabled=False, return_to_pick_collision_check_enabled=True,
               gripper_opening_collision_check_enabled=True, held_object_octomap_collision_check_enabled=True,
               rear_bin_a=json.loads((ROOT / 'configs/robot/dev_monitor_rear_bin_a.json').read_text()))
    assert configuration_issues(doc) == []
    route = RearBinRoute.from_config(doc, arm)
    node = object.__new__(JetsonOperations)
    node.config, node.report = doc, {}
    names = (*ARM_JOINT_NAMES[arm], arm+'_gripper_joint')
    q = [-.2, .3, .5, -.8, 1.33, .32]
    state = NS(name=names, position=q)
    node.real = NS(monitor=lambda: state)
    node.arm_config = NS(home=tuple(q[:5]), closed=-.37, opened=2.41)
    stages = []
    node.event = lambda stage, **kwargs: stages.append(stage)
    node.move_joint_pose = lambda side, target, stage: (stages.append(stage), q.__setitem__(slice(5), target))
    node.motion = NS(set_current_state=lambda state: None, set_gripper_position=lambda *args: None,
                     forward_kinematics=lambda *args: NS(position=route.release_xyz),
                     _validated_cubic_path=lambda side, path: path, state_is_valid=lambda *args: True)
    commands = []
    def dispatch(side, message, controller):
        commands.append([tuple(point.positions) for point in message.points])
        q[:5] = commands[-1][-1]
    node.execute_trajectory = dispatch
    node.jaw = lambda side, target, stage, **kwargs: (stages.append(stage), q.__setitem__(-1, target))
    pose = Pose()
    pose.orientation.w = 1.
    node.carrying = {'arm': arm, 'grasp_tcp_pose': pose}
    node.payload_selected = NS(selected_object=NS(obb_pose=pose, obb_size=Vector3(x=.012, y=.025, z=.030)))
    drop = Pose()
    drop.orientation.w = 1.
    drop.position.x, drop.position.y, drop.position.z = route.release_xyz
    node.pose = lambda side: drop
    node.detach = lambda: setattr(node, 'carrying', None)
    node.scene = NS(restore=lambda: None)
    node.release_grasp_scene_freeze = lambda: None
    context = NS(selected=NS(selected_arm=arm), goal=NS(destination_id='rear_bin_a'),
                 release_confirmed=False, arm_recovered=False)
    return node, arm, context, route, commands, stages, drop


def test_carry_release_and_return(trial):
    node, arm, context, route, commands, stages, _ = trial
    node.check_rear_bin_standby(arm)
    node.carry(context)
    node.check_placement(context)
    node.current_node = 'OpenGripperAtDestination'
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=123))
    node.open(context)
    node.current_node = 'ConfirmRelease'
    node.confirm_release(context)
    node.current_node = 'ReturnArm'
    node.return_arm(context)
    outbound = route.outbound(1.33)
    assert commands[0] == list(outbound)
    assert stages[:4] == ['rear_bin_entry', 'rear_bin_transport', 'release', 'release_assumed_at_rear_bin']
    if arm == 'left':
        assert route.return_side_clearance != outbound[2][:4]
        assert commands[1:] == [[outbound[-1], outbound[-2], outbound[-3],
                                 (*route.return_side_clearance, 1.33), route.park, node.arm_config.home]]
        assert stages[4:] == ['rear_bin_return', 'rear_bin_close_at_standby']
    else:
        assert commands[1:] == [list(reversed(outbound[2:])),
                               [outbound[2], route.park], [route.park, node.arm_config.home]]
        assert stages[4:] == ['rear_bin_retreat', 'rear_bin_park', 'rear_bin_standby', 'rear_bin_close_at_standby']
    assert context.arm_recovered and node.report['placement_state'] == 'ASSUMED'


def test_release_accepts_vertical_tcp_offset_when_box_clears_rim(trial):
    node, _, context, route, _, _, drop = trial
    drop.position.x += .0033
    drop.position.y += .0033
    drop.position.z -= .0125
    assert route.release_xyz[2] - drop.position.z > route.tolerance
    node.check_placement(context)


def test_release_rejects_box_below_rim_even_when_tcp_is_above_it(trial):
    node, _, context, route, commands, stages, drop = trial
    drop.position.z = route.rim_z + .010
    with pytest.raises(OperationError, match='Attached payload does not clear the rear opening'):
        node.check_placement(context)
    assert not commands and 'release' not in stages


@pytest.mark.parametrize('valid', [True, False])
def test_standalone_rear_release_checks_payload_before_opening(trial, valid):
    node, arm, context, route, _, stages, drop = trial
    node.config.update(pregrasp_motion_enabled=False, approach_execution_replanning_enabled=False,
                       lift_height_correction_enabled=False, wrist_observation_enabled=False)
    context.selected.selected_candidate = NS()
    node.scene.begin = lambda *_: None
    node.scene.allow_contacts_for = lambda _: None
    node.record_approach_commands = node.record_pickup_departure = lambda _: nullcontext()
    node.approach_target = node.close_until_contact = lambda *_: None
    node.attach = lambda *_: None
    node.linear = lambda *_, **__: None
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=123))
    node.wrist_observation_required = lambda: False
    node.real.wait_for_contact = lambda *_: None
    node.stop_wrist_camera = lambda: None
    node.return_from_rear_bin = lambda _: stages.append('return')
    node.releasing = False
    report = {}
    check_payload = node.check_rear_bin_release

    def check(selected_arm):
        stages.append('release_check')
        check_payload(selected_arm)

    node.check_rear_bin_release = check
    if valid:
        CollectionPipeline.after_pregrasp(node, context.selected, None, report)
        assert stages[-3:] == ['release_check', 'release_at_rear_bin', 'return']
        assert report['released'] and node.carrying is None
    else:
        drop.position.z = route.rim_z + .010
        with pytest.raises(ValueError, match='Attached payload does not clear the rear opening'):
            CollectionPipeline.after_pregrasp(node, context.selected, None, report)
        assert stages[-1] == 'release_check'
        assert 'release_at_rear_bin' not in stages and not node.releasing
        assert report['payload_preserved_on_failure'] and node.carrying['arm'] == arm


def test_jetson_home_clearance_and_cached_return_endpoints():
    robot = ROOT / 'configs/robot'
    profile = json.loads((robot / 'cad_grasp.json').read_text())
    arm_file = ROOT / profile['arm_config']
    arm_config = yaml.safe_load(arm_file.read_text())
    directory = motor_calibration_directory(arm_file, arm_config)
    calibrations = {arm: json.loads((directory / cfg['calibration']).read_text())
                    for arm, cfg in arm_config['arms'].items()}
    joints = {joint.name: joint for joint in mappings(arm_config, calibrations)}

    home_file = robot / 'arm_return_home.json'
    home = load_arm_home(home_file)
    cache_file = robot / 'jetson/rear_bin_trajectories.json'
    document = json.loads(cache_file.read_text())
    assert document['source_files']['home']['sha256'] == sha256(home_file.read_bytes()).hexdigest()
    cache = RearBinTrajectories.load(str(cache_file))
    for arm, stage in [('left', 'rear_bin_return'), ('right', 'rear_bin_standby')]:
        positions = dict(zip(ARM_JOINT_NAMES[arm], home[arm], strict=True))
        shoulder = joints[f'{arm}_shoulder_pitch_joint']
        elbow = joints[f'{arm}_elbow_pitch_joint']
        assert positions[shoulder.name] == pytest.approx(shoulder.radians(shoulder.range_min + 20), abs=1e-12, rel=0)
        assert positions[elbow.name] == pytest.approx(elbow.radians(elbow.range_max - 20), abs=1e-12, rel=0)
        assert tuple(document['home_joint_rad'][arm]) == home[arm]
        path = cache.paths[arm][stage]
        assert path.joint_names == ARM_JOINT_NAMES[arm]
        assert path.points[-1].positions == home[arm]


@pytest.fixture
def cached_trial(trial):
    node, arm, _, route, *_ = trial
    names = ARM_JOINT_NAMES[arm]
    outbound = route.outbound(1.60)
    paths = {'rear_bin_transport': outbound}
    if route.return_side_clearance is not None:
        paths['rear_bin_return'] = (outbound[-1], *route.retreat(1.60), route.park, node.arm_config.home)
    else:
        paths.update(rear_bin_retreat=tuple(reversed(outbound[2:])),
                     rear_bin_park=(outbound[2], route.park), rear_bin_standby=(route.park, node.arm_config.home))
    paths = {name: RecordedTrajectory(names, tuple(CommandPoint(i * 10**9, q, (0.,) * 5)
             for i, q in enumerate(points))) for name, points in paths.items()}
    node.description = 'runtime model'
    node.rear_bin_trajectories = RearBinTrajectories(sha256(node.description.encode()).hexdigest(), {arm: paths})
    node.real.arm_holds = {}
    node.real.value = {'hardware_start_tolerance_rad': .005, 'hardware_arrival_tolerance_rad': .10,
                       'hardware_command_continuity_max_offset_rad': .06}.__getitem__
    node.motion._validated_cubic_path = lambda *args: pytest.fail('Compatible cache should avoid replanning')
    return trial


def test_load_timed_trajectory_with_stationary_roundoff(tmp_path):
    artifact = {'schema_version': 1, 'source_files': {'robot_urdf': {'sha256': 'a' * 64}}, 'arms': {}}
    for arm, names in ARM_JOINT_NAMES.items():
        trajectory = {'joint_names': list(names), 'points': [
            {'positions': [.0] * 5, 'velocities': [.0] * 5, 'time_from_start': {'sec': 0, 'nanosec': 0}},
            {'positions': [.1] * 5, 'velocities': [1e-15] * 5, 'time_from_start': {'sec': 1, 'nanosec': 0}},
        ]}
        artifact['arms'][arm] = {'stages': {stage: {'trajectory': trajectory} for stage in TRAJECTORY_STAGES}}
    path = tmp_path / 'trajectories.json'
    path.write_text(json.dumps(artifact))
    assert set(RearBinTrajectories.load(str(path)).paths) == {'left', 'right'}
    artifact['arms']['left']['stages']['rear_bin_transport']['trajectory']['points'][-1]['velocities'][0] = .01
    path.write_text(json.dumps(artifact))
    with pytest.raises(ValueError, match='stationary'):
        RearBinTrajectories.load(str(path))


def test_precomputed_legs_use_held_commands_and_collision_checked_replay(cached_trial):
    node, arm, context, route, commands, stages, _ = cached_trial
    dispatch = []
    move = node.move_joint_pose
    entry = route.outbound(1.33)[0]
    held = (entry[0] + .01, *entry[1:])

    def enter(side, goal, stage):
        move(side, goal, stage)
        node.real.arm_holds[side] = ArmHold(goal, held)

    def replay(side, recorded, *, check_collisions, retime):
        assert check_collisions and retime
        assert recorded.points[0].positions == node.real.arm_holds[side].command
        q = recorded.points[-1].positions
        node.real.monitor().position[:5] = q
        node.real.arm_holds[side] = ArmHold(q, q)
        dispatch.append(recorded)

    node.move_joint_pose, node.real.execute_recorded_trajectory = enter, replay
    node.carry(context)
    node.current_node = 'OpenGripperAtDestination'
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=123))
    node.open(context)
    node.current_node = 'ConfirmRelease'
    node.confirm_release(context)
    node.current_node = 'ReturnArm'
    node.return_arm(context)
    assert stages[:2] == ['rear_bin_entry', 'rear_bin_transport']
    assert not commands and len(dispatch) == (2 if arm == 'left' else 4)
    assert dispatch[0].points[0].positions == held
    assert all(point.positions[-1] == 1.33 for point in (*dispatch[0].points, *dispatch[1].points[:4]))
    if arm == 'left':
        assert dispatch[1].points[-2].positions == route.park
    assert dispatch[-1].points[-1].positions == node.arm_config.home


@pytest.mark.parametrize('mismatch', ['model', 'start', 'target'])
def test_cache_mismatch_returns_to_live_planning(cached_trial, mismatch):
    node, arm, _, route, commands, _, _ = cached_trial
    start, *targets = route.outbound(1.33)
    node.real.monitor().position[:5] = start
    node.real.arm_holds[arm] = ArmHold(start, start)
    node.real.execute_recorded_trajectory = lambda *a, **k: pytest.fail('Incompatible cache dispatched')
    if mismatch == 'model':
        node.description = 'updated model'
    elif mismatch == 'start':
        node.real.monitor().position[0] += .12
    else:
        targets[-1] = (targets[-1][0] + .02, *targets[-1][1:])
    measured = tuple(node.real.monitor().position[:5])
    node.motion._validated_cubic_path = lambda side, path: path
    node.rear_bin_path(arm, tuple(targets), 'rear_bin_transport')
    assert commands == [[measured, *targets]]


def test_live_replay_collision_never_falls_through(cached_trial):
    node, arm, _, route, commands, _, _ = cached_trial
    start, *targets = route.outbound(1.33)
    node.real.monitor().position[:5] = start
    node.real.arm_holds[arm] = ArmHold(start, start)

    def collide(side, trajectory, *, check_collisions, retime):
        assert check_collisions
        raise CollisionValidationError('live map', phase='command_path', sample_index=19)

    node.real.execute_recorded_trajectory = collide
    with pytest.raises(CollisionValidationError, match='live map'):
        node.rear_bin_path(arm, tuple(targets), 'rear_bin_transport')
    assert not commands


@pytest.mark.parametrize('failure', ['collision', 'tcp', 'rim', 'standby', 'model', 'cancel'])
def test_abort_without_wrong_release_or_return(trial, failure):
    node, arm, context, route, commands, stages, drop = trial
    if failure == 'model':
        node.motion.forward_kinematics = lambda *args: NS(position=(0., 0., 0.))
        with pytest.raises(RuntimeError, match='robot model'):
            node.check_rear_bin_standby(arm)
    elif failure in ('collision', 'standby'):
        node.motion._validated_cubic_path = lambda *args: None
        with pytest.raises(RuntimeError):
            if failure == 'standby':
                node.check_rear_bin_standby(arm)
            else:
                node.rear_bin_path(arm, route.outbound(1.33), 'transport')
    elif failure == 'cancel':
        node.current_node = 'ReturnArmAfterCancel'
        node.move_arm_return = lambda *args, **kwargs: stages.append('recovery')
        node.return_arm(context)
        assert stages == ['recovery']
    else:
        if failure == 'tcp':
            drop.position.x += route.tolerance + .001
        else:
            node.payload_selected.selected_object.obb_size.x = .22
        with pytest.raises(OperationError):
            node.check_placement(context)
    assert not commands and 'release' not in stages
