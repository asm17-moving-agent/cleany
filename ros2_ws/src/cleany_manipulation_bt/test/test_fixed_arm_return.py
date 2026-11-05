"""Fixed-waypoint selection and rejection without ROS or motor access."""
import json
from types import SimpleNamespace as NS
import pytest
from sensor_msgs.msg import JointState
from cleany_manipulation_bt.arm_return_motion import load_return_waypoints, move_arm_return
from cleany_skill_executor.core.physical_grasp_selection import ARM_JOINT_NAMES, InfrastructureError


def waypoint_document():
    return dict(frame='base_link',
        joint_names={arm:list(names) for arm,names in ARM_JOINT_NAMES.items()},
        arms={arm:[[.1]*5, [.3]*5] for arm in ARM_JOINT_NAMES})


def test_load_fixed_return_preserves_two_poses_and_joint_order(tmp_path):
    document = waypoint_document()
    path = tmp_path/'waypoints.json'
    path.write_text(json.dumps(document))
    assert load_return_waypoints(str(path)) == {
        arm:((.1,)*5, (.3,)*5) for arm in ARM_JOINT_NAMES}


@pytest.mark.parametrize('invalid', [None, [], [[0.]*5], [[0.]*5]*3,
    [[0.]*4, [0.]*5], [[True]*5, [0.]*5], [[float('nan')]*5, [0.]*5],
    [[float('inf')]*5, [0.]*5], [['0']*5, [0.]*5]])
def test_malformed_fixed_poses_are_rejected(tmp_path, invalid):
    document = waypoint_document()
    document['arms']['left'] = invalid
    path = tmp_path/'waypoints.json'
    path.write_text(json.dumps(document))
    with pytest.raises(ValueError, match='two finite five-joint'):
        load_return_waypoints(str(path))


@pytest.mark.parametrize('change', ['frame', 'missing_arm', 'order'])
def test_ambiguous_frame_ownership_or_order_is_rejected(tmp_path, change):
    document = waypoint_document()
    if change == 'frame':
        document['frame'] = 'world'
    elif change == 'missing_arm':
        del document['arms']['right']
    else:
        document['joint_names']['left'].reverse()
    path = tmp_path/'waypoints.json'
    path.write_text(json.dumps(document))
    with pytest.raises(ValueError, match='explicit base_link joint order'):
        load_return_waypoints(str(path))


@pytest.mark.parametrize('arm', ['left', 'right'])
@pytest.mark.parametrize('outcome', ['valid', 'blocked', 'unavailable'])
@pytest.mark.parametrize('check_collisions', [True, False])
def test_fixed_return_checks_one_complete_cubic_and_never_calls_rrt(arm, outcome, check_collisions):
    names = ARM_JOINT_NAMES[arm]
    other = 'right' if arm == 'left' else 'left'
    state = JointState(name=[*names, f'{arm}_gripper_joint',
                             *ARM_JOINT_NAMES[other], f'{other}_gripper_joint',
                             'head_pan_joint', 'head_tilt_joint'],
                       position=[0.]*5+[.8]+[.5]*5+[.2, .3, .4])
    selected, sent, events, snapshots = [], [], [], []
    poses = {side:((.1,)*5, (.3,)*5) for side in ARM_JOINT_NAMES}
    def validate(selected_arm, path):
        assert selected_arm == arm
        selected.append(path)
        if outcome == 'unavailable':
            raise InfrastructureError('Scene unavailable')
        return path if outcome == 'valid' else None
    def dispatch(*args, **kwargs):
        assert kwargs == ({'scene_barrier': 'same_pose'} if check_collisions
                          else {'check_collisions': False})
        sent.append(args)
    ops = NS(return_waypoints=poses, check_execution=lambda: None,
        value=lambda name: check_collisions,
        event=lambda name, **fields: events.append((name, fields)),
        motion=NS(set_current_state=snapshots.append, set_gripper_position=lambda *_: None,
                  _validated_cubic_path=validate,
                  plan=lambda *a, **k: pytest.fail('Fixed return invoked RRT')),
        real=NS(monitor=lambda: state, capture_scene_barrier=lambda _: 'same_pose',
                execute_arm_gripper=dispatch))
    home = (.4,)*5
    if outcome == 'valid' or not check_collisions:
        move_arm_return(ops, arm, home, -.37)
        assert len(sent) == 1
        returned_arm, trajectory, jaw = sent[0]
        assert returned_arm == arm
        assert tuple(trajectory.joint_names) == names
        assert [tuple(p.positions) for p in trajectory.points] == [
            (0.,)*5, *poses[arm], home]
        assert jaw.joint_names == [f'{arm}_gripper_joint']
        assert jaw.points[0].positions == pytest.approx([.8])
        assert jaw.points[-1].positions == pytest.approx([-.37])
        planned = events[-1][1]
        assert planned['route'] == 'fixed_waypoints'
        assert planned['collision_checks_enabled'] is check_collisions
        assert planned['fixed_waypoints_joint_rad'] == list(poses[arm])
    else:
        with pytest.raises(RuntimeError if outcome == 'blocked' else InfrastructureError,
                           match='RRT is disabled' if outcome == 'blocked' else 'Scene unavailable'):
            move_arm_return(ops, arm, home, -.37)
        assert not sent
    assert snapshots == [state]
    assert len(selected) == int(check_collisions)
    if check_collisions:
        assert selected[0].points == ((0.,)*5, *poses[arm], home)
    assert [fields['phase'] for name, fields in events if name == 'arm_return_planning_phase'] == (
        ['fixed_validation'] if check_collisions else [])
