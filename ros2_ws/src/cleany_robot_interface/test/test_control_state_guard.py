import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from control_msgs.msg import DynamicJointState, InterfaceValue

from cleany_robot_interface.control_state_guard import ControlStateGuard, EXPECTED_JOINTS, decode_feedback


def state(stamp=10_000_000_000, names=EXPECTED_JOINTS):
    names = sorted(names)
    rows = []
    for name in names:
        keys, values = ['position'], [0.0]
        if not name.startswith('head_'):
            keys += ['feedback_stamp_ns', 'hardware_fault']
            values += [float(stamp), 0.0]
        rows.append(InterfaceValue(interface_names=keys, values=values))
    return DynamicJointState(joint_names=names, interface_values=rows)


def guard():
    node = SimpleNamespace(fault='', require_driver_fault=True, expected_joint_names=EXPECTED_JOINTS,
        max_age=.25, last_stamp=None, publisher=Mock(), now_ns=10_100_000_000,
        cad_coordinates=None, get_logger=Mock(), _dependent_positions={}, _gripper_positions=None)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=node.now_ns))
    node.latch = lambda reason: setattr(node, 'fault', reason)
    node.feedback_is_fresh = lambda stamp: ControlStateGuard.feedback_is_fresh(node, stamp)
    node.expand_positions = lambda values: ControlStateGuard.expand_positions(node, values)
    return node


@pytest.mark.parametrize('problem', ['fault', 'missing_fault_interface', 'nan', 'different_stamp',
                                     'duplicate', 'missing_head', 'missing_position'])
def test_invalid_snapshot_blocks_ready(problem):
    node, message = guard(), state()
    arm = message.interface_values[2]
    if problem == 'fault':
        arm.values[2] = 1.0
    elif problem == 'missing_fault_interface':
        arm.interface_names.pop()
        arm.values.pop()
    elif problem == 'nan':
        arm.values[0] = math.nan
    elif problem == 'different_stamp':
        arm.values[1] += 1_000_000.
    elif problem == 'duplicate':
        message.joint_names[0] = message.joint_names[1]
    elif problem == 'missing_head':
        message.joint_names.pop(0)
        message.interface_values.pop(0)
    else:
        arm.interface_names.pop(0)
        arm.values.pop(0)
    ControlStateGuard.receive_hardware(node, message)
    assert node.fault and node.last_stamp is None
    node.publisher.publish.assert_not_called()


def test_single_arm_feedback_requires_every_configured_joint():
    expected = {name for name in EXPECTED_JOINTS if not name.startswith('right_')}
    message = state(names=expected)
    positions, stamp = decode_feedback(message, expected, True)
    assert positions == dict.fromkeys(expected, 0.0) and stamp == 10_000_000_000
    with pytest.raises(ValueError, match='Incomplete'):
        decode_feedback(message, EXPECTED_JOINTS, True)


def test_legacy_driver_does_not_ignore_present_fault_bit():
    node, message = guard(), state()
    node.require_driver_fault = False
    message.interface_values[2].interface_names.pop()
    message.interface_values[2].values.pop()
    ControlStateGuard.receive_hardware(node, message)
    assert not node.fault
    message.interface_values[3].values[2] = 1.0
    ControlStateGuard.receive_hardware(node, message)
    assert 'driver latched' in node.fault


def test_fresh_broadcaster_header_cannot_refresh_frozen_hardware_feedback():
    node, message = guard(), state()
    message.header.stamp.sec = 50
    ControlStateGuard.receive_hardware(node, message)
    result = node.publisher.publish.call_args.args[0]
    assert result.header.stamp.sec == 10 and set(result.name) == EXPECTED_JOINTS
    node.now_ns += 100_000_000
    ControlStateGuard.receive_hardware(node, message)
    node.publisher.publish.assert_called_once()
    node.now_ns += 100_000_000
    ControlStateGuard.receive_hardware(node, message)
    assert node.fault == 'Hardware feedback timeout'
    assert node.last_stamp == 10_000_000_000


@pytest.mark.parametrize('offset_ns', [-1_000_000_000, 1_000_000_000])
def test_stale_or_future_sample_cannot_make_guard_ready(offset_ns):
    node = guard()
    ControlStateGuard.receive_hardware(node, state(node.now_ns + offset_ns))
    assert node.fault == 'Hardware feedback timeout'
    node.publisher.publish.assert_not_called()


def test_cad_linkage_reused_until_a_gripper_changes():
    node = guard()
    node.cad_coordinates = Mock()
    node.cad_coordinates.dependent_positions.side_effect = [{'cad_finger': .1}, {'cad_finger': .2}]
    for index in range(3):
        message = state(10_000_000_000 + index * 20_000_000)
        joint = 'left_elbow_pitch_joint' if index < 2 else 'left_gripper_joint'
        message.interface_values[message.joint_names.index(joint)].values[0] = float(index)
        ControlStateGuard.receive_hardware(node, message)
    assert not node.fault
    results = [dict(zip(call.args[0].name, call.args[0].position)) for call in node.publisher.publish.call_args_list]
    assert [r['cad_finger'] for r in results] == [.1, .1, .2]
    assert results[1]['left_elbow_pitch_joint'] == 1.0
    assert node.cad_coordinates.dependent_positions.call_count == 2
