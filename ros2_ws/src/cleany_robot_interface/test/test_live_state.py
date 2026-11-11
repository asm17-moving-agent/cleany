import json
import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from builtin_interfaces.msg import Time

from cleany_robot_interface.core import JointMapping
from cleany_robot_interface.reader import ReadOnlyBus
from cleany_robot_interface.rgbd_frame_bridge import inverse_transform
from cleany_robot_interface.robot_state_reader import RobotStateReader
import cleany_robot_interface.robot_state_reader as reader
from cleany_robot_interface.state_profile import head_positions


HEAD = {'reference_ticks': [2010, 2150], 'encoder_directions': [-1, 1],
        'encoder_scale_rad_per_tick': math.tau / 4096}


def test_head_reference_direction_and_no_silent_wrapping():
    assert head_positions([2010, 2150], HEAD) == [0, 0]
    assert head_positions([3034, 3174], HEAD) == pytest.approx([-math.pi/2, math.pi/2])
    for raw in ([-1, 2150], [2010, 4096]):
        with pytest.raises(ValueError):
            head_positions(raw, HEAD)
    with pytest.raises(ValueError):
        head_positions([2010, 2150], {**HEAD, 'encoder_directions': [0, 1]})


def test_sdk_inverse_includes_translation_and_rotation():
    value = math.sqrt(.5)
    translation, quaternion = inverse_transform([1., 2., 3.], [0., 0., value, value])
    assert translation == pytest.approx([-2., 1., -3.])
    assert quaternion == pytest.approx([0., 0., -value, value])
    with pytest.raises(ValueError):
        inverse_transform([0., 0., 0.], [0., 0., 0., 0.])


def fake_node():
    joints = [JointMapping(f'joint_{i}', i, 777, 2048., 1, 0., 0, 0, 4095)
              for i in range(1, 13)]
    return SimpleNamespace(
        joints=joints, arm=SimpleNamespace(read=Mock(return_value=2048)),
        head=SimpleNamespace(read=Mock(side_effect=[2010, 2150])), head_mapping=HEAD,
        publisher=SimpleNamespace(publish=Mock()), last_warning=0.,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=1))),
        get_logger=lambda: SimpleNamespace(error=Mock()))


def test_complete_14_joint_frame_without_invented_velocity_or_effort():
    node = fake_node()
    RobotStateReader.poll(node)
    message = node.publisher.publish.call_args.args[0]
    assert len(message.name) == len(message.position) == 14
    assert message.name[-2:] == ['head_pan_joint', 'head_tilt_joint']
    assert list(message.position) == [0.] * 14
    assert not message.velocity and not message.effort


@pytest.mark.parametrize('failing_bus', ['arm', 'head'])
def test_either_bus_failure_suppresses_entire_frame(failing_bus):
    node = fake_node()
    getattr(node, failing_bus).read = Mock(side_effect=RuntimeError('Disconnected'))
    RobotStateReader.poll(node)
    node.publisher.publish.assert_not_called()


def test_failed_serial_open_cleanup_does_not_mask_original_error():
    bus = ReadOnlyBus.__new__(ReadOnlyBus)
    bus.port = SimpleNamespace(ser=None, closePort=Mock())
    bus.exclusive = False
    bus.close()
    bus.port.closePort.assert_not_called()


def shared_reader_inputs(tmp_path, monkeypatch, *, alias=False, head_baudrate=1_000_000):
    port = tmp_path / 'serial'
    port.touch()
    head_port = tmp_path / 'head_serial' if alias else port
    if alias:
        head_port.symlink_to(port)
    arm = {'port': str(port), 'baudrate': 1_000_000, 'calibration_dir': str(tmp_path),
           'arms': {side: {'calibration': side + '.json'} for side in ('left', 'right')}}
    head = {'serial_port': str(head_port), 'baudrate': head_baudrate,
            'motor_ids': {'pan': 13, 'tilt': 14}}
    parameters = {'rate_hz': 20.}
    for name, config in [('arm_config_file', arm), ('head_config_file', head),
                         ('head_mapping_file', HEAD)]:
        path = tmp_path / (name + '.json')
        path.write_text(json.dumps(config))
        parameters[name] = str(path)
    for side in ('left', 'right'):
        (tmp_path / (side + '.json')).write_text('{}')
    # Exercise the real reader constructor and poll with no ROS graph or device access.
    monkeypatch.setattr(reader.Node, '__init__', lambda *_args, **_kwargs: None)
    monkeypatch.setattr(reader.Node, 'declare_parameter', lambda *_args: None)
    monkeypatch.setattr(reader.Node, 'get_parameter', lambda _, name: SimpleNamespace(value=parameters[name]))
    monkeypatch.setattr(reader.Node, 'create_publisher', lambda *_args: SimpleNamespace(publish=Mock()))
    monkeypatch.setattr(reader.Node, 'create_timer', lambda *_args: Mock())
    monkeypatch.setattr(reader.Node, 'get_logger', lambda _: Mock())
    monkeypatch.setattr(reader.Node, 'get_clock', lambda _: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=1))))
    monkeypatch.setattr(reader.Node, 'destroy_node', lambda _: True)
    monkeypatch.setattr(reader, 'mappings', lambda *_args: fake_node().joints)
    monkeypatch.setattr(reader, 'verify_arm_calibration', lambda *_args: None)
    bus = Mock(spec=['read', 'close'])
    bus.read.side_effect = lambda motor, address, size: (
        777 if address == 3 else {13: 2010, 14: 2150}.get(motor, 2048))
    opened = Mock(return_value=bus)
    monkeypatch.setattr(reader, 'ReadOnlyBus', opened)
    return opened, bus


@pytest.mark.parametrize('alias', [False, True])
def test_shared_bus_publishes_all_motors_and_closes_once(tmp_path, monkeypatch, alias):
    opened, bus = shared_reader_inputs(tmp_path, monkeypatch, alias=alias)
    node = RobotStateReader()
    assert opened.call_count == 1
    bus.read.reset_mock()
    node.poll()
    assert [call.args for call in bus.read.call_args_list] == [
        (motor, 56, 2) for motor in range(1, 15)]
    message = node.publisher.publish.call_args.args[0]
    assert len(message.position) == 14
    assert not message.velocity and not message.effort
    node.destroy_node()
    bus.close.assert_called_once()


def test_shared_bus_head_model_failure_releases_single_owner(tmp_path, monkeypatch):
    opened, bus = shared_reader_inputs(tmp_path, monkeypatch)
    bus.read.return_value = 0
    bus.read.side_effect = None
    with pytest.raises(ValueError, match='Head ID 13'):
        RobotStateReader()
    assert opened.call_count == 1
    bus.close.assert_called_once()


def test_shared_bus_baudrate_mismatch_cannot_open_serial(tmp_path, monkeypatch):
    opened, _ = shared_reader_inputs(tmp_path, monkeypatch, head_baudrate=115_200)
    with pytest.raises(ValueError, match='same baudrate'):
        RobotStateReader()
    opened.assert_not_called()
