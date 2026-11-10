import json
from types import SimpleNamespace as NS

from cleany_robot_interface.motor_telemetry import telemetry_record


def test_recorder_preserves_raw_evidence_and_never_emits_nan():
    message = NS(
        header=NS(stamp=NS(sec=2, nanosec=3)),
        joint_names=['left_shoulder_pitch_joint', 'head_pan_joint'],
        interface_values=[
            NS(interface_names=['position', 'command_position', 'torque_enabled', 'voltage_raw'],
               values=[.1, float('nan'), 1., 120.]),
            NS(interface_names=['position'], values=[0.]),
        ],
    )
    record = telemetry_record(message, 2_000_000_020)
    assert record['state_stamp_ns'] == 2_000_000_003
    assert len(record['joints']) == 1
    assert record['joints']['left_shoulder_pitch_joint'] == {
        'position': .1, 'command_position': None, 'torque_enabled': 1., 'voltage_raw': 120.,
    }
    json.dumps(record, allow_nan=False)
