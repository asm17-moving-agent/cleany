"""ROS receives boot calibration; host/Agent startup never starts calibration."""
import os
from pathlib import Path
import threading
import time

import pytest

os.environ['ROS_DOMAIN_ID'] = os.environ.get('BASE_TEST_ROS_DOMAIN_ID', '173')

import rclpy
from cleany_base_interfaces.msg import ImuState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu

from cleany_base_driver.imu_node import ImuDriver


STATUS_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
BIAS = [.01, -.02, .03]
COVARIANCE = [1e-6, 0., 0., 0., 2e-6, 0., 0., 0., 3e-6]


def _wait(predicate, timeout=4.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.01)
    assert predicate()


def _parameters(**overrides):
    values = {
        'expected_device_mac': '90:e5:b1:d5:3f:34',
        'stale_timeout_s': .15,
    }
    values.update(overrides)
    return [Parameter(k, value=v) for k, v in values.items()]


def _values(message):
    return {item.key: item.value for item in message.status[0].values}


def test_gyro_relay_calibration_reboot_and_fault_recovery():
    rclpy.init()
    driver = ImuDriver(parameter_overrides=_parameters())
    monitor = rclpy.create_node('imu_graph_monitor')
    raw, corrected, statuses, diagnostics = [], [], [], []
    monitor.create_subscription(Imu, 'imu/data_raw', raw.append, qos_profile_sensor_data)
    monitor.create_subscription(Imu, 'imu/data', corrected.append, qos_profile_sensor_data)
    monitor.create_subscription(DiagnosticArray, 'imu/status', statuses.append, STATUS_QOS)
    monitor.create_subscription(DiagnosticArray, 'diagnostics', diagnostics.append, 10)
    publisher = monitor.create_publisher(ImuState, 'base/imu_state', qos_profile_sensor_data)
    state = [ImuState.RUNNING]  # PC connects after MCU calibration.
    counter, faults, boot = [0], [0], [19]
    bias, gyro = [list(BIAS)], [[.11, -.22, .33]]
    origin = [time.monotonic_ns()]

    def emit():
        counter[0] += 1
        selected = state[0] != ImuState.CALIBRATING
        publisher.publish(ImuState(
            protocol_version=2, device_mac=[0x90, 0xe5, 0xb1, 0xd5, 0x3f, 0x34],
            boot_id=boot[0], sequence=counter[0],
            timestamp_us=(time.monotonic_ns() - origin[0]) // 1000 + 1,
            who_am_i=0x68, fault_bits=faults[0], angular_velocity_rad_s=gyro[0],
            acceleration_mps2=[0., 0., 9.80665], temperature_c=27.5,
            calibration_state=state[0],
            calibration_reason=ImuState.REASON_TIMEOUT if state[0] == ImuState.FALLBACK else 0,
            corrected_angular_velocity_rad_s=(
                [x - b for x, b in zip(gyro[0], bias[0])] if selected else [0.] * 3),
            gyro_bias_rad_s=bias[0] if selected else [0.] * 3,
            gyro_covariance_rad2_s2=COVARIANCE if selected else [0.] * 9,
        ))

    timer = monitor.create_timer(.01, emit)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(driver)
    executor.add_node(monitor)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        _wait(lambda: len(raw) >= 3 and bool(statuses))
        # A late host gets ready immediately; it does not collect another window.
        _wait(lambda: bool(corrected) and statuses[-1].status[0].message == 'RUNNING', timeout=1.)
        assert corrected[-1].angular_velocity.z == pytest.approx(.3)
        assert corrected[-1].angular_velocity.x == pytest.approx(.1)
        assert corrected[-1].angular_velocity.y == pytest.approx(-.2)
        assert list(corrected[-1].angular_velocity_covariance) == COVARIANCE
        assert corrected[-1].orientation_covariance[0] == -1.
        assert corrected[-1].linear_acceleration_covariance[0] == -1.
        assert raw[-1].angular_velocity.z == pytest.approx(.33)
        assert list(raw[-1].angular_velocity_covariance) == [0.] * 9
        assert raw[-1].header.frame_id == 'imu_link'
        _wait(lambda: bool(diagnostics) and statuses[-1] == diagnostics[-1])
        fields = _values(statuses[-1])
        assert fields['bias_source'] == 'startup'
        assert float(fields['feedback_age_s']) < .15
        publishers = monitor.get_publishers_info_by_topic('/imu/status')
        assert publishers and all(
            info.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for info in publishers)
        for topic in ('/cmd_vel', '/base/wheel_command', '/odom', '/tf', '/tf_static'):
            assert not monitor.get_publishers_info_by_topic(topic)

        faults[0] = ImuState.FAULT_SENSOR
        _wait(lambda: statuses[-1].status[0].message == 'ERROR')
        assert statuses[-1].status[0].level == DiagnosticStatus.ERROR
        faults[0] = 0
        _wait(lambda: statuses[-1].status[0].message == 'RUNNING')
        assert _values(statuses[-1])['last_rejection']
        timer.cancel()
        _wait(lambda: statuses[-1].status[0].message == 'ERROR')
        assert _values(statuses[-1])['reason'] == 'stale IMU feedback'
        output_count = len(corrected)
        time.sleep(.15)
        assert len(corrected) == output_count
        timer.reset()
        _wait(lambda: statuses[-1].status[0].message == 'RUNNING')

        # A new MCU boot reports CALIBRATING and withholds corrected gyro again.
        state[0], boot[0], origin[0] = ImuState.CALIBRATING, 20, time.monotonic_ns()
        _wait(lambda: statuses[-1].status[0].message == 'CALIBRATING')
        output_count = len(corrected)
        time.sleep(.08)
        assert len(corrected) == output_count
        bias[0] = [.02, -.03, .04]
        state[0] = ImuState.FALLBACK
        _wait(lambda: statuses[-1].status[0].message == 'FALLBACK')
        _wait(lambda: corrected[-1].angular_velocity.z == pytest.approx(.29))
        assert _values(statuses[-1])['boot_id'] == '20'
        assert _values(statuses[-1])['bias_source'] == 'fallback'
        assert _values(statuses[-1])['reason'] == 'calibration timeout'
        assert statuses[-1].status[0].level == DiagnosticStatus.WARN

        time.sleep(.1)
        status_count, sample_count = len(statuses), len(raw)
        time.sleep(1.25)
        assert 1 <= len(statuses) - status_count <= 2
        assert len(raw) - sample_count > 20
    finally:
        executor.shutdown()
        thread.join(timeout=2)
        driver.destroy_node()
        monitor.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('params', [
    {'frame_id': ''}, {'frame_id': '/imu_link'},
    {'stale_timeout_s': -1.}, {'expected_device_mac': 'bad'},
])
def test_invalid_imu_configuration(params):
    rclpy.init()
    try:
        with pytest.raises(ValueError):
            ImuDriver(parameter_overrides=_parameters(**params))
    finally:
        rclpy.shutdown()


def test_installed_imu_launch_and_profile():
    import importlib.util

    from ament_index_python.packages import get_package_share_directory
    from launch_ros.actions import Node
    import yaml

    share = Path(get_package_share_directory('cleany_base_driver'))
    spec = importlib.util.spec_from_file_location('imu_bringup', share / 'launch/imu_bringup.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    argument, node = module.generate_launch_description().entities
    assert ''.join(part.text for part in argument.default_value) == str(share / 'config/imu_bringup.yaml')
    assert isinstance(node, Node)
    profile = yaml.safe_load((share / 'config/imu_bringup.yaml').read_text())['cleany_imu']['ros__parameters']
    assert 'calibration_samples' not in profile
    assert 'fallback_bias_rad_s' not in profile
    root = Path(__file__).resolve().parents[4]
    source = yaml.safe_load((root / 'configs/robot/imu_bringup.yaml').read_text())
    assert profile == source['cleany_imu']['ros__parameters']
    rclpy.init()
    try:
        driver = ImuDriver(parameter_overrides=[Parameter(k, value=v) for k, v in profile.items()])
        driver.destroy_node()
    finally:
        rclpy.shutdown()
