"""Exercise installed gyro correction and input recovery with synthetic IMU."""
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import pytest

os.environ['ROS_DOMAIN_ID'] = os.environ.get('BASE_TEST_ROS_DOMAIN_ID', '173')

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
import yaml

from cleany_base_driver.imu_node import ImuDriver


STATUS_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
BIAS = [.01, -.02, .03]
COVARIANCE = [1e-6, 0., 0., 0., 2e-6, 0., 0., 0., 3e-6]


def _wait(predicate, timeout=6.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.01)
    assert predicate()


def _parameters(**overrides):
    values = {
        'gyro_bias_rad_s': BIAS, 'gyro_covariance_rad2_s2': COVARIANCE,
        'stale_timeout_s': .15,
    }
    values.update(overrides)
    return [Parameter(k, value=v) for k, v in values.items()]


def _values(message):
    return {item.key: item.value for item in message.status[0].values}


def test_installed_gyro_correction_and_fault_recovery(tmp_path):
    from ament_index_python.packages import get_package_share_directory

    share = Path(get_package_share_directory('cleany_base_driver'))
    config = yaml.safe_load((share / 'config/imu_bringup.yaml').read_text())
    config['cleany_imu']['ros__parameters'].update({
        p.name: p.value for p in _parameters()
    })
    profile = tmp_path / 'imu.yaml'
    profile.write_text(yaml.safe_dump(config))
    rclpy.init()
    monitor = rclpy.create_node('imu_graph_monitor')
    calibrated, statuses, transforms = [], [], []
    monitor.create_subscription(
        Imu, 'imu/data_raw', calibrated.append, qos_profile_sensor_data)
    monitor.create_subscription(DiagnosticArray, 'imu/status', statuses.append, STATUS_QOS)
    monitor.create_subscription(TFMessage, 'tf', transforms.append, 10)
    monitor.create_subscription(TFMessage, 'tf_static', transforms.append, 10)
    publisher = monitor.create_publisher(Imu, 'imu_uncalibrated/data_raw', qos_profile_sensor_data)
    gyro, acceleration, unavailable, stamp_offset = [list(BIAS)], [9.80665], [False], [0.]

    def emit():
        message = Imu()
        stamp_ns = monitor.get_clock().now().nanoseconds + int(stamp_offset[0] * 1e9)
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        message.header.frame_id = 'imu_link'
        message.orientation_covariance[0] = -1.
        message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = gyro[0]
        message.linear_acceleration.z = acceleration[0]
        message.linear_acceleration_covariance[0] = -1. if unavailable[0] else 0.
        publisher.publish(message)

    timer = monitor.create_timer(.01, emit)
    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    thread = threading.Thread(target=executor.spin, daemon=True)
    with (tmp_path / 'launch.log').open('w') as log:
        process = subprocess.Popen(
            ['ros2', 'launch', 'cleany_base_driver', 'imu_bringup.launch.py',
             f'config:={profile}'], stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        thread.start()
        try:
            _wait(lambda: len(calibrated) >= 5 and bool(statuses))
            assert process.poll() is None
            assert statuses[-1].status[0].message == 'RUNNING'
            assert _values(statuses[-1])['bias_source'] == 'factory'
            assert calibrated[-1].angular_velocity.z == pytest.approx(0.)
            assert list(calibrated[-1].angular_velocity_covariance) == COVARIANCE
            assert calibrated[-1].orientation_covariance[0] == -1.
            assert calibrated[-1].linear_acceleration.z == 9.80665
            assert calibrated[-1].header.frame_id == 'imu_link'
            assert abs(monitor.get_clock().now().nanoseconds - (
                calibrated[-1].header.stamp.sec * 1_000_000_000
                + calibrated[-1].header.stamp.nanosec)) < 200_000_000
            for topic in ('/cmd_vel', '/base/wheel_command', '/odom'):
                assert not monitor.get_publishers_info_by_topic(topic)
            assert not monitor.get_publishers_info_by_topic('/imu/data')
            raw_info = monitor.get_publishers_info_by_topic('/imu/data_raw')
            assert len(raw_info) == 1
            assert raw_info[0].node_name == 'cleany_imu'
            assert raw_info[0].qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
            _wait(lambda: 'cleany_imu' in {
                endpoint.node_name
                for endpoint in monitor.get_subscriptions_info_by_topic('/imu_uncalibrated/data_raw')
            })
            gyro[0] = [BIAS[0], BIAS[1], BIAS[2] + .3]
            acceleration[0] = 0.
            _wait(lambda: calibrated[-1].angular_velocity.z == pytest.approx(.3)
                  and calibrated[-1].linear_acceleration.z == 0.)
            assert calibrated[-1].orientation_covariance[0] == -1.

            unavailable[0] = True
            _wait(lambda: statuses[-1].status[0].message == 'ERROR')
            assert statuses[-1].status[0].level == DiagnosticStatus.ERROR
            time.sleep(.1)
            output_count = len(calibrated)
            time.sleep(.1)
            assert len(calibrated) == output_count
            unavailable[0] = False
            _wait(lambda: statuses[-1].status[0].message == 'RUNNING')
            stamp_offset[0] = -.5
            _wait(lambda: 'backlog' in _values(statuses[-1])['reason'])
            stamp_offset[0] = 0.
            _wait(lambda: statuses[-1].status[0].message == 'RUNNING')

            timer.cancel()
            _wait(lambda: _values(statuses[-1])['reason'] == 'stale IMU feedback')
            output_count = len(calibrated)
            time.sleep(.15)
            assert len(calibrated) == output_count
            gyro[0] = list(BIAS)
            timer.reset()
            _wait(lambda: statuses[-1].status[0].message == 'RUNNING')
            _wait(lambda: calibrated[-1].angular_velocity.z == pytest.approx(0.))
            assert _values(statuses[-1])['bias_source'] == 'factory'
            assert not transforms
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2.)
            executor.shutdown()
            thread.join(timeout=2.)
            monitor.destroy_node()
            rclpy.shutdown()
    assert process.returncode == 0, (tmp_path / 'launch.log').read_text()


@pytest.mark.parametrize('params', [
    {'frame_id': ''}, {'frame_id': '/imu_link'},
    {'stale_timeout_s': -1.}, {'gyro_bias_rad_s': [0., 0.]},
    {'gyro_covariance_rad2_s2': [0.] * 9},
])
def test_invalid_imu_configuration(params):
    rclpy.init()
    try:
        with pytest.raises(ValueError):
            ImuDriver(parameter_overrides=_parameters(**params))
    finally:
        rclpy.shutdown()


def test_factory_coefficients_required_and_read_only():
    rclpy.init()
    try:
        with pytest.raises(ValueError, match='must be configured'):
            ImuDriver()
        driver = ImuDriver(parameter_overrides=_parameters())
        try:
            result = driver.set_parameters([Parameter('gyro_bias_rad_s', value=[0.] * 3)])
            assert not result[0].successful
            assert driver.calibration.bias == tuple(BIAS)
        finally:
            driver.destroy_node()
    finally:
        rclpy.shutdown()


def test_installed_imu_launch_and_profile():
    import importlib.util

    from ament_index_python.packages import get_package_share_directory
    from launch_ros.actions import Node

    share = Path(get_package_share_directory('cleany_base_driver'))
    spec = importlib.util.spec_from_file_location('imu_bringup', share / 'launch/imu_bringup.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    argument, driver_node = module.generate_launch_description().entities
    assert ''.join(part.text for part in argument.default_value) == str(share / 'config/imu_bringup.yaml')
    assert isinstance(driver_node, Node)
    profile = yaml.safe_load((share / 'config/imu_bringup.yaml').read_text())
    root = Path(__file__).resolve().parents[4]
    source = yaml.safe_load((root / 'ros2_ws/src/cleany_base_driver/config/imu_bringup.yaml').read_text())
    assert profile == source
    rclpy.init()
    try:
        driver = ImuDriver(parameter_overrides=[
            Parameter(k, value=v) for k, v in profile['cleany_imu']['ros__parameters'].items()
        ])
        driver.destroy_node()
    finally:
        rclpy.shutdown()
