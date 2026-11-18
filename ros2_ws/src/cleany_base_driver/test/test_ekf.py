"""Test installed wheel/gyro fusion with mock wheels and synthetic raw IMU."""
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest

os.environ['ROS_DOMAIN_ID'] = os.environ.get('BASE_TEST_ROS_DOMAIN_ID', '173')
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage
import yaml


SHARE = Path(get_package_share_directory('cleany_base_driver'))
CONFIG = yaml.safe_load((SHARE / 'config/base_ekf.yaml').read_text())


def test_fusion_uses_planar_wheel_twist_and_fixed_bias_gyro():
    params = CONFIG['ekf_filter_node']['ros__parameters']
    assert [i for i, enabled in enumerate(params['odom0_config']) if enabled] == [6, 7, 11]
    assert [i for i, enabled in enumerate(params['imu0_config']) if enabled] == [11]
    assert params['imu0'] == 'imu/data_raw'
    assert params['two_d_mode'] and not params['publish_tf']
    assert params['world_frame'] == params['odom_frame'] == 'odom'
    diagonal = CONFIG['wheel_odometry']['ros__parameters']['twist_covariance_diagonal']
    assert len(diagonal) == 6 and all(math.isfinite(v) and v >= 0 for v in diagonal)
    assert all(diagonal[i] > 0 for i in (0, 1, 5))


@pytest.mark.parametrize(
    'mode, imu_variance_scale, mount_roll, expected_yaw_rate',
    [('shadow', .01, 0., .4), ('canonical', 100., 0., .2),
     ('shadow', .01, math.pi, .4)],
    ids=['shadow-imu-dominant', 'canonical-wheel-dominant', 'rotated-sensor'],
)
def test_installed_fusion_and_tf_ownership(
        tmp_path, mode, imu_variance_scale, mount_roll, expected_yaw_rate):
    diagonal = CONFIG['wheel_odometry']['ros__parameters']['twist_covariance_diagonal']
    profile = yaml.safe_load((SHARE / 'config/imu_bringup.yaml').read_text())
    params = profile['cleany_imu']['ros__parameters']
    bias = params['gyro_bias_rad_s']
    variance = diagonal[5] * imu_variance_scale
    # Vary synthetic uncertainty to verify both inputs influence the estimator.
    params['gyro_covariance_rad2_s2'] = [variance, 0., 0., 0., variance, 0., 0., 0., variance]
    imu_config = tmp_path / 'synthetic_imu.yaml'
    imu_config.write_text(yaml.safe_dump(profile))
    output_topic = 'odom' if mode == 'canonical' else 'odometry/filtered'
    tf_owner = 'ekf_filter_node' if mode == 'canonical' else 'base_odom_relay'
    env = dict(os.environ, ROS_LOG_DIR=str(tmp_path / 'ros'))
    with (tmp_path / 'launch.log').open('w+') as log:
        process = subprocess.Popen([
            'ros2', 'launch', 'cleany_base_driver', 'base_ekf.launch.py',
            'mock:=true', f'mode:={mode}', f'imu_config:={imu_config}',
            f'base_config:={SHARE / "config/base_synthetic.yaml"}',
            'publish_imu_tf:=true', f'imu_roll:={mount_roll}', 'imu_pitch:=0.0', 'imu_yaw:=0.0',
        ], env=env, stdout=log, stderr=log, start_new_session=True)
        rclpy.init(args=[])
        node = rclpy.create_node('ekf_graph_probe')
        fused, wheel, transforms, mounts, corrected, canonical = [], [], [], [], [], []
        node.create_subscription(Odometry, output_topic, fused.append, 10)
        node.create_subscription(Odometry, 'wheel/odom', wheel.append, 10)
        node.create_subscription(Imu, 'imu/data_raw', corrected.append, qos_profile_sensor_data)
        node.create_subscription(Odometry, 'odom', canonical.append, 10)
        node.create_subscription(TFMessage, 'tf', transforms.append, 10)
        node.create_subscription(
            TFMessage, 'tf_static', mounts.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        imu = node.create_publisher(Imu, 'imu_uncalibrated/data_raw', qos_profile_sensor_data)
        commands = node.create_publisher(Twist, 'cmd_vel', 10)
        enable = node.create_client(SetBool, 'base/enable')
        sensor_sign = -1. if mount_roll else 1.

        def pump(duration, command=None):
            end = time.monotonic() + duration
            while time.monotonic() < end:
                assert process.poll() is None, 'fusion launch exited'
                packet = Imu()
                packet.header.stamp = node.get_clock().now().to_msg()
                packet.header.frame_id = 'imu_link'
                packet.orientation_covariance[0] = -1.
                packet.angular_velocity.x, packet.angular_velocity.y = bias[:2]
                packet.angular_velocity.z = bias[2] + sensor_sign * .4
                packet.linear_acceleration.z = sensor_sign * 9.80665
                imu.publish(packet)
                if command is not None:
                    commands.publish(command)
                rclpy.spin_once(node, timeout_sec=.01)
                time.sleep(.005)

        def owners(topic):
            return [p.node_name for p in node.get_publishers_info_by_topic(topic)]

        def odom_edges():
            return [t for msg in transforms for t in msg.transforms
                    if t.header.frame_id == 'odom' and t.child_frame_id == 'base_link']

        def observations_ready():
            # Topic samples, graph discovery and TF can become available separately.
            return bool(enable.service_is_ready() and fused and wheel and corrected and canonical
                        and owners('base/wheel_state') == ['mock_base_mcu'] and odom_edges()
                        and any(t.header.frame_id == 'base_link' and t.child_frame_id == 'imu_link'
                                for msg in mounts for t in msg.transforms))

        try:
            end = time.monotonic() + 15
            while time.monotonic() < end and not observations_ready():
                pump(.05)
            assert observations_ready(), {
                'enable': enable.service_is_ready(), 'fused': len(fused), 'wheel': len(wheel),
                'imu': len(corrected), 'odom': len(canonical), 'tf': len(odom_edges()),
                'wheel_state_owners': owners('base/wheel_state'), 'mounts': len(mounts),
            }
            future = enable.call_async(SetBool.Request(data=True))
            end = time.monotonic() + 2
            while not future.done() and time.monotonic() < end:
                pump(.02)
            assert future.done() and future.result().success
            command = Twist()
            command.linear.x, command.linear.y, command.angular.z = .1, -.05, .2
            pump(2.5, command)
            velocity = fused[-1].twist.twist
            assert velocity.linear.x == pytest.approx(.1, abs=.025)
            assert velocity.linear.y == pytest.approx(-.05, abs=.02)
            assert velocity.angular.z == pytest.approx(expected_yaw_rate, abs=.02)
            def stamp_seconds(message):
                stamp = message.header.stamp
                return stamp.sec + stamp.nanosec * 1e-9

            # Encoder quantization and receipt-time jitter affect single samples.
            recent = [m for m in wheel if stamp_seconds(m) >= stamp_seconds(wheel[-1]) - 1.]
            first, last = recent[0], recent[-1]
            yaw_first = 2. * math.atan2(first.pose.pose.orientation.z, first.pose.pose.orientation.w)
            yaw_last = 2. * math.atan2(last.pose.pose.orientation.z, last.pose.pose.orientation.w)
            delta_yaw = math.atan2(math.sin(yaw_last - yaw_first), math.cos(yaw_last - yaw_first))
            assert delta_yaw / (stamp_seconds(last) - stamp_seconds(first)) == pytest.approx(.2, abs=.025)
            assert [wheel[-1].twist.covariance[i * 7] for i in range(6)] == diagonal
            assert corrected[-1].angular_velocity.z == pytest.approx(sensor_sign * .4)
            assert corrected[-1].orientation_covariance[0] == -1.
            assert fused[-1].header.frame_id == 'odom'
            assert fused[-1].child_frame_id == 'base_link'
            assert fused[-1].pose.pose.position.z == pytest.approx(0., abs=1e-6)
            end = time.monotonic() + 3.
            while time.monotonic() < end and (
                    owners(output_topic) != ['ekf_filter_node'] or owners('odom') != [tf_owner]):
                pump(.05, command)
            assert owners(output_topic) == ['ekf_filter_node']
            assert owners('odom') == [tf_owner]
            # Broadcaster endpoints also exist when publish_tf=false in Humble.
            client = node.create_client(GetParameters, 'ekf_filter_node/get_parameters')
            assert client.wait_for_service(timeout_sec=2.)
            future = client.call_async(GetParameters.Request(names=['publish_tf']))
            end = time.monotonic() + 2.
            while not future.done() and time.monotonic() < end:
                pump(.02, command)
            assert future.done() and future.result().values[0].bool_value == (mode == 'canonical')
            edges = odom_edges()
            assert edges and canonical
            transform = edges[-1].transform
            pose = canonical[-1].pose.pose
            assert transform.translation.x == pytest.approx(pose.position.x, abs=.02)
            assert transform.translation.y == pytest.approx(pose.position.y, abs=.02)
            assert transform.rotation.z == pytest.approx(pose.orientation.z, abs=.02)
            assert transform.rotation.w == pytest.approx(pose.orientation.w, abs=.02)
            assert any(t.header.frame_id == 'base_link' and t.child_frame_id == 'imu_link'
                       for msg in mounts for t in msg.transforms)
            assert ('base_odom_relay' in node.get_node_names()) == (mode == 'shadow')
        except Exception:
            log.flush()
            log.seek(0)
            print(log.read())
            raise
        finally:
            # A failed launch may exit before its mock child nodes do.
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                pass
            finally:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()
            node.destroy_node()
            rclpy.shutdown()
