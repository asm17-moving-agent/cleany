from __future__ import annotations

from math import cos, sin

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from cleany_base_odometry.mecanum_odometry import (
    MecanumGeometry,
    MecanumOdometry,
    WheelPositions,
)


class WheelOdometryNode(Node):
    """누적 휠 회전량으로 평면 odometry를 추정합니다.

    Subscriptions:
        joint_states (sensor_msgs/msg/JointState): 네 휠의 위치(rad).
            input_topic으로 입력 이름을 바꿀 수 있습니다.

    Publishers:
        wheel/odom (nav_msgs/msg/Odometry): odom 좌표계의 위치와 base_link의 속도.
            토픽과 좌표계 이름은 파라미터 기본값입니다.

    Services:
        ~/reset (std_srvs/srv/Trigger): 매핑 담당자가 STOP을 확인하고 이전 SLAM을
            종료한 뒤 위치와 encoder 기준점을 초기화합니다.

    TF:
        직접 발행하지 않으며 odometry relay가 odom → base_link를 담당합니다.

    QoS:
        입력은 sensor_data, 출력은 reliable과 depth 10을 사용합니다.

    Notes:
        첫 표본, 증가하지 않은 시각과 속도 창 초기화 구간에는 발행하지 않습니다.
        무효한 휠 값은 검증 오류를 발생시킵니다.
    """

    def __init__(self) -> None:
        super().__init__('wheel_odometry')
        self.declare_parameter('input_topic', 'joint_states')
        self.declare_parameter('output_topic', 'wheel/odom')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('velocity_window_s', 0.0)
        self.declare_parameter('yaw_scale', 1.0)
        self.declare_parameter('wheel_radius_m', 0.0635)
        self.declare_parameter('wheelbase_m', 0.30)
        self.declare_parameter('wheel_separation_m', 0.51)
        self.declare_parameter(
            'front_left_joint', 'front_left_wheel_joint'
        )
        self.declare_parameter(
            'front_right_joint', 'front_right_wheel_joint'
        )
        self.declare_parameter('rear_left_joint', 'rear_left_wheel_joint')
        self.declare_parameter('rear_right_joint', 'rear_right_wheel_joint')

        geometry = MecanumGeometry(
            wheel_radius_m=float(self.get_parameter('wheel_radius_m').value),
            wheelbase_m=float(self.get_parameter('wheelbase_m').value),
            wheel_separation_m=float(
                self.get_parameter('wheel_separation_m').value
            ),
        )
        self._geometry = geometry
        self._odometry = MecanumOdometry(
            geometry,
            velocity_window_s=float(self.get_parameter('velocity_window_s').value),
            yaw_scale=float(self.get_parameter('yaw_scale').value),
        )
        self.create_service(Trigger, '~/reset', self._on_reset)
        self._odom_frame_id = str(self.get_parameter('odom_frame_id').value)
        self._base_frame_id = str(self.get_parameter('base_frame_id').value)
        self._joint_names = {
            'front_left': str(self.get_parameter('front_left_joint').value),
            'front_right': str(
                self.get_parameter('front_right_joint').value
            ),
            'rear_left': str(self.get_parameter('rear_left_joint').value),
            'rear_right': str(self.get_parameter('rear_right_joint').value),
        }
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self._publisher = self.create_publisher(Odometry, output_topic, 10)
        self.create_subscription(
            JointState,
            input_topic,
            self._on_joint_state,
            qos_profile_sensor_data,
        )

    def _on_reset(self, request, response):
        """위치와 encoder 기준점을 초기화합니다. 세션 관리자가 먼저 STOP을 확인해야 합니다."""
        self._odometry = MecanumOdometry(
            self._geometry,
            velocity_window_s=float(self.get_parameter('velocity_window_s').value),
            yaw_scale=float(self.get_parameter('yaw_scale').value))
        response.success = True
        response.message = 'Odometry reset; waiting for fresh encoder baseline'
        return response

    def _on_joint_state(self, message: JointState) -> None:
        """네 휠의 완전한 표본을 적분하며 stamp가 0이면 노드 시각을 사용합니다.

        위치나 휠 이름이 부족하면 무시합니다. 유한하지 않은 휠 값은 검증에서
        오류가 발생하며 정지 상태로 치환하지 않습니다.
        """
        if len(message.position) < len(message.name):
            self.get_logger().warning(
                'Ignoring JointState with fewer positions than names',
                throttle_duration_sec=5.0,
            )
            return

        position_by_name = dict(zip(message.name, message.position))
        missing = [
            joint_name
            for joint_name in self._joint_names.values()
            if joint_name not in position_by_name
        ]
        if missing:
            self.get_logger().warning(
                f'Ignoring JointState missing wheel joints: {missing}',
                throttle_duration_sec=5.0,
            )
            return

        positions = WheelPositions(
            front_left=float(
                position_by_name[self._joint_names['front_left']]
            ),
            front_right=float(
                position_by_name[self._joint_names['front_right']]
            ),
            rear_left=float(
                position_by_name[self._joint_names['rear_left']]
            ),
            rear_right=float(
                position_by_name[self._joint_names['rear_right']]
            ),
        )
        stamp = message.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()
        stamp_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        estimate = self._odometry.update(positions, stamp_s)
        if estimate is None:
            return

        output = Odometry()
        output.header.stamp = stamp
        output.header.frame_id = self._odom_frame_id
        output.child_frame_id = self._base_frame_id
        output.pose.pose.position.x = estimate.x_m
        output.pose.pose.position.y = estimate.y_m
        output.pose.pose.orientation.z = sin(estimate.yaw_rad / 2.0)
        output.pose.pose.orientation.w = cos(estimate.yaw_rad / 2.0)
        output.twist.twist.linear.x = estimate.linear_x_mps
        output.twist.twist.linear.y = estimate.linear_y_mps
        output.twist.twist.angular.z = estimate.angular_z_rps
        self._publisher.publish(output)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass
