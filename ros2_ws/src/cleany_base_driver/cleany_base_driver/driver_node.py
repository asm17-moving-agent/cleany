"""고정 크기 micro-ROS 휠 통신의 ROS 어댑터입니다."""
from __future__ import annotations

import time
from math import isfinite

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from cleany_base_interfaces.msg import WheelCommand, WheelState
from cleany_base_interfaces.srv import SetBaseEnabled
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

from .core import EncoderAdapter, Geometry, Limits, finite_twist_axes, newer, wheel_speeds

# 명시적인 해제는 정지한 사용 가능 상태로 남기고 그 외 사유는 고장으로 취급합니다.
SAFE_DISABLE_REASONS = frozenset({'stopped by request', 'cmd_vel timeout', 'MCU command timeout',
                                  'non-finite cmd_vel', 'driver shutdown'})


class BaseDriver(Node):
    """차체 명령을 MCU의 휠 구동 허가 프로토콜로 변환합니다.

    Subscriptions:
        cmd_vel (geometry_msgs/msg/Twist): 차체 좌표계의 평면 속도 요청.
        base/wheel_state (cleany_base_interfaces/msg/WheelState): 부팅 식별자,
            명령 수신 확인과 encoder count를 포함한 MCU 피드백.

    Publishers:
        base/wheel_command (cleany_base_interfaces/msg/WheelCommand): FL/FR/RL/RR 순서의
            구동 허가, 정지 또는 만료 시각이 있는 휠 속도 명령.
        joint_states (sensor_msgs/msg/JointState): 래핑을 푼 휠 회전량(rad)과 측정 각속도.
        diagnostics (diagnostic_msgs/msg/DiagnosticArray): 피드백 상태.

    Services:
        base/set_enabled (cleany_base_interfaces/srv/SetBaseEnabled): 보낸 명령의
            sequence와 MCU 부팅 식별자를 포함하는 순서 보장 구동 허가 RPC.
        base/enable (std_srvs/srv/SetBool): 구동 허가 또는 즉시 정지 요청.
            허가 응답의 성공은 전송을 뜻하며 MCU의 수신 확인은 별도입니다.

    QoS:
        휠 통신 양방향 모두 best-effort, volatile, depth 1을 사용합니다.

    Notes:
        구동 해제로 시작하며 형상과 제한값을 지정해야 합니다. MCU 수신 확인 전에는
        속도 명령을 차단합니다. 단조 증가 시계로 피드백 노후화, 명령 만료, 재부팅과
        호환되지 않는 피드백을 검사하고 정지합니다.
    """
    def __init__(self, parameter_overrides=None) -> None:
        super().__init__('cleany_base_driver', parameter_overrides=parameter_overrides or [])
        self.declare_parameter('mock', False)
        for key, default in (('geometry.wheel_radius_m', 0.), ('geometry.wheelbase_m', 0.),
                             ('geometry.wheel_separation_m', 0.)):
            self.declare_parameter(key, default)
        for key in ('linear_x_mps', 'linear_y_mps', 'angular_z_rad_s', 'wheel_rad_s', 'command_timeout_s'):
            self.declare_parameter('limits.' + key, 0.)
        self.declare_parameter('counts_per_revolution', 3172)
        self.disarmed_velocity_rad_s = float(self.declare_parameter('disarmed_velocity_rad_s', .02).value)
        if not isfinite(self.disarmed_velocity_rad_s) or self.disarmed_velocity_rad_s <= 0:
            raise ValueError('disarmed_velocity_rad_s must be finite and positive')
        self.geometry = Geometry(*(float(self.get_parameter('geometry.' + p).value)
                                   for p in ('wheel_radius_m', 'wheelbase_m', 'wheel_separation_m')))
        self.limits = Limits(*(float(self.get_parameter('limits.' + p).value)
                               for p in ('linear_x_mps', 'linear_y_mps', 'angular_z_rad_s',
                                         'wheel_rad_s', 'command_timeout_s')))
        self.encoder = EncoderAdapter(int(self.get_parameter('counts_per_revolution').value))
        self.command_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                       durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(WheelCommand, 'base/wheel_command', self.command_qos)
        self.joints = self.create_publisher(JointState, 'joint_states', 10)
        self.diags = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.create_subscription(WheelState, 'base/wheel_state', self.on_state, self.command_qos)
        self.create_subscription(Twist, 'cmd_vel', self.on_twist, 10)
        self.create_service(SetBool, 'base/enable', self.on_enable)
        self.create_service(SetBaseEnabled, 'base/set_enabled', self.on_ordered_enable)
        self.permission_requests = {}
        self.enabled = False
        self.latest = self.last_state_receipt = None
        self.last_cmd_receipt = self.last_state_sequence = self.last_mcu_timestamp = None
        self.latest_twist = None
        self.boot_id = None
        self.command_sequence = None
        self.pending_enable = None
        self.diagnostic_error = ''
        self.fault_reason = ''
        self._stop_sent = False
        self._stop_sequence = self._stop_sent_at = None
        self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(.1, self.publish_diagnostics, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _deadline(self) -> int:
        return self._mcu_now() + min(200000, int(self.limits.timeout_s * 1e6))

    def _mcu_now(self) -> int:
        return self.latest.timestamp_us + max(0, int((time.monotonic() - self.last_state_receipt) * 1e6))

    def _send(self, mode: int, values=(0., 0., 0., 0.), sequence=None, deadline=0) -> WheelCommand:
        msg = WheelCommand()
        msg.protocol_version = 2
        if sequence is None:
            self.command_sequence = ((self.command_sequence or 0) + 1) & 0xffffffff
            sequence = self.command_sequence
        msg.sequence, msg.mode = sequence, mode
        msg.velocity_rad_s = list(values)
        msg.valid_until_us = deadline
        self.publisher.publish(msg)
        return msg

    def _disable(self, reason: str, send_stop=True) -> None:
        self.enabled = False
        self.pending_enable = None
        self.last_cmd_receipt = None
        self.latest_twist = None
        self.diagnostic_error = reason
        if reason not in SAFE_DISABLE_REASONS and not self.fault_reason:
            self.fault_reason = reason  # 다음 ENABLE의 MCU 수신 확인까지 유지합니다.
        if send_stop and not self._stop_sent:
            self._stop_sequence = self._send(WheelCommand.STOP).sequence
            self._stop_sent, self._stop_sent_at = True, time.monotonic()

    def _stop_in_flight(self, msg: WheelState, now: float) -> bool:
        """MCU가 STOP을 처리하기 전 피드백에는 허가 상태가 남을 수 있어 시간 한계를 적용합니다."""
        return (self._stop_sequence is not None and newer(self._stop_sequence, msg.last_command_sequence)
                and now - self._stop_sent_at <= .25)

    def on_ordered_enable(self, request, response):
        """같은 제어기의 더 최신 STOP 이후에 도착한 지연 RPC를 거부합니다."""
        client = request.client_id
        if (not 1 <= len(client) <= 80 or request.request_sequence <= self.permission_requests.get(client, 0)
                or (client not in self.permission_requests and len(self.permission_requests) >= 1024)):
            response.reason = 'stale permission request or invalid client'
            return response
        self.permission_requests[client] = request.request_sequence
        permission = SetBool.Request(data=request.enabled)
        result = self.on_enable(permission, SetBool.Response())
        response.accepted = result.success
        response.command_sequence = self.command_sequence or 0
        response.boot_id = self.boot_id or 0
        response.reason = result.message
        return response

    def on_enable(self, request, response):
        if not request.data:
            self._stop_sent = False  # 명시적인 STOP마다 관측 가능한 독립 MCU sequence를 부여합니다.
            self._disable('stopped by request')
            response.success, response.message = True, 'STOP sent'
            return response
        if self.enabled:
            response.success, response.message = True, 'already enabled'
            return response
        if self.latest is None or self.last_state_receipt is None or time.monotonic() - self.last_state_receipt > .25:
            response.success, response.message = False, 'no fresh wheel feedback'
            return response
        if self.command_sequence is None:
            self.command_sequence = self.latest.last_command_sequence
        elif newer(self.latest.last_command_sequence, self.command_sequence):
            self.command_sequence = self.latest.last_command_sequence
        self.command_sequence = (self.command_sequence + 1) & 0xffffffff
        deadline = self._deadline()
        self.pending_enable = (self.command_sequence, deadline)
        self.enabled = True
        self._stop_sent = False
        self.last_cmd_receipt = None
        self.latest_twist = None
        self._send(WheelCommand.ENABLE, sequence=self.command_sequence, deadline=deadline)
        response.success, response.message = True, 'ENABLE sent; awaiting MCU acknowledgement'
        return response

    def on_twist(self, msg: Twist) -> None:
        axes = (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z)
        if not finite_twist_axes(*axes):
            if self.enabled:
                self._disable('non-finite cmd_vel')
            return
        if not self.enabled:
            return
        self.last_cmd_receipt = time.monotonic()
        self.latest_twist = (msg.linear.x, msg.linear.y, msg.angular.z)
        if any(v != 0 for v in (msg.linear.z, msg.angular.x, msg.angular.y)):
            self.get_logger().warning('unsupported cmd_vel axes ignored', throttle_duration_sec=2)

    def on_state(self, msg: WheelState) -> None:
        """상태의 유효 시간을 갱신하거나 구동 허가를 진행하기 전에 피드백을 검증합니다.

        중복 sequence와 증가하지 않은 MCU 시각은 watchdog을 갱신하지 않습니다.
        부팅 식별자가 바뀌면 구동을 해제하고 encoder 변화량의 기준을 다시 잡습니다.
        대기 중인 ENABLE의 sequence가 피드백에서 확인되어야 속도 명령을 보낼 수 있습니다.
        """
        now = time.monotonic()
        if msg.protocol_version != 2 or msg.boot_id == 0 or msg.counts_per_revolution != self.encoder.scale or \
                msg.max_velocity_rad_s != 10.0 or msg.watchdog_ms != 250:
            self._disable('WheelState contract mismatch')
            return
        if self.boot_id == msg.boot_id and self.last_state_sequence is not None and \
                not newer(msg.sequence, self.last_state_sequence):
            return
        if self.boot_id == msg.boot_id and self.last_mcu_timestamp is not None and \
                msg.timestamp_us <= self.last_mcu_timestamp:
            return
        reboot = self.boot_id is not None and self.boot_id != msg.boot_id
        if reboot:
            self.command_sequence = msg.last_command_sequence
            self._stop_sent = False
            self._disable('MCU reboot detected')
            self.last_state_sequence = self.last_mcu_timestamp = None
            self.encoder.rebase()
        if not all(isfinite(float(v)) for v in (*msg.velocity_rad_s, *msg.target_rad_s, *msg.commanded_rad_s)):
            self._disable('non-finite wheel feedback')
            return
        self.boot_id = msg.boot_id
        self.last_state_sequence = msg.sequence
        self.last_mcu_timestamp = msg.timestamp_us
        self.latest, self.last_state_receipt = msg, now
        if self.command_sequence is None or newer(msg.last_command_sequence, self.command_sequence):
            self.command_sequence = msg.last_command_sequence
        if self.pending_enable is not None:
            seq, _ = self.pending_enable
            if msg.last_command_sequence == seq and msg.enabled:
                self.pending_enable = None
                self.diagnostic_error = self.fault_reason = ''
        timeout_stop = (msg.fault_bits == WheelState.FAULT_COMMAND_TIMEOUT and not msg.enabled and
                        msg.control_mode == WheelState.TIMEOUT_STOP and
                        all(value == 0 for value in (*msg.target_rad_s, *msg.commanded_rad_s, *msg.pwm_percent)))
        if timeout_stop and self.pending_enable is None:
            stop_ack = (self._stop_sent and self._stop_sequence is not None and not self.enabled and
                        (msg.last_command_sequence == self._stop_sequence or
                         newer(msg.last_command_sequence, self._stop_sequence)))
            self._disable('MCU command timeout' if stop_ack else 'MCU fault bits: 0x1')
            if stop_ack and self.fault_reason == 'MCU fault bits: 0x1':
                self.fault_reason = ''  # STOP 수신 확인으로 해제되는 고장은 watchdog 고장뿐입니다.
        elif self.enabled and not msg.enabled and self.pending_enable is None:
            self._disable('MCU unexpectedly disabled')
        elif not self.enabled and msg.enabled and not self._stop_in_flight(msg, now):
            self._disable('MCU enabled without host permission')
        if msg.fault_bits and not timeout_stop:
            if self.pending_enable is None:
                self._disable(f'MCU fault bits: {msg.fault_bits:#x}')
        positions = self.encoder.update(msg.boot_id, tuple(msg.encoder_counts))
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = ['front_left_wheel_joint', 'front_right_wheel_joint',
                    'rear_left_wheel_joint', 'rear_right_wheel_joint']
        out.position, out.velocity = list(positions), [float(v) for v in msg.velocity_rad_s]
        self.joints.publish(out)

    def tick(self) -> None:
        now = time.monotonic()
        if self._stop_sent and (self.latest is None or self.latest.enabled or
                                newer(self._stop_sequence, self.latest.last_command_sequence)):
            # best-effort 통신에서는 STOP이 유실될 수 있어 원래 sequence로 재전송합니다.
            # 이때 구동 허가를 연장하거나 호출자에게 수신 완료로 보고하지 않습니다.
            self._send(WheelCommand.STOP, sequence=self._stop_sequence)
        if self.last_state_receipt is None or now - self.last_state_receipt > .25:
            if self.enabled:
                self._disable('stale wheel feedback')
            if self.latest is not None:
                self.latest = self.last_state_receipt = None
                self.encoder.rebase()
            return
        if not self.enabled:
            return
        if self.last_cmd_receipt is not None and now - self.last_cmd_receipt > self.limits.timeout_s:
            self._disable('cmd_vel timeout')
            return
        if self.pending_enable is not None:
            seq, deadline = self.pending_enable
            if self._mcu_now() >= deadline:
                self._disable('ENABLE acknowledgement timeout')
                return
            self._send(WheelCommand.ENABLE, sequence=seq, deadline=deadline)
            return
        if not self.latest.enabled:
            return
        twist = self.latest_twist
        wheels = (0.,) * 4 if twist is None else wheel_speeds(*twist, self.geometry, self.limits)
        self._send(WheelCommand.VELOCITY, wheels, deadline=self._deadline())

    def publish_diagnostics(self) -> None:
        status = DiagnosticStatus()
        age = float('inf') if self.last_state_receipt is None else time.monotonic() - self.last_state_receipt
        status.name, status.hardware_id = 'base_driver', 'micro_ros_base'
        status.level = DiagnosticStatus.ERROR if age > .25 or self.diagnostic_error else DiagnosticStatus.OK
        status.message = self.diagnostic_error or ('stale wheel feedback' if age > .25 else 'feedback current')
        # fault에는 명시적인 안전 해제(STOP 요청, cmd_vel 시간 초과)를 포함하지 않습니다.
        # 이 상태는 메시지와 진단 수준에 남지만 베이스 사용 불가를 뜻하지는 않습니다.
        status.values = [KeyValue(key='enabled', value=str(self.enabled)), KeyValue(key='feedback_age_s', value=str(age)),
                         KeyValue(key='feedback_received', value=str(self.last_state_receipt is not None)),
                         KeyValue(key='fault', value=self.fault_reason),
                         KeyValue(key='disarmed', value=str(self._disarmed(age)))]
        arr = DiagnosticArray()
        arr.header.stamp, arr.status = self.get_clock().now().to_msg(), [status]
        self.diags.publish(arr)

    def _disarmed(self, feedback_age_s: float) -> bool:
        """전체 로봇의 안전이나 STOP ACK를 대신하지 않고 측정된 베이스 정지만 보고합니다."""
        state = self.latest
        return bool(0 <= feedback_age_s <= .25 and state is not None and not self.enabled and
                    self.pending_enable is None and not state.enabled and
                    state.fault_bits in (0, WheelState.FAULT_COMMAND_TIMEOUT) and
                    state.control_mode in (WheelState.STOPPED, WheelState.TIMEOUT_STOP) and
                    all(value == 0 for value in (*state.target_rad_s, *state.commanded_rad_s, *state.pwm_percent)) and
                    all(isfinite(value) and abs(value) <= self.disarmed_velocity_rad_s
                        for value in state.velocity_rad_s))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = BaseDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node._disable('driver shutdown')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
