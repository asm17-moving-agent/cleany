"""Single ROS node: bounded callbacks, asynchronous transports, monotonic policy."""
from threading import RLock
from time import monotonic

import rclpy
from ament_index_python.packages import get_package_share_directory
from cleany_interfaces import msg as messages
from cleany_interfaces.action import RequestRecovery
from cleany_interfaces.srv import CheckPermission, RequestFaultReset
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.task import Future
from sensor_msgs.msg import Image, LaserScan

from .config import load
from .models import Availability, RecoveryState
from .ros_transport import MissionStopPort, RecoveryPort
from .sensors import StampGuard, valid_image, valid_pose, valid_scan
from .supervisor import SystemSupervisor


class SupervisorNode(Node):
    def __init__(self, *, config=None, parameter_overrides=None):
        super().__init__('system_supervisor', parameter_overrides=parameter_overrides or [])
        self.declare_parameter('config_file', get_package_share_directory('cleany_system_supervisor') +
                               '/config/system_supervisor.yaml')
        config = config or load(self.get_parameter('config_file').value)
        self.settings, self.lock = config['ros'], RLock()
        self.action_group = ReentrantCallbackGroup()
        self.core = SystemSupervisor(config, recovery_port=RecoveryPort(self, self.action_group),
                                     stop_port=MissionStopPort(self, self.settings['stop_action'], self.action_group))
        self.stamps = StampGuard(self.settings['max_sensor_age_s'])
        self.lifecycle = {}
        self.lifecycle_pending = {}
        self.lifecycle_clients = {name: self.create_client(GetState, name + '/get_state')
                                  for name in set(self.settings['navigation_nodes'] +
                                                  self.settings['localization_nodes'])}
        self.pose_receipt, self.pose_valid = None, False
        self.waiters = []
        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publishers_by_name = {name: self.create_publisher(kind, 'system/' + name, state_qos)
                                   for name, kind in (
                                       ('state', messages.SystemState),
                                       ('capabilities', messages.SystemCapabilities),
                                       ('faults', messages.SystemFaults),
                                       ('recoveries', messages.SystemRecoveries))}
        self.heartbeat = self.create_publisher(messages.SystemHeartbeat, 'system/heartbeat', 1)
        self.create_subscription(DiagnosticArray, self.settings['diagnostics_topic'], self._diagnostics, 10)
        self.create_subscription(LaserScan, self.settings['scan_topic'], self._scan, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, self.settings['pose_topic'], self._pose,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, self.settings['camera_topic'], self._image, qos_profile_sensor_data)
        self.create_subscription(messages.ComponentHealth, self.settings['component_health_topic'],
                                 self._component, 10)
        self.create_subscription(messages.SafetyStatus, self.settings['safety_topic'], self._safety, 10)
        self.create_service(CheckPermission, 'system/check_permission', self._permission)
        self.create_service(RequestFaultReset, 'system/request_fault_reset', self._reset)
        self.recovery_server = ActionServer(
            self, RequestRecovery, 'system/request_recovery', self._recover,
            goal_callback=self._recovery_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.action_group)
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.05, self._tick, clock=steady)
        self.create_timer(self.settings['lifecycle_poll_s'], self._poll_lifecycle, clock=steady)

    def _fresh(self, source, header):
        return self.stamps.accept(source, header.stamp, self.get_clock().now().nanoseconds)

    def _observe(self, source, valid, detail):
        self.core.observe(source, Availability.AVAILABLE if valid else Availability.UNAVAILABLE, detail)

    def _scan(self, msg):
        with self.lock:
            self._observe('lidar', self._fresh('lidar', msg.header) and valid_scan(
                msg, self.settings['minimum_scan_valid_ratio']), 'scan timestamp, geometry and usable beams')

    def _pose(self, msg):
        with self.lock:
            self.pose_receipt = monotonic()
            self.pose_valid = self._fresh('pose', msg.header) and valid_pose(
                msg, self.settings['localization_max_variance'])

    def _image(self, msg):
        with self.lock:
            self._observe('camera', self._fresh('camera', msg.header) and valid_image(msg),
                          'image timestamp and payload; perception readiness is checked separately')

    def _component(self, msg):
        with self.lock:
            if msg.source not in self.settings['component_sources']:
                return  # Generic reports cannot impersonate independent safety or built-in monitors.
            status = {msg.AVAILABLE: Availability.AVAILABLE, msg.UNAVAILABLE: Availability.UNAVAILABLE}
            value = status.get(msg.status, Availability.UNKNOWN)
            if not self._fresh('component:' + msg.source, msg.header):
                value = Availability.UNKNOWN
            self.core.observe(msg.source, value, msg.detail)

    def _safety(self, msg):
        with self.lock:
            fresh = self._fresh('safety', msg.header)
            self._observe('safety', fresh and msg.protection_ready, 'independent protection verification')
            # The reset service cannot invent physical release evidence.
            self._observe('estop', fresh and not msg.estop_engaged and msg.physical_release_confirmed,
                          'physical e-stop release verification')
            self._observe('stopped', fresh and msg.protection_ready and msg.stopped,
                          'independent whole-robot stopped evidence for recovery dispatch')

    def _diagnostics(self, msg):
        with self.lock:
            for status in msg.status:
                if status.name == 'base_driver':
                    values = {entry.key: entry.value for entry in status.values}
                    try:
                        age = float(values.get('feedback_age_s', 'inf'))
                    except ValueError:
                        age = float('inf')
                    self._observe('base', self._fresh('base', msg.header) and status.level == DiagnosticStatus.OK and
                                  0 <= age <= self.core.config['sources']['base']['stale_s'], status.message)

    def _poll_lifecycle(self):
        now = monotonic()
        for name, client in self.lifecycle_clients.items():
            pending = self.lifecycle_pending.get(name)
            if pending and not pending[0].done():
                if now - pending[1] < self.settings['lifecycle_poll_s'] * 2:
                    continue
                client.remove_pending_request(pending[0])
            if client.service_is_ready():
                future = client.call_async(GetState.Request())
                self.lifecycle_pending[name] = (future, now)
                future.add_done_callback(lambda done, source=name: self._lifecycle_result(source, done))

    def _lifecycle_result(self, name, future):
        with self.lock:
            try:
                self.lifecycle[name] = (future.result().current_state.id == 3, monotonic())
            except Exception:
                self.lifecycle[name] = (False, monotonic())

    def _lifecycle_available(self, names, now):
        return bool(names) and all(name in self.lifecycle and self.lifecycle[name][0] and
                                   now - self.lifecycle[name][1] < self.settings['lifecycle_poll_s'] * 3
                                   for name in names)

    def _tick(self):
        with self.lock:
            now = monotonic()
            # Leave entirely unobserved sources UNKNOWN during boot; explicit bad states remain bad.
            if self.lifecycle:
                self._observe('navigation', self._lifecycle_available(self.settings['navigation_nodes'], now),
                              'all required Nav2 lifecycle nodes active and responding')
            if self.pose_receipt is not None:
                self._observe('localization', self.pose_valid and
                              now - self.pose_receipt <= self.core.config['sources']['localization']['stale_s'] and
                              self._lifecycle_available(self.settings['localization_nodes'], now),
                              'fresh bounded covariance pose and active localization lifecycle')
            self.core.tick()
            self._publish(now)
            waiting, self.waiters = self.waiters, []
            for future in waiting:
                if not future.done():
                    future.set_result(None)

    def _publish(self, now):
        stamp = self.get_clock().now().to_msg()

        def envelope(kind):
            msg = kind(epoch=self.core.epoch, revision=self.core.revision)
            msg.header.stamp = stamp
            return msg
        state = envelope(messages.SystemState)
        state.operating_state = self.core.state.value
        state.permissions = [messages.MissionPermission(
            mission_id=lease.mission_id, lease_id=lease.lease_id, version=lease.version,
            capabilities=list(lease.capabilities), granted=lease.granted,
            valid_for_s=max(0., lease.expires_at - now) if lease.granted else 0., reason=lease.reason)
            for lease in self.core.permissions.leases.values()]
        caps = envelope(messages.SystemCapabilities)
        caps.capabilities = [messages.SystemCapability(name=name, status=status.value)
                             for name, status in self.core.capabilities.states.items()]
        faults = envelope(messages.SystemFaults)
        faults.faults = [messages.SystemFault(
            code=f.code, source=f.source, severity=f.severity, capabilities=f.capabilities,
            first_seen_monotonic_s=f.first_seen, last_seen_monotonic_s=f.last_seen,
            active=f.active, latched=f.latched, recovery_policy=f.recovery_policy, detail=f.detail)
            for f in self.core.faults.faults.values()]
        recoveries = envelope(messages.SystemRecoveries)
        recoveries.recoveries = [messages.SystemRecovery(
            job_id=j.job_id, fault_code=j.fault_code, state=j.state.value, attempt=j.attempt,
            verifying_health=j.verify_after is not None, resource_held=j.resource_held, detail=j.detail)
            for j in self.core.recoveries.jobs.values()]
        for name, msg in (('state', state), ('capabilities', caps), ('faults', faults), ('recoveries', recoveries)):
            self.publishers_by_name[name].publish(msg)
        heartbeat = envelope(messages.SystemHeartbeat)
        heartbeat.timeout_s = self.core.config['heartbeat_timeout_s']
        self.heartbeat.publish(heartbeat)

    def _permission(self, request, response):
        with self.lock:
            result = self.core.check_permission(**{name: getattr(request, name) for name in
                ('epoch', 'mission_id', 'capabilities', 'lease_id', 'version', 'release')})
            for name in ('granted', 'epoch', 'lease_id', 'version', 'valid_for_s', 'reason'):
                setattr(response, name, getattr(result, name))
        return response

    def _reset(self, request, response):
        with self.lock:
            response.reset, response.reason = self.core.reset_fault(request.epoch, request.fault_code)
        return response

    def _recovery_goal(self, request):
        with self.lock:
            fault = self.core.faults.faults.get(request.fault_code)
            return (GoalResponse.ACCEPT if request.epoch == self.core.epoch and fault and
                    fault.active and fault.recovery_policy else GoalResponse.REJECT)

    async def _recover(self, goal):
        with self.lock:
            try:
                job = self.core.request_recovery(goal.request.epoch, goal.request.fault_code)
            except ValueError as exc:
                goal.abort()
                return RequestRecovery.Result(restored=False, state='FAILED', detail=str(exc))
        while rclpy.ok():
            with self.lock:
                if goal.is_cancel_requested:
                    self.core.recoveries.cancel(job, monotonic())
                if job.state not in (RecoveryState.IDLE, RecoveryState.RUNNING):
                    result = RequestRecovery.Result(restored=job.state == RecoveryState.SUCCEEDED,
                        job_id=job.job_id, state=job.state.value, detail=job.detail)
                    if goal.is_cancel_requested:
                        goal.canceled()
                    elif result.restored:
                        goal.succeed()
                    else:
                        goal.abort()
                    return result
                goal.publish_feedback(RequestRecovery.Feedback(job_id=job.job_id,
                                      state=job.state.value, attempt=job.attempt))
                future = Future()
                self.waiters.append(future)
            await future
        goal.abort()
        return RequestRecovery.Result(restored=False, state='FAILED', detail='supervisor shutdown')


def main(args=None):
    rclpy.init(args=args)
    node = SupervisorNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
