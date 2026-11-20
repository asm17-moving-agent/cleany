"""Semantic localization health and stationary recovery owned by the localization component."""
from __future__ import annotations

from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
from threading import Lock, RLock
from time import monotonic, sleep

import numpy as np
import rclpy
from cleany_interfaces.action import RecoverComponent
from cleany_interfaces.msg import ComponentHealth
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import SpeedLimit
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener

from .localization_alignment import AlignmentHealth, AlignmentMap, AlignmentSettings, search_pose, transform, wrap


def yaw(q) -> float:
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def stamp(header) -> float:
    return header.stamp.sec+header.stamp.nanosec*1e-9


class LocalizationGuard(Node):
    """Check scan/map agreement and recover only while the base is disarmed.

    Subscriptions:
        map (OccupancyGrid): Static map; transient-local depth 1.
        scan (LaserScan): Filtered planar ranges; sensor-data QoS.
        amcl_pose (PoseWithCovarianceStamped): Fresh finite covariance evidence.
        diagnostics (DiagnosticArray): Base driver's validated disarmed evidence.
    Publishers:
        system/component_health (ComponentHealth): localization_alignment health.
        localization/status (String): JSON measurements and recovery progress.
        navigation/localization_speed_limit (SpeedLimit): Percentage speed cap;
            degraded retains verified permission with reduced speed.
        initialpose (PoseWithCovarianceStamped): Bounded validated recovery pose.
    Actions:
        localization/recover (RecoverComponent): Serialized component recovery.
    Services:
        request_nomotion_update (Empty): Refresh real AMCL observations while stopped.
    TF:
        Reads scan-time map -> base_link and the scan extrinsic; publishes none.
    Notes:
        Sends no wheel or velocity commands. The Supervisor owns permission and
        stop dispatch. Candidate search uses separate fit/validation beams and
        refuses ambiguous map matches; it cannot prove absolute ground truth.
    """

    def __init__(self, **kwargs):
        super().__init__('localization_guard', **kwargs)
        self.settings = AlignmentSettings(**{k:self.declare_parameter(k, v).value
                                           for k, v in asdict(AlignmentSettings()).items()})
        defaults = dict(maximum_input_age_s=.5, pose_timeout_s=1., verification_duration_s=1.5,
                        operation_timeout_s=12., minimum_range_m=.5, maximum_range_m=8.,
                        maximum_pose_variance=.5, covariance_roundoff_tolerance=1e-9,
                        initial_position_std_m=.1, initial_yaw_std_rad=.0872664626)
        self.options = {k:float(self.declare_parameter(k, v).value) for k, v in defaults.items()}
        if not all(math.isfinite(v) and v > 0 for v in self.options.values()):
            raise ValueError('Localization timing/range/covariance settings must be positive')
        if self.options['minimum_range_m'] >= self.options['maximum_range_m']:
            raise ValueError('Invalid localization range bounds')
        self.evidence_directory = self.declare_parameter('evidence_directory', '').value
        self.degraded_speed_percentage = float(self.declare_parameter('degraded_speed_percentage', 30.).value)
        if not 0 < self.degraded_speed_percentage < 100:
            raise ValueError('Degraded speed percentage must be between 0 and 100')
        self.lock, self.busy = RLock(), Lock()
        self.health = AlignmentHealth(self.settings)
        self.model = self.map_revision = self.scan = self.pose_message = None
        self.last_scan_stamp = -math.inf
        self.observation = None
        self.state, self.reason = 'unknown', 'waiting_for_map_scan_pose'
        self.phase, self.recovery_reason = 'idle', ''
        self.disarmed, self.disarmed_at = False, -math.inf
        self.initial_revision, self.own_initial_stamp = 0, None
        self.last_nomotion = -math.inf
        self.status = self.create_publisher(String, 'localization/status', 1)
        self.speed_limit = self.create_publisher(SpeedLimit, 'navigation/localization_speed_limit', 10)
        self.component = self.create_publisher(ComponentHealth, 'system/component_health', 10)
        self.initial = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.nomotion = self.create_client(Empty, 'request_nomotion_update')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(OccupancyGrid, 'map', self.on_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_pose, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.on_initial, 10)
        self.create_subscription(DiagnosticArray, 'diagnostics', self.on_diagnostics, 10)
        self.action_group = ReentrantCallbackGroup()
        self.server = ActionServer(self, RecoverComponent, 'localization/recover', self.recover,
                                   goal_callback=self.admit_recovery,
                                   cancel_callback=lambda _:CancelResponse.ACCEPT,
                                   callback_group=self.action_group)
        self.create_timer(.2, self.tick)

    def on_map(self, message):
        p, q = message.info.origin.position, message.info.origin.orientation
        try:
            cells = np.array(message.data).reshape(message.info.height, message.info.width)
            metadata = [message.header.frame_id, message.info.width, message.info.height,
                        message.info.resolution, p.x, p.y, yaw(q)]
            if message.header.frame_id != 'map':
                raise ValueError('map must use the map frame')
            revision = hashlib.sha256(json.dumps(metadata).encode()+cells.tobytes()).hexdigest()
            if revision == self.map_revision:
                return
            model = AlignmentMap(cells, message.info.resolution, (p.x, p.y, yaw(q)))
        except (ValueError, TypeError) as error:
            self.get_logger().error(str(error))
            with self.lock:self.model = None
            return
        with self.lock:
            self.model, self.map_revision = model, revision
            self.health = AlignmentHealth(self.settings)
            self.observation = None
            self.last_scan_stamp = -math.inf

    def on_scan(self, message):
        with self.lock:self.scan = message

    def on_pose(self, message):
        with self.lock:self.pose_message = message

    def on_initial(self, message):
        with self.lock:
            if stamp(message.header) != self.own_initial_stamp:
                self.initial_revision += 1
                self.health = AlignmentHealth(self.settings)

    def on_diagnostics(self, message):
        now = self.get_clock().now().nanoseconds/1e9
        for status in message.status:
            if status.name != 'base_driver':
                continue
            values = {v.key:v.value for v in status.values}
            try:
                valid = (0 <= now-stamp(message.header) <= .3 and
                         0 <= float(values.get('feedback_age_s', 'inf')) <= .25 and
                         values.get('disarmed') == 'True' and not values.get('fault'))
            except ValueError:valid = False
            with self.lock:self.disarmed, self.disarmed_at = valid, monotonic()

    def is_stopped(self) -> bool:
        with self.lock:return self.disarmed and monotonic()-self.disarmed_at <= .3

    def _publish(self):
        with self.lock:
            observation = self.observation
            report = dict(state=self.state, reason=self.reason, phase=self.phase,
                          recovery_reason=self.recovery_reason, map_revision=self.map_revision,
                          reconvergence_reason=self.health.reason)
            if observation:
                report.update(stamp=observation['stamp'], pose=observation['pose'].tolist(),
                              score=asdict(observation['score']))
            percentage = 100. if self.state == 'available' else (
                self.degraded_speed_percentage if self.state == 'degraded' else 0.)
            report['speed_limit_percentage'] = percentage
            value = {'available':ComponentHealth.AVAILABLE, 'degraded':ComponentHealth.AVAILABLE,
                     'unavailable':ComponentHealth.UNAVAILABLE}.get(
                self.state, ComponentHealth.UNKNOWN)
        message = ComponentHealth(source='localization_alignment', status=value, detail=self.reason)
        message.header.stamp = self.get_clock().now().to_msg()
        self.component.publish(message)
        limit = SpeedLimit(percentage=True, speed_limit=percentage)
        limit.header.stamp = message.header.stamp
        self.speed_limit.publish(limit)
        self.status.publish(String(data=json.dumps(report)))

    def tick(self):
        with self.lock:model, revision, scan, pose_msg = self.model, self.map_revision, self.scan, self.pose_message
        now = self.get_clock().now().nanoseconds/1e9
        reason = confidence_reason = None
        if (self.is_stopped() and (self.state != 'available' or self.phase == 'verifying_amcl') and
                monotonic()-self.last_nomotion >= .5 and self.nomotion.service_is_ready()):
            self.nomotion.call_async(Empty.Request())
            self.last_nomotion = monotonic()
        if model is None or scan is None or pose_msg is None:
            reason = 'waiting_for_map_scan_pose'
        elif not 0 <= now-stamp(scan.header) <= self.options['maximum_input_age_s']:
            reason = 'scan_stale'
        elif not 0 <= now-stamp(pose_msg.header) <= self.options['pose_timeout_s']:
            reason = 'amcl_pose_stale'
        elif not scan.header.frame_id or not math.isfinite(scan.angle_increment) or scan.angle_increment <= 0:
            reason = 'invalid_scan_geometry'
        elif pose_msg.header.frame_id != 'map':
            reason = 'invalid_pose_frame'
        else:
            covariance = pose_msg.pose.covariance
            q = pose_msg.pose.pose.orientation
            norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
            if not math.isfinite(norm) or abs(norm-1.) > .05:
                reason = 'invalid_pose_orientation'
            elif any(not math.isfinite(v) for v in covariance) or any(
                    covariance[i] < -self.options['covariance_roundoff_tolerance'] for i in (0, 7, 35)):
                reason = 'invalid_pose_covariance'
            elif any(covariance[i] > self.options['maximum_pose_variance'] for i in (0, 7, 35)):
                confidence_reason = 'amcl_covariance_unverified'
        if reason:
            with self.lock:
                self.state, self.reason = 'unknown', reason
                self.observation = None
                self.health.invalidate()
            self._publish()
            return
        when = stamp(scan.header)
        if when > self.last_scan_stamp:
            try:
                mount = self.buffer.lookup_transform('base_link', scan.header.frame_id, Time()).transform
                current = self.buffer.lookup_transform('map', 'base_link', Time.from_msg(scan.header.stamp)).transform
                ranges = np.array(scan.ranges)
                angles = scan.angle_min+np.arange(len(ranges))*scan.angle_increment
                valid = np.isfinite(ranges) & (ranges >= self.options['minimum_range_m']) & (
                    ranges <= min(scan.range_max, self.options['maximum_range_m']))
                points = np.column_stack((ranges[valid]*np.cos(angles[valid]), ranges[valid]*np.sin(angles[valid])))
                points = transform(points, np.array([mount.translation.x, mount.translation.y, yaw(mount.rotation)]))
                if len(points) > self.settings.max_points:
                    points = points[np.linspace(0, len(points)-1, self.settings.max_points, dtype=int)]
                pose = np.array([current.translation.x, current.translation.y, yaw(current.rotation)])
                score = model.score(points, pose, self.settings)
                with self.lock:
                    if revision == self.map_revision:
                        self.last_scan_stamp = when
                        self.state = self.health.observe(when, score)
                        self.reason = 'aligned' if self.state == 'available' else (
                            'map_scan_mismatch' if self.state == 'unavailable' else (
                                'reconverging' if self.state == 'degraded' else 'alignment_warming_up'))
                        if confidence_reason:
                            self.state, self.reason = 'unknown', confidence_reason
                            self.health.invalidate()
                        self.observation = dict(stamp=when, pose=pose, points=points, score=score,
                                                map_revision=revision, model=model)
            except Exception as error:
                with self.lock:
                    self.state, self.reason = 'unknown', 'scan_time_tf_unavailable'
                    self.observation = None
                    self.health.invalidate()
                self.get_logger().debug(str(error))
        self._publish()

    def admit_recovery(self, request):
        allowed = (bool(request.job_id) and request.source in ('localization', 'localization_alignment') and
                   request.fault_code in ('LOCALIZATION_UNAVAILABLE', 'LOCALIZATION_ALIGNMENT_UNAVAILABLE'))
        return GoalResponse.ACCEPT if allowed and self.busy.acquire(blocking=False) else GoalResponse.REJECT

    def _phase(self, phase, reason=''):
        with self.lock:self.phase, self.recovery_reason = phase, reason

    def _fresh_observation(self):
        with self.lock:observation = self.observation
        if not observation or not 0 <= self.get_clock().now().nanoseconds/1e9-observation['stamp'] <= self.options['maximum_input_age_s']:
            return None
        return observation

    def _save(self, job_id, observation, result):
        if not self.evidence_directory:
            return
        folder = Path(self.evidence_directory)
        folder.mkdir(parents=True, exist_ok=True)
        key = hashlib.sha256(job_id.encode()).hexdigest()[:16]
        payload = dict(job_id=job_id, map_revision=observation['map_revision'],
                       stamp=observation['stamp'], seed=observation['pose'].tolist(),
                       settings=asdict(self.settings), result=result)
        (folder/f'{key}.json').write_text(json.dumps(payload, indent=2))
        np.savez_compressed(folder/f'{key}.npz', cells=observation['model'].cells,
                            resolution=observation['model'].resolution, origin=observation['model'].origin,
                            points=observation['points'], seed=observation['pose'])

    def recover(self, handle):
        deadline = monotonic()+self.options['operation_timeout_s']
        result = {'accepted':False, 'reason':'waiting_for_fresh_evidence'}
        observation = None
        initial_revision = self.initial_revision
        def canceled():
            return (handle.is_cancel_requested or monotonic() >= deadline or not self.is_stopped() or
                    initial_revision != self.initial_revision or
                    (observation is not None and self.map_revision != observation['map_revision']))
        try:
            self._phase('waiting_for_stop')
            while monotonic() < deadline and rclpy.ok():
                observation = self._fresh_observation()
                if self.is_stopped() and observation is not None:break
                if handle.is_cancel_requested:break
                sleep(.05)
            if observation is None or canceled():
                result['reason'] = 'stop_or_fresh_evidence_unverified'
            elif observation['score'].acceptable(self.settings) and self.state == 'available':
                result = {'accepted':True, 'reason':'already_aligned'}
            else:
                self._phase('searching')
                handle.publish_feedback(RecoverComponent.Feedback(detail='bounded stationary scan/map search'))
                result = search_pose(observation['model'], observation['points'], observation['pose'],
                                     self.settings, canceled,
                                     require_improvement=not observation['score'].acceptable(self.settings))
                if result['accepted'] and not canceled():
                    self._phase('verifying_candidate')
                    # Validate the same candidate against another genuinely new scan before changing AMCL.
                    candidate = np.array(result['pose'])
                    next_observation = None
                    while not canceled():
                        next_observation = self._fresh_observation()
                        if next_observation and next_observation['stamp'] > observation['stamp']+.15:break
                        sleep(.05)
                    if (canceled() or next_observation is None or not next_observation['model'].score(
                            next_observation['points'], candidate, self.settings).acceptable(self.settings, recovery=True)):
                        result.update(accepted=False, reason='candidate_failed_new_scan_verification')
                    else:
                        message = PoseWithCovarianceStamped()
                        message.header.frame_id = 'map'
                        message.header.stamp = self.get_clock().now().to_msg()
                        message.pose.pose.position.x, message.pose.pose.position.y = result['pose'][:2]
                        message.pose.pose.orientation.z = math.sin(candidate[2]/2)
                        message.pose.pose.orientation.w = math.cos(candidate[2]/2)
                        message.pose.covariance[0] = message.pose.covariance[7] = self.options['initial_position_std_m']**2
                        message.pose.covariance[35] = self.options['initial_yaw_std_rad']**2
                        applied_stamp = stamp(message.header)
                        self.own_initial_stamp = applied_stamp
                        self.initial.publish(message)
                        self._phase('verifying_amcl')
                        good_since = None
                        result.update(accepted=False, reason='amcl_verification_failed')
                        while not canceled():
                            current = self._fresh_observation()
                            with self.lock:state, pose_msg = self.state, self.pose_message
                            aligned = (current is not None and current['stamp'] > applied_stamp and pose_msg is not None and
                                       stamp(pose_msg.header) > applied_stamp and state == 'available' and
                                       np.linalg.norm(current['pose'][:2]-candidate[:2]) < .2 and
                                       abs(wrap(current['pose'][2]-candidate[2])) < .15)
                            if aligned:
                                if good_since is None:good_since = monotonic()
                                if monotonic()-good_since >= self.options['verification_duration_s']:
                                    result.update(accepted=True, reason='recovered_and_verified');break
                            else:good_since = None
                            sleep(.05)
                elif result['accepted']:
                    result.update(accepted=False, reason='stop_or_map_changed')
            if observation is not None:
                self._save(handle.request.job_id, observation, result)
            if handle.is_cancel_requested:
                handle.canceled();result.update(accepted=False, reason='canceled')
            elif result['accepted']:handle.succeed()
            else:handle.abort()
            self._phase('recovered' if result['accepted'] else 'failed', result['reason'])
            return RecoverComponent.Result(success=result['accepted'], detail=result['reason'])
        except Exception as error:
            handle.abort();self._phase('failed', str(error))
            self.get_logger().error(f'Localization recovery failed: {error}')
            return RecoverComponent.Result(success=False, detail=str(error))
        finally:
            self.busy.release()


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationGuard()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
