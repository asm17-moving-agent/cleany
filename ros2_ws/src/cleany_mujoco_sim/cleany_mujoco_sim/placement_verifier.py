"""Simulation-only outcome oracle, deliberately separate from perception."""
from collections import defaultdict, deque
from pathlib import Path
import time
from uuid import uuid4

from ament_index_python.packages import get_package_share_directory
from cleany_interfaces.srv import RegisterPlacementTarget, VerifyPlacement
import numpy as np
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray

from cleany_mujoco_sim.sorting_scene import load_bins
from cleany_mujoco_sim.placement_identity import BodyGeometry, associate_body


DEFAULT_LABEL_BODIES = {
    'cup': 'study_cafe_cup',
    'mouse': 'study_cafe_mouse',
    'computer mouse': 'study_cafe_mouse',
    'wireless mouse': 'study_cafe_mouse',
    'crumpled tissue': 'study_cafe_tissue',
    'lego brick': 'study_cafe_lego',
}


class PlacementVerifier(Node):
    def __init__(self):
        super().__init__('simulation_placement_verifier')
        default = (Path(get_package_share_directory('cleany_mujoco_sim'))
                   / 'config' / 'robot_top_bins.yaml')
        self.declare_parameter('bins_config', str(default))
        self.declare_parameter('minimum_samples', 5)
        self.declare_parameter('maximum_age_sec', 1.0)
        self.declare_parameter('maximum_drift_m', 0.003)
        self.declare_parameter('settled_duration_sec', 0.4)
        self.declare_parameter('labels', list(DEFAULT_LABEL_BODIES))
        self.declare_parameter('bodies', list(DEFAULT_LABEL_BODIES.values()))
        labels = self.get_parameter('labels').value
        bodies = self.get_parameter('bodies').value
        if len(labels) != len(bodies) or len(set(zip(labels, bodies))) != len(labels):
            raise ValueError('Verifier requires distinct label/body pairs')
        self._body_candidates = defaultdict(set)
        for label, body in zip(labels, bodies):
            self._body_candidates[label].add(body)
        # Label-only compatibility is allowed only for a single configured body.
        self._bodies = {label: next(iter(values)) for label, values in self._body_candidates.items()
                        if len(values) == 1}
        self.declare_parameter('registration_maximum_distance_m', 0.05)
        self.declare_parameter('registration_ambiguity_margin_m', 0.01)
        self.declare_parameter('registration_maximum_size_error_m', 0.08)
        self.declare_parameter('registration_observation_maximum_age_sec', 120.0)
        self.declare_parameter('registration_ttl_sec', 600.0)
        self.declare_parameter('registration_maximum_entries', 128)
        self._registrations = {}
        self._execution_registrations = {}
        self._bins = {b.name: b for b in load_bins(
            self.get_parameter('bins_config').value)}
        self._samples = defaultdict(lambda: deque(maxlen=30))
        self.create_subscription(MarkerArray,
                                 '/simulation/sorting_ground_truth',
                                 self._observe, 10)
        self.create_service(RegisterPlacementTarget, '/sorting/register_placement_target',
                            self._register)
        self.create_service(VerifyPlacement, '/sorting/verify_placement',
                            self._verify)

    def _observe(self, message):
        for marker in message.markers:
            if marker.header.frame_id != 'base_link':
                continue
            stamp = (marker.header.stamp.sec * 1_000_000_000
                     + marker.header.stamp.nanosec)
            p = marker.pose.position
            s = marker.scale
            samples = self._samples[marker.text]
            if samples and stamp < samples[-1][0]:
                self._samples.clear()
                self._registrations.clear()
                self._execution_registrations.clear()
                samples = self._samples[marker.text]
            if not samples or stamp > samples[-1][0]:
                samples.append((stamp, (p.x, p.y, p.z), (s.x, s.y, s.z)))

    def _register(self, request, response):
        response.message = 'Invalid placement registration'
        now = self.get_clock().now().nanoseconds
        stamp = request.header.stamp.sec * 10**9 + request.header.stamp.nanosec
        obj = request.target
        center, size = obj.obb_pose.position, obj.obb_size
        position, extents = (center.x, center.y, center.z), (size.x, size.y, size.z)
        signature = (request.destination_id, obj.label, obj.object_id, obj.track_id, stamp, position, extents)
        wall = time.monotonic()
        ttl = float(self.get_parameter('registration_ttl_sec').value)
        expired = [key for key, value in self._registrations.items() if wall - value[3] >= ttl]
        for key in expired:
            execution = self._registrations.pop(key)[4]
            self._execution_registrations.pop(execution, None)
        existing = self._execution_registrations.get(request.execution_id)
        if existing is not None:
            entry = self._registrations[existing]
            if signature != entry[2]:
                response.message = 'Execution ID already registered to another observation'
                return response
            response.success, response.verification_id = True, existing
            response.message = 'Existing individual registration'
            return response
        if (not request.execution_id or request.destination_id not in self._bins
                or request.header.frame_id != 'base_link' or obj.object_id <= 0
                or stamp <= 0 or not 0 <= (now - stamp) / 1e9 <= float(
                    self.get_parameter('registration_observation_maximum_age_sec').value)):
            return response
        candidates = []
        for body in self._body_candidates.get(obj.label, ()):
            samples = self._samples[body]
            if not samples:
                continue
            sample_stamp, point, dimensions = samples[-1]
            if not 0 <= (now-sample_stamp)/1e9 <= float(self.get_parameter('maximum_age_sec').value):
                continue
            candidates.append(BodyGeometry(body, point, dimensions))
        try:
            body = associate_body(position, extents, candidates,
                maximum_distance_m=float(self.get_parameter('registration_maximum_distance_m').value),
                ambiguity_margin_m=float(self.get_parameter('registration_ambiguity_margin_m').value),
                maximum_size_error_m=float(self.get_parameter('registration_maximum_size_error_m').value))
        except ValueError as error:
            response.message = str(error)
            return response
        if len(self._registrations) >= int(self.get_parameter('registration_maximum_entries').value):
            response.message = 'Placement registration capacity reached'
            return response
        key = uuid4().hex
        self._registrations[key] = (body, request.destination_id, signature, wall, request.execution_id)
        self._execution_registrations[request.execution_id] = key
        response.success, response.verification_id = True, key
        # Simulator identity is private to this evaluator.
        response.message = 'Individual placement target registered'
        return response

    def _verify(self, request, response):
        body = self._bodies.get(request.label)
        if request.verification_id:
            registration = self._registrations.get(request.verification_id)
            if (registration is None or registration[1] != request.destination_id
                    or time.monotonic() - registration[3] >= float(self.get_parameter('registration_ttl_sec').value)):
                response.message = 'Unknown, expired or mismatched individual registration'
                return response
            body = registration[0]
        bin_ = self._bins.get(request.destination_id)
        response.message = 'Awaiting fresh, settled placement evidence'
        if body is None or bin_ is None or request.after_stamp_ns <= 0:
            response.message = 'Invalid verification request'
            return response
        required = int(self.get_parameter('minimum_samples').value)
        samples = list(self._samples[body])[-required:]
        if len(samples) < required or samples[0][0] <= request.after_stamp_ns:
            return response
        now = self.get_clock().now().nanoseconds
        age = (now - samples[-1][0]) / 1e9
        if not 0 <= age <= self.get_parameter('maximum_age_sec').value:
            return response
        if (samples[-1][0] - samples[0][0]) / 1e9 < (
            self.get_parameter('settled_duration_sec').value
        ):
            return response
        points = np.asarray([point for _, point, _ in samples])
        sizes = np.asarray([size for _, _, size in samples])
        low, high = points - sizes / 2, points + sizes / 2
        inside_low = np.array((
            bin_.center_xy[0] - bin_.outside_size[0] / 2 + bin_.wall,
            bin_.center_xy[1] - bin_.outside_size[1] / 2 + bin_.wall,
            bin_.bottom_z + bin_.wall,
        ))
        inside_high = np.array((
            bin_.center_xy[0] + bin_.outside_size[0] / 2 - bin_.wall,
            bin_.center_xy[1] + bin_.outside_size[1] / 2 - bin_.wall,
            bin_.top_z,
        ))
        # Small contact penetration is expected from soft contact physics.
        if (not np.isfinite(sizes).all() or np.any(sizes <= 0)
                or not np.all(low >= inside_low - 0.002)
                or not np.all(high <= inside_high + 0.002)):
            response.message = f'{request.label} is not inside {bin_.name}'
            return response
        if np.linalg.norm(np.ptp(points, axis=0)) > (
            self.get_parameter('maximum_drift_m').value
        ):
            return response
        response.success = True
        response.message = (
            f'SIMULATION VERIFIED individual in {bin_.name}: '
            f'center={points[-1].round(4).tolist()}'
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PlacementVerifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
