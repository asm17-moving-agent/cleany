"""Simulation-only outcome oracle, deliberately separate from perception."""
from collections import defaultdict, deque
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from cleany_interfaces.srv import VerifyPlacement
import numpy as np
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray

from cleany_mujoco_sim.sorting_scene import load_bins


DEFAULT_LABEL_BODIES = {
    'cup': 'study_cafe_cup',
    'wallet': 'study_cafe_wallet',
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
        if len(labels) != len(bodies) or len(set(labels)) != len(labels):
            raise ValueError('Verifier requires unique label/body mapping')
        self._bodies = dict(zip(labels, bodies, strict=True))
        self._bins = {b.name: b for b in load_bins(
            self.get_parameter('bins_config').value, include_staging=True)}
        self._samples = defaultdict(lambda: deque(maxlen=30))
        self.create_subscription(MarkerArray,
                                 '/simulation/sorting_ground_truth',
                                 self._observe, 10)
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
                samples.clear()
            if not samples or stamp > samples[-1][0]:
                samples.append((stamp, (p.x, p.y, p.z), (s.x, s.y, s.z)))

    def _verify(self, request, response):
        body = self._bodies.get(request.label)
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
        if bin_.kind == 'table_zone':
            inside_high[2] = bin_.bottom_z + bin_.outside_size[2]
            if not np.all(np.abs(low[:, 2] - bin_.bottom_z) <= 0.005):
                response.message = 'Object is not resting on the table collection zone'
                return response
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
            f'SIMULATION VERIFIED {body} in {bin_.name}: '
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
