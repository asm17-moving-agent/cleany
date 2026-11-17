"""Publish calibrated camera poses from a frozen, read-only kinematic model."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)


def origin(element: ET.Element | None) -> np.ndarray:
    result = np.eye(4)
    if element is not None:
        result[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
        rpy = np.fromstring(element.get('rpy', '0 0 0'), sep=' ')
        for axis, angle in reversed(list(zip(np.eye(3), rpy))):
            result[:3, :3] = result[:3, :3] @ rotation(axis, angle)
    return result


class CameraPoseModel:
    """Evaluate base-to-optical poses without opening a device or commanding a joint.

    Affine joint conversions map the operational JointState coordinates into the
    exact coordinates used during camera fitting. They do not change controllers.
    """

    def __init__(self, path: Path):
        self.path = path.resolve()
        self.config = json.loads(path.read_text())
        self.sha256 = hashlib.sha256(path.read_bytes()).hexdigest()
        for source, expected in self.config.get('source_dependencies_sha256', {}).items():
            dependency = self.path.parent / Path(source).expanduser()
            if hashlib.sha256(dependency.read_bytes()).hexdigest() != expected:
                raise ValueError('Camera joint mapping or motor calibration changed: ' + source)
        model = Path(self.config['model_file']).expanduser()
        if not model.is_absolute():
            model = path.parent / model
        if hashlib.sha256(model.read_bytes()).hexdigest() != self.config['model_sha256']:
            raise ValueError('Camera kinematic model hash mismatch')
        root = ET.parse(model).getroot()
        self.incoming = {j.find('child').get('link'): j for j in root.findall('joint')}
        self.chains = {}
        for camera, cfg in self.config['cameras'].items():
            chain, child = [], cfg['parent_frame']
            while child != 'base_link':
                joint = self.incoming[child]
                chain.append(joint)
                child = joint.find('parent').get('link')
            self.chains[camera] = list(reversed(chain))
            matrix = np.asarray(cfg['parent_T_camera'], dtype=float)
            if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
                    or not np.allclose(matrix[3], [0, 0, 0, 1])
                    or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-8)
                    or not math.isclose(np.linalg.det(matrix[:3, :3]), 1., abs_tol=1e-8)):
                raise ValueError('Invalid camera mount transform')

    def poses(self, positions: dict[str, float]) -> dict[str, np.ndarray]:
        result = {}
        for camera, chain in self.chains.items():
            matrix = np.eye(4)
            try:
                for joint in chain:
                    matrix = matrix @ origin(joint.find('origin'))
                    if joint.get('type') != 'fixed':
                        cfg = self.config['joint_coordinates'][joint.get('name')]
                        q = positions[cfg['source_joint']] * cfg['scale'] + cfg['offset_rad']
                        if not math.isfinite(q):
                            raise ValueError('Nonfinite camera joint coordinate')
                        turn = np.eye(4)
                        turn[:3, :3] = rotation(np.fromstring(joint.find('axis').get('xyz'), sep=' '), q)
                        matrix = matrix @ turn
            except KeyError:
                continue
            result[camera] = matrix @ np.array(self.config['cameras'][camera]['parent_T_camera'])
        return result


def remove_camera_frames(model: str, frames: set[str]) -> str:
    """Give the optical TF branches to the calibrated pose publisher."""
    root = ET.fromstring(model)
    removed = set(frames)
    changed = True
    while changed:
        previous = len(removed)
        for joint in root.findall('joint'):
            if joint.find('parent').get('link') in removed:
                removed.add(joint.find('child').get('link'))
        changed = len(removed) != previous
    for joint in root.findall('joint'):
        if joint.find('child').get('link') in removed:
            if joint.get('type') != 'fixed':
                raise ValueError('Cannot detach a movable robot joint')
            root.remove(joint)
    for link in root.findall('link'):
        if link.get('name') in removed:
            if link.find('visual') is not None or link.find('collision') is not None:
                raise ValueError('Camera optical frame contains robot geometry')
            root.remove(link)
    return ET.tostring(root, encoding='unicode')


def main(args=None):
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from tf2_ros import TransformBroadcaster

    class Publisher(Node):
        """Subscribe to /joint_states and publish the calibrated optical TF branches."""

        def __init__(self):
            super().__init__('calibrated_camera_pose')
            self.declare_parameter('config_file', '')
            self.model = CameraPoseModel(Path(self.get_parameter('config_file').value))
            self.tf = TransformBroadcaster(self)
            self.status = self.create_publisher(String, '/camera_calibration/pose_status', 1)
            self.create_subscription(JointState, '/joint_states', self.update, qos_profile_sensor_data)
            self.get_logger().info(f'Camera pose configuration: {self.model.path}')

        def update(self, msg):
            if len(msg.name) != len(msg.position):
                return
            stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
            if not 0 <= age < .5:
                return
            poses = self.model.poses(dict(zip(msg.name, msg.position)))
            messages = []
            for camera, matrix in poses.items():
                cfg = self.model.config['cameras'][camera]
                # A symmetric eigenproblem avoids a scipy dependency near pi rotations.
                r = matrix[:3, :3]
                k = np.array([[r[0,0]-r[1,1]-r[2,2], r[0,1]+r[1,0], r[0,2]+r[2,0], r[2,1]-r[1,2]],
                              [r[0,1]+r[1,0], r[1,1]-r[0,0]-r[2,2], r[1,2]+r[2,1], r[0,2]-r[2,0]],
                              [r[0,2]+r[2,0], r[1,2]+r[2,1], r[2,2]-r[0,0]-r[1,1], r[1,0]-r[0,1]],
                              [r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], np.trace(r)]])
                quat = np.linalg.eigh(k)[1][:, -1]
                for frame in cfg['optical_frames']:
                    transform = TransformStamped()
                    transform.header = msg.header
                    transform.header.frame_id = 'base_link'
                    transform.child_frame_id = frame
                    p, q = transform.transform.translation, transform.transform.rotation
                    p.x, p.y, p.z = map(float, matrix[:3, 3])
                    q.x, q.y, q.z, q.w = map(float, quat)
                    messages.append(transform)
            self.tf.sendTransform(messages)
            self.status.publish(String(data=json.dumps({'config_file': str(self.model.path),
                'config_sha256': self.model.sha256,
                'cameras': list(poses), 'joint_state_age_s': age})))

    rclpy.init(args=args)
    node = Publisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
