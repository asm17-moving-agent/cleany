#!/usr/bin/env python3
"""Render a fresh, axis-aligned top view of the study-cafe Gazebo world."""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import time
from pathlib import Path
from xml.etree import ElementTree

import rclpy
from ament_index_python.packages import get_package_share_directory
from PIL import Image
from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMessage

from cleany_gazebo_sim.world_generator import materialize_study_cafe_world


def add_top_camera(world_path: Path) -> None:
    tree = ElementTree.parse(world_path)
    world = tree.getroot().find('world')
    if world is None:
        raise RuntimeError('generated SDF has no world')
    model = ElementTree.SubElement(world, 'model', {'name': 'evaluation_top_camera'})
    ElementTree.SubElement(model, 'static').text = 'true'
    # Gazebo cameras look along local +X. Positive pitch rotates +X toward -Z;
    # yaw=+90 deg places image-up along world +Y and image-right along world +X.
    ElementTree.SubElement(model, 'pose').text = (
        '0 0 18.0 0 1.57079632679 1.57079632679'
    )
    link = ElementTree.SubElement(model, 'link', {'name': 'camera_link'})
    sensor = ElementTree.SubElement(
        link, 'sensor', {'name': 'top_camera', 'type': 'camera'}
    )
    ElementTree.SubElement(sensor, 'always_on').text = 'true'
    ElementTree.SubElement(sensor, 'update_rate').text = '2'
    ElementTree.SubElement(sensor, 'topic').text = '/cleany_top_view/image'
    ElementTree.SubElement(sensor, 'visualize').text = 'false'
    camera = ElementTree.SubElement(sensor, 'camera')
    ElementTree.SubElement(camera, 'horizontal_fov').text = '0.72'
    image = ElementTree.SubElement(camera, 'image')
    ElementTree.SubElement(image, 'width').text = '1600'
    ElementTree.SubElement(image, 'height').text = '1430'
    ElementTree.SubElement(image, 'format').text = 'R8G8B8'
    clip = ElementTree.SubElement(camera, 'clip')
    ElementTree.SubElement(clip, 'near').text = '0.1'
    ElementTree.SubElement(clip, 'far').text = '30.0'
    tree.write(world_path, encoding='unicode', xml_declaration=True)


class ImageCapture(Node):
    def __init__(self) -> None:
        super().__init__('cleany_top_view_capture')
        self.image: ImageMessage | None = None
        self.create_subscription(
            ImageMessage, '/cleany_top_view/image', self._receive, 1
        )

    def _receive(self, message: ImageMessage) -> None:
        self.image = message


def stop(process: subprocess.Popen[bytes]) -> None:
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=3)


def save_image(message: ImageMessage, output: Path) -> None:
    encoding = message.encoding.lower()
    if encoding not in {'rgb8', 'bgr8'}:
        raise RuntimeError(f'unsupported Gazebo image encoding: {encoding}')
    image = Image.frombytes(
        'RGB',
        (message.width, message.height),
        bytes(message.data),
        'raw',
        'RGB' if encoding == 'rgb8' else 'BGR',
        message.step,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--output',
        type=Path,
        default=Path(
            'ros2_ws/slam_results/algorithm_comparison/'
            'gazebo_top_view_reference.png'
        ),
    )
    parser.add_argument('--timeout', type=float, default=45.0)
    args = parser.parse_args()

    package_share = Path(get_package_share_directory('cleany_gazebo_sim'))
    description_share = Path(get_package_share_directory('cleany_description'))
    world_path = args.output.parent / 'gazebo_top_view_world.sdf'
    materialize_study_cafe_world(
        package_share / 'worlds/cleany_mecanum_harmonic.sdf',
        world_path,
        simulator='harmonic',
    )
    add_top_camera(world_path)

    environment = os.environ.copy()
    resource_path = environment.get('GZ_SIM_RESOURCE_PATH', '')
    environment['GZ_SIM_RESOURCE_PATH'] = os.pathsep.join(
        value for value in (resource_path, str(description_share)) if value
    )
    gazebo = subprocess.Popen(
        [
            'gz', 'sim', '-r', '-s', '--headless-rendering',
            '--render-engine-server', 'ogre2', str(world_path),
        ],
        env=environment,
        start_new_session=True,
    )
    bridge = subprocess.Popen(
        [
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/cleany_top_view/image@sensor_msgs/msg/Image@gz.msgs.Image',
        ],
        env=environment,
        start_new_session=True,
    )
    try:
        rclpy.init()
        node = ImageCapture()
        deadline = time.monotonic() + args.timeout
        while node.image is None and time.monotonic() < deadline:
            if gazebo.poll() is not None:
                raise RuntimeError(f'Gazebo exited with status {gazebo.returncode}')
            rclpy.spin_once(node, timeout_sec=0.25)
        if node.image is None:
            raise TimeoutError('timed out waiting for the Gazebo top camera')
        save_image(node.image, args.output)
        node.destroy_node()
        rclpy.shutdown()
        print(args.output)
    finally:
        stop(bridge)
        stop(gazebo)


if __name__ == '__main__':
    main()
