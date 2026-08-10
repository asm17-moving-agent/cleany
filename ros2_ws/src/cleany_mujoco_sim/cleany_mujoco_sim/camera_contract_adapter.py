from __future__ import annotations

from collections import OrderedDict
from copy import deepcopy
from pathlib import Path
from typing import TypeVar

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from cleany_mujoco_sim.camera_contract import CameraContract
from cleany_mujoco_sim.scene_manifest import (
    default_manifest_path,
    load_handeye_scene_manifest,
    preflight_manifest,
)


MessageT = TypeVar('MessageT', Image, CameraInfo)
StampKey = tuple[int, int]


def _stamp_key(message: Image | CameraInfo) -> StampKey:
    return (message.header.stamp.sec, message.header.stamp.nanosec)


def _has_nonzero_stamp(message: Image | CameraInfo) -> bool:
    return _stamp_key(message) != (0, 0)


class CameraContractAdapter(Node):
    """Expose one deterministic camera contract from the vendor renderer.

    The Humble release renderer owns the source timestamp.  Images and source
    CameraInfo are paired by that exact timestamp before either public message
    is emitted, so the adapter never invents or wall-times a sample.
    """

    _MAX_PENDING_SAMPLES = 12

    def __init__(self) -> None:
        super().__init__('left_wrist_camera_contract_adapter')
        self.declare_parameter('manifest_path', str(default_manifest_path()))
        manifest_parameter = self.get_parameter(
            'manifest_path'
        ).get_parameter_value()
        manifest_path = Path(
            manifest_parameter.string_value
        )
        manifest = load_handeye_scene_manifest(manifest_path)
        preflight_manifest(manifest, profile='simulation')
        self._contract = manifest.camera_contract

        self._image_publisher = self.create_publisher(
            Image,
            self._contract.public_image_topic,
            qos_profile_sensor_data,
        )
        self._info_publisher = self.create_publisher(
            CameraInfo,
            self._contract.public_info_topic,
            qos_profile_sensor_data,
        )
        self._images: OrderedDict[StampKey, Image] = OrderedDict()
        self._infos: OrderedDict[StampKey, CameraInfo] = OrderedDict()
        self._blocked_reason: str | None = None
        self.create_subscription(
            Image,
            self._contract.internal_image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self._contract.internal_info_topic,
            self._on_info,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            'camera contract ready: '
            f'{self._contract.width}x{self._contract.height}, '
            f'frame={self._contract.frame_id}, '
            f'image={self._contract.public_image_topic}, '
            f'info={self._contract.public_info_topic}'
        )

    def _on_image(self, message: Image) -> None:
        if self._blocked_reason is not None:
            return
        if not _has_nonzero_stamp(message):
            return
        if (
            message.width != self._contract.width
            or message.height != self._contract.height
        ):
            self._block(
                'vendor image dimensions do not match the preflight contract: '
                f'{message.width}x{message.height}'
            )
            return
        self._insert(self._images, message)
        self._publish_if_complete(_stamp_key(message))

    def _on_info(self, message: CameraInfo) -> None:
        if self._blocked_reason is not None:
            return
        if not _has_nonzero_stamp(message):
            return
        if (
            message.width != self._contract.width
            or message.height != self._contract.height
        ):
            self._block(
                'vendor CameraInfo dimensions do not match the preflight '
                f'contract: {message.width}x{message.height}'
            )
            return
        self._insert(self._infos, message)
        self._publish_if_complete(_stamp_key(message))

    def _insert(
        self,
        cache: OrderedDict[StampKey, MessageT],
        message: MessageT,
    ) -> None:
        cache[_stamp_key(message)] = message
        cache.move_to_end(_stamp_key(message))
        while len(cache) > self._MAX_PENDING_SAMPLES:
            cache.popitem(last=False)

    def _publish_if_complete(self, stamp: StampKey) -> None:
        image = self._images.pop(stamp, None)
        info = self._infos.pop(stamp, None)
        if image is None or info is None:
            if image is not None:
                self._images[stamp] = image
            if info is not None:
                self._infos[stamp] = info
            return

        public_image = deepcopy(image)
        public_image.header.frame_id = self._contract.frame_id
        public_info = camera_info_for_image(public_image, self._contract)
        self._image_publisher.publish(public_image)
        self._info_publisher.publish(public_info)

    def _block(self, reason: str) -> None:
        self._blocked_reason = reason
        self._images.clear()
        self._infos.clear()
        self.get_logger().fatal(f'camera collection blocked: {reason}')


def camera_info_for_image(
    image: Image,
    contract: CameraContract,
) -> CameraInfo:
    result = CameraInfo()
    result.header = deepcopy(image.header)
    result.header.frame_id = contract.frame_id
    result.height = contract.height
    result.width = contract.width
    result.distortion_model = contract.distortion_model
    result.d = list(contract.d)
    result.k = list(contract.k)
    result.r = list(contract.r)
    result.p = list(contract.p)
    return result


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CameraContractAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
