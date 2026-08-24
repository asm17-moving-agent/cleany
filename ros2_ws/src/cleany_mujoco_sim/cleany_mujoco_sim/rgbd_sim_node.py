from __future__ import annotations

import rclpy
from rclpy.parameter import Parameter

from cleany_mujoco_sim.rgbd import (
    GroundTruthSpec,
    RgbdSensorBridge,
    RgbdSensorConfig,
)
from cleany_mujoco_sim.sim_node import MujocoSimNode


class MujocoRgbdSimNode(MujocoSimNode):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self._rgbd_bridge: RgbdSensorBridge | None = None

        self.declare_parameter('rgbd_width', 640)
        self.declare_parameter('rgbd_height', 480)
        self.declare_parameter('rgbd_rate_hz', 5.0)
        self.declare_parameter('rgbd_camera_name', 'head_realsense_rgb')
        self.declare_parameter(
            'rgbd_color_frame_id',
            'head_camera_rgb_optical_frame',
        )
        self.declare_parameter(
            'rgbd_depth_frame_id',
            'head_camera_depth_optical_frame',
        )
        self.declare_parameter(
            'rgbd_color_image_topic',
            'camera/color/image_raw',
        )
        self.declare_parameter(
            'rgbd_color_info_topic',
            'camera/color/camera_info',
        )
        self.declare_parameter(
            'rgbd_depth_image_topic',
            'camera/depth/image_raw',
        )
        self.declare_parameter(
            'rgbd_depth_info_topic',
            'camera/depth/camera_info',
        )
        self.declare_parameter('ground_truth_topic', 'ground_truth/objects')
        self.declare_parameter(
            'ground_truth_geom_names',
            Parameter.Type.STRING_ARRAY,
        )
        self.declare_parameter(
            'ground_truth_labels',
            Parameter.Type.STRING_ARRAY,
        )

        geom_names = list(
            self.get_parameter_or(
                'ground_truth_geom_names',
                Parameter(
                    'ground_truth_geom_names',
                    Parameter.Type.STRING_ARRAY,
                    [],
                ),
            ).value
        )
        labels = list(
            self.get_parameter_or(
                'ground_truth_labels',
                Parameter(
                    'ground_truth_labels',
                    Parameter.Type.STRING_ARRAY,
                    [],
                ),
            ).value
        )
        if len(geom_names) != len(labels):
            self.destroy_node()
            raise ValueError(
                'ground_truth_geom_names and ground_truth_labels must have '
                'the same length'
            )

        config = RgbdSensorConfig(
            width=self.get_parameter('rgbd_width').value,
            height=self.get_parameter('rgbd_height').value,
            rate_hz=self.get_parameter('rgbd_rate_hz').value,
            camera_name=self.get_parameter('rgbd_camera_name').value,
            base_body_name=self.get_parameter('base_body_name').value,
            base_frame_id=self.get_parameter('base_frame_id').value,
            color_frame_id=self.get_parameter('rgbd_color_frame_id').value,
            depth_frame_id=self.get_parameter('rgbd_depth_frame_id').value,
            color_image_topic=self.get_parameter(
                'rgbd_color_image_topic'
            ).value,
            color_info_topic=self.get_parameter(
                'rgbd_color_info_topic'
            ).value,
            depth_image_topic=self.get_parameter(
                'rgbd_depth_image_topic'
            ).value,
            depth_info_topic=self.get_parameter(
                'rgbd_depth_info_topic'
            ).value,
            ground_truth_topic=self.get_parameter('ground_truth_topic').value,
        )
        specs = tuple(
            GroundTruthSpec(geom_name=geom_name, label=label)
            for geom_name, label in zip(geom_names, labels)
        )
        try:
            self._rgbd_bridge = RgbdSensorBridge(
                self,
                self.simulation_context,
                config,
                specs,
            )
        except Exception:
            self.destroy_node()
            raise
        self.add_step_observer(self._rgbd_bridge)

    def destroy_node(self) -> None:
        if self._rgbd_bridge is not None:
            self._rgbd_bridge.close()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MujocoRgbdSimNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
