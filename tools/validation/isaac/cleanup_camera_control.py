"""ROS camera-rate adapter for the existing sorting coordinator."""
from rcl_interfaces.msg import SetParametersResult


def create_camera_control(rclpy):
    node = rclpy.create_node('sorting_cameras')
    node.declare_parameter('active_camera', 'head')
    node.declare_parameter('head_depth_boost', False)
    node.declare_parameter('head_high_rate', False)

    def validate(parameters):
        for parameter in parameters:
            if parameter.name == 'active_camera' and parameter.value not in ('head', 'left', 'right'):
                return SetParametersResult(successful=False, reason='Expected head, left or right camera')
        return SetParametersResult(successful=True)
    node.add_on_set_parameters_callback(validate)
    return node
