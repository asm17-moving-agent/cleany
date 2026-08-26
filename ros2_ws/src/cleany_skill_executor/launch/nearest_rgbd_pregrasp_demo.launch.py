from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_file(package: str, filename: str) -> str:
    share = Path(get_package_share_directory(package))
    return str(share / 'launch' / filename)


def generate_launch_description() -> LaunchDescription:
    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')
    use_image_view = LaunchConfiguration('use_image_view')
    detector_type = LaunchConfiguration('detector_type')
    segmenter_type = LaunchConfiguration('segmenter_type')
    gemini_model = LaunchConfiguration('gemini_model')
    sam2_model_config = LaunchConfiguration('sam2_model_config')
    sam2_checkpoint = LaunchConfiguration('sam2_checkpoint')
    sam2_device = LaunchConfiguration('sam2_device')
    mujoco_share = Path(get_package_share_directory('cleany_mujoco_sim'))
    moveit_share = Path(get_package_share_directory('cleany_moveit_config'))
    perception_share = Path(get_package_share_directory('cleany_perception'))
    grasping_share = Path(get_package_share_directory('cleany_grasping'))
    skill_share = Path(get_package_share_directory('cleany_skill_executor'))

    backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file('cleany_mujoco_sim', 'handeye_backend.launch.py')
        ),
        launch_arguments={
            'scene_path': str(
                mujoco_share / 'scenes' / 'can_grasp_execution_demo.xml.in'
            ),
            'controller_config': str(
                mujoco_share / 'config' / 'grasp_demo_ros2_controllers.yaml'
            ),
            'headless': headless,
            'sim_speed_factor': '1.0',
            'camera_name': 'pick_demo_rgbd',
            'camera_frame_name': 'pick_demo_rgbd_optical_frame',
            'enable_camera_contract_adapter': 'false',
            'enable_gripper_controllers': 'true',
        }.items(),
    )
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.140938350461',
            '--y', '-0.002',
            '--z', '0.774117299010',
            '--qx', '-0.655239668156',
            '--qy', '0.655239668156',
            '--qz', '-0.265821325847',
            '--qw', '0.265821325847',
            '--frame-id', 'base_link',
            '--child-frame-id', 'pick_demo_rgbd_optical_frame',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file('cleany_moveit_config', 'move_group.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'use_rviz': use_rviz,
            'allow_trajectory_execution': 'true',
        }.items(),
    )
    collision_scene = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file(
                'cleany_moveit_config',
                'handeye_collision_scene.launch.py',
            )
        ),
        launch_arguments={
            'scene_config': str(
                moveit_share / 'config' / 'pick_demo_collision_objects.yaml'
            )
        }.items(),
    )
    perception = Node(
        package='cleany_perception',
        executable='inspection_node',
        parameters=[
            str(perception_share / 'config' / 'inspect_scene.yaml'),
            {
                'use_sim_time': True,
                'detector_type': detector_type,
                'segmenter_type': segmenter_type,
                'gemini_model': gemini_model,
                'sam2_model_config': sam2_model_config,
                'sam2_checkpoint': sam2_checkpoint,
                'sam2_device': sam2_device,
                'color_image_topic': (
                    '/cleany/internal/mujoco/left_wrist_camera/image_raw'
                ),
                'color_info_topic': (
                    '/cleany/internal/mujoco/left_wrist_camera/camera_info'
                ),
                'depth_image_topic': (
                    '/cleany/internal/mujoco/left_wrist_camera/depth'
                ),
                'depth_info_topic': (
                    '/cleany/internal/mujoco/left_wrist_camera/camera_info'
                ),
                'target_frame': 'base_link',
                # Visible RGB-D surfaces can be nearly planar at this view.
                'minimum_obb_extent_m': 0.001,
            },
        ],
        output='screen',
    )
    grasp_server = Node(
        package='cleany_grasping',
        executable='grasp_server',
        parameters=[
            str(grasping_share / 'config' / 'anygrasp.yaml'),
            {'use_sim_time': True},
        ],
        output='screen',
    )
    selector = Node(
        package='cleany_skill_executor',
        executable='grasp_selection_server',
        parameters=[
            str(skill_share / 'config' / 'grasp_selection.yaml'),
            {'use_sim_time': True, 'pregrasp_offset_m': 0.14},
        ],
        output='screen',
    )
    coordinator = Node(
        package='cleany_skill_executor',
        executable='nearest_pregrasp_coordinator',
        parameters=[
            str(skill_share / 'config' / 'nearest_pregrasp.yaml'),
            {'use_sim_time': True},
        ],
        output='screen',
    )
    image_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        arguments=['/perception/debug_image_latched'],
        condition=IfCondition(use_image_view),
        output='log',
    )
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_image_view', default_value='true'),
        DeclareLaunchArgument(
            'detector_type', default_value='simulation_color'
        ),
        DeclareLaunchArgument(
            'segmenter_type', default_value='simulation_color'
        ),
        DeclareLaunchArgument(
            'gemini_model', default_value='gemini-robotics-er-2-preview'
        ),
        DeclareLaunchArgument(
            'sam2_model_config',
            default_value='configs/sam2.1/sam2.1_hiera_t.yaml',
        ),
        DeclareLaunchArgument(
            'sam2_checkpoint',
            default_value='/home/ubuntu/models/sam2/sam2.1_t.pt',
        ),
        DeclareLaunchArgument('sam2_device', default_value='cpu'),
        backend,
        camera_tf,
        move_group,
        collision_scene,
        perception,
        grasp_server,
        selector,
        coordinator,
        image_view,
    ])
