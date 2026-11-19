"""Compatibility launch using the common AMCL/Nav2 implementation."""
from pathlib import Path
from tempfile import mkdtemp
import yaml
from cleany_navigation.runtime_config import load_navigation_profile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _launch(context):
    navigation = Path(get_package_share_directory('cleany_navigation'))
    overlay = Path(LaunchConfiguration('params_file').perform(context))
    parameters = load_navigation_profile(navigation/'config/nav2_amcl.yaml', overlay)
    output = Path(mkdtemp(prefix='cleany-gazebo-amcl-'))/'parameters.yaml'
    output.write_text(yaml.safe_dump(parameters, sort_keys=False))
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(navigation/'launch/amcl_nav2.launch.py')),
            launch_arguments={**{name: LaunchConfiguration(name) for name in
            ('map', 'localization_params_file', 'use_sim_time', 'autostart')},
            'params_file': str(output)}.items())]


def generate_launch_description():
    share = Path(get_package_share_directory('cleany_gazebo_sim'))
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=str(share/'maps/study_cafe_26cm.yaml')),
        DeclareLaunchArgument('params_file', default_value=str(share/'config/nav2_amcl.yaml')),
        DeclareLaunchArgument('localization_params_file', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        OpaqueFunction(function=_launch),
    ])
