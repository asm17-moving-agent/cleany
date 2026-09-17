"""CAD model with frozen navigation pose and Harmonic sensor/drive adapters."""
from pathlib import Path
from tempfile import mkdtemp
import json

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import yaml

from cleany_gazebo_sim.world.generator import materialize_study_cafe_world


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory('cleany_gazebo_sim'))
    directory = Path(mkdtemp(prefix='cleany-cad-launch-'))
    world = materialize_study_cafe_world(
        share/'worlds/cleany_mecanum_harmonic.sdf', target_path=directory/'world.sdf',
        lidar_translation=(0.16, 0.0, -0.08), robot_model='cad_frame')
    model = json.loads(world.with_suffix('.model.json').read_text())
    odometry = Path(get_package_share_directory('cleany_base_odometry'))
    wheel = yaml.safe_load((odometry/'config/wheel_odometry.yaml').read_text())
    for key in ('wheel_radius', 'wheelbase', 'wheel_separation'):
        wheel['wheel_odometry']['ros__parameters'][key+'_m'] = model[key]
    wheel_path = directory/'wheel_odometry.yaml'
    wheel_path.write_text(yaml.safe_dump(wheel))
    sensor = yaml.safe_load((share/'config/base.yaml').read_text())
    sensor['gazebo_sensor_tf_publisher']['ros__parameters']['lidar_translation'] = [0.16, 0.0, -0.08]
    sensor_path = directory/'sensor_tf.yaml'
    sensor_path.write_text(yaml.safe_dump(sensor))
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('sensor_profile', default_value='lidar_depth_nav'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share/'launch/gazebo_harmonic.launch.py')),
            launch_arguments=dict(world=str(world), headless=LaunchConfiguration('headless'),
                                  sensor_profile=LaunchConfiguration('sensor_profile'),
                                  sensor_config=str(sensor_path), odometry_source='wheel',
                                  wheel_odometry_config=str(wheel_path)).items()),
    ])
