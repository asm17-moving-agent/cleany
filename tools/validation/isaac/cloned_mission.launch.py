"""Run the unchanged Mission launch with environment-local ROS endpoints."""
import importlib.util
import os
from pathlib import Path
import yaml

from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import SetRemap, PushRosNamespace
from launch.substitutions import LaunchConfiguration

from ros_scope import remappings


def generate_launch_description():
    path = Path(__file__).with_name('mission.launch.py')
    spec = importlib.util.spec_from_file_location('cleany_original_mission_launch', path)
    original = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(original)
    prefix = os.environ['CLEANY_ROS_PREFIX']
    original_start = original.start

    def start(context):
        nodes = original_start(context)
        output = Path(LaunchConfiguration('output').perform(context))
        path = output/'config/navigation.yaml'
        canonical = path.read_text()
        (output/'config/navigation-unscoped.yaml').write_text(canonical)
        path.write_text(yaml.safe_dump({prefix: yaml.safe_load(canonical)}))
        return nodes

    original.start = start
    return LaunchDescription([GroupAction([
        PushRosNamespace(prefix),
        *[SetRemap(src=source, dst=destination)
          for source, destination in remappings(prefix)],
        *original.generate_launch_description().entities])])
