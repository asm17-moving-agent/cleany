"""Check the independently built runtime without capturing any IK graphs."""
import json
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_share_directory
from cleany_interfaces.action import SelectReachableGrasp
import cleany_interfaces.action as interfaces
import cleany_skill_executor.physical_grasp_selection_node as selector
import curobo
import torch
import xacro

from cleany_skill_executor.curobo_model import CuRoboConfig, RobotModel

assert torch.cuda.is_available()
description = Path(get_package_share_directory('cleany_description'))
moveit = Path(get_package_share_directory('cleany_moveit_config'))
assert str(description).startswith('/opt/cleany-motion/workspace/install/')
assert str(moveit).startswith('/opt/cleany-motion/workspace/install/')
assert interfaces.__file__.startswith('/opt/cleany-motion/workspace/install/')
assert selector.__file__.startswith('/opt/cleany-motion/workspace/install/')
urdf = xacro.process_file(str(description / 'urdf/cleany.urdf.xacro'),
                          mappings={'include_head_camera': 'true'}).toxml()
srdf = xacro.process_file(str(moveit / 'config/cleany.srdf.xacro'),
                          mappings={'include_head_camera': 'true'}).toxml()
model = RobotModel(urdf, srdf, CuRoboConfig(num_seeds=16))
print(json.dumps(dict(cuda_available=True, torch=torch.__version__, cuda=torch.version.cuda,
    device=torch.cuda.get_device_name(0), curobo_file=curobo.__file__,
    curobo_commit=subprocess.check_output(['git', '-c', 'safe.directory=/opt/src/curobo',
        '-C', '/opt/src/curobo', 'rev-parse', 'HEAD'], text=True).strip(),
    description=str(description), moveit=str(moveit), collision_links=len(model.collision_links),
    ros_action=SelectReachableGrasp.__name__, interfaces_file=interfaces.__file__,
    selector_file=selector.__file__, host_install_used=False), indent=2))
