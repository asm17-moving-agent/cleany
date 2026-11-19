Import("env")

import importlib.util
import os
from pathlib import Path

project = Path(env.subst("$PROJECT_DIR"))
controller = project if (project / ".venv").is_dir() else project.parent
component = controller / "micro_ros/micro_ros_espidf_component"
if not component.is_dir():
    raise RuntimeError("Run `make firmware-setup` before building micro-ROS firmware")

spec = importlib.util.spec_from_file_location(
    "micro_ros_setup", controller / "scripts/micro_ros_setup.py"
)
setup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(setup)
setup.refresh_component(project, env.subst("$PIOENV"))

for variable in (
    "ROS_DISTRO", "ROS_VERSION", "ROS_PYTHON_VERSION", "AMENT_PREFIX_PATH",
    "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH", "PYTHONPATH",
):
    env["ENV"][variable] = ""
env["ENV"]["PATH"] = os.pathsep.join(
    path for path in env["ENV"].get("PATH", "").split(os.pathsep)
    if not path.startswith("/opt/ros/")
)
# ESP-IDF는 자체 Python venv를 사용하므로 고정 모듈 경로를 SCons에 제공합니다.
# PlatformIO 빌더가 PYTHONPATH를 지우기 때문에 cmake/microros_python.cmake가
# CMake 구성 단계에서도 이 경로를 복구합니다.
env["ENV"]["PYTHONPATH"] = str(controller / ".venv/lib/python3.10/site-packages")
env.PrependENVPath("PATH", str(controller / ".venv/bin"))
env.PrependENVPath("PATH", str(controller / "micro_ros/bin"))
env["ENV"]["SDKCONFIG_DEFAULTS"] = str(controller / "sdkconfig.defaults")
