"""Shared selector runtime policy. Native launches never require Docker."""
from __future__ import annotations

import platform
from pathlib import Path
import shlex


def resolve_motion_runtime(backend: str, requested: str = 'auto', *,
                           machine: str | None = None,
                           jetson_release: bool | None = None) -> str:
    if backend not in ('moveit', 'curobo', 'curobo_ik_moveit'):
        raise ValueError(f'Unknown motion backend: {backend}')
    if requested not in ('auto', 'native', 'container'):
        raise ValueError(f'Unknown motion runtime: {requested}')
    if requested != 'auto':
        return requested
    if backend == 'moveit':
        return 'native'
    machine = platform.machine() if machine is None else machine
    jetson_release = Path('/etc/nv_tegra_release').is_file() if jetson_release is None else jetson_release
    return 'container' if machine == 'aarch64' and jetson_release else 'native'


def selector_prefix(backend: str, requested: str, root: Path, python: str) -> str:
    runtime = resolve_motion_runtime(backend, requested)
    if runtime == 'container':
        return shlex.quote(str(root / 'tools/launch-motion-container'))
    return shlex.quote(python) if backend != 'moveit' else ''


def motion_launch_arguments():
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

    return [
        DeclareLaunchArgument('motion_runtime', default_value='auto',
                              choices=['auto', 'native', 'container']),
        DeclareLaunchArgument('curobo_python', default_value=PathJoinSubstitution([
            LaunchConfiguration('repository_root'), 'runtime/curobo-venv/bin/python'])),
    ]


# Keep ROS imports optional so policy tests run in an ordinary PC/VM Python.
def motion_runtime_prefix():
    from launch import Substitution
    from launch.substitutions import LaunchConfiguration

    class MotionPrefix(Substitution):
        def perform(self, context) -> str:
            value = lambda key: LaunchConfiguration(key).perform(context)
            return selector_prefix(value('motion_backend'), value('motion_runtime'),
                                   Path(value('repository_root')).expanduser().resolve(),
                                   value('curobo_python'))

    return MotionPrefix()


def selector_exit_handler(selector):
    from launch.actions import EmitEvent, LogInfo, RegisterEventHandler
    from launch.event_handlers import OnProcessExit
    from launch.events import Shutdown

    return RegisterEventHandler(OnProcessExit(target_action=selector,
        on_exit=[LogInfo(msg='Motion selector exited; shutting down launch'),
                 EmitEvent(event=Shutdown(reason='Motion selector exited'))]))
