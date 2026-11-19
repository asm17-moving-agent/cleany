"""Resolve motion defaults while accepting deprecated argument names."""
from __future__ import annotations


def resolve_motion_profile(profile: str | None, legacy: str | None = None, *, default: str = '') -> str:
    """Reject conflicting aliases and preserve explicit profile selections."""
    if legacy and profile is not None and profile != legacy:
        raise ValueError('motion_profile conflicts with deprecated phone_motion_profile')
    selected = profile if profile is not None else legacy or default
    if selected not in ('', 'longitudinal', 'omni'):
        raise ValueError(f'Invalid motion profile: {selected}')
    return selected


def motion_profile_arguments(default: str) -> list:
    """Resolve legacy selections before the default and validate before startup."""
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    from launch.substitutions import LaunchConfiguration, PythonExpression

    def validate(context):
        resolve_motion_profile(context.launch_configurations.get('motion_profile'),
                               context.launch_configurations.get('phone_motion_profile'), default=default)
        return []

    return [
        DeclareLaunchArgument('phone_motion_profile', default_value='',
                              choices=['', 'longitudinal', 'omni'],
                              description='Deprecated alias for motion_profile.'),
        DeclareLaunchArgument('motion_profile', choices=['', 'longitudinal', 'omni'],
                              default_value=PythonExpression([
                                  "'", LaunchConfiguration('phone_motion_profile'), "' or '", default, "'"])),
        OpaqueFunction(function=validate),
    ]
