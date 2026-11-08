"""Phone navigation defaults to longitudinal movement and rotation."""
from cleany_navigation.launch_runtime import navigation_launch_description


def generate_launch_description():
    return navigation_launch_description(safety=False, phone_motion_profile='longitudinal', lidar_only=True)
