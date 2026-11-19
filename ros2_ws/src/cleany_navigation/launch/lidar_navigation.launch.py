"""LiDAR 기반 주행은 기본적으로 Smac Lattice와 MPPI Omni를 사용합니다."""
from cleany_navigation.launch_runtime import navigation_launch_description


def generate_launch_description():
    return navigation_launch_description(safety=False, motion_profile='omni', lidar_only=True)
