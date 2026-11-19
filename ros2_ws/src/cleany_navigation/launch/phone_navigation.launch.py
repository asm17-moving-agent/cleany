"""이전 launch 이름을 LiDAR 기반 주행으로 연결하는 호환 진입점입니다."""
from cleany_navigation.launch_runtime import navigation_launch_description


def generate_launch_description():
    return navigation_launch_description(safety=False, motion_profile='omni', lidar_only=True)
