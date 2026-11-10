"""핸드폰 주행의 기본 동작은 전후 이동과 회전입니다."""
from cleany_navigation.launch_runtime import navigation_launch_description


def generate_launch_description():
    return navigation_launch_description(safety=False, phone_motion_profile='longitudinal', lidar_only=True)
