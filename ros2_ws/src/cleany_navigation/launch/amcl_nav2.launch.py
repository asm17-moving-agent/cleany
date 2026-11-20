"""외부 로봇 센서 및 TF 백엔드를 사용하는 Cleany 주행입니다."""
from cleany_navigation.launch_runtime import navigation_launch_description

def generate_launch_description():
    return navigation_launch_description(safety=False)
