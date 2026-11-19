"""D-HUB 좌표계에서 로컬 ROLY 의자를 포함한 18층 전체 시설을 실행합니다."""
from cleany_gazebo_sim.launch_helpers.study_cafe import study_cafe_launch_description


def generate_launch_description():
    return study_cafe_launch_description(facility=True)
