"""Humble / Gazebo Fortress 스터디카페 시나리오를 실행합니다."""

from cleany_gazebo_sim.launch_helpers.study_cafe import (
    study_cafe_launch_description,
)


def generate_launch_description():
    return study_cafe_launch_description()
