"""Launch the Humble / Gazebo Fortress study-cafe scenario."""

from cleany_gazebo_sim.launch_helpers.study_cafe import (
    study_cafe_launch_description,
)


def generate_launch_description():
    return study_cafe_launch_description('fortress')
