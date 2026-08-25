"""Launch the Jazzy / Gazebo Harmonic study-cafe scenario."""

from cleany_gazebo_sim.study_cafe_launch import study_cafe_launch_description


def generate_launch_description():
    return study_cafe_launch_description('harmonic')
