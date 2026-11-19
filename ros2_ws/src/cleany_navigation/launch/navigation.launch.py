"""Cleany navigation using external robot sensor/TF backends."""
from cleany_navigation.launch_runtime import navigation_launch_description

def generate_launch_description():
    return navigation_launch_description(safety=True)
