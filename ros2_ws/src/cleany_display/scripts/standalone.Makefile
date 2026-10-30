SHELL := /bin/bash
.PHONY: build test
build:
	cd ros2_ws && source /opt/ros/humble/setup.bash && CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --base-paths src --packages-select cleany_interfaces cleany_display --parallel-workers 2 --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
test:
	PYTHONPATH=ros2_ws/src/cleany_display python3 -m pytest ros2_ws/src/cleany_display/test -q
