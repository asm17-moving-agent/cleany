SHELL := /bin/bash

REPO_ROOT := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
ROS2_WS := $(REPO_ROOT)ros2_ws
ROS_SETUP := /opt/ros/humble/setup.bash

.PHONY: help deps build test test-mission test-mujoco sim clean

help:
	@echo "Cleany native ROS 2 commands"
	@echo "  make deps          Install workspace dependencies with rosdep"
	@echo "  make build         Build the ROS 2 workspace"
	@echo "  make test          Build and run all colcon tests"
	@echo "  make test-mission  Run Mission Manager pytest"
	@echo "  make test-mujoco   Run MuJoCo simulation pytest"
	@echo "  make sim           Build and run the headless MuJoCo simulation"
	@echo "  make clean         Remove ROS 2 build, install, and log outputs"

deps:
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	rosdep install --from-paths src --ignore-src -r -y

build:
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	colcon build --symlink-install

test: build
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source install/setup.bash && \
	colcon test && \
	colcon test-result --verbose

test-mission: build
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source install/setup.bash && \
	python3 -m pytest src/cleany_mission_manager/tests/test_mission_flow.py

test-mujoco: build
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source install/setup.bash && \
	python3 -m pytest src/cleany_mujoco_sim/test/test_scene_loader.py

sim: build
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source install/setup.bash && \
	ros2 launch cleany_mujoco_sim mujoco_sim.launch.py headless:=true

clean:
	"$(REPO_ROOT)tools/ros2-clean"
