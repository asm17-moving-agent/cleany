SHELL := /bin/bash

REPO_ROOT := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
ROS2_WS := $(REPO_ROOT)ros2_ws
ROS_SETUP := /opt/ros/humble/setup.bash
HARMONIC_ROS_SETUP ?= /opt/ros/jazzy/setup.bash
HARMONIC_BUILD_BASE ?= build-harmonic
HARMONIC_INSTALL_BASE ?= install-harmonic
HARMONIC_LOG_BASE ?= log-harmonic

.PHONY: help deps build build-gazebo-harmonic test test-mission test-mujoco \
	test-gazebo test-gazebo-harmonic sim sim-gazebo sim-gazebo-harmonic clean

help:
	@echo "Cleany native ROS 2 commands"
	@echo "  make deps          Install workspace dependencies with rosdep"
	@echo "  make build         Build the ROS 2 workspace"
	@echo "  make test          Build and run all colcon tests"
	@echo "  make test-mission  Run Mission Manager pytest"
	@echo "  make test-mujoco   Run MuJoCo simulation pytest"
	@echo "  make test-gazebo   Run Gazebo simulation pytest"
	@echo "  make test-gazebo-harmonic  Run the isolated Harmonic profile pytest"
	@echo "  make sim           Build and run the headless MuJoCo simulation"
	@echo "  make sim-gazebo    Build and run the headless Gazebo simulation"
	@echo "  make sim-gazebo-harmonic  Build and run the isolated Harmonic profile"
	@echo "  make clean         Remove ROS 2 build, install, and log outputs"

deps:
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	rosdep install --from-paths src --ignore-src -r -y

build:
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	colcon build --symlink-install

build-gazebo-harmonic:
	source "$(HARMONIC_ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	colcon --log-base "$(HARMONIC_LOG_BASE)" build --symlink-install \
		--build-base "$(HARMONIC_BUILD_BASE)" \
		--install-base "$(HARMONIC_INSTALL_BASE)" \
		--packages-up-to cleany_gazebo_sim

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

test-gazebo: build
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source install/setup.bash && \
	python3 -m pytest src/cleany_gazebo_sim/test

test-gazebo-harmonic: build-gazebo-harmonic
	source "$(HARMONIC_ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source "$(HARMONIC_INSTALL_BASE)/setup.bash" && \
	python3 -m pytest src/cleany_gazebo_sim/test/test_harmonic_profile.py

sim: build
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source install/setup.bash && \
	ros2 launch cleany_mujoco_sim mujoco_sim.launch.py headless:=true

sim-gazebo: build
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source install/setup.bash && \
	ros2 launch cleany_gazebo_sim gazebo_sim.launch.py headless:=true

sim-gazebo-harmonic: build-gazebo-harmonic
	source "$(HARMONIC_ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source "$(HARMONIC_INSTALL_BASE)/setup.bash" && \
	ros2 launch cleany_gazebo_sim gazebo_harmonic.launch.py headless:=true

clean:
	"$(REPO_ROOT)tools/ros2-clean"
