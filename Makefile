SHELL := /bin/bash

REPO_ROOT := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
ROS2_WS := $(REPO_ROOT)ros2_ws
ROS_SETUP := /opt/ros/humble/setup.bash
GAZEBO_PROFILE_TOOL := $(REPO_ROOT)tools/gazebo_profile.py
HANDEYE_PROFILE_DIR ?= $(REPO_ROOT)artifacts/handeye/profiles/mujoco_seed_20260810
HANDEYE_POSE_MANIFEST ?= $(HANDEYE_PROFILE_DIR)/materialized_poses.yaml
HANDEYE_RUNTIME_CONFIG ?= $(HANDEYE_PROFILE_DIR)/materialized_runtime.json
HANDEYE_ARTIFACT_ROOT ?= $(REPO_ROOT)artifacts/handeye/runs
HANDEYE_RUN_ID ?= mujoco_seed_20260810
HANDEYE_RUN_DIR ?= $(HANDEYE_ARTIFACT_ROOT)/$(HANDEYE_RUN_ID)
HANDEYE_VALIDATION_OUTPUT ?= $(HANDEYE_RUN_DIR)/dataset_validation.json
HANDEYE_MAX_TRANSLATION_NORM_M ?= 1.0
HANDEYE_DATASET_MODE ?= strict
HANDEYE_PACKAGES := cleany_description cleany_mujoco_sim \
	cleany_moveit_config cleany_handeye_calibration

.PHONY: help deps deps-gazebo check-gazebo-env build build-gazebo \
	build-gazebo-harmonic build-handeye test test-mission test-mujoco \
	test-handeye test-gazebo handeye-generate-mujoco \
	handeye-validate-mujoco \
	test-gazebo-harmonic test-gazebo-nav-runtime sim sim-gazebo \
	sim-gazebo-harmonic sim-gazebo-office sim-gazebo-study-cafe \
	handeye-mujoco clean

help:
	@echo "Cleany native ROS 2 commands"
	@echo "  make deps          Install workspace dependencies with rosdep"
	@echo "  make deps-gazebo   Install dependencies for the detected Gazebo profile"
	@echo "  make check-gazebo-env  Detect and verify Humble/Fortress or Jazzy/Harmonic"
	@echo "  make build         Build the ROS 2 workspace"
	@echo "  make build-gazebo  Build the detected Gazebo profile"
	@echo "  make build-handeye Build hand-eye packages and dependencies"
	@echo "  make test          Build and run all colcon tests"
	@echo "  make test-mission  Run Mission Manager pytest"
	@echo "  make test-mujoco   Run MuJoCo simulation pytest"
	@echo "  make test-handeye  Build and test the hand-eye package boundary"
	@echo "  make handeye-generate-mujoco  Generate analyzed random 20+5 poses"
	@echo "  make handeye-validate-mujoco  Validate the completed 20+5 dataset"
	@echo "  make test-gazebo   Test the detected Gazebo profile"
	@echo "  make test-gazebo-nav-runtime  Run LiDAR, IMU, odom, and TF runtime test"
	@echo "  make test-gazebo-harmonic  Compatibility alias selecting Harmonic"
	@echo "  make sim           Build and run the headless MuJoCo simulation"
	@echo "  make sim-gazebo    Build and run the detected Gazebo profile"
	@echo "  make sim-gazebo-harmonic  Compatibility alias selecting Harmonic"
	@echo "  make sim-gazebo-office  Run Cleany in Husarion Office on Harmonic"
	@echo "  make sim-gazebo-study-cafe  Run the spacious study cafe with GUI"
	@echo "  make handeye-mujoco  Run reviewed 20+5 calibration with viewer"
	@echo "  make clean         Remove ROS 2 build, install, and log outputs"

deps:
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	rosdep install --from-paths src --ignore-src -r -y

deps-gazebo:
	eval "$$(python3 "$(GAZEBO_PROFILE_TOOL)" --shell)" && \
	source "$${CLEANY_ROS_SETUP}" && \
	cd "$(ROS2_WS)" && \
	rosdep install --from-paths src/cleany_description src/cleany_gazebo_sim \
		--ignore-src --skip-keys mujoco --rosdistro "$${CLEANY_ROS_DISTRO}" -r -y

check-gazebo-env:
	eval "$$(python3 "$(GAZEBO_PROFILE_TOOL)" --shell)" && \
	source /etc/os-release && \
	test "$${VERSION_ID}" = "$${CLEANY_UBUNTU_VERSION}" && \
	source "$${CLEANY_ROS_SETUP}" && \
	test "$${ROS_DISTRO}" = "$${CLEANY_ROS_DISTRO}" && \
	test "$$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')" = "$${CLEANY_PYTHON_VERSION}" && \
	ros2 pkg prefix ros_gz_sim >/dev/null && \
	ros2 pkg prefix ros_gz_bridge >/dev/null && \
	echo "Gazebo profile: $${CLEANY_GAZEBO_PROFILE} ($${CLEANY_ROS_DISTRO})"

build:
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	colcon build --symlink-install

build-gazebo: check-gazebo-env
	eval "$$(python3 "$(GAZEBO_PROFILE_TOOL)" --shell)" && \
	source "$${CLEANY_ROS_SETUP}" && \
	cd "$(ROS2_WS)" && \
	colcon --log-base "$${CLEANY_LOG_BASE}" build --symlink-install \
		--build-base "$${CLEANY_BUILD_BASE}" \
		--install-base "$${CLEANY_INSTALL_BASE}" \
		--packages-up-to cleany_gazebo_sim

build-gazebo-harmonic:
	$(MAKE) GAZEBO_PROFILE=harmonic build-gazebo

build-handeye:
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	colcon build --symlink-install \
		--packages-up-to cleany_handeye_calibration

test: build
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source install/setup.bash && \
	colcon test && \
	colcon test-result --verbose && \
	python3 -m pytest "$(REPO_ROOT)tools/test_gazebo_profile.py"

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

test-handeye: build-handeye
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source install/setup.bash && \
	colcon test --packages-select $(HANDEYE_PACKAGES) \
		--event-handlers console_cohesion+ && \
	for package in $(HANDEYE_PACKAGES); do \
		colcon test-result --test-result-base "build/$${package}" \
			--verbose || exit 1; \
	done

test-gazebo: build-gazebo
	eval "$$(python3 "$(GAZEBO_PROFILE_TOOL)" --shell)" && \
	source "$${CLEANY_ROS_SETUP}" && \
	cd "$(ROS2_WS)" && \
	source "$${CLEANY_INSTALL_BASE}/setup.bash" && \
	python3 -m pytest "$(REPO_ROOT)tools/test_gazebo_profile.py" \
		src/cleany_gazebo_sim/test

test-gazebo-harmonic:
	$(MAKE) GAZEBO_PROFILE=harmonic test-gazebo

test-gazebo-nav-runtime: build-gazebo
	eval "$$(python3 "$(GAZEBO_PROFILE_TOOL)" --shell)" && \
	source "$${CLEANY_ROS_SETUP}" && \
	cd "$(ROS2_WS)" && \
	source "$${CLEANY_INSTALL_BASE}/setup.bash" && \
	python3 -m pytest -s \
		src/cleany_gazebo_sim/test/test_runtime_navigation.py \
		--run-sim-runtime --sim-profile="$${CLEANY_GAZEBO_PROFILE}"

sim: build
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source install/setup.bash && \
	ros2 launch cleany_mujoco_sim mujoco_sim.launch.py headless:=true

sim-gazebo: build-gazebo
	eval "$$(python3 "$(GAZEBO_PROFILE_TOOL)" --shell)" && \
	source "$${CLEANY_ROS_SETUP}" && \
	cd "$(ROS2_WS)" && \
	source "$${CLEANY_INSTALL_BASE}/setup.bash" && \
	ros2 launch cleany_gazebo_sim "$${CLEANY_GAZEBO_LAUNCH}" headless:=true

sim-gazebo-harmonic:
	$(MAKE) GAZEBO_PROFILE=harmonic sim-gazebo

<<<<<<< HEAD
handeye-generate-mujoco: build-handeye
	@test ! -e "$(HANDEYE_PROFILE_DIR)" || \
		(echo "pose profile already exists: $(HANDEYE_PROFILE_DIR)" >&2; exit 2)
	@mkdir -p "$(HANDEYE_ARTIFACT_ROOT)"
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source install/setup.bash && \
	ros2 launch cleany_handeye_calibration pose_generation_mujoco.launch.py \
		output_directory:="$(HANDEYE_PROFILE_DIR)" \
		artifact_root:="$(HANDEYE_ARTIFACT_ROOT)" \
		repository_root:="$(REPO_ROOT)" \
		run_id:="$(HANDEYE_RUN_ID)" \
		headless:=true

handeye-mujoco: build-handeye
	@test -n "$(HANDEYE_POSE_MANIFEST)" || \
		(echo "HANDEYE_POSE_MANIFEST=/absolute/materialized_poses.yaml is required" >&2; exit 2)
	@case "$(HANDEYE_POSE_MANIFEST)" in /*) ;; *) \
		echo "HANDEYE_POSE_MANIFEST must be an absolute path" >&2; exit 2;; esac
	@test -f "$(HANDEYE_POSE_MANIFEST)" || \
		(echo "pose manifest does not exist: $(HANDEYE_POSE_MANIFEST); run 'make handeye-generate-mujoco' first" >&2; exit 2)
	@test -n "$(HANDEYE_RUNTIME_CONFIG)" || \
		(echo "HANDEYE_RUNTIME_CONFIG=/absolute/materialized_runtime.json is required" >&2; exit 2)
	@case "$(HANDEYE_RUNTIME_CONFIG)" in /*) ;; *) \
		echo "HANDEYE_RUNTIME_CONFIG must be an absolute path" >&2; exit 2;; esac
	@test -f "$(HANDEYE_RUNTIME_CONFIG)" || \
		(echo "runtime config does not exist: $(HANDEYE_RUNTIME_CONFIG); run 'make handeye-generate-mujoco' first" >&2; exit 2)
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source install/setup.bash && \
	ros2 launch cleany_handeye_calibration multi_pose_mujoco.launch.py \
		pose_manifest:="$(HANDEYE_POSE_MANIFEST)" \
		runtime_config:="$(HANDEYE_RUNTIME_CONFIG)" \
		headless:=false \
		use_rviz:=true

handeye-validate-mujoco: build-handeye
	@test -f "$(HANDEYE_RUN_DIR)/samples.jsonl" || \
		(echo "completed dataset does not exist: $(HANDEYE_RUN_DIR)/samples.jsonl" >&2; exit 2)
	@test -f "$(HANDEYE_POSE_MANIFEST)" || \
		(echo "pose manifest does not exist: $(HANDEYE_POSE_MANIFEST)" >&2; exit 2)
	@test -f "$(HANDEYE_RUNTIME_CONFIG)" || \
		(echo "runtime config does not exist: $(HANDEYE_RUNTIME_CONFIG)" >&2; exit 2)
	@test -f "$(HANDEYE_PROFILE_DIR)/cleany_handeye.urdf" || \
		(echo "materialized URDF does not exist: $(HANDEYE_PROFILE_DIR)/cleany_handeye.urdf" >&2; exit 2)
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source install/setup.bash && \
	sim_share="$$(ros2 pkg prefix --share cleany_mujoco_sim)" && \
	ros2 run cleany_handeye_calibration validate_handeye_dataset \
		--samples "$(HANDEYE_RUN_DIR)/samples.jsonl" \
		--pose-manifest "$(HANDEYE_POSE_MANIFEST)" \
		--runtime-config "$(HANDEYE_RUNTIME_CONFIG)" \
		--urdf "$(HANDEYE_PROFILE_DIR)/cleany_handeye.urdf" \
		--ground-truth "$${sim_share}/config/handeye_scene.yaml" \
		--max-translation-norm-m "$(HANDEYE_MAX_TRANSLATION_NORM_M)" \
		--dataset-mode "$(HANDEYE_DATASET_MODE)" \
		--output "$(HANDEYE_VALIDATION_OUTPUT)"
=======
sim-gazebo-office:
	$(MAKE) GAZEBO_PROFILE=harmonic build-gazebo
	eval "$$(GAZEBO_PROFILE=harmonic python3 "$(GAZEBO_PROFILE_TOOL)" --shell)" && \
	source "$${CLEANY_ROS_SETUP}" && \
	cd "$(ROS2_WS)" && \
	source "$${CLEANY_INSTALL_BASE}/setup.bash" && \
	ros2 launch cleany_gazebo_sim gazebo_office.launch.py headless:=true

sim-gazebo-study-cafe:
	$(MAKE) GAZEBO_PROFILE=harmonic build-gazebo
	eval "$$(GAZEBO_PROFILE=harmonic python3 "$(GAZEBO_PROFILE_TOOL)" --shell)" && \
	source "$${CLEANY_ROS_SETUP}" && \
	cd "$(ROS2_WS)" && \
	source "$${CLEANY_INSTALL_BASE}/setup.bash" && \
	ros2 launch cleany_gazebo_sim gazebo_study_cafe.launch.py headless:=false
>>>>>>> 750110d (feat(gazebo): add SLAM test environments)

clean:
	"$(REPO_ROOT)tools/ros2-clean"
