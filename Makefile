SHELL := /bin/bash

REPO_ROOT := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
ROS2_WS := $(REPO_ROOT)ros2_ws
ROS_SETUP := /opt/ros/humble/setup.bash
MISSION_PACKAGES := cleany_interfaces cleany_mission_manager cleany_control_bridge
MISSION_BUILD_BASE ?= build
MISSION_INSTALL_BASE ?= install
MISSION_LOG_BASE ?= log
GAZEBO_PROFILE_TOOL := $(REPO_ROOT)tools/gazebo_profile.py
GAZEBO_GUI_RENDER_ENGINE ?= ogre
SAFETY_MODE ?= sensors
ROBOT_MODEL ?= cad_frame
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
ROS2_CONTAINER ?= ros2-humble
BASE_TEST_ROS_DOMAIN_ID ?= 173
# Run natively inside the Ubuntu Distrobox, enter it when invoked from the host.
BASE_EXEC ?= $(shell . /etc/os-release; \
	if test "$$ID:$$VERSION_ID" != "ubuntu:22.04"; then \
		printf 'distrobox enter --name $(ROS2_CONTAINER) --no-tty --clean-path --'; fi)

.PHONY: test-mission-core build-mission-runtime build-mission-sim test-mission-runtime help deps deps-gazebo check-gazebo-env build build-gazebo \
	build-gazebo-harmonic build-handeye build-telemetry test test-mission test-mujoco \
	test-handeye test-gazebo test-gazebo-harmonic \
	test-telemetry test-gazebo-nav-runtime test-gazebo-evaluation \
	test-gazebo-safety view-gazebo-costmap \
	handeye-generate-mujoco handeye-validate-mujoco handeye-mujoco \
	sim sim-gazebo sim-gazebo-harmonic sim-gazebo-office \
	sim-gazebo-study-cafe clean firmware-setup firmware-smoke firmware-build \
	firmware-upload test-motor-core build-base test-base \
	micro-ros-agent-build test-micro-ros-setup

help:
	@echo "Cleany native ROS 2 commands"
	@echo "  make deps          Install workspace dependencies with rosdep"
	@echo "  make deps-gazebo   Install dependencies for the detected Gazebo profile"
	@echo "  make check-gazebo-env  Detect and verify Humble/Fortress or Jazzy/Harmonic"
	@echo "  make build         Build the ROS 2 workspace"
	@echo "  make build-gazebo  Build the detected Gazebo profile"
	@echo "  make build-handeye Build hand-eye packages and dependencies"
	@echo "  make build-telemetry Build the ROS telemetry package"
	@echo "  make test          Build and run all colcon tests"
	@echo "  make test-mission  Run Mission Manager pytest"
	@echo "  make test-mission-core  Run ROS-independent FSM, BT and bridge tests"
	@echo "  make build-mission-runtime  Build interfaces, FSM and Backend bridge"
	@echo "  make build-mission-sim  Build Gazebo + navigation + runtime + pose relay"
	@echo "  make test-mission-runtime  Build and test mission ROS packages"
	@echo "  make test-mujoco   Run MuJoCo simulation pytest"
	@echo "  make test-handeye  Build and test the hand-eye package boundary"
	@echo "  make test-telemetry Test the ROS telemetry package"
	@echo "  make handeye-generate-mujoco  Generate analyzed random 20+5 poses"
	@echo "  make handeye-validate-mujoco  Validate the completed 20+5 dataset"
	@echo "  make test-gazebo   Test the detected Gazebo profile"
	@echo "  make test-gazebo-nav-runtime  Run LiDAR, IMU, odom, and TF runtime test"
	@echo "  make test-gazebo-evaluation  Run temporary SLAM evaluation checks"
	@echo "  make test-gazebo-safety  Run SCRUM-306 sensors/monitor/avoid evaluation"
	@echo "  make view-gazebo-costmap  Show live costmap over saved SLAM map in RViz"
	@echo "  make test-gazebo-harmonic  Compatibility alias selecting Harmonic"
	@echo "  make sim           Build and run the headless MuJoCo simulation"
	@echo "  make sim-gazebo    Build and run the detected Gazebo profile"
	@echo "  make sim-gazebo-harmonic  Compatibility alias selecting Harmonic"
	@echo "  make sim-gazebo-study-cafe  Run the spacious study cafe with GUI"
	@echo "  make handeye-mujoco  Run reviewed 20+5 calibration with viewer"
	@echo "  make clean         Remove ROS 2 build, install, and log outputs"
	@echo "  make firmware-setup / firmware-smoke / firmware-build"
	@echo "  make test-motor-core / build-base / test-base"
	@echo "  make micro-ros-agent-build  Build pinned Agent and check --help"
	@echo "  make firmware-upload CLEANY_ESP_PORT=/dev/... CONFIRM_UPLOAD=1"

firmware-setup:
	$(BASE_EXEC) bash -lc 'cd "$(REPO_ROOT)" && source /opt/ros/humble/setup.bash && /usr/bin/python3 tools/micro_ros_setup.py'

test-micro-ros-setup:
	$(BASE_EXEC) bash -lc 'cd "$(REPO_ROOT)" && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest -q tools/test_micro_ros_setup.py'

firmware-smoke: firmware-setup
	$(BASE_EXEC) bash -lc 'cd "$(REPO_ROOT)" && env -u ROS_DISTRO -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u PYTHONPATH esp32/.venv/bin/platformio run -d esp32/microros_smoke -e esp32-s3-microros-smoke'

firmware-build: firmware-setup
	$(BASE_EXEC) bash -lc 'cd "$(REPO_ROOT)" && env -u ROS_DISTRO -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u PYTHONPATH esp32/.venv/bin/platformio run -d esp32 -e esp32-s3-microros'

firmware-upload: firmware-setup
	@test -n "$(CLEANY_ESP_PORT)" || (echo "CLEANY_ESP_PORT is required" >&2; exit 2)
	@test "$(CONFIRM_UPLOAD)" = 1 || (echo "CONFIRM_UPLOAD=1 is required" >&2; exit 2)
	$(BASE_EXEC) bash -lc 'cd "$(REPO_ROOT)" && env -u ROS_DISTRO -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u PYTHONPATH esp32/.venv/bin/platformio run -d esp32 -e esp32-s3-microros -t upload --upload-port "$(CLEANY_ESP_PORT)"'

test-motor-core:
	$(BASE_EXEC) bash -lc 'set -e; cd "$(REPO_ROOT)"; for f in esp32/host_tests/*.cpp; do g++ -std=c++17 -Wall -Wextra -Werror -pedantic -Iesp32/src "$$f" -o "/tmp/$$(basename "$$f" .cpp)"; "/tmp/$$(basename "$$f" .cpp)"; done'

build-base:
	$(BASE_EXEC) bash -lc 'source /opt/ros/humble/setup.bash && cd "$(ROS2_WS)" && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon build --symlink-install --packages-up-to cleany_base_driver cleany_base_odometry'

test-base: build-base
	$(BASE_EXEC) bash -lc 'source /opt/ros/humble/setup.bash && cd "$(ROS2_WS)" && source install/setup.bash && export ROS_DOMAIN_ID=$(BASE_TEST_ROS_DOMAIN_ID) && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --packages-select cleany_base_interfaces cleany_base_driver cleany_base_odometry && for p in cleany_base_interfaces cleany_base_driver cleany_base_odometry; do colcon test-result --test-result-base "build/$$p" --verbose || exit 1; done && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest src/cleany_base_driver/test src/cleany_base_odometry/test'

micro-ros-agent-build:
	$(BASE_EXEC) bash -lc 'cd "$(REPO_ROOT)" && source /opt/ros/humble/setup.bash && /usr/bin/python3 tools/micro_ros_setup.py --agent-build && source esp32/micro_ros/agent/install/local_setup.bash && out=$$(ros2 run micro_ros_agent micro_ros_agent --help 2>&1 | tr -d "\000" | sed "/^\\[ros2run\\]: Process exited/d" || true); printf "%s\n" "$$out"; grep -q "Usage:.*micro_ros_agent" <<<"$$out"'

deps:
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	rosdep install --from-paths src --ignore-src -r -y

deps-gazebo:
	eval "$$(python3 "$(GAZEBO_PROFILE_TOOL)" --shell)" && \
	source "$${CLEANY_ROS_SETUP}" && \
	cd "$(ROS2_WS)" && \
	rosdep install --from-paths src/cleany_description src/cleany_navigation \
		src/cleany_gazebo_sim \
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

build-telemetry:
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	colcon build --symlink-install --packages-select cleany_telemetry

test-telemetry: build-telemetry
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source install/setup.bash && \
	python3 -m pytest src/cleany_telemetry/test

test: build
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source install/setup.bash && \
	colcon test && \
	colcon test-result --verbose && \
	python3 -m pytest "$(REPO_ROOT)tools/test_gazebo_profile.py"

test-mission-core:
	PYTHONDONTWRITEBYTECODE=1 \
	PYTHONPATH="$(ROS2_WS)/src/cleany_mission_manager:$(ROS2_WS)/src/cleany_control_bridge:$${PYTHONPATH:-}" \
	python3 -m pytest -p no:cacheprovider "$(ROS2_WS)/src/cleany_mission_manager/tests" \
		"$(ROS2_WS)/src/cleany_control_bridge/test"

build-mission-runtime:
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	colcon --log-base "$(MISSION_LOG_BASE)" build --symlink-install \
		--build-base "$(MISSION_BUILD_BASE)" --install-base "$(MISSION_INSTALL_BASE)" \
		--packages-select $(MISSION_PACKAGES)

build-mission-sim:
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	colcon --log-base "$(MISSION_LOG_BASE)" build --symlink-install \
		--build-base "$(MISSION_BUILD_BASE)" --install-base "$(MISSION_INSTALL_BASE)" \
		--packages-up-to cleany_bringup

test-mission-runtime: build-mission-runtime
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source "$(MISSION_INSTALL_BASE)/setup.bash" && \
	colcon --log-base "$(MISSION_LOG_BASE)" test --build-base "$(MISSION_BUILD_BASE)" \
		--install-base "$(MISSION_INSTALL_BASE)" --packages-select $(MISSION_PACKAGES) && \
	colcon test-result --test-result-base "$(MISSION_BUILD_BASE)" --verbose

test-mission: test-mission-core

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
		src/cleany_gazebo_sim/test && \
	"$${CLEANY_BUILD_BASE}/cleany_axis_controller/test_axis_generator" && \
	"$${CLEANY_BUILD_BASE}/cleany_axis_controller/test_yield_wait"

test-gazebo-harmonic:
	$(MAKE) GAZEBO_PROFILE=harmonic test-gazebo

test-gazebo-nav-runtime: build-gazebo
	eval "$$(python3 "$(GAZEBO_PROFILE_TOOL)" --shell)" && \
	source "$${CLEANY_ROS_SETUP}" && \
	cd "$(ROS2_WS)" && \
	source "$${CLEANY_INSTALL_BASE}/setup.bash" && \
	python3 -m pytest -s \
		src/cleany_gazebo_sim/test/test_runtime_simulation.py \
		--run-sim-runtime --sim-profile="$${CLEANY_GAZEBO_PROFILE}"

test-gazebo-evaluation: build-gazebo
	eval "$$(python3 "$(GAZEBO_PROFILE_TOOL)" --shell)" && \
	source "$${CLEANY_ROS_SETUP}" && \
	cd "$(ROS2_WS)" && \
	source "$${CLEANY_INSTALL_BASE}/setup.bash" && \
	python3 -m pytest src/cleany_gazebo_sim/test/evaluation \
		--run-evaluation-tests

test-gazebo-safety: build-gazebo
	@test -n "$(SAFETY_MAP)" && test -n "$(SAFETY_OUTPUT)" || \
		(echo "SAFETY_MAP and a new SAFETY_OUTPUT directory are required" >&2; exit 2)
	eval "$$(python3 "$(GAZEBO_PROFILE_TOOL)" --shell)" && \
	test "$${CLEANY_GAZEBO_PROFILE}" = harmonic && \
	source "$${CLEANY_ROS_SETUP}" && \
	source "$(ROS2_WS)/$${CLEANY_INSTALL_BASE}/setup.bash" && \
	python3 "$(REPO_ROOT)tools/navigation_evaluation/run_safety_evaluation.py" \
		--map "$(SAFETY_MAP)" --output "$(SAFETY_OUTPUT)" --mode "$(SAFETY_MODE)" --robot-model "$(ROBOT_MODEL)"

view-gazebo-costmap: build-gazebo
	@test -n "$(SAFETY_MAP)" && test -n "$(SAFETY_OUTPUT)" || \
		(echo "SAFETY_MAP and a new SAFETY_OUTPUT directory are required" >&2; exit 2)
	eval "$$(python3 "$(GAZEBO_PROFILE_TOOL)" --shell)" && \
	test "$${CLEANY_GAZEBO_PROFILE}" = harmonic && \
	source "$${CLEANY_ROS_SETUP}" && \
	source "$(ROS2_WS)/$${CLEANY_INSTALL_BASE}/setup.bash" && \
	python3 "$(REPO_ROOT)tools/navigation_evaluation/live_costmap.py" \
		--map "$(SAFETY_MAP)" --output "$(SAFETY_OUTPUT)" --robot-model "$(ROBOT_MODEL)" $(if $(filter 1,$(SAFETY_DRIVE)),--drive,)

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
	ros2 launch cleany_gazebo_sim gazebo_study_cafe.launch.py \
		headless:=false gui_render_engine:="$(GAZEBO_GUI_RENDER_ENGINE)"

clean:
	"$(REPO_ROOT)tools/ros2-clean"
