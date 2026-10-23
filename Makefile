SHELL := /bin/bash

export PYTEST_DISABLE_PLUGIN_AUTOLOAD := 1

REPO_ROOT := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
ROS2_WS := $(REPO_ROOT)ros2_ws
ROS_SETUP := /opt/ros/humble/setup.bash
GEOMETRY_PYTHON ?= python3
CUROBO_GPU_TEST_FILE ?= $(ROS2_WS)/src/cleany_skill_executor/test/test_curobo_gpu.py

COLCON_BUILD_ARGS ?=
SORTING_ARGS ?=
# Keep data and generated outputs outside the source worktree.
CLEANY_DATA_ROOT ?= $(abspath $(REPO_ROOT)/../cleany-data)
CLEANY_BUILD_ROOT ?= $(CLEANY_DATA_ROOT)/builds/$(notdir $(patsubst %/,%,$(REPO_ROOT)))
CLEANY_INSTALL_BASE ?= $(INSTALL_BASE)
export CLEANY_DATA_ROOT CLEANY_BUILD_ROOT CLEANY_INSTALL_BASE
BUILD_BASE ?= $(CLEANY_BUILD_ROOT)/build
INSTALL_BASE ?= $(CLEANY_BUILD_ROOT)/install
LOG_BASE ?= $(CLEANY_BUILD_ROOT)/log
NAVIGATION_BUILD_BASE ?= $(CLEANY_BUILD_ROOT)/build-navigation
NAVIGATION_INSTALL_BASE ?= $(CLEANY_BUILD_ROOT)/install-navigation
NAVIGATION_LOG_BASE ?= $(CLEANY_BUILD_ROOT)/log-navigation
MISSION_PACKAGES := cleany_interfaces cleany_mission_manager cleany_control_bridge
MISSION_MOCK_PACKAGES := $(MISSION_PACKAGES) cleany_skill_executor
MISSION_BUILD_BASE ?= $(CLEANY_BUILD_ROOT)/build
MISSION_INSTALL_BASE ?= $(CLEANY_BUILD_ROOT)/install
MISSION_LOG_BASE ?= $(CLEANY_BUILD_ROOT)/log
GAZEBO_GUI_RENDER_ENGINE ?= ogre
HANDEYE_PROFILE_DIR ?= $(CLEANY_DATA_ROOT)/fixtures/handeye/mujoco_seed_20260810
HANDEYE_POSE_MANIFEST ?= $(HANDEYE_PROFILE_DIR)/materialized_poses.yaml
HANDEYE_RUNTIME_CONFIG ?= $(HANDEYE_PROFILE_DIR)/materialized_runtime.json
HANDEYE_ARTIFACT_ROOT ?= $(CLEANY_DATA_ROOT)/runs/handeye
HANDEYE_RUN_ID ?= mujoco_seed_20260810
HANDEYE_RUN_DIR ?= $(HANDEYE_ARTIFACT_ROOT)/$(HANDEYE_RUN_ID)
HANDEYE_VALIDATION_OUTPUT ?= $(HANDEYE_RUN_DIR)/dataset_validation.json
HANDEYE_MAX_TRANSLATION_NORM_M ?= 1.0
HANDEYE_DATASET_MODE ?= strict
HANDEYE_PACKAGES := cleany_description cleany_mujoco_sim \
	cleany_moveit_config cleany_handeye_calibration
ROS2_CONTAINER ?= ros2-humble
BASE_TEST_ROS_DOMAIN_ID ?= 173
IMU_CALIBRATION_OUTPUT ?= $(REPO_ROOT)artifacts/imu_calibration/result.json
IMU_CALIBRATION_TIMEOUT ?= 60
IMU_CALIBRATION_ARGS ?=
# Run natively inside the Ubuntu Distrobox, enter it when invoked from the host.
BASE_EXEC ?= $(shell . /etc/os-release; \
	if test "$$ID:$$VERSION_ID" != "ubuntu:22.04"; then \
		printf 'distrobox enter --name $(ROS2_CONTAINER) --no-tty --clean-path --'; fi)

.PHONY: test-gazebo-facility-tools test-mission-core build-mission-runtime build-mission-sim test-mission-runtime help deps deps-gazebo check-gazebo-env build build-gazebo \
	build-handeye build-telemetry test test-mission test-mujoco \
	test-handeye test-gazebo \
	test-telemetry test-gazebo-nav-runtime test-gazebo-evaluation \
	handeye-generate-mujoco handeye-validate-mujoco handeye-mujoco \
	sim sim-gazebo \
	sim-gazebo-study-cafe sim-gazebo-facility clean firmware-setup firmware-smoke firmware-build \
	firmware-upload test-motor-core build-base test-base \
	micro-ros-agent-build test-micro-ros-setup imu-calibration-build \
	imu-calibration-upload imu-calibration-capture test-imu-calibration test-odometry-evaluation test-imu-runtime

.PHONY: build-manipulation build-manipulation-bt test-manipulation-core \
	test-manipulation test-manipulation-bt sim-mujoco-manipulation

help:
	@echo "Cleany native ROS 2 commands"
	@echo "  make test-collision-geometry  Test ROS-independent conservative geometry (GEOMETRY_PYTHON override)"
	@echo "  make deps          Install workspace dependencies with rosdep"
	@echo "  make deps-gazebo   Install Gazebo Fortress dependencies with rosdep"
	@echo "  make check-gazebo-env  Verify Ubuntu 22.04 / Humble / Fortress environment"
	@echo "  make build         Build the ROS 2 workspace"
	@echo "  make build-manipulation  Build mock manipulation Action packages"
	@echo "  make build-manipulation-bt  Build native BT and MuJoCo Action packages"
	@echo "  make test-manipulation-core  Test manipulation core and monitor"
	@echo "  make test-manipulation  Test mock Action, cancellation and recovery"
	@echo "  make test-manipulation-bt  Test BT and related perception/manipulation contracts"
	@echo "  make sim-mujoco-manipulation  Run the single-object MuJoCo Action stack"
	@echo "  make build-gazebo  Build cleany_gazebo_sim and its dependencies"
	@echo "  make build-navigation  Build common navigation without Gazebo"
	@echo "  make test-navigation  Test common navigation including C++ plugins"
	@echo "  make test-navigation-runtime  Test Nav2 serial safety with a synthetic backend"
	@echo "  make build-handeye Build hand-eye packages and dependencies"
	@echo "  make build-telemetry Build the ROS telemetry package"
	@echo "  make test          Build and run all colcon tests"
	@echo "  make test-mission  Run Mission Manager pytest"
	@echo "  make build-dev-monitor / run-dev-monitor  Build or serve the read-only robot monitor"
	@echo "  make logger LOGGER_ARGS='status'  Control the independent recorder"
	@echo "  make test-logger    Test the recording engine without Monitor or ROS"
	@echo "  make test-dev-monitor  Test Logger, HTTP API and ROS observation"
	@echo "  make test-mission-core  Run ROS-independent FSM, BT and bridge tests"
	@echo "  make build-mission-runtime  Build interfaces, FSM and Backend bridge"
	@echo "  make build-mission-sim  Build Gazebo + navigation + runtime + pose relay"
	@echo "  make test-mission-runtime  Build and test mission ROS packages"
	@echo "  make build-mission-mock / test-mission-mock  Build and test the integrated Mock Action"
	@echo "  make test-gazebo-manipulation  Check study-cafe Nav2 + two Mock Action collections"
	@echo "  make test-mujoco   Run MuJoCo simulation pytest"
	@echo "  make test-handeye  Build and test the hand-eye package boundary"
	@echo "  make test-telemetry Test the ROS telemetry package"
	@echo "  make handeye-generate-mujoco  Generate analyzed random 20+5 poses"
	@echo "  make handeye-validate-mujoco  Validate the completed 20+5 dataset"
	@echo "  make test-gazebo   Test Gazebo Fortress and navigation packages"
	@echo "  make test-gazebo-nav-runtime  Run LiDAR, IMU, odom, and TF runtime test"
	@echo "  make test-gazebo-evaluation  Run temporary SLAM evaluation checks"
	@echo "  make sim           Build and run the headless MuJoCo simulation"
	@echo "  make sim-gazebo    Build and run headless Gazebo Fortress simulation"
	@echo "  make sim-gazebo-study-cafe  Run the spacious study cafe with GUI"
	@echo "  make sim-gazebo-facility  Run the full 18F facility with ROLY chairs"
	@echo "  make handeye-mujoco  Run reviewed 20+5 calibration with viewer"
	@echo "  make clean         Remove ROS 2 build, install, and log outputs"
	@echo "  make firmware-setup / firmware-smoke / firmware-build"
	@echo "  make test-motor-core / build-base / test-base"
	@echo "  make test-odometry-evaluation  Test config-driven wheel odom evaluation"
	@echo "  make micro-ros-agent-build  Build pinned Agent and check --help"
	@echo "  make firmware-upload CLEANY_ESP_PORT=/dev/... CONFIRM_UPLOAD=1"
	@echo "  make imu-calibration-build / test-imu-calibration"
	@echo "  make test-imu-runtime  Test MCU calibration config and ROS gyro relay"
	@echo "  make imu-calibration-upload CLEANY_ESP_PORT=/dev/... CONFIRM_UPLOAD=1"
	@echo "  make imu-calibration-capture CLEANY_ESP_PORT=/dev/... [IMU_CALIBRATION_ARGS=--calibrate]"

firmware-setup:
	$(BASE_EXEC) bash -lc 'cd "$(REPO_ROOT)" && source /opt/ros/humble/setup.bash && /usr/bin/python3 esp32/scripts/micro_ros_setup.py'

test-micro-ros-setup:
	$(BASE_EXEC) bash -lc 'cd "$(REPO_ROOT)" && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest -q esp32/host_tests/test_micro_ros_setup.py'

test-gazebo-facility-tools:
	$(BASE_EXEC) bash -lc 'cd "$(REPO_ROOT)" && PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH="$(ROS2_WS)/src/cleany_gazebo_sim:$${PYTHONPATH:-}" /usr/bin/python3 -m pytest -p no:cacheprovider -q ros2_ws/src/cleany_gazebo_sim/test/test_mission_targets.py'

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

imu-calibration-build:
	$(BASE_EXEC) bash -lc 'cd "$(REPO_ROOT)" && env -u ROS_DISTRO -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u PYTHONPATH esp32/.venv/bin/platformio run -d esp32/imu_calibration -e esp32-s3-imu-calibration'

imu-calibration-upload:
	@test -n "$(CLEANY_ESP_PORT)" || (echo "CLEANY_ESP_PORT is required" >&2; exit 2)
	@test "$(CONFIRM_UPLOAD)" = 1 || (echo "CONFIRM_UPLOAD=1 is required" >&2; exit 2)
	$(BASE_EXEC) bash -lc 'cd "$(REPO_ROOT)" && env -u ROS_DISTRO -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u PYTHONPATH esp32/.venv/bin/platformio run -d esp32/imu_calibration -e esp32-s3-imu-calibration -t upload --upload-port "$(CLEANY_ESP_PORT)"'

imu-calibration-capture:
	@test -n "$(CLEANY_ESP_PORT)" || (echo "CLEANY_ESP_PORT is required" >&2; exit 2)
	$(BASE_EXEC) bash -lc 'cd "$(REPO_ROOT)" && esp32/.venv/bin/python tools/imu_calibration/capture.py --port "$(CLEANY_ESP_PORT)" --output "$(IMU_CALIBRATION_OUTPUT)" --timeout "$(IMU_CALIBRATION_TIMEOUT)" $(IMU_CALIBRATION_ARGS)'

test-imu-calibration: test-motor-core
	$(BASE_EXEC) bash -lc 'cd "$(REPO_ROOT)" && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest -q tools/imu_calibration'

build-base:
	$(BASE_EXEC) bash -lc 'source /opt/ros/humble/setup.bash && cd "$(ROS2_WS)" && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon --log-base "$(LOG_BASE)" build --symlink-install --build-base "$(BUILD_BASE)" --install-base "$(INSTALL_BASE)" --packages-up-to cleany_base_driver cleany_base_odometry'

test-base: build-base
	$(BASE_EXEC) bash -lc 'source /opt/ros/humble/setup.bash && cd "$(ROS2_WS)" && source "$(INSTALL_BASE)/setup.bash" && export ROS_DOMAIN_ID=$(BASE_TEST_ROS_DOMAIN_ID) && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon --log-base "$(LOG_BASE)" test --build-base "$(BUILD_BASE)" --install-base "$(INSTALL_BASE)" --packages-select cleany_base_interfaces cleany_base_driver cleany_base_odometry && for p in cleany_base_interfaces cleany_base_driver cleany_base_odometry; do colcon --log-base "$(LOG_BASE)" test-result --test-result-base "$(BUILD_BASE)/$$p" --verbose || exit 1; done'

test-imu-runtime: build-base
	$(BASE_EXEC) bash -lc 'set -e; source /opt/ros/humble/setup.bash; source "$(INSTALL_BASE)/setup.bash"; cd "$(REPO_ROOT)"; export ROS_DOMAIN_ID=$(BASE_TEST_ROS_DOMAIN_ID); PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest -q ros2_ws/src/cleany_base_interfaces/test/test_contract.py ros2_ws/src/cleany_base_driver/test/test_imu_core.py ros2_ws/src/cleany_base_driver/test/test_imu_graph.py tools/test_imu_runtime_config.py'

test-odometry-evaluation:
	$(BASE_EXEC) bash -lc 'set -e; cd "$(REPO_ROOT)"; source /opt/ros/humble/setup.bash; if test -f ros2_ws/install/setup.bash; then source ros2_ws/install/setup.bash; fi; for launcher in run_wheel_only_eval.sh run_motion_response.sh; do bash -n "tools/odometry_evaluation/$$launcher"; done; PYTHONNOUSERSITE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest -q tools/odometry_evaluation tools/wheel_calibration'

micro-ros-agent-build:
	$(BASE_EXEC) bash -lc 'cd "$(REPO_ROOT)" && source /opt/ros/humble/setup.bash && /usr/bin/python3 esp32/scripts/micro_ros_setup.py --agent-build && source esp32/micro_ros/agent/install/local_setup.bash && out=$$(ros2 run micro_ros_agent micro_ros_agent --help 2>&1 | tr -d "\000" | sed "/^\\[ros2run\\]: Process exited/d" || true); printf "%s\n" "$$out"; grep -q "Usage:.*micro_ros_agent" <<<"$$out"'

deps:
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	rosdep install --from-paths src --ignore-src -r -y

deps-gazebo:
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	rosdep install --from-paths src/cleany_description src/cleany_navigation \
		src/cleany_gazebo_sim \
		--ignore-src --skip-keys python3-mujoco-pip --rosdistro humble -r -y

check-gazebo-env:
	source /etc/os-release && \
	test "$${VERSION_ID}" = "22.04" && \
	source "$(ROS_SETUP)" && \
	test "$${ROS_DISTRO}" = "humble" && \
	test "$$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')" = "3.10" && \
	bash -c 'v=$$(ign gazebo --versions 2>&1); [[ "$$v" =~ ^6\. ]]' && \
	ros2 pkg prefix ros_gz_sim >/dev/null && \
	ros2 pkg prefix ros_gz_bridge >/dev/null && \
	echo "Gazebo profile: fortress (humble)"

.PHONY: build-navigation test-navigation test-navigation-runtime

build-navigation:
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	colcon --log-base "$(NAVIGATION_LOG_BASE)" build --symlink-install \
		--build-base "$(NAVIGATION_BUILD_BASE)" \
		--install-base "$(NAVIGATION_INSTALL_BASE)" \
		--packages-up-to cleany_navigation

test-navigation: build-navigation
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source "$(NAVIGATION_INSTALL_BASE)/setup.bash" && \
	colcon --log-base "$(NAVIGATION_LOG_BASE)" test \
		--build-base "$(NAVIGATION_BUILD_BASE)" \
		--install-base "$(NAVIGATION_INSTALL_BASE)" \
		--packages-select cleany_navigation \
		--return-code-on-test-failure && \
	colcon --log-base "$(LOG_BASE)" test-result --test-result-base "$(NAVIGATION_BUILD_BASE)" --verbose

test-navigation-runtime: build-navigation
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source "$(NAVIGATION_INSTALL_BASE)/setup.bash" && \
	CLEANY_RUN_NAVIGATION_RUNTIME=1 python3 -m pytest -s \
		src/cleany_navigation/runtime_test/test_navigation.py

build:
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	colcon --log-base "$(LOG_BASE)" build --symlink-install --build-base "$(BUILD_BASE)" --install-base "$(INSTALL_BASE)" $(COLCON_BUILD_ARGS)

PERCEPTION_TEST_PATHS ?= ros2_ws/src/cleany_perception/test
PERCEPTION_PYTEST_ARGS ?=
.PHONY: test-perception
test-perception:
	source "$(ROS_SETUP)" && \
	if test -f "$(INSTALL_BASE)/setup.bash"; then source "$(INSTALL_BASE)/setup.bash"; fi && \
	PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$(ROS2_WS)/src/cleany_perception:$(ROS2_WS)/src/cleany_skill_executor:$(ROS2_WS)/src/cleany_dev_monitor:$${PYTHONPATH:-}" \
	python3 -m pytest -q -p no:cacheprovider $(PERCEPTION_TEST_PATHS) $(PERCEPTION_PYTEST_ARGS)

build-gazebo: check-gazebo-env
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	colcon --log-base "$(LOG_BASE)" build --symlink-install \
		--build-base "$(BUILD_BASE)" \
		--install-base "$(INSTALL_BASE)" \
		--packages-up-to cleany_gazebo_sim

build-manipulation:
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	colcon --log-base "$(LOG_BASE)" build --symlink-install \
		--build-base "$(BUILD_BASE)" --install-base "$(INSTALL_BASE)" \
		--packages-up-to cleany_skill_executor $(COLCON_BUILD_ARGS)

build-manipulation-bt:
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	colcon --log-base "$(LOG_BASE)" build --symlink-install \
		--build-base "$(BUILD_BASE)" --install-base "$(INSTALL_BASE)" \
		--packages-up-to cleany_manipulation_bt $(COLCON_BUILD_ARGS)

test-manipulation-core:
	PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
	PYTHONPATH="$(ROS2_WS)/src/cleany_skill_executor:$${PYTHONPATH:-}" \
	python3 -m pytest -q -p no:cacheprovider \
		"$(ROS2_WS)/src/cleany_skill_executor/test/test_manipulation_core.py" \
		"$(ROS2_WS)/src/cleany_skill_executor/test/test_manipulation_monitor.py"

test-manipulation: build-manipulation
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source "$(INSTALL_BASE)/setup.bash" && \
	python3 -m pytest -q src/cleany_interfaces/test/test_interface_contract.py \
		src/cleany_skill_executor/test/test_manipulation_core.py \
		src/cleany_skill_executor/test/test_manipulation_runtime.py \
		src/cleany_skill_executor/test/test_manipulation_monitor.py \
		src/cleany_skill_executor/test/test_manipulation_monitor_runtime.py

test-manipulation-bt: build-manipulation-bt
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source "$(INSTALL_BASE)/setup.bash" && \
	python3 -m pytest -q src/cleany_manipulation_bt/test \
		src/cleany_perception/test/test_inspection_node.py \
		src/cleany_skill_executor/test/test_sorting_coordinator.py \
		src/cleany_skill_executor/test/test_nearest_object.py \
		src/cleany_skill_executor/test/test_learned_runtime_launch.py

sim-mujoco-manipulation: build-manipulation-bt
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source "$(INSTALL_BASE)/setup.bash" && \
	ros2 launch cleany_manipulation_bt manipulation_mujoco.launch.py $(SORTING_ARGS)

build-handeye:
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	colcon --log-base "$(LOG_BASE)" build --symlink-install --build-base "$(BUILD_BASE)" --install-base "$(INSTALL_BASE)" \
		--packages-up-to cleany_handeye_calibration

build-telemetry:
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	colcon --log-base "$(LOG_BASE)" build --symlink-install --build-base "$(BUILD_BASE)" --install-base "$(INSTALL_BASE)" --packages-select cleany_telemetry

test-telemetry: build-telemetry
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source "$(INSTALL_BASE)/setup.bash" && \
	python3 -m pytest src/cleany_telemetry/test

test: COLCON_BUILD_ARGS += --cmake-force-configure
test: build
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source "$(INSTALL_BASE)/setup.bash" && \
	{ \
		test_status=0; \
		colcon --log-base "$(LOG_BASE)" test --build-base "$(BUILD_BASE)" --install-base "$(INSTALL_BASE)" --return-code-on-test-failure || test_status=$$?; \
		colcon --log-base "$(LOG_BASE)" test-result --test-result-base "$(BUILD_BASE)" --verbose || test_status=$$?; \
		exit $$test_status; \
	}

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
	colcon --log-base "$(LOG_BASE)" test-result --test-result-base "$(MISSION_BUILD_BASE)" --verbose

test-mission: test-mission-core

.PHONY: build-mission-mock test-mission-mock test-gazebo-manipulation
build-mission-mock:
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon --log-base "$(MISSION_LOG_BASE)" build --symlink-install \
		--build-base "$(MISSION_BUILD_BASE)" --install-base "$(MISSION_INSTALL_BASE)" \
		--packages-up-to $(MISSION_MOCK_PACKAGES)

test-mission-mock: build-mission-mock
	source "$(ROS_SETUP)" && \
	source "$(MISSION_INSTALL_BASE)/setup.bash" && \
	ROS_DOMAIN_ID=$(BASE_TEST_ROS_DOMAIN_ID) CLEANY_RUN_MANIPULATION_TESTS=1 CLEANY_RUN_ROS_TESTS=1 \
	PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONDONTWRITEBYTECODE=1 \
	python3 -m pytest -p no:cacheprovider \
		"$(ROS2_WS)/src/cleany_interfaces/test" \
		"$(ROS2_WS)/src/cleany_skill_executor/test/test_manipulation_core.py" \
		"$(ROS2_WS)/src/cleany_skill_executor/test/test_manipulation_runtime.py" \
		"$(ROS2_WS)/src/cleany_mission_manager/tests" \
		"$(ROS2_WS)/src/cleany_control_bridge/test"

test-gazebo-manipulation: build-mission-sim
	source "$(ROS_SETUP)" && \
	source "$(MISSION_INSTALL_BASE)/setup.bash" && \
	ROS_DOMAIN_ID=$(BASE_TEST_ROS_DOMAIN_ID) CLEANY_RUN_GAZEBO_MANIPULATION_TESTS=1 \
	PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONDONTWRITEBYTECODE=1 \
	python3 -m pytest -p no:cacheprovider -s \
		"$(ROS2_WS)/src/cleany_mission_manager/tests/test_ros_gazebo_manipulation.py"

test-mujoco: build
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source "$(INSTALL_BASE)/setup.bash" && \
	python3 -m pytest src/cleany_mujoco_sim/test/test_scene_loader.py

test-handeye: build-handeye
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source "$(INSTALL_BASE)/setup.bash" && \
	colcon --log-base "$(LOG_BASE)" test --build-base "$(BUILD_BASE)" --install-base "$(INSTALL_BASE)" --packages-select $(HANDEYE_PACKAGES) \
		--event-handlers console_cohesion+ && \
	for package in $(HANDEYE_PACKAGES); do \
		colcon --log-base "$(LOG_BASE)" test-result --test-result-base "$(BUILD_BASE)/$${package}" \
			--verbose || exit 1; \
	done

test-gazebo: build-gazebo
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source "$(INSTALL_BASE)/setup.bash" && \
	python3 -m pytest \
		src/cleany_gazebo_sim/test src/cleany_navigation/test && \
	"$(BUILD_BASE)/cleany_navigation/test_axis_generator" && \
	"$(BUILD_BASE)/cleany_navigation/test_yield_wait"

test-gazebo-nav-runtime: build-gazebo
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source "$(INSTALL_BASE)/setup.bash" && \
	python3 -m pytest -s \
		src/cleany_gazebo_sim/test/test_runtime_simulation.py \
		--run-sim-runtime

test-gazebo-evaluation: build-gazebo
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source "$(INSTALL_BASE)/setup.bash" && \
	python3 -m pytest src/cleany_gazebo_sim/test/evaluation \
		--run-evaluation-tests

sim: build
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source "$(INSTALL_BASE)/setup.bash" && \
	ros2 launch cleany_mujoco_sim mujoco_sim.launch.py headless:=true

sim-gazebo: build-gazebo
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source "$(INSTALL_BASE)/setup.bash" && \
	ros2 launch cleany_gazebo_sim gazebo_fortress.launch.py headless:=true

handeye-generate-mujoco: build-handeye
	@test ! -e "$(HANDEYE_PROFILE_DIR)" || \
		(echo "pose profile already exists: $(HANDEYE_PROFILE_DIR)" >&2; exit 2)
	@mkdir -p "$(HANDEYE_ARTIFACT_ROOT)"
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source "$(INSTALL_BASE)/setup.bash" && \
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
	source "$(INSTALL_BASE)/setup.bash" && \
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
	source "$(INSTALL_BASE)/setup.bash" && \
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

sim-gazebo-study-cafe: build-gazebo
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source "$(INSTALL_BASE)/setup.bash" && \
	ros2 launch cleany_gazebo_sim gazebo_study_cafe_fortress.launch.py \
		headless:=false gui_render_engine:="$(GAZEBO_GUI_RENDER_ENGINE)"

sim-gazebo-facility: build-gazebo
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source "$(INSTALL_BASE)/setup.bash" && \
	ros2 launch cleany_gazebo_sim gazebo_facility_18f.launch.py \
		headless:=false gui_render_engine:="$(GAZEBO_GUI_RENDER_ENGINE)"

clean:
	"$(REPO_ROOT)tools/ros2/ros2-clean"

DEV_MONITOR_WEB := $(ROS2_WS)/src/cleany_dev_monitor/web
.PHONY: build-dev-monitor-web build-dev-monitor-ros build-dev-monitor test-logger test-dev-monitor run-dev-monitor
build-dev-monitor-web:
	pnpm --dir "$(DEV_MONITOR_WEB)" install --frozen-lockfile
	pnpm --dir "$(DEV_MONITOR_WEB)" build

build-dev-monitor-ros:
	@test -f "$(DEV_MONITOR_WEB)/dist/index.html" || (echo "Run make build-dev-monitor-web first" >&2; exit 2)
	source "$(ROS_SETUP)" && cd "$(ROS2_WS)" && \
	colcon --log-base "$(MISSION_LOG_BASE)" build --symlink-install \
		--build-base "$(MISSION_BUILD_BASE)" --install-base "$(MISSION_INSTALL_BASE)" \
		--packages-up-to cleany_dev_monitor cleany_control_bridge cleany_mission_manager $(COLCON_BUILD_ARGS)

build-dev-monitor: build-dev-monitor-web build-dev-monitor-ros

LOGGER_TEST_PATHS ?= "$(ROS2_WS)/src/cleany_logger/tests"
LOGGER_PYTEST_ARGS ?=
DEV_MONITOR_TEST_PATHS ?= "$(ROS2_WS)/src/cleany_dev_monitor/tests"
DEV_MONITOR_PYTEST_ARGS ?=
test-logger:
	PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$(ROS2_WS)/src/cleany_logger:$${PYTHONPATH:-}" \
	python3 -m pytest -p no:cacheprovider $(LOGGER_TEST_PATHS) $(LOGGER_PYTEST_ARGS)

test-dev-monitor: test-logger
	PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$(ROS2_WS)/src/cleany_dev_monitor:$(ROS2_WS)/src/cleany_logger:$${PYTHONPATH:-}" \
	python3 -m pytest -p no:cacheprovider $(DEV_MONITOR_TEST_PATHS) $(DEV_MONITOR_PYTEST_ARGS)

DEV_MONITOR_E2E_ARGS ?=
.PHONY: test-dev-monitor-web
test-dev-monitor-web:
	pnpm --dir "$(DEV_MONITOR_WEB)" exec playwright test $(DEV_MONITOR_E2E_ARGS)

run-dev-monitor:
	source "$(ROS_SETUP)" && source "$(if $(filter /%,$(MISSION_INSTALL_BASE)),$(MISSION_INSTALL_BASE),$(ROS2_WS)/$(MISSION_INSTALL_BASE))/setup.bash" && \
	ros2 run cleany_dev_monitor dev_monitor --ros-args \
		--params-file "$(ROS2_WS)/src/cleany_dev_monitor/config/monitor.yaml" $(MONITOR_ROS_ARGS)

.PHONY: print-install-base print-navigation-install-base
print-install-base:
	@printf '%s\n' '$(INSTALL_BASE)'
print-navigation-install-base:
	@printf '%s\n' '$(NAVIGATION_INSTALL_BASE)'

.PHONY: build-logger run-logger logger
build-logger:
	source "$(ROS_SETUP)" && cd "$(ROS2_WS)" && \
	colcon --log-base "$(MISSION_LOG_BASE)" build --symlink-install \
		--build-base "$(MISSION_BUILD_BASE)" --install-base "$(MISSION_INSTALL_BASE)" \
		--packages-select cleany_interfaces cleany_logger

run-logger:
	source "$(ROS_SETUP)" && source "$(if $(filter /%,$(MISSION_INSTALL_BASE)),$(MISSION_INSTALL_BASE),$(ROS2_WS)/$(MISSION_INSTALL_BASE))/setup.bash" && \
	ros2 launch cleany_logger logger.launch.py $(LOGGER_LAUNCH_ARGS)

logger:
	source "$(ROS_SETUP)" && source "$(if $(filter /%,$(MISSION_INSTALL_BASE)),$(MISSION_INSTALL_BASE),$(ROS2_WS)/$(MISSION_INSTALL_BASE))/setup.bash" && \
	ros2 run cleany_logger logger $(LOGGER_ARGS)

.PHONY: test-logger-independence
test-logger-independence:
	@test -n "$(LOGGER_TEST_OUTPUT)" -a -n "$(LOGGER_BASE_COMMIT)" || (echo "Set LOGGER_TEST_OUTPUT and LOGGER_BASE_COMMIT" >&2; exit 2)
	python3 tools/validation/logger/independence.py --output "$(LOGGER_TEST_OUTPUT)" --base-commit "$(LOGGER_BASE_COMMIT)"

.PHONY: test-logger-cli
test-logger-cli:
	@test -n "$(LOGGER_TEST_OUTPUT)" -a -n "$(LOGGER_BASE_COMMIT)" || (echo "Set LOGGER_TEST_OUTPUT and LOGGER_BASE_COMMIT" >&2; exit 2)
	python3 tools/validation/logger/cli.py --output "$(LOGGER_TEST_OUTPUT)" --base-commit "$(LOGGER_BASE_COMMIT)"

.PHONY: test-ci-tools
test-ci-tools:
	PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests/tools/ci -v

JETSON_SOURCE_PATH := $(ROS2_WS)/src/cleany_perception:$(ROS2_WS)/src/cleany_grasping:$(ROS2_WS)/src/cleany_skill_executor:$(ROS2_WS)/src/cleany_manipulation_bt:$(ROS2_WS)/src/cleany_robot_interface:$(ROS2_WS)/src/cleany_mujoco_sim:$(ROS2_WS)/src/cleany_mujoco_observer:$(ROS2_WS)/src/cleany_handeye_calibration:$(ROS2_WS)/src/cleany_moveit_config

.PHONY: test-jetson-core
test-jetson-core:
	source "$(ROS_SETUP)" && \
	source "$(INSTALL_BASE)/setup.bash" && \
	PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONDONTWRITEBYTECODE=1 ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=221 \
	PYTHONPATH="$(JETSON_SOURCE_PATH):$${PYTHONPATH:-}" \
	python3 -m pytest -q -p no:cacheprovider --import-mode=importlib \
		$(ROS2_WS)/src/cleany_perception/test/test_byte_buffer.py \
		$(ROS2_WS)/src/cleany_perception/test/test_reconstruction_cache.py \
		$(ROS2_WS)/src/cleany_grasping/test/test_anygrasp_worker.py \
		$(ROS2_WS)/src/cleany_skill_executor/test/test_physical_grasp_selection.py \
		$(ROS2_WS)/src/cleany_manipulation_bt/test/test_stationary_feedback.py

.PHONY: test-jetson-tools
test-jetson-tools:
	source "$(ROS_SETUP)" && \
	source "$(INSTALL_BASE)/setup.bash" && \
	PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONDONTWRITEBYTECODE=1 ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=222 \
	PYTHONPATH="$(REPO_ROOT)tools:$(JETSON_SOURCE_PATH):$${PYTHONPATH:-}" \
	python3 -m pytest -q -p no:cacheprovider --import-mode=importlib tests/tools $(JETSON_PYTEST_ARGS)

.PHONY: test-jetson-packages
JETSON_TEST_PATHS ?= $(ROS2_WS)/src/cleany_interfaces/test \
	$(ROS2_WS)/src/cleany_perception/test \
	$(ROS2_WS)/src/cleany_grasping/test \
	$(ROS2_WS)/src/cleany_robot_interface/test \
	$(ROS2_WS)/src/cleany_skill_executor/test \
	$(ROS2_WS)/src/cleany_manipulation_bt/test
test-jetson-packages:
	source "$(ROS_SETUP)" && \
	source "$(INSTALL_BASE)/setup.bash" && \
	PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONDONTWRITEBYTECODE=1 ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=223 \
	PYTHONPATH="$(REPO_ROOT)tools:$(subst $(ROS2_WS)/src/cleany_manipulation_bt:,,$(JETSON_SOURCE_PATH)):$(ROS2_WS)/src/cleany_perception/test:$(ROS2_WS)/src/cleany_skill_executor/test:$(ROS2_WS)/src/cleany_manipulation_bt/test:$${PYTHONPATH:-}" \
	python3 -m pytest -q -p no:cacheprovider --import-mode=importlib \
		$(JETSON_TEST_PATHS) $(JETSON_PYTEST_ARGS)

.PHONY: test-jetson-native
test-jetson-native:
	source "$(ROS_SETUP)" && \
	source "$(INSTALL_BASE)/setup.bash" && \
	cd "$(ROS2_WS)" && \
	ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=224 \
	colcon --log-base "$(LOG_BASE)" test --build-base "$(BUILD_BASE)" \
		--install-base "$(INSTALL_BASE)" --return-code-on-test-failure \
		--packages-select cleany_scene_mapping cleany_feetech_hardware cleany_moveit_capabilities

.PHONY: build-jetson scene-mapping-setup moveit-octomap-lock-setup
build-jetson:
	source "$(ROS_SETUP)" && source "$(REPO_ROOT)tools/scene-mapping-env" && \
	$(MAKE) build COLCON_BUILD_ARGS="--packages-up-to cleany_manipulation_bt cleany_dev_monitor --parallel-workers 2 $(COLCON_BUILD_ARGS)"

scene-mapping-setup:
	"$(REPO_ROOT)tools/setup-scene-mapping"

moveit-octomap-lock-setup:
	"$(REPO_ROOT)tools/setup-moveit-octomap-lock"

.PHONY: perception-run hybrid-run motion-container-image motion-container-prepare motion-container-check
perception-run:
	"$(REPO_ROOT)tools/vision-container" perception-run
hybrid-run:
	"$(REPO_ROOT)tools/vision-container" hybrid-run
motion-container-image:
	"$(REPO_ROOT)tools/motion-container" image
motion-container-prepare:
	"$(REPO_ROOT)tools/motion-container" prepare
motion-container-check:
	"$(REPO_ROOT)tools/motion-container" check

.PHONY: collection-start
collection-start:
	"$(REPO_ROOT)tools/collection-runtime" start

.PHONY: collection-recover-control
collection-recover-control:
	"$(REPO_ROOT)tools/collection-runtime" recover-control

.PHONY: collection-recover-planning
collection-recover-planning:
	"$(REPO_ROOT)tools/collection-runtime" recover-planning

.PHONY: collection-plan
collection-plan:
	"$(REPO_ROOT)tools/collection-runtime" plan

.PHONY: collection-execute
collection-execute:
	"$(REPO_ROOT)tools/collection-runtime" execute

.PHONY: collection-run
collection-run:
	"$(REPO_ROOT)tools/collection-runtime" run

.PHONY: collection-status
collection-status:
	"$(REPO_ROOT)tools/collection-runtime" status

.PHONY: collection-logs
collection-logs:
	"$(REPO_ROOT)tools/collection-runtime" logs

.PHONY: collection-stop
collection-stop:
	"$(REPO_ROOT)tools/collection-runtime" stop

.PHONY: manipulation-start
manipulation-start:
	"$(REPO_ROOT)tools/collection-runtime" action-start

.PHONY: manipulation-run
manipulation-run:
	"$(REPO_ROOT)tools/collection-runtime" action-run

.PHONY: manipulation-restart
manipulation-restart:
	"$(REPO_ROOT)tools/collection-runtime" action-restart

.PHONY: manipulation-recover
manipulation-recover:
	"$(REPO_ROOT)tools/collection-runtime" action-recover

.PHONY: arm-return
arm-return:
	"$(REPO_ROOT)tools/collection-runtime" arm-return

.PHONY: benchmark-dev-monitor-cpu
benchmark-dev-monitor-cpu:
	source "$(ROS_SETUP)" && source "$(INSTALL_BASE)/setup.bash" && \
	python3 "$(REPO_ROOT)tools/validation/monitor/cpu.py" --output "$(MONITOR_CPU_OUTPUT)" \
		--source "$(if $(MONITOR_CPU_SOURCE),$(MONITOR_CPU_SOURCE),$(ROS2_WS)/src/cleany_dev_monitor)"

DEV_MONITOR_COMMANDS := start stop status control-on control-off control-key
.PHONY: $(addprefix dev-monitor-,$(DEV_MONITOR_COMMANDS))
$(addprefix dev-monitor-,$(DEV_MONITOR_COMMANDS)):
	"$(REPO_ROOT)tools/dev-monitor" $(patsubst dev-monitor-%,%,$@)

VISION_COMMANDS := init host-setup build feature-id
.PHONY: $(addprefix vision-,$(VISION_COMMANDS)) vision-license-check
$(addprefix vision-,$(VISION_COMMANDS)):
	"$(REPO_ROOT)tools/vision-container" $(patsubst vision-%,%,$@)
vision-license-check:
	"$(REPO_ROOT)tools/vision-container" check-license

VISION_SERVICE_COMMANDS := hybrid-config hybrid-up hybrid-down anygrasp-up anygrasp-run perception-up
.PHONY: $(VISION_SERVICE_COMMANDS)
$(VISION_SERVICE_COMMANDS):
	"$(REPO_ROOT)tools/vision-container" $@

.PHONY: yoloe-trt-build yoloe-trt-export yoloe-trt-validate
yoloe-trt-build yoloe-trt-export yoloe-trt-validate:
	python3 "$(REPO_ROOT)tools/yoloe_trt_container.py" $(patsubst yoloe-trt-%,%,$@) $(YOLOE_TRT_ARGS)

.PHONY: test-jetson-controller
test-jetson-controller:
	source "$(ROS_SETUP)" && source "$(INSTALL_BASE)/setup.bash" && \
	PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONDONTWRITEBYTECODE=1 ROS_LOCALHOST_ONLY=1 \
	python3 -m pytest -q -p no:cacheprovider \
		$(ROS2_WS)/src/cleany_robot_interface/runtime_test/test_controller_emulator.py $(JETSON_CONTROLLER_TEST_ARGS)

.PHONY: test-collision-geometry
test-collision-geometry:
	PYTHONPATH=$(ROS2_WS)/src/cleany_skill_executor $(GEOMETRY_PYTHON) -m pytest -q \
		$(ROS2_WS)/src/cleany_skill_executor/test/test_collision_geometry.py \
		$(ROS2_WS)/src/cleany_skill_executor/test/test_sphere_reduction.py \
		$(ROS2_WS)/src/cleany_skill_executor/test/test_geometry_cache.py \
		$(ROS2_WS)/src/cleany_skill_executor/test/test_robot_geometry_cache.py

.PHONY: test-arm-collision-runtime test-curobo-geometry
test-arm-collision-runtime:
	source "$(ROS_SETUP)" && source "$(INSTALL_BASE)/setup.bash" && \
	ROS_LOCALHOST_ONLY=1 CLEANY_TEST_ARM_MESH_RUNTIME=1 \
	python3 -m pytest -q -s $(ROS2_WS)/src/cleany_description/test/test_collision_meshes.py \
		$(ROS2_WS)/src/cleany_moveit_config/test/test_arm_collision_mesh_runtime.py $(COLLISION_TEST_ARGS)

test-curobo-geometry:
	source "$(ROS_SETUP)" && source "$(INSTALL_BASE)/local_setup.bash" && \
	ROS_LOCALHOST_ONLY=1 CLEANY_RUN_CUROBO=1 \
	PYTHONPATH=$(ROS2_WS)/src/cleany_skill_executor:$${PYTHONPATH:-} \
	python3 -m pytest -q -s $(CUROBO_GPU_TEST_FILE) $(COLLISION_TEST_ARGS)

.PHONY: test-curobo-model
test-curobo-model:
	source "$(ROS_SETUP)" && source "$(INSTALL_BASE)/local_setup.bash" && \
	PYTHONPATH=$(ROS2_WS)/src/cleany_skill_executor:$${PYTHONPATH:-} \
	python3 -m pytest -q $(ROS2_WS)/src/cleany_skill_executor/test/test_curobo.py \
		$(ROS2_WS)/src/cleany_skill_executor/test/test_curobo_batch.py \
		$(ROS2_WS)/src/cleany_skill_executor/test/test_hybrid_adapter.py \
		$(ROS2_WS)/src/cleany_skill_executor/test/test_hybrid_validation_order.py $(COLLISION_TEST_ARGS)
