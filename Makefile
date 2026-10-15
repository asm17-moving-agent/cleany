SHELL := /bin/bash

REPO_ROOT := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
ROS2_WS := $(REPO_ROOT)ros2_ws
ROS_SETUP := /opt/ros/humble/setup.bash
# Keep data and generated outputs outside the source worktree.
CLEANY_DATA_ROOT ?= $(abspath $(REPO_ROOT)/../cleany-data)
CLEANY_BUILD_ROOT ?= $(CLEANY_DATA_ROOT)/builds/$(notdir $(patsubst %/,%,$(REPO_ROOT)))
export CLEANY_DATA_ROOT CLEANY_BUILD_ROOT
BUILD_BASE ?= $(CLEANY_BUILD_ROOT)/build
INSTALL_BASE ?= $(CLEANY_BUILD_ROOT)/install
LOG_BASE ?= $(CLEANY_BUILD_ROOT)/log
NAVIGATION_BUILD_BASE ?= $(CLEANY_BUILD_ROOT)/build-navigation
NAVIGATION_INSTALL_BASE ?= $(CLEANY_BUILD_ROOT)/install-navigation
NAVIGATION_LOG_BASE ?= $(CLEANY_BUILD_ROOT)/log-navigation
MISSION_PACKAGES := cleany_interfaces cleany_mission_manager cleany_control_bridge
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
	micro-ros-agent-build test-micro-ros-setup

help:
	@echo "Cleany native ROS 2 commands"
	@echo "  make deps          Install workspace dependencies with rosdep"
	@echo "  make deps-gazebo   Install Gazebo Fortress dependencies with rosdep"
	@echo "  make check-gazebo-env  Verify Ubuntu 22.04 / Humble / Fortress environment"
	@echo "  make build         Build the ROS 2 workspace"
	@echo "  make build-gazebo  Build cleany_gazebo_sim and its dependencies"
	@echo "  make build-navigation  Build common navigation without Gazebo"
	@echo "  make test-navigation  Test common navigation including C++ plugins"
	@echo "  make test-navigation-runtime  Test Nav2 serial safety with a synthetic backend"
	@echo "  make build-handeye Build hand-eye packages and dependencies"
	@echo "  make build-telemetry Build the ROS telemetry package"
	@echo "  make test          Build and run all colcon tests"
	@echo "  make test-mission  Run Mission Manager pytest"
	@echo "  make build-dev-monitor / run-dev-monitor  Build or serve the read-only robot monitor"
	@echo "  make test-dev-monitor  Test bounded recording, HTTP API and ROS observation"
	@echo "  make test-mission-core  Run ROS-independent FSM, BT and bridge tests"
	@echo "  make build-mission-runtime  Build interfaces, FSM and Backend bridge"
	@echo "  make build-mission-sim  Build Gazebo + navigation + runtime + pose relay"
	@echo "  make test-mission-runtime  Build and test mission ROS packages"
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
	@echo "  make micro-ros-agent-build  Build pinned Agent and check --help"
	@echo "  make firmware-upload CLEANY_ESP_PORT=/dev/... CONFIRM_UPLOAD=1"

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

build-base:
	$(BASE_EXEC) bash -lc 'source /opt/ros/humble/setup.bash && cd "$(ROS2_WS)" && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon --log-base "$(LOG_BASE)" build --symlink-install --build-base "$(BUILD_BASE)" --install-base "$(INSTALL_BASE)" --packages-up-to cleany_base_driver cleany_base_odometry'

test-base: build-base
	$(BASE_EXEC) bash -lc 'source /opt/ros/humble/setup.bash && cd "$(ROS2_WS)" && source "$(INSTALL_BASE)/setup.bash" && export ROS_DOMAIN_ID=$(BASE_TEST_ROS_DOMAIN_ID) && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon --log-base "$(LOG_BASE)" test --build-base "$(BUILD_BASE)" --install-base "$(INSTALL_BASE)" --packages-select cleany_base_interfaces cleany_base_driver cleany_base_odometry && for p in cleany_base_interfaces cleany_base_driver cleany_base_odometry; do colcon --log-base "$(LOG_BASE)" test-result --test-result-base "$(BUILD_BASE)/$$p" --verbose || exit 1; done && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest src/cleany_base_driver/test src/cleany_base_odometry/test'

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
	colcon --log-base "$(LOG_BASE)" build --symlink-install --build-base "$(BUILD_BASE)" --install-base "$(INSTALL_BASE)"

build-gazebo: check-gazebo-env
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	colcon --log-base "$(LOG_BASE)" build --symlink-install \
		--build-base "$(BUILD_BASE)" \
		--install-base "$(INSTALL_BASE)" \
		--packages-up-to cleany_gazebo_sim

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

test: build
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source "$(INSTALL_BASE)/setup.bash" && \
	colcon --log-base "$(LOG_BASE)" test --build-base "$(BUILD_BASE)" --install-base "$(INSTALL_BASE)" --return-code-on-test-failure && \
	colcon --log-base "$(LOG_BASE)" test-result --test-result-base "$(BUILD_BASE)" --verbose

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
		--run-sim-runtime --sim-profile=fortress

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
.PHONY: build-dev-monitor-web build-dev-monitor-ros build-dev-monitor test-dev-monitor run-dev-monitor
build-dev-monitor-web:
	pnpm --dir "$(DEV_MONITOR_WEB)" install --frozen-lockfile
	pnpm --dir "$(DEV_MONITOR_WEB)" build

build-dev-monitor-ros:
	@test -f "$(DEV_MONITOR_WEB)/dist/index.html" || (echo "Run make build-dev-monitor-web first" >&2; exit 2)
	source "$(ROS_SETUP)" && cd "$(ROS2_WS)" && \
	colcon --log-base "$(MISSION_LOG_BASE)" build --symlink-install \
		--build-base "$(MISSION_BUILD_BASE)" --install-base "$(MISSION_INSTALL_BASE)" \
		--packages-select cleany_interfaces cleany_control_bridge cleany_mission_manager cleany_dev_monitor

build-dev-monitor: build-dev-monitor-web build-dev-monitor-ros

test-dev-monitor:
	PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$(ROS2_WS)/src/cleany_dev_monitor:$${PYTHONPATH:-}" \
	python3 -m pytest -p no:cacheprovider "$(ROS2_WS)/src/cleany_dev_monitor/tests"

run-dev-monitor:
	source "$(ROS_SETUP)" && source "$(if $(filter /%,$(MISSION_INSTALL_BASE)),$(MISSION_INSTALL_BASE),$(ROS2_WS)/$(MISSION_INSTALL_BASE))/setup.bash" && \
	ros2 run cleany_dev_monitor dev_monitor --ros-args \
		--params-file "$(ROS2_WS)/src/cleany_dev_monitor/config/monitor.yaml" $(MONITOR_ROS_ARGS)

.PHONY: print-install-base print-navigation-install-base
print-install-base:
	@printf '%s\n' '$(INSTALL_BASE)'
print-navigation-install-base:
	@printf '%s\n' '$(NAVIGATION_INSTALL_BASE)'
