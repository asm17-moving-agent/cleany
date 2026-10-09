SHELL := /bin/bash

# Keep unrelated site-package pytest plugins out of ROS test discovery and runs.
export PYTEST_DISABLE_PLUGIN_AUTOLOAD := 1

REPO_ROOT := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
ROS2_WS := $(REPO_ROOT)ros2_ws
ROS_SETUP := /opt/ros/humble/setup.bash
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

GRASP_PREGRASP_PACKAGES := cleany_interfaces cleany_description \
	cleany_mujoco_sim cleany_mujoco_observer cleany_moveit_config cleany_perception \
	cleany_grasping cleany_skill_executor cleany_scene_mapping
CLEANY_ROS_PERCEPTION_PREFIX ?= $(HOME)/.local/share/cleany/moveit-perception/opt/ros/humble
CLEANY_BT_PREFIX ?= $(HOME)/.local/share/cleany/behaviortree-cpp/opt/ros/humble
define use_local_behaviortree
if test -d "$(CLEANY_BT_PREFIX)/share/behaviortree_cpp"; then \
  export AMENT_PREFIX_PATH="$(CLEANY_BT_PREFIX):$${AMENT_PREFIX_PATH}"; \
  export CMAKE_PREFIX_PATH="$(CLEANY_BT_PREFIX):$${CMAKE_PREFIX_PATH}"; \
  export LD_LIBRARY_PATH="$(CLEANY_BT_PREFIX)/lib:$${LD_LIBRARY_PATH}"; \
fi
endef
define use_local_moveit_perception
if ! ros2 pkg prefix moveit_ros_perception >/dev/null 2>&1 && \
    test -d "$(CLEANY_ROS_PERCEPTION_PREFIX)/share/moveit_ros_perception"; then \
  export AMENT_PREFIX_PATH="$(CLEANY_ROS_PERCEPTION_PREFIX):$${AMENT_PREFIX_PATH}"; \
  export CMAKE_PREFIX_PATH="$(CLEANY_ROS_PERCEPTION_PREFIX):$${CMAKE_PREFIX_PATH}"; \
  export LD_LIBRARY_PATH="$(CLEANY_ROS_PERCEPTION_PREFIX)/lib:$${LD_LIBRARY_PATH}"; \
fi
endef
SORTING_ARGS ?=
GRASP_PREGRASP_SKILL_TESTS := \
	src/cleany_skill_executor/test/test_can_rgbd.py \
	src/cleany_skill_executor/test/test_grasp_execution_demo_contract.py \
	src/cleany_skill_executor/test/test_reobservation.py \
	src/cleany_skill_executor/test/test_seeded_cartesian.py \
	src/cleany_skill_executor/test/test_gripper_geometry.py \
	src/cleany_skill_executor/test/test_cartesian.py \
	src/cleany_skill_executor/test/test_service_trace.py \
	src/cleany_skill_executor/test/test_sorting.py \
	src/cleany_skill_executor/test/test_sorting_coordinator.py \
	src/cleany_skill_executor/test/test_learned_runtime_launch.py \
	src/cleany_skill_executor/test/test_sensor_scene.py \
	src/cleany_skill_executor/test/test_rgbd_projection.py \
	src/cleany_skill_executor/test/test_nearest_object.py \
	src/cleany_skill_executor/test/test_grasp_selection.py \
	src/cleany_skill_executor/test/test_grasp_selection_node.py \
	src/cleany_skill_executor/test/test_moveit_adapter.py \
	src/cleany_skill_executor/test/test_planning_scene.py
GRASP_PREGRASP_MOVEIT_TESTS := \
	src/cleany_moveit_config/test/test_simulation_collision.py \
	src/cleany_moveit_config/test/test_study_cafe_collision_scene.py \
	src/cleany_moveit_config/test/test_moveit_config.py
GRASP_PREGRASP_MUJOCO_TESTS := \
	src/cleany_mujoco_sim/test/test_tabletop_performance.py \
	src/cleany_mujoco_sim/test/test_study_cafe_scene.py \
	src/cleany_mujoco_sim/test/test_tabletop_shapes.py \
	src/cleany_mujoco_sim/test/test_placement_verifier.py \
	src/cleany_mujoco_sim/test/test_sorting_scene.py \
	src/cleany_mujoco_sim/test/test_study_cafe_backend_config.py
GRASP_PREGRASP_DESCRIPTION_TESTS := \
	src/cleany_description/test/test_model_parity.py

.PHONY: anygrasp-feature-id anygrasp-license-check anygrasp-run anygrasp-shell anygrasp-up \
	build build-base build-gazebo build-gazebo-harmonic build-grasp-pregrasp \
	build-handeye build-manipulation build-manipulation-bt build-mujoco-observer \
	build-scene-mapping build-telemetry check-gazebo-env clean deps deps-gazebo \
	firmware-build firmware-setup firmware-smoke firmware-upload \
	handeye-generate-mujoco handeye-mujoco handeye-validate-mujoco help hybrid-config \
	hybrid-down hybrid-run hybrid-up micro-ros-agent-build perception-run \
	perception-shell perception-up profile-mujoco-tabletop profile-scene-mask sim \
	sim-gazebo sim-gazebo-harmonic sim-gazebo-office sim-gazebo-study-cafe \
	sim-mujoco-manipulation sim-mujoco-pipeline sim-mujoco-sorting \
	sim-mujoco-study-cafe test test-base test-gazebo test-gazebo-evaluation \
	test-gazebo-harmonic test-gazebo-nav-runtime test-gazebo-safety \
	test-grasp-pregrasp test-grasp-pregrasp-runtime test-handeye test-manipulation \
	test-manipulation-bt test-manipulation-core test-micro-ros-setup test-mission \
	test-motor-core test-mujoco test-mujoco-observer test-scene-mapping test-telemetry \
	view-gazebo-costmap vision-build vision-config vision-down vision-feature-id \
	vision-host-setup vision-init vision-license-check vision-run vision-shell \
	vision-up

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
	@echo "  make build-grasp-pregrasp  Build RGB-D grasp/pre-grasp packages"
	@echo "  make build-manipulation  Build mock manipulation Action packages"
	@echo "  make build-manipulation-bt  Build native BT and MuJoCo Action packages"
	@echo "  make test-manipulation-bt  Test real BT with fake operations and ROS boundaries"
	@echo "  make sim-mujoco-manipulation  Run single-object MuJoCo BT Action stack"
	@echo "  make test-manipulation-core  Run manipulation core tests with a fake clock"
	@echo "  make test-manipulation  Build/test mock Action, DDS traffic and crash recovery"
	@echo "  make test-grasp-pregrasp  Run focused RGB-D grasp/pre-grasp tests"
	@echo "  make test-scene-mapping  Build/test optional known-geometry OctoMap updater"
	@echo "  make test-mujoco-observer  Build/test read-only contact observation"
	@echo "  make test-grasp-pregrasp-runtime  Run the MoveIt mock execution test"
	@echo "  make sim-mujoco-study-cafe  Run the study cafe in MuJoCo"
	@echo "  make sim-mujoco-pipeline  Run YOLOE-seg + Gemini GUI (API key, plan-only)"
	@echo "  make sim-mujoco-sorting   Run simulation rule-based pick/sort/place"
	@echo "  make profile-mujoco-tabletop  Compare opt-in tabletop physics/render profiles"
	@echo "  make vision-init   Install /etc/cleany/jetson-identity.env (sudo)"
	@echo "  make vision-host-setup  Enable Docker bridge networking on Jetson (sudo)"
	@echo "  make vision-build  Build the Jetson vision development image"
	@echo "  make anygrasp-up/run    Start/run the isolated AnyGrasp service"
	@echo "  make perception-up/run Start/run the perception service"
	@echo "  make hybrid-up/down     Start/stop all implemented GPU services"
	@echo "  make vision-feature-id Verify the pinned AnyGrasp ID (compatibility alias)"

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
	$(use_local_moveit_perception) && \
	$(use_local_behaviortree) && \
	cd "$(ROS2_WS)" && \
	colcon build --symlink-install $(COLCON_BUILD_ARGS)

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

# Re-register CMake tests that an earlier broken pytest environment omitted.
test: COLCON_BUILD_ARGS += --cmake-force-configure
test: build
	source "$(ROS_SETUP)" && \
	$(use_local_behaviortree) && \
	cd "$(ROS2_WS)" && \
	source install/setup.bash && \
	$(use_local_moveit_perception) && \
	colcon test --python-testing pytest && \
	colcon test-result --verbose && \
	python3 -m pytest "$(REPO_ROOT)tools/test_gazebo_profile.py" \
		"$(REPO_ROOT)containers/vision/test"

test-mission: build
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source install/setup.bash && \
	python3 -m pytest src/cleany_mission_manager/tests/test_mission_flow.py

test-mujoco: build
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source install/setup.bash && \
	export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 && \
	python3 -m pytest src/cleany_mujoco_sim/test

test-handeye: build-handeye
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source install/setup.bash && \
	colcon test --python-testing pytest --packages-select $(HANDEYE_PACKAGES) \
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
build-manipulation:
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	colcon build --symlink-install --packages-select cleany_interfaces cleany_skill_executor

build-manipulation-bt:
	source "$(ROS_SETUP)" && \
	$(use_local_moveit_perception) && \
	$(use_local_behaviortree) && \
	cd "$(ROS2_WS)" && \
	colcon build --symlink-install --packages-up-to cleany_manipulation_bt $(COLCON_BUILD_ARGS)

test-manipulation-bt: build-manipulation-bt
	source "$(ROS_SETUP)" && \
	$(use_local_behaviortree) && \
	cd "$(ROS2_WS)" && \
	source install/setup.bash && \
	python3 -m pytest -q src/cleany_manipulation_bt/test \
		src/cleany_perception/test/test_inspection_node.py \
		src/cleany_skill_executor/test/test_sorting_coordinator.py \
		src/cleany_skill_executor/test/test_nearest_object.py \
		src/cleany_skill_executor/test/test_learned_runtime_launch.py

sim-mujoco-manipulation: build-manipulation-bt
	source "$(ROS_SETUP)" && \
	$(use_local_moveit_perception) && \
	$(use_local_behaviortree) && \
	cd "$(ROS2_WS)" && \
	source install/setup.bash && \
	ros2 launch cleany_manipulation_bt manipulation_mujoco.launch.py $(SORTING_ARGS)

test-manipulation-core:
	PYTHONPATH="$(ROS2_WS)/src/cleany_skill_executor:$${PYTHONPATH}" \
	python3 -m pytest -q -p no:cacheprovider \
		"$(ROS2_WS)/src/cleany_skill_executor/test/test_manipulation_core.py" \
		"$(ROS2_WS)/src/cleany_skill_executor/test/test_manipulation_monitor.py"

test-manipulation: build-manipulation
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source install/setup.bash && \
	python3 -m pytest -q src/cleany_interfaces/test/test_interface_contract.py \
		src/cleany_skill_executor/test/test_manipulation_core.py \
		src/cleany_skill_executor/test/test_manipulation_runtime.py \
		src/cleany_skill_executor/test/test_manipulation_monitor.py \
		src/cleany_skill_executor/test/test_manipulation_monitor_runtime.py

build-grasp-pregrasp:
	source "$(ROS_SETUP)" && \
	$(use_local_moveit_perception) && \
	cd "$(ROS2_WS)" && \
	colcon build --symlink-install \
		--packages-up-to $(GRASP_PREGRASP_PACKAGES)

build-scene-mapping:
	source "$(ROS_SETUP)" && \
	$(use_local_moveit_perception) && \
	cd "$(ROS2_WS)" && \
	colcon build --symlink-install --packages-select cleany_scene_mapping

test-scene-mapping: build-scene-mapping
	source "$(ROS_SETUP)" && \
	$(use_local_moveit_perception) && \
	cd "$(ROS2_WS)" && \
	source install/setup.bash && \
	colcon test --packages-select cleany_scene_mapping --event-handlers console_direct+ && \
	colcon test-result --test-result-base build/cleany_scene_mapping --verbose

build-mujoco-observer:
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	colcon build --symlink-install --packages-select cleany_mujoco_observer

test-mujoco-observer: build-mujoco-observer
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source install/setup.bash && \
	colcon test --packages-select cleany_mujoco_observer --event-handlers console_direct+ && \
	colcon test-result --test-result-base build/cleany_mujoco_observer --verbose

profile-scene-mask: build-scene-mapping
	source "$(ROS_SETUP)" && \
	source "$(ROS2_WS)/install/setup.bash" && \
	$(use_local_moveit_perception) && \
	"$(ROS2_WS)/build/cleany_scene_mapping/profile_mask" "$(MASK_FIXTURE)"

test-grasp-pregrasp: build-grasp-pregrasp
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source install/setup.bash && \
	$(use_local_moveit_perception) && \
	export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 && \
	colcon test --packages-select cleany_scene_mapping cleany_mujoco_observer --event-handlers console_cohesion+ && \
	colcon test-result --test-result-base build/cleany_scene_mapping --verbose && \
	colcon test-result --test-result-base build/cleany_mujoco_observer --verbose && \
	python3 -m pytest -q \
		src/cleany_interfaces/test/test_interface_contract.py && \
	python3 -m pytest -q src/cleany_perception/test && \
	python3 -m pytest -q src/cleany_grasping/test && \
	python3 -m pytest -q $(GRASP_PREGRASP_SKILL_TESTS) && \
	python3 -m pytest -q $(GRASP_PREGRASP_MOVEIT_TESTS) && \
	python3 -m pytest -q $(GRASP_PREGRASP_MUJOCO_TESTS) && \
	python3 -m pytest -q $(GRASP_PREGRASP_DESCRIPTION_TESTS)

test-grasp-pregrasp-runtime: build-grasp-pregrasp
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source install/setup.bash && \
	export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 && \
	python3 -m pytest -q -s \
		src/cleany_moveit_config/test/test_mock_planning_runtime.py

sim-mujoco-study-cafe: build
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source install/setup.bash && \
	ros2 launch cleany_mujoco_sim mujoco_study_cafe.launch.py headless:=false

profile-mujoco-tabletop: build-grasp-pregrasp
	source "$(ROS_SETUP)" && \
	source "$(ROS2_WS)/install/setup.bash" && \
	python3 "$(ROS2_WS)/src/cleany_mujoco_sim/tools/benchmark_tabletop.py" $(BENCHMARK_ARGS)

sim-mujoco-sorting sim-mujoco-pipeline: build-grasp-pregrasp
	source "$(ROS_SETUP)" && \
	source "$(ROS2_WS)/install/setup.bash" && \
	if ! ros2 pkg prefix moveit_ros_perception >/dev/null 2>&1 && \
		test -d "$(CLEANY_ROS_PERCEPTION_PREFIX)/share/moveit_ros_perception"; then \
		echo "Using user-local MoveIt perception: $(CLEANY_ROS_PERCEPTION_PREFIX)"; \
		export AMENT_PREFIX_PATH="$(CLEANY_ROS_PERCEPTION_PREFIX):$${AMENT_PREFIX_PATH}"; \
		export LD_LIBRARY_PATH="$(CLEANY_ROS_PERCEPTION_PREFIX)/lib:$${LD_LIBRARY_PATH}"; \
	fi && \
	ros2 launch cleany_skill_executor \
		$(if $(filter sim-mujoco-sorting,$@),study_cafe_sorting.launch.py $(SORTING_ARGS),study_cafe_nearest_grasp_demo.launch.py)

vision-init:
	"$(REPO_ROOT)tools/vision-container" init

vision-host-setup:
	"$(REPO_ROOT)tools/vision-container" host-setup

vision-config:
	$(MAKE) hybrid-config

vision-build:
	"$(REPO_ROOT)tools/vision-container" build

vision-up:
	$(MAKE) hybrid-up

vision-down:
	$(MAKE) hybrid-down

vision-shell:
	$(MAKE) anygrasp-shell

vision-feature-id:
	$(MAKE) anygrasp-feature-id

vision-license-check:
	$(MAKE) anygrasp-license-check

vision-run:
	$(MAKE) hybrid-run

anygrasp-up:
	"$(REPO_ROOT)tools/vision-container" anygrasp-up

anygrasp-run:
	"$(REPO_ROOT)tools/vision-container" anygrasp-run

anygrasp-shell:
	"$(REPO_ROOT)tools/vision-container" anygrasp-shell

anygrasp-feature-id:
	"$(REPO_ROOT)tools/vision-container" feature-id

anygrasp-license-check:
	"$(REPO_ROOT)tools/vision-container" check-license

perception-up:
	"$(REPO_ROOT)tools/vision-container" perception-up

perception-run:
	"$(REPO_ROOT)tools/vision-container" perception-run

perception-shell:
	"$(REPO_ROOT)tools/vision-container" perception-shell

hybrid-config:
	"$(REPO_ROOT)tools/vision-container" hybrid-config

hybrid-up:
	"$(REPO_ROOT)tools/vision-container" hybrid-up

hybrid-run:
	"$(REPO_ROOT)tools/vision-container" hybrid-run

hybrid-down:
	"$(REPO_ROOT)tools/vision-container" hybrid-down
