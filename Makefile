SHELL := /bin/bash

# Keep unrelated user-installed pytest plugins out of ROS/system Python tests.
export PYTEST_DISABLE_PLUGIN_AUTOLOAD ?= 1

REPO_ROOT := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
ROS2_WS := $(REPO_ROOT)ros2_ws
ROS_SETUP := /opt/ros/humble/setup.bash
COLCON_BUILD_ARGS ?=
GAZEBO_PROFILE_TOOL := $(REPO_ROOT)tools/gazebo_profile.py
GAZEBO_GUI_RENDER_ENGINE ?= ogre
SAFETY_MODE ?= sensors
ROBOT_MODEL ?= cad_frame
GRASP_PREGRASP_PACKAGES := cleany_interfaces cleany_description \
	cleany_mujoco_sim cleany_mujoco_observer cleany_moveit_config cleany_perception \
	cleany_grasping cleany_skill_executor cleany_scene_mapping
CLEANY_ROS_PERCEPTION_PREFIX ?= $(HOME)/.local/share/cleany/moveit-perception/opt/ros/humble
define use_local_moveit_perception
if ! ros2 pkg prefix moveit_ros_perception >/dev/null 2>&1 && \
    test -d "$(CLEANY_ROS_PERCEPTION_PREFIX)/share/moveit_ros_perception"; then \
  export AMENT_PREFIX_PATH="$(CLEANY_ROS_PERCEPTION_PREFIX):$${AMENT_PREFIX_PATH}"; \
  export CMAKE_PREFIX_PATH="$(CLEANY_ROS_PERCEPTION_PREFIX):$${CMAKE_PREFIX_PATH}"; \
  export LD_LIBRARY_PATH="$(CLEANY_ROS_PERCEPTION_PREFIX)/lib:$${LD_LIBRARY_PATH}"; \
fi
endef
SORTING_ARGS ?=
PIPELINE_ARGS ?=
SIM_ARGS ?=
GRASP_PREGRASP_SKILL_TESTS := \
	\
	src/cleany_skill_executor/test/test_reobservation.py \
	src/cleany_skill_executor/test/test_seeded_cartesian.py \
	src/cleany_skill_executor/test/test_gripper_geometry.py \
	src/cleany_skill_executor/test/test_cartesian.py \
	src/cleany_skill_executor/test/test_service_trace.py \
	src/cleany_skill_executor/test/test_sorting.py \
	src/cleany_skill_executor/test/test_sorting_coordinator.py \
	src/cleany_skill_executor/test/test_wrist_recovery.py \
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

.PHONY: help deps deps-gazebo check-gazebo-env build build-gazebo profile-scene-mask profile-mujoco-tabletop profile-mujoco-runtime \
	build-grasp-pregrasp build-scene-mapping test-scene-mapping test \
	build-mujoco-observer test-mujoco-observer \
	test-mission test-mujoco test-grasp-pregrasp \
	test-grasp-pregrasp-runtime test-gazebo test-gazebo-nav-runtime \
	test-gazebo-evaluation \
	sim sim-gazebo \
	sim-mujoco-study-cafe sim-gazebo-study-cafe clean \
	sim-mujoco-pipeline sim-mujoco-sorting \
	vision-init vision-host-setup vision-config vision-build vision-up vision-down vision-shell \
	vision-feature-id vision-license-check vision-run \
	anygrasp-up anygrasp-run anygrasp-shell anygrasp-feature-id anygrasp-license-check \
	perception-up perception-run perception-shell \
	hybrid-config hybrid-up hybrid-run hybrid-down

.PHONY: build-gazebo-harmonic build-telemetry test-telemetry \
	test-gazebo-harmonic test-gazebo-safety view-gazebo-costmap \
	sim-gazebo-harmonic sim-gazebo-office

help:
	@echo "Cleany native ROS 2 commands"
	@echo "  make deps          Install workspace dependencies with rosdep"
	@echo "  make deps-gazebo   Install dependencies for the detected Gazebo profile"
	@echo "  make check-gazebo-env  Detect and verify Humble/Fortress or Jazzy/Harmonic"
	@echo "  make build         Build the ROS 2 workspace"
	@echo "  make build-gazebo  Build the detected Gazebo profile"
	@echo "  make build-grasp-pregrasp  Build RGB-D grasp/pre-grasp packages"
	@echo "  make test          Build and run all colcon tests"
	@echo "  make test-mission  Run Mission Manager pytest"
	@echo "  make test-mujoco   Run MuJoCo simulation pytest"
	@echo "  make test-grasp-pregrasp  Run focused RGB-D grasp/pre-grasp tests"
	@echo "  make test-scene-mapping  Build/test optional known-geometry OctoMap updater"
	@echo "  make test-mujoco-observer  Build/test read-only contact observation"
	@echo "  make test-grasp-pregrasp-runtime  Run the MoveIt mock execution test"
	@echo "  make build-telemetry Build the ROS telemetry package"
	@echo "  make test-telemetry Test the ROS telemetry package"
	@echo "  make test-gazebo   Test the detected Gazebo profile"
	@echo "  make test-gazebo-nav-runtime  Run LiDAR, IMU, odom, and TF runtime test"
	@echo "  make test-gazebo-evaluation  Run temporary SLAM evaluation checks"
	@echo "  make test-gazebo-safety  Run SCRUM-306 sensors/monitor/avoid evaluation"
	@echo "  make view-gazebo-costmap  Show live costmap over saved SLAM map in RViz"
	@echo "  make test-gazebo-harmonic  Compatibility alias selecting Harmonic"
	@echo "  make sim           Build and run the headless MuJoCo simulation"
	@echo "  make sim-mujoco-study-cafe  Run the study cafe in MuJoCo"
	@echo "  make sim-mujoco-pipeline  Run YOLOE-seg + Gemini GUI (API key, plan-only)"
	@echo "  make sim-mujoco-sorting   Run simulation rule-based pick/sort/place"
	@echo "  make profile-mujoco-tabletop  Compare opt-in tabletop physics/render profiles"
	@echo "  make profile-mujoco-runtime   Measure pipeline clock/RGB-D/CPU, then stop it"
	@echo "  make sim-gazebo    Build and run the detected Gazebo profile"
	@echo "  make sim-gazebo-harmonic  Compatibility alias selecting Harmonic"
	@echo "  make sim-gazebo-study-cafe  Run the spacious study cafe with GUI"
	@echo "  make vision-init   Install /etc/cleany/jetson-identity.env (sudo)"
	@echo "  make vision-host-setup  Enable Docker bridge networking on Jetson (sudo)"
	@echo "  make vision-build  Build the Jetson vision development image"
	@echo "  make anygrasp-up/run    Start/run the isolated AnyGrasp service"
	@echo "  make perception-up/run Start/run the perception service"
	@echo "  make hybrid-up/down     Start/stop all implemented GPU services"
	@echo "  make vision-feature-id Verify the pinned AnyGrasp ID (compatibility alias)"
	@echo "  make clean         Remove ROS 2 build, install, and log outputs"

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

build-telemetry:
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	colcon build --symlink-install --packages-select cleany_telemetry

test-telemetry: build-telemetry
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source install/setup.bash && \
	python3 -m pytest src/cleany_telemetry/test

test: COLCON_BUILD_ARGS += --cmake-force-configure
test: build
	source "$(ROS_SETUP)" && \
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
	ros2 launch cleany_mujoco_sim mujoco_study_cafe.launch.py headless:=true $(SIM_ARGS)

sim-mujoco-study-cafe: build
	source "$(ROS_SETUP)" && \
	cd "$(ROS2_WS)" && \
	source install/setup.bash && \
	ros2 launch cleany_mujoco_sim mujoco_study_cafe.launch.py headless:=false $(SIM_ARGS)

profile-mujoco-tabletop: build-grasp-pregrasp
	source "$(ROS_SETUP)" && \
	source "$(ROS2_WS)/install/setup.bash" && \
	python3 "$(ROS2_WS)/src/cleany_mujoco_sim/tools/benchmark_tabletop.py" $(BENCHMARK_ARGS)

profile-mujoco-runtime: build-grasp-pregrasp
	source "$(ROS_SETUP)" && \
	source "$(ROS2_WS)/install/setup.bash" && \
	$(use_local_moveit_perception) && \
	python3 "$(ROS2_WS)/src/cleany_mujoco_sim/tools/benchmark_runtime.py" \
		$(BENCHMARK_ARGS) -- ros2 launch cleany_skill_executor \
		study_cafe_nearest_grasp_demo.launch.py $(PIPELINE_ARGS)

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
		$(if $(filter sim-mujoco-sorting,$@),study_cafe_sorting.launch.py $(SORTING_ARGS),study_cafe_nearest_grasp_demo.launch.py $(PIPELINE_ARGS))

sim-gazebo: build-gazebo
	eval "$$(python3 "$(GAZEBO_PROFILE_TOOL)" --shell)" && \
	source "$${CLEANY_ROS_SETUP}" && \
	cd "$(ROS2_WS)" && \
	source "$${CLEANY_INSTALL_BASE}/setup.bash" && \
	ros2 launch cleany_gazebo_sim "$${CLEANY_GAZEBO_LAUNCH}" headless:=true

sim-gazebo-harmonic:
	$(MAKE) GAZEBO_PROFILE=harmonic sim-gazebo

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

clean:
	"$(REPO_ROOT)tools/ros2-clean"
