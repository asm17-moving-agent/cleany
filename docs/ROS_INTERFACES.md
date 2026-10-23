# ROS 인터페이스 색인

각 노드의 발행, 구독, 서비스 제공/호출, 액션 제공/호출을 찾는 색인이다. 상세 표는 코드 옆 `docs/interfaces.md`가 관리하고, 동작과 안전 규칙은 [INTEGRATION](INTEGRATION.md)의 계약 reference를 따른다.

표는 현재 소스와 launch의 기본 이름을 기록한다. 실행별 namespace, node name, parameter와 remap에 따라 실제 graph는 달라진다. 조건부 endpoint, 외부 server, 미구현 타입을 구분하며 모든 노드가 동시에 실행되는 것으로 해석하지 않는다.

## 표 읽기

| 표기 | 뜻 |
|---|---|
| 토픽 발행 / 구독 | 해당 노드에서 나가는 데이터 / 들어오는 데이터 |
| 서비스 제공 / 호출 | Server / client |
| 액션 제공 / 호출 | Action server / client |
| R10, R20, R1 | Reliable, volatile, keep-last depth 10 / 20 / 1 |
| B1 | Best-effort, volatile, keep-last depth 1 |
| Sensor | Humble `qos_profile_sensor_data`: best-effort, volatile, keep-last depth 5 |
| R/T1, R/T2048 | Reliable, transient-local, keep-last depth 1 / 2048 |
| Service 기본값 | Humble service QoS: reliable, volatile, keep-last depth 10 |
| Action 기본값 | rclpy 기본 request/result/cancel service와 feedback/status QoS. Status는 reliable, transient-local, depth 1 |
| TF 기본 QoS | tf2_ros broadcaster/listener 기본값. 응용 코드가 별도 QoS를 지정하지 않음 |
| 상대 이름 / `/절대 이름` / `~/private` | Node namespace 적용 / root 이름 / node namespace와 node name 적용 |
| 주기 | 소스의 기본 timer 또는 callback 조건. 실제 수신 빈도와 동일하지 않을 수 있음 |

## 종류별 통합 목록

[Topic](#topic-목록), [Action](#action-목록), [Service](#service-목록)에서 이름으로 찾는다. 노드 링크는 패키지의 상세 표로 연결하며, 아직 표에 없는 runtime endpoint는 구현 소스로 연결한다. QoS와 호출 조건은 연결된 문서와 소스를 따른다.

아래 이름은 **root namespace에서 기본 이름을 해석한 값**이다. 상대 이름을 절대 이름으로 고정하는 설정이 아니며, namespace, remap과 parameter override가 있으면 달라진다. `cleany_dev_monitor`의 `runtime_namespace`는 기본 빈 값으로 표시했다. `<...>`는 실행 시 결정되는 이름 패턴이다.

발행/제공 목록의 여러 노드는 실행 구성에 따른 대안이거나 서로 다른 데이터를 같은 토픽으로 보내는 노드다. 동시에 실행한다는 뜻은 아니다. `mock_*`, Gazebo, MuJoCo와 `sorting_cameras`는 mock 또는 시뮬레이션 경로이며, 외부 Nav2, MoveIt과 controller는 별도 실행이 필요하다. `—`는 이 색인에서 상대 노드를 지정하지 않은 항목이며, endpoint의 부재나 미구현을 뜻하지 않는다. `SortingObserver`는 MuJoCo host 노드에 붙는 plugin이며 독립 노드 이름이 아니다. Monitor의 동적 구독은 discovery와 사용자 선택에 따라 생성된다.

ROS 자동 인터페이스는 [별도 표](#ros가-자동-생성하는-인터페이스)에 둔다. 응용 코드가 직접 호출하거나 관찰하는 action cancel/status와 카메라 parameter 서비스는 통합 목록에도 표시한다. 외부 패키지 전체 API와 실행 중 동적으로 발견되는 모든 이름을 열거하지는 않는다.

### Topic 목록

| 기본 이름 (root namespace) | ROS 타입 | 발행 노드 | 구독 노드 | 조건 / 연결 |
|---|---|---|---|---|
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | — | [`cleany_dev_monitor`][ros-cleany-dev-monitor] | — |
| `/base/imu_state` | `cleany_base_interfaces/msg/ImuState` | [`cleany_motor_controller`][ros-cleany-motor-controller] | [`cleany_imu`][ros-cleany-imu] | — |
| `/base/wheel_command` | `cleany_base_interfaces/msg/WheelCommand` | [`cleany_base_driver`][ros-cleany-base-driver] | [`mock_base_mcu`][ros-mock-base-mcu], [`cleany_motor_controller`][ros-cleany-motor-controller] | — |
| `/base/wheel_state` | `cleany_base_interfaces/msg/WheelState` | [`mock_base_mcu`][ros-mock-base-mcu], [`cleany_motor_controller`][ros-cleany-motor-controller] | [`cleany_base_driver`][ros-cleany-base-driver] | — |
| `/behavior_tree_log` | `nav2_msgs/msg/BehaviorTreeLog` | — | [`cleany_dev_monitor`][ros-cleany-dev-monitor], [`nav2_bt_trace`][ros-nav2-bt-trace] | — |
| `/body_guard_3d/risk_voxels` | `sensor_msgs/msg/PointCloud2` | [`voxel_guard`][ros-voxel-guard] | [`cleany_dev_monitor`][ros-cleany-dev-monitor] | — |
| `/body_guard_3d/state` | `std_msgs/msg/String` | [`voxel_guard`][ros-voxel-guard] | [`cleany_dev_monitor`][ros-cleany-dev-monitor] | — |
| `/bt_navigator/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | — | [`nav2_bt_trace`][ros-nav2-bt-trace] | — |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | [`sorting_cameras`][ros-sorting-cameras] | — | 발행자는 MuJoCo 전용; 실물 camera 제공자는 별도 |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | [`sorting_cameras`][ros-sorting-cameras] | [`perception_inspector`][ros-perception-inspector] | 발행자는 MuJoCo 전용; 실물 camera 제공자는 별도 |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | [`sorting_cameras`][ros-sorting-cameras] | [`perception_inspector`][ros-perception-inspector] | 발행자는 MuJoCo 전용; 실물 camera 제공자는 별도 |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | — | [`perception_inspector`][ros-perception-inspector], [`depth_scene_node`][ros-depth-scene-node] | — |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | — | [`perception_inspector`][ros-perception-inspector] | — |
| `/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | — | [`depth_scene_node`][ros-depth-scene-node] | — |
| `/camera/head/color/image_raw` | `sensor_msgs/msg/Image` | [`ros_gz_bridge`][ros-ros-gz-bridge] | — | — |
| `/camera/head/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | [`ros_gz_bridge`][ros-ros-gz-bridge] | [`depth_clearance`][ros-depth-clearance], [`depth_coverage`][ros-depth-coverage] | — |
| `/camera/head/depth/image_raw` | `sensor_msgs/msg/Image` | [`ros_gz_bridge`][ros-ros-gz-bridge] | [`depth_clearance`][ros-depth-clearance], [`depth_coverage`][ros-depth-coverage] | — |
| `/camera/head/depth/obstacle_points` | `sensor_msgs/msg/PointCloud2` | [`depth_clearance`][ros-depth-clearance] | [`cleany_dev_monitor`][ros-cleany-dev-monitor] | — |
| `/camera/head/depth/points` | `sensor_msgs/msg/PointCloud2` | [`depth_clearance`][ros-depth-clearance] | [`pan_motion_gate`][ros-pan-motion-gate], [`reactive_guard`][ros-reactive-guard], [`obstacle_memory`][ros-obstacle-memory] | — |
| `/camera/left_wrist/color/image_raw` | `sensor_msgs/msg/Image` | [`ros_gz_bridge`][ros-ros-gz-bridge] | — | — |
| `/camera/right_wrist/color/image_raw` | `sensor_msgs/msg/Image` | [`ros_gz_bridge`][ros-ros-gz-bridge] | — | — |
| `/cleany/internal/mujoco/left_wrist_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | — | [`left_wrist_camera_contract_adapter`][ros-left-wrist-camera-contract-adapter], [`nearest_pregrasp_coordinator`][ros-nearest-pregrasp-coordinator], [`sorting_coordinator`][ros-sorting-coordinator] | — |
| `/cleany/internal/mujoco/left_wrist_camera/image_raw` | `sensor_msgs/msg/Image` | — | [`left_wrist_camera_contract_adapter`][ros-left-wrist-camera-contract-adapter] | — |
| `/clock` | `rosgraph_msgs/msg/Clock` | [`gazebo_feedback_sampler`][ros-gazebo-feedback-sampler], [`ros_gz_bridge`][ros-ros-gz-bridge] | [`cleany_dev_monitor`][ros-cleany-dev-monitor] | Bridge 직접 발행은 override 구성; profile은 sampler 경유 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | [`ground_truth_route_follower`][ros-ground-truth-route-follower], [`reactive_guard`][ros-reactive-guard], [`voxel_guard`][ros-voxel-guard] | [`cleany_base_driver`][ros-cleany-base-driver], [`cleany_dev_monitor`][ros-cleany-dev-monitor], [`gazebo_command_guard`][ros-gazebo-command-guard], [`mujoco_sim`][ros-mujoco-sim] | — |
| `/collision_monitor_state` | `discovery 타입` | — | [`cleany_dev_monitor`][ros-cleany-dev-monitor] | 실행 graph에서 타입 확인 |
| `/constraint_list` | `visualization_msgs/msg/MarkerArray` | — | [`cleany_dev_monitor`][ros-cleany-dev-monitor] | — |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | [`cleany_base_driver`][ros-cleany-base-driver], [`cleany_imu`][ros-cleany-imu] | — | — |
| `/evaluation` | `discovery 타입` | — | [`cleany_dev_monitor`][ros-cleany-dev-monitor] | 실행 graph에서 타입 확인; 발행자 존재는 실행 구성에서 확인 |
| `/gazebo/raw_clock` | `rosgraph_msgs/msg/Clock` | [`ros_gz_bridge`][ros-ros-gz-bridge] | [`gazebo_feedback_sampler`][ros-gazebo-feedback-sampler] | Gazebo profile에서 raw 이름으로 remap |
| `/gazebo/raw_joint_states` | `sensor_msgs/msg/JointState` | [`ros_gz_bridge`][ros-ros-gz-bridge] | [`gazebo_feedback_sampler`][ros-gazebo-feedback-sampler] | Gazebo profile에서 raw 이름으로 remap |
| `/gazebo_cmd_vel` | `geometry_msgs/msg/Twist` | [`gazebo_command_guard`][ros-gazebo-command-guard] | [`ros_gz_bridge`][ros-ros-gz-bridge] | — |
| `/gazebo_odom` | `nav_msgs/msg/Odometry` | [`ros_gz_bridge`][ros-ros-gz-bridge] | [`gazebo_odom_tf_publisher`][ros-gazebo-odom-tf-publisher] | — |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | — | [`cleany_dev_monitor`][ros-cleany-dev-monitor] | — |
| `/grasp/collision_geometry` | `cleany_interfaces/msg/ObservedObjectGeometry` | [`grasp_server`][ros-grasp-server] | [`nearest_pregrasp_coordinator`][ros-nearest-pregrasp-coordinator], [`sorting_coordinator`][ros-sorting-coordinator], [`grasp_selection_server`][ros-grasp-selection-server] | use_observed_collision_geometry=true; publish_collision_geometry=true |
| `/grasp/debug_image` | `sensor_msgs/msg/Image` | [`grasp_server`][ros-grasp-server] | — | — |
| `/grasp/debug_image_latched` | `sensor_msgs/msg/Image` | [`nearest_pregrasp_coordinator`][ros-nearest-pregrasp-coordinator], [`sorting_coordinator`][ros-sorting-coordinator] | — | — |
| `/ground_truth/odom` | `nav_msgs/msg/Odometry` | [`ros_gz_bridge`][ros-ros-gz-bridge] | [`cleany_dev_monitor`][ros-cleany-dev-monitor], [`ground_truth_route_follower`][ros-ground-truth-route-follower] | — |
| `/head_pan/command` | `std_msgs/msg/Float64` | [`pan_motion_gate`][ros-pan-motion-gate] | [`head_pan_bridge`][ros-head-pan-bridge] | Gazebo dynamic pan 구성 |
| `/head_pan/coverage` | `std_msgs/msg/String` | [`depth_coverage`][ros-depth-coverage] | [`cleany_dev_monitor`][ros-cleany-dev-monitor] | — |
| `/head_pan/state` | `std_msgs/msg/String` | [`pan_motion_gate`][ros-pan-motion-gate] | [`cleany_dev_monitor`][ros-cleany-dev-monitor] | — |
| `/imu/data` | `sensor_msgs/msg/Imu` | [`ros_gz_bridge`][ros-ros-gz-bridge], [`cleany_imu`][ros-cleany-imu] | — | 실행 구성별 대안 |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | [`cleany_imu`][ros-cleany-imu] | — | — |
| `/imu/status` | `diagnostic_msgs/msg/DiagnosticArray` | [`cleany_imu`][ros-cleany-imu] | — | — |
| `/joint_states` | `sensor_msgs/msg/JointState` | [`cleany_base_driver`][ros-cleany-base-driver], [`gazebo_feedback_sampler`][ros-gazebo-feedback-sampler], [`ros_gz_bridge`][ros-ros-gz-bridge], [`mujoco_sim`][ros-mujoco-sim] | [`wheel_odometry`][ros-wheel-odometry], [`gazebo_pan_tf_publisher`][ros-gazebo-pan-tf-publisher], [`simulated_encoder`][ros-simulated-encoder], [`single_pose_handeye_orchestrator`][ros-single-pose-handeye-orchestrator], [`multi_pose_handeye_orchestrator`][ros-multi-pose-handeye-orchestrator], [`pan_motion_gate`][ros-pan-motion-gate], [`reactive_guard`][ros-reactive-guard], [`grasp_selection_server`][ros-grasp-selection-server], [`nearest_pregrasp_coordinator`][ros-nearest-pregrasp-coordinator], [`sorting_coordinator`][ros-sorting-coordinator], [`robot_state_publisher`][ros-robot-state-publisher] | Bridge 직접 발행은 override 구성; profile은 sampler 경유 |
| `/left_arm_controller/controller_state` | `control_msgs/msg/JointTrajectoryControllerState` | [`left_arm_controller`][ros-left-arm-controller] | [`nearest_pregrasp_coordinator`][ros-nearest-pregrasp-coordinator], [`sorting_coordinator`][ros-sorting-coordinator] | — |
| `/left_wrist_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | [`left_wrist_camera_contract_adapter`][ros-left-wrist-camera-contract-adapter], [`sorting_cameras`][ros-sorting-cameras] | [`single_pose_handeye_orchestrator`][ros-single-pose-handeye-orchestrator], [`multi_pose_handeye_orchestrator`][ros-multi-pose-handeye-orchestrator], [`perception_inspector`][ros-perception-inspector] | 손목 구독은 enable_wrist_observation=true; MuJoCo camera 또는 adapter 경로 선택; 실물 camera 제공자는 별도 |
| `/left_wrist_camera/image_raw` | `sensor_msgs/msg/Image` | [`left_wrist_camera_contract_adapter`][ros-left-wrist-camera-contract-adapter], [`sorting_cameras`][ros-sorting-cameras] | [`single_pose_handeye_orchestrator`][ros-single-pose-handeye-orchestrator], [`multi_pose_handeye_orchestrator`][ros-multi-pose-handeye-orchestrator], [`perception_inspector`][ros-perception-inspector] | 손목 구독은 enable_wrist_observation=true; MuJoCo camera 또는 adapter 경로 선택; 실물 camera 제공자는 별도 |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | — | [`cleany_dev_monitor`][ros-cleany-dev-monitor] | — |
| `/local_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | — | [`cleany_dev_monitor`][ros-cleany-dev-monitor] | — |
| `/local_plan` | `nav_msgs/msg/Path` | — | [`cleany_dev_monitor`][ros-cleany-dev-monitor] | — |
| `/map` | `nav_msgs/msg/OccupancyGrid` | — | [`cleany_dev_monitor`][ros-cleany-dev-monitor], [`obstacle_memory`][ros-obstacle-memory] | — |
| `/mission/debug_snapshot` | `cleany_interfaces/msg/RuntimeDebugSnapshot` | [`mission_runtime`][ros-mission-runtime] | [`cleany_dev_monitor`][ros-cleany-dev-monitor] | — |
| `/mission/result` | `cleany_interfaces/msg/MissionResult` | [`mission_runtime`][ros-mission-runtime] | [`cleany_dev_monitor`][ros-cleany-dev-monitor] | — |
| `/mission/runtime_events` | `cleany_interfaces/msg/RuntimeEvent` | [`mission_runtime`][ros-mission-runtime] | [`cleany_dev_monitor`][ros-cleany-dev-monitor] | — |
| `/mission/status` | `cleany_interfaces/msg/MissionStatus` | [`mission_runtime`][ros-mission-runtime] | — | — |
| `/mock/manipulation/execution_events` | `discovery 타입` | — | [`cleany_dev_monitor`][ros-cleany-dev-monitor] | 실행 graph에서 타입 확인; 발행자 존재는 실행 구성에서 확인 |
| `/monitored_planning_scene` | `moveit_msgs/msg/PlanningScene` | [`move_group`][ros-move-group] | [`nearest_pregrasp_coordinator`][ros-nearest-pregrasp-coordinator], [`sorting_coordinator`][ros-sorting-coordinator] | — |
| `/mujoco_sim/joint_cmd` | `sensor_msgs/msg/JointState` | — | [`mujoco_sim`][ros-mujoco-sim] | — |
| `/nav2/cmd_vel` | `geometry_msgs/msg/Twist` | — | [`cleany_dev_monitor`][ros-cleany-dev-monitor], [`depth_coverage`][ros-depth-coverage], [`pan_motion_gate`][ros-pan-motion-gate], [`reactive_guard`][ros-reactive-guard] | — |
| `/navigate_to_pose/_action/status` | `action_msgs/msg/GoalStatusArray` | [`bt_navigator`][ros-bt-navigator] | [`mission_runtime`][ros-mission-runtime], [`nav2_bt_trace`][ros-nav2-bt-trace] | — |
| `/navigation/bt_trace` | `std_msgs/msg/String` | [`nav2_bt_trace`][ros-nav2-bt-trace] | [`cleany_dev_monitor`][ros-cleany-dev-monitor] | — |
| `/obstacle_memory/depth` | `nav_msgs/msg/OccupancyGrid` | [`obstacle_memory`][ros-obstacle-memory] | [`cleany_dev_monitor`][ros-cleany-dev-monitor] | — |
| `/obstacle_memory/grid` | `nav_msgs/msg/OccupancyGrid` | [`obstacle_memory`][ros-obstacle-memory] | [`cleany_dev_monitor`][ros-cleany-dev-monitor] | — |
| `/obstacle_memory/lidar` | `nav_msgs/msg/OccupancyGrid` | [`obstacle_memory`][ros-obstacle-memory] | [`cleany_dev_monitor`][ros-cleany-dev-monitor] | — |
| `/obstacle_memory/local_voxels` | `sensor_msgs/msg/PointCloud2` | [`obstacle_memory`][ros-obstacle-memory] | — | — |
| `/obstacle_memory/navigation_grid` | `nav_msgs/msg/OccupancyGrid` | [`obstacle_memory`][ros-obstacle-memory] | — | — |
| `/obstacle_memory/voxel_status` | `std_msgs/msg/String` | [`obstacle_memory`][ros-obstacle-memory] | [`voxel_guard`][ros-voxel-guard] | — |
| `/obstacle_memory/voxels` | `sensor_msgs/msg/PointCloud2` | [`obstacle_memory`][ros-obstacle-memory] | [`voxel_guard`][ros-voxel-guard] | — |
| `/odom` | `nav_msgs/msg/Odometry` | [`base_odom_relay`][ros-base-odom-relay], [`gazebo_odom_tf_publisher`][ros-gazebo-odom-tf-publisher], [`mujoco_sim`][ros-mujoco-sim] | [`mission_runtime`][ros-mission-runtime], [`depth_coverage`][ros-depth-coverage], [`pan_motion_gate`][ros-pan-motion-gate], [`reactive_guard`][ros-reactive-guard], [`voxel_guard`][ros-voxel-guard], [`cleany_telemetry`][ros-cleany-telemetry] | — |
| `/pan_ready/cmd_vel` | `geometry_msgs/msg/Twist` | [`pan_motion_gate`][ros-pan-motion-gate] | — | — |
| `/perception/debug_image` | `sensor_msgs/msg/Image` | [`perception_inspector`][ros-perception-inspector] | — | — |
| `/perception/debug_image_latched` | `sensor_msgs/msg/Image` | [`perception_inspector`][ros-perception-inspector] | [`nearest_pregrasp_coordinator`][ros-nearest-pregrasp-coordinator], [`sorting_coordinator`][ros-sorting-coordinator] | — |
| `/perception/detections_2d` | `cleany_interfaces/msg/DetectedObject2DArray` | [`perception_inspector`][ros-perception-inspector] | — | — |
| `/perception/objects` | `cleany_interfaces/msg/DetectedObject3DArray` | [`perception_inspector`][ros-perception-inspector] | — | — |
| `/perception/scene_cloud` | `sensor_msgs/msg/PointCloud2` | [`depth_scene_node`][ros-depth-scene-node] | [`move_group`][ros-move-group] | occupancy updater 설정 시 |
| `/perception/scene_cloud_filtered` | `sensor_msgs/msg/PointCloud2` | [`move_group`][ros-move-group] | [`scene_cloud_receipt`][ros-scene-cloud-receipt], [`nearest_pregrasp_coordinator`][ros-nearest-pregrasp-coordinator], [`sorting_coordinator`][ros-sorting-coordinator] | occupancy updater 설정 시 |
| `/perception/scene_cloud_receipt` | `std_msgs/msg/Header` | [`scene_cloud_receipt`][ros-scene-cloud-receipt] | [`nearest_pregrasp_coordinator`][ros-nearest-pregrasp-coordinator], [`sorting_coordinator`][ros-sorting-coordinator] | sensor_scene_receipt_topic 지정 시 cloud 구독 대체 |
| `/plan` | `nav_msgs/msg/Path` | — | [`cleany_dev_monitor`][ros-cleany-dev-monitor] | — |
| `/reactive_guard/state` | `std_msgs/msg/String` | [`reactive_guard`][ros-reactive-guard] | [`cleany_dev_monitor`][ros-cleany-dev-monitor] | — |
| `/right_arm_controller/controller_state` | `control_msgs/msg/JointTrajectoryControllerState` | [`right_arm_controller`][ros-right-arm-controller] | [`nearest_pregrasp_coordinator`][ros-nearest-pregrasp-coordinator], [`sorting_coordinator`][ros-sorting-coordinator] | — |
| `/right_wrist_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | [`sorting_cameras`][ros-sorting-cameras] | [`perception_inspector`][ros-perception-inspector] | 손목 구독은 enable_wrist_observation=true; MuJoCo camera 또는 adapter 경로 선택; 실물 camera 제공자는 별도 |
| `/right_wrist_camera/image_raw` | `sensor_msgs/msg/Image` | [`sorting_cameras`][ros-sorting-cameras] | [`perception_inspector`][ros-perception-inspector] | 손목 구독은 enable_wrist_observation=true; MuJoCo camera 또는 adapter 경로 선택; 실물 camera 제공자는 별도 |
| `/robot/safety_fault` | `std_msgs/msg/String` | — | [`mission_runtime`][ros-mission-runtime] | — |
| `/robot/safety_released` | `std_msgs/msg/String` | — | [`mission_runtime`][ros-mission-runtime] | — |
| `/robot_description` | `std_msgs/msg/String` | [`robot_state_publisher`][ros-robot-state-publisher] | [`nearest_pregrasp_coordinator`][ros-nearest-pregrasp-coordinator], [`sorting_coordinator`][ros-sorting-coordinator], [`grasp_selection_server`][ros-grasp-selection-server] | — |
| `/rosout` | `rcl_interfaces/msg/Log` | — | [`cleany_dev_monitor`][ros-cleany-dev-monitor] | — |
| `/safety/slow_zone` | `geometry_msgs/msg/PolygonStamped` | — | [`cleany_dev_monitor`][ros-cleany-dev-monitor] | — |
| `/safety/stop_zone` | `geometry_msgs/msg/PolygonStamped` | — | [`cleany_dev_monitor`][ros-cleany-dev-monitor] | — |
| `/safety_2d/cmd_vel` | `geometry_msgs/msg/Twist` | — | [`cleany_dev_monitor`][ros-cleany-dev-monitor], [`reactive_guard`][ros-reactive-guard], [`voxel_guard`][ros-voxel-guard] | — |
| `/scan` | `sensor_msgs/msg/LaserScan` | [`ros_gz_bridge`][ros-ros-gz-bridge], [`mujoco_sim`][ros-mujoco-sim] | [`cleany_dev_monitor`][ros-cleany-dev-monitor], [`reactive_guard`][ros-reactive-guard], [`obstacle_memory`][ros-obstacle-memory] | — |
| `/scan_matched_points2` | `sensor_msgs/msg/PointCloud2` | — | [`cleany_dev_monitor`][ros-cleany-dev-monitor] | — |
| `/simulation/contact_diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | [`SortingObserver`][ros-sortingobserver] | — | MuJoCo 전용; publish_contacts=true |
| `/simulation/sorting_ground_truth` | `visualization_msgs/msg/MarkerArray` | [`SortingObserver`][ros-sortingobserver] | [`simulation_placement_verifier`][ros-simulation-placement-verifier] | MuJoCo 전용 |
| `/sorting/status` | `std_msgs/msg/String` | [`sorting_coordinator`][ros-sorting-coordinator] | — | — |
| `/submap_list` | `cartographer_ros_msgs/msg/SubmapList` | — | [`cleany_dev_monitor`][ros-cleany-dev-monitor] | — |
| `/tf` | `tf2_msgs/msg/TFMessage` | [`base_odom_relay`][ros-base-odom-relay], [`gazebo_odom_tf_publisher`][ros-gazebo-odom-tf-publisher], [`gazebo_pan_tf_publisher`][ros-gazebo-pan-tf-publisher], [`mujoco_sim`][ros-mujoco-sim], [`robot_state_publisher`][ros-robot-state-publisher] | [`cleany_dev_monitor`][ros-cleany-dev-monitor], [`grasp_server`][ros-grasp-server], [`mission_runtime`][ros-mission-runtime], [`depth_clearance`][ros-depth-clearance], [`depth_coverage`][ros-depth-coverage], [`reactive_guard`][ros-reactive-guard], [`obstacle_memory`][ros-obstacle-memory], [`voxel_guard`][ros-voxel-guard], [`nearest_pregrasp_coordinator`][ros-nearest-pregrasp-coordinator], [`sorting_coordinator`][ros-sorting-coordinator] | — |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | [`gazebo_sensor_tf_publisher`][ros-gazebo-sensor-tf-publisher], [`mujoco_sim`][ros-mujoco-sim], [`robot_state_publisher`][ros-robot-state-publisher] | [`cleany_dev_monitor`][ros-cleany-dev-monitor], [`grasp_server`][ros-grasp-server], [`mission_runtime`][ros-mission-runtime], [`depth_clearance`][ros-depth-clearance], [`depth_coverage`][ros-depth-coverage], [`reactive_guard`][ros-reactive-guard], [`obstacle_memory`][ros-obstacle-memory], [`voxel_guard`][ros-voxel-guard], [`nearest_pregrasp_coordinator`][ros-nearest-pregrasp-coordinator], [`sorting_coordinator`][ros-sorting-coordinator] | — |
| `/trajectory_node_list` | `visualization_msgs/msg/MarkerArray` | — | [`cleany_dev_monitor`][ros-cleany-dev-monitor] | — |
| `/wheel/odom` | `nav_msgs/msg/Odometry` | [`wheel_odometry`][ros-wheel-odometry], [`simulated_odometry_error`][ros-simulated-odometry-error] | [`base_odom_relay`][ros-base-odom-relay], [`cleany_dev_monitor`][ros-cleany-dev-monitor] | — |
| `/wheel/odom_raw` | `nav_msgs/msg/Odometry` | — | [`simulated_odometry_error`][ros-simulated-odometry-error] | — |
| `/wheel_encoder/joint_states` | `sensor_msgs/msg/JointState` | [`simulated_encoder`][ros-simulated-encoder] | — | — |
| `<사용자 선택 / camera / recording 토픽>` | `discovery 타입` | — | [`cleany_dev_monitor`][ros-cleany-dev-monitor] | 실행 graph에서 타입 확인 |

### Action 목록

| 기본 이름 (root namespace) | ROS 타입 | Server | Client | 조건 / 연결 |
|---|---|---|---|---|
| `/execute_trajectory` | `moveit_msgs/action/ExecuteTrajectory` | [`move_group`][ros-move-group] | [`single_pose_handeye_orchestrator`][ros-single-pose-handeye-orchestrator], [`multi_pose_handeye_orchestrator`][ros-multi-pose-handeye-orchestrator], [`nearest_pregrasp_coordinator`][ros-nearest-pregrasp-coordinator], [`sorting_coordinator`][ros-sorting-coordinator] | — |
| `/grasp/select_reachable` | `cleany_interfaces/action/SelectReachableGrasp` | [`grasp_selection_server`][ros-grasp-selection-server] | [`nearest_pregrasp_coordinator`][ros-nearest-pregrasp-coordinator], [`sorting_coordinator`][ros-sorting-coordinator] | 도달성 검사; controller 실행 없음 |
| `/left_arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | [`left_arm_controller`][ros-left-arm-controller] | [`move_group`][ros-move-group] | MoveIt이 호출하는 외부 controller |
| `/left_gripper_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | [`left_gripper_controller`][ros-left-gripper-controller] | [`nearest_pregrasp_coordinator`][ros-nearest-pregrasp-coordinator], [`sorting_coordinator`][ros-sorting-coordinator] | ros2_control backend 구성 필요 |
| `/mock/manipulation/execute_skill` | `cleany_interfaces/action/ExecuteManipulationSkill` | — | [`mission_runtime`][ros-mission-runtime] | action_mock 전용; 외부 server 별도 필요 |
| `/move_action` | `moveit_msgs/action/MoveGroup` | [`move_group`][ros-move-group] | [`single_pose_handeye_orchestrator`][ros-single-pose-handeye-orchestrator], [`multi_pose_handeye_orchestrator`][ros-multi-pose-handeye-orchestrator], [`grasp_selection_server`][ros-grasp-selection-server], [`nearest_pregrasp_coordinator`][ros-nearest-pregrasp-coordinator], [`sorting_coordinator`][ros-sorting-coordinator] | grasp_selection_server는 plan-only |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | [`bt_navigator`][ros-bt-navigator] | [`mission_runtime`][ros-mission-runtime] | 외부 Nav2; navigation_backend=nav2 |
| `/perception/inspect_scene` | `cleany_interfaces/action/InspectScene` | [`perception_inspector`][ros-perception-inspector] | [`nearest_pregrasp_coordinator`][ros-nearest-pregrasp-coordinator], [`sorting_coordinator`][ros-sorting-coordinator] | — |
| `/right_arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | [`right_arm_controller`][ros-right-arm-controller] | [`move_group`][ros-move-group] | MoveIt이 호출하는 외부 controller |
| `/right_gripper_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | [`right_gripper_controller`][ros-right-gripper-controller] | [`nearest_pregrasp_coordinator`][ros-nearest-pregrasp-coordinator], [`sorting_coordinator`][ros-sorting-coordinator] | ros2_control backend 구성 필요 |

### Service 목록

| 기본 이름 (root namespace) | ROS 타입 | Server | Client | 조건 / 연결 |
|---|---|---|---|---|
| `/apply_planning_scene` | `moveit_msgs/srv/ApplyPlanningScene` | [`move_group`][ros-move-group] | [`collision_scene_applier`][ros-collision-scene-applier], [`grasp_selection_server`][ros-grasp-selection-server], [`nearest_pregrasp_coordinator`][ros-nearest-pregrasp-coordinator], [`sorting_coordinator`][ros-sorting-coordinator] | — |
| `/base/enable` | `std_srvs/srv/SetBool` | [`cleany_base_driver`][ros-cleany-base-driver] | — | — |
| `/check_state_validity` | `moveit_msgs/srv/GetStateValidity` | [`move_group`][ros-move-group] | [`single_pose_handeye_orchestrator`][ros-single-pose-handeye-orchestrator], [`multi_pose_handeye_orchestrator`][ros-multi-pose-handeye-orchestrator], [`grasp_selection_server`][ros-grasp-selection-server], [`nearest_pregrasp_coordinator`][ros-nearest-pregrasp-coordinator], [`sorting_coordinator`][ros-sorting-coordinator] | — |
| `/compute_fk` | `moveit_msgs/srv/GetPositionFK` | [`move_group`][ros-move-group] | [`single_pose_handeye_orchestrator`][ros-single-pose-handeye-orchestrator], [`multi_pose_handeye_orchestrator`][ros-multi-pose-handeye-orchestrator], [`nearest_pregrasp_coordinator`][ros-nearest-pregrasp-coordinator], [`sorting_coordinator`][ros-sorting-coordinator], [`grasp_selection_server`][ros-grasp-selection-server] | — |
| `/compute_ik` | `moveit_msgs/srv/GetPositionIK` | [`move_group`][ros-move-group] | [`single_pose_handeye_orchestrator`][ros-single-pose-handeye-orchestrator], [`multi_pose_handeye_orchestrator`][ros-multi-pose-handeye-orchestrator], [`grasp_selection_server`][ros-grasp-selection-server], [`nearest_pregrasp_coordinator`][ros-nearest-pregrasp-coordinator], [`sorting_coordinator`][ros-sorting-coordinator] | — |
| `/get_planning_scene` | `moveit_msgs/srv/GetPlanningScene` | [`move_group`][ros-move-group] | [`single_pose_handeye_orchestrator`][ros-single-pose-handeye-orchestrator], [`multi_pose_handeye_orchestrator`][ros-multi-pose-handeye-orchestrator], [`grasp_selection_server`][ros-grasp-selection-server], [`nearest_pregrasp_coordinator`][ros-nearest-pregrasp-coordinator], [`sorting_coordinator`][ros-sorting-coordinator] | — |
| `/grasp/plan` | `cleany_interfaces/srv/PlanGrasp` | [`grasp_server`][ros-grasp-server] | [`nearest_pregrasp_coordinator`][ros-nearest-pregrasp-coordinator], [`sorting_coordinator`][ros-sorting-coordinator] | — |
| `/grasp/prepare_window` | `std_srvs/srv/SetBool` | [`grasp_server`][ros-grasp-server] | `perception_inspector` | AnyGrasp 준비 뒤 cuRobo 예열; `vlm_anygrasp_prepare_service` |
| `/handeye/cancel_run` | `std_srvs/srv/Trigger` | [`multi_pose_handeye_orchestrator`][ros-multi-pose-handeye-orchestrator] | — | — |
| `/mission/cancel` | `cleany_interfaces/srv/CancelMission` | [`mission_runtime`][ros-mission-runtime] | [`control_bridge`][ros-control-bridge] | — |
| `/mission/offer` | `cleany_interfaces/srv/OfferMission` | [`mission_runtime`][ros-mission-runtime] | [`control_bridge`][ros-control-bridge] | — |
| `/mission/reset_error` | `std_srvs/srv/Trigger` | [`mission_runtime`][ros-mission-runtime] | — | — |
| `/mission/snapshot` | `cleany_interfaces/srv/GetRuntimeSnapshot` | [`mission_runtime`][ros-mission-runtime] | [`control_bridge`][ros-control-bridge] | — |
| `/mock/drop_next_enable` | `std_srvs/srv/Trigger` | [`mock_base_mcu`][ros-mock-base-mcu] | — | — |
| `/mock/force_disarm` | `std_srvs/srv/Trigger` | [`mock_base_mcu`][ros-mock-base-mcu] | — | — |
| `/mock/manipulation/execute_skill/_action/cancel_goal` | `action_msgs/srv/CancelGoal` | — | [`mission_runtime`][ros-mission-runtime] | action_mock 전용; 외부 server 별도 필요 |
| `/mock/manipulation/get_execution` | `cleany_interfaces/srv/GetManipulationExecution` | — | [`mission_runtime`][ros-mission-runtime] | action_mock 전용; 외부 server 별도 필요 |
| `/mock/reboot` | `std_srvs/srv/Trigger` | [`mock_base_mcu`][ros-mock-base-mcu] | — | — |
| `/mock/transport` | `std_srvs/srv/SetBool` | [`mock_base_mcu`][ros-mock-base-mcu] | — | — |
| `/navigate_to_pose/_action/cancel_goal` | `action_msgs/srv/CancelGoal` | [`bt_navigator`][ros-bt-navigator] | [`mission_runtime`][ros-mission-runtime] | — |
| `/obstacle_memory/save` | `std_srvs/srv/Trigger` | [`obstacle_memory`][ros-obstacle-memory] | — | — |
| `/obstacle_memory/set_enabled` | `std_srvs/srv/SetBool` | [`obstacle_memory`][ros-obstacle-memory] | — | — |
| `/perception/observe_wrist_target` | `cleany_interfaces/srv/ObserveWristTarget` | [`perception_inspector`][ros-perception-inspector] | [`sorting_coordinator`][ros-sorting-coordinator] | enable_wrist_observation=true |
| `/sorting/verify_placement` | `cleany_interfaces/srv/VerifyPlacement` | [`simulation_placement_verifier`][ros-simulation-placement-verifier] | [`sorting_coordinator`][ros-sorting-coordinator] | 현재 server는 시뮬레이션 전용; 실물은 독립 센서 제공자 필요 |
| `/sorting_cameras/set_parameters` | `rcl_interfaces/srv/SetParameters` | [`sorting_cameras`][ros-sorting-cameras] | [`sorting_coordinator`][ros-sorting-coordinator] | MuJoCo 카메라 선택; 명시적으로 호출하는 parameter 서비스 |
| `/submap_query` | `cartographer_ros_msgs/srv/SubmapQuery` | — | [`cleany_dev_monitor`][ros-cleany-dev-monitor] | — |
| `<lifecycle_node>/get_state` | `lifecycle_msgs/srv/GetState` | [`<lifecycle_node>`][ros-lifecycle-node] | [`mission_runtime`][ros-mission-runtime] | navigation_backend=nav2; required_navigation_lifecycle_nodes |

[ros-base-odom-relay]: ../ros2_ws/src/cleany_base_driver/docs/interfaces.md#base_odom_relay
[ros-bt-navigator]: ../ros2_ws/src/cleany_navigation/launch/navigation.launch.py
[ros-cleany-imu]: ../ros2_ws/src/cleany_base_driver/docs/interfaces.md#cleany_imu
[ros-cleany-base-driver]: ../ros2_ws/src/cleany_base_driver/docs/interfaces.md#cleany_base_driver
[ros-cleany-dev-monitor]: ../ros2_ws/src/cleany_dev_monitor/docs/interfaces.md#cleany_dev_monitor
[ros-cleany-motor-controller]: ../esp32/README.md#ros-인터페이스
[ros-cleany-telemetry]: ../ros2_ws/src/cleany_telemetry/docs/interfaces.md#cleany_telemetry
[ros-collision-scene-applier]: ../ros2_ws/src/cleany_moveit_config/cleany_moveit_config/apply_collision_scene.py
[ros-control-bridge]: ../ros2_ws/src/cleany_control_bridge/docs/interfaces.md#control_bridge
[ros-depth-clearance]: ../ros2_ws/src/cleany_navigation/docs/interfaces.md#depth_clearance
[ros-depth-coverage]: ../ros2_ws/src/cleany_navigation/docs/interfaces.md#depth_coverage
[ros-depth-scene-node]: ../ros2_ws/src/cleany_perception/cleany_perception/depth_scene_node.py
[ros-gazebo-command-guard]: ../ros2_ws/src/cleany_gazebo_sim/docs/interfaces.md#gazebo_command_guard
[ros-gazebo-feedback-sampler]: ../ros2_ws/src/cleany_gazebo_sim/docs/interfaces.md#gazebo_feedback_sampler
[ros-gazebo-odom-tf-publisher]: ../ros2_ws/src/cleany_gazebo_sim/docs/interfaces.md#gazebo_odom_tf_publisher
[ros-gazebo-pan-tf-publisher]: ../ros2_ws/src/cleany_gazebo_sim/docs/interfaces.md#gazebo_pan_tf_publisher
[ros-gazebo-sensor-tf-publisher]: ../ros2_ws/src/cleany_gazebo_sim/docs/interfaces.md#gazebo_sensor_tf_publisher
[ros-grasp-selection-server]: ../ros2_ws/src/cleany_skill_executor/docs/interfaces.md#grasp_selection_server
[ros-grasp-server]: ../ros2_ws/src/cleany_grasping/docs/interfaces.md#grasp_server
[ros-ground-truth-route-follower]: ../ros2_ws/src/cleany_gazebo_sim/docs/interfaces.md#ground_truth_route_follower
[ros-head-pan-bridge]: ../ros2_ws/src/cleany_gazebo_sim/launch/amcl_nav2_safety.launch.py
[ros-lifecycle-node]: ../ros2_ws/src/cleany_mission_manager/cleany_mission_manager/adapters/nav2.py
[ros-left-arm-controller]: ../ros2_ws/src/cleany_mujoco_sim/launch/study_cafe_backend.launch.py
[ros-left-gripper-controller]: ../ros2_ws/src/cleany_mujoco_sim/launch/study_cafe_backend.launch.py
[ros-left-wrist-camera-contract-adapter]: ../ros2_ws/src/cleany_mujoco_sim/docs/interfaces.md#left_wrist_camera_contract_adapter
[ros-mission-runtime]: ../ros2_ws/src/cleany_mission_manager/docs/interfaces.md#mission_runtime
[ros-mock-base-mcu]: ../ros2_ws/src/cleany_base_driver/docs/interfaces.md#mock_base_mcu
[ros-move-group]: ../ros2_ws/src/cleany_moveit_config/docs/planning.md
[ros-mujoco-sim]: ../ros2_ws/src/cleany_mujoco_sim/docs/interfaces.md#mujoco_sim
[ros-multi-pose-handeye-orchestrator]: ../ros2_ws/src/cleany_handeye_calibration/docs/interfaces.md#multi_pose_handeye_orchestrator
[ros-nav2-bt-trace]: ../ros2_ws/src/cleany_navigation/docs/interfaces.md#nav2_bt_trace
[ros-nearest-pregrasp-coordinator]: ../ros2_ws/src/cleany_skill_executor/cleany_skill_executor/nearest_pregrasp_coordinator.py
[ros-obstacle-memory]: ../ros2_ws/src/cleany_navigation/docs/interfaces.md#obstacle_memory
[ros-pan-motion-gate]: ../ros2_ws/src/cleany_navigation/docs/interfaces.md#pan_motion_gate
[ros-perception-inspector]: ../ros2_ws/src/cleany_perception/cleany_perception/inspection_node.py
[ros-reactive-guard]: ../ros2_ws/src/cleany_navigation/docs/interfaces.md#reactive_guard
[ros-right-arm-controller]: ../ros2_ws/src/cleany_mujoco_sim/launch/study_cafe_backend.launch.py
[ros-right-gripper-controller]: ../ros2_ws/src/cleany_mujoco_sim/launch/study_cafe_backend.launch.py
[ros-robot-state-publisher]: ../ros2_ws/src/cleany_description/docs/model.md
[ros-ros-gz-bridge]: ../ros2_ws/src/cleany_gazebo_sim/docs/interfaces.md#ros_gz_bridge
[ros-scene-cloud-receipt]: ../ros2_ws/src/cleany_perception/cleany_perception/scene_cloud_receipt_node.py
[ros-simulated-encoder]: ../ros2_ws/src/cleany_gazebo_sim/docs/interfaces.md#simulated_encoder
[ros-simulated-odometry-error]: ../ros2_ws/src/cleany_gazebo_sim/docs/interfaces.md#simulated_odometry_error
[ros-simulation-placement-verifier]: ../ros2_ws/src/cleany_mujoco_sim/docs/interfaces.md#simulation_placement_verifier
[ros-single-pose-handeye-orchestrator]: ../ros2_ws/src/cleany_handeye_calibration/docs/interfaces.md#single_pose_handeye_orchestrator
[ros-sorting-cameras]: ../ros2_ws/src/cleany_mujoco_observer/src/scheduled_cameras.cpp
[ros-sorting-coordinator]: ../ros2_ws/src/cleany_skill_executor/cleany_skill_executor/sorting_coordinator.py
[ros-sortingobserver]: ../ros2_ws/src/cleany_mujoco_observer/src/sorting_observer.cpp
[ros-voxel-guard]: ../ros2_ws/src/cleany_navigation/docs/interfaces.md#voxel_guard
[ros-wheel-odometry]: ../ros2_ws/src/cleany_base_odometry/docs/interfaces.md#wheel_odometry

## 구현 노드

| 노드 이름 | 패키지 | 실행 파일 | 인터페이스 표 |
|---|---|---|---|
| `cleany_base_driver` | `cleany_base_driver` | `base_driver_node` | [표](../ros2_ws/src/cleany_base_driver/docs/interfaces.md#cleany_base_driver) |
| `cleany_imu` | `cleany_base_driver` | `imu_driver_node` | [표](../ros2_ws/src/cleany_base_driver/docs/interfaces.md#cleany_imu) |
| `mock_base_mcu` | `cleany_base_driver` | `mock_mcu_node` | [표](../ros2_ws/src/cleany_base_driver/docs/interfaces.md#mock_base_mcu) |
| `base_odom_relay` | `cleany_base_driver` | `odom_relay_node` | [표](../ros2_ws/src/cleany_base_driver/docs/interfaces.md#base_odom_relay) |
| `wheel_odometry` | `cleany_base_odometry` | `wheel_odometry_node` | [표](../ros2_ws/src/cleany_base_odometry/docs/interfaces.md#wheel_odometry) |
| `control_bridge` | `cleany_control_bridge` | `control_bridge` | [표](../ros2_ws/src/cleany_control_bridge/docs/interfaces.md#control_bridge) |
| `mission_runtime` | `cleany_mission_manager` | `mission_runtime` | [표](../ros2_ws/src/cleany_mission_manager/docs/interfaces.md#mission_runtime) |
| `manipulation_server` | `cleany_skill_executor` | `manipulation_server` | [표](../ros2_ws/src/cleany_skill_executor/docs/interfaces.md#manipulation_server) |
| `grasp_server` | `cleany_grasping` | `grasp_server` | [표](../ros2_ws/src/cleany_grasping/docs/interfaces.md#grasp_server) |
| `grasp_selection_server` | `cleany_skill_executor` | `grasp_selection_server` | [표](../ros2_ws/src/cleany_skill_executor/docs/interfaces.md#grasp_selection_server) |
| `collision_scene_applier` | `cleany_moveit_config` | `apply_collision_scene` | [구현](../ros2_ws/src/cleany_moveit_config/cleany_moveit_config/apply_collision_scene.py) |
| `cleany_telemetry` | `cleany_telemetry` | `telemetry_node` | [표](../ros2_ws/src/cleany_telemetry/docs/interfaces.md#cleany_telemetry) |
| `depth_clearance` | `cleany_navigation` | `depth_clearance` | [표](../ros2_ws/src/cleany_navigation/docs/interfaces.md#depth_clearance) |
| `depth_coverage` | `cleany_navigation` | `depth_coverage` | [표](../ros2_ws/src/cleany_navigation/docs/interfaces.md#depth_coverage) |
| `pan_motion_gate` | `cleany_navigation` | `pan_motion_gate` | [표](../ros2_ws/src/cleany_navigation/docs/interfaces.md#pan_motion_gate) |
| `reactive_guard` | `cleany_navigation` | `reactive_guard` | [표](../ros2_ws/src/cleany_navigation/docs/interfaces.md#reactive_guard) |
| `obstacle_memory` | `cleany_navigation` | `obstacle_memory` | [표](../ros2_ws/src/cleany_navigation/docs/interfaces.md#obstacle_memory) |
| `voxel_guard` | `cleany_navigation` | `voxel_guard` | [표](../ros2_ws/src/cleany_navigation/docs/interfaces.md#voxel_guard) |
| `nav2_bt_trace` | `cleany_navigation` | `nav2_bt_trace` | [표](../ros2_ws/src/cleany_navigation/docs/interfaces.md#nav2_bt_trace) |
| `gazebo_command_guard` | `cleany_gazebo_sim` | `gazebo_command_guard` | [표](../ros2_ws/src/cleany_gazebo_sim/docs/interfaces.md#gazebo_command_guard) |
| `gazebo_feedback_sampler` | `cleany_gazebo_sim` | `gazebo_feedback_sampler` | [표](../ros2_ws/src/cleany_gazebo_sim/docs/interfaces.md#gazebo_feedback_sampler) |
| `gazebo_odom_tf_publisher` | `cleany_gazebo_sim` | `gazebo_odom_tf_publisher` | [표](../ros2_ws/src/cleany_gazebo_sim/docs/interfaces.md#gazebo_odom_tf_publisher) |
| `gazebo_pan_tf_publisher` | `cleany_gazebo_sim` | `gazebo_pan_tf_publisher` | [표](../ros2_ws/src/cleany_gazebo_sim/docs/interfaces.md#gazebo_pan_tf_publisher) |
| `gazebo_sensor_tf_publisher` | `cleany_gazebo_sim` | `gazebo_sensor_tf_publisher` | [표](../ros2_ws/src/cleany_gazebo_sim/docs/interfaces.md#gazebo_sensor_tf_publisher) |
| `simulated_encoder` | `cleany_gazebo_sim` | `simulated_encoder_node` | [표](../ros2_ws/src/cleany_gazebo_sim/docs/interfaces.md#simulated_encoder) |
| `simulated_odometry_error` | `cleany_gazebo_sim` | `simulated_odometry_error_node` | [표](../ros2_ws/src/cleany_gazebo_sim/docs/interfaces.md#simulated_odometry_error) |
| `ground_truth_route_follower` | `cleany_gazebo_sim` | `ground_truth_route_follower` | [표](../ros2_ws/src/cleany_gazebo_sim/docs/interfaces.md#ground_truth_route_follower) |
| `mujoco_sim` | `cleany_mujoco_sim` | `mujoco_sim_node` | [표](../ros2_ws/src/cleany_mujoco_sim/docs/interfaces.md#mujoco_sim) |
| `left_wrist_camera_contract_adapter` | `cleany_mujoco_sim` | `camera_contract_adapter` | [표](../ros2_ws/src/cleany_mujoco_sim/docs/interfaces.md#left_wrist_camera_contract_adapter) |
| `single_pose_handeye_orchestrator` | `cleany_handeye_calibration` | `single_pose_calibration` | [표](../ros2_ws/src/cleany_handeye_calibration/docs/interfaces.md#single_pose_handeye_orchestrator) |
| `multi_pose_handeye_orchestrator` | `cleany_handeye_calibration` | `multi_pose_calibration` | [표](../ros2_ws/src/cleany_handeye_calibration/docs/interfaces.md#multi_pose_handeye_orchestrator) |
| `cleany_logger` | `cleany_logger` | `logger_node` | [표](../ros2_ws/src/cleany_logger/docs/interfaces.md#cleany_logger) |
| `cleany_logger` | `cleany_logger_cli_<suffix>` | `logger` | [CLI](../ros2_ws/src/cleany_logger/docs/interfaces.md#cli) |
| `cleany_dev_monitor` | `cleany_dev_monitor` | `dev_monitor` | [표](../ros2_ws/src/cleany_dev_monitor/docs/interfaces.md#cleany_dev_monitor) |
| `cleany_dev_monitor_operator_<pid>_<id>` | `cleany_dev_monitor` | 인증된 웹 작업의 adapter | [승인 제안과 관측](../ros2_ws/src/cleany_dev_monitor/docs/interfaces.md#승인-기반-작업-제안) |
| `cleany_motor_controller` | ESP32 micro-ROS | `esp32/src/main.cpp` | [MCU 표](../esp32/README.md#ros-인터페이스) |

Single/multi pose 실행 전 설정 파일만 읽는 `single_pose_handeye_config`, `multi_pose_handeye_config`는 임시 bootstrap 노드다. 응용 endpoint가 없으며 ROS 기본 인터페이스만 생성한다. `cleany_description`, `cleany_bringup`은 모델/launch를 제공하고, `cleany_interfaces`, `cleany_base_interfaces`는 타입을 정의한다. `cleany_planner`에는 현재 등록된 자체 ROS 노드가 없다. `cleany_logger`는 독립 기록 노드이며 [기록 계약](../ros2_ws/src/cleany_logger/docs/data.md)을 따른다. 새 perception과 sorting 실행은 아래 패키지 문서에서 확인한다. 이전 grasp 실행 데모, RGB-D pick 노드와 MuJoCo calibration pose generator는 제거됐다.

## Runtime integration에서 추가된 실행

| 노드 이름 | 실행 파일 | 구현과 API 안내 |
|---|---|---|
| `perception_inspector` | `inspection_node` | [Perception](../ros2_ws/src/cleany_perception/docs/runtime.md#ros-api) |
| `depth_scene_node` | `depth_scene_node` | [Depth 점군](../ros2_ws/src/cleany_perception/docs/runtime.md#전체-depth-환경-포인트클라우드) |
| `scene_cloud_receipt` | `scene_cloud_receipt_node` | [점군 수신 확인](../ros2_ws/src/cleany_perception/docs/runtime.md#필터링-점군-수신-확인) |
| `nearest_pregrasp_coordinator` | `nearest_pregrasp_coordinator` | [조작 경로](../ros2_ws/src/cleany_skill_executor/docs/manipulation.md#가까운-객체-자동-pre-grasp) |
| `nearest_pregrasp_coordinator` 상속 이름 | `sorting_coordinator` | [분류와 수거](../ros2_ws/src/cleany_skill_executor/docs/manipulation.md#시뮬레이션-분리-수거-통합-검증-진행-중); launch remap 적용 여부 확인 |
| `simulation_placement_verifier` | `placement_verifier` | [MuJoCo 표](../ros2_ws/src/cleany_mujoco_sim/docs/interfaces.md#simulation_placement_verifier) |
| `manipulation_server` | `manipulation_server` | [Mock Action 서버](../ros2_ws/src/cleany_skill_executor/docs/interfaces.md#manipulation_server) |
| `manipulation_monitor` | `manipulation_monitor` | [읽기 전용 트리 모니터](../ros2_ws/src/cleany_skill_executor/docs/interfaces.md#manipulation_monitor) |
| `manipulation_test_client` | `manipulation_test_client` | [요청·취소·조회 CLI](../ros2_ws/src/cleany_skill_executor/docs/interfaces.md#manipulation_test_client) |
| `manipulation_mujoco_server` | `manipulation_mujoco_server` | [MuJoCo BT Action 서버](../ros2_ws/src/cleany_manipulation_bt/README.md#계약과-증거) |

`perception_inspector`는 `GetSceneSnapshot` 타입의 `/perception/get_scene_snapshot`을
제공해 기존 cache를 조회합니다. `simulation_placement_verifier`의
`/sorting/register_placement_target`은 `RegisterPlacementTarget`으로 개체를 고정하며,
`VerifyPlacement.verification_id`로 해당 개체만 검증합니다.
상세 필드는 [관측 계약](../ros2_ws/src/cleany_interfaces/docs/observation-contracts.md)을 따릅니다.

## 외부 노드와 타입만 있는 계약

| 경계 | 연결 위치와 현재 상태 |
|---|---|
| Gazebo `ros_gz_bridge` | [Bridge 표](../ros2_ws/src/cleany_gazebo_sim/docs/interfaces.md#ros_gz_bridge). ROS/Gazebo sensor와 명령 변환 |
| Nav2 / AMCL / Collision Monitor | [Navigation launch](../ros2_ws/src/cleany_navigation/launch/navigation.launch.py)와 [설정](../ros2_ws/src/cleany_navigation/config)이 조합을 선택. Mission이 호출하는 action/lifecycle 서비스는 Mission 노드 표에 포함 |
| MoveIt `move_group` | `/compute_ik`, `/compute_fk`, `/check_state_validity`, `/get_planning_scene`, `/apply_planning_scene`, `/move_action`, `/execute_trajectory`를 이용. 자체 호출 노드 표와 [Planning 계약](../ros2_ws/src/cleany_moveit_config/docs/planning.md)에서 연결 확인 |
| ros2_control | Arm/gripper의 FollowJointTrajectory와 joint_state_broadcaster. [Controller 설정](../ros2_ws/src/cleany_moveit_config/config/mock_ros2_controllers.yaml)과 [MuJoCo backend launch](../ros2_ws/src/cleany_mujoco_sim/launch/study_cafe_backend.launch.py)이 선택 |
| `robot_state_publisher` | `joint_states` 구독, `/tf`, `/tf_static` 발행. URDF와 [모델 계약](../ros2_ws/src/cleany_description/docs/model.md)이 frame 의미를 관리 |
| `InspectScene` | `perception_inspector`가 `perception/inspect_scene`을 제공. [Perception API](../ros2_ws/src/cleany_perception/docs/runtime.md#ros-api) |
| `ExecuteManipulationSkill` | Runtime의 action client와 [타입](../ros2_ws/src/cleany_interfaces/action/ExecuteManipulationSkill.action)은 존재. 기본 mock는 process 내부 구현이며 `action_mock`은 선택형 Python Mock 서버에 연결 |

위 표는 외부 패키지의 연결 경계다. 외부 패키지의 전체 API 목록은 해당 패키지 문서와 실행 graph를 기준으로 확인한다. RViz, rqt, controller spawner와 검증용 probe 노드는 실행 보조 도구이며 자체 제품 endpoint 목록에 포함하지 않는다.

## ROS가 자동 생성하는 인터페이스

Parameter 서비스와 이벤트, rosout, sim clock 구독, action 내부 전송은 응용 표에서 반복하지 않는다. 아래 패턴이 노드 옵션과 실행 상태에 따라 추가된다.

| 항목 | 기본 패턴 / 타입 |
|---|---|
| Parameter 서비스 | `<node>/{describe_parameters,get_parameter_types,get_parameters,list_parameters,set_parameters,set_parameters_atomically}`; `rcl_interfaces/srv/*` |
| Parameter 변경 이벤트 | `/parameter_events`; `rcl_interfaces/msg/ParameterEvent` |
| 로그 | `/rosout`; `rcl_interfaces/msg/Log`, rosout 활성화 시 |
| Simulation clock | `/clock`; `rosgraph_msgs/msg/Clock`, ROS time source 연결 시; `use_sim_time`으로 시간 적용 선택 |
| Action 요청 / 결과 / 취소 | `<action>/_action/{send_goal,get_result,cancel_goal}` 서비스 |
| Action feedback / status | `<action>/_action/{feedback,status}` 토픽 |

명시적으로 만든 cancel client와 status subscription은 호출 노드의 표에도 표시한다. ESP32 rclc 노드는 표에 적힌 wheel interface만 초기화하며 rclpy의 자동 parameter 서비스가 동일하게 생긴다고 가정하지 않는다.

## 실행 중 확인

실행한 구성과 같은 DDS domain, 같은 overlay의 터미널에서 확인한다.

```bash
ros2 node list
ros2 node info /mission_runtime
ros2 topic list -t
ros2 topic info /cmd_vel --verbose
ros2 service list -t
ros2 action list -t
ros2 action info /grasp/select_reachable
```

실행 그래프는 해당 실행 시점의 실제 연결 상태를 나타냅니다. 조건부 노드가 실행되지 않았다고 해서 소스 코드의 전체 인터페이스 정의가 누락된 것은 아닙니다. 코드 및 인터페이스를 변경할 때는 관련 패키지의 인터페이스 표와 이 색인 문서를 함께 갱신합니다.

## Jetson 실물 경로의 추가 노드

| 노드 및 실행 파일 | 역할 및 인터페이스 상세 |
|---|---|
| `arm_state_reader`, `robot_state_reader`, `control_state_guard`, `wrist_rgb_camera`, `rgbd_frame_bridge`, `motor_telemetry_recorder` | [cleany_robot_interface 인터페이스 규격](../ros2_ws/src/cleany_robot_interface/docs/interfaces.md) |
| `physical_grasp_selection_server` | [실물 파지 선택 인터페이스](../ros2_ws/src/cleany_skill_executor/docs/interfaces.md#physical_grasp_selection_server) |
| `validate_pick_server`, `manipulation_evidence`, `collection_pipeline`, `hardware_pregrasp`, `pregrasp_pipeline`, `pregrasp_sim_ready` | [실물 파이프라인 인터페이스](../ros2_ws/src/cleany_skill_executor/docs/interfaces.md#실물-준비-검증과-증거-노드) |
| `manipulation_jetson_server`, `manipulation_recovery`, `arm_return` | [Jetson BT 인터페이스 규격](../ros2_ws/src/cleany_manipulation_bt/README.md#jetson-실물-backend) |

> [!NOTE]
> - 기존 MuJoCo 시뮬레이션 및 Mock 실행 파일은 그대로 유지됩니다.
> - `/perception/check_rgbd` 서비스(`std_srvs/Trigger`)를 통해 최신 RGB-D 영상 수신 상태를 확인할 수 있습니다.
> - 실물과 시뮬레이터가 동일한 Action 토픽명을 사용하는 경우, 혼선을 피하기 위해 백엔드와 네임스페이스(`namespace`)를 명시적으로 구분하여 실행합니다.
> - `manipulation_io_probe` 및 `manipulation_io_report`는 모니터링 및 성능 분석 도구입니다. 실행 중인 토픽을 수동적으로 수신(Passive Subscribe)하여 지연 및 Hz를 기록하며, 모터나 센서에 직접적인 제어 명령을 보내지 않습니다. 상세 내용은 [Dev Monitor 안내](../ros2_ws/src/cleany_dev_monitor/README.md)를 참고하세요.
