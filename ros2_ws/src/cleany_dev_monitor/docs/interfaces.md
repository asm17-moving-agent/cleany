# cleany_dev_monitor 노드 인터페이스

응용 인터페이스와 adapter에서 생성하는 endpoint를 노드별로 정리한다. 이름과 QoS 표기, ROS 자동 인터페이스는 [전체 색인](../../../../docs/ROS_INTERFACES.md)을 따른다. 이름 설정이 없는 항목은 ROS remap으로 변경한다.

## cleany_dev_monitor

실행 파일: `dev_monitor`. 원본: [node.py](../cleany_dev_monitor/node.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `<runtime_namespace>/mission/debug_snapshot` | `cleany_interfaces/msg/RuntimeDebugSnapshot` | 동적 QoS; `runtime_namespace` |
| 토픽 구독 | `<runtime_namespace>/mission/runtime_events` | `cleany_interfaces/msg/RuntimeEvent` | 동적 QoS; `runtime_namespace`, depth 2048 |
| 토픽 구독 | `<runtime_namespace>/mission/result` | `cleany_interfaces/msg/MissionResult` | 동적 QoS; `runtime_namespace` |
| 토픽 구독 | `/rosout` | `rcl_interfaces/msg/Log` | 동적 QoS |
| 토픽 구독 | `/clock` | `rosgraph_msgs/msg/Clock` | 동적 QoS |
| 토픽 구독 | `/tf` | `tf2_msgs/msg/TFMessage` | TF 기본 QoS |
| 토픽 구독 | `/tf_static` | `tf2_msgs/msg/TFMessage` | TF 기본 QoS |
| 토픽 구독 | `/map` | `nav_msgs/msg/OccupancyGrid` | 동적 QoS; `map_topic` |
| 토픽 구독 | `/plan` | `nav_msgs/msg/Path` | 동적 QoS; `path_topic` |
| 토픽 구독 | `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 동적 QoS; `local_costmap_topic` |
| 토픽 구독 | `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 동적 QoS; `global_costmap_topic` |
| 토픽 구독 | `/nav2/cmd_vel` | `geometry_msgs/msg/Twist` | 동적 QoS; `velocity_topics` |
| 토픽 구독 | `/safety_2d/cmd_vel` | `geometry_msgs/msg/Twist` | 동적 QoS; `velocity_topics` |
| 토픽 구독 | `/cmd_vel` | `geometry_msgs/msg/Twist` | 동적 QoS; `velocity_topics` |
| 토픽 구독 | `/wheel/odom` | `nav_msgs/msg/Odometry` | 동적 QoS; `odometry_topic` |
| 토픽 구독 | `/ground_truth/odom` | `nav_msgs/msg/Odometry` | 동적 QoS; `ground_truth_topic` |
| 토픽 구독 | `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 동적 QoS; `amcl_pose_topic` |
| 토픽 구독 | `/head_pan/state` | `std_msgs/msg/String` | 동적 QoS; `pan_state_topic` |
| 토픽 구독 | `/reactive_guard/state` | `std_msgs/msg/String` | 동적 QoS; `reactive_guard_topic` |
| 토픽 구독 | `/body_guard_3d/state` | `std_msgs/msg/String` | 동적 QoS; `voxel_guard_topic` |
| 토픽 구독 | `/head_pan/coverage` | `std_msgs/msg/String` | 동적 QoS; `coverage_topic` |
| 토픽 구독 | `/behavior_tree_log` | `nav2_msgs/msg/BehaviorTreeLog` | 동적 QoS; `nav2_bt_topic` |
| 토픽 구독 | `/navigation/bt_trace` | `std_msgs/msg/String` | 동적 QoS; `nav2_bt_trace_topic` |
| 토픽 구독 | `/collision_monitor_state` | `discovery 타입` | 동적 QoS; `collision_state_topic` |
| 토픽 구독 | `<runtime_namespace>/mock/manipulation/execution_events` | `discovery 타입` | 동적 QoS; `manipulation_events_topic` |
| 토픽 구독 | `/evaluation` | `discovery 타입` | 선택 시 동적 구독; `controller_evaluation_topic` |
| 토픽 구독 | `/scan` | `sensor_msgs/msg/LaserScan` | 선택 시 동적 구독; `scan_topic` |
| 토픽 구독 | `/camera/head/depth/obstacle_points` | `sensor_msgs/msg/PointCloud2` | 선택 시 동적 구독; `depth_points_topic` |
| 토픽 구독 | `/body_guard_3d/risk_voxels` | `sensor_msgs/msg/PointCloud2` | 선택 시 동적 구독; `risk_points_topic` |
| 토픽 구독 | `/local_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | 선택 시 동적 구독; `footprint_topic` |
| 토픽 구독 | `/safety/stop_zone` | `geometry_msgs/msg/PolygonStamped` | 선택 시 동적 구독; `stop_polygon_topic` |
| 토픽 구독 | `/safety/slow_zone` | `geometry_msgs/msg/PolygonStamped` | 선택 시 동적 구독; `slow_polygon_topic` |
| 토픽 구독 | `/local_plan` | `nav_msgs/msg/Path` | 선택 시 동적 구독; `local_path_topic` |
| 토픽 구독 | `/obstacle_memory/grid` | `nav_msgs/msg/OccupancyGrid` | 선택 시 동적 구독; `obstacle_memory_topic` |
| 토픽 구독 | `/obstacle_memory/lidar` | `nav_msgs/msg/OccupancyGrid` | 선택 시 동적 구독; `lidar_memory_topic` |
| 토픽 구독 | `/obstacle_memory/depth` | `nav_msgs/msg/OccupancyGrid` | 선택 시 동적 구독; `depth_memory_topic` |
| 토픽 구독 | `/submap_list` | `cartographer_ros_msgs/msg/SubmapList` | 동적 QoS; `slam_submaps_topic` |
| 토픽 구독 | `/trajectory_node_list` | `visualization_msgs/msg/MarkerArray` | 동적 QoS; `slam_trajectory_topic` |
| 토픽 구독 | `/constraint_list` | `visualization_msgs/msg/MarkerArray` | 동적 QoS; `slam_constraints_topic` |
| 토픽 구독 | `/scan_matched_points2` | `sensor_msgs/msg/PointCloud2` | 동적 QoS; `slam_matched_points_topic` |
| 서비스 요청 | `/submap_query` | `cartographer_ros_msgs/srv/SubmapQuery` | allowlist된 읽기 전용 요청; `slam_submap_query_service` |
| 토픽 구독 | `<action>/_action/status` | `action_msgs/msg/GoalStatusArray` | Discovery된 최대 32개 status/feedback 토픽 내에서 선택 |
| 토픽 구독 | `<action>/_action/feedback` | `<action package>/action/<Action>_FeedbackMessage` | Discovery 타입과 publisher QoS |
| 토픽 구독 | `<사용자 선택 / camera / recording 토픽>` | `discovery 타입` | 설정과 웹 선택에 따라 구독 생성 / 해제 |

응용 ROS publisher, service server와 action server/client는 없다. SubmapQuery service client만 서브맵을 읽는다. Discovery에서 해당 이름과 타입이 확인돼야 실제 구독을 생성한다. 표의 타입은 기본 입력의 기대 타입이며 구독은 discovery 타입을 사용한다. `runtime_namespace`가 비어 있으면 prefix는 없다. TF listener 구독은 별도로 생성된다.

동적 QoS는 모든 publisher와 호환되는 reliability/durability를 선택한다. Depth는 image 1, runtime_events 2048, manipulation execution events 100, 그 밖의 토픽 5다. 구독의 존재와 발행 주기는 보장하지 않는다.

HTTP/WebSocket은 `host=127.0.0.1`, `port=8768`이 기본값이다. 주요 경로는 `/api/snapshot`, `/api/camera`, `/api/recordings`, `/api/recordings/{session}/replay`, `/ws`이며 전체 route는 [server.py](../cleany_dev_monitor/server.py)를 따른다. 기록 설정 API는 Monitor 내부 기록을 변경하며 로봇 구동 명령은 보내지 않는다. [관측 데이터 계약](data.md)을 함께 확인한다.
