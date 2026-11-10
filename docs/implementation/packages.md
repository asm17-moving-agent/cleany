# 기능별 패키지 지도


패키지 이름보다 작업 영역에서 시작하세요. 각 링크는 역할과 실행 조건, 인터페이스 또는 상세 설정으로 이어집니다.

| 영역 | 구성과 실행 | 상세 계약 |
|---|---|---|
| 매핑·주행 | [Navigation](../../ros2_ws/src/cleany_navigation/README.md), [Bringup](../../ros2_ws/src/cleany_bringup/docs/hardware-mapping.md) | [계획](../../ros2_ws/src/cleany_navigation/docs/planning.md), [매핑](../../ros2_ws/src/cleany_navigation/docs/mapping.md), [안전](../../ros2_ws/src/cleany_navigation/docs/safety.md) |
| 인식·파지 후보 | [Perception](../../ros2_ws/src/cleany_perception/docs/runtime.md), [Grasping](../../ros2_ws/src/cleany_grasping/docs/predictors.md) | [관측 계약](../../ros2_ws/src/cleany_interfaces/docs/observation-contracts.md), [Scene·Grasp](../../ros2_ws/src/cleany_interfaces/docs/scene-and-grasp.md) |
| 조작 실행 | [Skill Executor](../../ros2_ws/src/cleany_skill_executor/docs/manipulation.md), [MuJoCo BT](../../ros2_ws/src/cleany_manipulation_bt/README.md) | [실행 계약](../../ros2_ws/src/cleany_manipulation_bt/docs/execution-contract.md), [MoveIt](../../ros2_ws/src/cleany_moveit_config/docs/planning.md) |
| Mission·허가 | [Mission](../../ros2_ws/src/cleany_mission_manager/docs/manipulation-mock-integration.md), [Supervisor](../../ros2_ws/src/cleany_system_supervisor/docs/supervision.md) | [미션 계약](../../ros2_ws/src/cleany_interfaces/docs/mission.md), [Safety adapter](../../ros2_ws/src/cleany_safety_adapter/docs/interfaces.md) |
| Host·Backend 연결 | [Control bridge](../../ros2_ws/src/cleany_control_bridge/docs/interfaces.md) | [Pose telemetry](../../ros2_ws/src/cleany_telemetry/docs/interfaces.md) |
| 구동·펌웨어 | [Base driver](../../ros2_ws/src/cleany_base_driver/docs/command-and-odometry.md), [ESP32](../../esp32/README.md) | [Wheel protocol](../../ros2_ws/src/cleany_base_interfaces/docs/protocol.md), [Wheel odometry](../../ros2_ws/src/cleany_base_odometry/docs/interfaces.md) |
| 센서·보정 | [IMU](../../ros2_ws/src/cleany_base_driver/docs/imu.md), [ToF](../../ros2_ws/src/cleany_tof/docs/interfaces.md), [Hand-eye](../../ros2_ws/src/cleany_handeye_calibration/docs/offline-tools.md) | [좌표](../../ros2_ws/src/cleany_handeye_calibration/docs/frames-and-data.md) |
| 모델·시뮬레이션 | [Robot model](../../ros2_ws/src/cleany_description/docs/model.md), [Gazebo](../../ros2_ws/src/cleany_gazebo_sim/docs/interfaces.md), [MuJoCo](../../ros2_ws/src/cleany_mujoco_sim/docs/scenes.md) | [Model runtime](../../ros2_ws/src/cleany_description/docs/model-runtime.md), [Backend](../../ros2_ws/src/cleany_mujoco_sim/docs/backends.md) |
| 기록·관측 | [Logger](../../ros2_ws/src/cleany_logger/docs/runtime.md), [Monitor](../../ros2_ws/src/cleany_dev_monitor/docs/data.md), [Display](../../ros2_ws/src/cleany_display/docs/interfaces.md) | [Logger data](../../ros2_ws/src/cleany_logger/docs/data.md), [Monitor API](../../ros2_ws/src/cleany_dev_monitor/docs/interfaces.md) |

패키지별 목록은 사이드 메뉴에서도 찾을 수 있습니다. 모든 노드가 동시에 실행되는 것은 아니므로 실제 구성은 [시스템 연결](integration.md)에서 확인하세요.
