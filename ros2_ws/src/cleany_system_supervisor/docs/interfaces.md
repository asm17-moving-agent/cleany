# cleany_system_supervisor 노드 인터페이스

응용 인터페이스와 adapter에서 생성하는 endpoint를 노드별로 정리한다. 이름과 QoS 표기, ROS 자동 인터페이스는 [전체 색인](../../../../docs/ROS_INTERFACES.md)을 따른다. 이름 설정이 없는 항목은 ROS remap으로 변경한다. 동작 계약은 [System Supervisor](supervision.md)를 따른다.

## system_supervisor

실행 파일: `system_supervisor`. 원본: [node.py](../cleany_system_supervisor/node.py), [ros_transport.py](../cleany_system_supervisor/ros_transport.py). 설정: [system_supervisor.yaml](../config/system_supervisor.yaml)의 `ros`.

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 발행 | `system/state` | `cleany_interfaces/msg/SystemState` | R/T1; 20 Hz; epoch, revision, 운영 상태, 전체 MissionPermission |
| 토픽 발행 | `system/capabilities` | `cleany_interfaces/msg/SystemCapabilities` | R/T1; 20 Hz |
| 토픽 발행 | `system/faults` | `cleany_interfaces/msg/SystemFaults` | R/T1; 20 Hz; 활성과 retention 중인 해제 Fault |
| 토픽 발행 | `system/recoveries` | `cleany_interfaces/msg/SystemRecoveries` | R/T1; 20 Hz |
| 토픽 발행 | `system/heartbeat` | `cleany_interfaces/msg/SystemHeartbeat` | R1; 20 Hz |
| 토픽 발행 | `system/events` | `cleany_interfaces/msg/SystemEvent` | R100; 이벤트 발생 시 |
| 토픽 발행 | `diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | R10; `diagnostics_period_s` 1초; `system_supervisor: ...` status |
| 토픽 구독 | `diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | R10; `base_driver` status만 사용; `diagnostics_topic` |
| 토픽 구독 | `scan` | `sensor_msgs/msg/LaserScan` | Sensor; `scan_topic` |
| 토픽 구독 | `amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Sensor; `pose_topic` |
| 토픽 구독 | `camera/head/color/image_raw` | `sensor_msgs/msg/Image` | Sensor; `camera_topic` |
| 토픽 구독 | `system/component_health` | `cleany_interfaces/msg/ComponentHealth` | R10; `component_sources`의 source만 수락 |
| 토픽 구독 | `safety/status` | `cleany_interfaces/msg/SafetyStatus` | R10; `safety_topic` |
| 서비스 호출 | `<nav2 node>/get_state` | `lifecycle_msgs/srv/GetState` | Service 기본값; `lifecycle_poll_s` 0.5초; `navigation_nodes`, `localization_nodes` |
| 서비스 제공 | `system/check_permission` | `cleany_interfaces/srv/CheckPermission` | Service 기본값 |
| 서비스 제공 | `system/request_fault_reset` | `cleany_interfaces/srv/RequestFaultReset` | Service 기본값; `operator_id` 필수 |
| 액션 제공 | `system/request_recovery` | `cleany_interfaces/action/RequestRecovery` | Action 기본값 |
| 액션 호출 | `mission/stop` | `cleany_interfaces/action/StopMission` | Action 기본값; `stop_action`; 서버는 `mission_runtime` |
| 액션 호출 | `<source>/recover` | `cleany_interfaces/action/RecoverComponent` | Action 기본값; `recovery_policies.*.action` |

Graph 조회로 `trusted_publishers`의 topic publisher를 1초마다 검사한다. 상태 snapshot의 retained 값은 허가가 아니며, client는 heartbeat freshness, epoch와 lease 만료를 별도로 검사한다.

## supervisor_operator

실행 파일: `supervisor_operator`. 원본: [operator_cli.py](../cleany_system_supervisor/operator_cli.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `system/heartbeat` | `cleany_interfaces/msg/SystemHeartbeat` | R1 |
| 토픽 구독 | `system/faults` | `cleany_interfaces/msg/SystemFaults` | R/T1 |
| 서비스 호출 | `system/request_fault_reset` | `cleany_interfaces/srv/RequestFaultReset` | Service 기본값; reset 인자가 있을 때만 |

인자 없이 실행하면 활성 Fault를 나열하고, `<FAULT_CODE> --operator <name>`은 reset을 요청한다. 하드웨어를 해제하지 않는다.
