# 이벤트 궤적 제어기

`cleany_trajectory_controller/JointTrajectoryController`는 ROS 2 Humble의
`joint_trajectory_controller` 2.53.3을 상속하는 controller plugin이다.
궤적 보간, 모터 명령, 허용 오차와 도착 판정은 기존 구현을 사용하고,
완료 결과를 기다리는 상위 작업에는 executor 이벤트로 결과를 전달한다.

## 결과 전달과 진행 상태

완료 결과와 진행 상태의 전달 주기를 구분한다.

| 항목 | 기존 제어기 | 이벤트 제어기 |
|---|---|---|
| 성공, 실패와 취소 결과 | `action_monitor_rate` 타이머에서 전달 | 종료를 감지하면 executor를 깨워 전달 |
| 진행 상태 피드백 | `action_monitor_rate` 타이머에서 전달 | 같은 설정의 타이머에서 전달; 처리할 목표가 없으면 타이머 정지 |
| 궤적 보간과 도착 판정 | 기존 JTC 구현 | 기존 JTC 구현 |

종료 상황별 결과는 다음과 같다.

| 항목 | 처리 |
|---|---|
| 성공, 궤적 허용 오차 실패 | 기존 종료 판정과 결과 코드를 전달 |
| 사용자 취소 | 기존 취소와 hold 처리를 수행하고, ROS 취소 상태 전이 뒤 `CANCELED` 전달 |
| 새 명령으로 교체 | 이전 목표를 `ABORTED / INVALID_GOAL`로 종료하고 새 목표 실행 |
| 활성 제어기 비활성화 | 기존 중단 결과를 이벤트로 전달 |

서버에 의한 명령 교체에는 사용자 취소 상태 전이가 없으므로
`EXECUTING → CANCELED`를 직접 시도하지 않는다. 수락한 사용자 취소와
완료 판정이 겹치면 사용자 취소 결과를 전달한다. 목표별 handle은 결과
전달까지 보존하며 다음 명령이 이전 결과를 덮어쓰지 않는다.

Action callback과 결과 전달은 같은 상호 배타 callback group을 사용한다.
제어 update는 guard condition만 알리고, 결과 메시지는 executor에서 발행한다.
운영체제와 RMW의 최악 지연 상한을 보장하지 않으며, 일반 controller state
토픽과 하드웨어 제어 주기는 바꾸지 않는다.

## 인터페이스

| 기본 이름 | ROS 타입 | 역할 |
|---|---|---|
| `<controller_name>/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | 기존 Action 요청, 결과, 취소와 피드백 유지 |
| `<controller_name>/query_state` | `control_msgs/srv/QueryTrajectoryState` | upstream 조회 서비스 유지 |
| `~/controller_state`, `~/state` | `control_msgs/msg/JointTrajectoryControllerState` | upstream 상태 발행 유지 |

## 설정

실물 제어 프로파일에서 다음과 같이 선택한다. 설정은 다음 제어기 생성 시 적용한다.

```yaml
controller:
  type: cleany_trajectory_controller/JointTrajectoryController
  action_monitor_rate: 20.0
```

| 설정 | 기본값과 조건 |
|---|---|
| `controller.type` | 생략 시 `joint_trajectory_controller/JointTrajectoryController`; 위의 두 plugin 이름만 허용 |
| `controller.action_monitor_rate` | 기본 20Hz, 유한한 수 1–100Hz; 이벤트 방식에서는 진행 상태 주기 |

기존 `trajectory_tolerance`, `goal_tolerance`, `goal_time`은 그대로 사용한다.

## 빌드 호환성과 검증

이 plugin은 upstream 내부 상태를 사용하므로 JTC 버전을 2.53.3으로 고정한다.
다른 버전에서는 CMake 구성이 실패한다. 의존 버전을 변경할 때는 취소와
명령 교체를 포함한 종료 동작을 다시 검증한다.

빌드, 종료 동작 검사와 CPU 및 지연 비교는
[테스트 가이드](../../../../docs/TESTING.md#이벤트-결과-전달-검증)를 따른다.
`src/event_controller.cpp`의 목표 수락 절차는 Apache-2.0인
[upstream 2.53.3 구현](https://github.com/ros-controls/ros2_controllers/blob/2.53.3/joint_trajectory_controller/src/joint_trajectory_controller.cpp)을 기반으로 한다.
