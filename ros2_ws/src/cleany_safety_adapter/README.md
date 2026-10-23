# cleany_safety_adapter

독립 안전 계층의 상태 증거를 `/safety/status`(`cleany_interfaces/msg/SafetyStatus`)로
System Supervisor에 보고하는 node다. 이 node는 안전 계층이 아니며 구동을 차단하거나
E-stop을 해제하지 않는다. 확인할 수 없는 항목은 항상 안전하지 않은 값으로 보고한다.

## 입력과 판정

| 출력 필드 | 증거 | 증거가 없거나 오래됐을 때 |
|---|---|---|
| `protection_ready` | `protection_ready_topic`(std_msgs/Bool, true=보호 계층 준비) | false |
| `estop_engaged` / `physical_release_confirmed` | `estop_released_topic`(std_msgs/Bool, true=물리 해제 및 회로 reset 완료) | engaged=true, release=false |
| `stopped` | 최신 `odom` twist와 `joint_states` velocity가 모두 정지 임계값 이하 | false |
| `detail` | 판정 이유 | 미설정 source와 stale 항목을 명시 |

- 두 Bool topic의 기본값은 빈 문자열이다. 현재 Cleany에는 E-stop 회로나 독립 보호
  계층의 상태를 ROS로 내보내는 하드웨어가 없으므로, 기본 실행에서 Supervisor는
  `SAFETY_UNVERIFIED`/`ESTOP` Fault로 운영을 잠근다. 이것이 의도된 동작이다.
- Bool topic에는 header가 없으므로 freshness는 수신 monotonic 시각으로 판단한다
  (`evidence_timeout_s`, 기본 0.3초). 하드웨어 adapter가 이 topic을 주기적으로
  발행해야 한다.
- `stopped`는 wheel odometry와 관절 encoder velocity에서 계산한 실제 측정 증거다.
  `monitored_joints`가 비어 있으면 수신된 모든 관절을 요구한다. 이 값은 ROS stack을
  거친 측정값이므로 독립 정지 센서나 STO 상태를 대체하지 않는다.

| Parameter | 기본값 |
|---|---|
| `estop_released_topic` | `''` |
| `protection_ready_topic` | `''` |
| `evidence_timeout_s` | `0.3` |
| `odom_topic` / `joint_states_topic` | `odom` / `joint_states` |
| `stopped_linear_mps` / `stopped_angular_radps` / `stopped_joint_radps` | `0.01` / `0.02` / `0.02` |
| `monitored_joints` | 전체 수신 관절 |
| `rate_hz` | `20.0` |

임계값은 초기 통합값이며 실물 정지 판정 기준으로 검증되지 않았다.

## 실행과 검증

```bash
ros2 run cleany_safety_adapter safety_adapter --ros-args \
  -p estop_released_topic:=/hardware/estop_released \
  -p protection_ready_topic:=/hardware/protection_ready
make test-supervisor-core   # evidence core pytest 포함
make test-supervisor        # Supervisor E2E graph에서 실제 node로 사용
```

Supervisor는 `ros.trusted_publishers`로 `/safety/status` publisher가 `/safety_adapter`
하나뿐인지 검사한다. 이 adapter를 다른 이름으로 실행하면 `UNTRUSTED_PUBLISHER`로 잠긴다.

## 남은 하드웨어 작업

- E-stop 회로(접점/릴레이)와 보호 계층의 실제 상태를 읽어 위 Bool topic을 발행하는
  하드웨어 adapter 또는 MCU 펌웨어. 회로 설계 자체가 KB에서 미정이다.
- 독립 정지 증거(STO 피드백, 모터 드라이버 상태)로 `stopped`를 보강하는 작업.
