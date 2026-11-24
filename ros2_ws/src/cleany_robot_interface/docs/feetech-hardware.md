# Feetech 하드웨어 플러그인

`cleany_feetech_hardware/FeetechSystem`은 `cleany_robot_interface`에 포함된
Feetech STS3215 및 STS3250 서보 모터용 `ros2_control` 하드웨어 플러그인입니다.
- **제어 범위**: 양팔 및 그리퍼 12개 관절의 위치(Position) 명령 제어 + 머리 2개 관절의 위치 상태 조회
- **구동 모드**: 기본 기동은 **READ 전용(읽기 전용)** 모드이며, 모터 쓰기(제어) 모드는 로봇별 외부 프로파일을 명시할 때만 활성화됩니다.

## 1. 실행 및 검증

실물 제어 실행은 [실행 가이드](../../../../docs/RUNNING.md#jetson-실물-실행-경로),
C++ 드라이버와 가상 시리얼 검사는 [테스트 가이드](../../../../docs/TESTING.md#jetson-실물-변경-검증)를 따릅니다.

## 2. 구현 계약 및 안전 정책

제어 프로파일의 `joint_defaults`와 `joint_overrides`를 합친 관절별 속도 및
가속도 제한은 MoveIt `joint_limits.yaml`에 적용한다.
Jetson 양팔 운영 프로파일의 그리퍼 궤적 상한은 속도 2.0 rad/s, 가속도 8.0 rad/s²다.
양쪽 그리퍼의 모터 가속도는 각 모터에서 확인한 상한인 raw 50을 사용한다.
토크 OFF에서 기동할 때 제어기는 쓰기가 활성화된 STS3215의 최대 가속도(주소 85, 1바이트)를 먼저 읽는다.
요청값이 상한을 넘으면 RAM 설정 전에 관절명, 모터 ID, 요청값과 상한을 기록하고 준비를 중단한다.
드라이버는 위치 명령을 ROS 관절 범위로 clamp한 뒤 tick으로 변환해 전송한다.
전송 목표의 속도를 다시 제한하거나 별도 추종오차 Fault를 걸지 않으며,
추종오차는 JTC의 궤적 및 목표 허용오차로 처리한다. 기존 프로파일의
`tracking_error`는 사용하지 않는다. 모터의 `speed_raw`, `acceleration_raw`,
`torque_limit_raw` 설정과 기동 검사는 유지한다.

### 통신 복구 정책 (Read Recovery Budget)
정상 수거 프로파일에는 `serial_read_retries=1`, `serial_read_recovery_budget_ms=60`이 적용되어 모터 응답 지연 시 1회 재시도합니다.
- **복구 대상**: READ 응답 지연이나 잘못된 프레임에 적용합니다. 같은 시간 한도와 256바이트 한도 안에서 유효한 프레임을 찾고, 설정된 횟수만큼 읽기를 다시 시도합니다. 모터 상태 오류, 포트 단절과 WRITE 실패에는 재시도를 적용하지 않습니다.
- **자세 유지(Hold)**: 재시도 중에는 마지막으로 확인된 유효 실측 위치로 모터 목표를 고정하여 토크를 유지합니다.
- **완전한 프레임 갱신**: 복구 성공 후 부분 프레임을 버리고 14개 모터 전체를 새로 읽어 스냅샷을 갱신합니다. (100ms 타임아웃 유지)
- **설정 미지정 시**: 기존 프로파일은 `serial_read_retries=0`으로 즉시 실패 처리됩니다.

| 항목 | 동작 규격 및 안전 조치 |
|---|---|
| **시리얼 버스** | 양팔(ID 1–12) 및 머리(ID 13–14), 1Mbps 통신. 현재 실물 설정은 공용 포트 하나에서 14개 모터를 통합 조회하며 머리 쓰기는 원천 차단 |
| **포트 보호** | `flock` 및 `TIOCEXCL`로 다중 점유 방지; 라이프사이클·읽기·쓰기를 동일 Mutex로 직렬화 |
| **기동 검사** | 14개 모터 모델 번호, 홈(Homing), 최소/최대 한계 및 위치 제어 모드를 EEPROM과 대조 |
| **변환** | 기존 방향·영점·오프셋 재사용; 초기 범위 복귀는 calibrated raw tick으로 수행 |
| **상태 제공** | 14관절 실측 위치 및 양팔 12관절 텔레메트리 제공; 머리 제어 명령 없음 |
| **쓰기 차단** | `read_only=true`일 경우 쓰기 함수 및 Controller 활성화를 즉시 거부 |
| **활성화 순서** | 쓰기 모드 기동 시: RAM 속도·가속도·토크 설정 → `clamp(raw, range_min, range_max)` 목표 1회 전송 → 토크 ON → 범위 내 피드백 확인 |
| **토크 유지 복구** | `allow_torque_on_recovery=true`이면 범위 내 정상 Hold 목표를 유지하고, 범위 밖 관절만 가까운 calibrated 경계로 복귀 |
| **목표 전송** | 명령 인터페이스 관절을 먼저 검사한 뒤 동기 쓰기(Sync Write) 수행 |
| **위치 명령** | ROS 관절 범위로 clamp 후 tick 변환. 유한값, 인코더 표현 범위와 상태 신선도 검사 유지 |
| **궤적 제한** | MoveIt에서 속도·가속도 제한, JTC에서 추종오차 처리. 드라이버의 추가 속도 제한 및 추종오차 Fault 없음 |
| **정지 유지** | JTC의 취소·중단 시 Hold 명령도 같은 위치 명령 경로로 즉시 전송 |
| **이상 피드백** | 모든 관절을 순차 조회한 완전한 프레임만 갱신. MACD 필터 온도 초과 또는 통신 불가 시 Fault 고정 |
| **장애 대응** | 신규 목표 즉시 차단, 통신 가능한 모터에 현재 위치 Hold 시도 (자동 재연결 없음) |
| **종료 동작** | 모터 주저앉음을 방지하기 위해 토크를 자동 OFF하지 않음; 현재 위치 Hold 후 포트 해제 |

> [!CAUTION]
> 프로세스 종료 시 모터 토크가 자동 해제되지 않으므로, 작업 완료 후 모터 토크를 풀려면 명시적인 도구(`tools/left-arm-torque-off`, `tools/right-arm-torque-off`)를 실행해야 합니다.

### 초기 범위 복귀

모터 제어가 시작되는 `on_activate()`에서 제어 대상으로 선택한 팔 관절의
토크를 켭니다. 측정 raw tick이 캘리브레이션 범위를 벗어나면 이탈량과
무관하게 가장 가까운 최소·최대 tick을 목표로 보냅니다. IK나 FJT를 거치지
않고 모터의 기존 `speed_raw`, `acceleration_raw`, `torque_limit_raw`를 사용합니다.
RAM 설정을 먼저 기록하고 목표는 한 번만 보낸 뒤, 위치를 조회해 범위 안에
들어왔을 때 활성화를 완료합니다. 복귀 중에는 활성화가 대기 상태입니다.

이미 토크가 켜진 관절은 기존 복구 설정을 따릅니다. 범위 내 Hold는 기존
목표와 RAM 설정을 인수하며 목표-실측 32 tick 검사를 유지합니다. 범위 밖
관절에는 이 편차 검사를 적용하지 않고 경계 목표를 보냅니다. `read_only`
실행, 명령 대상에서 제외된 관절과 머리에는 복귀 명령을 보내지 않습니다.

`CLEANY_REQUIRE_POWERED_HOLD_RECOVERY=1`로 시작한 복구에서는 명령 대상
모터가 모두 토크를 유지하고 있고 현재 위치가 보정 범위 안에 있어야 합니다.
조건이 맞으면 저장된 목표를 유지하며, 제어기가 초기 엔코더 위치를 다시
보내는 동안에도 기존 목표를 보존합니다. 범위 복귀는 일반 기동 경로에서 수행합니다.

### 온도 MACD 필터

모터별 빠른 EMA와 느린 EMA의 차이를 MACD로 계산하고, 그 차이의 EMA를
신호선으로 사용합니다. 판정 온도는 `느린 EMA + 신호선`으로 섭씨 단위를
유지합니다. 각 EMA의 계수는 steady clock의 실제 경과 시간으로
`1 - exp(-dt / tau)`를 계산합니다. 연속 레지스터 응답의 원시 온도가
적용 한도 이상이면 온도 레지스터 63을 한 번 단독 재조회하고 그 값을
필터에 넣습니다. 최초 원시 값은 `temperature_raw`에 보존합니다.
재조회 통신 실패는 기존 통신 Fault로 처리하며, 확인 횟수 카운터와 별도
원시 온도 지속 시간 판정은 사용하지 않습니다.

```yaml
temperature_macd_fast_tau_s: 0.1
temperature_macd_slow_tau_s: 0.5
temperature_macd_signal_tau_s: 0.1
```

세 시정수는 양수이고 `fast < slow`여야 합니다. 기존
`temperature_fault_confirmation_samples`는 제거하고 위 설정으로 교체합니다.
초기 온도는 첫 실측값으로 설정하며, 이미 뜨거운 상태의 기동을 평활화로
숨기지 않습니다. 이후 필터 온도를 기존 `temperature_limit_c`와 비교합니다.
모터 상태와 전류 보호는 온도 필터를 거치지 않습니다. 필터 이력은 유효한
14개 모터 전체 프레임과 함께 갱신하며, 통신 복구로 버린 부분 프레임은
이력을 바꾸지 않습니다.

## 3. ROS 인터페이스

팔과 그리퍼의 궤적 제어기는 제어 프로파일의 `controller.type`으로 선택한다.
생략하면 기존 `joint_trajectory_controller/JointTrajectoryController`를 사용한다.
이벤트 방식의 완료 결과 전달이 필요하면
[`cleany_trajectory_controller/JointTrajectoryController`](../../cleany_trajectory_controller/docs/interfaces.md#설정)를
지정한다. 두 제어기의 Action 이름과 메시지 타입은 동일하다.
지원하지 않는 `controller.type`이나 1~100 Hz 범위의 유한한 숫자가 아닌
`controller.action_monitor_rate`는 로봇 모델을 생성하기 전에 거부한다.

| 인터페이스 | 타입 | 설명 |
|---|---|---|
| `/{arm}_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | 팔 5관절 단위 표준 궤적 추종 액션 |
| `/{gripper}_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | 그리퍼 1관절 단위 구동 액션 |
| `/joint_state_broadcaster/joint_states` | `sensor_msgs/msg/JointState` | 컨트롤러 원시 상태 (미지원 항목은 NaN) |
| `/joint_states` | `sensor_msgs/msg/JointState` | 상태 감시 노드가 검증한 14관절 위치 상태 (20 Hz) |
| `/real_control/status` | `std_msgs/msg/String` (Transient Local) | 제어 상태 (`waiting/ready/fault`), 중단 사유 JSON (20 Hz) |
| `/real_control/stop` | `std_srvs/srv/Trigger` | 비상 정지 및 컨트롤러 비활성화 요청 서비스 |

> [!NOTE]
> `/real_control/status`의 `state=ready`는 관절 센서 피드백이 정상 수신 중임을 의미하며, 모터 토크 활성화나 카메라 준비 완료를 뜻하지 않습니다.
> 비상 정지(`stop`) 호출 시 `status.actions_cancelled`와 `status.controllers_stopped`를 통해 소프트웨어 정지 완료 여부를 확인합니다.

## 4. 실물 구동 전 필수 확인 사항

> [!WARNING]
> - 컨트롤러 한계치(`control_config`) 및 모션 감시(`hardware_monitor_config`)는 외부 YAML 설정 파일로 명시해야 합니다.
> - 드라이버는 설정 온도와 모터 EEPROM 보호 온도 중 더 낮은 값을 적용하며, 임의로 EEPROM을 수정하지 않습니다.
> - 호스트 프로세스 중단 또는 USB 케이블 단절 시에도 모터가 마지막 목표 위치를 유지할 수 있으므로, **물리적인 외부 비상 정지 스위치 및 팔 하중 지지 수단**을 사전에 반드시 준비해야 합니다.

구성 근거: [Humble hardware plugin](https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html) · [Joint Trajectory Controller](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html).
Feetech 통신 패킷 포맷은 `feetech-servo-sdk==1.0.0` 레지스터 맵을 따릅니다.

## 5. 상세 텔레메트리 (Telemetry) 인터페이스

양팔 및 그리퍼 모터(ID 1~12)는 `ros2_control`을 통해 상세한 진단 정보를 제공합니다:

| 분류 | 주요 인터페이스 필드 | 설명 |
|---|---|---|
| **전기 및 부하** | `current_raw`, `load_magnitude_raw`, `voltage_raw` | 실제 모터 전류(mA 단위 변환 전 원시값), PWM 듀티 부하 크기, 구동 전압 |
| **온도 및 상태** | `temperature`, `temperature_raw`, `motor_status`, `protocol_error_code` | MACD 판정 온도(℃), 원시 온도(℃), 하드웨어 상태 플래그, 통신 프로토콜 에러 코드 |
| **명령 및 추종** | `command_position`, `sent_position`, `goal_position_raw` | ROS 위치 명령(rad), 시리얼 전송된 양자화 위치(rad), 모터 레지스터 목표 tick |
| **통신 품질** | `feedback_stamp_ns`, `communication_recovery_count` | 스냅샷 취득 시각(ROS timestamp), READ 응답 지연 복구 누적 횟수 |

- **안전한 기록 보관**: 수거 런타임은 제어 루프와 완전히 분리된 백그라운드 ROS 구독기를 통해 `logs/motor-telemetry.jsonl`에 주기적(기본 50 Hz)으로 기록합니다.
- **이상치 대응**: 체크섬 오류나 일시적 통신 실패 시 마지막 정상 스냅샷을 유지하되 타임스탬프를 갱신하지 않아 상위 감시기가 지연(Stale) 상태를 즉시 감지할 수 있도록 설계되었습니다.

현장 부팅 프로필은 `CLEANY_FIELD_PARK_JOINTS`에 열두 관절의 이름과 실측 정지 각도(rad)를 전달한다. 쓰기 모드 활성화 직전 드라이버가 새 표본에서 양팔의 수납 각도, 두 그리퍼의 열린 각도와 토크 해제를 확인한다. 그리퍼에 물체가 남았는지는 현장에서 확인한다. 허용 편차는 0.05 rad다. 현장 부팅 경로는 기존 토크를 인수하거나 범위를 벗어난 자세를 보정하지 않는다.
