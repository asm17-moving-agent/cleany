# cleany_feetech_hardware

Feetech STS3215 및 STS3250 서보 모터를 위한 네이티브 `ros2_control` 하드웨어 플러그인입니다.
- **제어 범위**: 양팔 및 그리퍼 12개 관절의 위치(Position) 명령 제어 + 머리 2개 관절의 위치 상태 조회
- **구동 모드**: 기본 기동은 **READ 전용(읽기 전용)** 모드이며, 모터 쓰기(제어) 모드는 로봇별 외부 프로파일을 명시할 때만 활성화됩니다.

## 1. 실행 및 검증

설정과 기동은 `cleany_robot_interface/real_control.launch.py`가 담당합니다. 로봇별 `profile_file`과 `control_file`을 명시해야 합니다.

```bash
# 기본 READ 전용 모드 실행 (모터 쓰기 차단)
ros2 launch cleany_robot_interface real_control.launch.py

# 쓰기 허용 모드 실행 (로봇 프로파일 명시)
ros2 launch cleany_robot_interface real_control.launch.py read_only:=false
```

- **테스트 및 검증**:
  - C++ 통신 및 패킷 검증: `make test-jetson-native`
  - 프로파일 및 상태 감시 검증: `make test-jetson-packages`
  - 가상 시리얼(PTY) 기반 제어기 검사: `make test-jetson-controller`

## 2. 구현 계약 및 안전 정책

### 통신 복구 정책 (Read Recovery Budget)
정상 수거 프로파일에는 `serial_read_retries=1`, `serial_read_recovery_budget_ms=60`이 적용되어 모터 응답 지연 시 1회 재시도합니다.
- **복구 대상**: 일시적인 READ 응답 지연/타임아웃에만 적용됩니다. (포트 단절, WRITE 실패, 체크섬 오류 등에는 미적용)
- **자세 유지(Hold)**: 재시도 중에는 마지막으로 확인된 유효 실측 위치로 모터 목표를 고정하여 토크를 유지합니다.
- **완전한 프레임 갱신**: 복구 성공 후 부분 프레임을 버리고 14개 모터 전체를 새로 읽어 스냅샷을 갱신합니다. (100ms 타임아웃 유지)
- **설정 미지정 시**: 기존 프로파일은 `serial_read_retries=0`으로 즉시 실패 처리됩니다.

| 항목 | 동작 규격 및 안전 조치 |
|---|---|
| **시리얼 버스** | 양팔(ID 1–12) 및 머리(ID 13–14), 1Mbps 통신. 현재 실물 설정은 공용 포트 하나에서 14개 모터를 통합 조회하며 머리 쓰기는 원천 차단 |
| **포트 보호** | `flock` 및 `TIOCEXCL`로 다중 점유 방지; 라이프사이클·읽기·쓰기를 동일 Mutex로 직렬화 |
| **기동 검사** | 14개 모터 모델 번호, 홈(Homing), 최소/최대 한계 및 위치 제어 모드를 EEPROM과 대조 |
| **변환** | 기존 방향·영점·오프셋 재사용; 불필요한 범위 강제 자르기 없음 |
| **상태 제공** | 14관절 실측 위치 및 양팔 12관절 텔레메트리 제공; 머리 제어 명령 없음 |
| **쓰기 차단** | `read_only=true`일 경우 쓰기 함수 및 Controller 활성화를 즉시 거부 |
| **활성화 순서** | 쓰기 모드 기동 시: 토크 OFF 상태에서 현재 자세 확인 → 목표 사전 기록 → RAM 한계 확인 → 토크 ON |
| **토크 유지 복구** | `allow_torque_on_recovery=true`일 때만 목표-실측 편차 32 tick 이하인 기존 정상 Hold 상태를 모터 쓰기 없이 안전하게 인수 |
| **목표 전송** | 명령 인터페이스 관절을 먼저 검사한 뒤 동기 쓰기(Sync Write) 수행 |
| **명령 검사** | 유한값, URDF/EEPROM 범위, 상태 신선도를 검사하여 비정상 명령 차단 |
| **속도 제한** | 실제 루프 주기와 `max_velocity`를 기반으로 모터 전송 목표의 최대 이동량 제한 |
| **정지 유지** | 목표가 현재 실측 tick과 같으면 즉시 실측 위치로 고정(Hold) |
| **이상 피드백** | 모든 관절을 순차 조회한 완전한 프레임만 갱신. 연속 과온 또는 통신 불가 시 안전 Fault 고정 |
| **장애 대응** | 신규 목표 즉시 차단, 통신 가능한 모터에 현재 위치 Hold 시도 (자동 재연결 없음) |
| **종료 동작** | 모터 주저앉음을 방지하기 위해 토크를 자동 OFF하지 않음; 현재 위치 Hold 후 포트 해제 |

> [!CAUTION]
> 프로세스 종료 시 모터 토크가 자동 해제되지 않으므로, 작업 완료 후 모터 토크를 풀려면 명시적인 도구(`tools/left-arm-torque-off`, `tools/right-arm-torque-off`)를 실행해야 합니다.

## 3. ROS 인터페이스

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
| **온도 및 상태** | `temperature`, `motor_status`, `protocol_error_code` | 모터 내부 온도(℃), 하드웨어 상태 플래그, 통신 프로토콜 에러 코드 |
| **명령 및 추종** | `command_position`, `sent_position`, `goal_position_raw` | ROS 위치 명령(rad), 시리얼 전송된 양자화 위치(rad), 모터 레지스터 목표 tick |
| **통신 품질** | `feedback_stamp_ns`, `communication_recovery_count` | 스냅샷 취득 시각(ROS timestamp), READ 응답 지연 복구 누적 횟수 |

- **안전한 기록 보관**: 수거 런타임은 제어 루프와 완전히 분리된 백그라운드 ROS 구독기를 통해 `logs/motor-telemetry.jsonl`에 주기적(기본 50 Hz)으로 기록합니다.
- **이상치 대응**: 체크섬 오류나 일시적 통신 실패 시 마지막 정상 스냅샷을 유지하되 타임스탬프를 갱신하지 않아 상위 감시기가 지연(Stale) 상태를 즉시 감지할 수 있도록 설계되었습니다.
