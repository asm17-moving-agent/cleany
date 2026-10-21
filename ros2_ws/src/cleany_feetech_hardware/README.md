# cleany_feetech_hardware

Feetech STS3215/STS3250용 native `ros2_control` hardware plugin.
양팔·gripper 12개 position 명령과 머리 2개 position 상태를 제공한다.
기본 기동은 READ 전용이며 쓰기 모드는 로봇별 외부 프로파일을 명시한다.

## 실행과 검증

설정과 기동은 `cleany_robot_interface/real_control.launch.py`가 담당한다. 로봇별 `profile_file`과 `control_file`을 명시한다. 기본은 READ 전용이며 쓰기 모드는 `read_only:=false`로 선택한다. 실제 실행 절차는 [RUNNING](../../../docs/RUNNING.md#jetson-실물-실행-경로)을 따른다.

`make test-jetson-native`는 C++ 통신과 명령 검사를 실행한다. `make test-jetson-packages`는 프로파일과 상태 검사를 포함한다. `cleany_robot_interface/runtime_test/`의 controller 검사는 PTY 가상 모터를 사용하는 별도 명시 실행 검사다.

## 구현 계약

정상 collection 프로필은 `serial_read_retries=1`,
`serial_read_recovery_budget_ms=60`으로 READ 응답 timeout을 한 번 재시도한다.
READ 응답 지연만 복구 대상으로 분류하며, port 단절·WRITE 실패·checksum/ID/길이
오류·모터 packet error에는 적용하지 않는다. 활성 운전 중 재시도 전에 마지막 유효
실측 위치로 명령 관절을 hold하고 토크를 유지한다. 복구 후 기존 부분 프레임을 버리고
14개 모터를 모두 새로 읽는다. 전체 snapshot의 기존 100ms 한계와 반복 오류 차단은
유지한다. 다음 write도 새 실측 자세를 hold하며, executor는 복구 횟수를 받아 기존
Action 취소/남은 경로 재검사 후 재개한다. 초기 configure/activate READ에도 제한된
재읽기를 적용하며, 오래된 상태를 새 feedback으로 표시하지 않는다.
설정이 없는 기존 프로필은 `serial_read_retries=0`으로 즉시 실패 동작을 유지한다.

| 항목 | 동작 |
|---|---|
| 버스 | 양팔 ID 1–12와 머리 ID 13–14, 1Mbps. 현재 실물 설정은 `…5B14033435-if00` 공용 포트 하나에서 14개 모터를 읽는다. 같은 실경로의 `arm_port`와 `head_port`는 포트를 중복으로 열지 않는다. 머리 쓰기 차단은 동일하다. |
| 포트 보호 | `flock`·`TIOCEXCL`; lifecycle·읽기·쓰기를 같은 mutex로 직렬화 |
| 시작 검사 | 14개 모델, 팔 homing/min/max·position mode 대조 |
| 변환 | 기존 방향·영점·offset 재사용; homing 중복 적용·범위 강제 자르기 없음 |
| 상태 | 14관절 실측 position과 양팔 12관절 원시/명령 telemetry 제공; 머리 명령 없음 |
| 쓰기 차단 | `read_only=true`는 쓰기 함수·controller 활성화를 모두 거부 |
| 활성화 | 명시적 쓰기 모드에서 토크 OFF·현재 자세 확인 → 현재 목표 사전 기록 → RAM 제한 확인 → 토크 ON |
| 토크 유지 복구 | `allow_torque_on_recovery=true`일 때만 명령 모터 전체가 토크 ON이고 status·온도·현재 목표가 정상인 기존 hold를 인수한다. 목표와 실측 차이는 최대 32 tick이다. 활성화 시에도 목표와 RAM 설정을 READ로 재검사하고, 기존 목표를 ROS 시작 명령으로 사용한다. 목표·RAM·토크를 재기록하지 않으며 설정 불일치는 거부한다. |
| 목표 | 명령 인터페이스가 있는 관절을 먼저 모두 검사한 뒤 한 sync-write; broadcast 성공은 모터 도착 확인이 아님 |
| 명령 검사 | 유한값·URDF/EEPROM 범위·상태 나이. `tracking_error>0`이면 추종 오차도 검사하며 `0`은 감독형 시험의 비활성화 sentinel |
| 속도 제한 | controller 주기 흔들림에 민감한 명령 미분 fault 대신, 실제 주기와 `max_velocity`로 다음 wire 목표의 최대 이동량을 제한한다. 가속도는 MoveIt/JTC 궤적 한계가 담당한다. |
| 취소·목표 실패 hold | 목표가 현재 실측 tick과 같으면 즉시 실측 위치를 hold한다. 지연된 목표에서 현재 위치로 멈추는 명령을 허용하며 범위·상태 검사는 유지한다. |
| 실물 피드백 | 모든 관절을 순차 조회한 완전한 프레임만 갱신. checksum을 통과한 status·torque 이상치는 즉시 재조회한다. 운전 중 과온은 설정된 연속 완전 스냅샷 수(정상 프로필 3회)로 확인하며, 시작 시 과온·지속 과온·복구되지 않은 통신 실패는 fault로 고정한다. 온도 한계값 자체는 완화하지 않는다. |
| 장애 | 새 목표 차단, 통신 가능한 모터에 현재 위치 hold 시도; 자동 재연결·재시작 없음 |
| controller 중지 | 해당 관절을 최초 정지 위치에 hold; 이후 fault·비활성화·정리·소멸의 중복 hold는 그 목표를 재사용한다. 처진 실측 위치로 목표를 반복해서 낮추지 않는다. 모든 명령 controller가 중지되면 드라이버 fault로 고정 |
| 종료 | 토크를 자동으로 끄지 않음; 쓰기 모드에서는 현재 위치 hold 시도 후 포트 해제 |

RAM 쓰기는 ID 1–12의 `40=Torque_Enable(1)`, `41=Acceleration`,
`42=Goal_Position`, `44=Goal_Time(0)`, `46=Goal_Velocity`, `48=Torque_Limit`만 허용한다.
EEPROM·머리 목표·자동 토크 OFF 쓰기는 제공하지 않는다.
controller가 아직 점유하지 않았거나 반납한 관절에는 주기적 목표 쓰기를 보내지 않아
모터의 기존 hold를 유지한다. 활성 controller의 새 명령을 전송하면 해당 관절의 정지
목표 기록을 해제하므로 이후 정지는 새 실측 위치에서 수행한다. READ 통신 복구의
실측 정지 동작과 실제 움직임 중 최초 정지의 목표 재설정은 유지한다.

부분 운전 프로필의 `command_joints`로 명령 관절을 제한할 수 있다. 생략하면 기존 12관절이다.
선택하지 않은 관절은 상태 인터페이스만 제공하며 RAM·목표·토크 쓰기를 하지 않는다.
부분 관절 프로필은 `with_moveit:=false`로 기동한다. controller와 상태 감시 노드도 해당
프로필의 controller 목록을 사용한다. 시험용 profile은 `cleany_robot_interface/test/data/`에 합성 입력으로만 보관하며 배포 기본값으로 사용하지 않는다.

TCP 지점 저장·재방문은 `tools/tcp-waypoints`와 `tcp_waypoints.launch.py`를 사용한다.
선택한 팔의 5관절만 명령하며, 고정된 세션 모델을 plan-only MoveIt과 native controller가
공유한다. `allow_torque_on_recovery=true`로 수동 저장 때의 전체 5관절 hold를 인수한다.
실행 방법과 측정 정의는 [robot interface](../cleany_robot_interface/docs/interfaces.md#tcp-지점-저장과-재방문)를 따른다.

## ROS 인터페이스

| 인터페이스 | 역할 |
|---|---|
| `/{left,right}_arm_controller/follow_joint_trajectory` | 팔 5관절씩 표준 `FollowJointTrajectory` |
| `/{left,right}_gripper_controller/follow_joint_trajectory` | gripper 1관절씩 동일 action |
| `/joint_state_broadcaster/joint_states` | controller 원시 상태; 미지원 velocity/effort는 NaN일 수 있음 |
| `/joint_states` | 상태 감시 노드가 검사한 14개 position; velocity/effort는 빈 배열 |
| `/real_control/status` | `waiting/ready/fault`, 중단 이유·controller 중단 확인; JSON String |
| `/real_control/stop` | `std_srvs/Trigger`; 중단 고정·controller 비활성 요청 |

`status.state=ready`는 관절 상태 수신이 정상이라는 뜻이다. controller 활성·카메라·전체 수거 준비를 뜻하지 않는다.

상태 감시 노드는 누락·NaN·오래된/정지된 timestamp를 차단한다. 장애나 소프트웨어 중단 시
각 FollowJointTrajectory의 전체 goal 취소를 직접 요청해 액션을 non-success로 종료하고 controller를 비활성화한다. `stop`의
즉시 응답은 요청 접수이며, 완료 여부는 `status.actions_cancelled`와
`status.controllers_stopped`로 확인한다. 물리 정지 확인값은 아니다.

## 실물 구동 전 조건

controller limits와 motion monitor 설정은 외부 파일로 제공한다. 검사 비활성화와 torque-on 인수는 명시된 profile의 정책이며 공유 기본값으로 배포하지 않는다. 드라이버는 설정 온도와 모터 EEPROM 보호온도 중 낮은 값을 적용하고 EEPROM은 쓰지 않는다. host 중단과 USB 단절 때 마지막 목표를 유지할 수 있으므로 motor-side 정지 수단과 하중 지지는 별도 준비 조건이다.

구성 근거: [Humble hardware plugin](https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html) ·
[Joint Trajectory Controller](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html).
Feetech wire 형식·주소는 설치된 `feetech-servo-sdk==1.0.0`와 기존 READ/실행 도구를 대조했다.

## 팔·그리퍼 telemetry state interfaces

ID 1–12: position 외 load_raw, torque_enabled, motor_status, temperature,
voltage_raw, goal_position_raw, command_position, sent_position, sent_goal_raw를 공개한다.
추가로 position_raw, current_raw, load_magnitude_raw, moving_flag, protocol_error_code,
torque_limit_readback_raw, current_limit_raw, temperature_raw, motor_status_raw,
feedback_stamp_ns와 communication_recovery_count를 공개한다.
기존 position-only/5/10/20-interface 모델도 허용한다.
address 40부터 31byte READ로 current(69–70)까지 함께 읽으며
추가 serial owner·register WRITE·토크 변경은 없다. 스냅샷 전체 성공 시 함께 갱신하고 fault 시
마지막 완전한 스냅샷을 보존하되 timestamp가 갱신되지 않아 stale로 판정된다. load_raw는 보정된 힘/토크가 아니다. 모터 상태 fault 로그에는
status·temperature·torque 원시 값을 포함하여 후속 원인 확인이 가능하게 한다.

current_raw는 bit15 부호를 제거한 전류 크기, load_magnitude_raw는 bit10 부호를
제거한 PWM 크기다. temperature_raw/motor_status_raw는 재조회 보정 전 원본 값이다.
packet error/checksum 실패는 snapshot 자체를 거부한다. current_limit_raw는
EEPROM 28–29 readback이며 EEPROM 쓰기는 없다. ID6/12의 powered 전류가 이 한도에
도달하면 재조회로 확인하고 fault/hold한다. joint별 temperature_limit_c는 전역 한도와
모터 보호 한도를 넘길 수 없다. feedback_stamp_ns는 완전한 READ의 ROS acquisition time이다.

- `voltage_raw`: address 62의 원시값, `goal_position_raw`: address 42–43에서 읽은 목표 tick.
- `command_position`: ROS controller의 위치 명령(rad), `sent_position`/`sent_goal_raw`:
  마지막 성공한 wire 쓰기의 양자화된 위치(rad)/tick. broadcast 쓰기는 수신 ACK가 아니며
  실제 적용 확인은 다음 READ의 `goal_position_raw`와 비교한다. 쓰기 전에는 NaN이다.
- 완전한 READ 프레임과 마지막 쓰기 값을 broadcaster 주기(기본 50Hz)로 함께 공개한다.
  순차 조회이므로 모든 모터의 전기적 동시 샘플이나 순간 차단 검출을 보장하지 않는다.
  fault 후 보존된 값은 새 실측으로 해석하지 말고 `/real_control/status`와 함께 확인한다.
- collection runtime은 read-only ROS subscriber로 `logs/motor-telemetry.jsonl`을 기록한다.
  드라이버 제어 스레드에 파일 I/O나 추가 serial 조회는 넣지 않는다. recorder 자체는
  토크·목표·controller를 변경하지 않는다.
