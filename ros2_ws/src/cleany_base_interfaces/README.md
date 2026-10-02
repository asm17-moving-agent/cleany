# cleany_base_interfaces

실물 base의 ROS adapter와 ESP32-S3 사이에 사용하는 고정 크기 micro-ROS 메시지다.
원본은 이 패키지의 `msg/`에만 둔다. 펌웨어 빌드는 이 원본으로 C type support를
생성한다. Native USB stream은 XRCE-DDS custom transport 전용이다.
한 MCU의 wheel-command publisher는 하나의 driver가 소유한다.

## 토픽과 단위

| 상대 토픽 | 타입 | 방향 |
|---|---|---|
| `base/wheel_command` | `WheelCommand` | ROS adapter → MCU |
| `base/wheel_state` | `WheelState` | MCU → ROS adapter |

두 토픽은 best effort, volatile, keep last 1이며 명령과 피드백은 50 Hz다.
배열 순서는 **FL, FR, RL, RR**이다. MCU만 PCB 순서 M1, M2, M4, M3로 변환하고
모터와 encoder 극성을 적용한다. ROS가 부호를 다시 반전하지 않는다.
각도는 출력축 rad, 속도는 출력축 rad/s, encoder는 signed int32 누적 count,
PWM은 signed percent다. 출력축 1회전은 3172 counts다.

## 명령 계약 v2

`WheelCommand`는 `protocol_version`, `sequence`, `valid_until_us`, `mode`,
`velocity_rad_s[4]`로 구성한다.

| mode | 값 | 동작 |
|---|---:|---|
| STOP | 0 | 출력 0, 목표 삭제, 구동 허가 해제 |
| ENABLE | 1 | 목표 0으로 구동 허가 설정 |
| VELOCITY | 2 | 구동 허가 상태에서 바퀴 목표 적용 |

- `protocol_version`은 2다. ROS interfaces, driver와 MCU firmware를
  같은 revision으로 빌드해 실행한다.
- `sequence`는 unsigned uint32 증가 번호다.
  `0 < (new - old) mod 2^32 < 2^31`인 명령만 새 명령이다. MCU는 정지와
  연결 단절 뒤에도 마지막 번호를 유지한다. Driver는 새 피드백의
  `last_command_sequence`를 기준으로 다음 번호를 정한다.
- `valid_until_us`는 MCU monotonic microseconds 기준 유효기한이다.
  Driver는 최신 MCU timestamp와 로컬 monotonic 경과 시간으로 계산한다.
  ENABLE과 VELOCITY는 `now < deadline <= now + 250000`을 만족해야 한다.
- ENABLE의 목표는 모두 0이어야 하며, VELOCITY는 유한한 `[-10,10] rad/s` 값이다.
  이미 허가된 상태의 ENABLE이나 중복 명령은 목표와 유효기한을 갱신하지 않는다.
- STOP은 버전, 목표와 유효기한에 관계없이 정지한다. STOP의 순번은 마지막 번호보다
  새로울 때만 반영해, 그보다 오래된 ENABLE이 다시 구동을 허가하지 못하게 한다.

MCU는 매 control tick에서 **명령 처리 전에** 활성 유효기한을 검사한다.
만료하면 목표와 허가를 해제하며, 뒤늦게 온 VELOCITY만으로 재개하지 않는다.
`watchdog_ms=250`은 최대 명령 유효기간을 알리는 필드이며 별도 watchdog 시각을
관리하지 않는다. 거부한 명령과 동일 ENABLE 재전송은 유효기한을 연장하지 않는다.

## 구동과 정지

MCU는 PWM 0, `enabled=false`로 부팅한다. Driver의 `/base/enable` true 요청은
이전 `/cmd_vel`을 버리고 zero ENABLE을 보낸다. 전달 중 손실은 동일한 순번과
유효기한의 ENABLE로 재시도한다. MCU의 enabled 확인 후에는 요청 이후 수신한
새 목표만 전송한다. 아직 목표가 없으면 zero VELOCITY로 대기한다.

STOP, 명령 만료, 연결 단절과 재부팅 뒤에는 새 명시적 ENABLE과 새 `/cmd_vel`이
필요하다. 연결 복구와 피드백 수신만으로 이전 목표를 복원하지 않는다.
Callback은 최신 명령 mailbox와 우선 STOP latch를 사용하며, STOP과 연결 단절은
대기 명령을 버린다. PWM과 PI 계산은 독립 motor-control task가 담당한다.

## 피드백

`WheelState.boot_id`는 부팅마다 생성하는 nonzero uint32이며, 재부팅과 encoder
baseline 변경 감지에만 사용한다. 구동 명령에는 포함하지 않는다.
`timestamp_us`는 MCU monotonic 시간이며 ROS epoch가 아니다. JointState stamp는
ROS 수신 시각이다. 피드백 `sequence`는 부팅 내 증가 번호다.
Driver는 중복과 역순 피드백을 무시하고, 누락 개수 대신 실제 수신 지연을 감시한다.

`enabled`는 소프트웨어 구동 허가다. `command_age_ms=65535`는 활성 유효 명령이
없음을 뜻한다. Fault bit 0은 명령 만료이며 새 ENABLE 수용 시 초기화한다.
Adapter는 encoder rollover를 modular delta로 누적한다. 재부팅과 피드백 단절 뒤에는
baseline을 재설정하고 누적 각도를 유지한다. 단절 중 이동량은 관측하지 못한다.

## 빌드

Ubuntu 22.04 / Humble 환경에서 레포 루트 기준:

```bash
make build-base test-base
make firmware-smoke firmware-build
```

펌웨어와 Agent 준비는
[`esp32/README.md`](../../../esp32/README.md)와
[`docs/DEVELOPMENT_SETUP.md`](../../../docs/DEVELOPMENT_SETUP.md)를 따른다.
