# cleany_base_interfaces

실물 base의 ROS adapter와 ESP32-S3 사이에 사용하는 고정 크기 micro-ROS 메시지다.
원본은 이 패키지의 `msg/`에만 둔다. 펌웨어 빌드는 이 패키지를 official
ESP-IDF component의 extra package 입력으로 사용해 C type support를 생성한다.
기존 COBS 바이너리 프로토콜과 다른 계약이며 두 transport를 같은 USB stream에서
동시에 실행하지 않는다.

## 토픽과 단위

| 상대 토픽 | 타입 | 방향 |
|---|---|---|
| `base/wheel_command` | `WheelCommand` | ROS adapter → MCU |
| `base/wheel_state` | `WheelState` | MCU → ROS adapter |

두 토픽은 best effort, volatile, keep last 1이다. 명령은 50 Hz, 피드백은 50 Hz다.
명령 재전송보다 최신 값과 MCU watchdog을 우선한다.
배열 순서는 **FL, FR, RL, RR**이다. MCU만 PCB 순서 M1, M2, M4, M3로 변환한다.
회전 부호는 기존 논리 부호와 encoder 극성을 유지한다. ROS가 다시 반전하지 않는다.
각도는 출력축 rad, 속도는 출력축 rad/s, encoder는 signed 32-bit 누적 count,
PWM은 signed percent다. Encoder는 출력축 1회전당 3172 counts다.
IMU 메시지와 orientation은 현재 계약에 포함하지 않는다.

## 식별자와 시간

- `protocol_version`: 현재 1. 다른 버전은 구동 명령으로 수용하지 않는다.
- `boot_id`: MCU 부팅마다 생성하는 nonzero random uint32.
- `session_id`: ROS adapter가 새 연결마다 생성하는 nonzero random uint32.
- command `sequence`: session 내 unsigned 32-bit 증가 번호. 차이가
  `0 < (new - old) mod 2^32 < 2^31`인 명령만 새 명령이다.
- state `sequence`: 부팅 내 피드백 증가 번호. `last_command_sequence`는 MCU가
  마지막으로 수용한 명령 번호다.
- `timestamp_us`: MCU monotonic microseconds. **ROS epoch로 변환하지 않는다.**
  Adapter는 수신 시 ROS clock으로 JointState에 stamp를 붙인다.
- `valid_until_us`: MCU monotonic 명령 유효기한. Adapter는 최신 MCU stamp와
  로컬 monotonic 경과 시간을 사용해 보수적으로 계산한다. MCU는 이미 지난
  deadline이나 watchdog보다 먼 미래의 deadline을 수용하지 않는다.
  STOP은 deadline과 session 일치 여부에 무관하게 정지를 요청할 수 있다.

## 구동 허용 절차

1. MCU는 PWM 0, session 0, disarmed로 부팅한다. Agent 연결은 구동 허가가 아니다.
2. Adapter는 최신 상태의 버전, boot ID, count scale, wheel limit와 watchdog을
   확인하고 새로운 session의 **zero BEGIN_SESSION**을 보낸다.
3. MCU는 새 session을 수용하면서 이전 목표, PI와 구동 허가를 초기화한다.
4. Adapter의 `base/enable` (`std_srvs/SetBool`)에 명시적인 true 요청이 있어야
   **zero ARM**을 보낸다. 이전 `/cmd_vel`은 폐기한다.
5. armed feedback 확인 뒤 수신한 새로운 유효 `/cmd_vel`만 VELOCITY로 전달한다.
6. STOP, 통신 단절, watchdog 또는 재부팅 뒤에는 이전 목표를 복원하지 않는다.
   MCU watchdog은 250 ms이며 새 유효 명령만 갱신한다. 재개에는 새 연결 절차,
   명시적 enable과 새로운 `/cmd_vel`이 필요하다.

ARM과 VELOCITY는 boot/session, sequence, deadline, 유한 값과 `[-10,10] rad/s`
범위를 검증한다. BEGIN_SESSION과 ARM의 wheel 목표는 모두 0이어야 한다.
Callback은 검증된 목표를 넘기는 역할이며 PI 계산은 독립 motor-control task가 한다.

## 피드백 해석

`command_age_ms=65535`는 유효 명령이 아직 없음을 뜻한다. Fault bit 0은 command
watchdog timeout이며 새 session에서 초기화한다. `armed`는 소프트웨어 구동 허가이고
물리 emergency stop 상태를 뜻하지 않는다.

Adapter는 rollover를 signed modular count 차이로 누적한다. boot 변경 시 새 count를
baseline으로 삼고 기존 누적 각도를 유지해 odometry에 가짜 이동을 만들지 않는다.
재부팅 및 피드백 중단 동안의 실제 이동은 관측할 수 없다.

## 빌드

레포 루트에서 Ubuntu 22.04 / Humble 환경을 적용한 뒤 실행한다.

```bash
make build-base
```

펌웨어와 Agent 준비 명령은
[`motor_controller/README.md`](../../../../motor_controller/README.md)와
[`docs/DEVELOPMENT_SETUP.md`](../../../../docs/DEVELOPMENT_SETUP.md)를 따른다.
