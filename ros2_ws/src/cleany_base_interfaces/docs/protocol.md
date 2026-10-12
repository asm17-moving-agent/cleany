# Wheel protocol v2

ROS driver와 MCU는 이 패키지의 msg/를 원본으로 같은 계약의 type support를 빌드한다. 한 MCU의 command publisher는 driver 하나가 소유한다.

## 인터페이스와 단위

| 토픽 | 방향 | QoS / 주기 |
|---|---|---|
| base/wheel_command | ROS → MCU | best effort, volatile, keep last 1 / 50 Hz |
| base/wheel_state | MCU → ROS | best effort, volatile, keep last 1 / 50 Hz |

| 항목 | 계약 |
|---|---|
| 배열 순서 | FL, FR, RL, RR |
| PCB 매핑 | MCU에서만 M1, M2, M4, M3 및 극성 적용 |
| 각도 / 속도 | 출력축 rad / rad/s |
| Encoder / PWM | signed int32 count / signed percent |
| Encoder scale | 출력축 1회전 = 3172 counts |

## 명령 수락

| Mode | 값 | 동작 |
|---|---:|---|
| STOP | 0 | 목표 삭제·출력 0·허가 해제 |
| ENABLE | 1 | 네 목표가 0일 때 구동 허가 |
| VELOCITY | 2 | 허가된 상태에서 유한한 [-10,10] rad/s 목표 적용 |

Protocol version은 2다. Sequence는 uint32이며 `0 < (new-old) mod 2^32 < 2^31`일 때만 새 명령이다. MCU는 정지·단절 뒤에도 마지막 번호를 유지하고, driver는 fresh feedback의 last_command_sequence 다음 번호를 사용한다.

`valid_until_us`는 MCU monotonic µs다. Driver는 MCU stamp와 로컬 monotonic 경과로 계산한다. ENABLE/VELOCITY는 `now < deadline <= now+250000`을 만족해야 한다.

| 예외 | 처리 |
|---|---|
| 중복·거부·이미 enabled인 ENABLE | 목표와 기한 갱신 없음 |
| STOP | 버전·목표·기한에 관계없이 정지; 새 sequence일 때만 번호 갱신 |
| Deadline 만료 | 목표·허가 삭제 |

STOP의 새 sequence는 오래된 ENABLE의 재허가를 막는다. 매 control tick에서 **명령 처리 전에** 기한을 검사한다. watchdog_ms=250은 최대 유효기간이며 별도 watchdog 시각이 아니다.

## 정지와 재개

부팅·STOP·만료·단절 뒤에는 새 ENABLE과 새 cmd_vel이 필요하다. 연결 복구만으로 이전 목표를 재개하지 않는다. STOP latch와 단절은 대기 명령을 지우며, PWM/PI는 별도 motor task가 계산한다.

## 피드백

| 필드 | 의미 |
|---|---|
| boot_id | 부팅별 nonzero uint32; reboot/baseline 감지 |
| timestamp_us | MCU monotonic; JointState는 ROS 수신 stamp 사용 |
| enabled | 소프트웨어 구동 허가 |
| command_age_ms=65535 | 활성 유효 명령 없음 |
| Fault bit 0 | 명령 만료; 새 ENABLE 수락 시 초기화 |

Driver는 중복·역순 feedback을 무시하고 실제 수신 지연을 감시한다. Encoder rollover는 modular delta로 누적한다. Reboot·단절 뒤에는 baseline을 재설정하되 누적 각도를 유지한다. 단절 중 이동량은 관측할 수 없다.

관련 안내: [Driver](../../cleany_base_driver/docs/command-and-odometry.md), [펌웨어](../../../../esp32/README.md), [검증](../../../../docs/TESTING.md).
