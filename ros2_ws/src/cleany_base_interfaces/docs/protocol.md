# Wheel protocol v2

메시지 원본은 이 패키지의 `msg/`다. 펌웨어 type support와 ROS driver를 같은 계약으로 빌드한다.
한 MCU의 command publisher는 driver 하나가 소유한다.

| 토픽 | 방향 | QoS와 주기 |
|---|---|---|
| `base/wheel_command` | ROS → MCU | best effort, volatile, keep last 1, 50 Hz |
| `base/wheel_state` | MCU → ROS | best effort, volatile, keep last 1, 50 Hz |

배열은 FL, FR, RL, RR다. MCU에서만 PCB M1, M2, M4, M3 순서와 극성을 적용한다.
각도와 속도는 출력축 rad와 rad/s, encoder는 signed int32 누적 count, PWM은 signed percent다.
출력축 1회전은 3172 counts다.

## 명령 수락과 만료

- 버전은 2다. STOP=0은 목표 삭제·출력 0·구동 허가 해제, ENABLE=1은 목표 0으로 허가, VELOCITY=2는 허가된 상태의 목표 갱신이다.
- uint32 sequence는 `0 < (new-old) mod 2^32 < 2^31`일 때만 새 명령이다. MCU는 정지·단절 뒤에도 마지막 번호를 유지한다. Driver는 fresh feedback의 `last_command_sequence`에서 다음 번호를 정한다.
- `valid_until_us`는 MCU monotonic microseconds다. ROS epoch가 아니다. Driver는 MCU timestamp와 로컬 monotonic 경과로 계산한다.
- ENABLE/VELOCITY는 `now < deadline <= now+250000`이어야 한다. ENABLE은 네 목표 모두 0, VELOCITY는 유한한 [-10,10] rad/s다.
- 이미 허가된 상태의 ENABLE과 중복 명령은 목표·기한을 갱신하지 않는다. 거부한 명령도 기한을 연장하지 않는다.
- STOP은 버전·목표·기한과 관계없이 정지한다. 순번은 새 번호일 때만 반영해 오래된 ENABLE의 재허가를 막는다.

MCU는 매 control tick의 명령 처리 **전에** 기한을 검사한다. 만료하면 목표와 허가를 지운다.
`watchdog_ms=250`은 최대 명령 유효기간이며 별도 watchdog 시각이 아니다.
부팅·STOP·만료·단절 뒤에는 새 ENABLE과 새 cmd_vel이 필요하다. 연결 복구만으로 재개하지 않는다.
STOP latch와 단절은 대기 명령을 버린다. PWM/PI는 별도 motor-control task가 계산한다.

## 피드백 해석

`boot_id`는 부팅마다 nonzero uint32이며 reboot/baseline 감지용이다. `timestamp_us`도 MCU monotonic이다.
JointState stamp는 ROS 수신 시각이다. Driver는 중복·역순 feedback을 버리고 실제 수신 지연을 감시한다.
`enabled`는 소프트웨어 허가다. `command_age_ms=65535`는 활성 유효 명령 없음,
fault bit 0은 명령 만료이며 새 ENABLE 수락 시 초기화한다.
Encoder rollover는 modular delta로 누적한다. Reboot·feedback 단절 후 baseline을 재설정하되
누적 각도를 유지한다. 단절 중 이동량은 관측하지 못한다.

[Driver와 odometry](../../cleany_base_driver/docs/command-and-odometry.md),
[펌웨어](../../../../esp32/README.md), [검증 선택](../../../../docs/TESTING.md).
