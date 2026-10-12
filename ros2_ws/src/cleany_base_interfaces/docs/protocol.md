# Wheel protocol v2

- 메시지 원본: 패키지 msg/. ROS driver·firmware type support의 계약 일치 필수
- MCU command publisher: driver 하나

| 토픽 | 방향 | QoS / 주기 |
|---|---|---|
| base/wheel_command | ROS → MCU | best effort, volatile, keep last 1 / 50 Hz |
| base/wheel_state | MCU → ROS | best effort, volatile, keep last 1 / 50 Hz |

- 배열: FL, FR, RL, RR. MCU에서만 PCB M1, M2, M4, M3 매핑·극성 적용
- 단위: 출력축 rad·rad/s, signed int32 encoder count, signed percent PWM
- Encoder: 출력축 1회전=3172 counts

## 명령 수락

| Mode | 값 | 동작 |
|---|---:|---|
| STOP | 0 | 목표 삭제·출력 0·허가 해제 |
| ENABLE | 1 | 목표 0으로 허가 |
| VELOCITY | 2 | 허가된 상태의 목표 갱신 |

- Version: 2
- 새 sequence: `0 < (new-old) mod 2^32 < 2^31`
- MCU: 정지·단절 후 마지막 sequence 유지. Driver: fresh last_command_sequence 다음 번호 사용
- valid_until_us: MCU monotonic µs. Driver는 MCU stamp+로컬 monotonic 경과로 계산
- ENABLE/VELOCITY 기한: `now < deadline <= now+250000`
- ENABLE: 네 목표 모두 0. VELOCITY: 유한한 [-10,10] rad/s
- 중복·거부·이미 enabled인 ENABLE: 목표·기한 갱신 없음
- STOP: 버전·목표·기한 무관 정지. 새 sequence만 반영해 오래된 ENABLE 차단

## 만료·재개

- 매 tick **명령 처리 전** 기한 검사. 만료 시 목표·허가 삭제
- watchdog_ms=250: 최대 유효기간. 별도 watchdog 시각 아님
- 부팅·STOP·만료·단절 후: 새 ENABLE + 새 cmd_vel 필요. 재연결만으로 재개 금지
- STOP latch·단절: 대기 명령 삭제. PWM/PI: 별도 motor task

## 피드백

- boot_id: 부팅별 nonzero uint32; reboot/baseline 감지용
- timestamp_us: MCU monotonic. JointState stamp: ROS 수신 시각
- 중복·역순 feedback 무시; 실제 수신 지연 감시
- enabled: 소프트웨어 허가
- command_age_ms=65535: 활성 유효 명령 없음
- Fault bit 0: 명령 만료; 새 ENABLE 수락 시 초기화
- Encoder: rollover modular delta. Reboot/단절 후 baseline 재설정·누적 각도 유지
- 단절 중 이동량: 미관측
- 안내: [Driver](../../cleany_base_driver/docs/command-and-odometry.md), [펌웨어](../../../../esp32/README.md), [검증](../../../../docs/TESTING.md)
