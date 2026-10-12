# Base 명령과 odometry

- 계약: [Twist](../../cleany_interfaces/docs/mobile_base.md), [MCU v2](../../cleany_base_interfaces/docs/protocol.md)

## 명령

- 여섯 축 중 nonfinite 포함: STOP
- 지원 밖 축: 경고 후 무시
- Wheel limit 초과: 네 바퀴를 같은 비율로 축소
- Enable true: 기존 cmd_vel 삭제 → zero ENABLE → fresh enabled 확인 → 새 목표만 전송
- ENABLE 손실: 같은 sequence·deadline으로 재시도
- 새 목표 없음: zero VELOCITY
- STOP·단절·만료·reboot: 새 ENABLE + 새 cmd_vel 필요. 재연결만으로 재개 금지

## 설정과 시간

- 원본: `configs/robot/base_hardware.yaml`, `base_synthetic.yaml`
- 실물 geometry: radius 0.0635 m / wheelbase 0.350 m / separation 0.610 m
- 초기 제한: 바퀴를 띄운 시험용. 바닥 주행 승인 아님
- Mock: 합성 geometry. 실물 보정값으로 사용 금지
- Driver deadline·cmd_vel timeout: 설정값 사용
- MCU timestamp: monotonic. ROS epoch 아님
- JointState stamp: ROS 수신 시각. 단절 중 이동량 미관측

## Odometry와 TF

- Wheel odometry: joint position 차이 적분 → `/wheel/odom`; TF 미발행
- 실물 driver: reboot/단절 baseline·rollover 처리 → 연속 position 제공
- Joint velocity: 필수 입력 아님
- `velocity_window_s`: 0=샘플 차분, 양수=구간 평균 twist
- 시작·시간 역행·긴 공백: 구간 준비 전 미발행; 미관측 속도를 0으로 보고 금지
- 구간 평균: 지연 존재. 제동·도착 별도 검증
- `yaw_scale`: 유한한 양수의 회전 보정; geometry 변경 없음
- Sim 보정값: 독립 검증 없이 실물 적용 금지
- Covariance: config/구현 기준. Slip·바닥 오차 보정 없음

| TF / 출력 | 소유자 |
|---|---|
| 실물·mock `/odom`, odom→base_link | base_odom_relay |
| Gazebo wheel_odom_raw→error adapter→wheel_odom 및 canonical odom/TF | simulator |
| map→odom | AMCL |
| robot fixed/joint TF | robot_state_publisher |

- 외부 estimator 사용: `relay_odom:=false`, 중복 publisher 제거
- 안내: [ESP32](../../../../esp32/README.md), [실행](../../../../docs/RUNNING.md), [검증](../../../../docs/TESTING.md)
