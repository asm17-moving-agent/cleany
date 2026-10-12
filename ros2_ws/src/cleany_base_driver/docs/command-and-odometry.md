# Base 명령과 odometry

공통 Twist는 [mobile_base](../../cleany_interfaces/docs/mobile_base.md), MCU wire는
[protocol v2](../../cleany_base_interfaces/docs/protocol.md)를 따른다.

`cmd_vel`의 여섯 성분 중 하나라도 nonfinite이면 STOP한다. 지원하지 않는 축은 경고하고 무시한다.
Mecanum 변환 후 wheel limit 초과는 네 바퀴를 같은 비율로 축소해 방향을 보존한다.
`/base/enable` true는 이전 cmd_vel을 지우고 zero ENABLE을 보낸다.
손실 시 같은 sequence·deadline으로 재시도하며 fresh MCU enabled 확인 후 요청 이후 새 목표만 보낸다.
목표가 없으면 zero VELOCITY다. STOP·단절·만료·reboot 뒤 재연결만으로 구동을 복구하지 않는다.

## 설정과 시간

원본은 저장소 `configs/robot/base_hardware.yaml`, `base_synthetic.yaml`이다.
실물 초기 geometry는 radius 0.0635 m, wheelbase 0.350 m, 좌우 간격 0.610 m다.
초기 속도 제한은 바퀴를 띄운 시험용이며 바닥 주행 승인이나 실측 보정값이 아니다.
Mock geometry는 합성값이다. Driver의 MCU deadline·cmd_vel timeout은 설정에서 확인한다.
MCU monotonic timestamp를 ROS epoch로 변환하지 않는다. JointState stamp는 ROS 수신 시각이다.
피드백 단절 중 이동량은 알 수 없다.

## Wheel odometry와 TF 소유권

cleany_base_odometry는 wheel joint position의 차이를 적분해 `/wheel/odom`을 발행하고 TF를 발행하지 않는다.
실물 driver가 reboot·feedback 단절 후 encoder baseline과 rollover를 처리해 연속 position을 제공한다.
velocity 필드는 필수 입력이 아니다.
`velocity_window_s`는 twist 계산의 최소 관측 구간이다. 0은 샘플 간 차분, 양수는 구간 평균이다.
시작·시간 역행·구간보다 긴 공백 이후 구간이 준비되기 전까지 발행하지 않는다.
관측하지 못한 속도를 측정된 0으로 보고하지 않는다. 평균 속도에는 지연이 있으므로 제동·도착 검증이 필요하다.
`yaw_scale`은 유한한 양수의 회전 보정 계수이며 geometry를 바꾸지 않는다.
시뮬레이션 보정값을 독립 검증 없이 실물에 적용하지 않는다. Covariance는 해당 config와 구현을 따른다. Wheel odometry는 slip과 바닥 오차를 보정하지 않는다.
실물·mock driver의 base_odom_relay가 canonical `/odom` 및 odom→base_link를 소유한다.
외부 estimator를 연결할 때 `relay_odom:=false`로 중복 TF publisher를 막는다.
Gazebo는 wheel_odom_raw → simulation error adapter → wheel_odom 경로와 canonical odom/TF를 자체 소유한다.
AMCL은 map→odom, robot_state_publisher는 robot fixed/joint TF를 소유한다.

실물 연결·upload는 [ESP32](../../../../esp32/README.md),
장치 없는 실행은 [RUNNING](../../../../docs/RUNNING.md), 검증은 [TESTING](../../../../docs/TESTING.md).
