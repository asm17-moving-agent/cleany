# robot configs

로봇, 센서, frame, workspace, safety limit 관련 설정을 둔다.

## LiDAR 평면 extrinsic

- `lidar_extrinsic_20261015.json`: 2026-10-15 정지 스캔 158개와 CAD의 네 프로파일
  단면으로 추정한 `laser → base_link`의 yaw와 x/y lever arm이다.
  실물 전방≈laser −Y, 좌측≈laser +X를 대응 기준으로 사용했다.
  Z와 roll/pitch를 실측한 6DoF 보정값이 아니며, 정합 잔차를 절대 정확도로
  해석하지 않는다. 센서나 프로파일 장착이 바뀌면 재측정한다.
- 편도 평가의 `--extrinsic-config`로 명시적으로 선택한다.
  실행 명령과 측정용 근거리 제거 설정은
  [오도메트리 평가 README](../../tools/odometry_evaluation/README.md#속도별-3회-휠-odom-재측정)를 따른다.

## 휠 평가 런타임

`wheel_evaluation_runtime.json`은 config 기반 실험 launcher의 실물 노드 기동
설정이다. MCU와 LiDAR의 persistent `/dev/serial/by-id/` 경로, baudrate, hardware
base profile, LiDAR scan 옵션과 시작·종료 대기 시간을 포함한다.

- 정상 동작 중인 stack은 재사용하고, 없는 Agent·driver·LiDAR만 기동한다.
- `restart_stale`이 켜져 있으면 설정된 USB 경로와 `restart_node_names`에 일치하는
  같은 사용자 LiDAR 프로세스만 정체 시 재시작한다. USB 재연결로 `/dev/ttyUSB*`
  번호가 바뀌어도 by-id 경로를 사용한다.
- 준비 확인은 `/wheel/odom`, `/scan`, `/base/wheel_state`의 실제 수신과
  `/base/enable` service를 확인한다. 노드나 topic 이름만 있다고 통과하지 않는다.

## 메카넘 base

- `base_hardware.yaml`: 사용자 확인 geometry는 휠 직경 127 mm, 앞뒤 휠 중심 간
  350 mm, 좌우 휠 중심 간 610 mm다. 현재 기구학 설정은 지면 회전 보정값인
  유효 `wheel_separation_m: 0.590`을 사용하며 실측 중심 간격과 구분한다.
  바퀴를 띄운 초기 시험용으로 x/y 입력 상한
  0.5 m/s, yaw 1 rad/s, 휠 10 rad/s와 `cmd_vel` timeout 0.3초를 사용한다.
  입력 축별 상한과 휠 상한은 각각 지정된 값이다.
  휠 비율 scaling으로 복합 명령에서도 각 휠 상한을 지킨다.
  바닥 주행에는 하중과 정지 성능을 검증한 별도 제한값을 사용한다.
- `base_synthetic.yaml`: 장치 없는 mock 테스트용 합성 profile. `mock:=true`에서만
  사용하며 실물 설정의 기본값으로 복사하지 않는다.

두 profile 원본은 `cleany_base_driver` package share에 설치된다. 같은 geometry를
driver 역기구학과 wheel odometry에 전달한다. 필드, launch와 검증 방법은
[`cleany_base_driver` README](../../ros2_ws/src/cleany_base_driver/README.md)를 따른다.

## IMU

- `imu_calibration.json`: MCU 부팅 보정의 안정화 시간, 샘플 수, timeout,
  잡음 기준과 기존 측정 bias·covariance다. `make firmware-build`에서 검사하고
  펌웨어에 포함한다. 변경 후 다시 빌드·업로드한다.
- `imu_bringup.yaml`: PC 수신 node의 장치 MAC, 센서 frame과 수신 timeout이다.
  PC는 MCU의 보정 결과를 그대로 발행하며 보정값을 재검사하거나 다시 적용하지 않는다.

부팅 보정과 모터 구동 차단은 [펌웨어 README](../../esp32/README.md#power-on-calibration),
ROS 토픽과 실행은
[base driver README](../../ros2_ws/src/cleany_base_driver/README.md#imu-실시간-수신)를 따른다.
