# cleany_base_odometry

4개 Mecanum drive wheel의 누적 회전각으로 planar wheel odometry를 계산합니다.
시뮬레이션은 Gazebo `/joint_states`를 사용하고, 실제 로봇은 MCU encoder adapter가
같은 기구학 계산을 사용합니다. 실제 ESP32 연결은 `hardware_odom_node`가
MCU snapshot을 검증하고 휠 각도, odometry와 TF를 함께 발행합니다.

## ROS 계약

- 입력: `/joint_states` (`sensor_msgs/msg/JointState`)
- 출력: `/wheel/odom` (`nav_msgs/msg/Odometry`)
- pose frame: `odom`
- child/twist frame: `base_link`
- TF: 기존 `wheel_odometry_node`는 발행하지 않음. 실물 `hardware_odom_node`는
  설정에 따라 `odom -> base_link`를 발행함.

노드는 휠 `position`의 이전 값과 현재 값의 차이를 적분합니다. `velocity` 필드는
필수 입력이 아닙니다. 휠 이름과 반경·wheelbase·wheel separation은
`config/wheel_odometry.yaml`에서 설정합니다.

Gazebo launch는 가상 encoder를 통해 tick 양자화와 선택적인 휠별 scale 및 tick
편차를 적용할 수 있습니다. `wheel_odometry_synthetic_error.yaml`은 nominal 값에서
휠 반경을 +1%, wheelbase와 wheel separation을 -1%로 둔 미보정 합성 profile입니다.
이 수치는 실측 보정값이 아니며 백래시, 추가 slip model과 통신 지연은 포함하지
않습니다. `/wheel/odom`은 평가 기준인 `/ground_truth/odom`과 구분해서 사용해야
합니다.

## 실행

```bash
ros2 launch cleany_base_odometry wheel_odometry.launch.py \
  use_sim_time:=true
```

Gazebo Cleany launch는 가상 encoder의 `/wheel_encoder/joint_states`를 입력으로
이 노드를 자동 실행하고 출력을 `/wheel/odom_raw`로 override합니다. Simulation 전용
error node가 이를 `/wheel/odom`으로 변환합니다. 독립 실행과 실제 로봇에서는 기본
`/joint_states`를 입력으로 받아 `/wheel/odom`을 직접 발행합니다. Gazebo launch의
기본 `odometry_source:=wheel`은 최종 `/wheel/odom`으로 canonical `/odom`과
`odom -> base_link` TF를 발행합니다. 해당 TF와 `/odom`의 publisher 소유권은
`cleany_gazebo_sim`에 남아 있습니다.

Gazebo에서 합성 odometry parameter 오차를 선택하려면 `wheel_odometry_config`에
`config/wheel_odometry_synthetic_error.yaml`을 지정합니다. encoder 합성 profile과
함께 사용하는 전체 명령은 `cleany_gazebo_sim/README.md`를 참고합니다.

## 검증

```bash
python3 -m pytest \
  ros2_ws/src/cleany_base_odometry/test/test_mecanum_odometry.py
```

## ESP32 원본 틱 수신

`encoder_http_node`는 ESP32 motor controller의 `GET /api/status`를 기본 최대 20 Hz로
읽어 `/wheel/encoder_ticks` (`cleany_interfaces/msg/WheelEncoderTicks`)에 발행한다.
원본 encoder 배열 계약은 `origin/main`의 `f834637`에서 시작했고, 현재 펌웨어는
`origin/feat/motor-pid-control`의 `6c0fa41`에 AP+STA와 MCU snapshot metadata를
추가한 구현이다. PI 제어, 가감속, USB protocol과 진단 필드는 함께 유지한다.
기존 API에 접근 가능한
네트워크라면 펌웨어 변경 없이 읽을 수 있다. 별도 Jetson 네트워크 어댑터가 없는
경우를 위해 [펌웨어 AP+STA 설정](../../../motor_controller/README.md)도 제공한다.

```json
{"encoders":[123,-456,789,-1011]}
```

배열 순서는 **M1 전방 왼쪽, M2 전방 오른쪽, M3 후방 오른쪽, M4 후방 왼쪽**이다.
모든 값은 MCU 부팅 이후 누적된 signed int32 quadrature tick이며 방향 보정 전
원본이다. HTTP 연결을 재사용하고 한 번에 요청 하나만 처리한다. 20 Hz는 요청
목표 상한이며 실제 수신 주기는 Wi-Fi/응답 지연에 따라 낮아질 수 있다.

원본 틱의 `header.stamp`는 Jetson ROS clock의 **응답 수신 시각**이다.
MCU 측정 시각은 별도 `sample_time_us`이며 Unix epoch가 아니다.
새 펌웨어는 `protocol_version: 1`, 부팅별 `boot_id`, uint32 `sample_seq`,
`sample_time_us`를 `encoders`와 함께 반환한다. 카운터와 MCU 시각은 같은 critical
section에서 snapshot한다. 기존 응답도 읽을 수 있지만 `has_mcu_time=false`가 되어
실물 odometry에는 사용하지 않는다. 요청 전체 소요 시간이
`request_timeout_sec`를 넘은 응답은 버린다. 소켓의 각 blocking 작업에도 같은
timeout을 적용하지만 HTTP 요청 전체의 강제 취소 deadline을 보장하지는 않는다.
잘못된 JSON/개수/정수 범위, HTTP 오류, timeout에는 발행을 생략하고 경고하며
다음 요청에서 재연결한다. 예전 값이나 가짜 0을 재발행하지 않는다.

복원된 `target`, `applied`, `target_rad_s`, `commanded_rad_s`, `omega_rad_s`,
`reverse_waiting`, `calibration_move_active`는 제어 진단용 추가 필드다.
수신기는 이 필드를 odometry 계산에 사용하지 않으며, `encoders`의 원시 부호와
PCB 순서는 바뀌지 않는다. 펌웨어 재부팅 후에는 새 boot ID에서 기준점을 다시
설정하고 기존 적분 pose를 유지한다.

### 네트워크와 실행

기존 ESP32는 `Cleany` AP와 `192.168.4.1` 상태 서버를 제공한다. Jetson에서 이
주소로 라우팅할 수 있어야 한다. Jetson의 Wi-Fi 하나를 사무실 AP에서 `Cleany`로
옮기면 기존 인터넷/SSH 경로가 끊길 수 있다. 별도 네트워크 어댑터가 없으면
ESP32의 AP+STA 지원을 추가하고 Jetson과 같은 사무실 Wi-Fi에 연결하는 구성이
필요하다. 이때 `host`에는 ESP32가 사무실 DHCP에서 받은 주소를 지정한다.

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-up-to cleany_base_odometry
source install/setup.bash
ros2 launch cleany_base_odometry encoder_http.launch.py
# 다른 주소를 사용할 때:
ros2 run cleany_base_odometry encoder_http_node --ros-args -p host:=192.168.4.1
ros2 topic echo /wheel/encoder_ticks
ros2 topic hz /wheel/encoder_ticks
```

launch와 `ros2 run`은 둘 중 하나만 실행한다. 설정 파일은
`config/encoder_http.yaml`이며 `host`, `port`, `poll_rate_hz`,
`request_timeout_sec`, `output_topic`을 바꿀 수 있다. 실제 수신 노드는
`use_sim_time=false`로 실행한다. 이 노드는 모터 명령을 보내지 않는다.

### 실물 wheel odometry

`hardware_odom_node`는 원본 snapshot을 검증하고 기존 `MecanumOdometry` core로
적분한다. MCU 재시작과 baseline 변경을 적분기에 같은 callback 안에서 전달한다.
출력은 다음과 같다.

- `/wheel_encoder/joint_states`: rad 단위 휠 각도, 순서 FL/FR/RL/RR.
- `/wheel/odom`: `nav_msgs/Odometry`, pose는 `odom`, twist는 `base_link` 기준.
- `/odom`: 같은 추정값을 canonical topic으로 발행. `canonical_topic: ''`로 비활성화.
- `odom -> base_link`: `publish_tf: true`일 때 같은 시각/pose로 발행.

`config/hardware_wheel_odometry.yaml`의 encoder 설정 근거는
`origin/feat/motor-pid-control` 커밋 `6c0fa41`이다. 해당 브랜치의
`motor_command_filter.hpp`는 3172 count/rev를 사용하고, `main.cpp`의
`encoderPolarity`는 M1/M2/M3/M4에 `[1, -1, -1, 1]`을 적용한다.
HTTP 값은 원본 PCB 순서 FL/FR/RR/RL이므로 이 부호를 한 번 적용하고 ROS joint
순서로 재배치한다. 해당 브랜치의 binary serial telemetry는 이미 부호 보정된
FL/FR/RL/RR 순서여서 이 HTTP adapter에 그대로 넣으면 안 된다.

휠 반경 0.0635 m, 전후 축 간격 0.30 m, 좌우 중심 간격 0.51 m는 기존 모델 값이다.
별도 실물 기구 치수 측정 기록과 거리/yaw 보정 결과는 확인되지 않았다.
`calibration_verified: false`가 이를 표시한다. covariance diagonal도 설정한
초기 불확실성 가정이며 실측 잔차로 추정한 통계값이 아니다.

```bash
# 기존 encoder_http_node를 종료한 뒤 통합 launch 하나를 실행한다.
ros2 launch cleany_base_odometry hardware_odometry.launch.py host:=172.16.202.248
# 새 메시지/펌웨어로 실행 중인 수신기를 재사용한다면 start_receiver:=false
ros2 topic echo /wheel/odom
ros2 run tf2_ros tf2_echo odom base_link
```

예시 IP는 2026-09-10 station DHCP 주소다. 재접속 후 달라지면 boot log에서 확인한다.
이 launch와 기존 `wheel_odometry.launch.py`, Gazebo odom relay, EKF의 odom TF
발행을 겹쳐 실행하지 않는다. 추후 EKF가 TF를 소유하면 `publish_tf: false`로 둔다.
LiDAR 장착 TF와 SLAM/localization은 이 launch에 포함하지 않는다.

### 시간과 재연결 처리

- 첫 응답의 요청/응답 중간 시각으로 MCU clock의 ROS epoch를 근사하고, 이후 시각
  간격은 MCU `sample_time_us`를 사용한다. 네트워크 왕복 시간 변화가 그대로 속도
  계산의 dt가 되지 않는다. MCU와 Jetson의 절대 시계 동기화가 완료된 것은 아니다.
- signed int32 카운터의 wrap과 uint32 sequence wrap을 처리한다. 중복/역순,
  같은 부팅에서 뒤로 간 MCU 시각, 이미 교체된 boot ID의 지연 메시지는 버린다.
- 짧은 누락은 누적 틱 차이로 이동량을 회수한다. MCU 재시작, 0.5초 초과 sample gap,
  0.25초 초과 clock 불일치, 설정한 50 rad/s 초과 tick jump는 baseline을 다시 잡는다.
  그 구간의 이동량을 적분하지 않으며 마지막 pose를 유지한다. 미관측 구간의 실제
  이동을 복원한다는 의미가 아니다.
- baseline을 다시 잡은 첫 sample에는 odom/TF를 내보내지 않는다. 입력이 끊기면
  odom/TF 발행도 멈추고 경고한다. 마지막 twist를 현재 속도로 재발행하지 않는다.
- ROS 메시지가 0.5초 이상 오래되었거나 미래 시각이면 버린다. 위 임계값과 encoder
  부호/분해능, 기구 치수, covariance는 모두 ROS parameter로 바꿀 수 있다.

```bash
PYTHONPATH=ros2_ws/src/cleany_base_odometry python3 -m pytest \
  ros2_ws/src/cleany_base_odometry/test
```

HTTP 테스트는 loopback의 모의 상태 서버로 정상 수신, signed count 보존,
잘못된 응답, timeout, 재연결을 검사한다. 실물 ESP32의 tick 변화 검증과 구분한다.
`test_encoder_odometry.py`는 순서/부호, 회전량, wrap, packet 누락, MCU 재시작,
clock 변경, 비정상 jump를 검사한다. `test_hardware_odom_ros.py`는 합성 틱으로
odom/TF까지의 메시지 연결과 재시작 시 pose 유지, 입력 중단 시 발행 정지를 검사한다.
ROS runtime 테스트는 `ROS_DOMAIN_ID=73` 같은 별도 domain에서 실행한다.

### 2026-09-10 원본 틱 수신 확인

- Jetson ROS 2 Humble에서 관련 두 패키지 빌드, odometry/HTTP/ROS integration
  테스트 23개와 기존 interface 테스트 5개 통과.
- ESP32-S3 AP+STA 펌웨어 빌드와 USB 업로드 후 사무실 Wi-Fi를 통한 HTTP 수신 확인.
- 사용자가 바퀴를 수동으로 돌릴 때 M2 원본 틱 `0 → 81 → -580` 변화 확인.
  다른 세 바퀴의 변화, 실제 바퀴 위치 대응, encoder 부호/분해능은 미검증.
- 후속 15초 수집에서 214개 ROS 메시지 수신. DDS 연결 이후 메시지 간격 기준
  17.47 Hz, 최대 간격 87.4 ms, HTTP 왕복 중앙값 55.5 ms. 당시 요청 설정은 20 Hz.
- 원본 틱 수신까지 확인했으며 wheel odometry, TF, SLAM 연결은 수행하지 않음.

### 2026-09-10 후속 odometry 확인

- MCU snapshot metadata 추가 펌웨어 빌드/업로드 성공. Jetson에서 두 패키지 빌드와
  순수 core, HTTP, ROS 메시지/TF, interface 테스트 총 50개 통과.
- 실물 수동 회전에서 M2 `0 → -558`이 front-right joint의 약 `+1.1053 rad`로
  변환됨. 60초 기록에서 odom, canonical odom, TF가 각 1,017개 수신됐고,
  공통 timestamp 1,017개에서 x/y/quaternion 값이 모두 일치함. 약 17.28 Hz.
- MCU 재시작/오류 처리 검사는 합성 입력 테스트로 확인함. 실물 주행 거리/yaw,
  모든 바퀴의 부호 재측정, 기구 치수 보정, LiDAR 장착 TF와 SLAM은 미검증.
