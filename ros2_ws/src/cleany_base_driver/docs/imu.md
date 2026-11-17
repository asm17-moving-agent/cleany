# IMU 실시간 수신

MCU는 MPU6050 각속도와 가속도를 `sensor_msgs/msg/Imu`로 발행한다. Jetson의
`cleany_imu`가 기존 gyro bias와 공분산을 적용한다. EKF는 보정된 각속도와
wheel odometry의 평면 속도를 융합한다.

```text
MCU → imu_uncalibrated/data_raw → cleany_imu → imu/data_raw → EKF
                                                         wheel/odom ↗
```

## 데이터와 좌표계

| 토픽 | 발행자 | 내용과 QoS |
|---|---|---|
| `imu_uncalibrated/data_raw` | MCU | 센서 축 gyro(rad/s), 중력 포함 가속도(m/s²); B1 |
| `imu/data_raw` | `cleany_imu` | 고정 bias를 뺀 gyro, 원본 가속도; B1 |
| `imu/status` | `cleany_imu` | 입력과 고정 bias 적용 상태; reliable, transient local, depth 1 |

B1은 best effort, volatile, keep last 1이다. 모든 IMU 데이터는 동일한
`imu_link` 센서 frame과 취득 stamp를 유지한다. 미보정 입력과 `imu/data_raw`의
`orientation_covariance[0]=-1`은 자세 미제공을 뜻한다. 미보정 입력 covariance의
0은 unknown이며, `imu/data_raw` gyro에는 설정한 전체 3×3 covariance를 적용한다.
가속도는 수신값을 그대로 전달한다.

[REP-145](https://github.com/ros-infrastructure/rep/blob/master/rep-0145.rst)의
토픽 관례에 따라 `imu/data_raw`에는 gyro와 가속도를 담는다. Bias 미보정 입력은
`imu_uncalibrated` namespace에 두고, EKF가 `imu/data_raw`의 Z축 각속도를 구독한다.
`imu_link` 장착 TF는 실제 장착에 맞는 URDF 또는 명시적인 static TF로 제공한다.
IMU bringup은 장착 회전을 추정하거나 TF를 만들지 않는다.

## 실행

1. [펌웨어 절차](../../../../esp32/README.md)에 따라 변경한 모터 이미지를 빌드,
   업로드하고 같은 DDS domain에서 Agent를 실행한다.
2. Jetson의 Humble 환경에 `make deps`로 의존성을 설치한다.
3. Host 패키지를 빌드하고 실행한다. 브랜치 전환 전 생성물과 분리한 경로를 사용한다.

```bash
source /opt/ros/humble/setup.bash
export CLEANY_BUILD_ROOT="$(dirname "$PWD")/cleany-data/builds/cleany-imu-factory"
make build-base
source "$(make -s print-install-base)/setup.bash"
export ROS_DOMAIN_ID=0
ros2 launch cleany_base_driver imu_bringup.launch.py
```

4. 동일 overlay와 domain의 별도 터미널에서 각 단계를 확인한다.

```bash
ros2 topic echo /imu_uncalibrated/data_raw --once --qos-reliability best_effort
ros2 topic hz /imu_uncalibrated/data_raw
ros2 topic echo /imu/data_raw --once --qos-reliability best_effort
ros2 topic echo /imu/status --once --qos-durability transient_local
```

Launch는 host 보정 노드를 시작한다. Agent와 모터 driver는 별도로 실행한다.
실물 IMU와 Gazebo IMU를 같은 namespace에서 동시에 발행하지 않는다.

## 고정 factory 설정

[`imu_bringup.yaml`](../config/imu_bringup.yaml)이 보정 계수와 입력 검사 설정의 단일 원본이다.
기존 `bench_20261014T142933Z/run06_persistent`의 gyro bias와 covariance를
고정값으로 사용한다. 적용 식은 센서 축에서 `gyro_calibrated = gyro_raw - bias`다.
원본 bench 결과의 `accepted=false`는 자동 판정 한도 미검토 기록이며, 기존 계수
재사용과 별개로 보존한다. 이 계수 사용을 새 실물 정확도 검증으로 해석하지 않는다.

| Host parameter | 단위 또는 조건 |
|---|---|
| `gyro_bias_rad_s` | X/Y/Z의 유한한 3개 값; 필수 |
| `gyro_covariance_rad2_s2` | 행 우선 9개 값, 대칭 양의 정부호; 필수 |
| `calibration_source` | 측정 출처 |
| `frame_id` | 입력과 일치하는 상대 frame명; 기본 `imu_link` |
| `max_transport_delay_s` | 과거 지연과 미래 stamp 허용 한도; 기본 0.10 s |
| `max_clock_jump_s` | ROS/steady 경과 차이 한도; 기본 0.05 s |
| `stale_timeout_s` | 정상 입력 단절 한도; 기본 0.25 s |

Host parameter는 실행 중 read-only다. 계수 변경은 YAML 수정 후 bringup 재시작으로
적용하며 MCU 재빌드는 필요 없다. Agent, host 또는 MCU를 재시작해도 같은 계수를
사용한다. 새 측정 절차는 [캘리브레이션 가이드](../../../../docs/CALIBRATION.md#imu-factory-gyro-bias)를 따른다.

표준 Imu에는 MAC과 부팅 ID가 없다. 여러 장치는 namespace/remap으로 분리하고
각 센서에 맞는 계수를 선택한다. 센서 교체 후 이전 계수를 그대로 사용하지 않는다.

## 시각과 오류

MCU는 Agent session 시각 동기화 후 `Agent epoch - 취득 이후 경과`를
`header.stamp`에 기록한다. 동기화 실패 시 1초 간격으로 재시도하고 그동안 IMU를
발행하지 않는다. 재연결마다 다시 동기화한다. Host는 수신 시각으로 restamp하지 않는다.

| 검사 또는 상태 | 처리 |
|---|---|
| MCU 센서 오류, 포화, 비유한 값, 100 ms 초과 sample age | 해당 sample 미발행 |
| Host frame 불일치, 비유한 값, gyro/가속도 unavailable | `imu/data_raw` 미발행, `ERROR` |
| 중복/역순, backlog, 미래 stamp 한도 초과 | `imu/data_raw` 미발행, `ERROR` |
| 첫 정상 입력 전 | `WAITING` (WARN), timeout 후 `ERROR` |
| 정상 입력 | `RUNNING` (OK), `bias_source=factory` |
| 정상 입력 단절 | `ERROR`; 입력 회복 시 같은 고정 계수로 재개 |
| ROS clock jump | `ERROR` 유지; host 보정 노드를 재시작해 입력 시간 기준 초기화 |

상태는 변경 즉시, 유지 시 1 Hz로 발행하고 `diagnostics`에도 복제한다.
`imu/status`는 host 입력과 bias 적용 상태를 나타낸다. MCU 센서 오류의 세부 원인은
firmware log에서 확인한다.

IMU 상태와 주행 정지의 연동, 장착 축 및 wheel 시간 정렬은 실물에서 확인한다.

## 검증

```bash
make test-imu-runtime
make test-motor-core
make test-base
make firmware-build
```

합성 ROS 검사는 host 보정 노드를 실행해 고정 bias와 공분산, acceleration 전달,
취득 시각, 오류와 수신 회복을 검사한다. 실물에서는 수신율, 시각 오차,
정지 잡음, Z축 부호와 USB/I2C 부하의 모터 주기 영향을 확인한다.
