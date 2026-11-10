# IMU 실시간 수신


[센서 확인 절차](../../../../docs/implementation/how-to/sensors.md#실물-imu) · [펌웨어](../../../../esp32/README.md)

MCU가 부팅할 때 정지 보정을 한 번 수행하며 실패하면 기존 bench bias·covariance를
적용합니다. PC는 `base/imu_state`의 시각을 ROS로 변환하고 받은 결과를 그대로 발행합니다.
Agent나 PC 재시작으로 보정을 반복하지 않습니다.

| 토픽 | 내용 |
|---|---|
| `imu/data_raw` | MCU의 보정 전 각속도 |
| `imu/data` | MCU에서 보정한 각속도, `RUNNING`·`FALLBACK`에서 발행 |
| `imu/status` | `DiagnosticArray` 보정·수신 상태 |

IMU 입력·출력은 best effort / volatile / keep last 1입니다.
3축 gyro는 센서 좌표계이며 orientation·linear acceleration은 covariance 첫 원소
`-1`로 unavailable을 표시합니다. Raw gyro covariance의 0은 unknown입니다.
향후 EKF 입력은 장착 TF를 확인한 Z축 각속도만 사용합니다.

### 실행

같은 v2 인터페이스로 빌드한 모터 펌웨어를 업로드하고 Agent를 실행합니다.
**MCU 전원을 켤 때 로봇과 센서를 정지시킵니다.** PC는 보정 도중이나 완료 후 연결해도 됩니다.
레포 루트의 Humble 환경에서:

```bash
make build-base
source /opt/ros/humble/setup.bash
source "$(make -s print-install-base)/setup.bash"
export ROS_DOMAIN_ID=0
ros2 launch cleany_base_driver imu_bringup.launch.py

# 다른 terminal에서 동일 overlay/domain 적용
ros2 topic echo /imu/data_raw --once --qos-reliability best_effort
ros2 topic hz /imu/data_raw
ros2 topic echo /imu/status --once --qos-durability transient_local
ros2 topic echo /imu/data --once --qos-reliability best_effort
```

PC 설정은 [`imu_bringup.yaml`](../config/imu_bringup.yaml)의 장치 MAC,
센서 좌표계과 수신 시간 제한입니다. Launch는 수신 노드만 시작하며 모터 명령·TF·Agent를
실행하지 않습니다. `imu_link`의 실제 장착 TF는 별도로 측정해 URDF에 반영합니다.

### MCU 보정 설정

[`imu_calibration.json`](../config/imu_calibration.json)은 빌드 시 검사해
펌웨어에 포함합니다. 기본값은 5초 안정화, 100 Hz 보정 1,000샘플 + 검증 500샘플,
25초 timeout입니다. 보정 중 모터를 차단하고, 완료 후에도 새 ENABLE이 필요합니다.

MCU는 `gyro_calibration.hpp`의 통계를 재사용해 두 window의 잡음·평균 잔차를
검사합니다. 정상일 때 새 평균을 bias로 사용하고 covariance는 검증·기준 분산 중
큰 값으로 대각화합니다. 실패하면 설정의 bias와 전체 covariance를 사용합니다.
기준 측정 출처는 JSON의 `source`에 보관합니다. 새 결과는 MCU RAM에만 적용하며,
재보정은 MCU 재부팅으로 수행합니다. 센서 교체 시 MAC과 기준 보정값을 갱신합니다.
PC는 보정 파일이나 계수를 재검사하지 않습니다.

### 상태 토픽

`imu/status`의 `status[0].name=imu_driver`, `message`에 아래 상태를 담습니다.

| 상태 | Level | 의미 |
|---|---|---|
| `CALIBRATING` | WARN | 시작 수신 대기 또는 정지 보정 중 |
| `RUNNING` | OK | 새 보정값으로 실측 각속도 발행 중 |
| `FALLBACK` | WARN | 기존 bench 보정값으로 실측 각속도 발행 중 |
| `ERROR` | ERROR | 센서 오류·수신 timeout |

`values`는 `bias_source` (`none/startup/fallback`), bias, 이유, 마지막 정상 샘플 age와
수신 통계를 담습니다. 상태 변경 시 즉시, 유지 시 1 Hz로 발행하며 QoS는
reliable / transient local / keep last 1입니다. `diagnostics`에도 같은 내용을 발행합니다.
센서 오류·수신 단절은 보정 상태보다 우선합니다. 오류 중에는 측정값을 만들거나
재생하지 않으며, 수신 회복 후 MCU가 선택한 상태로 돌아갑니다.

### Timestamp와 상태 검사

첫 정상 수신 시각에 MCU 시각을 맞추고 이후 취득 간격을 보존합니다. 최초 USB
지연은 남으므로 EKF 연결 전 wheel JointState와 시간 정렬을 실측합니다.
Protocol·장치·센서 오류, 중복/역순 샘플, 100 ms 초과 backlog/미래 시각과
50 ms 초과 clock jump를 거부합니다. MCU 재부팅이나 ROS clock jump 후 기준점을
다시 잡습니다. 기본 250 ms 수신 단절은 `ERROR`입니다. 이 상태의 주행 정지 연동은
융합 주행을 연결할 때 검증합니다.

### 장치 없는 검증

```bash
make test-imu-runtime
make test-motor-core
make test-base
make firmware-build
```

부팅 보정/fallback·모터 차단, 측정 시각, raw/corrected 전달과 장애·수신 회복을
검사합니다. 실물에서는 수신율·시각 오차·정지 잡음·Z축 부호·모터 주기 영향을 확인합니다.
