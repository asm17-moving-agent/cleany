# Holonomic Odometry Evaluation (실물 바닥 오도메트리 검증)

메카넘 4륜 홀로노믹 베이스의 실물 바닥 주행 시 누적 휠 오도메트리 오차를 정량 계측·검증하는 도구입니다.

- **속도 기준**: 설정 상한 최고 속도 ($v_x = 0.5\text{ m/s}, v_y = 0.5\text{ m/s}, \omega_z = 1.0\text{ rad/s}$)
- **작업 공간 한계**: 시작 원점 기준 최대 변위 $\le 3.0\text{ m}$
- **기준점 방식**: 바닥 마킹 테이프(십자선 및 정렬 기준선)를 이용한 폐루프 복귀 잔류 오차 계측
- **실행 환경**: `distrobox` (`ros2-humble`) 자동 전환 지원

---

## 1. 사전 준비 및 바닥 마킹

1. **테스트 영역 확보**: 최소 $3.5\text{ m} \times 3.5\text{ m}$ 이상의 평탄하고 장애물이 없는 바닥 구역을 확보합니다.
2. **원점 마킹 (출발점)**:
   - 바닥에 마스킹 테이프로 십자(+) 선을 부착합니다.
   - 로봇의 외곽 기준점(예: 범퍼 중앙 또는 휠 접지점 모서리)이 십자선 교차점과 정렬되도록 표시합니다.
3. **편도 기준 마킹 (선택)**:
   - 직진 2.5m 지점 및 횡이동 2.0m 지점에 보조 테이프를 부착하면 편도 도달 시의 스케일 오차도 함께 관측할 수 있습니다.
4. **안전 장치**: 물리적 Emergency Stop 스위치를 즉시 조작할 수 있도록 준비합니다.

---

## 2. 노드 기동

실물 로봇 및 Micro-ROS Agent가 기동되어 있어야 합니다.

```bash
# 터미널 1: Micro-ROS Agent (DDS domain 0)
source /opt/ros/humble/setup.bash
source esp32/micro_ros/agent/install/local_setup.bash
export ROS_DOMAIN_ID=0
ros2 run micro_ros_agent micro_ros_agent serial --dev "$CLEANY_ESP_PORT" -b 115200

# 터미널 2: Base Driver & Odometry Node
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch cleany_base_driver base_driver.launch.py \
  mock:=false config:="$(pwd)/configs/robot/base_hardware.yaml"
```

---

## 3. 평가 스크립트 실행

호스트 또는 컨테이너 내부에서 [run_eval.sh](run_eval.sh)를 실행할 수 있습니다.
Ubuntu 22.04가 아닌 호스트에서는 `ros2-humble` Distrobox로 진입합니다.

### 3.1 드라이 런 (궤적 검증)
```bash
# X축 직진 왕복 (기본 2.5m)
bash tools/odometry_evaluation/run_eval.sh --pattern surge --dry-run

# 횡이동 왕복 (기본 2.0m)
bash tools/odometry_evaluation/run_eval.sh --pattern sway --dry-run

# 사각 폐루프 (기본 2.0m x 2.0m)
bash tools/odometry_evaluation/run_eval.sh --pattern box --dry-run
```

### 3.2 실물 검증 주행

#### 패턴 1: X축 전후진 왕복 (Surge)
```bash
bash tools/odometry_evaluation/run_eval.sh --pattern surge --distance 2.5
```
- 동작: $+X$ 방향으로 2.5m 전진 ($0.5\text{ m/s}$) $\to$ 2초 정지 $\to$ $-X$ 방향으로 2.5m 후진하여 원점 복귀.
- 검증: 바퀴 반경($r$) 오차 및 전후 가감속 슬립.

#### 패턴 2: Y축 횡이동 왕복 (Sway)
```bash
bash tools/odometry_evaluation/run_eval.sh --pattern sway --distance 2.0
```
- 동작: $+Y$ 방향(좌측)으로 2.0m 이동 ($0.5\text{ m/s}$) $\to$ 2초 정지 $\to$ $-Y$ 방향(우측)으로 2.0m 이동하여 원점 복귀.
- 검증: 메카넘 45° 롤러의 고유 횡슬립 계측.

#### 패턴 3: 제자리 순수 회전 (Yaw)
```bash
bash tools/odometry_evaluation/run_eval.sh --pattern yaw --turns 2.0
```
- 동작: 제자리 좌회전 2바퀴($720^\circ$, $1.0\text{ rad/s}$) $\to$ 2초 정지 $\to$ 우회전 2바퀴 복귀.
- 검증: 휠베이스·트랙폭 기구학 계수 $\frac{L+W}{2}$ 및 회전 마찰 슬립.

#### 패턴 4: 사각 폐루프 주행 (Holonomic Box)
```bash
bash tools/odometry_evaluation/run_eval.sh --pattern box --distance 2.0
```
- 동작: 회전 없이 전진 2m $\to$ 좌측 2m $\to$ 후진 2m $\to$ 우측 2m 이동 (대각 최대 반경 $2.828\text{ m} \le 3.0\text{ m}$).
- 검증: 4개 휠의 복합 주행 후 최종 2D 위치 드리프트 누적치 계측.

#### 단방향 원위치 복귀 (Return X / Return Y)
- `--pattern return_x --distance <m>`: 직진 후 원점으로 복귀
- `--pattern return_y --distance <m>`: 횡이동 후 원점으로 복귀

---

## 4. Ground-Truth 계측 방식 (LiDAR ICP vs 바닥 테이프 수동 측정)

### 4.1 2D LiDAR ICP 자동 계측 (기본 활성화)
RPLIDAR A1 또는 호환 2D LiDAR이 `/scan`을 발행하면 출발 전 기준 스캔과
복귀 후 스캔을 Point-to-Line ICP로 정합하여 상대 변위를 추정합니다.
RMSE, 인라이어 비율과 관측성 판정을 통과한 결과만 유효한 계측으로 사용합니다.

- **원리**: 실내 벽면 및 고정 구조물을 기준으로 360° 포인트 클라우드를 다중 버스트(1초) 수집 후 최적 강체 변환 $(\Delta x, \Delta y, \Delta \theta)$를 계산.
- **신뢰성 지표**: RMSE (잔차 표준편차), 인라이어 비율(Inlier Ratio), 헤시안 조건수(Condition Number)를 자동 계산하여 신뢰도 평가.
- **결과 플롯**: 평가 JSON과 같은 이름의 `.png`에 스캔 오버레이 및 ICP 결과 저장.
- **비활성화 옵션**: LiDAR가 없거나 수동 마킹 테이프 입력이 필요한 경우 `--no-lidar` 플래그를 전달합니다.

#### ICP 수렴과 관측성

`lidar_scan_matcher.py`는 `p_ref = R(yaw) p_target + t`와 일치하는 additive
pose Jacobian을 사용합니다. 최종 30 mm inlier에서 같은 robust Jacobian으로
병진·전체 SE(2) 관측성을 다시 계산합니다.

수렴은 correspondence 거리의 annealing이 40 mm 최종 단계에 도달한 뒤만
판정합니다. 병진 `1e-5 m`와 회전 `1e-5 rad`의 strict step 조건을 각각 확인하며,
작은 반복 진동은 다음 조건을 모두 만족할 때 별도로 종료합니다.

- 주기 2~6의 pose가 세 주기 연속 반복.
- 대응하는 모든 pose 쌍이 strict 병진·회전 공차 안에서 일치.
- 전체 반복 구간의 pose 간 최대 차이가 1 mm·0.02° 이하.

반환값은 마지막 pose이며, 평균 pose나 odometry 값으로 대체하지 않습니다.
수렴 후에도 inlier·RMSE·관측성 검사를 통과해야 유효합니다.
`ScanMatchResult`는 `convergence_reason`(`step_tolerance`, `bounded_cycle` 등),
마지막 보정량과 적용한 `convergence_settings`를 기록합니다.
`LidarScanMatcher(convergence_config=...)`로 병진·회전 strict 공차,
최대 반복 진동 폭과 주기를 각각 설정합니다.

#### 저장 스캔 재검증

`replay_scan_matches.py`는 저장된 `response_summary.json`과 NPZ만 읽어
outbound·inbound·직접 loop 정합을 다시 계산합니다. ROS나 장치를 사용하지
않습니다. 저장된 extrinsic과 odometry로 laser-axis 초기값을 재구성하며,
loop 초기값은 두 편도의 SE(2) 합성으로 계산합니다.

```bash
# 레포 루트, Ubuntu 22.04 외 호스트의 Distrobox 환경
distrobox enter --name ros2-humble --no-tty --clean-path -- \
  env PYTHONNOUSERSITE=1 /usr/bin/python3 \
  tools/odometry_evaluation/replay_scan_matches.py \
  artifacts/odometry_evaluation/all_axis_validation/*/response_summary.json \
  --min-inlier-ratio .30 \
  --output artifacts/odometry_evaluation/icp_replay_new.json
```

`--min-inlier-ratio`를 생략하면 각 원본 결과의 적용값(없으면 40%)을 사용합니다.
새 JSON에는 기존·새 품질 판정, 수렴 사유, 적용 설정, pose 변화량과 회귀 건수를
기록합니다. 원본을 수정하지 않으며 기존 출력 파일도 덮어쓰지 않습니다.
Pose 변화량은 알고리즘 간 차이이며 실제 위치 오차가 아닙니다.

2026-10-21 저장 정합 87건을 inlier 하한 30%로 재검증한 결과, 기존 유효 79건을
모두 유지하고 기존 무효 8건 중 7건이 유효해졌습니다. 이 중 4건은 미수렴
수정, 3건은 40%→30% 하한 변경에 해당합니다. Inlier 25.6%인 1건은 계속
거부했습니다. 기존 유효 정합의 최대 pose 변화는 병진 2.45 mm·yaw 0.029° 미만입니다.
원본과 상세 비교는 Git 제외 경로
`artifacts/odometry_evaluation/icp_regression/20261021_bounded_cycle/`에 있습니다.

### 4.2 바닥 마킹 테이프 수동 입력 (Fallback 또는 `--no-lidar`)
LiDAR를 사용하지 않거나 ICP가 거부되면 명시적인 수동 측정이 필요합니다.
`--manual-fallback`을 지정하면 바닥 테이프 기준의 측정치를 프롬프트로 입력받습니다.
빈 입력을 0으로 간주하지 않습니다.

```text
Enter physical X error relative to tape mark in mm (forward/overshoot + / undershoot -):
Enter physical Y error relative to tape mark in mm (left + / right -):
Enter physical Yaw error relative to tape mark in deg (CCW + / CW -):
```

입력 또는 LiDAR 추정 완료 시 요약 분석표가 출력되며, `artifacts/odometry_evaluation/odom_eval_<pattern>_<timestamp>.json`에 상세 기록이 저장됩니다.

`--tape-error-x`, `--tape-error-y`, `--tape-error-yaw`를 모두 지정해 직접
측정값을 전달할 수도 있습니다. 유효한 ICP나 완전한 수동 측정이 없으면
성공으로 처리하지 않습니다. 시작·종료 스캔은 같은 이름의 `_start.npz`,
`_end.npz`로 저장하며, 기존 평가 JSON을 덮어쓰지 않습니다.

## 5. 제어 설정과 보조 도구

- `motion_settings.json`: 병진·yaw P 이득, 허용오차, 감속 및 정지 설정.
  순수 X/Y는 `linear_x_kp`, `linear_y_kp`를 사용하고, 생략하면 `linear_kp`를
  사용합니다. 대각선 병진은 `linear_kp`, 회전은 `yaw_kp`를 사용합니다.
  `--motion-config`로 다른 JSON을 지정합니다. 현재 기본값의
  `candidate_settings: true`는 실물 재검증 대상임을 나타냅니다.
  현재 X/Y/yaw와 대각선 P는 모두 6.0입니다. 이 P 제어는 각 시험 구간의
  거리·각도 정지를 위한 일시적인 평가 제어입니다. 휠 odom의 기구학 보정
  계수나 상시 주행 controller 설정으로 사용하지 않습니다.
- `motion_control.py`, `motion_runtime.py`: 평가와 휠 진단의 공통 주행 경로.
  stale·역행·중복 stamp 또는 비정상 odometry는 STOP과 disable을 요청하며,
  안전 중단 후 같은 실행에서 주행을 재개하지 않습니다.
- `run_oneway_eval.py`: 편도 이동의 목표 정지 잔차와
  `odom − LiDAR` 추정 오차를 분리합니다. `--extrinsic-config`의 평면 보정값을
  ICP 초기 추정과 결과 변환에 함께 사용하며, `--odom-topic`으로 평가할
  odometry 입력을 선택합니다.
- `run_wheel_only_eval.sh`, `run_wheel_only_eval.py`: ROS 환경을 준비하고
  Agent·base driver·LiDAR를 준비한 뒤 config 기반 편도 평가를 반복합니다.
  준비 실패·주행 실패·중단 시 후속 반복을 시작하지 않습니다.
- `run_multi_speed_eval.py`: 속도별 반복 평가와 비교 플롯.
- `tune_yaw_p.py`: yaw P 후보별 스텝 응답 및 정착·오버슛 분석.
- `run_motion_response.sh`, `run_motion_response.py`: 현재 편도 평가 제어기를
  그대로 사용하는 X/Y/yaw 실물 P 응답 측정. 기존 `tune_yaw_p.py`의 독립 제어기나
  합성 응답이 아닌 실제 wheel odom trace를 저장합니다.
- `capture_preflight.py`: 정지 상태 odometry·WheelState·스캔 점검.
- [휠 진단](../wheel_calibration/README.md): 정상상태 휠 속도·PWM 분석.

주행 도구들은 `/tmp/cleany_odom_eval.lock`으로 중복 실행을 막습니다.
실제 이동 전 다른 `/cmd_vel` publisher가 있는지도 검사합니다.
기존 평가의 기본 LiDAR → base 변환은 yaw `-7.85°`, x lever arm `0.029 m`입니다.
변경된 장착 상태의 편도 평가는 아래처럼 새 보정 파일을 명시적으로 선택합니다.
`run_eval.sh`와 `run_multi_speed_eval.py`에는 이 선택 옵션이 없으므로,
아래 재측정에는 config 기반 전용 launcher를 사용합니다.
측정 결과는 Git 제외 경로인 `artifacts/odometry_evaluation/`에 저장됩니다.

### 속도별 3회 휠 odom 재측정

실험 설정은 [`configs/mission/wheel_odometry_evaluation.json`](../../configs/mission/wheel_odometry_evaluation.json)에
모읍니다. 기본값은 ±X/±Y와 두 대각선 각각 편도 총거리 0.5 m, ±90° 회전,
병진 0.20/0.35/0.50 m/s와 회전 0.30/0.60/1.00 rad/s 각각 3회입니다.
현재 `motion_settings.json`의 X/Y/yaw와 대각선 P=6.0을 사용합니다.
`--dry-run`에서 패턴 목록, 적용 P와 전체 편도 수를 확인합니다. 긴 거리 실험은 config의
`translation_distance_m`를 `1.0` 또는 `2.0`으로 바꿉니다.

레포 루트에서 실행합니다. MCU와 LiDAR를 연결하고 시험 공간과 물리 emergency
stop을 준비합니다. Launcher는 `runtime_config`의 설정으로 필요한 실물 노드를
자동 기동합니다. 정상 실행 중인 노드는 재사용하며 다른 `/cmd_vel` 명령원이
있으면 주행 전에 중단합니다.
기본 config는 명시적으로 `/wheel/odom`을 구독하여 IMU·localization 융합 출력을
평가 입력으로 사용하지 않습니다. LiDAR ICP는 비교 기준이며 odom에 융합하지 않습니다.

```bash
# 이동 없이 config 검증과 전체 계획 확인
bash tools/odometry_evaluation/run_wheel_only_eval.sh --dry-run

# 노드 자동 기동 → 준비 확인 → 기본 config로 실물 반복 실험
bash tools/odometry_evaluation/run_wheel_only_eval.sh

# 다른 config로 실험
bash tools/odometry_evaluation/run_wheel_only_eval.sh \
  --config configs/mission/wheel_odometry_evaluation.json
```

Ubuntu 22.04 외 호스트에서는 `ros2-humble` Distrobox로 자동 진입하고 ROS overlay를
적용합니다. 설치된 micro-ROS Agent overlay도 함께 적용합니다.
별도 터미널에서 노드를 기동하거나 source할 필요가 없습니다.
Config의 파일 경로와 `output_dir`는 레포 루트 기준이며, 작업 디렉터리와 무관합니다.
`--dry-run`은 ROS 노드·주행 프로세스를 실행하거나 결과 디렉터리를 만들지 않습니다.

#### 자동 기동과 준비 확인

[`configs/robot/wheel_evaluation_runtime.json`](../../configs/robot/wheel_evaluation_runtime.json)에서
MCU·LiDAR by-id USB 경로, base hardware profile, LiDAR 옵션과 대기 시간을 바꿉니다.

- Agent가 없으면 연결된 MCU에 serial Agent를 시작하고, base driver가 없으면
  `mock:=false`로 wheel odometry를 함께 시작합니다.
- LiDAR가 정상 스캔을 발행하면 재사용합니다. USB 재연결 후 스캔이 끊긴 기존
  프로세스는 같은 사용자·설정된 serial 경로·허용 node 이름까지 식별되는 경우만
  종료하고 by-id 경로로 다시 시작합니다. 식별되지 않는 프로세스를 임의 종료하거나
  중복 driver를 추가하지 않습니다.
- `/base/enable` service와 timestamp가 진행하는 실제 odom·scan·WheelState를
  확인합니다. MCU가 disabled·fault-free이고 wheel 목표/PWM이 0이며,
  차체 필터 뒤 외부 환경 반사점이 충분히 수신되어야 주행을 시작합니다.
- 기본 30초 안에 준비되지 않으면 누락·정체 원인을 기록하고 평가 프로세스를
  시작하지 않습니다. 자체 기동 프로세스의 로그는 배치 결과 디렉터리에 남깁니다.
- 완료·실패·Ctrl+C에서는 평가 프로세스의 STOP·disable 정리 후 자체 기동한
  프로세스만 종료합니다. 재사용한 정상 프로세스는 유지합니다.

MCU에는 micro-ROS **모터 제어 펌웨어**가 실행되어야 합니다.
독립 IMU 캘리브레이션 펌웨어는 `cleany.imu_calibration/v1` JSON만 응답하며
`/base/wheel_state`를 발행하지 않습니다. 이 경우 Agent를 켜도 odom은 생성되지
않습니다. Launcher는 이 누락을 보고하며 펌웨어를 자동으로 덮어쓰지 않습니다.
모터 펌웨어 복원은 [IMU 진단의 복원 절차](../../esp32/imu_calibration/README.md#hardware-acceptance-and-restoration)를 따릅니다.

한 번의 pattern·speed 실행은 outbound와 inbound 두 편도를 각각 평가합니다.
전체는 45회 왕복, 90개 편도이며, 이동 실패나 ICP 거부 시 다음 실행으로
넘어가지 않습니다. 거리의 정지 목표는 wheel odom 기준이고 실제 변위는
LiDAR ICP로 별도 추정합니다.

`lidar_min_range_m: 0.45`는 이번 장착의 프로파일 반사점(관측 거리 약 0.23~0.39 m)을
평가용 ICP에서 제외하는 거리 필터입니다. `/scan` 자체나 장애물 회피 입력을
변경하지 않습니다. 필터 후에도 서로 다른 방향의 고정 구조물이 충분히 보여야 합니다.
이는 보정용 프로파일 정합과 달리 외부 환경 스캔을 비교하는 주행 평가입니다.
새 센서 위치에서 필터를 재사용하려면 차체 반사점의 최대 거리를 다시 확인합니다.

보정 파일은 `yaw_deg`, `lever_arm_x_m`, `lever_arm_y_m`의 유한한 수치 세 개를
필수로 포함하는 JSON입니다. 추가 provenance 필드는 허용합니다. 적용한 보정값과
파일 식별 정보, odom 입력 및 최소 거리는 개별 결과와 summary에 기록합니다.
배치 실행은 config·motion·extrinsic 파일을 사전 검증하고 config 사본과 전체 실행
결과를 `output_dir/batch_<시각>_<pid>/`에 보존합니다. `config_snapshot.json`은
실험 설정과 원본 파일 식별 정보, `motion_config.json`과 `extrinsic_config.json`은
각 반복에 적용하는 고정 사본, `batch_summary.json`은 배치 종료 상태입니다.
각 반복의 측정 결과는 `run_1/`, `run_2/`, `run_3/` 아래에 저장합니다.
런타임 설정과 hardware profile 사본은 `runtime_config.json`, `base_hardware.yaml`,
자체 기동 프로세스 로그는 `runtime_logs/agent.log`, `base.log`, `lidar.log`에 저장합니다.
중단 신호는 실행 중인 주행 프로세스에 전달하여 STOP·disable
정리를 기다리고 후속 반복을 시작하지 않습니다.
하위 `run_oneway_eval.py`를 직접 사용할 때 보정 파일을 생략하면 이전 장착의
기본값을 사용하므로 현재 장착에는 생략하지 않습니다.

### 전 축 P=6 검증 프로토콜

[전 축 검증 프로토콜](VALIDATION_PROTOCOL.md)은 X/Y/yaw와 두 대각선의
시험 행렬, 파라미터 조정, 준비·중단 절차와 결과 판정을 정리합니다.
설정 파일은
[`configs/mission/all_axis_validation.json`](../../configs/mission/all_axis_validation.json)이며
`run_motion_response.sh`의 기본값입니다. 각 축·속도의 `kp`와 `speed`,
`repetitions`, 거리·각도 및 `guards`를 한 파일에서 조정합니다.
P는 각 trial에만 적용하며, 기본 motion 파일에 되쓰지 않습니다.

```bash
# P=6, 45회 왕복 / 90개 편도 계획 확인. ROS 기동과 이동 없음.
bash tools/odometry_evaluation/run_motion_response.sh --dry-run

# 시험 준비 확인 후 전체 검증
bash tools/odometry_evaluation/run_motion_response.sh \
  --config configs/mission/all_axis_validation.json
```

기본 `require_valid_icp=true`는 외부 비교 거부 시 후속 시험을 중단합니다.
전 축 preset의 `icp_min_inlier_ratio=0.30`은 최소 inlier 비율 30%를 뜻하며,
적용값을 결과에 기록합니다. 생략한 config는 40%를 사용합니다.
제어 응답 완료와 외부 정확도 판정은 별도로 확인합니다.

2026-10-21 수정된 ICP로 **45회 왕복·90개 편도 전체를 완료**했습니다.
`response_summary.status=success`이며 편도 90개와 직접 loop 45개, 총 135개
정합이 모두 유효했습니다. 수렴 사유는 `step_tolerance` 133개,
`bounded_cycle` 2개이며 최대 반복 수는 33회였습니다. 최소 inlier 비율은
42.78%, 최대 RMSE는 17.21 mm였습니다. 적용한 strict 공차는 병진 `1e-5 m`·
회전 `1e-5 rad`, cycle 폭은 1 mm·0.02°입니다.
종료 후 STOP·disable과 자체 기동 프로세스·serial 해제를 확인했습니다.
전체 기록과 그래프:
[`20261021_163717/response_summary.json`](../../artifacts/odometry_evaluation/all_axis_validation/20261021_163717/response_summary.json),
[`response.png`](../../artifacts/odometry_evaluation/all_axis_validation/20261021_163717/response.png).

### X/Y/yaw P 응답 측정과 현재 적용값

실험 config는
[`configs/mission/motion_response_tuning.json`](../../configs/mission/motion_response_tuning.json)입니다.
`trials`에 축(`x`, `y`, `yaw`), P 계수와 속도 상한을 지정합니다.
각 trial은 같은 계수로 정·역방향을 각각 측정합니다. 현재 세션의 범위는 편도
0.5 m·90° 이하이며, 실물 공간과 비상정지 준비 후 실행합니다.

```bash
bash tools/odometry_evaluation/run_motion_response.sh \
  --config configs/mission/motion_response_tuning.json --dry-run
bash tools/odometry_evaluation/run_motion_response.sh \
  --config configs/mission/motion_response_tuning.json
```

- Agent·base·LiDAR 자동 준비와 실제 데이터 수신 확인 후, disabled 상태에서
  service handshake를 마칩니다. 150 ms odom watchdog은 유지합니다.
- `guards`는 세션 시작점 기준 이동 반경, 횡방향 이탈, 병진 중 yaw 변화,
  이동·회전 과도 변위를 제한하는 **시험용 중단값**입니다.
- raw trace에는 위치·각도, 오차, 명령 속도와 odom 속도를 기록합니다.
  정착 진입 시간은 위치와 속도가 모두 허용범위 안에 들어와 이후 유지되는
  시각입니다. 0.5초 hold 완료 시간, 별도 1.5초 dwell과 구분합니다.
- `response.png`는 주행 응답이며 종료 후 dwell·스캔 수집 시간은 제외합니다.
  정·역방향 진행량을 같은 양의 목표로 정규화하고 속도는 크기로 표시합니다.
- `require_valid_icp` 기본값은 `true`입니다. 제어 응답만 검증하는 세션에서
  `false`로 설정하면 정상 주행 trace는 유지하되 미수렴 ICP 비교값은 무효로
  남깁니다. 주행 실패·watchdog·경계 이탈 시에는 항상 후속 시험을 중단합니다.
  일반 wheel-odom 평가의 ICP 실패 처리와 별개입니다.

2026-10-15 실물 응답 39회 왕복(78개 편도)으로 `motion_settings.json`에
**X=6.0, Y=6.0, yaw=6.0**을 적용했습니다. X/Y는 1·2·3·4·6,
yaw는 2·3·4·6을 비교했습니다. 선택 계수의 중·고속 제어 응답은
정·역방향 각각 최소 3회 검증했습니다.

| 축 | 저속 비교 P | 마지막 20% 구간 평균 시간 |
|---|---|---|
| X | 1 → 6 | 마지막 10 cm: 3.289 → 0.690 s |
| Y | 1 → 6 | 마지막 10 cm: 3.301 → 0.689 s |
| yaw | 2 → 6 | 마지막 18°: 1.620 → 1.251 s |

선택 계수의 종료 잔차는 wheel odom 기준 병진 3.13~4.63 mm,
yaw 0.56~0.75°였고 목표 초과는 관측되지 않았습니다. 이 값은 제어기 피드백
잔차이며 외부 기준 절대 위치 정확도가 아닙니다. 중·고속 yaw에서 P=6은 P=4와
정착 시간이 비슷하면서 종료 잔차가 작아 선택했습니다. 감속 envelope,
최소속도, 정지 허용오차와 hold/dwell은 유지했습니다.

측정 데이터·그래프·선택 근거는 Git 제외 경로
[`artifacts/odometry_evaluation/p_response_tuning/REPORT.md`](../../artifacts/odometry_evaluation/p_response_tuning/REPORT.md)에 있습니다.
ICP 3건은 미수렴으로 외부 정확도 비교에서 제외했으며 원본과 거부 사유를 보존합니다.

### 대각선 P 튜닝과 외부 오차 측정

`diagonal_left`는 전좌↔후우(+45°/−135°), `diagonal_right`는
전우↔후좌(−45°/+135°) 왕복입니다. 속도와 목표 거리는 **벡터 크기**입니다.
편도 0.5 m의 X/Y 성분은 각각 약 0.354 m이며, 각 축을 0.5 m씩 이동하지 않습니다.
`run_oneway_eval.py --patterns`와 wheel 평가 config의 `patterns`에도
두 대각선 이름을 사용할 수 있습니다.

```bash
# 계획 확인
bash tools/odometry_evaluation/run_motion_response.sh \
  --config configs/mission/diagonal_response_tuning.json --dry-run

# 0.20 m/s에서 P=1/3/4/6 비교, 두 대각선 왕복
bash tools/odometry_evaluation/run_motion_response.sh \
  --config configs/mission/diagonal_response_tuning.json

# P=6, 속도상한 0.20/0.35/0.50 m/s, 각 대각선 3회 왕복
bash tools/odometry_evaluation/run_motion_response.sh \
  --config configs/mission/diagonal_response_validation.json
```

대각선 제어는 **진행방향에 투영한 위치 오차**로 속도를 정합니다.
횡방향 위치·yaw 편차는 별도 측정과 경계 이탈 중단 대상입니다.
5 mm 정지 허용오차도 진행방향 투영값이며 2D 위치오차 norm과 구분합니다.
대각선 후보는 fallback `linear_kp`만 바꾸며 X/Y/yaw 전용 계수는 유지합니다.

2026-10-15 대각선 실물 28회 왕복(56개 편도) 측정 후
**`linear_kp=6.0`**을 적용했습니다. 현재 설정 파일의 fallback·X·Y·yaw P는
모두 6.0입니다. 검증한 대각선 방향은 ±45°입니다.

| 저속 조건 | 마지막 10 cm 시간, P=1 → P=6 |
|---|---|
| 전좌↔후우 | 3.502 → 0.666 s |
| 전우↔후좌 | 3.653 → 0.662 s |

선택 P의 wheel-odom 진행방향 종료 잔차는 1.30~4.51 mm, 횡방향 잔차 최대
2.82 mm, 2D 종료 잔차 최대 5.17 mm였습니다. 진행방향 목표 초과는 없었습니다.
저·중·고속 상한 각각 방향별 최소 3회 검증했으며 실측 최고 속도는
약 0.384 m/s입니다.

`endpoint_errors`는 다음 세 값을 분리합니다.

- `wheel_goal`: wheel odom의 목표 잔차·횡편차·yaw 변화.
- `icp_goal`: 유효 ICP로 추정한 변위와 명령 목표의 차이.
- `odom_minus_icp`: wheel odom 변위와 ICP 변위의 차이.

56개 편도 중 ICP 52개가 유효했습니다. 무효 4개는 `null`로 보존하며 오차 평균에
포함하지 않습니다. 대각선 config의 `require_valid_icp=false`는 이 무효 처리를
유지하면서 제어 응답을 계속 측정하는 설정입니다.

**빠른 제어 응답과 외부 위치 정확도는 구분합니다.** P=6에서 ICP 목표 차이의
조건별 평균은 6.53~20.32 mm였고, 전우↔후좌의 고속 상한 조건에서 최대
43.21 mm가 관측됐습니다. 휠 odom 피드백으로 보이지 않는 편차와 ICP 측정
불확실성을 포함하며, 원인을 P 또는 휠 슬립만으로 확정하지 않았습니다.

그래프·편도별 원본·선택 근거:
[`artifacts/odometry_evaluation/diagonal_p_tuning/REPORT.md`](../../artifacts/odometry_evaluation/diagonal_p_tuning/REPORT.md).
시험 중 LiDAR 종료 후 재기동에서 장치 정보/health 응답과 별개로 scan 시작이
실패하는 현상이 발생했습니다. 이 경우 주행 전에 중단됐고, PC 쪽 LiDAR USB를
완전히 재연결해 실제 스캔 수신을 확인한 후 재개했습니다. 재기동 문제의 원인은
아직 확정되지 않았습니다.

---

## 6. 단위 테스트

```bash
make test-odometry-evaluation
```

이 target은 배치 config, 변환, ICP, 공통 주행 로직과 휠 진단의 pytest 및
배치 launcher의 Bash 문법 검사를 실행합니다. 장치나 실물 주행 없이 검증합니다.
