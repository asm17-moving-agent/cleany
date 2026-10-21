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
  `--motion-config`로 다른 JSON을 지정합니다. 현재 기본값의
  `candidate_settings: true`는 실물 재검증 대상임을 나타냅니다.
- `motion_control.py`, `motion_runtime.py`: 평가와 휠 진단의 공통 주행 경로.
  stale·역행·중복 stamp 또는 비정상 odometry는 STOP과 disable을 요청하며,
  안전 중단 후 같은 실행에서 주행을 재개하지 않습니다.
- `run_oneway_eval.py`: 편도 이동의 목표 정지 잔차와
  `odom − LiDAR` 추정 오차를 분리합니다.
- `run_multi_speed_eval.py`: 속도별 반복 평가와 비교 플롯.
- `tune_yaw_p.py`: yaw P 후보별 스텝 응답 및 정착·오버슛 분석.
- `capture_preflight.py`: 정지 상태 odometry·WheelState·스캔 점검.
- [휠 진단](../wheel_calibration/README.md): 정상상태 휠 속도·PWM 분석.

주행 도구들은 `/tmp/cleany_odom_eval.lock`으로 중복 실행을 막습니다.
실제 이동 전 다른 `/cmd_vel` publisher가 있는지도 검사합니다.
LiDAR → base 변환의 현재 적용값은 yaw `-7.85°`, x lever arm `0.029 m`이며,
센서 장착이 바뀌면 재측정과 코드 반영이 필요합니다.
측정 결과는 Git 제외 경로인 `artifacts/odometry_evaluation/`에 저장됩니다.

---

## 6. 단위 테스트

```bash
distrobox enter ros2-humble --no-tty --clean-path -- bash -c \
  "export PYTHONNOUSERSITE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 && \
   source /opt/ros/humble/setup.bash && \
   cd tools/odometry_evaluation && python3 -m pytest"
```
