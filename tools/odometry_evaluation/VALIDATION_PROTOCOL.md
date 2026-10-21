# 전 축 실물 검증 프로토콜

X/Y, yaw와 두 대각선의 정지 응답 및 wheel odom과 LiDAR ICP의 차이를
같은 절차로 측정한다. 기본 시험용 P는 모든 축에서 **6.0**이다.

P는 각 편도의 거리·각도 정지를 위한 일시적인 평가 제어다. trial에만 적용하며
실행 후 설정 파일에 되쓰지 않는다. MCU 휠 속도 PI·FF, 휠 odom geometry와
상시 주행 controller의 튜닝과 구분한다.

## 1. 설정과 시험 행렬

편집할 파일은
[`configs/mission/all_axis_validation.json`](../../configs/mission/all_axis_validation.json)이다.
`run_motion_response.sh`의 기본 config이며, 기존 후보 sweep과 별개다.

| `axis` | outbound → inbound | 편도 목표 | 속도 상한 |
|---|---|---|---|
| `x` | 전진 → 후진 | 0.5 m | 0.20 / 0.35 / 0.50 m/s |
| `y` | 좌측 → 우측 | 0.5 m | 0.20 / 0.35 / 0.50 m/s |
| `yaw` | 반시계 → 시계 | 90° | 0.30 / 0.60 / 1.00 rad/s |
| `diagonal_left` | 전좌 → 후우 | 벡터 총거리 0.5 m | 0.20 / 0.35 / 0.50 m/s |
| `diagonal_right` | 전우 → 후좌 | 벡터 총거리 0.5 m | 0.20 / 0.35 / 0.50 m/s |

각 속도·축에서 3회 왕복, 총 **45회 왕복 / 90개 편도**다. 반복마다 저속의
다섯 축, 중속의 다섯 축, 고속의 다섯 축 순서로 실행한다.
대각선의 각 X/Y 성분은 약 0.354 m다. 속도는 명령 상한이며, 짧은 구간의
가감속과 휠 속도 제한 때문에 실측 최고 속도와 다를 수 있다.

### 파라미터 편집

| 위치 | 의미와 적용 범위 |
|---|---|
| `trials[].axis` | `x`, `y`, `yaw`, `diagonal_left`, `diagonal_right` |
| `trials[].kp` | 해당 왕복의 P. 허용 범위 `0 < kp <= 8`. 기본 6.0 |
| `trials[].speed` | 해당 왕복의 속도 상한. 병진 최대 0.5 m/s, yaw 최대 1 rad/s |
| `repetitions` | 전체 trial 목록 반복 수. 1~10 |
| `translation_distance_m` | 모든 병진 편도의 벡터 총거리. 최대 0.5 m |
| `rotation_angle_deg` | 회전 편도 각도. 최대 90° |
| `require_valid_icp` | 기본 `true`. 외부 비교가 거부되면 후속 trial 중단 |
| `icp_min_inlier_ratio` | 유효 ICP의 최소 inlier 비율. 현재 preset은 `0.30`(30%), 생략 시 `0.40` |
| `icp_convergence` | 병진·회전 strict 공차, bounded cycle의 최대 폭과 주기. 아래 기본값 참조 |
| `guards` | 아래 시험용 중단 경계 |
| `output_dir` | 결과 묶음의 상위 디렉터리 |

동일 축의 P를 모든 속도에서 바꾸려면 해당 축의 세 trial을 함께 편집한다.
특정 조건만 재시험하려면 복사한 config의 `trials`에 필요한 행만 남긴다.
튜닝 때는 P, 속도와 감속 설정을 동시에 바꾸지 않고 원본과 변경 결과를 구분한다.
현재 전 축 preset은 inlier 하한을 30%로 사용한다. 적용값은 결과에 기록한다.
ICP 최대 50회, 병진 `1e-5 m`·회전 `1e-5 rad`의 strict 공차를 사용한다.
수렴은 correspondence 거리의 annealing이 최종 40 mm 단계에 도달한 뒤만
판정한다. 주기 2~6이 세 주기 반복되고 반복 구간의 pose 차이가 1 mm·0.02°
이하인 bounded cycle도 수렴으로 기록한다. 수렴 후 inlier·RMSE·관측성 검사를
통과해야 유효하다. 알고리즘과 저장 스캔 재검증은
[ICP 수렴과 관측성](README.md#icp-수렴과-관측성)을 따른다.

`icp_convergence`의 설정 키는 `translation_tolerance_m`, `rotation_tolerance_rad`,
`max_translation_span_m`, `max_rotation_span_rad`, `max_period`다.
현재 기본값은 각각 `1e-5`, `1e-5`, `0.001`, `0.00034906585039886593`(0.02°), `6`이다.
적용값과 각 정합의 `convergence_reason`은 결과에 기록한다.

`evaluation_config`가 가리키는
[`wheel_odometry_evaluation.json`](../../configs/mission/wheel_odometry_evaluation.json)에서는
odom topic, timeout, LiDAR 필터·extrinsic, runtime과 기본 motion 파일을 읽는다.
이 프로토콜의 축·P·속도·거리·반복 수는 **validation config가 우선**하며,
wheel config의 `patterns`나 속도 목록을 수정해도 이 trial 행렬은 바뀌지 않는다.
경로는 레포 루트 기준이며 절대 경로도 사용할 수 있다.

기본 motion의 정지 공차, 최소속도, 감속과 hold는
[`motion_settings.json`](motion_settings.json)에서 관리한다.
X/Y trial은 축별 P, 두 대각선 trial은 fallback `linear_kp`, yaw trial은
`yaw_kp`만 실행 중 덮어쓴다. 다른 축의 설정은 보존한다.

## 2. 시험 전 준비

1. 평탄하고 사람·장애물이 없는 3.5 m × 3.5 m 이상 구역과 물리 emergency
   stop을 준비한다. 센서·프로파일과 적재 상태를 고정한다.
2. 시작 위치와 heading을 표시한다. 대각선 0.5 m가 각 축 0.5 m가 아님을
   확인하고 전체 차체의 회전 공간을 확보한다.
3. MCU의 모터 제어 펌웨어, MCU·LiDAR USB 연결과
   [`wheel_evaluation_runtime.json`](../../ros2_ws/src/cleany_base_driver/config/wheel_evaluation_runtime.json)의
   by-id 경로를 확인한다. 다른 teleop·Nav2·평가 명령원을 종료한다.
4. 장착 상태와
   [`lidar_extrinsic_20261015.json`](../../ros2_ws/src/cleany_base_driver/config/lidar_extrinsic_20261015.json)을
   대조한다. 거리 필터 0.45 m 밖에 서로 다른 방향의 고정 구조물이 보여야 한다.
5. 아래 offline 검사와 dry-run에서 모든 P=6, 90개 편도와 결과 경로를 확인한다.
   dry-run은 장치 준비 확인을 대신하지 않으며 ROS 노드나 이동을 시작하지 않는다.

```bash
make test-odometry-evaluation

bash tools/odometry_evaluation/run_motion_response.sh \
  --config configs/mission/all_axis_validation.json --dry-run
```

## 3. 실물 실행과 중단

새 장착·노면·적재 또는 변경한 계수의 첫 시험은 config를 복사해 저속 다섯
trial과 `repetitions: 1`만 남겨 먼저 확인한다. 정상 응답을 확인한 뒤 전체
행렬을 실행한다.

```bash
# 시험 공간과 emergency stop 준비를 확인한 뒤 실행
bash tools/odometry_evaluation/run_motion_response.sh \
  --config configs/mission/all_axis_validation.json
```

Launcher는 Agent·base·LiDAR를 준비하고 진행하는 odom·scan·WheelState,
disabled·fault-free MCU와 `/base/enable`을 확인한다.
주행 전 passive STOP handshake를 완료하며, 주행 중 odom watchdog은 150 ms다.
정상 실행 중인 stack은 재사용한다.

| `guards` 기본값 | 중단 조건 |
|---|---|
| `session_radius_m: 0.75` | 세션 시작 위치에서 wheel-odom 변위 반경 초과 |
| `cross_track_m: 0.08` | 병진 경로 수직 이탈 또는 회전 중심 병진 이탈 초과 |
| `translation_overshoot_m: 0.025` | 편도 시작 기준 진행축 변위의 절댓값이 목표 + 25 mm 초과 |
| `translation_yaw_drift_deg: 10.0` | 병진 중 yaw 변화 절댓값 초과 |
| `rotation_overshoot_deg: 5.0` | 회전 변화 절댓값이 목표 + 5° 초과 |

경계는 wheel odom 기반 **시험 중단값**이며 외부 정확도의 합격 기준이 아니다.
주행 실패, watchdog, 경계 이탈, STOP·disable 미확인이나 유효하지 않은 ICP는
후속 시험을 중단한다. Ctrl+C도 STOP·disable 정리를 요청한다.
세션 누적 이탈로 중단되면 원인과 실제 위치를 확인하고 새 실행으로 재시험한다.

LiDAR health 응답만 있고 실제 scan이 없으면 시작하지 않는다. 재기동 실패 시
로그를 보존하고 USB 연결과 scan 수신을 확인한 뒤 새 실행을 시작한다.

## 4. 기록과 판정

결과는 `artifacts/odometry_evaluation/all_axis_validation/<시각>/`에 저장한다.

- `config.json`, `evaluation_config.json`, `base_motion_config.json`: 적용 설정.
- `runtime_ready.json`, `runtime_logs/`: stack 준비 근거와 자체 기동 로그.
- `rep*_*.json`: 왕복별 제어 trace, 편도·loop ICP, 응답 지표와 endpoint 오차.
- `*_start.npz`, `*_destination.npz`, `*_returned.npz`, `*_scans.npz`: 원본 스캔.
- `response_summary.json`, `response.png`: 전체 실행 상태와 응답 비교.

각 축·속도·방향을 분리해 완료 수, 유효 ICP 수, 평균·표준편차·최대값과
중단 사유를 보고한다. 서로 다른 P의 결과를 같은 통계에 섞지 않는다.

1. **제어 응답:** 두 편도의 `is_valid`, `reached_tolerance`와 `metrics.valid`를
   확인한다. 현재 병진 진행축 공차 5 mm 또는 yaw 공차 1°와 정지 속도 조건을
   0.5초 유지해야 한다. 정착 진입, hold 완료와 1.5초 dwell은 구분한다.
   오버슛과 실측 최고 속도도 기록한다.
2. **외부 비교:** outbound·inbound·직접 loop ICP의 품질 판정을 확인한다.
   병진의 `endpoint_errors`는 wheel 목표 잔차, ICP 목표 차이와
   `odom_minus_icp`를 구분한다. yaw는 각 편도의
   `one_way_error.error_yaw_deg`와 병진 드리프트를 확인한다.
   무효 ICP는 `null`과 거부 사유를 유지하고 통계에서 제외한다.
3. **전체 완료:** 45개 왕복 record와 90개 정상 편도, 반복 수 및
   `response_summary.status == "success"`를 확인한다. 이는 설정된 실행·ICP
   조건의 완료이며 외부 위치 정확도 합격과는 별도다. 외부 정확도 판정에는
   별도로 합의한 위치·yaw 기준과 독립적인 테이프·각도 측정을 사용한다.

제어 응답만 진단할 때는 복사한 config에서 `require_valid_icp: false`를
명시할 수 있다. 이 경우 결과를 제어 전용으로 표시하며 ICP 거부를 정확도
검증 성공으로 바꾸지 않는다. 주행·watchdog·경계 중단은 유지한다.
