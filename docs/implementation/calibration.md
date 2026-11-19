# 캘리브레이션 작업 가이드

측정할 대상을 먼저 고릅니다. **관측·분석 결과를 얻는 것과 실제 보정값을 적용하는 것은 별도 작업**입니다.

| 확인하려는 것 | 작업 | 결과 |
|---|---|---|
| D435와 LiDAR가 같은 벽을 보는가? | [벽 비교](#d435와-lidar-벽-비교-및-헤드-관절-관측) | 거리·법선 잔차 |
| 휠 odom과 실제 관측 이동이 다른가? | [bag 분석](#bag-기반-슬립-의심-구간-분석) | 의심 구간·판정 불가 구간 |
| 저장된 카메라·관절 보정값을 쓰려는가? | [기존 프로필 적용](#jetson-실물-모델과-2026-10-09-보정-프로파일) | 보정된 모델 TF |
| 새 Hand-eye 데이터를 수집하려는가? | [수집과 평가](#hand-eye-수집과-평가) | 검토용 변환과 평가 |
| ToF 장착·정지 조건을 정하려는가? | [Commissioning](#tof-근접-보호-commissioning) | 실측에 근거한 로봇 설정 |

## D435와 LiDAR 벽 비교 및 헤드 관절 관측

로봇을 **동일한 평평한 벽 앞에 정지**시킨 상태에서 비교합니다. Humble, NumPy, matplotlib, pyserial과 실제 영상·CameraInfo·`/scan_raw` 입력이 필요합니다.

![D435와 LiDAR의 벽 투영, 거리 차이와 법선 각도 비교 출력](../assets/examples/calibration-wall.png)

왼쪽은 두 센서의 벽 투영, 가운데는 카메라−LiDAR 거리 차이, 오른쪽은 수평 방향 차이와 카메라 벽 법선 기울기입니다. **기록된 출력 예시이며, 그림의 수치를 합격 기준으로 사용하지 않습니다.**

```mermaid
flowchart LR
  Encoder["헤드 encoder / ID 13·14"] --> TF["촬영 시각의 카메라 TF"]
  Depth["D435 깊이 영상"] --> Wall["벽 거리·법선 비교"]
  TF --> Wall
  Scan["LiDAR scan"] --> Wall
  Wall --> Result["비교 PNG / JSON"]
```

USB by-id·모터 ID와 [헤드 설정](../../tools/validation/config/d435_pan_tilt.json)을 검토한 뒤 실행합니다. 기존 차체→카메라 고정 TF 발행자는 중복 실행하지 않습니다.

```bash
# 터미널 A: 읽기 전용 관절 관측
python3 tools/validation/head_pan_tilt_probe.py \
  --config tools/validation/config/d435_pan_tilt.json
# 터미널 B: 자료 수집 후 비교. 사용할 새 경로로 바꿉니다.
python3 tools/validation/capture_depth_lidar.py --output /path/to/new/capture
python3 tools/validation/compare_depth_lidar_wall.py \
  --capture /path/to/new/capture --mount tools/validation/config/d435_cad_mount.json
```

결과 폴더의 `index.html`, `wall-comparison.png`, `wall-comparison.json`에서 잔차와 분석 조건을 함께 확인합니다.

| 결과·모드 | 의미와 한계 |
|---|---|
| 실제 관절 TF 있음 | 카메라 시각과 150ms 이내인 TF 사용. 없으면 공칭 자세 비교로 표시 |
| 거리·법선 잔차 | 한 벽으로 높이·벽에 평행한 이동을 검증할 수 없음. 벽 기울기와 장착 오차도 분리할 수 없음 |
| 기본 실행 | 읽기 전용. `/head/joint_states`와 동적 TF 발행. 휠 `/joint_states`와 분리하며 피드백 지연·오류 시 종료 |
| `--exercise --output …` | **실제 헤드 모터 시험.** 다른 버스 소유자 종료, 차체 정지와 주변 공간 확인 후 별도 실행 |
| 시험 전 검사 | 토크 OFF, 모델 777, 위치 모드, 오류 없음과 온도 확인 |
| 시험 동작·종료 | 현재 자세 부근 축별 ±5°. EEPROM은 쓰지 않으며 종료 시 토크 OFF 확인. 복귀 실패 시 추가 이동 없이 토크 OFF |
| 남는 RAM 설정 | 가속도 5, 속도 80tick/s, 토크 한도 300과 마지막 목표값. `--poses`로 미확인 자세만 별도 시험 가능 |

이 도구는 costmap이나 보정값을 자동 변경하지 않습니다. 광학 TF는 RealSense 드라이버가 계속 소유합니다.

## bag 기반 슬립 의심 구간 분석

```mermaid
flowchart LR
  Odom["기록된 wheel odom"] --> Align["같은 시각·차체 좌표로 비교"]
  Scan["기록된 LiDAR"] --> ICP["휠 초기값 없는 ICP"]
  ICP --> Align
  Align --> Quality{"정합·시간 품질 유효?"}
  Quality -->|"아니오"| Unknown["unknown"]
  Quality -->|"예"| Decision["이동 벡터 오차·연속 창 판정"]
```

Humble과 기록 당시 custom message overlay를 source합니다. NumPy, SciPy와 PyYAML을 사용하며, 입력 bag에는 `/scan_raw`, `/odom`, `/tf_static`이 필요합니다. `/base/wheel_state`는 선택적인 보조 통계입니다.

```bash
python3 tools/validation/slip_detection.py \
  --bag /path/to/bag_directory \
  --start 2026-11-06T07:22:40+09:00 --end 2026-11-06T07:30:25+09:00 \
  --output /path/to/new_analysis_directory --jobs 2
```

| 확인할 것 | 위치·해석 |
|---|---|
| 사건과 창별 판정 | `summary.json`, `windows.jsonl`을 함께 확인 |
| 입력·설정 식별 | `manifest.json`. 원본은 읽기 전용 snapshot으로 열며 기존 출력은 덮어쓰지 않음 |
| 판정 기준 | [slip_detection.json](../../tools/validation/slip_detection.json). 다른 설정은 `--config`, 형상은 `--mask-file`로 지정 |
| 같은 입력 재분석 | `--bag` 대신 `--cache …/inputs.npz` 사용 |
| `unknown` / `no_evidence` | 각각 품질 부족 / 해당 기준에서 증거 없음. 정상·슬립 없음의 확정 판정이 아님 |
| 적용 한계 | 슬립과 걸림을 구별하는 진단이 아님. 느린 슬립·끌림, scan 중 움직임·가림에 한계. per-beam deskew 없음 |

겹치는 창은 독립 표본이나 정확한 사건 시작 시각으로 해석하지 않습니다. 이 도구는 명령·TF·보정값을 발행하거나 수정하지 않으며, 제어 연동에는 별도 실시간 검증이 필요합니다.

## Jetson 실물 모델과 2026-10-09 보정 프로파일

[jetson_20261009](../../ros2_ws/src/cleany_description/config/calibration/jetson_20261009)는 **잠정 보정 프로필**입니다. 실제 장착이 달라졌다면 재검증해야 합니다. CAD 마운트 변경만으로 보정이 갱신되는 것은 아닙니다.

| 자료 | 적용 범위 |
|---|---|
| `*_extrinsics.yaml` | 헤드 RGB·양손목 optical TF. `parent_T_child`는 child→parent 변환, m·rad 단위 |
| `arm_joint_calibration.yaml` | 팔 영점·방향·오프셋의 기준. `q = direction × (tick − zero) × 2π/4096 + offset` |
| `head_joint_calibration.json` | 헤드 pan/tilt 기준과 방향. 수동 영점이며 회전축 추정값은 아님 |
| `intrinsics/`, `motors/`, `reference/` | 원본·참고 자료. 아래 launch가 CameraInfo를 발행하거나 모터 영점을 적용하지 않음 |

```bash
ros2 launch cleany_description description.launch.py \
  use_sim_time:=false model_variant:=legacy_runtime model_profile:=jetson_20261009 \
  calibration_profile:=jetson_20261009
```

`robot_state_publisher`만 실행하므로 `/joint_states`는 별도로 필요합니다. 기본 `calibration_profile`은 빈 문자열이며, 명목 TF를 사용합니다. 보정은 optical 좌표계에 적용하고 카메라 mesh와 헤드 depth 좌표계의 명목값을 옮기지 않습니다. EEPROM homing offset을 관절식에 이중 적용하지 않습니다.

카메라 평가는 고정 보드 가정의 일관성 검사입니다. RMS·수집 조건은 각 보정 YAML을 따르며, `absolute_calibration_verified: false`를 유지합니다. 절대 위치 정확도·노출 동기화·관절 절대각이 검증됐다는 의미는 아닙니다. [모델과 좌표 규약](../../ros2_ws/src/cleany_description/docs/model.md)도 확인하세요.

## Hand-eye 수집과 평가

[수집·오프라인 평가 도구](../../ros2_ws/src/cleany_handeye_calibration/docs/offline-tools.md)와 [좌표·저장 계약](../../ros2_ws/src/cleany_handeye_calibration/docs/frames-and-data.md)을 따릅니다. 결과는 검토용으로 저장하며 TF에 자동 적용하지 않습니다.

## ToF 근접 보호 commissioning

[ToF 기본 설정](../../ros2_ws/src/cleany_tof/config/tof.yaml)은 미실측 후보입니다. 초기 관측은 모터를 분리하거나 바퀴를 띄우고 수행합니다.

| 시험 | 측정·확인 항목 |
|---|---|
| 장착·방향 | 작은 표적을 각 native zone에 이동. flip, roll/pitch/yaw, 위치·유효 FOV |
| 거리·평면 | 알려진 평판의 거리·기울기, sigma·residual, 접근 거리·yaw 오차. 선택 ROI가 책상 전면을 보는지 확인 |
| 표면·사각 | 빈 공간·검정·유리·금속·햇빛·다리·얇은 물체의 유효율, `NO_RETURN`, 불신뢰, ROI 높이. 다른 센서와 함께 놓치는 경우 포함 |
| 시간·단절 | USB 부하·read 지연·재연결·watchdog. first-좌표계 offset 불확실성도 reaction budget에 포함 |
| 실제 제동 | 감독자·독립 E-stop이 있는 통제 공간에서 payload·바닥·배터리별 전후·횡이동·회전의 최악 제동거리와 감속도 하한 |
| 접근·취소 | 저속·비스듬한 평판·가림, Nav2 동시 goal, cancel, Mission/Supervisor/guard 재시작, lease 회수와 완료 후 정지 유지 |
| ROI 필터 | 바닥·몸체 return 구분. 15Hz에서 인접 수·cluster 거리·confirm/release·hysteresis 조정. 비상 단일 return과 일반 검출의 지연·오탐·미탐 분리 |
| 책상 품질 | 분산된 유효 row/column·최소 폭·residual. 일부 불량에서 평면 유지와 평면 소실 시 zero/abort 구분 |

실측으로 `deceleration`, `reaction_s`, footprint와 margin을 정하고 근거·설정을 보존한 뒤에만 `calibrated=true`를 적용합니다. 약 3.5m 미검출은 검출 범위·빈 공간의 보장이 아닙니다. ToF만으로 평면 좌우 중심은 관측할 수 없으므로 staging과 팔 도달성은 별도로 검증합니다. [상태·필터·제동 계약](../../ros2_ws/src/cleany_tof/docs/interfaces.md)을 따릅니다.

## D435 미리보기의 LiDAR 높이 하한

| 좌표 기준 | 값과 의미 |
|---|---|
| 카메라 CAD 공칭 높이 | CAD 원점에서 1.0218m. 바닥 z=−0.0955m를 적용한 추정 지상 높이는 약 1.1173m |
| 기존 LiDAR 평면 규약 | Z=0. 실제 장착 높이 측정값과 구분 |
| D435 필터의 `min_base_z` | `base_link` 기준 0.192m = 하우징 바닥 0.150 + 레이저 오프셋 0.042m. 추정 지상 높이는 0.2875m |

D435 점군은 촬영 시각의 실제 head encoder TF로 변환합니다. 높이 하한은 로봇 좌표에 고정되어 tilt·바닥 평면 추정에 따라 바뀌지 않습니다. **하한보다 낮은 물체도 제외**되므로 낮은 장애물 검출을 보장하지 않습니다. 측정 바닥면은 표시 grid와 상한 기준으로만 사용하며, 위 값은 CAD 기반 추정입니다.

## 2026-11-09 머리 모터 교체와 허용 범위

사용자가 지정한 좌우 끝과 상하 끝을 각각 5 Hz로 15회 수집해 다음 범위를
등록했다. 각 끝점의 15개 tick은 모두 같았다. 14번의 정면 영점은 별도로
수집한 425 tick이며 13번은 2009 tick을 사용한다.

| 모터 | 허용 tick 범위 | 정면 영점 tick | 정면 기준 각도 범위 |
|---|---|---|---|
| 13번 좌우 | 8~4001 | 2009 | −175.078125°~+175.869141° |
| 14번 상하 | 44~1101 | 425 | −33.486328°~+59.414063° |

모터 EEPROM의 최소 위치 주소 9와 최대 위치 주소 11에 범위를 저장하고
잠금 주소 55를 다시 잠갔다. 포트를 닫았다 다시 열어 저장값과 토크 OFF를
확인했다. 회전 방향은 왼쪽 및 위쪽 이동에서 raw tick이 감소하는 측정과
대조했다.

운영 매핑은 `configs/robot/head_preview_joints.json`이며
`joint_position_bounds`의 rad 범위를 상태 모델과 제어 모델에 적용한다.
읽기 전용 RViz의 별도 매핑은 Jetson
`~/.config/cleany/head-rviz-mapping.json`에 저장한다. 원본 측정, 변경 전 값과
적용 검증은 [실행 근거](../EVIDENCE.md)에 연결한다.


## 양팔의 수동 ROS 영점 정렬

2026-11-02에 사용자가 양팔을 차례로 URDF의 다섯 관절이 모두 0 rad인
자세로 유지한 상태에서 ID 1–5와 7–11의 엔코더를 읽었다. 현재 운영 설정
`cleany_robot_interface/config/arm_preview.yaml`은 다음 `zero_tick`을 사용한다.

| 관절 | 왼팔 모터 | 왼팔 영점 tick | 오른팔 모터 | 오른팔 영점 tick |
|---|---:|---:|---:|---:|
| shoulder_pan | 1 | 1997 | 7 | 2042 |
| shoulder_lift | 2 | 3075 | 8 | 3111 |
| elbow_flex | 3 | 1066 | 9 | 997 |
| wrist_flex | 4 | 1995 | 10 | 2081 |
| wrist_roll | 5 | 3092.5 | 11 | 3055.5 |

초기값은 20 Hz로 읽은 연속 60개 표본의 중앙값이다. 오른팔 팔꿈치의 최종
영점은 사용자가 양쪽 팔꿈치의 접힘 각도가 같다고 지정한 자세에서
왼쪽 ID 3의 ROS 각도와 일치하도록 보완한 값이다. 이는 왼팔 기준의
상대 각도 보정이다. 채택한 구간의 관절별
최대 변동은 양팔 모두 8 tick 이하였다. `direction`과 `offset_rad`, 양쪽
그리퍼와 모터 EEPROM은 변경하지 않았다. 이는 사용자 수동 정렬에 대한
기준점이며 절대 각도 정확도나 구동 검증 결과가 아니다.

손목 회전은 실물의 양쪽 카메라 마운트가 위쪽을 향하는 자세를 기준으로
RViz에서 방향을 확인한 뒤 보완했다. 현재 영점은 이 미리보기의 회전량을
실측 tick의 ROS 변환에 반영한 값이다. 카메라 외부 보정과 다른 관절 값은
유지했으며, 시각적 방향 확인을 카메라 광학축의 정밀 보정으로 간주하지 않는다.

다시 정렬할 때는 제어기와 조회 프로세스의 포트 점유를 해제하고, 토크가
꺼진 팔을 기준 자세로 지지한 뒤 안정된 엔코더 구간을 수집한다. 현재 팔의
`offset_rad`는 0이므로 해당 중앙값을 `zero_tick`으로 사용한다. 새 설정은
읽기 전용 상태 조회 또는 제어 프로세스가 다음에 시작할 때 읽는다.
원본 측정과 적용 근거는 [실행 근거](../EVIDENCE.md)를 따른다.

## 2026-11-15 양팔 보정과 그리퍼 복원 기준

양팔 모터의 영점과 범위는 [현장 보정 기록](../../ros2_ws/src/cleany_robot_interface/config/calibration/jetson_20261115/manifest.json)에 보관한다.
이후 [개방 0° 보정](../../ros2_ws/src/cleany_robot_interface/config/calibration/jetson_20261115_open160/manifest.json)에서 그리퍼 6번과 12번만 갱신했다.

| 모터 | Homing offset | 최대 개방 | 닫힘 목표 |
| --- | ---: | ---: | ---: |
| 6, 왼쪽 | −1400 | 2048 tick | 3868 tick |
| 12, 오른쪽 | −1618 | 2048 tick | 3868 tick |

모터 혼 정면에서 시계 방향 닫힘 160°는 1820 tick이다. 모터 보정 JSON,
EEPROM 읽기 근거와 로봇별 설정에 병합할 조각은 스냅샷에 담았다.
실측은 양쪽 닫힘 목표 도달을 확인했다. 왕복 동작, 개구 폭 곡선과 힘은
이 측정에서 검증하지 않았다.

운영 시 [양팔 매핑](../../ros2_ws/src/cleany_robot_interface/config/arm_preview.yaml)은 같은 패키지의 `calibration/jetson_20261115_open160/motors/`에서 좌우 모터 JSON을 읽는다. 두 파일은 Jetson LeRobot 캐시의 운영 사본과 바이트 단위로 같다. [카메라 자세](../../configs/robot/jetson/calibration/camera_pose.json)는 이 매핑, 모터 보정과 기준 파일들의 SHA-256을 확인한다. 모델과 검증 입력 경로는 자세 파일을 기준으로 해석한다. 과거 촬영 기록에 남은 장비 경로는 당시 데이터의 출처다.

## LiDAR 평면 extrinsic

[`wheel_odometry_evaluation.json`](../../configs/mission/wheel_odometry_evaluation.json)은
[`lidar_extrinsic_20261116.json`](../../ros2_ws/src/cleany_base_driver/config/lidar_extrinsic_20261116.json)을
선택한다. `p_base = R(yaw) p_laser + t`이며 전진 +X는 CAD −Y,
좌측 +Y는 CAD +X다. 평가기의 ICP 초기 추정과 결과 변환에 적용한다.

| 항목 | 적용값 |
|---|---:|
| X | 0.143429742 m |
| Y | −0.001191569 m |
| yaw | 179.997034° |

2026-11-16의 정지 스캔 180.03초를 현재 `Cleany.FCStd`의 여섯 수직 프로파일에
정합한 값이다. 재측정 시 SDK 원시 각도와 거리를 보존하고, 라이다 높이의
실제 홈과 곡면을 포함한 단면을 추출한다. 반복 관측의 각도별 중앙값을
라이다에서 보이는 외곽 면에 정합한 뒤, 위 좌표계로 변환해 새 날짜의 파일에
저장한다. 측정 원본과 검증은 [실행 근거](../EVIDENCE.md#2026-11-16-lidar-평면-extrinsic)에 연결한다.

## IMU factory gyro bias

Jetson은 [`imu_bringup.yaml`](../../ros2_ws/src/cleany_base_driver/config/imu_bringup.yaml)의
고정 gyro bias와 covariance를 사용한다. 부팅 시 자동 보정은 수행하지 않는다.

기본값은 기존 `bench_20261014T142933Z/run06_persistent` 결과다. 기존 값을
재사용할 때는 새 측정 없이 이 설정으로 실행한다. 다음 절차는 계수를 새로
산출하기로 한 경우에만 수행한다.

1. [독립 IMU 진단 절차](../../esp32/imu_calibration/README.md)에 따라 센서를 정지시키고
   새 bench 측정과 검증 결과를 저장한다.
2. 센서 축과 rad/s 단위를 확인하고 YAML의 `gyro_bias_rad_s`,
   `gyro_covariance_rad2_s2`, `calibration_source`를 함께 갱신한다.
3. 모터 runtime 펌웨어를 복원하고 host `imu_bringup.launch.py`를 재시작한다.
4. `imu_uncalibrated/data_raw`, `imu/data_raw`, `imu/status`에서 정지 잔차, 계수 출처와
   covariance를 확인한다. 측정 환경, code commit와 원본 위치를 결과에 남긴다.

측정 결과는 런타임에 자동 적용되지 않는다. 데이터와 filter 계약은
[IMU reference](../../ros2_ws/src/cleany_base_driver/docs/imu.md)를 따른다.
