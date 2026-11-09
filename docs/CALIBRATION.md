# 캘리브레이션 작업 가이드

## D435와 LiDAR 벽 비교 및 헤드 관절 관측

`tools/validation/head_pan_tilt_probe.py`는 지정된 직렬 버스의 헤드 ID 13과 14를
읽어 `/head/joint_states`와 `base_link → d435_pan_link → d435_tilt_link → d435_link`
동적 TF를 발행한다. 기본 실행은 읽기 전용이다. RealSense 드라이버가 발행하는
`d435_link` 아래 광학 TF는 유지한다. `/joint_states`의 휠 발행자와 합치지 않는다.
피드백 오류나 지연이 있으면 종료하며 마지막 관절각을 새 시각으로 재발행하지 않는다.

1. Humble 환경에서 D435 깊이 영상, CameraInfo, `/scan_raw`가 수신되는지 확인한다.
   NumPy, matplotlib와 pyserial을 사용한다. USB by-id와 모터 ID를 확인하고
   `tools/validation/config/d435_pan_tilt.json`의 버스와 CAD 공칭값을 검토한다.
2. `python3 tools/validation/head_pan_tilt_probe.py --config tools/validation/config/d435_pan_tilt.json`
   으로 실제 관절각을 발행한다. 기존 차체→카메라 고정 TF 발행자와 동시에 쓰지 않는다.
3. 로봇을 같은 평평한 벽을 향해 정지시키고
   `python3 tools/validation/capture_depth_lidar.py --output /absolute/new/run/capture`
   로 깊이 영상, scan, odom과 관절 TF를 수집한다.
4. `python3 tools/validation/compare_depth_lidar_wall.py --capture /absolute/new/run/capture --mount tools/validation/config/d435_cad_mount.json`
   으로 벽 거리, 수평 방향과 수직 법선 잔차를 확인한다. 관절 TF가 기록되어 있으면
   카메라 시각에서 150ms 이내인 실제 TF를 사용하고, 없으면 공칭 자세 비교임을 표시한다.

`--exercise --output /absolute/new/run/exercise`는 실제 모터를 움직이는 명시 시험이다.
다른 버스 소유자를 종료한 뒤, 차체 정지와 헤드 주변 공간을 확인하고 실행한다.
현재 토크 OFF, 모델 777, 위치 모드, 오류 없음과 온도를 먼저 검사한다.
현재 자세 부근에서 축별 ±5°를 저속으로 이동하며 각 자세에서 벽 자료를 수집한다.
EEPROM은 쓰지 않으며 시험 후 토크 OFF를 확인한다. RAM 가속도 5, 속도 80tick/s,
토크 한도 300과 마지막 목표값은 남는다. 복귀 실패 시 추가 이동 없이 토크를 끈다.
`--poses`로 기존 실패 근거를 보존하며 아직 확인하지 않은 자세만 별도 실행할 수 있다.

CAD 원점 기준 카메라 공칭 높이 1.0218m는 지면 높이가 아니다. CAD 바퀴 축 -0.032m와
반지름 0.0635m로 추정한 바닥 기준 약 1.1173m다. 현재 LiDAR의 Z=0은 기존 평면 관례다.
벽 한 면은 높이와 벽에 평행한 이동을 검증하지 못하며, 수직 법선 잔차는 벽 기울기와
장착 오차를 분리하지 못한다. 이 도구는 costmap 입력이나 자동 보정값을 변경하지 않는다.

## bag 기반 슬립 의심 구간 분석

`tools/validation/slip_detection.py`는 휠 odom과 독립 LiDAR 정합의 이동 벡터를
비교한다. 바닥 슬립과 물체에 걸린 상태를 구별하는 확정 진단은 아니며,
오프라인 분석만 수행한다. 제어 명령, TF와 보정값을 발행하거나 수정하지 않는다.

1. Humble과 기록에 사용한 custom message overlay를 source한다. 기존 odometry
   평가 도구의 NumPy, SciPy와 PyYAML 의존성을 사용한다.
2. `/scan_raw`, `/odom`, `/tf_static`이 있는 SQLite bag에서 분석 시간을 지정한다.
   선택적인 `/base/wheel_state`는 명령 및 측정 휠 속도의 보조 통계에 사용한다.

   ```bash
   python3 tools/validation/slip_detection.py \
     --bag /absolute/path/to/bag_directory \
     --start 2026-11-06T07:22:40+09:00 \
     --end 2026-11-06T07:30:25+09:00 \
     --output /absolute/path/to/new_analysis_directory --jobs 2
   ```

3. `summary.json`의 사건 목록과 `windows.jsonl`의 창별 판정을 함께 확인한다.
   `manifest.json`에는 입력 cache, 코드 해시와 설정을 보존한다. 입력 bag은
   읽기 전용 SQLite snapshot으로 열며, 기존 출력 디렉터리는 덮어쓰지 않는다.
   같은 추출 입력을 재분석할 때는 `--bag` 대신 `--cache .../inputs.npz`를 사용한다.

| 항목 | 기본 동작 |
|---|---|
| 비교 시각 | scan header + scan_time/2, 같은 시각의 odom pose 보간 |
| 좌표 | 기록된 planar `base_link → scan frame` static TF, 창 시작의 차체 좌표 |
| 입력 범위 | 0.5–8m, CAD self-mask, 5cm voxel |
| 정합 | 약 1초 간격, 0.5초마다, 휠 초기값 없이 ICP |
| 품질 | 기존 matcher 품질 검사, 정역방향 일관성, 1.5m 밖 점 정합과 비교 |
| 병진 의심 | 휠 이동 ≥2.5cm, 벡터 오차 ≥3cm, 상대 오차 ≥35% |
| 회전 의심 | 휠 회전 ≥0.08rad, 회전 오차 ≥0.07rad, 상대 오차 ≥35% |
| 사건 | 신뢰 가능한 의심 창 2개 이상 연속, 판정 불가 창은 연속을 끊음 |

판정 설정은 `tools/validation/slip_detection.json`에서 관리하고 다른 설정은
`--config`로 전달한다. 다른 로봇 형상은 `--mask-file`로 교체한다.
`unknown`은 정합이나 시간 품질이 부족한 상태이며, `no_evidence`도 슬립이
없다는 보장이 아니다. 아주 느린 슬립과 바퀴가 잠긴 채 끌리는 움직임은 검출
범위 밖이다. per-beam deskew를 하지 않으며 scan 취득 중 움직임과 환경의
가림은 오차를 만들 수 있다. 겹치는 비교 창을 독립 표본이나 정확한 사건
시작 시각으로 해석하지 않는다. 자동 제어 연동 전에는 별도 실시간 검증이 필요하다.

## Jetson 실물 모델과 2026-10-09 보정 프로파일

`cleany_description/config/calibration/jetson_20261009/`는 Jetson에서 사용한
헤드와 양손목 카메라 외부 보정, 해당 RGB 내부 보정, 팔과 헤드의 관절 기준을
보존한다. 카메라 외부 보정은 각각 20개 학습 뷰와 별도 5개 검증 뷰로 평가한
`provisional_20plus5_manual_stationary` 결과다.

| 변환 | 적용 내용 |
|---|---|
| `head_camera_rgb_frame` → `head_camera_rgb_optical_frame` | 헤드 RGB 외부 보정 |
| `left_gripper_frame` → `left_wrist_camera_optical_frame` | 왼손목 외부 보정 |
| `right_gripper_frame` → `right_wrist_camera_optical_frame` | 오른손목 외부 보정 |
| 각 `*_wrist_camera_optical_frame` → `*_wrist_rgb_optical_frame` | 항등 변환 alias |

`xyz`는 parent frame 기준 m, `rpy`는 고정축 XYZ의 rad이며,
`parent_T_child`는 child 좌표를 parent 좌표로 변환한다. 보정은 광학 좌표계에
적용하며 카메라 케이스 mesh의 CAD 배치를 옮기지 않는다. 헤드 depth frame은
명목값을 유지한다.

v8 손목 마운트 형상 반영 시 Jetson의 양손목 외부 보정 파일은 이전 수집본과
바이트 단위로 같았다. 형상에서 추정한 광학 좌표로 이 보정값을 대체하지 않는다.
이번 변경은 새로운 hand-eye 보정이나 v8 장착 상태에서의 정확도 검증을 포함하지
않는다. 실제 카메라 장착 위치가 보정 당시와 달라졌다면 재검증이 필요하다.

빌드한 workspace의 `setup.bash`를 읽은 뒤 모델을 실행한다.

```bash
ros2 launch cleany_description description.launch.py \
  use_sim_time:=false calibration_profile:=jetson_20261009
```

이 launch는 `robot_state_publisher`만 실행한다. 관절 상태는 별도 `/joint_states`
입력이 필요하며, 저장된 encoder 영점으로 tick을 변환하거나 모터를 제어하지 않는다.
`calibration_profile`의 기본값은 빈 문자열이다. 기본 실행은 명목 헤드 TF를
유지하고 손목 보정 optical frame을 추가하지 않는다.

노드를 실행하지 않고 보정된 URDF만 확인하려면 다음 명령을 사용한다.

```bash
xacro "$(ros2 pkg prefix --share cleany_description)/urdf/cleany.urdf.xacro" \
  calibration_profile:=jetson_20261009 > /tmp/cleany-calibrated.urdf
```

`arm_joint_calibration.yaml`에는 좌우 12개 모터의 현재 영점, 방향과 rad 오프셋을
담았다. `q = direction × (Present_Position - zero_tick) × 2π/4096 + offset_rad`로
해석한다. Present_Position에 이미 반영된 EEPROM homing offset을 다시 더하지 않는다.
`motors/`의 원본 JSON과 `reference/`의 과거 자세 기록은 측정 근거이며 자동 적용하지
않는다. 현재 매핑은 `arm_joint_calibration.yaml`이 기준이다. 오른쪽 elbow 영점처럼
과거 자세 기록 이후 보완된 값이 있다. 절대 관절 각도 정확도는 검증되지 않았다.

`head_joint_calibration.json`의 pan/tilt 기준 tick은 각각 2009와 2067,
방향은 -1과 +1이다. 이는 수동 영점과 회전 방향 확인이며 회전축 추정 결과가 아니다.
내부 보정 JSON은 외부 보정 입력을 보존하기 위한 자료로, 이 launch에서
`CameraInfo`를 발행하거나 카메라 드라이버 설정을 바꾸지 않는다.

| 카메라 | 검증 보드 위치 RMS | 검증 보드 회전 RMS |
|---|---:|---:|
| 헤드 RGB | 1.246 mm | 0.247° |
| 왼손목 | 8.525 mm | 1.540° |
| 오른손목 | 8.895 mm | 1.739° |

이 수치는 고정 보드 가정에 대한 일관성이다. 절대 위치 ground truth와 하드웨어
노출 동기화는 검증되지 않았다. 오른손목 학습 뷰 3개에는 manual torque hold
기록이 없으며, k3=0 모델은 held-out 평가 전에 선택했다. 각 YAML의 원본 검증
조건과 `absolute_calibration_verified: false`를 유지한다.
원본 보고서 경로와 SHA-256은 프로파일의 `provenance.json`에 기록하며,
절대 경로는 과거 근거 위치로만 사용한다.

## Hand-eye 수집과 평가

Calibration core, ROS 수집 adapter와 오프라인 평가는
[패키지 안내](../ros2_ws/src/cleany_handeye_calibration/docs/offline-tools.md)를 따른다.
좌표계와 저장 계약은 [reference](../ros2_ws/src/cleany_handeye_calibration/docs/frames-and-data.md)에서 확인한다.
결과 transform은 검토용으로 저장하며 TF에 자동 적용하지 않는다.

## ToF 근접 보호 commissioning

[ToF 설정](../ros2_ws/src/cleany_tof/config/tof.yaml)은 실측하지 않은 후보값이다.
모터를 분리하거나 바퀴를 띄운 상태에서 다음 항목을 확인한 뒤 별도 로봇 프로파일을 만든다.

- 각 native zone에 작은 표적을 움직여 flip, 장착 roll/pitch/yaw, 위치와 유효 FOV를 측정한다.
- 알려진 거리와 기울기의 넓은 평판으로 radial range, 거리 오차, sigma, fit residual,
  접근 거리와 yaw 오차를 확인한다. 선택 행이 책상 앞면을 실제로 관측해야 한다.
  약 3.5m 미검출 관찰은 검출 보장 거리나 자유 공간 하한으로 사용하지 않는다.
- 빈 공간, 검정 표면, 유리, 금속, 햇빛, 책상 다리와 얇은 물체에서 zone 유효율과 사각을
  기록한다. NO_RETURN과 불신뢰 분류, ROI의 보호 높이, 얇은 물체와 다른 센서도 놓치는 경우를 검증한다.
- USB 부하, read 지연, 끊김과 재연결에서 acquisition 추정, 실제 정지 지연과 watchdog을
  측정한다. first-frame 시간 offset의 불확실성도 reaction budget에 포함한다.
- 감독자와 독립 e-stop이 있는 통제 공간에서 payload, 바닥, 배터리 조건별 전후진,
  횡이동, 회전의 최악 제동 거리와 감속도 하한을 측정한다. `deceleration`,
  `reaction_s`, footprint와 margin을 이에 맞추며 software 정지와 실제 정지를 구분한다.
- 저속 접근, 비스듬한 평판, 전면 가림, Nav2 동시 goal, cancel, Mission/Supervisor/가드
  재시작, lease 회수와 완료 후 정지 유지가 실제 구동에서 성립하는지 확인한다.
- 정지 상태에서 ROI와 검출 점을 기록하고 바닥/몸체 return을 구분한다. 15Hz 기준 인접
  검출 수, cluster 거리, confirm_s, release_s와 거리 hysteresis를 실제 물체와 순간 노이즈로
  튜닝한다. 비상 단일 return과 일반 검출의 지연/오탐/미탐을 각각 측정한다.
- 책상 ROI의 분산된 유효 column/row, 최소 폭, residual를 확인한다. 일부 품질 불량에서
  평면을 유지하는 경우와 평면 소실 시 zero/abort를 구분하고 접근 거리/yaw 오차를 측정한다.

위 근거와 사용한 설정을 보존한 뒤에만 `calibrated=true`를 적용한다. 평면의 좌우 중심은
ToF만으로 관측되지 않으므로 staging 위치와 팔 도달성은 별도 검증한다.


## D435 미리보기의 LiDAR 높이 하한

D435 점군은 촬영 시각의 실제 head encoder TF로 `base_link`에 변환한다.
현재 실험의 하한은 `min_base_z=0.192`m로, CAD LiDAR 하우징 바닥 z=0.150m와
레이저 중심 오프셋 0.042m의 합이다. CAD 바닥 z=-0.0955m 기준 지상 0.2875m다.
이 높이보다 낮은 점은 모두 제외하므로 낮은 물체도 장애물 후보에 남지 않는다.
카메라 tilt가 변해도 하한은 로봇 좌표에 고정된다. 바닥 평면 추정은 이 하한을 바꾸지 않는다.
측정 바닥면은 표시 grid와 상한 기준으로만 남긴다. 이 값은 CAD 기반이며 실측 교정 완료를 뜻하지 않는다.
