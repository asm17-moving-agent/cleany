# mission configs

Mission Manager, FSM, planner, retry, timeout 관련 설정을 둔다.

## 휠 odom 평가

`wheel_odometry_evaluation.json`은 실물 휠 odom과 LiDAR ICP를 비교하는 반복 실험
설정이다. 기본값은 ±X/±Y와 두 대각선 각각 편도 총거리 0.5 m, ±90° 회전,
저속·중속·고속 각 3회다. 전체 45회 왕복, 90개 편도를 평가한다.
현재 시험용 P는 X/Y/yaw와 대각선 모두 6.0이다. 거리·각도 정지를 위한
일시적인 평가 제어이며, 휠 odom 보정 계수나 상시 주행 제어 설정이 아니다.

- `translation_distance_m`: 병진 편도 거리. 1 m 또는 2 m 평가에는 `1.0` 또는 `2.0`.
- `linear_speeds_mps`, `angular_speeds_rad_s`: 병진 속도와 회전 속도 목록.
- `repetitions`: 전체 속도·패턴 묶음의 반복 횟수.
- `patterns`: `surge`, `sway`, `rotation`, `diagonal_left`, `diagonal_right`.
  대각선 거리·속도는 X/Y 성분이 아닌 벡터 크기.
- `odom_topic`: 기본 `/wheel/odom`. LiDAR ICP는 비교 기준으로만 사용.
- `extrinsic_config`, `motion_config`: 센서 평면 변환과 주행 제어 설정 파일.
- `runtime_config`: Agent·base driver·LiDAR 자동 기동 설정. USB 경로와 시작 대기
  시간은 `configs/robot/wheel_evaluation_runtime.json`에서 관리.
- `lidar_min_range_m`: 평가용 ICP의 근거리 차체 반사점 제거 거리.
- `ros_domain_id`, `odom_timeout_s`, `plots`, `output_dir`: 실행 domain, odom
  freshness 제한, 그림 생성 여부와 결과 저장 경로.

파일 경로는 config 파일 위치나 실행 디렉터리가 아니라 **레포 루트 기준**이다.
실행과 검증은 [오도메트리 평가 README](../../tools/odometry_evaluation/README.md#속도별-3회-휠-odom-재측정)를 따른다.

`all_axis_validation.json`은 X/Y/yaw와 두 대각선의 P=6 검증 preset이며,
`run_motion_response.sh`의 기본 설정이다. `trials`에서 각 축·속도별 P와
속도 상한을 편집한다. 전체 45회 왕복, 90개 편도이며 기본적으로 ICP 거부 시
중단한다. 거리·반복·중단 경계와 설정 우선순위는
[전 축 검증 프로토콜](../../tools/odometry_evaluation/VALIDATION_PROTOCOL.md)을 따른다.
현재 `icp_min_inlier_ratio: 0.30`으로 ICP inlier 하한을 30%로 설정한다.
`icp_convergence`에서 병진·회전 수렴 공차와 bounded cycle의 폭·주기를 조정한다.
해당 P 변경은 trial에만 적용하며 기본 motion 파일에 되쓰지 않는다.

`motion_response_tuning.json`은 실제 X/Y/yaw P 응답 후보 시험 설정이다.
후보 sweep에는 `run_motion_response.sh --config configs/mission/motion_response_tuning.json`을
명시한다.
`trials`의 `axis`, `kp`, `speed`와 `repetitions`를 편집하며,
`guards`에서 시험용 경계 이탈 중단값을 지정한다. 기본적으로 미수렴 ICP도
중단 사유다. 제어 응답만 측정할 때 `require_valid_icp: false`를 명시하면
ICP 결과의 무효 표시를 유지하면서 정상 주행 trace만 검증할 수 있다.
적용할 계수는 `tools/odometry_evaluation/motion_settings.json`에서 관리한다.

`diagonal_response_tuning.json`은 대각선 ±45°에서 fallback `linear_kp`의
1·3·4·6 후보를 비교한다. `diagonal_response_validation.json`은 선택한 P=6을
속도상한 0.20/0.35/0.50 m/s에서 각각 3회 왕복 검증한다.
두 config 모두 편도 총거리 0.5 m이며, `require_valid_icp: false`로
제어 응답과 유효 ICP 외부 오차를 분리해 기록한다.
