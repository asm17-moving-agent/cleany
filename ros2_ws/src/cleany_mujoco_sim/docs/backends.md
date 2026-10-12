# MuJoCo backend 계약

- 한 실행에 backend 하나. Joint state·sensor·TF·controller 중복 소유 금지

| Backend | 책임 |
|---|---|
| Custom simulation | base velocity·joint feedback·LiDAR/depth·clock |
| Hand-eye ros2_control | MuJoCo·팔 FollowJointTrajectory·calibration camera |
| RGB-D pick demo | 검출·후보 선택·접근 경로 시각 확인 |

- MuJoCo 결과: 실물 구동·집기·제동거리 검증 아님
- 원본: 패키지 config/ + cleany_description 모델
- CAD geometry: wheelbase 0.350 m / separation 0.6038 m / radius 0.0635 m; 실물 측정값과 구분
- Sim time·capture stamp: backend 소유. 소비자는 같은 use_sim_time 사용
- Hand-eye: [동기화 계약](../../cleany_handeye_calibration/docs/frames-and-data.md)
- MoveIt: [Planning 계약](../../cleany_moveit_config/docs/planning.md)
- 인자: launch/*.launch.py, `ros2 launch ... --show-args`
- Custom/ros2_control 동시 실행 금지
- 안내: [실행](../../../../docs/RUNNING.md), [검증](../../../../docs/TESTING.md)
