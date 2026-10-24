# 공통8 goal preflight 및 잠금

실제18F full facility, CAD frame robot/sensors, 시작 world/map [-1.865,-4.705,1.5708]. 목표 좌표 변경0. 모든 의자를 복구한 원본330models/82chairs에서, 대상 seat goal만 해당 의자1개를 제거했다. 기존 template에서 빠져 있던 office_chair_13/43을 복구했다.

LiDAR frame z=.38-.08=.30m에서 primitive/mesh 충돌 geometry를 절단해 goal별 static map을 생성했다. map/world 좌표계를 그대로 공유한다. footprint/padding.02/inflation.8m는 축소하지 않았다.

5cm raster에서는 M1 doorway의 full-cell SAT footprint 검사와 격자 중심 배치가 경로를 막았다. 원래 목표와 방향을 유지한 채 지도/공통 lattice metadata를2.5cm로 바꿨다. metric primitive/turning radius.5m는 그대로다. planner max_planning_time80s/body_heading_grid_bins32를 공통 고정했다. 실패 진단 자료는 common에 보존됐다.

|index|goal|좌표 [x,y,yaw]|static planner status|path m|body yaw change rad|
|---|---|---|---|---|---|
|0|straight|[-1.865, -3.705, 1.5708]|4|1.000|0.00000|
|1|turn_arrival|[-1.865, -2.8, 0]|4|1.905|1.57080|
|2|chair_gap|[0.575, -1.625, 1.570796]|4|5.391|0.00000|
|3|far_heading|[-1.865, -0.3, -1.570796]|4|4.405|3.14159|
|4|distant|[-5.525, 4.775, -1.570796]|4|12.971|3.14159|
|5|lateral|[-1.265, -4.705, 1.5708]|4|0.600|0.00000|
|6|narrow_seat|[-3.125, -4.475, 1.570796]|4|1.281|0.00000|
|7|other_room|[-16.775, 1.775, 1.570796]|4|23.120|2.74890|

이 결과는 static LiDAR map+full footprint+inflation에서 실제 StraightFirstPlanner가 생성한 경로다. dynamic depth/memory와 controller 주행 성공을 증명하지 않는다. narrow_seat는 heading이 거의 고정된1.28m 경로로, 다중 회전 경로라고 부르지 않는다. other_room는23.12m이며 중간 heading 변경이 있다.

Headless Gazebo scene service에서330models와 asset materialization을 확인했고, missing mesh/unresolved URI가 없었다. 화면/실물 검증은 아니다.

lock SHA256: `296c9abae047f5ba4f27ced4e2d80bd159565d5b370460fee1ce4d29e58022ac`

원격 common: `/home/donglee/workspace/cleany/controller-comparison-8goals-20261014/common`. 최종 manifest204개 파일 해시가 일치한다. 로컬/원격 READY를 전송 검증 후 생성했다. 소비자 공통 입력은 immutable 유지한다.
