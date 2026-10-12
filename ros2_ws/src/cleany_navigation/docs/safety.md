# Navigation 안전 계약


```text
Nav2 -> nav2/cmd_vel -> pan gate -> pan_ready/cmd_vel
     -> Collision Monitor -> safety_2d/cmd_vel -> voxel guard -> cmd_vel
```

모든 속도는 `geometry_msgs/Twist`의 x/y/yaw다. 최종 `cmd_vel` publisher는
voxel guard 한 개다. Backend는 아래 입력/TF를 제공한다.

| 입력 | 계약 |
| --- | --- |
| `scan` | LaserScan, 실제 LiDAR frame과 ROS stamp |
| `odom` | Odometry, `child_frame_id=base_link`, 실제 측정 x/y/yaw 속도 |
| `joint_states` | head_pan_joint의 position과 velocity, 유효한 stamp |
| `camera/head/depth/image_raw` | Rectified depth, 32FC1 m 또는 16UC1 mm; 0/NaN/-Inf는 관측 없음 |
| `camera/head/depth/camera_info` | Image와 동일 크기/optical frame, 유효한 K 또는 monocular P, D=0 |
| TF | Backend: odom→base_link와 base→LiDAR/head/optical; AMCL: map→odom |

Depth projection은 optical Z-depth를 원래 image stamp의 TF로 output frame에
변환한다. TF·calibration이 없으면 cloud를 발행하지 않는다. Park posture의 팔과 고정 몸체·카메라 기둥 형상으로
센서 원점부터 hit까지의 선분이 몸체를 통과하는 ray를 제거한다. 레일 뒤로 튀는
depth 경계 픽셀도 제거하며, 몸체에 가리지 않은 외부 장애물은 유지한다.
자기 필터는 각 OBB의 세 축에 대해 닫힌 선분 교차 구간을 누적해 계산한다.
박스마다 큰 임시 배열을 만들고 축약하던 비용을 줄였으며, 평행 ray,
표면 접촉, 센서가 박스 내부인 경우의 보수적인 제거 기준은 유지한다.
회전하는 head 부품은 정적 자기 필터에 포함하지 않으며,
몸체에 가린 ray를 free-space로 바꾸지 않는다. `depth_clearance.self_filter_margin_m`의 기본 1mm
여유는 depth/CAD 표면의 수치 오차를 처리한다. 실제 로봇에서는 보정 오차에
맞게 검증해야 한다. 공통 pan gate는
`head_pan/command` Float64를 발행하고 TF를 소유하지 않는다. Backend의 head
actuator adapter와 실제 joint feedback이 필요하다. 공통 노드의 topic/frame은
ROS parameter로 설정 가능하며 TF edge owner는 중복되면 안 된다.
`pan_min_position_rad`/`pan_max_position_rad`는 position actuator에 보낼 수 있는
관절 위치 범위다. 기본값은 기존 명령 범위인 [-π, π]이며 실제 하드웨어의
보정된 범위를 증명하지 않는다. Backend의 검증된 한계를 명시해야 한다.
목표 방위각과 정확히 같은 방향의 위치(2π의 정수 배 차이) 중 실제 pan에서
가장 가까운 범위 내 값을 명령한다. +π/−π 경계의 불필요한 한 바퀴 회전을
방지하되, 기계적 한계를 넘는 짧은 경로는 선택하지 않는다. 범위 안에 목표
방위각을 나타낼 수 없으면 `PAN_TARGET_OUT_OF_RANGE`로 속도를 차단한다.
정렬 tolerance와 fresh depth 조건은 동일하며 목표각을 clipping하지 않는다.

기본 주행은 전진·후진과 제자리 회전이다. 횡이동은 기본 차단한다.
PoseProgressChecker는 XY 0.15m 이동 또는 yaw 0.25rad 회전을 진행으로 인정하며,
20초 동안 둘 다 없으면 실패로 처리한다.
Pan은 자유 이동 모드에서 방향 정렬과 정렬 후 fresh depth를 확인한다. 축 제한 모드에서는 정지·settle도 확인한다. 후진 시 카메라를
뒤쪽(π rad)으로 돌리고 정지 후 새 depth를 받아야 속도를 통과시킨다.
뒤쪽 시야가 확보되었다는 물리 검증은 별도로 필요하며, 후진을 제외하려면
`pan_motion_gate.allow_reverse: false`로 설정한다. LiDAR/depth를 Collision
Monitor와 장애물 메모리에 전달하고, 3D voxel guard는 몸체 OBB와 예측 제동
sweep를 검사한다. 필수 입력 단절, stale/future stamp, TF 누락, localization
jump, voxel freeze 및 계산 시간 초과 시 최종 속도를 0으로 만든다. Pan/voxel의
steady timer는 `/clock`이 멈춘 상황에서도 정지 출력을 유지한다.
Swept-volume 검사는 각 예상 자세에서 `BodyBox`를 다시 생성·검증하지 않고
검증된 몸체의 변환과 SAT 축을 계산한다. 회전/병진 샘플 간격, voxel 크기,
몸체 padding과 제동 horizon은 동일하며, 측정 속도와 요청 속도가 정확히 같을 때만
동일한 제동 sweep의 중복 계산을 생략한다. 속도가 0이어도 관성으로 움직이는
측정 속도에 대한 검사는 유지한다.

몸체 guard는 **설정된 park posture**를 가정한다. 실시간 팔 FK 감시나 팔 자세
유지는 이 패키지에 구현되지 않았다. 관측한 voxel만 검사하며 unknown 공간을
free로 인증하지 않는다. 물리 E-stop, 드라이버 watchdog와 실측 제동 검증은
로봇 통합 작업에서 별도로 수행해야 한다.

Global/local costmap 모두 `obstacle_memory/grid`를 max 병합하여 planner와
controller가 동일한 누적 장애물을 검사한다.

`obstacle_memory/grid`, `obstacle_memory/voxels`, `obstacle_memory/voxel_status`,
`body_guard_3d/state`, `head_pan/state`를 진단에 사용한다. Memory의
`obstacle_memory/save`, `obstacle_memory/set_enabled` 서비스를 유지한다.
`output_directory`는 기본 빈 문자열이다. 지정했을 때만 memory/guard 로그와
스냅샷을 각각 하위 디렉터리에 기록한다. 디스크 기록 없이도 runtime이 동작한다.

`amcl_nav2_safety.launch.py`는 위 full runtime의 alias다. 안전 기능을 제외한
기본 Nav2 실험은 `amcl_nav2.launch.py map:=...`로 명시적으로 실행한다.
Full runtime의 `pan_enabled`, `memory_enabled`, `voxel_enabled`를 끌 수 있으나
voxel을 사용하려면 memory가 필요하다. 비활성화한 단계에 따라 속도 경로를
다시 구성하며, 기본값은 모든 단계 활성화다.


## 설정과 검증

설정은 이 패키지의 `config/`, 실행은 [RUNNING](../../../../docs/RUNNING.md),
검증과 결과 해석은 [TESTING](../../../../docs/TESTING.md)를 따른다.
과거 주행 점수와 진단 과정은 [EVIDENCE](../../../../docs/EVIDENCE.md)의 원본 보관본에서 확인한다.
