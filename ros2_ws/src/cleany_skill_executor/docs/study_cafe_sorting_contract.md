# 스터디카페 분류·운반 실행 계약

[패키지 안내로 돌아가기](../README.md)

## 분류·운반 실행 계약

`study_cafe_sorting.launch.py`와 일반 파이프라인은 `robot_top_bins.yaml`의
내부 후면 받침판·수거함을 기본 배치로 사용한다. 책상 양끝 분류 영역과 구분선은
없으며 수거함의 고정 투하 위치만 사용한다.
개별 물체 배치 성공 기록이 있지만 전체 정리 성공은 검증 중이다.
`table_sorting_policy.yaml`의 destination으로 Gemini의 category/reason을
연결한다. label allowlist로 category를 대체하지 않으며 위험 label은 보수적으로
거부한다. RGB-D 거리순으로 시도하고 인식된 수거 구역 물체는 다시 집지 않는다.
미처리/미확인 물체가 남으면 성공으로 보고하지 않는다. 관측한 작업 물체 수와
완료 항목 수를 대조하므로 이전에 관측한 수량보다 완료 수가 적으면 중단한다.
이는 수량 검사이며 객체 identity 추적이나 관찰 모드의 독립 안착 검증을 대신하지 않는다. 빈 작업 영역은 두 번
재확인하며 최대 search/action 횟수 12에 도달하면 실패한다.

MuJoCo oracle은 배치 후 전체 AABB가 수거함 안에 들어가 정지했는지 검증한다.
기본 `sorting_exit_on_finish=true`; coordinator 종료 시 launch가 다른 노드도
종료한다. GUI를 유지하려면 launch 인수 `shutdown_on_sorting_exit:=false`를 사용한다.
현재 전체 물리 정리 성공은 아직 검증 중이다. 기록: `artifacts/table_sorting_20260908/`.
sorting은 얇고 긴 객체의 RGB-D 점군에서 중심 외의 longitudinal contact 후보도
생성한다. 자세/충돌 한계는 유지하며, 세부 범위는 cleany_grasping README를 따른다.
sorting selector/executor는 `support_patch_margin_m=0.02`로 perception OBB
바닥의 지지면을 유한 collision patch로 유지한다(일반 node 기본 0, 비활성).
patch는 OBB XY 범위에 각 방향 2cm를 더하고, OBB 바닥 아래 1cm 두께로 만든다.
2cm는 현재 target mask padding 1.5cm를 포함하는 로컬 범위다. 8cm 확장은
가까운 물체에서 로봇 본체까지 가상의 면을 확장하는 문제가 있어 사용하지 않는다.
이는 perception이 지지면 법선을 OBB Z축으로, 지지면을 바닥으로 쓰는 계약에
의존한다. 수평에서 20°를 넘는 면/잘못된 pose는 거부한다. 새 테이블 정답은
읽지 않으며, 관측 밖 연장 부분은 보수적인 로컬 평면 가정이다.
물체/support pair만 접촉 허용하고 robot/support는 허용하지 않는다. attachment
이후에도 patch가 남아 target OctoMap masking 아래의 지지면이 사라지지 않으며,
transaction 종료 시 target과 함께 제거하고 원래 ACM으로 복원한다.
`require_gripper_closure_clearance=true`는 실제 full-robot 형상으로 open→close
구간을 `gripper_sweep_step_rad=0.05` 이하 간격에서 검사한다. target 접촉만
허용하고 환경/지지면 충돌은 거부한다. 이 검사는 기하학적 샘플 검사이며 접촉
동역학/실제 파지 유지 성공을 보장하지 않는다. grasping의 support-plane 간이
검사 위임은 이 support patch 및 open/closure 검사와 함께만 사용한다.
seeded cubic 경로가 충돌로 거부되면 최대 8개 contact의 body pair, 위치/frame,
penetration depth를 오류에 기록해 주변 물체/지지면/잔상 원인 분석에 사용한다.
`sorting_head_reference_refresh_age_sec` 기본값은 `0.0`으로, pregrasp 도착 후
시간 경과만을 이유로 하는 Gemini 재인식을 비활성화한다. `(0, 30]`초로 설정하면
기존 ROS clock 기준 만료 검사와 재인식·위치 연관·동일 팔 재계획을 다시 활성화한다.
손목 서비스 자체의 관측 유효성 검사는 유지하며 관측 timestamp를 갱신하지 않는다.

고정 수거함 모드에서는 YOLOE-seg bbox/depth 거리로 정렬한 후보를 하나씩 처리한다.
`fixed_jaw_clearance_m`은 고정 손가락 여유거리(수거 기본 0.003 m),
`grasp_opening_margin_m`은 후보 벌림 여유폭(기본 0.008 m)이다. 여유거리는
여유폭의 절반 이하여야 한다. 5 mm 시험은 각각 `0.005`, `0.010`으로 설정한다.
벌림 여유폭은 후보 생성기와 두 TCP 보정 경로에 동일하게 전달된다.
현재 후보의 YOLOE-seg mask에만 3D 복원을 수행하고 파지 가능한 팔을 찾으면 탐색을 끝낸다.
복원·파지 계획·양팔 도달성 검사가 실패하면 다음 후보를 복원하며, 같은 주기의
복원 결과는 캐시한다. review 분류 후보는 정밀 복원 없이 미해결 대상으로 남긴다.
선택되지 않은 물체도 전체 depth 기반 OctoMap의 충돌 검사에는 계속 포함된다.
다음 후보 처리 전 snapshot TTL이 만료되면 기존 cache 오류로 처리되며 과거 영상의
시각을 갱신하지 않는다. 미해결 물체와 기존 관측 물체의 누락 검사는 유지한다.
sorting selector의 `pose_refinement_iterations=30`은 runtime `/robot_description`
URDF의 로컬 FK로 위치와 접근/closing 방향의 잔차를 함께 줄이는 bounded numerical
refinement를 사용한다. 매 후보 탐색 전 MoveIt FK와 일치하는지 대조한다.
초기 자세 예산은 pregrasp에서 `pregrasp_aim_attempts`, grasp에서 0이 아닌
`grasp_pose_seed_attempts`를 따른다(0이면 pregrasp 예산 재사용). 수치 보정도
설정 예산 전체를 사용하며, 첫 유효 해에서 종료한다. seed 수 증가는 실패
후보의 계획 시간을 늘릴 수 있으나 유효성 허용값을 변경하지 않는다.
기본 adapter 값 0은 기존 탐색이다. 각도 오차 기준, 관절 여유, 충돌검사 및
실행 전 경로 검증은 유지한다. 후보 보정 자체는 로봇에 명령을 보내지 않는다.
실제 grasp는 `grasp_position_tolerance_m=0.0015`로 별도 검사한다. pregrasp의
20mm 허용값을 grasp에도 공유하던 오류를 수정했으며, 관절 자세가 맞더라도
TCP 위치 오차가 1.5mm를 넘는 후보는 거부한다. `pose_refinement_position_weight=10`
으로 위치 잔차를 우선 줄인다. 각도/충돌/관절 한계는 완화하지 않는다.

`grasp_use_aperture_centering=true`는 검출 폭에서 opening margin 8mm를 제외한
물체 폭을 사용해 `폭/2 - fixed_jaw_inner_x(8mm)`로 TCP 옆방향 보정량을 정한다.
계획과 실행이 같은 함수를 사용한다. 기존 30mm 고정 보정은 작은 물체가 두
손가락 사이에서 벗어나는 문제가 있었다. 시뮬레이션 jaw mesh 기준으로 닫힘
근방 aperture 기울기 0.065m/rad, command opening reduction 18mm를 사용한다.
관절 정지로 파지를 확인하지 못하면 `gripper_contact_retry_steps`(기본 node 0,
sorting launch 5; 허용 0–5)만큼 `gripper_contact_retry_step_rad=0.10`씩 더 닫는다.
각 재시도는 `gripper_contact_retry_motion_sec=1.2`이며 닫힘 하한과 기존 토크 제한을
넘지 않는다. 최초 열린 위치 기준의 접촉 판정을 유지하고, 실제 마지막 명령을
운반 중 감시에도 사용한다. 한도 내에서 파지를 확인하지 못하면 lift 전에 실패한다.
이는 실제 하드웨어 캘리브레이션 값이 아니며 물리 파지 성공은 별도 검사한다.

연속 sorting에서는 `gripper_force_full_close=true`로 기존 닫힘 하한을 목표로
유지한다. 접촉 때문에 실제 관절은 그 전에 멈출 수 있으며, 기존 토크 제한과
관절 잔차/속도 기반 접촉 감시는 그대로 적용한다. 추정 폭의 중간 목표에서 멈춰
물체 자세 변화 후 추가로 조여주지 못하는 현상을 피하기 위한 설정이다.
목표까지 완전히 닫혀 잔차가 사라지면 파지 성공으로 간주하지 않는다.
초기 파지에는 정지 속도 조건을 적용한다. 이미 확인된 파지의 비동기 운반 중에는
닫히는 방향의 추가 조임을 허용하되, 목표 대비 최소 잔차/열리는 속도 상한,
관절 피드백 신선도, 손목 추적과 접촉 상실 debounce는 계속 검사한다.
이는 힘 센서 확인이 아니며 잔차가 사라지기 전의 미끄러짐을 완전히 판별하지 못한다.
sorting의 attachment 후 지도 갱신 대기 상한은 10초다. CPU 저속 시뮬레이션에서
새 지도 발행을 기다릴 시간을 확보하되, 신선도 2초와 attachment 이후 새 depth
capture 조건은 완화하지 않는다. 새 데이터가 준비되면 즉시 진행한다.
변화 없는 OctoMap 전체 메시지가 재발행되지 않아도, tree 쓰기 뒤 발행되는
processed-cloud receipt의 새로운 capture stamp로 지도 활동을 확인한다.
기존 populated-map 확인과 capture/receipt 신선도 기준은 유지하고, 원본 cloud
수신이나 중복 receipt만으로 갱신하지 않는다. 빈 지도도 통과시키지 않는다.
sorting selector는 `require_open_grasp_clearance=true`로 실제 열림 각도의 grasp
끝점도 검사한다. 센서 OBB에 대한 jaw 접촉 허용을 잠시 해제하고, 선택된 gripper
관절만 열림 각도로 바꾼 전체 로봇 MoveIt validity를 확인한다. 기존 피드백/계획은
변경하지 않고 ACM은 성공/실패/예외 모두 복구한다. 이 검사는 시작/중간 경로의
추가 안전 검증을 대체하지 않으며, 보수적인 OBB 때문에 유효한 후보도 거절할 수 있다.
고정 jaw에는 sorting에서 `grasp_fixed_jaw_clearance_m=0.003`을 적용해 경계를
맞닿게 두지 않는다. selector와 coordinator에 동일하게 적용하며, 설정 여유는
opening margin의 절반 이하여야 한다. 기존 비-sorting 기본값은 0이다.
`direct_vertical_lift:=true`는 기본 false인 동작 진단 옵션이다. 접근 경로를
역으로 물러나는 단계 대신 현재 TCP 위치에서 수직 lift를 계획한다.
`direct_vertical_lift_extra_m`(기본 0, 허용 범위 0~0.05m)은 이 진단 경로의
TCP 상승 거리에만 추가된다. 물체 중심의 기존 최소 상승 높이 검사와 경로
충돌 검사는 그대로 적용되며, 높이가 부족하면 예측/요구 중심 높이를 오류에 남긴다.
`sorting_release_edge_margin_m`(기본 0.005m)은 수거함 입구 안쪽의 추가
가장자리 여유를 조절하는 진단용 launch 인자다. 물체의 관측 bounding sphere가
실제 입구 안에 들어가는 검사와 MoveIt 충돌 검사는 그대로 유지된다.
sorting에서는 끝점 position IK/FK를 확인하고 기존 seeded 위치 corridor로 계획한다.
기존 corridor 자세 한도(5°)를 넘는 끝점은 거절하고, 전체 경로의 FK/충돌/자세
변화를 검사한다. 정확한 자세 고정 LIN으로 표현하지 않는다. 파지/손목 감시는
유지하며 기본 전체 sorting 동작은 아직 역방향 retreat를 유지한다.

- [System Context](../../../../docs/cleany-docs/20_TECHNICAL/01%20-%20System%20Context.md)
- [Task Planning and Robot Capabilities](../../../../docs/cleany-docs/20_TECHNICAL/03%20-%20Task%20Planning%20and%20Robot%20Capabilities.md)
## 수거함 운반

### 분류된 수거함으로 직접 이동 / 손목 유지 (기본 모드)

현재 수거 시뮬레이션은 공유 `robot_top_bins.yaml`의
`simulation_ignore_mast_collision: true`로 고정 기둥 접촉을 MuJoCo에서 끄고
MoveIt의 기둥–로봇 링크 충돌을 제외한다. 외형 및 self-filter용 TF/형상은 유지한다.
이 편의 설정의 성공 결과는 실제 기둥을 피하는 실로봇 경로 검증이 아니다.

빈손 복귀와 그리퍼 닫기는 계속 동시에 실행하되,
`sorting_return_velocity_scaling: 0.12`, `sorting_return_acceleration_scaling: 0.15`로
복귀 동작만 별도 상한을 적용한다(더 낮은 전역 설정은 유지).
현 MJCF STS3215의 토크 한계 2.648Nm 및 감쇠 합 3.342Nm·s/rad에서는
무부하 정상 속도조차 약 0.78rad/s 이하라 기존 30% 어깨 yaw 상한 1.274rad/s를
지속 추종할 수 없다. 새 복귀 yaw 상한은 약 0.509rad/s이며 충돌 검사,
모터 토크 한계 및 0.12rad 궤적 추종 허용치는 변경하지 않는다.
이는 현재 시뮬레이터 모델용 설정이며 실제 모터 성능 인증값이 아니다.

운반은 고정 투하 방식이다. `config/nearest_pregrasp.yaml`의
고정 **물체 중심** 위치(base_link, m)는 다음과 같다. 수거함은 로봇 내부 후면에 부착된다.

- `sorting_fixed_release_lost_items_left_m`: `[-0.075, 0.105, 0.50]`
- `sorting_fixed_release_trash_right_m`: `[-0.075, -0.105, 0.50]`

분류 → 파지 → 안전한 들어 올리기 → 선택된 수거함 위 고정 투하점 → 놓기 → 복귀 순서다.
기본 모드에서는 파지 전 공통 경유점 IK 검사, 공동 손목 endpoint 검사 및 경유점 이동을
모두 생략한다. 물체 크기/그리퍼 안의 실제 추정 offset을 반영한 최종 투하 자세만 구한다.
도달 가능한 자세가 없으면 물체를 놓지 않고 중단한다. 직접 이동도 MoveIt의 현재
부착 물체/장애물 충돌 검사를 거친 경로로 실행하며 무검증 직선 명령이 아니다.

들어 올리기를 마치면 실제 손목 roll 각도를 캡처한다. 최종 투하 IK는 손목 roll만 변수에서
제외하고 어깨 yaw/pitch·팔꿈치·손목 pitch의 네 관절을 푼다. 손목을 위아래로 꺾는
pitch는 고정하지 않는다. 실행 경로의 roll tracking 허용
오차는 `sorting_fixed_release_wrist_tolerance_deg`(기본 0.5도)이며 자동 확대하지 않는다.
고정 투하점에서 물체 중심·손목 feedback·함 입구를 확인한 뒤에만 그리퍼를 연다.
물체 중심 도착 허용 오차는 1cm이고, IK의 위치 허용 오차는 축별 0.5mm다.

검증된 최종 자세는 프로세스 내 캐시에 저장하며 동일 팔/손목 각도/물체 offset/투하점일 때
재사용한다. 캐시 hit도 현재 부착 물체를 포함한 충돌 및 FK를 다시 확인한다.
센서 feedback이 달라 정확히 같은 키가 아니면 재계산하므로 캐시 hit나 시간 단축을
항상 보장하지 않는다. 직접 운반 중에는 넓은 영역 탐색이나 손목 완화로 우회하지 않는다.
이 설정은 시뮬레이션용이며 실제 로봇의 고정 투하 자세가 검증됐다는 뜻은 아니다.

### 파지 후 후퇴/들어 올리기의 손목 제약

수거 중 손목 roll만 안정된 파지 접촉 시점의 실제 관절각을 기준으로 제한한다.
pitch는 위아래 동작을 위해 자유롭게 사용하되 기존 물리 관절 한계/충돌 검사는 유지한다.
`config/nearest_pregrasp.yaml`의 ROS parameter `sorting_carry_wrist_tolerances_deg`
기본값은 `[2.0, 5.0, 10.0, 20.0, 30.0]`이다. 파지 후 후퇴/수직 lift는 빈 팔의
기존 pregrasp 관절 목표를 재사용하지 않고, 현재 허용 범위에서 새 endpoint IK와
전체 Cartesian 경로를 계획한다. endpoint나 검증된 경로를 찾지 못할 때 다음 범위로
넓힌다.
기준각은 구간마다 바꾸지 않으며, 한 번 확대한 범위는 놓기까지 유지한다.
관절 한계/FK/충돌 검사는 유지한다. 최대 범위에서도 실패하면 중단한다.

MoveIt 이동에는 같은 관절 path constraint를 전달하고, 직접 생성하는 Cartesian
궤적은 local IK/refinement 경계부터 손목 제약을 반영하고 실행 전 모든 waypoint를
검사한다. 계획/검증과 실행을 분리해 계획 실패만 재시도하며, 서비스/URDF 오류나
제어기 실패/접촉 상실 후에는 제한을 완화해 재실행하지 않는다. 일반 MoveIt 운반
구간은 여전히 IK 성공 후 경로 계획 실패 시 중단한다.

손목 유지 시 어깨/팔꿈치 움직임으로 그리퍼의 세계좌표 방향은 변할 수 있다.
후퇴/lift의 새 도착 방향은 기존 빈 팔 방향을 강제하지 않고 구간 시작 방향 대비
`sorting_carry_cartesian_rotation_limit_deg`(기본 30도)로 제한한다. 이 값은 시뮬레이션
운용 설정이며 실제 로봇의 안전한 기울기 한계가 검증됐다는 뜻은 아니다. 새 방향을
잇는 전체 FK corridor의 기존 자세 오차/속도/가속도/충돌 검사는 유지한다.
그리퍼를 열고 scene attachment를 해제한 뒤에는 손목 제한을 해제해 복귀한다.
손목 변화 최소화의 전역 최적해나 물체의 수평 자세 유지는 보장하지 않는다.

고정 투하 위치도 물체의 bounding sphere가 수거함 입구 내부에 들어가는지 검사한다.
벽 두께와 `sorting_release_edge_margin_m`(기본 0.005m)을 제외하고,
물체 아래쪽 높이는 테두리보다 `sorting_release_clearance_m`(기본 0.06m)에서
`sorting_release_maximum_clearance_m`(기본 0.21m) 사이여야 한다.
위치가 이 범위에 맞지 않으면 이동하지 않고 중단한다. 놓기 직전에도 실제
feedback으로 물체 중심·손목·함 입구를 다시 검사한다.

SAM 분할·기준 mask 추적·중앙 인계 상태는 제거했다. 현재 손목 검증은 YOLOE-seg의
HANDOFF/CHECK 재검출이며 이동 전후 접촉·충돌·수거함 개구부 검사는 유지한다.

예전 라벨 allowlist 정책, 테이블 배치 구역, 경유점/영역 탐색 운반은 제거했다.
