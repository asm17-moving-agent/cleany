# 스터디카페 실행과 진단

[패키지 안내로 돌아가기](../README.md)

## 센서 전용 study-cafe 검증

`study_cafe_nearest_grasp_demo.launch.py`는 YOLOE-seg + Gemini,
aligned RGB-D/TF, AnyGrasp 후보와 MoveIt 검증을 연결한다.
`cleany_perception/config/yoloe_seg_gemini.yaml`을 바탕으로 스터디카페
전용 head/손목 체크포인트를 설정한다. `GEMINI_API_KEY`와 로컬 YOLOE 모델이 필요하다.
Depth obstacle scene은 객체 검출과 별도로 유지하며 target 위치를 simulator에서 받지 않는다.

```bash
make sim-mujoco-pipeline
make sim-mujoco-sorting
```

## 시뮬레이션 분리 수거 (통합 검증 진행 중)

분리 수거 모드는 selector의 `require_pregrasp_visibility=true`를 사용한다.
관측한 OBB 8개 모서리를 현재 camera TF 기준으로 원근 투영하고, 이를 포함하는
16면 visibility cone을 구성한다. MoveIt `GetStateValidity`의
`VisibilityConstraint`가 준비 자세의 로봇 CAD와 이 영역의 겹침을 검사한다.
가리는 자세는 움직이기 전에 제외하고 다른 IK/후보를 검토한다. 반경은 물체
정답 크기나 고정 컵 치수가 아니라 RGB-D OBB에서 계산하며 기본 padding은
3 mm다. 동적 camera TF는 기본 0.5초 이내여야 하고, 검사 결과가 누락되면
통과로 취급하지 않는다. 일반 selector의 기본값은 false다.

`sorting_contact_diagnostics:=true`는 MuJoCo observer의 접촉 부위/힘
출력만 켠다. 기본은 false이며, controller 오차와 실제 접촉을 대조하는
진단용이다. coordinator나 grasp selector는 이 GT 토픽을 읽지 않는다.

```bash
make sim-mujoco-sorting

# 기본 sorting 실행은 headless이며 RViz/image_view를 시작하지 않는다.

# 수거 완료/실패 후에도 시뮬레이터와 GUI 유지 (수거 동작을 자동 재시도하지 않음)
make sim-mujoco-sorting SORTING_ARGS='headless:=false use_rviz:=true use_image_view:=true shutdown_on_sorting_exit:=false'
# 같은 실행에서 GUI 창만 생략 (vendor 카메라 렌더링에는 DISPLAY 필요)
make sim-mujoco-sorting SORTING_ARGS='headless:=true use_rviz:=false use_image_view:=false'
```

MoveIt adapter의 `First ranked grasp/pregrasp` 로그는 실제 반환 순서의 첫 IK
해의 오차를 기록한다. 예전 `Accepted grasp` 로그는 joint-motion 우선 순서와 달리
최소 방향 오차만 출력하므로 선택된 해의 자세로 해석하면 안 된다. 첫 해도 이후
충돌/계획 검사에서 탈락할 수 있어 최종 선택은 action result의 joint state가 근거다.

Study-cafe launch의 `gripper_open_position_rad`(기본 1.2rad)로 실행 전 개방 폭을
명시적으로 설정할 수 있다. Sorting은 홈에서 해당 개방까지의 sweep을 검사한 뒤,
그 상태로 pregrasp와 접근을 계획한다. 큰 물체 진입 실험용이며 close command의
보정식, 접촉 gate, joint/controller 허용치는 변경하지 않는다.

Sorting selector는 `joint_limit_margin_rad=0.005`로 arm IK endpoint에 관절 상한/하한
여유를 둔다(일반 selector 기본 0). 경계에 포화된 해는 순위 목록과 state validity
앞에서 제외한다. 43의 pregrasp가 손목 상한에 정확히 놓인 뒤 feedback가 12.44µrad
넘어 Cartesian 시작 검증이 실패한 것을 방지하기 위한 선택 조건이다. 실제 feedback나
trajectory를 clipping하지 않으며 hard joint bounds/controller 허용치는 그대로다.

Study-cafe launch는 모든 child process 시작 전에 `fastdds_profiles_file`을
`FASTRTPS_DEFAULT_PROFILES_FILE`로 전달한다. 이미 지정된 환경변수를 우선 보존하고,
없으면 `cleany_perception/config/fastdds_rgbd.xml`(UDP + participant별 16 MiB SHM)을
기본으로 쓴다. 빈 launch 값은 사용자 기본 middleware 설정으로 되돌리는 진단 옵션이다.
다른 RMW의 동작을 바꾸는 설정이 아니며 QoS·freshness 기준은 변경하지 않는다.
41의 실제 집기 구간에서 지도 receipt 2초 초과가 없었으나 모든 부하/장치에서의 보장은
아니다. 별도로 띄우는 기록기/카메라는 같은 DDS 환경을 설정해야 비교할 수 있다.

`sorting_reobserve_after_lift=true`는 head 재검출에 실패한 경우(기본 관측 경로에서는
검출 대상이 없을 경우) 한 번의 카메라 재관측 이동을 허용한다. 정착한 실제 TCP와 관측 OBB로 파지
offset을 잡고, CameraInfo/TF의 중앙 영상 ray 3개와 관측 높이 평면들의 교점을
제안한다. 관측 OBB의 bounding sphere 전체가 화면 안에 들어오는 후보만 쓰며
`sorting_reobserve_margin_px=24`, `sorting_reobserve_geometry_padding_m=0.01`,
`sorting_reobserve_max_translation_m=0.20`으로 여백·최대 이동을 제한한다.
현재 높이에서 맞지 않으면 `sorting_reobserve_max_lowering_m=0.10` 범위에서
높이를 최대 3단계로 검사한다. 낮추더라도 필수 상대 상승량에
`sorting_reobserve_height_clearance_m=0.02`를 더한 높이 아래로 내려가지 않는다.
attached geometry를 보존하는 collision-aware IK와
일반 MoveIt 경로 계획·실행을 통과해야 한다. 이동 전후 gripper contact를 확인하고
이후 같은 선택된 관측 경로의 상대 상승량 검증을 다시 수행한다. 관측 재실패·재구성 실패·
관측 높이 부족은 성공으로 처리하지 않는다. `sorting_held_association_tolerance_m=0.03`
내에서 관측 중심과 파지 추정 중심이 일치해야 운반으로 넘어간다. 이 기능은
센서 기반 재관측 동작이며 MuJoCo GT나 자동 물체 부착을 사용하지 않는다.
실제 재관측 성공 여부는 실행 기록으로 별도 검증해야 한다.
재관측/수거함 운반의 일반 joint-goal 이동에는 파지 중에만
sorting launch에서는 `sorting_payload_velocity_scaling=0.24`,
`sorting_payload_acceleration_scaling=0.20`를
적용한다. 기존 속도·가속도보다 높이는 설정은 적용하지 않으며 열린 그리퍼로
복귀할 때는 원래 값을 사용한다. 접근/역접근의 Cartesian retiming은 별도로
유지한다. 노드 단독 기본값은 둘 다 0.01이며 launch 인수로 이전 값을 재현할 수 있다.
이는 시뮬레이션 시험 설정이며, 지속 파지 성공이나 실제 하드웨어 적합성을 뜻하지 않는다.

`config/table_sorting_policy.yaml`은 Gemini의 category/reason을 받아
쓰레기를 `trash_right`, 분실물을 `lost_items_left`로 보낸다. 위험 라벨과 저신뢰,
category/reason 미제공 결과는 review로 남기고 조작하지 않는다.

현재 장면은 종이컵·마우스·휴지뭉치·확대한 레고를 사용한다. YOLOE의 명시 클래스는
`cup`, `computer mouse`, `crumpled tissue`, `lego brick`이며 Gemini는 고정 목록 없이
외형에 따른 자유 라벨을 생성한다. 모델 출력 category/reason으로 분류하고 위험·저신뢰
결과는 review로 남긴다. 장면의 형상·크기와 평가용 body mapping은
[MuJoCo README](../../cleany_mujoco_sim/README.md)를 따른다.

수거 위치는 로봇 뒤쪽 +Y(좌측)의 `lost_items_left`, -Y(우측)의
`trash_right`다. `cleany_mujoco_sim/config/robot_top_bins.yaml`은
수거 위치의 알려진 보정값/형상만 공유한다. 책상과 집을 물체의 위치/형상은
카메라에서 얻으며 scene의 target 좌표를 planner에 전달하지 않는다.
수거함은 바닥과 4개 벽으로 구성되고 MoveIt에도 같은 목적지 형상을 등록한다.
내부 후면 받침판도 같은 설정으로 MoveIt 충돌 형상에 등록한다.
이 좌표는 현재 고정 베이스 시뮬레이션 전용이며 이동 베이스 운용을 보장하지 않는다.

분류 → 물체에 가까운 팔의 후보를 먼저 검사하고 실패하면 반대 팔 검사 → 재인식/집기/후퇴/들기 →
부착 물체를 포함한 MoveIt 운반 → 수거함 개구부 확인 → 열기 → 복귀 →
선택적 놓기 검증 순서다. 운반 실패 또는 비정상 물체 중심·반경이면 그리퍼를 열지 않는다.
sorting 모드에서는 다각도 grasp 후보와 전체 팔 IK seed를 추가로 탐색하되
기존 pose/충돌 허용 기준은 유지한다. 운반 IK에는 전체 joint feedback과
`RobotState.is_diff=true`를 보내 부착한 OBB가 검사에서 빠지지 않도록 한다.
Sorting은 초기 양손 개방을 따로 명령하지 않는다. 선택된 팔의 pregrasp와
그리퍼 개방을 6관절 경로로 함께 계획하고, 목표 도착 시 모든 관절 피드백을 검사한다.
별도 닫기·놓기의 `gripper_motion_sec` 기본값은 2초이며 반환 시에도 팔 복귀와
턱 닫기를 같은 경로로 계획한다. 속도는 [Sorting 속도 설정 표](study_cafe_sorting_configuration.md#sorting-속도-설정-2026-09-08-변경)를 따른다.
재검출 후보를 selector에 전달하기 전 이전 snapshot의 target OBB를 제거해 중복
충돌체를 방지한다. 닫기 후 잔여 각도·속도의 안정 구간이 확인돼야 attachment/lift로 진행한다.
동작 실패 뒤 depth boost 복원까지 실패하면 두 오류를 기록하고 최초 동작 오류를 유지한다.

재선택한 grasp가 현재 팔 위치에서 LIN 방향 기준을 넘으면, 반환된 새
pregrasp로 충돌 검사 이동한다. 이 이동 중 target 접촉은 허용하지 않는다.
이후 새 RGB-D로 동일 물체의 OBB 모서리 집합이 기본 5 mm 이내에서 유지되는지
확인해야 접근한다. 없거나, 오래됐거나, 여러 물체가 일치하면 중단한다.
기존 LIN 10도 기준은 그대로이며, 보정 이동 후에도 다시 검사한다.

Sorting은 `use_joint_corridor_grasp=true`, `use_seeded_cartesian_grasp=true`로
선택된 grasp **관절값**을 보존하는 접근을 사용한다. TCP는 실제 시작점과
목표 사이의 반경 1 mm 원통 안으로 제한하고, 역접근은 실제 접근 시작 관절값을
목표로 동일하게 계획한다. 두 끝 관절값의 보간을 seed로 기본 10 mm 간격
collision-aware IK를 구한 뒤 단조 cubic 곡선을 구성한다. 모든 중간 검사는
전체 관절 피드백과 현재 attached body를 보존한다.
이는 5-DOF 팔에서 임의의 6-DOF 방향을 정확히 보간하는 Pilz LIN과 동일하지 않다.
반환된 모든 waypoint를 FK로 검사해 원통 이탈/역행을 거부하며, 시작·끝 방향은
0.01 rad 이내, 중간 방향은 두 끝 자세의 SLERP에서 기본 5도 이내여야 한다.
충돌·목표 접촉 허용 정책과 controller/contact 한계는 변경하지 않는다.
관절 속도와 가속도의 곡선 전체 상한을 분석해 시간을 정하고, 실제 JTC cubic
보간 곡선을 기본 1 mm 분할 및 관절별 0.005 rad 이하 간격으로 표본화해
MoveIt 충돌/constraint/FK 검사를 거친다. 최대 2000점 초과 시 중단한다.
JTC에 position+velocity를 전달하며 acceleration 필드는 비워 동일한 cubic
보간을 유지한다. 모델 관절 한계는 `cartesian_joint_limits_file`(기본 MoveIt
`joint_limits.yaml`)에서 읽는다. 가속도 미정 관절은 명시 파라미터
`cartesian_joint_acceleration_rad_s2=1.0`에 실행 scaling을 적용한다. 이는
MoveIt 기본 가속도와 같은 시험용 상한이지 하드웨어 검증값은 아니다.

추가로 FK sample의 병진/회전 속도와 병진 가속도를 기준으로 trajectory
시간만 균일하게 늘린다. 기본 Cartesian 상한은
0.10 m/s, 0.50 rad/s, 0.20 m/s²에 실행 scaling을 적용하고, 추가 시간 여유는
`corridor_time_margin=2.0`이다. 위치/경로는 바꾸지 않고 velocity/acceleration도
같은 비율로 줄인다. 이 FK 표본 검사는 연속 시간 속도/충돌 또는 실제 하드웨어
안전 인증을 대신하지 않는다. `use_seeded_cartesian_grasp=false`이고 joint
corridor만 켜면 기존 OMPL 계획+FK 검사 경로를 사용한다. 일반 데모의 두
모드는 opt-in이며 수직 lift는
여전히 endpoint 검사를 통과한 Pilz LIN을 사용한다.

sorting launch의 현재 기본값은 `approach_velocity_scaling=1.0`,
`retreat_velocity_scaling=0.8`, `corridor_time_margin=1.05`이다. 같은 이름의
launch 인수로 조절하며 일반 데모는 기존 0.2/0.4/2.0을 유지한다.
관절 속도·가속도 한계, 충돌/경로 검사, controller tolerance는 완화하지 않는다.
sorting joint-corridor의 grasp 접근에만 `sorting_approach_acceleration_scaling=1.0`을
적용한다(기존 0.4). 계획과 FK sample 시간 배율 검사에 같은 값을 사용한다.
관절 자체 한계는 제거하지 않는다. sorting Cartesian fallback 가속도는 4 rad/s²,
별도 gripper motion은 2초, LIN 가속 배율은 0.8이다. MoveIt TOTG의 URDF 미정
관절 fallback과 Cartesian fallback은 서로 다른 경로다. 현재값 전체는 [Sorting 속도 설정 표](study_cafe_sorting_configuration.md#sorting-속도-설정-2026-09-08-변경)를 따른다. 실기 검증값은 아니다.

sensor-scene 모드에서는 OBB attach 성공 뒤 **그 시점보다 새로운 촬영 시각**의
처리 완료 depth receipt를 기다린다. `attachment_scene_timeout_sec=5.0` 안에
오지 않으면 후퇴를 시작하지 않는다. 기존 capture/map age 2초 조건도 유지한다.
이 barrier는 새 frame 통합 전 계획을 막지만 지도 잔상이 모두 없어졌음을
보증하지는 않는다.

Sorting selector는 `align_grasp_wrist_roll=true`도 사용한다. 현재 Cleany
URDF의 wrist-roll 축과 TCP offset이 local -Y로 일치하므로, position IK가
찾은 해의 closing 방향을 이 축 둘레로 직접 맞춘 후보를 만든다. 관절 범위
안의 동치 각도만 검사하며 비대칭 jaw의 180도 반전을 허용하지 않는다.
회전한 후보는 전체 robot state 충돌 검사와 기존 FK 위치/approach/closing
허용 기준을 다시 통과해야 한다. 회전은 **계획 후보**에만 적용하며 실제
로봇/물체를 강제로 회전하지 않는다. 모델-parity 테스트가 축/TCP 가정을
검사하며 일반 selector 기본값은 false다.
부착 해제 시에는 MoveIt이 world에 다시 만든 임시 OBB까지 순차적으로
제거한다. `/sorting/status`의 실패 메시지에는 별도 `error`가 포함된다.
`sorting_artifact_directory:=/absolute/path`를 launch에 지정하면 실행마다
새 `run-<uuid>` 하위 폴더에 `inspection`(InspectScene.Result),
`grasps`(PlanGrasp.Response), `selection`(SelectReachableGrasp.Result)을
CDR로 저장한다. fresh depth/3D 추정과 실제 사용한 후보를 오프라인에서
비교하기 위한 출력 전용 옵션이며, 저장 데이터를 제어 입력으로 재사용하지 않는다.
기본값은 빈 문자열로 저장하지 않는다.
접근/후퇴/lift의 `motion_request`(MoveGroup.Goal)와 `motion_result`
(MoveGroup.Result)는 계획 실패/실행 전 거부 시에도 저장한다.
seeded 방식에서는 요청의 pipeline/planner ID가 `cleany_seeded_cartesian` /
`monotone_cubic`이며, 이는 계획 입력을 같은 형식으로 기록한 것이다.
해당 가상 pipeline ID를 MoveGroup 서버에 전송하지 않는다. 결과의 trajectory와
planning_time은 실제 IK/검증 계산으로 작성하며 실패도 기록한다.
실행 전 FK 검증을 통과한 trajectory는 `motion_plan` 이름의
`moveit_msgs/RobotTrajectory` CDR로 저장한다. 이 출력에는 corridor 시간 보정이
반영되며, 저장 자체가 물리 실행/파지 성공을 뜻하지는 않는다.
`grasp_approach_offset_m` launch 인수(시뮬레이션 수거 0.018 m, 그 외 0.016 m)는 selector의 IK 목표와
coordinator의 실행/표시 보정에 동일하게 전달한다. 회전식 비대칭 jaw의
벌림에 따른 실제 접촉 깊이를 보정하는 시험용 입력이며, 현재 기본값은
실측 하드웨어 보정이나 지속 파지 성공으로 검증된 값이 아니다.
같은 옵션은 selector의 출력 전용 service trace도 활성화한다. 별도
`services-<uuid>` 폴더에 각 MoveIt 서비스 요청/응답 CDR과 완료 시간,
취소 여부 JSON을 저장한다. 실패한 IK 요청의 seed/목표를 재현하기 위한
진단이며 응답이나 계획을 수정하지 않는다. 이 옵션의 파일 I/O가 포함된
시간을 저장 기능을 끈 운용 성능으로 해석하지 않는다.
`/sorting/status`는 단계, label, 분류, 목적지, 완료 항목을 JSON String으로
latched 발행한다. 기본은 관찰 모드이며 물체별 `placement_verified=false`와
`mission_complete_unverified`를 기록한다. 자동 검증을 켜면 물체별 안착 확인 후
`mission_complete`를 발행한다. 빈 작업영역은 연속 두 번 확인해야 한다.
`sorting_require_category_coverage=true`일 때만 `sorting_required_categories`의
모든 종류가 완료 항목에 포함됐는지 추가 검사한다.

`/sorting/verify_placement`는 독립된 결과 검증 port다. 현재는 시뮬레이션
oracle이 release 이후의 새 물체 AABB가 올바른 수거함 내부에서 안정적으로
정지했는지 확인한다. 이 oracle은 인식·분류·grasp·경로 계산에 사용하지
않는다. 실제 로봇에는 별도의 센서 기반 결과 검증 구현이 필요하다.
