# 스터디카페 수거 설정과 실험 기록

[패키지 안내로 돌아가기](../README.md)

## 작업자 관찰 모드와 파지 깊이

시뮬레이터를 시작하는 수거 launch는 `sorting_verify_placement:=false`가
기본이다. 물체를 놓고 팔이 복귀하면 `VerifyPlacement` 서비스 호출/대기 없이
다음 물체 탐색으로 넘어간다. 시작 시 해당 서비스 준비도 기다리지 않는다.
결과는 `complete_unverified`, `placement_verified: false`로 기록하며 자동
안착 검증 성공으로 취급하지 않는다. 정상 종료 단계도 `mission_complete_unverified`
이다. 실제 안착 여부는 작업자가 GUI로 확인한다.
기존 자동 검증은 `sorting_verify_placement:=true`로 복원한다.
이 변경은 운반 중 접촉 이탈 감시, 안전 정지, 충돌 검사, 다음 물체를 찾는
카메라 인식 및 최종 작업 영역 비움 판정을 제거하지 않는다.

휴지(`crumpled tissue`, `tissue`, `crumpled paper`)는 시뮬레이션 수거에서만
`tissue_grasp_extra_depth_m:=0.004`를 더해 총 접근 보정량 22mm를 사용한다.
시뮬레이션 수거의 공통 `grasp_approach_offset_m`는 18mm로, 이전 16mm보다
접근 방향으로 2mm 더 깊게 잡는다. Selector IK와 실행 목표는 launch의 공통 보정 설정을 공유한다.
pregrasp 기준 위치와 물체별 추가량(마우스 +14mm, 레고/휴지 +4mm)은 유지한다.
공통 깊이만 원복하려면 `grasp_approach_offset_m:=0.016`을 사용한다.
휴지 변경만 원복하려면 `tissue_grasp_extra_depth_m:=0.0`으로 실행한다.
새 기본값은 다음 launch부터 적용되며 이미 떠 있는 GUI의 파라미터를 소급 변경하지 않는다.

## 선택형 시뮬레이션 최적화와 원복

수거 launch에 `sim_performance_profile:=tabletop_fast`를 추가하면 먼 정적
배경 충돌을 비활성화하고 그림자 맵만 2048로 낮춘다. 기본값 `baseline`으로
재시작하면 기존 설정으로 돌아온다. 그림자 4096을 유지하는
`tabletop_collision`도 제공한다. 파지 깊이·물체 물리·관절 속도·YOLOE-seg·
MoveIt 충돌 검사와 완료 판정은 바꾸지 않는다.
적용 범위, 실행 예제와 비교 측정은
[`cleany_mujoco_sim/README.md`](../../cleany_mujoco_sim/README.md#되돌릴-수-있는-고정-책상-최적화)를 따른다.

MuJoCo sorting의 마우스는 `mouse_grasp_extra_depth_m=0.014`를 기본 적용한다.
`mouse`, `computer mouse`, `wireless mouse` 라벨에만 기본 접근 0.018m에
14mm를 더해 옆면을 잡는다. Selector와 executor가 같은 보정을 사용하며
pregrasp 위치, 다른 물체의 깊이, 충돌·접촉 유지 검사는 유지한다.
레고는 `lego_grasp_extra_depth_m=0.004`로 4mm만 추가한다(총 접근 22mm).
추가량은 설정된 라벨과 정확히 일치할 때만 적용된다. 예를 들어
`red building block`은 현재 레고 추가량 매칭 대상이 아니므로 공통 18mm를 사용한다.
실물/비sorting 기본 보정은 0이다. 일반 ROS 설정에서는 `deeper_grasp_labels`와
같은 길이의 `deeper_grasp_offsets_m` 배열을 selector/executor 양쪽에 동일하게 설정한다.
마우스 크기와 모양이 달라지면 재검증해야 하는 시뮬레이션 보정값이다.

그리퍼 닫기 명령 성공 후 접촉 판정은 새 joint-state 촬영 시각으로 확인한다.
기존 접촉 속도/잔차 기준을 연속 3개 이상, ROS 시간 기준 0.10초 이상 만족하면
즉시 진행한다. 튀는 속도는 안정 구간을 초기화한다. 실제 시간 최대 5초
(`gripper_contact_feedback_timeout_sec`) 동안 확인하며, 새 피드백이 없거나
안정되지 않으면 실패한다. `gripper_contact_stable_duration_sec`로 안정 구간을
조절한다. 확인 중 추가 닫기 명령을 보내지 않으며 controller 실패도 통과시키지 않는다.

현재 스터디카페의 분실물 대상은 레고와 마우스다. Gemini/YOLOE 프로필은
`computer mouse` 라벨을 사용하며 분류 정책은 `mouse`, `computer mouse`,
`wireless mouse`를 분실물로 처리한다. 시뮬레이션 배치 검증도 같은 별칭을 지원한다.

Top-down 비교 시험은 `make sim-mujoco-sorting SORTING_ARGS="topdown_only:=true"`로
실행한다. support plane 법선 아래 방향의 후보만 만들며 기울어진 접근 후보 탐색을
끄고, 수평 성분 잡음에 민감한 로봇 반대쪽 접근 필터도 이 모드에서만 끈다.
기존 IK/충돌 허용 기준은 유지한다. 평면 추정과 IK에는 기존 오차 허용이 있으므로
수학적으로 완벽한 수직 고정 제어는 아니다. 기본은 false이며 자동으로 사선 접근으로
되돌아가지 않는다. GUI 유지가 필요하면 `shutdown_on_sorting_exit:=false`를 추가한다.

Study-cafe 파이프라인의 초기 손목 roll은 양팔 모두 `+1.58 rad`이다.
오른손 카메라 장착부가 왼손과 같은 방향으로 시작하도록 하며, MuJoCo 초기
keyframe의 관절값과 actuator 유지 목표에 함께 적용된다. 카메라 TF/장착 형상이나
실물 모터 캘리브레이션은 변경하지 않는다. 실행 중 파지 이후 손목 제한과는 별개다.

### 파지 후 Depth 최신성

MuJoCo sorting은 `cleany_scene_mapping/KnownGeometryOctomapUpdater`를 기본으로
사용한다. 가려진 파지 대상 내부의 이전 OctoMap 점유를 정리하여, 들기 시작점에서
잡은 물체와 자기 잔상 사이의 충돌로 중단되는 것을 줄인다. 관측에서 등록한 형상
내부에 완전히 포함된 셀만 정리하며, 주변 장애물과 부분 겹침 셀은 유지한다.
실물 및 sorting 이외 실행은 기존 updater를 유지한다. `depth_octomap_plugin`으로
명시 변경할 수 있다. 로봇/물체 형상 오차가 있는 환경의 안전성을 보증하지 않는다.

손목 카메라 사용 시 접근·그리퍼 닫기·들어올리기 구간에
`/sorting_cameras`의 `head_depth_boost=true`를 설정한다. 손목 선택과 YOLOE-seg
추적은 유지하며, 구간 종료(실패 포함)에 false로 복원한다. 물체 attachment 이후
촬영된 Depth 및 최신 OctoMap 확인은 생략하지 않는다. 최신성 허용값 2초와
실물 sorting의 attachment 대기 상한 10초는 유지한다. MuJoCo sorting은
GUI·추적 부하로 시뮬레이션 진행이 느려지는 경우를 위해 실제 시간 기준 상한을
30초로 둔다. `attachment_scene_timeout_sec` 실행 인수로 조절한다.
최신 데이터가 도착하면 즉시 진행하므로 고정 30초 대기가 아니다.
MuJoCo 장애물 점군은 `scene_cloud_pixel_stride=4`로 샘플링하여 기존 stride 2
대비 최대 점 수를 1/4로 줄인다(640×480 기준 76,800 → 19,200).
인식·YOLOE-seg 입력 해상도는 유지한다. 작은 장애물의 점 밀도가 줄어드는 절충이
있으며, 비교 시 `scene_cloud_pixel_stride:=2`로 복원할 수 있다. 실물 기본은 2다.
카메라 노드도 함께 빌드해야 하며 boost 설정 거부 시 이동을 시작하지 않는다.

스터디카페 grasp 후보 생성의 어깨 기준점은 migrated CAD URDF의 좌우
shoulder origin을 사용한다. 기존 모델의 기준점을 혼용하지 않는다.

## Sorting 속도 설정 (2026-09-08 변경)

Simulation sorting launch의 감속 기본값을 상향했다. 일반 demo와 실제 로봇의
하드웨어 한계는 변경하지 않는다. 각 이름은 launch argument 및 ROS parameter다.

| 설정 | 이전 | 현재 |
|---|---:|---:|
| `velocity_scaling` / `acceleration_scaling` | 0.08 / 0.08 | 0.30 / 0.50 |
| `sorting_payload_velocity_scaling` / `sorting_payload_acceleration_scaling` | 0.04 / 0.02 | 0.24 / 0.20 |
| `cartesian_translation_speed_m_s` | 0.10 | 0.30 |
| `approach_velocity_scaling` / `retreat_velocity_scaling` | 0.7 / 0.6 | 1.0 / 0.8 |
| `cartesian_translation_acceleration_m_s2` | 0.20 | 0.80 |
| `cartesian_joint_acceleration_rad_s2` | 1.0 | 4.0 |
| `cartesian_rotation_speed_rad_s` | 0.50 | 1.50 |
| `lin_acceleration_scaling` | 0.4 | 0.8 |
| `corridor_time_margin` | 1.25 | 1.05 |
| `gripper_motion_sec` (별도 닫기/놓기 명령) | 8.0 | 2.0 |

접근 가속도 배율 `sorting_approach_acceleration_scaling=1.0`은 유지한다.
pregrasp와 그리퍼 열기는 기존 6관절 동시 계획을 유지하며, 별도 2초 대기를
추가하지 않는다. 운반은 기본 이동 배율과 payload 배율의 최솟값을 적용한다.
위 Cartesian 속도는 retiming 상한이지 실제 일정 속도를 보장하는 값이 아니다.
충돌/FK/관절 위치·속도 한계, 추적·접촉 감시, timeout 취소는 유지한다.
시험 runner의 강제 0.5배속도 제거해 launch 기본과 동일한 1.0배속을 사용한다.
CPU 시뮬레이션의 실측 RTF는 여전히 1 미만일 수 있다. 새 설정의 실제 3배
단축과 파지 유지 성공은 별도 실행으로 검증해야 하며 하드웨어 검증값이 아니다.

sorting launch의 `use_observed_collision_geometry=true`는 grasp 서버가 발행한
`/grasp/collision_geometry`를 target collision/attachment에 사용한다. 일반
기본 false는 기존 OBB 경로다. 64개 bounded cache에서 snapshot/object ID,
capture header와 OBB pose가 정확히 일치하는 mesh만 사용한다. finite/closed/
outward/convex 검사와 vertex/triangle 상한을 통과해야 하며, 활성화 상태에서
mesh가 없으면 timeout 후 실패하고 OBB로 조용히 대체하지 않는다. support patch,
전체 로봇 open/closure 검사는 유지한다. 운반/재관측/놓기 반경은 OBB 반대각선과
관측 mesh의 최대 local vertex 거리 중 큰 값이다. trimmed OBB 바깥 관측점도
포함하며 mesh 사용 모드에서 정확히 일치하는 geometry가 없으면 실패한다.
관측 프리즘은 숨은 실제 형상의 보장이 아니며, geometry cache는 identity 검사다.
시간 신선도 검사는 기존 sensor-scene/wrist 경로가 별도로 수행한다.
sorting artifact에는 수신한 `collision_geometry` CDR도 함께 기록한다.
헤드 재관측 후에는 기존 중심/방향 연속성 제한을 만족하는 후보만 동일 팔
reachability selector에 전달한다. 새 후보의 점수 순위 변화로 방향이 크게
다른 후보를 먼저 선택한 뒤 전체 작업을 중단하지 않도록 한다. 제한 자체는
완화하지 않으며 호환 후보가 없으면 이동 전에 실패한다.
Launch argument `sorting_head_reference_refresh_age_sec` 기본값은 0초(비활성)다.
양수 값으로 설정하면 단독 진단에서도 헤드 재관측 경로를 검증할 수 있다.
활성화 시 simulation clock 기준 새 관측+재계획도 같은 신선도 상한을
만족해야 하며, 설정값은 30초를 초과할 수 없다.

`planning_scene_timeout_sec`는 응답 대기 상한이다. 일반 selector/executor는
각각 1/2초, CPU sorting launch는 5초다. 고정 대기를 추가하지 않으며 timeout
발생 시 해당 작업은 실패한다. 충돌 조건이나 관측 신선도 상한과는 별개다.
selector의 `state_validity_timeout_sec`도 CPU sorting launch에서 5초로 설정한다
(일반 기본값 1초). 이는 `/check_state_validity` 응답 상한이며 충돌 검사를
생략하거나 실패 결과를 허용하지 않는다.
`fk_timeout_sec`도 sorting에서 5초다. `ik_response_margin_sec`와
`planning_response_margin_sec`는 각각 IK/MoveGroup 계산 예산 뒤에 더하는
응답 여유이며 일반 1초, sorting 5초다. aim IK에도 같은 IK 응답 여유를 쓴다.
IK 계산 예산 0.15초, aim IK 1초, MoveGroup 계획 예산 4초 자체는 늘리지 않는다.
Cartesian 실행 wall deadline은 `max(minimum_sec, trajectory_duration * factor +
margin_sec)`이다. `cartesian_execution_wall_timeout_` 접두사의 `minimum_sec`는
60초, `margin_sec`는 10초, `factor`는 일반 2/sorting CPU simulation 10이다.
이는 고정 대기나 동작 감속이 아니며 완료 결과가 도착하면 즉시 진행한다.
기존 접촉/추적 감시와 timeout 시 취소·종료 확인은 유지한다. 느린 시뮬레이션의
13.9초 궤적이 wall time 60초를 넘는 경우에도 고정 60초로 잘리지 않게 한다.

Sorting은 초기 양손 그리퍼 열기 단계를 실행하지 않는다. 선택된 팔만
`*_pregrasp_open` 6관절 MoveIt group으로 pregrasp 이동과 열기를 함께 계획·실행한다.
팔과 그리퍼를 별도 비동기 명령으로 겹치지 않고, 열린 jaw를 포함한 전체 경로를
충돌 검사한다. Target contact 허용은 여전히 grasp 접근 단계에서만 적용한다.
MoveIt이 기존 팔/그리퍼 controller에 궤적을 분배하며, 양쪽 action의 완료와
선택한 6관절 feedback를 확인하기 전에는 손목 인계/접근으로 넘어가지 않는다.
반대 팔 그리퍼는 움직이지 않는다. Grasp/retreat IK는 기존 5관절 계약을 유지한다.
투입 완료 후에는 `*_return_close` 6관절 group으로 저장된 원래 팔 자세로 복귀하면서
선택 그리퍼를 `sorting_return_gripper_position_rad=-0.30` rad로 함께 닫는다.
의도한 release가 완료되고 attachment가 해제된 빈 팔에서만 허용하며,
이 경로도 jaw를 포함해 충돌 검사하고 두 controller의 완료를 확인한다.
`gripper_open_position_rad`는 동시 이동의 최종 열림 위치다. 기존
`gripper_motion_sec=8`은 별도 파지/놓기 명령에만 적용하며 동시 이동 시간은
MoveIt의 속도·가속도 제한 및 경로가 결정한다. 시작 상태 표시는 `search`다.

2026-09-07 headless attempt56에서는 왼팔/왼쪽 그리퍼 controller의 첫 goal 수신
간격이 2.06 ms였고 두 controller가 같은 시점에 성공했다(동시 이동 약 9.65초).
오른쪽 그리퍼 goal과 초기 `prepare_grippers` 단계는 없었다. 컵 투입·복귀 확인은
coordinator `starting` 기준 145.53초, `pick` 기준 114.71초였다. 가속도 설정은
변경하지 않았다. 이전 headless attempt54의 152.44초보다 총 6.91초 짧았으나
인식/계획 시간 변동이 포함된 단회 비교이며 제거된 초기 열기 대기 시간과 같지는 않다.
컵 안착은 검증됐지만 전체 미션은 기존처럼 `lost_item` 미검증으로 종료했다.
실행 로그는 `artifacts/sorting_20260907/launch_attempt56_parallel.log`에 있다.

head RGB-D로 물체/3D grasp를 선택하고 pregrasp에 도착하면 해당 손목 RGB로
인계한다. 이때 head renderer는 10→2 Hz, 선택 손목은 10 Hz가 된다.
기존 3D 추정과 경로를 보존하고 손목 RGB로 대상 일관성을 검사하며,
이 분기에서 head 재검출/3D 재계산과 head 시야 복구 이동을 수행하지 않는다.
RGB 인계가 실패하면 중단하며 이전 3D 위치를 새 측정값으로 포장하지 않는다.
lift는 손목 mask·그리퍼 관절 feedback 기반 접촉 추정·kinematic clearance로 확인한다.
접촉 추정은 닫힘 명령 대비 정지한 관절 상태를 사용하며 독립 힘 센서 측정이 아니다. 이것은 독립적인
depth 상승량 검증이 아니다. 수거함은 `robot_top_bins.yaml`의 초기 base_link 기준 위치를
사용하고 운반 전후 접촉과 충돌·개구부 검사는 유지한다. 복귀 시작 전 head 활성 빈도로 복원한다.
시뮬레이션 손목 TF는 nominal CAD mount이며 실제 로봇에서는 측정한 양팔 hand-eye
calibration이 필요하다. `sorting_wrist_cameras_config`로 simulation nominal YAML을
지정하며 기본은 `cleany_mujoco_sim/config/sorting_wrist_cameras.yaml`이다.
외부 로봇 실행은 아직 지원하지 않는다. 손목 영상 기반 연속 위치 보정(visual servo)은
구현하지 않았으며 접근/운반 경로는 기존 head 3D 추정 및 관절 feedback에 의존한다.
기존 head-only 비교 시험은 `sorting_use_wrist_camera:=false`로 실행한다.

연속 추적 적용 전인 2026-09-07 headless attempt54에서는 batch 손목 확인으로 컵을 `trash_left`에 투입하고
복귀했다(pick 시작부터 안착 확인까지 110.60초, CPU 손목 확인 RPC 19.16초 포함).
현재 전체 미션은 `lost_item` 미검증으로 종료한다. 성공 1회는 연속 수거/반복 성공률
검증이 아니며 손목 영상 기반 실시간 위치 보정이 완성됐다는 의미도 아니다.

이전 캔·박스·무작위 pregrasp 데모는 제거했다. 스터디카페가 재사용하던
MoveIt 실행·joint feedback helper는 `grasp_execution.py`의 `GraspExecutionNode`,
카메라 투영·시각화 helper는 `core/rgbd_projection.py`로 분리했다.
현재 launch와 테스트는 이 공통 코드를 사용한다.
