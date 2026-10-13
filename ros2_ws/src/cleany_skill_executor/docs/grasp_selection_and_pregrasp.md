# Grasp 선택과 pre-grasp 실행

[패키지 안내로 돌아가기](../README.md)

## 상태

점수순 grasp 후보를 양팔 MoveIt plan-only 검증으로 평가한다. 운영 action은 실제
trajectory와 gripper 명령을 실행하지 않는다. 스터디카페 수거 시뮬레이션에서 선택 결과를
MuJoCo arm controller로 실행할 수 있다. `nearest_pregrasp_coordinator`는 perception이
제공한 거리 유효 객체를 가까운 순서로 시도한다. 기본
`execute_grasp_and_lift=false`에서는 첫 reachable grasp의 pre-grasp까지만 실행하고,
옵션을 켜면 재인식과 Pilz LIN 기반 접촉 파지·후퇴·상승까지 수행한다.

## Reachable grasp action

`grasp/select_reachable` (`SelectReachableGrasp`)는 가까운 팔부터 IK, state validity,
current→pre-grasp와 pre-grasp→grasp plan을 검사한다. pre-grasp는 접근 벡터 반대 방향
0.14 m다. 5축 position-only IK의 방향 손실을 막기 위해 먼저 TCP 위치 IK로 seed를 만든
뒤, TCP의 실제 `-Y` 접근축 앞 0.14 m에 있는 가상 aim tip을 grasp point에 맞춘다. 이로써
물리 gripper가 선택 객체를 바라보는 것은 강제된다. 위치 seed가 실패하면 현재 joint
state에서도 계속 시도하며, 전체 arm joint limit 안에 분산한 팔당 8개 seed에서 허용
오차를 만족하는 해는 현재 상태 대비 joint-limit 정규화 이동량으로 정렬하고 wrist
roll에는 2배 가중치를 둔다. 자세 오차는 동률 해의 tie-break로만 사용한다. 5축 기구가 정확한
후보 방향을 만들 수 없는 경우에는 접근축 최대 15도, parallel-jaw 대칭을 고려한 closing
축 최대 30도 안에서 가장 가까운 실행 가능 방향을 허용한다. 그보다 큰 오차는 다음 arm
또는 후보로 fallback한다. grasp 위치에서도 TCP 위치·접근축·closing 축을 FK로 다시
검사한다. 한 해의 validity 또는 planning이 실패하면 같은 팔의 다음 grasp/pre-grasp
해를 먼저 시도한 뒤 반대 팔과 다음 후보로 넘어간다. 선택 오차는 selector 로그에 남긴다.

## 제공 계약

실행 결과는 Mission Manager가 상태 전이에 사용할 수 있는 성공·실패·차단 결과로 반환한다.
Skill Executor는 Mission Manager의 FSM 상태를 직접 변경하지 않는다.

## 설정 및 검증

입력 후보는 snapshot/object/frame/target OBB가 같고 frame은 설정된 `planning_frame`과
일치해야 한다. 현재 12개 arm/gripper joint는 완전하고 0.5초 이내여야 한다. action
동안 target OBB와 ACM은 임시 변경되고, 복원에 성공한 뒤에만 최종 성공·실패·취소
상태를 확정한다. 복원 실패는 planning-scene 오류로 반환한다. 한 번에 goal 하나만
처리한다.

```bash
ros2 launch cleany_moveit_config mock_planning.launch.py
ros2 launch cleany_skill_executor grasp_selection.launch.py
pytest -q ros2_ws/src/cleany_skill_executor/test
```

테스트는 기능별로 묶는다. `test_seeded_cartesian.py`는 경로 보간·자세 보정·URDF FK,
`test_gripper_geometry.py`는 폭 보정·접촉 판정·피드백 안정화,


planning frame, timeout, planning attempt/scaling, 최대 후보 수는
`config/grasp_selection.yaml`의 ROS parameter로 설정한다. timeout/cancel은 각 IK,
state-validity와 planning 단계 전후에 확인한다.

simulation 실행은 planning attempts 3, velocity/acceleration scaling 0.08,
MoveGroup replan 2회와 0.25초 delay를 사용한다. controller 실패 시 joint별 tracking
error를 기록하고 현재 상태에서 한 번만 같은 joint goal로 안전 재계획한다.

## 가까운 객체 자동 pre-grasp

`nearest_pregrasp_coordinator`는 다음 순서로 한 번의 작업을 수행한다.

```text
InspectScene 1차 detection
→ distance_valid 후보를 base_link 거리순 정렬
→ selected-object inspection
→ PlanGrasp
→ SelectReachableGrasp
→ target OBB를 충돌물로 유지하며 pre-grasp 실행
→ (execute_grasp_and_lift=true) 새 RGB-D 재인식과 동일 arm 재선택
→ Pilz LIN 접근 → gripper 접촉 → LIN 역접근 → LIN 수직 상승
```

양쪽 gripper는 object evaluation 전에 open한다. 따라서 MoveIt 후보 검증과 실제 실행이
동일한 gripper joint state 및 collision geometry를 사용한다.

가장 가까운 객체가 segmentation, grasp 생성 또는 양팔 도달성 검사에서 실패하면 다음
가까운 객체로 fallback한다. depth가 불충분해 `distance_valid=false`인 detection은
자동 조작에서 제외한다. infrastructure, MoveIt 또는 controller 실패는 다른 객체로
넘기지 않고 전체 작업을 실패시킨다.

RGB-D camera, `perception/inspect_scene`, `grasp/plan`,
`grasp/select_reachable`, MoveGroup과 arm/gripper controller를 먼저 실행한 뒤 coordinator를
시작한다.

```bash
ros2 launch cleany_skill_executor nearest_pregrasp.launch.py
```

query, timeout, 속도/가속도 scaling과 gripper open 위치는
`config/nearest_pregrasp.yaml`에서 설정한다. pre-grasp joint target은 운영 selector가
aim-tip IK와 FK 방향 검증까지 통과한 결과를 사용한다. 성공한 target OBB는 완료 자세에서
Planning Scene에 유지하며, 실패하거나 coordinator가 종료될 때 기존 ACM과 함께 복원한다.

Study-cafe의 네 물체를 대상으로 인식부터 접촉 집기와 후퇴까지 실행하려면 다음 launch를
사용한다.

```bash
ros2 launch cleany_skill_executor study_cafe_nearest_grasp_demo.launch.py
```

기본값은 YOLOE-seg instance mask + Gemini 3.1 Flash-Lite 상세 분류 설정이다. 기존 color adapter는
과거 머그컵/휴대폰/지우개 fixture 전용이며 현재 종이컵/레고/휴지에 대응하지 않는다.
이 study-cafe MuJoCo launch의 YOLOE 기본 체크포인트는
`~/models/yoloe/study_cafe_sim_yoloe26s_seg.pt`다. 현재 시뮬레이션 head 영상으로
fine-tune한 파일이며, 생성·학습·평가 절차는
[`cleany_perception/README.md`](../../cleany_perception/README.md#스터디카페-mujoco-전용-yoloe-seg-학습)에 있다.
손목 HANDOFF는 별도 `~/models/yoloe/study_cafe_sim_all_views_yoloe26s_seg.pt`를
`wrist_yoloe_model_path`로 로드한다. 오른손목 HANDOFF는 head 체크포인트를
`wrist_right_yoloe_model_path`로 재사용한다. 실제 컵 pregrasp 오른손목 프레임에서
head 모델은 컵을 confidence 0.726으로 검출했고, 양손목 바닥 학습 모델은 검출하지
못했다. 파지 후 CHECK는
`~/models/yoloe/study_cafe_sim_held_yoloe26s_seg.pt`를
`wrist_check_yoloe_model_path`로 로드한다. Head 모델은 기존 학습본을 유지한다. 왼손목
시점 holdout의 마스크 IoU 0.5 일치율은 52/80→68/80으로 개선됐지만, 새 모델을
head에 쓸 때는 71/80→64/80으로 하락했기 때문이다. 오른손목에서 실제 라벨이
있는 컵·휴지의 일치율은 38/40→36/40이었다. 이 수치는 고정 시뮬레이션 자세와
별도 생성 영상의 평가이며 실제 집기 성공률을 뜻하지 않는다. 학습 결과의 `best.pt`를
각 손목 경로로 복사해 사용한다. 파지 후 레고 holdout은 CHECK 모델에서 18/20으로
개선됐으나, 같은 모델의 파지 후 컵·마우스·휴지 검출은 아직 0/10이다. 실제 카메라 영상에 대한 성능은
검증하지 않았으며, 실물 실행에는 별도 검증된 checkpoint를 `yoloe_model_path`로 지정한다.
스터디카페 기본 실행은 왼손목 마우스 CHECK에
`wrist_mouse_check_yoloe_model_path`의 전용 모델을 쓴다. 실제 MuJoCo 실패
프레임에서 기존 모델은 마우스 검출 0건, 전용 모델은 신뢰도 0.25였고, 마우스
단독 및 전체 실행에서 파지 후 CHECK를 통과했다. 오른손목 CHECK의 컵 전용
모델이 물체를 놓치면 기존 파지 후 통합 모델로 같은 RGB를 재검사한다. 실제
MuJoCo 휴지 CHECK 프레임에서 컵 전용 모델은 검출 0건, 통합 모델은 휴지를
신뢰도 0.364로 검출했다. 두 추가 검사 모두 예상 물체의 투영 위치와 mask
검증을 통과해야 한다.
YOLOE 추론의 최소 confidence는 0.08이고, 클래스별로 컵·마우스·휴지는 0.25,
레고는 0.08을 적용한다. 다른 클래스 목록이나 모델로 바꿀 때는
`yoloe_class_confidence_thresholds`도 같은 순서로 조정해야 한다.
시뮬레이션 수거 정책 `table_sorting_policy.yaml`의 최종 최소 confidence도 0.08로
맞췄다. 레고 외 물체는 YOLOE 단계에서 먼저 0.25 미만을 제거한다.
다른 YOLOE checkpoint는 `yoloe_model_path`로 지정할 수 있다.
전체 파이프라인 실행은 [스터디카페 실행 안내](study_cafe_sorting_usage.md)를 따른다.
