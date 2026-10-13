# 11. 테스트와 디버깅

> 목표: 낯선 영역에서도 입력·출력·실패를 따라가며 읽을 코드를 찾고, 테스트로 구현의 약속을 확인합니다.

## 11.1 데이터 경계와 증상 분리

“캔 점군은 있는데 팔이 움직이지 않는다”는 문제를 생각해 봅시다. 어느 파일이 원인인지 바로 알기는 어렵습니다. 대신 가장 최근에 **확인한 결과**와 그다음 **기대하는 결과**를 적어보세요.

```text
RGB-D → target/context 점군 → 파지 후보 → 도달 가능한 후보 → 계획 → 실행 피드백
                               ↑ 어디까지 확인했는가?
```

점군 생성, 후보 생성, 계획, 실행은 다른 단계입니다. 후보가 비었는지, 모든 후보가 도달 불가인지, 계획만 수행하도록 실행했는지 나누면 읽을 파일이 좁아집니다. 성공 메시지도 어느 단계의 성공인지 확인합니다.

| 질문 | 읽을 곳 |
| --- | --- |
| 입력 필드는 무엇인가? | [PlanGrasp.srv](../../../ros2_ws/src/cleany_interfaces/srv/PlanGrasp.srv) |
| 후보가 어디서 생성되는가? | [GraspNode._plan()](../../../ros2_ws/src/cleany_grasping/cleany_grasping/grasp_node.py) |
| 어떤 후보가 탈락하는가? | [rank_grasps()](../../../ros2_ws/src/cleany_grasping/cleany_grasping/core/selector.py) |
| 실제 실행까지 담당하는가? | [Skill Executor README](../../../ros2_ws/src/cleany_skill_executor/README.md) |

## 11.2 RGB-D부터 후보 선택까지 추적

실제 예제로 [can_grasp_execution_demo.py](../../../ros2_ws/src/cleany_skill_executor/cleany_skill_executor/can_grasp_execution_demo.py)의 `run()`을 읽어보세요. RGB-D 수신 후 `segment_red_can()`으로 분할하고 `PlanGrasp.Request`를 구성합니다. 이 빨간 캔 분할은 데모의 제한된 검출 경로이며 범용 perception 전체를 대신하지 않습니다.

요청에는 `snapshot_id`, `object_id`, target/context 점군과 target OBB가 들어갑니다. `GraspNode._validate()`는 object ID 일치, 점군의 frame·timestamp 일치, 유한한 OBB와 정규화된 quaternion을 확인합니다. 다음은 `_plan()`의 실제 후보 생성 부분입니다.

```python
candidates = self._predictor.predict(target, context, workspace)
ranked = rank_grasps(candidates, target, config)
grasp = ranked[0] if ranked else None
```

`predictor_type`은 geometric 또는 AnyGrasp를 선택합니다. `rank_grasps()`는 점수순 후보에서 허용 폭, 대상 접촉 범위와 중복 후보를 검사합니다. 통과 후보는 측정 시각의 TF로 `planning_frame`에 변환되어 응답에 담깁니다. `success=True`는 후보 생성 성공이며 팔 동작 성공이 아닙니다.

그 다음 `grasp/select_reachable` action은 [GraspSelector.select()](../../../ros2_ws/src/cleany_skill_executor/cleany_skill_executor/core/grasp_selection.py)를 사용합니다. 가까운 팔부터 pre-grasp IK → grasp IK → 두 상태의 유효성 → 두 구간의 계획을 검사하고 실패하면 다음 팔·후보로 진행합니다. 여기서 생성 predictor와 도달 가능성 selector는 서로 다른 책임입니다.

## 11.3 테스트로 필터의 의도 읽기

테스트는 입력 → 호출 → `assert` 순서로 읽습니다. 함수 내부를 모두 이해하기 전에 지켜야 하는 약속을 알 수 있습니다. [test_selector.py](../../../ros2_ws/src/cleany_grasping/test/test_selector.py)의 `test_selects_highest_scoring_target_contact_after_width_filter()`에는 다음 입력이 있습니다.

```python
predictor = FakePredictor(
    (
        grasp((0.0, 0.0, 0.52), 0.8),
        grasp((0.0, 0.0, 0.52), 0.99, width=0.2),
        grasp((0.3, 0.0, 0.52), 0.9),
    )
)

result = select_grasp(predictor, target, target)

assert result is not None
assert result.score == 0.8
```

0.99 후보의 요구 폭 0.2m는 기본 최대 폭 0.10m를 넘습니다. 0.9 후보는 target 접촉 범위를 벗어납니다. 따라서 0.8이 선택됩니다. 이 테스트가 말하는 것은 “점수가 가장 높은 후보”보다 **필터를 통과한 후보 중 최고 점수**가 중요하다는 것입니다.

`FakePredictor`는 모델 추론을 대신해 지정한 후보를 반환합니다. 따라서 이 테스트는 필터·순위 로직을 검증하며, 카메라 인식 정확도나 AnyGrasp 실제 추론 품질을 측정하지 않습니다. 같은 파일의 빈 후보와 중복 억제 테스트를 이어 읽으면 반환 `None`과 NMS 동작도 확인할 수 있습니다.

## 11.4 Plan-only와 로그 판독

운영 selector의 [moveit_adapter.py](../../../ros2_ws/src/cleany_skill_executor/cleany_skill_executor/moveit_adapter.py)는 다음 설정을 사용합니다.

```python
action_goal = MoveGroup.Goal()
action_goal.planning_options.plan_only = True
action_goal.planning_options.look_around = False
action_goal.planning_options.replan = False
```

그래서 `grasp/select_reachable` 성공 후 팔이 움직이지 않는 것은 이 계약에서 정상입니다. 별도 MuJoCo demo coordinator가 선택 결과를 받아 controller 실행을 수행합니다. 캔 데모도 gripper를 열고 방향을 보정한 pre-grasp까지 접근하며, close·attach·lift 성공을 뜻하지 않습니다.

이미 저장된 로그가 있다면 아래 경계를 먼저 찾으세요. 새 동작을 실행하기 전에 어디까지 완료됐는지 확인할 수 있습니다.

| 캔 데모 로그 | 확인된 단계 |
| --- | --- |
| `RGB-D can segmented` | target/context 점군과 OBB 생성 |
| `GEOMETRIC GRASP COMPLETE` | 후보 생성 |
| `Selected generated candidate` | selector 결과 수신 |
| `MoveIt execution succeeded: collision-checked aimed pre-grasp` | 데모의 pre-grasp 실행 |
| `CAN PREGRASP DEMO COMPLETE` | 해당 데모 완료 기준 도달 |

같은 이름의 코드라도 실행 branch·commit, launch, parameter가 다르면 결과가 달라집니다. [grasp_selection.yaml](../../../ros2_ws/src/cleany_skill_executor/config/grasp_selection.yaml)의 `planning_frame`, joint-state 최대 age 0.5초, 최대 후보 12개, pre-grasp offset 0.08m도 기록 대상입니다. 설정 변경과 물리 성능 개선은 따로 확인하세요.

## 11.5 Mission 실패와 검증 범위

[Mission 테스트](../../../ros2_ws/src/cleany_mission_manager/tests/test_mission_flow.py)의 `test_skill_retries_exhaust_and_preserve_partial_progress()`는 pick 성공 뒤 place가 반복 실패하는 입력을 사용합니다. 기대 결과 일부는 다음과 같습니다.

```python
assert report.status == "FAILED"
assert report.failure_code == FailureCode.PLACE_FAIL
# pick_object succeeded and must be preserved even though the mission overall failed.
assert report.completed_tasks == ["pick_object"]
assert report.failed_task == "place_object"
```

실패했어도 앞서 성공한 단계는 기록됩니다. `test_failed_skill_retries_only_that_skill_not_the_whole_plan()`은 실패한 place만 재시도하고 pick을 반복하지 않는지 확인합니다. `test_no_objects_detected_reports_blocked_without_planning()`은 물체가 없으면 planner를 호출하지 않는 경계를 검사합니다.

ROS 개발환경에서는 루트 `make test-mission`을 사용합니다. 이 target은 workspace build부터 수행합니다. 작은 순수 core 테스트는 해당 패키지 README의 pytest 절차를 따르세요. 테스트 통과의 범위는 사용한 입력과 대역까지이며, 시뮬레이션 실행이나 실물 측정의 증거와 구분합니다.

## 11.6 팀 코드 리뷰 실습

세 명이 역할을 바꾸어 20분 정도 읽어보는 것을 제안합니다. 한 명은 입력과 결과를 예상하고, 한 명은 호출 흐름을 추적하고, 담당자는 마지막에 구현 사실을 확인합니다.

이번 연습에서는 `test_selector.py`의 `width=0.2`를 `width=0.05`로 바꾼다고 가정하세요. **아직 코드를 수정하지 말고**, 선택 score와 영향을 받는 assertion을 적어봅니다. 0.99 후보가 폭·접촉 필터를 통과하면 0.8 후보는 가까운 중복으로 제거되므로 기대 score도 바뀝니다. 반대로 최대 폭을 0.20으로 늘리는 제품 설정 변경은 실제 gripper 폭과 충돌 영향까지 검토해야 합니다.

다음으로 담당자가 아닌 사람이 시작 파일 하나, 핵심 함수 하나, 실패 조건 하나를 설명합니다. 설명이 막힌 곳을 가이드 수정 대상으로 삼으세요. Agent에는 “수정 대신 읽을 함수와 힌트 하나”를 요청한 뒤 직접 예상과 구현을 비교해도 좋습니다.

**확인 질문:** 후보 생성 성공과 plan-only 성공만 기록된 로그로 실제 캔 파지 성공을 주장할 수 있을까요?

<details><summary>생각을 비교해 보기</summary>

후보와 계획까지만 확인했습니다. controller 실행, gripper 동작과 물체 상태 변화는 별도 증거가 필요합니다. 현재 캔 데모의 완료 기준도 pre-grasp까지입니다.

</details>
