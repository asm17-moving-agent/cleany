# 02. Mission Core와 Mock

> 목표: 실행 출력에서 시작해 요청, 모듈 주입, 데이터 전달과 skill 실행 순서를 실제 코드로 추적합니다. Python 3.10 이상이 필요합니다.

## 02.1 Mock 기반 실행

**mock**은 실제 장치 대신 정해진 응답을 돌려주는 코드입니다. 이동 성공 응답을 주면, 로봇 없이도 인식 단계로 넘어가는지 확인할 수 있습니다. 첫 실습은 기존 Mission core를 사용하고 장치 응답만 대체합니다.

저장소 루트, 즉 `Makefile`이 있는 폴더에서 실행합니다.

```bash
python3 docs/guide/examples/first_mission.py
```

주요 출력은 다음과 같습니다. 맨 앞에는 `scenario: success`도 나옵니다.

```text
initial: IDLE
start: NAVIGATE_TO_TARGET
01: NAVIGATE_TO_TARGET -> PERCEIVE
02: PERCEIVE -> PLAN_TASKS
03: PLAN_TASKS -> EXECUTE_TASKS
04: EXECUTE_TASKS -> EXECUTE_TASKS
05: EXECUTE_TASKS -> RETURN_HOME
06: RETURN_HOME -> REPORT
07: REPORT -> IDLE
report: SUCCESS
completed: pick_object, place_object
published: 1
```

아직 상태 이름을 모두 외울 필요는 없습니다. 시작, 반복된 실행 상태, 마지막 보고서를 찾아보세요. 이 출력이 보여주는 성공은 mock 응답을 받은 core의 성공입니다.

## 02.2 MissionRequest와 MissionContext

[models.py](../../../ros2_ws/src/cleany_mission_manager/cleany_mission_manager/core/models.py)의 실제 요청 모델입니다.

```python
@dataclass(frozen=True)
class MissionRequest:
    mission_id: str
    mission_type: str
    target_id: str
    requested_by: str
```

**dataclass**는 필드 중심의 작은 데이터를 표현하는 Python 도구입니다. `frozen=True`는 생성 후 필드 재할당을 막습니다. 요청에는 미션·종류·대상·요청자 식별자가 있습니다. 실제 목표 좌표나 ROS 메시지를 이 요청에 직접 넣는 구조는 아닙니다.

`MissionContext`는 진행 중인 데이터를 보관합니다.

| 필드 | 이후 어디에서 쓰는가? |
| --- | --- |
| `request` | Navigator와 Perception 호출 |
| `state` | `step()`에서 실행 분기 선택 |
| `world_state` | Perception 결과를 Planner에 전달 |
| `plan` | Planner가 반환한 작업과 skill sequence |
| `next_skill_index` | 이번에 실행할 skill 위치 |
| `completed_skills`, `report` | 완료 기록과 최종 결과 |

요청은 미션의 입력이고 context는 실행하면서 변하는 기록입니다. `start()`는 이전 context에 덧붙이지 않고 새 context를 만듭니다. 실제 [start()](../../../ros2_ws/src/cleany_mission_manager/cleany_mission_manager/core/manager.py)를 읽어보세요.

```python
def start(self, request: MissionRequest) -> None:
    if self.state != MissionState.IDLE:
        raise RuntimeError(f"Cannot start mission from state {self.state}")
    self.context = MissionContext(request=request, state=MissionState.NAVIGATE_TO_TARGET)
```

`IDLE`에서만 시작할 수 있다는 조건이 있습니다. 나중에 다른 상태에서 요청을 받는 ROS adapter를 만든다면 이 조건도 함께 다뤄야 합니다.

## 02.3 Port 주입과 모의 응답

Manager는 필요한 객체를 생성자에서 받습니다. 이미 만든 모듈을 넣는 방식을 **의존성 주입**이라고 부릅니다. [first_mission.py](../examples/first_mission.py)의 실제 구성입니다.

```python
reporter = InMemoryReporter()
manager = MissionManager(
    navigator=navigator,
    perception=MockPerception(),
    planner=planner,
    skill_executor=MockSkillExecutor(),
    reporter=reporter,
)
```

위에서 `navigator`와 `planner`는 시나리오에 따라 선택한 객체입니다. 정상 시나리오에는 `MockNavigator`와 `MockPlanner`가 들어갑니다. class는 객체의 설계이고, `MockPerception()`처럼 괄호를 붙이면 그 객체를 만듭니다.

[mocks/components.py](../../../ros2_ws/src/cleany_mission_manager/cleany_mission_manager/mocks/components.py)의 `MockPerception.perceive()`는 `objects`와 `snapshot_id`를 담은 성공 결과를 반환합니다. `MockPlanner.plan()`은 그 입력을 분석하지 않고 정해진 계획을 반환합니다. 따라서 mock 객체의 confidence만 바꿔도 자동으로 작업 판단이 달라질 것이라고 기대하면 안 됩니다.

## 02.4 step과 데이터 전달

[manager.py](../../../ros2_ws/src/cleany_mission_manager/cleany_mission_manager/core/manager.py)의 `step()`은 현재 상태에 따라 모듈 함수 하나를 호출하고 결과를 해석합니다. 인식 성공 처리의 실제 함수입니다.

```python
def _handle_perception_result(self, result: ModuleResult[object]) -> None:
    if result.status == ResultStatus.OK:
        self.context.world_state = result.data
        self.context.state = MissionState.PLAN_TASKS
        return
    self._handle_non_ok_result(result, retry_key=str(self.state), failed_task=self.state.name)
```

중요한 두 줄은 `world_state = result.data`와 `state = PLAN_TASKS`입니다. 다음 `step()`의 `PLAN_TASKS` 분기는 그 `world_state`를 `planner.plan()`에 전달합니다.

```text
MockPerception.perceive(request)
  → ModuleResult.data의 objects·snapshot_id
  → context.world_state
  → MockPlanner.plan(world_state)
  → context.plan
```

이 경로에서는 Python 객체가 함수 사이를 이동합니다. 프로세스 사이 ROS 통신은 아직 아닙니다. 다음 장에서 이 내부 모델과 ROS 메시지의 차이를 봅니다.

## 02.5 skill_sequence와 완료 기록

Planner의 mock 계획에는 높은 수준의 `tasks`와 실행 순서인 `skill_sequence`가 함께 있습니다. Manager는 후자를 읽어 개별 동작을 수행합니다. 실제 함수입니다.

```python
def _skill_sequence(self) -> list[object]:
    plan = self.context.plan or {}
    if isinstance(plan, dict):
        return list(plan.get("skill_sequence", []))
    return []
```

`plan`이 dict이고 `skill_sequence`가 있으면 목록으로 읽습니다. 이 core는 현재 계획 구조를 엄격한 ROS 타입으로 강제하지 않습니다. 계약을 확인할 때 mock 계획, core가 읽는 키와 테스트를 함께 보는 이유입니다.

`_execute_next_skill()`은 `next_skill_index`의 동작을 `execute_skill()`에 전달합니다. 성공하면 동작 이름을 `completed_skills`에 추가하고 index를 하나 올립니다. 목록이 끝나면 `RETURN_HOME`으로 바꿉니다.

첫 출력에서 실행 상태가 반복된 이유도 여기 있습니다. `pick_object`가 끝난 뒤 `place_object`가 남아 있기 때문입니다. `REPORT`는 보고서를 발행한 뒤 `IDLE`로 돌아가며, `InMemoryReporter.reports`에 저장된 개수가 출력의 `published`입니다.

## 02.6 상태 추적 실습

다음 실행의 첫 이동 상태를 예상해 보세요.

```bash
python3 docs/guide/examples/first_mission.py --scenario retry
```

첫 이동 결과는 재시도 가능한 실패, 다음 결과는 성공입니다. 첫 줄은 `NAVIGATE_TO_TARGET -> NAVIGATE_TO_TARGET`이며 다음 호출에서 `PERCEIVE`로 넘어갑니다. 10장에서 제한과 실패 보고를 자세히 봅니다.

이제 코드에서 다음 세 가지를 찾아 출력 옆에 적어보세요.

1. `MockPlanner`가 반환하는 두 skill의 이름
2. `_execute_next_skill()`이 index를 증가시키는 조건
3. `REPORT`가 실행될 때 보고서가 생성되는 위치

**확인 질문:** 왜 `step()` 호출 횟수와 완료 skill 개수가 같지 않을까요?

<details><summary>생각을 비교해 보기</summary>

이동·인식·계획·복귀·보고도 각각 step을 사용합니다. 실행 상태에서도 성공하지 않은 skill은 완료 목록에 추가되지 않습니다. 함수 호출, 상태 전이와 실제로 끝낸 동작을 구분해야 합니다.

</details>
