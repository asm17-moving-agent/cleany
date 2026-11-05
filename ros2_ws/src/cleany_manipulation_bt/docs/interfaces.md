# 실물 BT 인터페이스

`manipulation_jetson_server`는 실물 수거 실행, 팔 복귀와 복구 차단을 관리한다.
아래 경로는 namespace가 없는 기본값이다. 실행은
[실물 수거 안내](../../cleany_skill_executor/docs/physical-collection.md), 공통 취소와 기록은
[실행 계약](execution_contract.md)을 따른다.

| 종류 | 경로 | 타입 | 역할 |
|---|---|---|---|
| Action server | `manipulation/execute_skill` | `cleany_interfaces/action/ExecuteManipulationSkill` | 승인된 snapshot과 물체의 단일 수거 |
| Action server | `manipulation/return_arm` | `cleany_interfaces/action/ReturnArm` | 수거와 동시 실행하지 않는 팔 복귀 |
| Service server | `manipulation/cancel` | `cleany_interfaces/srv/CancelManipulation` | 실행 ID와 취소 모드로 중단 요청 |
| Service server | `manipulation/get_execution` | `cleany_interfaces/srv/GetManipulationExecution` | 저장된 실행 결과 조회 |
| Service server | `manipulation/control_state` | `std_srvs/srv/Trigger` | 현재 실행 소유권, guard와 복구 차단 조회 |
| Service server | `manipulation/confirm_recovery` | `cleany_interfaces/srv/ConfirmManipulationRecovery` | 정지와 빈 그리퍼 확인 후 과거 차단 해제 |
| Publisher | `manipulation/execution_events` | `cleany_interfaces/msg/ManipulationExecutionRecord` | 실행 기록. reliable, transient-local, depth 100 |
| Publisher | `manipulation/bt_transitions` | `std_msgs/msg/String` | BT 상태 전이 JSON. reliable, transient-local, depth 100 |

`arm_return`과 `manipulation_recovery`는 해당 Action과 Service의 CLI client다.
`ReturnArm`의 성공과 수거 성공은 별도 결과이며, 복구 확인만으로 동작을 시작하지 않는다.
노드 종료 시 취소한 명령의 응답과 실제 정지 근거를 제한 시간 안에서 수집한다.
