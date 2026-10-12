# 단일 객체 Manipulation 계약

원본은 `ExecuteManipulationSkill.action`, `ManipulationExecutionRecord.msg`,
`GetManipulationExecution.srv`다. Mission은 승인한 객체를 실행 서버에 전달하며 서버 내부 BT는 소유하지 않는다.
현재 mock 경계의 기본 이름은 `mock/manipulation/execute_skill`, `mock/manipulation/get_execution`이다.
`cleany_skill_executor`의 reachable-grasp plan-only action과는 별도 계약이다.

## 요청과 멱등성

요청은 mission/task/execution ID, skill_name, snapshot ID, uint32 object ID, destination ID다.
Runtime은 wire object ID를 Planner 제안과 대조하며 `trash-1` 같은 표시 ID에서 숫자를 추정하지 않는다.
목적지는 RuntimePolicy의 allowlist로 검사한다.
행동마다 execution UUID를 발급하고 **전체 요청을 SQLite에 영속 저장한 뒤** 전송한다.
이 UUID를 ROS goal UUID에도 사용한다. 지연 수락·재시작 취소는 정확한 goal만 대상으로 한다.
전송·결과가 불확실하면 같은 execution ID로 조회하며 새 ID로 자동 재전송하지 않는다.

## 성공과 주행 gate

ROS SUCCEEDED만으로 성공을 판단하지 않는다. 현재 peer는 BLOCKED도 ROS SUCCEEDED로 반환한다.
Runtime은 payload SUCCESS, 일치하는 execution ID와 mock 출처, LEFT_GRIPPER,
CONFIRMED, placement 검증 완료, 팔 복귀 및 정지 확인을 함께 검사한다.
Feedback stage는 완료 증거가 아니다. 결과 전체와 task/execution/destination ID를 journal과 report_json.actions에 보존한다.
Peer의 CANCELED는 Runtime CANCELLED로 변환한다.

정지 확인이 있어도 HELD/UNKNOWN, 팔 미복귀, 사람 확인 필요이면 home·다음 미션 이동을 차단한다.
취소 결과는 CANCELLED로 보존하면서 FSM은 ERROR를 유지할 수 있다.
reset_error도 주행 가능 조건을 우회하지 못한다. 현재 peer에는 수동 팔 복구·해제 계약이 없으며 Runtime이 임의 복구 동작을 실행하지 않는다.

`found=false`는 저장 오류와 기록 없음이 구분되지 않아 미실행·정지 증거가 아니다.
진행 중·INTERRUPTED·불일치 결과도 gate를 해제하지 않는다.
재시작 후 저장한 UUID의 취소·조회만 수행하며 미션을 자동 재개하지 않는다.
진단은 journal의 manipulation_boundary metadata에 보존한다.

[Mission](mission.md), [검증](../../../../docs/TESTING.md).
