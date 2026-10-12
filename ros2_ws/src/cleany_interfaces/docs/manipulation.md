# 단일 객체 Manipulation 계약

- 원본: ExecuteManipulationSkill.action / ManipulationExecutionRecord.msg / GetManipulationExecution.srv
- 기본 mock 이름: `mock/manipulation/execute_skill`, `mock/manipulation/get_execution`
- Mission: 승인 객체 전달. 서버 내부 BT는 executor 소유
- Reachable-grasp plan-only action과 별도 계약

## 요청·재전송

- 요청: mission/task/execution ID, skill_name, snapshot ID, uint32 object ID, destination ID
- Object: wire ID를 Planner 제안과 대조. 표시 ID에서 숫자 추정 금지
- Destination: RuntimePolicy allowlist 검사
- Execution UUID 발급 → **전체 요청 SQLite 저장 → 전송**
- ROS goal UUID도 execution UUID 사용. 지연 수락·재시작 취소는 해당 goal만 대상
- 결과 불명확: 같은 execution ID 조회. 새 ID 자동 재전송 금지

## 성공 조건

- ROS SUCCEEDED만으로 성공 판정 금지: peer의 BLOCKED도 SUCCEEDED로 반환 가능
- 필수: payload SUCCESS, 일치하는 execution ID·mock 출처, LEFT_GRIPPER, CONFIRMED
- 추가 필수: placement 검증 완료, 팔 복귀, 정지 확인
- Feedback stage: 완료 증거 아님
- 보존: 전체 결과·task/execution/destination ID → journal·report_json.actions
- 변환: peer CANCELED → Runtime CANCELLED

## 주행·복구 gate

- HELD/UNKNOWN·팔 미복귀·사람 확인 필요: home·다음 미션 이동 차단
- 취소 결과 CANCELLED여도 FSM ERROR 유지 가능
- reset_error: 주행 가능 조건 우회 금지
- 현재 peer: 수동 팔 복구·해제 계약 없음. Runtime 임의 복구 금지
- found=false: 저장 오류/기록 없음 미구분. 미실행·정지 증거 아님
- 진행 중·INTERRUPTED·불일치 결과: gate 유지
- 재시작: 저장 UUID의 취소·조회만 수행. 미션 자동 재개 금지
- 진단: journal의 manipulation_boundary metadata
- 안내: [Mission](mission.md), [검증](../../../../docs/TESTING.md)
