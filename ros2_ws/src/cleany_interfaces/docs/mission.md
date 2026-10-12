# Mission과 Gateway 계약

필드 원본은 `../msg`, `../srv`다. 설정은 `cleany_mission_manager/config/runtime.yaml`이다.
Backend는 queue·배차, Mission Runtime은 수락 이후 실행과 영속 결과를 소유한다.
Runtime은 별도 queue 없이 IDLE·모든 port의 준비/정지·지원 좌석·유형을 검사해 하나만 수락한다.

| 상대 이름 | 의미 |
|---|---|
| `mission/offer` | clean_desk, SEAT, canonical seat ID 요청 수락 |
| `mission/cancel` | 비동기 취소 수락 |
| `mission/snapshot` | 현재 상태와 보존된 최종 보고 |
| `mission/reset_error` | 정지·팔 주행 가능·Nav2 준비 확인 후 명시적 오류 해제 |
| `mission/status`, `mission/result` | transient-local 상태와 최신 최종 보고 |
| `robot/safety_fault`, `robot/safety_released` | 안전 소유자의 fault/release 알림 |

Offer/Cancel 응답의 accepted는 완료 증거가 아니다. 동일 mission ID·동일 요청은 재실행하지 않고,
다른 내용으로 ID를 재사용하면 거절한다. Status 진단 sequence와 Backend wire의 미션별 sequence는 별개다.

## 실행·취소·복구

FSM은 IDLE → NAVIGATE_TO_TARGET → WORKING → POST_MISSION → RETURN_HOME/IDLE이다.
중단은 CANCELLING, 치명적 오류·정지 확인 실패는 ERROR다.
청소 BT는 ObserveBefore 후 PlanNext → ValidateProposal → ExecuteOne → Reobserve → VerifyCompletion을 반복한다.
한 객체만 실행하고 재관측한다. snapshot/object ID·허용 capability를 검사한다.
Planner의 done 제안만으로 완료하지 않고 fresh 관측에서 처리 대상이 남았는지 확인한다.
skip·사람 확인·실패·취소를 보고에 보존한다. Mock 관측 ID는 실제 사진이 아니다.

Nav2 준비 검사는 설정된 lifecycle node 모두 active이고 응답이 lifecycle_timeout 이내여야 한다.
기본 목록은 map_server, amcl, planner_server, controller_server, bt_navigator, collision_monitor다.
Action server만 준비된 상태는 NAV2_NOT_READY이며 goal을 보내지 않는다.
취소는 Nav2 terminal 상태와 fresh odom 정지를 함께 확인한다. ACK만으로 IDLE로 전이하지 않는다.
기한 초과는 ERROR를 유지한다. Deadline은 steady clock 기준이며 값은 runtime.yaml을 따른다.

안전 알림 producer가 실제 정지를 먼저 수행한다. Runtime은 물리 E-stop 장치가 아니다.
일치하는 release와 정지 확인 이후에도 별도 reset이 필요하다.
SQLite journal은 수락·실행 출처·Nav2 goal UUID·최종 결과·오류를 저장한다.
재시작 시 미완료 미션은 INTERRUPTED와 사람 확인 필요로 보고하며 자동 재개하지 않는다.
저장된 UUID의 orphan goal만 취소하고 정지·준비를 확인한다.
Backend 단절은 수락한 미션을 중단하지 않으며 재연결 뒤 상태와 결과를 맞춘다.

## Gateway와 개발 관측

Robot Gateway transport는 `cleany_control_bridge`가 ROS service/topic에 연결한다.
Backend schema·인증·media 정책은 control-plane 구현 저장소가 소유한다.
Bridge 설정/실행은 [RUNNING](../../../../docs/RUNNING.md),
단일 객체의 결과 판정·주행 gate는 [Manipulation](manipulation.md)를 따른다.

`mission/debug_snapshot`은 reliable/transient-local depth 1, schema version 1이다.
`mission/runtime_events`는 reliable/transient-local depth 2048이며 boot ID와 연속 sequence를 가진다.
재시작·sequence gap을 구분한다. 관측 토픽은 명령 경로가 아니다.

## Gateway schema v1

`ws(s)://<backend>/api/robots/cleany-01/gateway/ws`를 사용한다.
`mission.offer: clean_seat`를 ROS clean_desk/SEAT로 변환하며 canonical seat ID를 보존한다.
연결·재연결·sync마다 fresh snapshot을 보내고 **snapshot ACK 이후** 해당 연결의 feedback·명령을 허용한다.
명령 ACK는 수신 확인이다. 수락은 mission.accepted, 취소 완료는 최종 mission.result로 확인한다.
ROS RPC timeout은 수락 여부가 불명확하므로 거절을 합성하지 않고 재동기화한다.
SQLite outbox는 event ID·미션별 sequence·payload를 ACK까지 유지하고 재연결 시 재전송한다.
Snapshot은 최근 100개 최종 보고를 포함한다. IDLE이라도 port 미준비면 가용 ERROR로 전달한다.
Mock 관측 ID는 외부 before/after_observation에 null을 보낸다. Outcome과 사람 확인을 보존한다.
정확한 parameter 기본값은 control_bridge node, peer schema는 control-plane 저장소를 따른다.
