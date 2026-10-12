# Mission과 Gateway 계약

Backend는 queue·배차를, Mission Runtime은 수락 이후 실행과 영속 결과를 소유한다. Runtime에는 별도 queue가 없으며 한 번에 미션 하나만 수락한다. 필드는 패키지 msg/srv, 설정은 cleany_mission_manager/config/runtime.yaml이 원본이다.

## 인터페이스

| 상대 이름 | 의미 |
|---|---|
| `mission/offer` | clean_desk, SEAT, canonical seat ID 요청 수락 |
| `mission/cancel` | 비동기 취소 수락 |
| `mission/snapshot` | 현재 상태와 보존된 최종 보고 |
| `mission/reset_error` | 정지·팔 주행 가능·Nav2 준비 확인 후 명시적 오류 해제 |
| `mission/status`, `mission/result` | transient-local 상태와 최신 최종 보고 |
| `robot/safety_fault`, `robot/safety_released` | 안전 소유자의 fault/release 알림 |

Offer/Cancel의 accepted는 수락 응답이며 완료 증거가 아니다. 같은 ID·같은 요청은 재실행하지 않고, 같은 ID에 다른 내용을 보내면 거절한다. Runtime 진단 sequence와 Backend 미션별 sequence는 별개다.

## 수락과 완료

```mermaid
sequenceDiagram
    participant B as Backend
    participant G as Gateway bridge
    participant M as Mission Runtime
    participant J as SQLite journal
    B->>G: mission.offer
    G->>M: OfferMission
    M->>M: 준비 / 요청 / 중복 검사
    alt 새 요청 수락
        M->>J: 수락 기록 영속 저장
        M-->>G: accepted
        G-->>B: mission.accepted
        M->>M: 주행 / 작업 / 종료 정책 실행
        M->>J: 최종 결과 영속 저장
        M-->>G: MissionResult
        G-->>B: mission.result
    else 거절 또는 중복
        M-->>G: 사유 또는 보존된 결과
        G-->>B: 응답 / 상태 동기화
    end
```

수신 ACK, 실행 수락, 최종 결과는 서로 다른 시점이다.

## 정상 실행

FSM은 `IDLE → NAVIGATE_TO_TARGET → WORKING → POST_MISSION → RETURN_HOME/IDLE` 순서다. 중단 시 CANCELLING, 치명적 오류나 정지 미확인 시 ERROR로 전이한다.

청소 BT는 ObserveBefore 이후 PlanNext → ValidateProposal → ExecuteOne → Reobserve → VerifyCompletion을 반복한다. Snapshot/object ID와 capability를 검사하고 객체 하나씩 실행한 뒤 재관측한다. Done 제안만으로 완료하지 않고 fresh 관측에서 잔여 대상을 확인한다.

보고에는 skip·사람 확인·실패·취소를 보존한다. Mock 관측 ID는 실제 사진이 아니다.

## 취소·안전·재시작

Nav2는 설정된 lifecycle node 모두 active이고 lifecycle_timeout 이내 응답해야 준비 상태다. 기본 목록은 map_server, amcl, planner_server, controller_server, bt_navigator, collision_monitor다. Deadline은 steady clock과 runtime.yaml 값을 사용한다.

| 조건 | 처리 |
|---|---|
| Nav2 action server만 준비 | NAV2_NOT_READY, goal 미전송 |
| 취소 요청 | Nav2 terminal + fresh odom 정지 확인; ACK만으로 IDLE 전이 금지 |
| 정지 확인 기한 초과 | ERROR 유지 |
| Safety fault | Producer가 실제 정지 먼저 수행; Runtime은 물리 E-stop 아님 |
| Fault 해제 | 일치하는 release + 정지 확인 + 명시적 reset |
| 재시작 당시 미완료 미션 | INTERRUPTED·사람 확인 필요; 자동 재개 금지 |
| Backend 단절 | 수락한 미션 계속 실행; 재연결 후 상태·결과 동기화 |

SQLite journal은 수락·출처·goal UUID·최종 결과·오류를 저장한다. 재시작 시 저장된 UUID의 orphan goal만 취소하고 정지·준비를 확인한다.

## Gateway schema v1

control_bridge는 `ws(s)://<backend>/api/robots/cleany-01/gateway/ws`를 ROS에 연결한다. mission.offer의 clean_seat를 ROS clean_desk/SEAT로 변환하고 canonical seat ID를 보존한다.

연결·재연결·sync마다 fresh snapshot을 보내고 **snapshot ACK 이후** 해당 연결의 feedback·명령을 허용한다. RPC timeout은 수락 여부가 불명확하므로 거절을 합성하지 않고 재동기화한다.

| 항목 | 의미 |
|---|---|
| ACK / mission.accepted / mission.result | 수신 / 수락 / 최종 결과 |
| SQLite outbox | Event ID·미션별 sequence·payload를 ACK까지 보존·재전송 |
| Snapshot | 최근 100개 최종 보고; IDLE이어도 port 미준비면 가용 ERROR |
| Mock before/after_observation | null; outcome·사람 확인은 보존 |
| Schema·인증·media 정책 | control-plane 저장소 소유 |
| Parameter 기본값 | control_bridge node 기준 |

## 개발 관측

| 토픽 | 계약 |
|---|---|
| debug_snapshot | reliable/transient-local depth 1, schema v1 |
| runtime_events | reliable/transient-local depth 2048, boot ID·연속 sequence |

관측 토픽은 명령 경로가 아니다. 소비자는 재시작과 sequence gap을 구분해야 한다.

관련 안내: [실행](../../../../docs/RUNNING.md), [Manipulation](manipulation.md).
